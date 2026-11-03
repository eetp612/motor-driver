#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"

#include "driver/ledc.h"
#include "driver/gpio.h"

#include <uros_network_interfaces.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>

#include <geometry_msgs/msg/twist.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

// ============================================================
// Utilidades
// ============================================================

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

// ============================================================
// Configuración micro-ROS
// ============================================================

#ifndef CONFIG_MICRO_ROS_APP_STACK
#define CONFIG_MICRO_ROS_APP_STACK 10240
#endif

#ifndef CONFIG_MICRO_ROS_APP_TASK_PRIO
#define CONFIG_MICRO_ROS_APP_TASK_PRIO 5
#endif

// ============================================================
// Configuración motores
// ============================================================

#define LEFT  0
#define RIGHT 1

typedef int16_t rpm_t;
typedef uint8_t duty_t;

// TODO: definir mediante regresión/calibración
// duty = COEFFICIENT * rpm
#define MOTOR_LEFT_RPM_DUTY_COEFFICIENT  0
#define MOTOR_RIGHT_RPM_DUTY_COEFFICIENT 0

// ------------------------------------------------------------
// PWM
// ------------------------------------------------------------

#define PWM_FREQ_HZ       20000
#define PWM_RESOLUTION    LEDC_TIMER_10_BIT

#define MOTOR_LEFT_PWM    GPIO_NUM_4
#define MOTOR_RIGHT_PWM   GPIO_NUM_7

#define MOTOR_LEFT_CH     LEDC_CHANNEL_0
#define MOTOR_RIGHT_CH    LEDC_CHANNEL_1

#define PWM_TIMER         LEDC_TIMER_0
#define PWM_MODE          LEDC_LOW_SPEED_MODE

// ------------------------------------------------------------
// Dirección de los motores
// ------------------------------------------------------------

#define MOTOR_LEFT_IN1    GPIO_NUM_5
#define MOTOR_LEFT_IN2    GPIO_NUM_6

#define MOTOR_RIGHT_IN1   GPIO_NUM_15
#define MOTOR_RIGHT_IN2   GPIO_NUM_16

// ------------------------------------------------------------
// Inicialización de los GPIO
// ------------------------------------------------------------

static void motor_gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask =
            (1ULL << MOTOR_LEFT_IN1)  |
            (1ULL << MOTOR_LEFT_IN2)  |
            (1ULL << MOTOR_RIGHT_IN1) |
            (1ULL << MOTOR_RIGHT_IN2),

        .mode = GPIO_MODE_OUTPUT,

        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,

        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Estado inicial: motores detenidos
    ESP_ERROR_CHECK(gpio_set_level(MOTOR_LEFT_IN1, 0));
    ESP_ERROR_CHECK(gpio_set_level(MOTOR_LEFT_IN2, 0));

    ESP_ERROR_CHECK(gpio_set_level(MOTOR_RIGHT_IN1, 0));
    ESP_ERROR_CHECK(gpio_set_level(MOTOR_RIGHT_IN2, 0));
}

// ============================================================
// Tipos de datos
// ============================================================

typedef struct
{
    duty_t duty;
    uint8_t in1;
    uint8_t in2;
} motor_drive_t;

typedef struct
{
    motor_drive_t left;
    motor_drive_t right;
} motor_drive_pair_t;

typedef struct
{
    float angular_z;
    float linear_x;
} cmd_vel_msg_trunc;

// ============================================================
// RPM -> DUTY
// ============================================================

duty_t duty_for(int motor, rpm_t rpm)
{
    if (motor == LEFT)
    {
        return (duty_t)MIN(
            100,
            MAX(
                0,
                MOTOR_LEFT_RPM_DUTY_COEFFICIENT * rpm
            )
        );
    }

    if (motor == RIGHT)
    {
        return (duty_t)MIN(
            100,
            MAX(
                0,
                MOTOR_RIGHT_RPM_DUTY_COEFFICIENT * rpm
            )
        );
    }

    return 0;
}

// ============================================================
// Transformación cmd_vel -> motores
// ============================================================
//
// Permitimos:
//
//     linear.x  ∈ [-1, 1]
//     angular.z ∈ [-1, 1]
//
// linear.x determina el DUTY.
//
// angular.z determina el sentido:
//
//     z > 0 -> motor izquierdo forward,
//               motor derecho reverse
//
//     z < 0 -> motor izquierdo reverse,
//               motor derecho forward
//
//     z = 0 -> ambos motores detenidos
//
// ============================================================

static float clamp_motor(float value)
{
    if (value > 1.0f) return 1.0f;
    if (value < -1.0f) return -1.0f;
    return value;
}

static motor_drive_t make_motor_drive(float velocity)
{
    motor_drive_t drive = {0};

    velocity = clamp_motor(velocity);

    drive.duty = (duty_t)(fabsf(velocity) * 100.0f);

    if (velocity > 0.0f)
    {
        drive.in1 = 1;
        drive.in2 = 0;
    }
    else if (velocity < 0.0f)
    {
        drive.in1 = 0;
        drive.in2 = 1;
    }
    else
    {
        drive.in1 = 0;
        drive.in2 = 0;
    }

    return drive;
}

motor_drive_pair_t transform_cmd_vel(cmd_vel_msg_trunc msgt)
{
    float linear = clamp_motor(msgt.linear_x);
    float angular = clamp_motor(msgt.angular_z);

    float left_velocity  = linear - angular;
    float right_velocity = linear + angular;

    // Normalizamos para mantener ambos valores en [-1, 1].
    float max_velocity = fmaxf(
        1.0f,
        fmaxf(fabsf(left_velocity), fabsf(right_velocity))
    );

    left_velocity  /= max_velocity;
    right_velocity /= max_velocity;

    return (motor_drive_pair_t) {
        .left = make_motor_drive(left_velocity),
        .right = make_motor_drive(right_velocity)
    };
}

// ============================================================
// PWM
// ============================================================

void motor_pwm_init(void)
{
    // --------------------------------------------------------
    // Timer
    // --------------------------------------------------------

    ledc_timer_config_t timer_config = {
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    ESP_ERROR_CHECK(
        ledc_timer_config(&timer_config)
    );

    // --------------------------------------------------------
    // Motor izquierdo
    // --------------------------------------------------------

    ledc_channel_config_t left_channel = {
        .gpio_num = MOTOR_LEFT_PWM,
        .speed_mode = PWM_MODE,
        .channel = MOTOR_LEFT_CH,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0,
        .flags.output_invert = 0,
    };

    ESP_ERROR_CHECK(
        ledc_channel_config(&left_channel)
    );

    // --------------------------------------------------------
    // Motor derecho
    // --------------------------------------------------------

    ledc_channel_config_t right_channel = {
        .gpio_num = MOTOR_RIGHT_PWM,
        .speed_mode = PWM_MODE,
        .channel = MOTOR_RIGHT_CH,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0,
        .flags.output_invert = 0,
    };

    ESP_ERROR_CHECK(
        ledc_channel_config(&right_channel)
    );
}

// ============================================================
// Set PWM
// ============================================================
//
// duty:
//     0.0 -> 0%
//     0.5 -> 50%
//     1.0 -> 100%
// ============================================================

void motor_pwm_set(ledc_channel_t channel, float duty)
{
    if (duty < 0.0f)
        duty = 0.0f;

    if (duty > 1.0f)
        duty = 1.0f;

    uint32_t duty_value =
        (uint32_t)(duty * 1023.0f);

    ESP_ERROR_CHECK(
        ledc_set_duty(
            PWM_MODE,
            channel,
            duty_value
        )
    );

    ESP_ERROR_CHECK(
        ledc_update_duty(
            PWM_MODE,
            channel
        )
    );
}

// ============================================================
// Set dirección motor izquierdo
// ============================================================

static void motor_left_set_direction(
    uint8_t in1,
    uint8_t in2
)
{
    gpio_set_level(MOTOR_LEFT_IN1, in1);
    gpio_set_level(MOTOR_LEFT_IN2, in2);
}

// ============================================================
// Set dirección motor derecho
// ============================================================

static void motor_right_set_direction(
    uint8_t in1,
    uint8_t in2
)
{
    gpio_set_level(MOTOR_RIGHT_IN1, in1);
    gpio_set_level(MOTOR_RIGHT_IN2, in2);
}

// ============================================================
// /cmd_vel callback
// ============================================================

void cmd_vel_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg =
        (const geometry_msgs__msg__Twist *)msgin;

    printf(
        "cmd_vel: linear.x = %.2f, angular.z = %.2f\n",
        msg->linear.x,
        msg->angular.z
    );

    // --------------------------------------------------------
    // Convertimos el mensaje ROS a nuestro tipo reducido
    // --------------------------------------------------------

    cmd_vel_msg_trunc msgt = {
        .linear_x = msg->linear.x,
        .angular_z = msg->angular.z
    };

    // --------------------------------------------------------
    // Transformamos cmd_vel -> comandos de ambos motores
    // --------------------------------------------------------

    motor_drive_pair_t motor_drive =
        transform_cmd_vel(msgt);

    // --------------------------------------------------------
    // Aplicamos dirección
    // --------------------------------------------------------

    motor_left_set_direction(
        motor_drive.left.in1,
        motor_drive.left.in2
    );

    motor_right_set_direction(
        motor_drive.right.in1,
        motor_drive.right.in2
    );

    // --------------------------------------------------------
    // Aplicamos PWM
    //
    // transform_cmd_vel() entrega duty en [0, 100].
    // motor_pwm_set() espera [0.0, 1.0].
    // --------------------------------------------------------

    motor_pwm_set(
        MOTOR_LEFT_CH,
        motor_drive.left.duty / 100.0f
    );

    motor_pwm_set(
        MOTOR_RIGHT_CH,
        motor_drive.right.duty / 100.0f
    );
}

// ============================================================
// ROS 2 objects
// ============================================================

rcl_subscription_t cmd_vel_subscriber;

geometry_msgs__msg__Twist cmd_vel_msg;

// ============================================================
// Error handling
// ============================================================

#define RCCHECK(fn)                                                        \
    {                                                                      \
        rcl_ret_t temp_rc = fn;                                            \
        if ((temp_rc) != RCL_RET_OK)                                       \
        {                                                                  \
            printf(                                                        \
                "Failed status on line %d: %d. Aborting.\n",               \
                __LINE__,                                                  \
                (int)temp_rc                                                \
            );                                                             \
            vTaskDelete(NULL);                                             \
        }                                                                  \
    }

#define RCSOFTCHECK(fn)                                                    \
    {                                                                      \
        rcl_ret_t temp_rc = fn;                                            \
        if ((temp_rc) != RCL_RET_OK)                                       \
        {                                                                  \
            printf(                                                        \
                "Failed status on line %d: %d. Continuing.\n",             \
                __LINE__,                                                  \
                (int)temp_rc                                                \
            );                                                             \
        }                                                                  \
    }

// ============================================================
// micro-ROS task
// ============================================================

void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator =
        rcl_get_default_allocator();

    rclc_support_t support;

    // --------------------------------------------------------
    // Initialize rcl init options
    // --------------------------------------------------------

    rcl_init_options_t init_options =
        rcl_get_zero_initialized_init_options();

    RCCHECK(
        rcl_init_options_init(
            &init_options,
            allocator
        )
    );

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE

    rmw_init_options_t *rmw_options =
        rcl_init_options_get_rmw_init_options(
            &init_options
        );

    // --------------------------------------------------------
    // Configure micro-ROS Agent
    // --------------------------------------------------------

    RCCHECK(
        rmw_uros_options_set_udp_address(
            CONFIG_MICRO_ROS_AGENT_IP,
            CONFIG_MICRO_ROS_AGENT_PORT,
            rmw_options
        )
    );

#endif

    // --------------------------------------------------------
    // Initialize micro-ROS support
    // --------------------------------------------------------

    RCCHECK(
        rclc_support_init_with_options(
            &support,
            0,
            NULL,
            &init_options,
            &allocator
        )
    );

    // --------------------------------------------------------
    // Create node
    // --------------------------------------------------------

    rcl_node_t node;

    RCCHECK(
        rclc_node_init_default(
            &node,
            "esp32_motor_driver",
            "",
            &support
        )
    );

    // --------------------------------------------------------
    // Create /cmd_vel subscriber
    // --------------------------------------------------------

    RCCHECK(
        rclc_subscription_init_default(
            &cmd_vel_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(
                geometry_msgs,
                msg,
                Twist
            ),
            "cmd_vel"
        )
    );

    // --------------------------------------------------------
    // Create executor
    // --------------------------------------------------------

    rclc_executor_t executor;

    RCCHECK(
        rclc_executor_init(
            &executor,
            &support.context,
            1,
            &allocator
        )
    );

    // --------------------------------------------------------
    // Add /cmd_vel subscription
    // --------------------------------------------------------

    RCCHECK(
        rclc_executor_add_subscription(
            &executor,
            &cmd_vel_subscriber,
            &cmd_vel_msg,
            &cmd_vel_callback,
            ON_NEW_DATA
        )
    );

    // --------------------------------------------------------
    // Main loop
    // --------------------------------------------------------

    while (1)
    {
        rclc_executor_spin_some(
            &executor,
            RCL_MS_TO_NS(100)
        );

        usleep(10000);
    }

    // --------------------------------------------------------
    // Cleanup
    // --------------------------------------------------------

    RCCHECK(
        rcl_subscription_fini(
            &cmd_vel_subscriber,
            &node
        )
    );

    RCCHECK(
        rcl_node_fini(&node)
    );

    vTaskDelete(NULL);
}

// ============================================================
// app_main
// ============================================================
void app_main(void)
{
    ESP_LOGI("MAIN", "Starting motor driver");

    ESP_LOGI("MAIN", "Initializing motor GPIO");
    motor_gpio_init();

    ESP_LOGI("MAIN", "Initializing motor PWM");
    motor_pwm_init();

    ESP_LOGI("MAIN", "Initializing micro-ROS network");

    esp_err_t err = uros_network_interface_initialize();

    if (err != ESP_OK) {
        ESP_LOGE("MAIN", "Network initialization failed: %s",
                 esp_err_to_name(err));
        return;
    }

    ESP_LOGI("MAIN", "Creating micro-ROS task");

    BaseType_t result = xTaskCreate(
        micro_ros_task,
        "uros_task",
        CONFIG_MICRO_ROS_APP_STACK,
        NULL,
        CONFIG_MICRO_ROS_APP_TASK_PRIO,
        NULL
    );

    if (result != pdPASS) {
        ESP_LOGE("MAIN", "Failed to create micro-ROS task");
        return;
    }

    ESP_LOGI("MAIN", "Application initialized successfully");
}