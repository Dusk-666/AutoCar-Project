#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

/* ==================== 固定硬件映射 ==================== */
#define MOTOR_LEFT_PWM_CHANNEL                       TIM_CHANNEL_3
#define MOTOR_RIGHT_PWM_CHANNEL                      TIM_CHANNEL_4

#define MOTOR_LEFT_DIR_FORWARD_PORT                  GPIOB
#define MOTOR_LEFT_DIR_FORWARD_PIN                   GPIO_PIN_12
#define MOTOR_LEFT_DIR_REVERSE_PORT                  GPIOB
#define MOTOR_LEFT_DIR_REVERSE_PIN                   GPIO_PIN_13

#define MOTOR_RIGHT_DIR_FORWARD_PORT                 GPIOB
#define MOTOR_RIGHT_DIR_FORWARD_PIN                  GPIO_PIN_14
#define MOTOR_RIGHT_DIR_REVERSE_PORT                 GPIOB
#define MOTOR_RIGHT_DIR_REVERSE_PIN                  GPIO_PIN_15

/* ==================== 内环默认参数 ==================== */
#define MOTOR_DEFAULT_SPEED_KP                       8.0f
#define MOTOR_DEFAULT_SPEED_KI                       0.25f
#define MOTOR_DEFAULT_INTEGRAL_LIMIT                 32.0f
#define MOTOR_DEFAULT_TARGET_LIMIT                   65
#define MOTOR_DEFAULT_TARGET_RAMP_STEP               6
#define MOTOR_DEFAULT_STOP_WINDOW                    3
#define MOTOR_DEFAULT_PWM_MAX_LIMIT                  300
#define MOTOR_DEFAULT_RIGHT_WHEEL_SCALE              1.08f
#define MOTOR_DEFAULT_STRAIGHT_BALANCE_WINDOW        12
#define MOTOR_DEFAULT_STRAIGHT_BALANCE_LIMIT         10
#define MOTOR_DEFAULT_STRAIGHT_BALANCE_GAIN          0.4f

typedef struct
{
    float proportional_gain;
    float integral_gain;
    float integral_sum;
    float integral_limit;
    int16_t output_limit;
} MotorSpeedPI;

typedef struct
{
    int16_t target_limit;
    int16_t target_ramp_step;
    int16_t stop_window;
    float right_wheel_scale;
    int16_t straight_balance_window;
    int16_t straight_balance_limit;
    float straight_balance_gain;
} MotorParams;

typedef struct
{
    TIM_HandleTypeDef *pwm_timer;
    uint32_t left_pwm_channel;
    uint32_t right_pwm_channel;
    MotorSpeedPI left_speed_pi;
    MotorSpeedPI right_speed_pi;
    MotorParams params;
    int16_t requested_left_target;
    int16_t requested_right_target;
    int16_t ramped_left_target;
    int16_t ramped_right_target;
    int16_t left_feedback_speed;
    int16_t right_feedback_speed;
    int16_t left_pwm_output;
    int16_t right_pwm_output;
} MotorHandle;

void Motor_Init(MotorHandle *handle, TIM_HandleTypeDef *pwm_timer);
void Motor_SetTargetSpeed(MotorHandle *handle, int16_t left_target_speed, int16_t right_target_speed);
void Motor_UpdateClosedLoop(MotorHandle *handle, int16_t left_feedback_speed, int16_t right_feedback_speed);
void Motor_Stop(MotorHandle *handle);

#endif
