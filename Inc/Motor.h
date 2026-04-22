#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

/* ==================== 速度 PI 参数 ==================== */
#define SPEED_PI_KP                     6.0f
#define SPEED_PI_KI                     0.8f

/* ==================== 电机驱动限幅参数 ==================== */
#define MOTOR_PWM_LIMIT                 1000
#define SPEED_PI_INTEGRAL_LIMIT         420.0f
#define MOTOR_TARGET_LIMIT              800
#define MOTOR_TARGET_RAMP_STEP          24

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
    TIM_HandleTypeDef *pwm_timer;
    uint32_t left_pwm_channel;
    uint32_t right_pwm_channel;
    MotorSpeedPI left_speed_pi;
    MotorSpeedPI right_speed_pi;
    int16_t requested_left_target;
    int16_t requested_right_target;
    int16_t ramped_left_target;
    int16_t ramped_right_target;
    int16_t left_pwm_output;
    int16_t right_pwm_output;
} MotorHandle;

/**
 * @brief 初始化电机模块并启动 PWM 输出
 * @param handle 电机模块句柄
 * @param pwm_timer PWM 所在定时器
 */
void Motor_Init(MotorHandle *handle, TIM_HandleTypeDef *pwm_timer);

/**
 * @brief 设置电机闭环控制的目标轮速
 * @param handle 电机模块句柄
 * @param left_target_speed 左轮目标速度
 * @param right_target_speed 右轮目标速度
 */
void Motor_SetTargetSpeed(MotorHandle *handle, int16_t left_target_speed, int16_t right_target_speed);

/**
 * @brief 执行一次左右电机速度闭环计算并刷新 PWM 输出
 * @param handle 电机模块句柄
 * @param left_feedback_speed 左轮速度反馈
 * @param right_feedback_speed 右轮速度反馈
 */
void Motor_UpdateClosedLoop(MotorHandle *handle, int16_t left_feedback_speed, int16_t right_feedback_speed);

/**
 * @brief 将电机输出安全拉低并清空 PI 内部状态
 * @param handle 电机模块句柄
 */
void Motor_Stop(MotorHandle *handle);

/**
 * @brief 计算单轮速度 PI 输出
 * @param controller 速度 PI 控制器
 * @param target_speed 目标速度
 * @param feedback_speed 反馈速度
 * @return PI 输出值，范围受 output_limit 限制
 */
int16_t Motor_ComputeSpeedPI(MotorSpeedPI *controller, int16_t target_speed, int16_t feedback_speed);

#endif
