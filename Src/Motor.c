#include "Motor.h"

/**
 * @brief 对有符号 16 位目标值做限幅
 * @param input_value 输入值
 * @param lower_limit 下限
 * @param upper_limit 上限
 * @return 限幅后的结果
 */
static int16_t Motor_ClampS16(int32_t input_value, int16_t lower_limit, int16_t upper_limit)
{
    if (input_value < lower_limit)
    {
        return lower_limit;
    }

    if (input_value > upper_limit)
    {
        return upper_limit;
    }

    return (int16_t)input_value;
}

/**
 * @brief 对浮点积分项做限幅
 * @param input_value 输入值
 * @param lower_limit 下限
 * @param upper_limit 上限
 * @return 限幅后的结果
 */
static float Motor_ClampFloat(float input_value, float lower_limit, float upper_limit)
{
    if (input_value < lower_limit)
    {
        return lower_limit;
    }

    if (input_value > upper_limit)
    {
        return upper_limit;
    }

    return input_value;
}

/**
 * @brief 按设定步长平滑逼近目标速度，减小车体抽动
 * @param current_value 当前值
 * @param target_value 目标值
 * @param step_per_cycle 每周期最大变化量
 * @return 斜坡处理后的结果
 */
static int16_t Motor_RampTarget(int16_t current_value, int16_t target_value, int16_t step_per_cycle)
{
    if (current_value < target_value)
    {
        current_value += step_per_cycle;
        if (current_value > target_value)
        {
            current_value = target_value;
        }
    }
    else if (current_value > target_value)
    {
        current_value -= step_per_cycle;
        if (current_value < target_value)
        {
            current_value = target_value;
        }
    }

    return current_value;
}

/**
 * @brief 根据带符号 PWM 输出设置单个电机方向和占空比
 * @param pwm_timer PWM 定时器
 * @param pwm_channel PWM 通道
 * @param in1_gpio_port 电机方向引脚 1 所在端口
 * @param in1_gpio_pin 电机方向引脚 1
 * @param in2_gpio_port 电机方向引脚 2 所在端口
 * @param in2_gpio_pin 电机方向引脚 2
 * @param pwm_output 带符号 PWM 输出
 */
static void Motor_ApplySingleWheel(TIM_HandleTypeDef *pwm_timer,
                                   uint32_t pwm_channel,
                                   GPIO_TypeDef *in1_gpio_port,
                                   uint16_t in1_gpio_pin,
                                   GPIO_TypeDef *in2_gpio_port,
                                   uint16_t in2_gpio_pin,
                                   int16_t pwm_output)
{
    GPIO_PinState in1_state = GPIO_PIN_RESET;
    GPIO_PinState in2_state = GPIO_PIN_RESET;
    uint16_t pwm_compare = (uint16_t)((pwm_output >= 0) ? pwm_output : -pwm_output);

    if (pwm_output >= 0)
    {
        in1_state = GPIO_PIN_SET;
        in2_state = GPIO_PIN_RESET;
    }
    else
    {
        in1_state = GPIO_PIN_RESET;
        in2_state = GPIO_PIN_SET;
    }

    HAL_GPIO_WritePin(in1_gpio_port, in1_gpio_pin, in1_state);
    HAL_GPIO_WritePin(in2_gpio_port, in2_gpio_pin, in2_state);

    __HAL_TIM_SET_COMPARE(pwm_timer, pwm_channel, pwm_compare);
}

/**
 * @brief 初始化单个速度 PI 控制器
 * @param controller PI 控制器句柄
 */
static void Motor_InitSpeedPI(MotorSpeedPI *controller)
{
    controller->proportional_gain = SPEED_PI_KP;
    controller->integral_gain = SPEED_PI_KI;
    controller->integral_sum = 0.0f;
    controller->integral_limit = SPEED_PI_INTEGRAL_LIMIT;
    controller->output_limit = MOTOR_PWM_LIMIT;
}

/**
 * @brief 初始化电机模块并启动 PWM 输出
 * @param handle 电机模块句柄
 * @param pwm_timer PWM 定时器
 */
void Motor_Init(MotorHandle *handle, TIM_HandleTypeDef *pwm_timer)
{
    handle->pwm_timer = pwm_timer;
    handle->left_pwm_channel = TIM_CHANNEL_3;
    handle->right_pwm_channel = TIM_CHANNEL_4;
    handle->requested_left_target = 0;
    handle->requested_right_target = 0;
    handle->ramped_left_target = 0;
    handle->ramped_right_target = 0;
    handle->left_pwm_output = 0;
    handle->right_pwm_output = 0;

    Motor_InitSpeedPI(&handle->left_speed_pi);
    Motor_InitSpeedPI(&handle->right_speed_pi);

    HAL_TIM_PWM_Start(handle->pwm_timer, handle->left_pwm_channel);
    HAL_TIM_PWM_Start(handle->pwm_timer, handle->right_pwm_channel);

    Motor_Stop(handle);
}

/**
 * @brief 设置左右轮的目标速度
 * @param handle 电机模块句柄
 * @param left_target_speed 左轮目标速度
 * @param right_target_speed 右轮目标速度
 */
void Motor_SetTargetSpeed(MotorHandle *handle, int16_t left_target_speed, int16_t right_target_speed)
{
    handle->requested_left_target = Motor_ClampS16(left_target_speed, -MOTOR_TARGET_LIMIT, MOTOR_TARGET_LIMIT);
    handle->requested_right_target = Motor_ClampS16(right_target_speed, -MOTOR_TARGET_LIMIT, MOTOR_TARGET_LIMIT);
}

/**
 * @brief 计算单轮速度 PI 输出
 * @param controller 速度 PI 控制器
 * @param target_speed 目标速度
 * @param feedback_speed 反馈速度
 * @return PI 输出值
 */
int16_t Motor_ComputeSpeedPI(MotorSpeedPI *controller, int16_t target_speed, int16_t feedback_speed)
{
    float speed_error = (float)target_speed - (float)feedback_speed;
    float controller_output;

    controller->integral_sum += speed_error;
    controller->integral_sum = Motor_ClampFloat(controller->integral_sum,
                                                -controller->integral_limit,
                                                controller->integral_limit);

    controller_output = controller->proportional_gain * speed_error +
                        controller->integral_gain * controller->integral_sum;

    return Motor_ClampS16((int32_t)controller_output,
                          (int16_t)(-controller->output_limit),
                          controller->output_limit);
}

/**
 * @brief 执行一次左右电机速度闭环计算并刷新 PWM
 * @param handle 电机模块句柄
 * @param left_feedback_speed 左轮速度反馈
 * @param right_feedback_speed 右轮速度反馈
 */
void Motor_UpdateClosedLoop(MotorHandle *handle, int16_t left_feedback_speed, int16_t right_feedback_speed)
{
    handle->ramped_left_target = Motor_RampTarget(handle->ramped_left_target,
                                                  handle->requested_left_target,
                                                  MOTOR_TARGET_RAMP_STEP);
    handle->ramped_right_target = Motor_RampTarget(handle->ramped_right_target,
                                                   handle->requested_right_target,
                                                   MOTOR_TARGET_RAMP_STEP);

    handle->left_pwm_output = Motor_ComputeSpeedPI(&handle->left_speed_pi,
                                                   handle->ramped_left_target,
                                                   left_feedback_speed);
    handle->right_pwm_output = Motor_ComputeSpeedPI(&handle->right_speed_pi,
                                                    handle->ramped_right_target,
                                                    right_feedback_speed);

    Motor_ApplySingleWheel(handle->pwm_timer,
                           handle->left_pwm_channel,
                           GPIOB,
                           GPIO_PIN_13,
                           GPIOB,
                           GPIO_PIN_12,
                           handle->left_pwm_output);
    Motor_ApplySingleWheel(handle->pwm_timer,
                           handle->right_pwm_channel,
                           GPIOB,
                           GPIO_PIN_15,
                           GPIOB,
                           GPIO_PIN_14,
                           handle->right_pwm_output);
}

/**
 * @brief 关闭电机输出并清空 PI 积分项
 * @param handle 电机模块句柄
 */
void Motor_Stop(MotorHandle *handle)
{
    handle->requested_left_target = 0;
    handle->requested_right_target = 0;
    handle->ramped_left_target = 0;
    handle->ramped_right_target = 0;
    handle->left_pwm_output = 0;
    handle->right_pwm_output = 0;
    handle->left_speed_pi.integral_sum = 0.0f;
    handle->right_speed_pi.integral_sum = 0.0f;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(handle->pwm_timer, handle->left_pwm_channel, 0);
    __HAL_TIM_SET_COMPARE(handle->pwm_timer, handle->right_pwm_channel, 0);
}
