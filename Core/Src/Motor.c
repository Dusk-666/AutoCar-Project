#include "Motor.h"

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

static int16_t Motor_AbsS16(int16_t input_value)
{
    return (input_value >= 0) ? input_value : (int16_t)(-input_value);
}

static int16_t Motor_RoundFloatToS16(float input_value)
{
    if (input_value >= 0.0f)
    {
        return (int16_t)(input_value + 0.5f);
    }

    return (int16_t)(input_value - 0.5f);
}

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

static uint8_t Motor_IsNearZero(int16_t target_speed, int16_t feedback_speed, int16_t stop_window)
{
    return (uint8_t)((Motor_AbsS16(target_speed) <= stop_window) &&
                     (Motor_AbsS16(feedback_speed) <= stop_window));
}

static int16_t Motor_ComputeStraightBalance(const MotorHandle *handle)
{
    int16_t target_delta = (int16_t)(handle->ramped_left_target - handle->ramped_right_target);
    float speed_delta;

    if ((handle->ramped_left_target <= handle->params.stop_window) ||
        (handle->ramped_right_target <= handle->params.stop_window) ||
        (Motor_AbsS16(target_delta) > handle->params.straight_balance_window))
    {
        return 0;
    }

    speed_delta = (float)handle->left_feedback_speed - (float)handle->right_feedback_speed;

    return Motor_ClampS16(Motor_RoundFloatToS16(speed_delta * handle->params.straight_balance_gain),
                          (int16_t)(-handle->params.straight_balance_limit),
                          handle->params.straight_balance_limit);
}

static void Motor_ApplySingleWheel(TIM_HandleTypeDef *pwm_timer,
                                   uint32_t pwm_channel,
                                   GPIO_TypeDef *forward_port,
                                   uint16_t forward_pin,
                                   GPIO_TypeDef *reverse_port,
                                   uint16_t reverse_pin,
                                   int16_t pwm_output,
                                   int16_t pwm_limit)
{
    int16_t clamped_output = Motor_ClampS16(pwm_output, (int16_t)(-pwm_limit), pwm_limit);
    uint16_t pwm_compare = (uint16_t)Motor_AbsS16(clamped_output);

    if (clamped_output > 0)
    {
        HAL_GPIO_WritePin(forward_port, forward_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(reverse_port, reverse_pin, GPIO_PIN_RESET);
    }
    else if (clamped_output < 0)
    {
        HAL_GPIO_WritePin(forward_port, forward_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(reverse_port, reverse_pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(forward_port, forward_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(reverse_port, reverse_pin, GPIO_PIN_RESET);
        pwm_compare = 0U;
    }

    __HAL_TIM_SET_COMPARE(pwm_timer, pwm_channel, pwm_compare);
}

static void Motor_InitSpeedPI(MotorSpeedPI *controller, int16_t output_limit)
{
    controller->proportional_gain = MOTOR_DEFAULT_SPEED_KP;
    controller->integral_gain = MOTOR_DEFAULT_SPEED_KI;
    controller->integral_sum = 0.0f;
    controller->integral_limit = MOTOR_DEFAULT_INTEGRAL_LIMIT;
    controller->output_limit = output_limit;
}

void Motor_Init(MotorHandle *handle, TIM_HandleTypeDef *pwm_timer)
{
    int16_t timer_pwm_limit;
    int16_t pwm_limit;
    int16_t right_pwm_limit;

    handle->pwm_timer = pwm_timer;
    handle->left_pwm_channel = MOTOR_LEFT_PWM_CHANNEL;
    handle->right_pwm_channel = MOTOR_RIGHT_PWM_CHANNEL;
    handle->params.target_limit = MOTOR_DEFAULT_TARGET_LIMIT;
    handle->params.target_ramp_step = MOTOR_DEFAULT_TARGET_RAMP_STEP;
    handle->params.stop_window = MOTOR_DEFAULT_STOP_WINDOW;
    handle->params.right_wheel_scale = MOTOR_DEFAULT_RIGHT_WHEEL_SCALE;
    handle->params.straight_balance_window = MOTOR_DEFAULT_STRAIGHT_BALANCE_WINDOW;
    handle->params.straight_balance_limit = MOTOR_DEFAULT_STRAIGHT_BALANCE_LIMIT;
    handle->params.straight_balance_gain = MOTOR_DEFAULT_STRAIGHT_BALANCE_GAIN;
    handle->requested_left_target = 0;
    handle->requested_right_target = 0;
    handle->ramped_left_target = 0;
    handle->ramped_right_target = 0;
    handle->left_feedback_speed = 0;
    handle->right_feedback_speed = 0;
    handle->left_pwm_output = 0;
    handle->right_pwm_output = 0;

    timer_pwm_limit = (int16_t)__HAL_TIM_GET_AUTORELOAD(handle->pwm_timer);
    pwm_limit = Motor_ClampS16(MOTOR_DEFAULT_PWM_MAX_LIMIT, 0, timer_pwm_limit);
    right_pwm_limit = Motor_ClampS16(Motor_RoundFloatToS16((float)pwm_limit * handle->params.right_wheel_scale),
                                     0,
                                     timer_pwm_limit);
    Motor_InitSpeedPI(&handle->left_speed_pi, pwm_limit);
    Motor_InitSpeedPI(&handle->right_speed_pi, right_pwm_limit);

    HAL_TIM_PWM_Start(handle->pwm_timer, handle->left_pwm_channel);
    HAL_TIM_PWM_Start(handle->pwm_timer, handle->right_pwm_channel);

    Motor_Stop(handle);
}

void Motor_SetTargetSpeed(MotorHandle *handle, int16_t left_target_speed, int16_t right_target_speed)
{
    int16_t right_target_limit = Motor_ClampS16(Motor_RoundFloatToS16((float)handle->params.target_limit *
                                                                      handle->params.right_wheel_scale),
                                                0,
                                                handle->right_speed_pi.output_limit);

    handle->requested_left_target = Motor_ClampS16(left_target_speed,
                                                   (int16_t)(-handle->params.target_limit),
                                                   handle->params.target_limit);
    handle->requested_right_target = Motor_ClampS16(Motor_RoundFloatToS16((float)right_target_speed *
                                                                          handle->params.right_wheel_scale),
                                                    (int16_t)(-right_target_limit),
                                                    right_target_limit);
}

int16_t Motor_ComputeSpeedPI(MotorSpeedPI *controller, int16_t target_speed, int16_t feedback_speed)
{
    float speed_error = (float)target_speed - (float)feedback_speed;
    float proportional_output = controller->proportional_gain * speed_error;
    float integral_candidate = controller->integral_sum + speed_error;
    float output_candidate;
    float controller_output;

    integral_candidate = Motor_ClampFloat(integral_candidate,
                                          -controller->integral_limit,
                                          controller->integral_limit);

    output_candidate = proportional_output + controller->integral_gain * integral_candidate;
    if (((output_candidate > (float)controller->output_limit) && (speed_error > 0.0f)) ||
        ((output_candidate < -(float)controller->output_limit) && (speed_error < 0.0f)))
    {
        integral_candidate = controller->integral_sum;
    }

    controller->integral_sum = integral_candidate;
    controller_output = proportional_output + controller->integral_gain * controller->integral_sum;

    if ((target_speed >= 0) && (controller_output < 0.0f))
    {
        controller_output = 0.0f;
    }
    else if ((target_speed <= 0) && (controller_output > 0.0f))
    {
        controller_output = 0.0f;
    }

    return Motor_ClampS16(Motor_RoundFloatToS16(controller_output),
                          (int16_t)(-controller->output_limit),
                          controller->output_limit);
}

void Motor_UpdateClosedLoop(MotorHandle *handle, int16_t left_feedback_speed, int16_t right_feedback_speed)
{
    int16_t straight_balance;
    int16_t left_target;
    int16_t right_target;

    handle->left_feedback_speed = left_feedback_speed;
    handle->right_feedback_speed = right_feedback_speed;

    handle->ramped_left_target = Motor_RampTarget(handle->ramped_left_target,
                                                  handle->requested_left_target,
                                                  handle->params.target_ramp_step);
    handle->ramped_right_target = Motor_RampTarget(handle->ramped_right_target,
                                                   handle->requested_right_target,
                                                   handle->params.target_ramp_step);

    straight_balance = Motor_ComputeStraightBalance(handle);
    left_target = Motor_ClampS16((int32_t)handle->ramped_left_target - straight_balance,
                                 (int16_t)(-handle->params.target_limit),
                                 handle->params.target_limit);
    right_target = Motor_ClampS16((int32_t)handle->ramped_right_target + straight_balance,
                                  (int16_t)(-handle->params.target_limit),
                                  handle->params.target_limit);

    if (Motor_IsNearZero(left_target,
                         handle->left_feedback_speed,
                         handle->params.stop_window) != 0U)
    {
        handle->left_speed_pi.integral_sum = 0.0f;
        handle->left_pwm_output = 0;
    }
    else
    {
        handle->left_pwm_output = Motor_ComputeSpeedPI(&handle->left_speed_pi,
                                                       left_target,
                                                       handle->left_feedback_speed);
    }

    if (Motor_IsNearZero(right_target,
                         handle->right_feedback_speed,
                         handle->params.stop_window) != 0U)
    {
        handle->right_speed_pi.integral_sum = 0.0f;
        handle->right_pwm_output = 0;
    }
    else
    {
        handle->right_pwm_output = Motor_ComputeSpeedPI(&handle->right_speed_pi,
                                                        right_target,
                                                        handle->right_feedback_speed);
    }

    Motor_ApplySingleWheel(handle->pwm_timer,
                           handle->left_pwm_channel,
                           MOTOR_LEFT_DIR_FORWARD_PORT,
                           MOTOR_LEFT_DIR_FORWARD_PIN,
                           MOTOR_LEFT_DIR_REVERSE_PORT,
                           MOTOR_LEFT_DIR_REVERSE_PIN,
                           handle->left_pwm_output,
                           handle->left_speed_pi.output_limit);
    Motor_ApplySingleWheel(handle->pwm_timer,
                           handle->right_pwm_channel,
                           MOTOR_RIGHT_DIR_FORWARD_PORT,
                           MOTOR_RIGHT_DIR_FORWARD_PIN,
                           MOTOR_RIGHT_DIR_REVERSE_PORT,
                           MOTOR_RIGHT_DIR_REVERSE_PIN,
                           handle->right_pwm_output,
                           handle->right_speed_pi.output_limit);
}

void Motor_ApplyRawOutput(MotorHandle *handle, int16_t left_pwm_output, int16_t right_pwm_output)
{
    handle->left_pwm_output = Motor_ClampS16(left_pwm_output,
                                             (int16_t)(-handle->left_speed_pi.output_limit),
                                             handle->left_speed_pi.output_limit);
    handle->right_pwm_output = Motor_ClampS16(right_pwm_output,
                                              (int16_t)(-handle->right_speed_pi.output_limit),
                                              handle->right_speed_pi.output_limit);

    Motor_ApplySingleWheel(handle->pwm_timer,
                           handle->left_pwm_channel,
                           MOTOR_LEFT_DIR_FORWARD_PORT,
                           MOTOR_LEFT_DIR_FORWARD_PIN,
                           MOTOR_LEFT_DIR_REVERSE_PORT,
                           MOTOR_LEFT_DIR_REVERSE_PIN,
                           handle->left_pwm_output,
                           handle->left_speed_pi.output_limit);
    Motor_ApplySingleWheel(handle->pwm_timer,
                           handle->right_pwm_channel,
                           MOTOR_RIGHT_DIR_FORWARD_PORT,
                           MOTOR_RIGHT_DIR_FORWARD_PIN,
                           MOTOR_RIGHT_DIR_REVERSE_PORT,
                           MOTOR_RIGHT_DIR_REVERSE_PIN,
                           handle->right_pwm_output,
                           handle->right_speed_pi.output_limit);
}

void Motor_Stop(MotorHandle *handle)
{
    handle->requested_left_target = 0;
    handle->requested_right_target = 0;
    handle->ramped_left_target = 0;
    handle->ramped_right_target = 0;
    handle->left_feedback_speed = 0;
    handle->right_feedback_speed = 0;
    handle->left_pwm_output = 0;
    handle->right_pwm_output = 0;
    handle->left_speed_pi.integral_sum = 0.0f;
    handle->right_speed_pi.integral_sum = 0.0f;

    HAL_GPIO_WritePin(MOTOR_LEFT_DIR_FORWARD_PORT, MOTOR_LEFT_DIR_FORWARD_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_LEFT_DIR_REVERSE_PORT, MOTOR_LEFT_DIR_REVERSE_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_RIGHT_DIR_FORWARD_PORT, MOTOR_RIGHT_DIR_FORWARD_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_RIGHT_DIR_REVERSE_PORT, MOTOR_RIGHT_DIR_REVERSE_PIN, GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(handle->pwm_timer, handle->left_pwm_channel, 0);
    __HAL_TIM_SET_COMPARE(handle->pwm_timer, handle->right_pwm_channel, 0);
}
