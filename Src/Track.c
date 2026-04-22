#include "Track.h"
#include "tim.h"

/**
 * @brief 将整数值限制在指定范围内
 * @param input_value 输入值
 * @param lower_limit 下限
 * @param upper_limit 上限
 * @return 限幅后的结果
 */
static int16_t Track_ClampS16(int32_t input_value, int16_t lower_limit, int16_t upper_limit)
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
 * @brief 读取单个红外数字传感器并转换为黑线=1、白底=0
 * @param gpio_port 传感器所在 GPIO 端口
 * @param gpio_pin 传感器所在 GPIO 引脚
 * @return 黑线返回 1，白底返回 0
 */
static uint8_t Track_ReadBinarySensor(GPIO_TypeDef *gpio_port, uint16_t gpio_pin)
{
    return (HAL_GPIO_ReadPin(gpio_port, gpio_pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

/**
 * @brief 读取四路循迹传感器并填充结构体
 * @param sensor_state 传感器状态输出结构体
 */
static void Track_ReadSensors(TrackSensorState *sensor_state)
{
    sensor_state->left_outer = Track_ReadBinarySensor(GPIOA, GPIO_PIN_12);
    sensor_state->left_inner = Track_ReadBinarySensor(GPIOA, GPIO_PIN_11);
    sensor_state->right_inner = Track_ReadBinarySensor(GPIOA, GPIO_PIN_10);
    sensor_state->right_outer = Track_ReadBinarySensor(GPIOA, GPIO_PIN_9);

    sensor_state->active_count = (uint8_t)(sensor_state->left_outer +
                                           sensor_state->left_inner +
                                           sensor_state->right_inner +
                                           sensor_state->right_outer);
    sensor_state->all_black = (sensor_state->active_count == 4U) ? 1U : 0U;
    sensor_state->all_white = (sensor_state->active_count == 0U) ? 1U : 0U;
}

/**
 * @brief 采用四路权重加权平均算法计算当前误差
 * @param sensor_state 当前传感器状态
 * @param last_valid_error 上一次有效误差
 * @return 加权平均误差；若全白则保持上一次有效误差方向
 */
static float Track_ComputeWeightedError(const TrackSensorState *sensor_state, float last_valid_error)
{
    static const float sensor_weights[4] = {-3.0f, -1.0f, 1.0f, 3.0f};
    float weighted_sum = 0.0f;
    float sensor_sum = 0.0f;
    uint8_t sensor_values[4];
    uint8_t index;

    sensor_values[0] = sensor_state->left_outer;
    sensor_values[1] = sensor_state->left_inner;
    sensor_values[2] = sensor_state->right_inner;
    sensor_values[3] = sensor_state->right_outer;

    for (index = 0U; index < 4U; ++index)
    {
        weighted_sum += sensor_weights[index] * (float)sensor_values[index];
        sensor_sum += (float)sensor_values[index];
    }

    if (sensor_sum <= 0.0f)
    {
        return last_valid_error;
    }

    return weighted_sum / sensor_sum;
}

/**
 * @brief 更新外侧传感器连续触发计数并记录待转向方向
 * @param handle 循迹模块句柄
 */
static void Track_UpdateOuterTriggerCounters(TrackHandle *handle)
{
    if ((handle->sensors.left_outer != 0U) && (handle->sensors.right_outer == 0U))
    {
        if (handle->left_outer_counter < 255U)
        {
            handle->left_outer_counter++;
        }

        if (handle->right_outer_counter > OUTER_TRIGGER_DECAY_STEP)
        {
            handle->right_outer_counter -= OUTER_TRIGGER_DECAY_STEP;
        }
        else
        {
            handle->right_outer_counter = 0U;
        }

        handle->pending_turn_direction = -1;
    }
    else if ((handle->sensors.right_outer != 0U) && (handle->sensors.left_outer == 0U))
    {
        if (handle->right_outer_counter < 255U)
        {
            handle->right_outer_counter++;
        }

        if (handle->left_outer_counter > OUTER_TRIGGER_DECAY_STEP)
        {
            handle->left_outer_counter -= OUTER_TRIGGER_DECAY_STEP;
        }
        else
        {
            handle->left_outer_counter = 0U;
        }

        handle->pending_turn_direction = 1;
    }
    else
    {
        if (handle->left_outer_counter > OUTER_TRIGGER_DECAY_STEP)
        {
            handle->left_outer_counter -= OUTER_TRIGGER_DECAY_STEP;
        }
        else
        {
            handle->left_outer_counter = 0U;
        }

        if (handle->right_outer_counter > OUTER_TRIGGER_DECAY_STEP)
        {
            handle->right_outer_counter -= OUTER_TRIGGER_DECAY_STEP;
        }
        else
        {
            handle->right_outer_counter = 0U;
        }
    }
}

/**
 * @brief 将基准速度和转向量转换为左右轮目标速度
 * @param handle 循迹模块句柄
 * @param base_speed 本周期基准速度
 * @param turn_output 本周期转向输出
 */
static void Track_OutputTrackingTarget(TrackHandle *handle, int16_t base_speed, float turn_output)
{
    int16_t turn_value = Track_ClampS16((int32_t)turn_output,
                                        -TRACK_TARGET_LIMIT,
                                        TRACK_TARGET_LIMIT);

    handle->base_speed = base_speed;
    handle->pd_turn_output = turn_output;
    handle->left_target_speed = Track_ClampS16((int32_t)base_speed - turn_value,
                                               0,
                                               TRACK_TARGET_LIMIT);
    handle->right_target_speed = Track_ClampS16((int32_t)base_speed + turn_value,
                                                0,
                                                TRACK_TARGET_LIMIT);
}

/**
 * @brief 进入指定循迹状态并清理该状态相关计数
 * @param handle 循迹模块句柄
 * @param next_state 下一状态
 */
static void Track_EnterState(TrackHandle *handle, TrackState next_state)
{
    handle->state = next_state;
    handle->state_tick = 0U;
    handle->turn_exit_counter = 0U;
    handle->pd_turn_output = 0.0f;
}

/**
 * @brief 检查 TURN 状态是否满足退出条件
 * @param handle 循迹模块句柄
 * @return 满足退出条件返回 1，否则返回 0
 */
static uint8_t Track_ShouldExitTurn(TrackHandle *handle)
{
    uint8_t line_pattern_recovered = 0U;
    float absolute_error = (handle->current_error >= 0.0f) ? handle->current_error : -handle->current_error;

    if ((handle->sensors.left_inner != 0U) && (handle->sensors.right_inner != 0U))
    {
        line_pattern_recovered = 1U;
    }
    else if ((handle->sensors.active_count >= 2U) && (absolute_error <= TURN_EXIT_ERROR_THRESHOLD))
    {
        line_pattern_recovered = 1U;
    }

    if (line_pattern_recovered != 0U)
    {
        if (handle->turn_exit_counter < 255U)
        {
            handle->turn_exit_counter++;
        }
    }
    else
    {
        handle->turn_exit_counter = 0U;
    }

    if (handle->turn_exit_counter >= TURN_EXIT_CONFIRM_TICKS)
    {
        return 1U;
    }

    if (handle->state_tick >= TURN_TIMEOUT_TICKS)
    {
        return 1U;
    }

    return 0U;
}

/**
 * @brief 在 TRACKING 或 APPROACH 状态下执行 PD 外环控制
 * @param handle 循迹模块句柄
 * @param proportional_gain 比例系数
 * @param derivative_gain 微分系数
 * @param base_speed_command 基准速度
 */
static void Track_RunPDControl(TrackHandle *handle,
                               float proportional_gain,
                               float derivative_gain,
                               int16_t base_speed_command)
{
    float effective_error = handle->current_error;
    float error_delta;
    float turn_output;

    if (handle->sensors.all_black != 0U)
    {
        effective_error = FULL_BLACK_ERROR_BAND;
        base_speed_command = (int16_t)((BASE_SPEED * FULL_BLACK_SPEED_RATIO) / 100);
    }
    else if (handle->sensors.all_white != 0U)
    {
        if (handle->last_valid_error > 0.1f)
        {
            effective_error = LOST_LINE_HOLD_ERROR;
        }
        else if (handle->last_valid_error < -0.1f)
        {
            effective_error = -LOST_LINE_HOLD_ERROR;
        }
        else if (handle->pending_turn_direction > 0)
        {
            effective_error = LOST_LINE_HOLD_ERROR;
        }
        else if (handle->pending_turn_direction < 0)
        {
            effective_error = -LOST_LINE_HOLD_ERROR;
        }
        else
        {
            effective_error = 0.0f;
        }

        base_speed_command = (int16_t)((BASE_SPEED * LOST_LINE_SPEED_RATIO) / 100);
    }
    else
    {
        handle->last_valid_error = handle->current_error;
    }

    error_delta = effective_error - handle->last_error;
    turn_output = proportional_gain * effective_error + derivative_gain * error_delta;

    handle->last_error = effective_error;
    Track_OutputTrackingTarget(handle, base_speed_command, turn_output);
}

/**
 * @brief 输出 TURN_LEFT 固定目标轮速，完全禁用 PD
 * @param handle 循迹模块句柄
 */
static void Track_RunTurnLeft(TrackHandle *handle)
{
    handle->base_speed = BASE_SPEED;
    handle->pd_turn_output = 0.0f;
    handle->left_target_speed = Track_ClampS16((int32_t)(BASE_SPEED * TURN_INNER_RATIO),
                                               -TRACK_TARGET_LIMIT,
                                               TRACK_TARGET_LIMIT);
    handle->right_target_speed = Track_ClampS16((int32_t)(BASE_SPEED * TURN_OUTER_RATIO),
                                                -TRACK_TARGET_LIMIT,
                                                TRACK_TARGET_LIMIT);
}

/**
 * @brief 输出 TURN_RIGHT 固定目标轮速，完全禁用 PD
 * @param handle 循迹模块句柄
 */
static void Track_RunTurnRight(TrackHandle *handle)
{
    handle->base_speed = BASE_SPEED;
    handle->pd_turn_output = 0.0f;
    handle->left_target_speed = Track_ClampS16((int32_t)(BASE_SPEED * TURN_OUTER_RATIO),
                                               -TRACK_TARGET_LIMIT,
                                               TRACK_TARGET_LIMIT);
    handle->right_target_speed = Track_ClampS16((int32_t)(BASE_SPEED * TURN_INNER_RATIO),
                                                -TRACK_TARGET_LIMIT,
                                                TRACK_TARGET_LIMIT);
}

/**
 * @brief 初始化循迹模块句柄
 * @param handle 循迹模块句柄
 */
void Track_Init(TrackHandle *handle)
{
    handle->state = TRACKING;
    handle->current_error = 0.0f;
    handle->last_error = 0.0f;
    handle->last_valid_error = 0.0f;
    handle->pd_turn_output = 0.0f;
    handle->base_speed = BASE_SPEED;
    handle->left_target_speed = 0;
    handle->right_target_speed = 0;
    handle->left_actual_speed = 0;
    handle->right_actual_speed = 0;
    handle->left_encoder_count = 0;
    handle->right_encoder_count = 0;
    handle->last_left_encoder_count = 0;
    handle->last_right_encoder_count = 0;
    handle->left_speed_error = 0.0f;
    handle->right_speed_error = 0.0f;
    handle->last_left_speed_error = 0.0f;
    handle->last_right_speed_error = 0.0f;
    handle->left_outer_counter = 0U;
    handle->right_outer_counter = 0U;
    handle->turn_exit_counter = 0U;
    handle->state_tick = 0U;
    handle->pending_turn_direction = 0;

    Track_ReadSensors(&handle->sensors);
}

/**
 * @brief 执行一次状态机更新并输出左右轮目标速度
 * @param handle 循迹模块句柄
 */
void Track_Update(TrackHandle *handle)
{
    // 1. 更新编码器计数和实际速度
    Track_UpdateEncoders(handle);
    
    // 2. 读取传感器并计算误差
    Track_ReadSensors(&handle->sensors);
    handle->current_error = Track_ComputeWeightedError(&handle->sensors, handle->last_valid_error);
    Track_UpdateOuterTriggerCounters(handle);

    handle->state_tick++;

    switch (handle->state)
    {
        case TRACKING:
        {
            if ((handle->left_outer_counter >= OUTER_TRIGGER_TURN_COUNT) &&
                (handle->right_outer_counter == 0U))
            {
                Track_EnterState(handle, TURN_LEFT);
                Track_RunTurnLeft(handle);
                Track_RunSpeedControl(handle);
                break;
            }

            if ((handle->right_outer_counter >= OUTER_TRIGGER_TURN_COUNT) &&
                (handle->left_outer_counter == 0U))
            {
                Track_EnterState(handle, TURN_RIGHT);
                Track_RunTurnRight(handle);
                Track_RunSpeedControl(handle);
                break;
            }

            if (((handle->sensors.left_outer != 0U) ^ (handle->sensors.right_outer != 0U)) != 0U)
            {
                Track_EnterState(handle, APPROACH);
                Track_RunPDControl(handle,
                                   APPROACH_KP,
                                   APPROACH_KD,
                                   (int16_t)((BASE_SPEED * APPROACH_BASE_SPEED_RATIO) / 100));
                Track_RunSpeedControl(handle);
                break;
            }

            Track_RunPDControl(handle, TRACK_KP, TRACK_KD, BASE_SPEED);
            Track_RunSpeedControl(handle);
            break;
        }

        case APPROACH:
        {
            if ((handle->left_outer_counter >= OUTER_TRIGGER_TURN_COUNT) &&
                (handle->pending_turn_direction <= 0))
            {
                Track_EnterState(handle, TURN_LEFT);
                Track_RunTurnLeft(handle);
                Track_RunSpeedControl(handle);
                break;
            }

            if ((handle->right_outer_counter >= OUTER_TRIGGER_TURN_COUNT) &&
                (handle->pending_turn_direction >= 0))
            {
                Track_EnterState(handle, TURN_RIGHT);
                Track_RunTurnRight(handle);
                Track_RunSpeedControl(handle);
                break;
            }

            if ((handle->sensors.left_outer == 0U) &&
                (handle->sensors.right_outer == 0U) &&
                (handle->state_tick >= APPROACH_HOLD_TICKS))
            {
                Track_EnterState(handle, TRACKING);
                Track_RunPDControl(handle, TRACK_KP, TRACK_KD, BASE_SPEED);
                Track_RunSpeedControl(handle);
                break;
            }

            Track_RunPDControl(handle,
                               APPROACH_KP,
                               APPROACH_KD,
                               (int16_t)((BASE_SPEED * APPROACH_BASE_SPEED_RATIO) / 100));
            Track_RunSpeedControl(handle);
            break;
        }

        case TURN_LEFT:
        {
            Track_RunTurnLeft(handle);
            Track_RunSpeedControl(handle);

            if (Track_ShouldExitTurn(handle) != 0U)
            {
                Track_EnterState(handle, TRACKING);
                handle->last_error = 0.0f;
                Track_RunPDControl(handle, TRACK_KP, TRACK_KD, BASE_SPEED);
                Track_RunSpeedControl(handle);
            }
            break;
        }

        case TURN_RIGHT:
        {
            Track_RunTurnRight(handle);
            Track_RunSpeedControl(handle);

            if (Track_ShouldExitTurn(handle) != 0U)
            {
                Track_EnterState(handle, TRACKING);
                handle->last_error = 0.0f;
                Track_RunPDControl(handle, TRACK_KP, TRACK_KD, BASE_SPEED);
                Track_RunSpeedControl(handle);
            }
            break;
        }

        default:
        {
            Track_EnterState(handle, TRACKING);
            Track_RunPDControl(handle, TRACK_KP, TRACK_KD, BASE_SPEED);
            Track_RunSpeedControl(handle);
            break;
        }
    }
}

/**
 * @brief 更新编码器计数并计算实际速度
 * @param handle 循迹模块句柄
 */
void Track_UpdateEncoders(TrackHandle *handle)
{
    int32_t current_left_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    int32_t current_right_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
    int32_t left_count_diff = current_left_count - handle->last_left_encoder_count;
    int32_t right_count_diff = current_right_count - handle->last_right_encoder_count;
    
    // 计算速度 (单位: 脉冲/控制周期)
    handle->left_actual_speed = (int16_t)left_count_diff;
    handle->right_actual_speed = (int16_t)right_count_diff;
    
    // 更新历史计数
    handle->last_left_encoder_count = current_left_count;
    handle->last_right_encoder_count = current_right_count;
}

/**
 * @brief 执行速度内环PD控制
 * @param handle 循迹模块句柄
 */
void Track_RunSpeedControl(TrackHandle *handle)
{
    float left_error_delta, right_error_delta;
    float left_speed_output, right_speed_output;
    int16_t left_pwm, right_pwm;
    
    // 计算速度误差
    handle->left_speed_error = (float)(handle->left_target_speed - handle->left_actual_speed);
    handle->right_speed_error = (float)(handle->right_target_speed - handle->right_actual_speed);
    
    // 计算误差变化率
    left_error_delta = handle->left_speed_error - handle->last_left_speed_error;
    right_error_delta = handle->right_speed_error - handle->last_right_speed_error;
    
    // PD控制计算
    left_speed_output = SPEED_KP * handle->left_speed_error + SPEED_KD * left_error_delta;
    right_speed_output = SPEED_KP * handle->right_speed_error + SPEED_KD * right_error_delta;
    
    // 更新历史误差
    handle->last_left_speed_error = handle->left_speed_error;
    handle->last_right_speed_error = handle->right_speed_error;
    
    // 计算最终PWM值
    left_pwm = Track_ClampS16((int32_t)(handle->left_target_speed + left_speed_output),
                              0, SPEED_TARGET_LIMIT);
    right_pwm = Track_ClampS16((int32_t)(handle->right_target_speed + right_speed_output),
                               0, SPEED_TARGET_LIMIT);
    
    // 应用PWM输出
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, left_pwm);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, right_pwm);
}
