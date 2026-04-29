#include "Track.h"


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

static float Track_ClampFloat(float input_value, float lower_limit, float upper_limit)
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

static float Track_AbsFloat(float input_value)
{
    return (input_value >= 0.0f) ? input_value : -input_value;
}

static int16_t Track_RoundFloatToS16(float input_value)
{
    if (input_value >= 0.0f)
    {
        return (int16_t)(input_value + 0.5f);
    }

    return (int16_t)(input_value - 0.5f);
}

static int16_t Track_AbsS16(int16_t input_value)
{
    return (input_value >= 0) ? input_value : (int16_t)(-input_value);
}

static uint16_t Track_StepServoPulse(uint16_t current_pulse, uint16_t target_pulse, uint16_t step_pulse)
{
    if (current_pulse < target_pulse)
    {
        uint32_t next_pulse = (uint32_t)current_pulse + step_pulse;
        if (next_pulse > target_pulse)
        {
            next_pulse = target_pulse;
        }
        return (uint16_t)next_pulse;
    }

    if (current_pulse > target_pulse)
    {
        uint16_t delta_pulse = current_pulse - target_pulse;
        if (delta_pulse > step_pulse)
        {
            return (uint16_t)(current_pulse - step_pulse);
        }
        return target_pulse;
    }

    return current_pulse;
}

static void Track_SelectSensorChannel(uint8_t channel_index)
{
    HAL_GPIO_WritePin(TRACK_SENSOR_AD0_PORT,
                      TRACK_SENSOR_AD0_PIN,
                      ((channel_index & 0x01U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TRACK_SENSOR_AD1_PORT,
                      TRACK_SENSOR_AD1_PIN,
                      ((channel_index & 0x02U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TRACK_SENSOR_AD2_PORT,
                      TRACK_SENSOR_AD2_PIN,
                      ((channel_index & 0x04U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Track_WaitSensorAddressSettle(void)
{
    volatile uint32_t delay_index;

    for (delay_index = 0U; delay_index < TRACK_SENSOR_ADDRESS_SETTLE_NOP_COUNT; delay_index++)
    {
        __NOP();
    }
}

static uint8_t Track_ReadSelectedSensor(void)
{
    return (HAL_GPIO_ReadPin(TRACK_SENSOR_OUT_PORT, TRACK_SENSOR_OUT_PIN) == TRACK_SENSOR_ACTIVE_LEVEL) ? 1U : 0U;
}

static void Track_ReadSensors(TrackSensorSample *sample)
{
    sample->mask = 0U;
    sample->active_count = 0U;

    for (uint8_t channel_index = 0U; channel_index < TRACK_SENSOR_CHANNEL_COUNT; channel_index++)
    {
        uint8_t channel_state;

        Track_SelectSensorChannel(channel_index);
        Track_WaitSensorAddressSettle();
        channel_state = Track_ReadSelectedSensor();
        sample->channel[channel_index] = channel_state;

        if (channel_state != 0U)
        {
            sample->mask |= (uint8_t)(1U << ((TRACK_SENSOR_CHANNEL_COUNT - 1U) - channel_index));
            sample->active_count++;
        }
    }
}

static uint8_t Track_HasActiveLeftEdge(const TrackSensorSample *sample)
{
    return sample->channel[0];
}

static uint8_t Track_HasActiveRightEdge(const TrackSensorSample *sample)
{
    return sample->channel[TRACK_SENSOR_CHANNEL_COUNT - 1U];
}

static uint8_t Track_HasExclusiveOuterTrigger(const TrackSensorSample *sample)
{
    return (uint8_t)(Track_HasActiveLeftEdge(sample) ^ Track_HasActiveRightEdge(sample));
}

static void Track_UpdateServoPulse(TrackHandle *handle)
{
    if (Track_HasExclusiveOuterTrigger(&handle->sensors) != 0U)
    {
        if (Track_HasActiveLeftEdge(&handle->sensors) != 0U)
        {
            handle->servo_target_pulse_us = TRACK_SERVO_CENTER_PULSE_US + TRACK_SERVO_TURN_OFFSET_US;
        }
        else
        {
            handle->servo_target_pulse_us = TRACK_SERVO_CENTER_PULSE_US - TRACK_SERVO_TURN_OFFSET_US;
        }
    }
    else
    {
        handle->servo_target_pulse_us = TRACK_SERVO_CENTER_PULSE_US;
    }

    handle->servo_pulse_us = Track_StepServoPulse(handle->servo_pulse_us,
                                                  handle->servo_target_pulse_us,
                                                  handle->params.servo_slew_step_us);
}

static void Track_CapturePassServoDirection(TrackHandle *handle)
{
    if (handle->servo_pulse_us > TRACK_SERVO_CENTER_PULSE_US)
    {
        handle->pass_servo_direction = 1;
    }
    else if (handle->servo_pulse_us < TRACK_SERVO_CENTER_PULSE_US)
    {
        handle->pass_servo_direction = -1;
    }
    else
    {
        handle->pass_servo_direction = 0;
    }
}

static void Track_UpdatePassServoPulse(TrackHandle *handle)
{
    if (handle->pass_servo_direction > 0)
    {
        handle->servo_target_pulse_us = TRACK_SERVO_CENTER_PULSE_US + TRACK_SERVO_TURN_OFFSET_US;
    }
    else if (handle->pass_servo_direction < 0)
    {
        handle->servo_target_pulse_us = TRACK_SERVO_CENTER_PULSE_US - TRACK_SERVO_TURN_OFFSET_US;
    }
    else
    {
        handle->servo_target_pulse_us = TRACK_SERVO_CENTER_PULSE_US;
    }

    handle->servo_pulse_us = Track_StepServoPulse(handle->servo_pulse_us,
                                                  handle->servo_target_pulse_us,
                                                  handle->params.servo_slew_step_us);
}

static TrackPatternClass Track_ClassifyPattern(const TrackSensorSample *sample)
{
    if (sample->active_count == 0U)
    {
        return TRACK_PATTERN_ALL_WHITE;
    }

    if (sample->active_count >= TRACK_SENSOR_CHANNEL_COUNT)
    {
        return TRACK_PATTERN_ALL_BLACK;
    }

    return TRACK_PATTERN_LINE;
}

static float Track_ComputeLineError(const TrackHandle *handle)
{
    float accumulated_error = 0.0f;
    float active_weight = 0.0f;
    float left_outer_weight = handle->params.error_outer;
    float left_inner_weight = handle->params.error_inner;
    float left_mid_outer_weight = ((2.0f * left_outer_weight) + left_inner_weight) / 3.0f;
    float left_mid_inner_weight = (left_outer_weight + (2.0f * left_inner_weight)) / 3.0f;
    float channel_weights[TRACK_SENSOR_CHANNEL_COUNT] =
    {
        -left_outer_weight,
        -left_mid_outer_weight,
        -left_mid_inner_weight,
        -left_inner_weight,
        left_inner_weight,
        left_mid_inner_weight,
        left_mid_outer_weight,
        left_outer_weight
    };

    for (uint8_t channel_index = 0U; channel_index < TRACK_SENSOR_CHANNEL_COUNT; channel_index++)
    {
        if (handle->sensors.channel[channel_index] != 0U)
        {
            accumulated_error += channel_weights[channel_index];
            active_weight += 1.0f;
        }
    }

    if (active_weight > 0.0f)
    {
        return accumulated_error / active_weight;
    }

    return handle->last_valid_error;
}

static void Track_EnterState(TrackHandle *handle, TrackState next_state)
{
    handle->state = next_state;
    handle->state_ticks = 0U;
    handle->abnormal_confirm_ticks = 0U;
    handle->normal_confirm_ticks = 0U;
    handle->outer_integral = 0.0f;
    handle->last_outer_error = handle->last_valid_error;
    if (next_state != TRACK_STATE_PASS)
    {
        handle->pass_servo_direction = 0;
    }
}

static int16_t Track_ComputeBaseSpeed(const TrackHandle *handle, int16_t nominal_speed)
{
    float speed_reduction = Track_AbsFloat(handle->position_error) * handle->params.error_brake_gain;
    int32_t base_speed = (int32_t)nominal_speed - (int32_t)Track_RoundFloatToS16(speed_reduction);

    return Track_ClampS16(base_speed,
                          handle->params.base_speed_floor,
                          handle->params.target_speed_limit);
}

static int16_t Track_ComputeOuterDiff(TrackHandle *handle, int16_t diff_limit)
{
    float error_delta = handle->position_error - handle->last_outer_error;
    float controller_output;

    if (Track_AbsFloat(handle->position_error) <= handle->params.outer_integral_band)
    {
        handle->outer_integral += handle->position_error;
    }
    else
    {
        handle->outer_integral *= 0.85f;
    }

    handle->outer_integral = Track_ClampFloat(handle->outer_integral,
                                              -handle->params.outer_integral_limit,
                                              handle->params.outer_integral_limit);

    controller_output = handle->params.outer_kp * handle->position_error +
                        handle->params.outer_ki * handle->outer_integral +
                        handle->params.outer_kd * error_delta;

    handle->last_outer_error = handle->position_error;

    return Track_ClampS16(Track_RoundFloatToS16(controller_output),
                          (int16_t)(-diff_limit),
                          diff_limit);
}

static void Track_ApplyTargets(TrackHandle *handle, int16_t base_speed, int16_t speed_diff)
{
    handle->base_speed = Track_ClampS16(base_speed, 0, handle->params.target_speed_limit);
    handle->speed_diff = speed_diff;
    handle->left_target_speed = Track_ClampS16((int32_t)handle->base_speed +
                                               speed_diff +
                                               handle->params.straight_trim,
                                               handle->params.target_speed_floor,
                                               handle->params.target_speed_limit);
    handle->right_target_speed = Track_ClampS16((int32_t)handle->base_speed -
                                                speed_diff -
                                                handle->params.straight_trim,
                                                handle->params.target_speed_floor,
                                                handle->params.target_speed_limit);
}

static void Track_ApplySignedTargets(TrackHandle *handle, int16_t left_target_speed, int16_t right_target_speed)
{
    int16_t clamped_left = Track_ClampS16(left_target_speed,
                                          (int16_t)(-handle->params.target_speed_limit),
                                          handle->params.target_speed_limit);
    int16_t clamped_right = Track_ClampS16(right_target_speed,
                                           (int16_t)(-handle->params.target_speed_limit),
                                           handle->params.target_speed_limit);

    handle->left_target_speed = clamped_left;
    handle->right_target_speed = clamped_right;
    handle->base_speed = Track_ClampS16((Track_AbsS16(clamped_left) + Track_AbsS16(clamped_right)) / 2,
                                        0,
                                        handle->params.target_speed_limit);
    handle->speed_diff = Track_ClampS16(((int32_t)clamped_left - (int32_t)clamped_right) / 2,
                                        (int16_t)(-handle->params.target_speed_limit),
                                        handle->params.target_speed_limit);
}

static int8_t Track_GetHardTurnDirection(const TrackHandle *handle)
{
    if (Track_AbsFloat(handle->position_error) < handle->params.hard_turn_error_threshold)
    {
        return 0;
    }

    if (handle->position_error < 0.0f)
    {
        return -1;
    }

    if (handle->position_error > 0.0f)
    {
        return 1;
    }

    return handle->last_turn_direction;
}

static void Track_RunHardTurnControl(TrackHandle *handle, int8_t turn_direction)
{
    if (turn_direction < 0)
    {
        Track_ApplySignedTargets(handle,
                                 handle->params.hard_turn_inner_speed,
                                 handle->params.hard_turn_outer_speed);
    }
    else if (turn_direction > 0)
    {
        Track_ApplySignedTargets(handle,
                                 handle->params.hard_turn_outer_speed,
                                 handle->params.hard_turn_inner_speed);
    }
    else
    {
        Track_ApplyTargets(handle, handle->params.base_speed_pass, 0);
    }
}

static void Track_RunNormalControl(TrackHandle *handle)
{
    int8_t hard_turn_direction = Track_GetHardTurnDirection(handle);
    int16_t base_speed;
    int16_t speed_diff;

    if (hard_turn_direction != 0)
    {
        Track_RunHardTurnControl(handle, hard_turn_direction);
        return;
    }

    base_speed = Track_ComputeBaseSpeed(handle, handle->params.base_speed_normal);
    speed_diff = Track_ComputeOuterDiff(handle, handle->params.normal_diff_limit);

    Track_ApplyTargets(handle, base_speed, speed_diff);
}

static void Track_RunRecoverControl(TrackHandle *handle)
{
    int8_t hard_turn_direction = Track_GetHardTurnDirection(handle);
    int16_t base_speed;
    int16_t speed_diff;

    if (hard_turn_direction != 0)
    {
        Track_RunHardTurnControl(handle, hard_turn_direction);
        return;
    }

    base_speed = Track_ComputeBaseSpeed(handle, handle->params.base_speed_recover);
    speed_diff = Track_ComputeOuterDiff(handle, handle->params.recover_diff_limit);

    Track_ApplyTargets(handle, base_speed, speed_diff);
}

static void Track_RunPassControl(TrackHandle *handle)
{
    if ((Track_AbsFloat(handle->last_valid_error) >= handle->params.hard_turn_error_threshold) &&
        (handle->last_turn_direction != 0))
    {
        handle->position_error = handle->last_valid_error;
        handle->last_outer_error = handle->position_error;
        Track_RunHardTurnControl(handle, handle->last_turn_direction);
        return;
    }

    int16_t forced_diff = 0;

    if (handle->last_turn_direction > 0)
    {
        forced_diff = handle->params.recover_diff_limit;
    }
    else if (handle->last_turn_direction < 0)
    {
        forced_diff = (int16_t)(-handle->params.recover_diff_limit);
    }

    handle->position_error = handle->last_valid_error;
    handle->last_outer_error = handle->position_error;
    Track_ApplyTargets(handle, handle->params.base_speed_pass, forced_diff);
}

void Track_Init(TrackHandle *handle)
{
    handle->params.outer_kp = TRACK_DEFAULT_OUTER_KP;
    handle->params.outer_ki = TRACK_DEFAULT_OUTER_KI;
    handle->params.outer_kd = TRACK_DEFAULT_OUTER_KD;
    handle->params.outer_integral_limit = TRACK_DEFAULT_OUTER_INTEGRAL_LIMIT;
    handle->params.outer_integral_band = TRACK_DEFAULT_OUTER_INTEGRAL_BAND;
    handle->params.error_outer = TRACK_DEFAULT_ERROR_OUTER;
    handle->params.error_inner = TRACK_DEFAULT_ERROR_INNER;
    handle->params.hard_turn_error_threshold = TRACK_DEFAULT_HARD_TURN_ERROR_THRESHOLD;
    handle->params.error_brake_gain = TRACK_DEFAULT_ERROR_BRAKE_GAIN;
    handle->params.base_speed_normal = TRACK_DEFAULT_BASE_SPEED_NORMAL;
    handle->params.base_speed_recover = TRACK_DEFAULT_BASE_SPEED_RECOVER;
    handle->params.base_speed_pass = TRACK_DEFAULT_BASE_SPEED_PASS;
    handle->params.base_speed_floor = TRACK_DEFAULT_BASE_SPEED_FLOOR;
    handle->params.target_speed_floor = TRACK_DEFAULT_TARGET_SPEED_FLOOR;
    handle->params.target_speed_limit = TRACK_DEFAULT_TARGET_SPEED_LIMIT;
    handle->params.normal_diff_limit = TRACK_DEFAULT_NORMAL_DIFF_LIMIT;
    handle->params.recover_diff_limit = TRACK_DEFAULT_RECOVER_DIFF_LIMIT;
    handle->params.hard_turn_outer_speed = TRACK_DEFAULT_HARD_TURN_OUTER_SPEED;
    handle->params.hard_turn_inner_speed = TRACK_DEFAULT_HARD_TURN_INNER_SPEED;
    handle->params.straight_trim = TRACK_DEFAULT_STRAIGHT_TRIM;
    handle->params.servo_slew_step_us = TRACK_SERVO_SLEW_STEP_US;
    handle->params.pass_enter_confirm_ticks = TRACK_DEFAULT_PASS_ENTER_CONFIRM_TICKS;
    handle->params.pass_exit_confirm_ticks = TRACK_DEFAULT_PASS_EXIT_CONFIRM_TICKS;
    handle->params.recover_hold_ticks = TRACK_DEFAULT_RECOVER_HOLD_TICKS;

    Track_ReadSensors(&handle->sensors);
    handle->pattern = Track_ClassifyPattern(&handle->sensors);
    handle->position_error = 0.0f;
    handle->last_valid_error = 0.0f;
    handle->last_outer_error = 0.0f;
    handle->outer_integral = 0.0f;
    handle->base_speed = 0;
    handle->speed_diff = 0;
    handle->left_target_speed = 0;
    handle->right_target_speed = 0;
    handle->last_turn_direction = 0;
    handle->pass_servo_direction = 0;
    handle->servo_target_pulse_us = TRACK_SERVO_CENTER_PULSE_US;
    handle->servo_pulse_us = TRACK_SERVO_CENTER_PULSE_US;
    handle->state_ticks = 0U;
    handle->abnormal_confirm_ticks = 0U;
    handle->normal_confirm_ticks = 0U;

    if (handle->pattern == TRACK_PATTERN_LINE)
    {
        handle->state = TRACK_STATE_NORMAL;
        handle->position_error = Track_ComputeLineError(handle);
        handle->last_valid_error = handle->position_error;
        handle->last_outer_error = handle->position_error;
        handle->last_turn_direction = 0;
        Track_UpdateServoPulse(handle);
    }
    else
    {
        handle->state = TRACK_STATE_PASS;
        Track_UpdateServoPulse(handle);
    }
}

void Track_Update(TrackHandle *handle)
{
    Track_ReadSensors(&handle->sensors);
    handle->pattern = Track_ClassifyPattern(&handle->sensors);
    handle->state_ticks++;

    switch (handle->state)
    {
        case TRACK_STATE_NORMAL:
        {
            if (handle->pattern != TRACK_PATTERN_LINE)
            {
                if (handle->abnormal_confirm_ticks < 255U)
                {
                    handle->abnormal_confirm_ticks++;
                }

                if (handle->abnormal_confirm_ticks >= handle->params.pass_enter_confirm_ticks)
                {
                    Track_CapturePassServoDirection(handle);
                    Track_EnterState(handle, TRACK_STATE_PASS);
                    Track_RunPassControl(handle);
                    Track_UpdatePassServoPulse(handle);
                }
                else
                {
                    handle->position_error = handle->last_valid_error;
                    Track_RunNormalControl(handle);
                    Track_UpdateServoPulse(handle);
                }
                break;
            }

            handle->abnormal_confirm_ticks = 0U;
            handle->position_error = Track_ComputeLineError(handle);
            handle->last_valid_error = handle->position_error;
            Track_UpdateServoPulse(handle);
            if (Track_HasActiveLeftEdge(&handle->sensors) != 0U)
            {
                handle->last_turn_direction = -1;
            }
            else if (Track_HasActiveRightEdge(&handle->sensors) != 0U)
            {
                handle->last_turn_direction = 1;
            }
            else if (handle->position_error < 0.0f)
            {
                handle->last_turn_direction = -1;
            }
            else if (handle->position_error > 0.0f)
            {
                handle->last_turn_direction = 1;
            }
            Track_RunNormalControl(handle);
            break;
        }

        case TRACK_STATE_PASS:
        {
            if (handle->pattern == TRACK_PATTERN_LINE)
            {
                handle->position_error = Track_ComputeLineError(handle);
                handle->last_valid_error = handle->position_error;
                handle->abnormal_confirm_ticks = 0U;

                if (handle->normal_confirm_ticks < 255U)
                {
                    handle->normal_confirm_ticks++;
                }

                if (handle->normal_confirm_ticks >= handle->params.pass_exit_confirm_ticks)
                {
                    Track_EnterState(handle, TRACK_STATE_RECOVER);
                    Track_RunRecoverControl(handle);
                    Track_UpdateServoPulse(handle);
                }
                else
                {
                    Track_RunRecoverControl(handle);
                    Track_UpdatePassServoPulse(handle);
                }
            }
            else
            {
                handle->normal_confirm_ticks = 0U;
                Track_RunPassControl(handle);
                Track_UpdatePassServoPulse(handle);
            }
            break;
        }

        case TRACK_STATE_RECOVER:
        {
            if (handle->pattern != TRACK_PATTERN_LINE)
            {
                Track_CapturePassServoDirection(handle);
                Track_EnterState(handle, TRACK_STATE_PASS);
                Track_RunPassControl(handle);
                Track_UpdatePassServoPulse(handle);
                break;
            }

            handle->position_error = Track_ComputeLineError(handle);
            handle->last_valid_error = handle->position_error;
            Track_UpdateServoPulse(handle);
            if (Track_HasActiveLeftEdge(&handle->sensors) != 0U)
            {
                handle->last_turn_direction = -1;
            }
            else if (Track_HasActiveRightEdge(&handle->sensors) != 0U)
            {
                handle->last_turn_direction = 1;
            }
            else if (handle->position_error < 0.0f)
            {
                handle->last_turn_direction = -1;
            }
            else if (handle->position_error > 0.0f)
            {
                handle->last_turn_direction = 1;
            }
            Track_RunRecoverControl(handle);

            if (handle->state_ticks >= handle->params.recover_hold_ticks)
            {
                Track_EnterState(handle, TRACK_STATE_NORMAL);
                Track_RunNormalControl(handle);
                Track_UpdateServoPulse(handle);
            }
            break;
        }

        default:
        {
            Track_CapturePassServoDirection(handle);
            Track_EnterState(handle, TRACK_STATE_PASS);
            Track_RunPassControl(handle);
            Track_UpdatePassServoPulse(handle);
            break;
        }
    }
}
