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
    static const uint8_t poll_order[TRACK_SENSOR_CHANNEL_COUNT] = {0U, 1U, 4U, 5U, 2U, 3U, 6U, 7U};

    sample->mask = 0U;
    sample->active_count = 0U;

    for (uint8_t logical_index = 0U; logical_index < TRACK_SENSOR_CHANNEL_COUNT; logical_index++)
    {
        uint8_t channel_state;
        uint8_t hardware_index = poll_order[logical_index];

        Track_SelectSensorChannel(hardware_index);
        Track_WaitSensorAddressSettle();
        channel_state = Track_ReadSelectedSensor();
        sample->channel[logical_index] = channel_state;

        if (channel_state != 0U)
        {
            sample->mask |= (uint8_t)(1U << ((TRACK_SENSOR_CHANNEL_COUNT - 1U) - logical_index));
            sample->active_count++;
        }
    }
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

static uint8_t Track_IsLeftCornerPattern(const TrackSensorSample *sample)
{
    return (uint8_t)((sample->channel[0] != 0U) &&
                     (sample->channel[1] != 0U) &&
                     (sample->channel[2] != 0U) &&
                     (sample->channel[6] == 0U) &&
                     (sample->channel[7] == 0U));
}

static uint8_t Track_IsRightCornerPattern(const TrackSensorSample *sample)
{
    return (uint8_t)((sample->channel[0] == 0U) &&
                     (sample->channel[1] == 0U) &&
                     (sample->channel[5] != 0U) &&
                     (sample->channel[6] != 0U) &&
                     (sample->channel[7] != 0U));
}

static float Track_ComputeError(const TrackHandle *handle)
{
    static const float sensor_positions[TRACK_SENSOR_CHANNEL_COUNT] =
    {
        3.5f, 2.5f, 1.5f, 0.5f, -0.5f, -1.5f, -2.5f, -3.5f
    };
    float position_sum = 0.0f;
    float active_sum = 0.0f;

    for (uint8_t channel_index = 0U; channel_index < TRACK_SENSOR_CHANNEL_COUNT; channel_index++)
    {
        if (handle->sensors.channel[channel_index] != 0U)
        {
            position_sum += sensor_positions[channel_index];
            active_sum += 1.0f;
        }
    }

    if (active_sum > 0.0f)
    {
        return position_sum / active_sum;
    }

    return handle->last_valid_error;
}

static void Track_UpdateServo(TrackHandle *handle)
{
    float limited_error = Track_ClampFloat(handle->filtered_error, -4.0f, 4.0f);
    float error_delta = limited_error - handle->servo_last_error;
    int16_t normal_limit = TRACK_SERVO_NORMAL_MAX_OFFSET_US;
    uint16_t servo_step = handle->params.servo_slew_step_us;
    int16_t target_offset;
    uint8_t edge_active = (uint8_t)((handle->sensors.channel[0] != 0U) ||
                                    (handle->sensors.channel[TRACK_SENSOR_CHANNEL_COUNT - 1U] != 0U));

    if ((edge_active == 0U) &&
        (handle->servo_pulse_us < (handle->params.servo_center_pulse_us - (uint16_t)normal_limit)))
    {
        target_offset = (int16_t)(-normal_limit);
        servo_step = TRACK_SERVO_FAST_RETURN_STEP_US;
    }
    else if ((edge_active == 0U) &&
             (handle->servo_pulse_us > (handle->params.servo_center_pulse_us + (uint16_t)normal_limit)))
    {
        target_offset = normal_limit;
        servo_step = TRACK_SERVO_FAST_RETURN_STEP_US;
    }
    else
    {
        int16_t limit = (edge_active != 0U) ? (int16_t)handle->params.servo_max_offset_us : normal_limit;
        float edge_scale = (edge_active != 0U) ? handle->params.servo_edge_gain_scale : 1.0f;
        float pd_output = edge_scale * (handle->params.servo_kp * limited_error +
                                        handle->params.servo_kd * error_delta);

        target_offset = Track_ClampS16(Track_RoundFloatToS16(pd_output),
                                       (int16_t)(-limit),
                                       limit);
    }

    handle->servo_last_error = limited_error;
    handle->servo_offset_command = (float)target_offset;
    handle->servo_target_pulse_us = (uint16_t)(handle->params.servo_center_pulse_us + target_offset);
    handle->servo_pulse_us = Track_StepServoPulse(handle->servo_pulse_us,
                                                  handle->servo_target_pulse_us,
                                                  servo_step);
}

static void Track_UpdateDirection(TrackHandle *handle)
{
    if (handle->filtered_error > 0.15f)
    {
        handle->last_turn_direction = 1;
    }
    else if (handle->filtered_error < -0.15f)
    {
        handle->last_turn_direction = -1;
    }
}

static void Track_RunNormal(TrackHandle *handle)
{
    float error_delta = handle->filtered_error - handle->last_error_for_diff;
    float servo_offset = (float)((int32_t)handle->servo_pulse_us - (int32_t)handle->params.servo_center_pulse_us);
    float wheel_diff = handle->params.wheel_kp * handle->filtered_error +
                       handle->params.wheel_kd * error_delta -
                       handle->params.servo_diff_gain * servo_offset;
    uint8_t edge_active = (uint8_t)((handle->sensors.channel[0] != 0U) ||
                                    (handle->sensors.channel[TRACK_SENSOR_CHANNEL_COUNT - 1U] != 0U));
    int16_t speed_diff;
    int16_t left_target;
    int16_t right_target;

    handle->last_error_for_diff = handle->filtered_error;
    handle->wheel_diff_command = wheel_diff;
    if (edge_active != 0U)
    {
        handle->base_speed = handle->params.base_speed;
    }
    else
    {
        handle->base_speed = Track_ClampS16((int32_t)handle->params.base_speed -
                                            (int32_t)Track_RoundFloatToS16(Track_AbsFloat(handle->filtered_error) *
                                                                           handle->params.error_brake_gain),
                                            0,
                                            handle->params.target_speed_limit);
    }
    speed_diff = Track_ClampS16(Track_RoundFloatToS16(wheel_diff),
                                (int16_t)(-handle->params.diff_limit),
                                handle->params.diff_limit);
    left_target = Track_ClampS16((int32_t)handle->base_speed - speed_diff,
                                 handle->params.target_speed_min,
                                 handle->params.target_speed_limit);
    right_target = Track_ClampS16((int32_t)handle->base_speed + speed_diff,
                                  handle->params.target_speed_min,
                                  handle->params.target_speed_limit);

    handle->speed_diff = speed_diff;
    handle->left_target_speed = left_target;
    handle->right_target_speed = right_target;
}

static void Track_RunLost(TrackHandle *handle)
{
    int8_t turn_direction = handle->last_turn_direction;

    if (turn_direction == 0)
    {
        turn_direction = (handle->last_valid_error >= 0.0f) ? 1 : -1;
    }

    if (turn_direction < 0)
    {
        handle->filtered_error = Track_ClampFloat(handle->filtered_error - 0.15f, -4.0f, 4.0f);
        handle->left_target_speed = handle->params.lost_forward_speed;
        handle->right_target_speed = handle->params.lost_reverse_speed;
    }
    else
    {
        handle->filtered_error = Track_ClampFloat(handle->filtered_error + 0.15f, -4.0f, 4.0f);
        handle->left_target_speed = handle->params.lost_reverse_speed;
        handle->right_target_speed = handle->params.lost_forward_speed;
    }

    handle->position_error = handle->last_valid_error;
    handle->base_speed = handle->params.lost_base_speed;
    handle->speed_diff = (int16_t)((handle->left_target_speed - handle->right_target_speed) / 2);
    handle->wheel_diff_command = (float)handle->speed_diff;
    handle->last_error_for_diff = handle->filtered_error;
    Track_UpdateServo(handle);
}

static void Track_RunAllBlack(TrackHandle *handle)
{
    handle->position_error = 0.0f;
    handle->filtered_error = 0.0f;
    handle->last_error_for_diff = 0.0f;
    handle->wheel_diff_command = 0.0f;
    handle->base_speed = handle->params.base_speed;
    handle->speed_diff = 0;
    handle->left_target_speed = handle->params.base_speed;
    handle->right_target_speed = handle->params.base_speed;
    handle->servo_offset_command = 0.0f;
    handle->servo_target_pulse_us = handle->params.servo_center_pulse_us;
    handle->servo_pulse_us = handle->servo_target_pulse_us;
}

static void Track_RunCorner(TrackHandle *handle, int8_t corner_direction)
{
    int16_t target_offset = (corner_direction < 0) ?
                            (int16_t)handle->params.servo_max_offset_us :
                            (int16_t)(-handle->params.servo_max_offset_us);

    handle->servo_offset_command = (float)target_offset;
    handle->servo_target_pulse_us = (uint16_t)(handle->params.servo_center_pulse_us + target_offset);
    handle->servo_pulse_us = handle->servo_target_pulse_us;
    handle->base_speed = handle->params.corner_outer_speed;
    handle->wheel_diff_command = (float)(handle->params.corner_outer_speed - handle->params.corner_inner_speed);

    if (corner_direction < 0)
    {
        handle->left_target_speed = handle->params.corner_inner_speed;
        handle->right_target_speed = handle->params.corner_outer_speed;
    }
    else
    {
        handle->left_target_speed = handle->params.corner_outer_speed;
        handle->right_target_speed = handle->params.corner_inner_speed;
    }

    handle->speed_diff = (int16_t)((handle->left_target_speed - handle->right_target_speed) / 2);
}

void Track_Init(TrackHandle *handle)
{
    Track_ReadSensors(&handle->sensors);

    handle->params.error_filter_alpha = TRACK_DEFAULT_ERROR_FILTER_ALPHA;
    handle->params.servo_kp = TRACK_DEFAULT_SERVO_KP;
    handle->params.servo_kd = TRACK_DEFAULT_SERVO_KD;
    handle->params.servo_edge_gain_scale = TRACK_DEFAULT_SERVO_EDGE_GAIN_SCALE;
    handle->params.wheel_kp = TRACK_DEFAULT_WHEEL_KP;
    handle->params.wheel_kd = TRACK_DEFAULT_WHEEL_KD;
    handle->params.servo_diff_gain = TRACK_DEFAULT_SERVO_DIFF_GAIN;
    handle->params.error_brake_gain = TRACK_DEFAULT_ERROR_BRAKE_GAIN;
    handle->params.base_speed = TRACK_DEFAULT_BASE_SPEED;
    handle->params.lost_base_speed = TRACK_DEFAULT_LOST_BASE_SPEED;
    handle->params.target_speed_min = TRACK_DEFAULT_TARGET_SPEED_MIN;
    handle->params.target_speed_limit = TRACK_DEFAULT_TARGET_SPEED_LIMIT;
    handle->params.diff_limit = TRACK_DEFAULT_DIFF_LIMIT;
    handle->params.lost_forward_speed = TRACK_DEFAULT_LOST_FORWARD_SPEED;
    handle->params.lost_reverse_speed = TRACK_DEFAULT_LOST_REVERSE_SPEED;
    handle->params.corner_inner_speed = TRACK_DEFAULT_CORNER_INNER_SPEED;
    handle->params.corner_outer_speed = TRACK_DEFAULT_CORNER_OUTER_SPEED;
    handle->params.servo_center_pulse_us = TRACK_SERVO_CENTER_PULSE_US;
    handle->params.servo_max_offset_us = TRACK_SERVO_MAX_OFFSET_US;
    handle->params.servo_slew_step_us = TRACK_SERVO_SLEW_STEP_US;
    handle->params.lost_enter_confirm_ticks = TRACK_DEFAULT_LOST_ENTER_CONFIRM_TICKS;
    handle->params.corner_hold_ticks = TRACK_DEFAULT_CORNER_HOLD_TICKS;

    handle->pattern = Track_ClassifyPattern(&handle->sensors);
    handle->position_error = 0.0f;
    handle->filtered_error = 0.0f;
    handle->last_valid_error = 0.0f;
    handle->last_error_for_diff = 0.0f;
    handle->servo_last_error = 0.0f;
    handle->outer_integral = 0.0f;
    handle->wheel_diff_command = 0.0f;
    handle->servo_offset_command = 0.0f;
    handle->base_speed = 0;
    handle->speed_diff = 0;
    handle->left_target_speed = 0;
    handle->right_target_speed = 0;
    handle->last_turn_direction = 0;
    handle->servo_target_pulse_us = handle->params.servo_center_pulse_us;
    handle->servo_pulse_us = handle->params.servo_center_pulse_us;
    handle->state_ticks = 0U;
    handle->abnormal_confirm_ticks = 0U;
    handle->normal_confirm_ticks = 0U;

    if (handle->pattern == TRACK_PATTERN_ALL_WHITE)
    {
        handle->state = TRACK_STATE_LOST;
        Track_RunLost(handle);
    }
    else
    {
        handle->state = TRACK_STATE_NORMAL;
        handle->position_error = Track_ComputeError(handle);
        handle->filtered_error = handle->position_error;
        handle->last_valid_error = handle->position_error;
        Track_UpdateDirection(handle);
        Track_UpdateServo(handle);
        Track_RunNormal(handle);
    }
}

void Track_Update(TrackHandle *handle)
{
    Track_ReadSensors(&handle->sensors);
    handle->pattern = Track_ClassifyPattern(&handle->sensors);
    handle->state_ticks++;

    if (handle->state == TRACK_STATE_CORNER_LEFT)
    {
        if (handle->state_ticks < handle->params.corner_hold_ticks)
        {
            Track_RunCorner(handle, -1);
            return;
        }

        if (handle->pattern == TRACK_PATTERN_LINE)
        {
            handle->state = TRACK_STATE_NORMAL;
        }
        else
        {
            Track_RunCorner(handle, -1);
            return;
        }
    }
    else if (handle->state == TRACK_STATE_CORNER_RIGHT)
    {
        if (handle->state_ticks < handle->params.corner_hold_ticks)
        {
            Track_RunCorner(handle, 1);
            return;
        }

        if (handle->pattern == TRACK_PATTERN_LINE)
        {
            handle->state = TRACK_STATE_NORMAL;
        }
        else
        {
            Track_RunCorner(handle, 1);
            return;
        }
    }

    if (Track_IsLeftCornerPattern(&handle->sensors) != 0U)
    {
        handle->state = TRACK_STATE_CORNER_LEFT;
        handle->state_ticks = 0U;
        Track_RunCorner(handle, -1);
        return;
    }

    if (Track_IsRightCornerPattern(&handle->sensors) != 0U)
    {
        handle->state = TRACK_STATE_CORNER_RIGHT;
        handle->state_ticks = 0U;
        Track_RunCorner(handle, 1);
        return;
    }

    if (handle->pattern == TRACK_PATTERN_ALL_BLACK)
    {
        handle->state = TRACK_STATE_NORMAL;
        handle->abnormal_confirm_ticks = 0U;
        Track_RunAllBlack(handle);
        return;
    }

    if (handle->pattern == TRACK_PATTERN_ALL_WHITE)
    {
        return;
    }

    handle->state = TRACK_STATE_NORMAL;
    handle->position_error = Track_ComputeError(handle);
    handle->last_valid_error = handle->position_error;
    handle->filtered_error += handle->params.error_filter_alpha *
                              (handle->position_error - handle->filtered_error);
    Track_UpdateDirection(handle);
    Track_UpdateServo(handle);
    Track_RunNormal(handle);
}
