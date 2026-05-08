#ifndef TRACK_H
#define TRACK_H

#include "main.h"

#define TRACK_CONTROL_PERIOD_MS                     10U

#define TRACK_SENSOR_CHANNEL_COUNT                  8U
#define TRACK_SENSOR_OUT_PORT                       GPIOA
#define TRACK_SENSOR_OUT_PIN                        GPIO_PIN_9
#define TRACK_SENSOR_AD0_PORT                       GPIOA
#define TRACK_SENSOR_AD0_PIN                        GPIO_PIN_10
#define TRACK_SENSOR_AD1_PORT                       GPIOA
#define TRACK_SENSOR_AD1_PIN                        GPIO_PIN_11
#define TRACK_SENSOR_AD2_PORT                       GPIOA
#define TRACK_SENSOR_AD2_PIN                        GPIO_PIN_12
#define TRACK_SENSOR_ACTIVE_LEVEL                   GPIO_PIN_SET
#define TRACK_SENSOR_ADDRESS_SETTLE_NOP_COUNT       32U

#define TRACK_SERVO_CENTER_PULSE_US                 1500U
#define TRACK_SERVO_MAX_OFFSET_US                   450U
#define TRACK_SERVO_NORMAL_MAX_OFFSET_US            400U
#define TRACK_SERVO_SLEW_STEP_US                    10U
#define TRACK_SERVO_FAST_RETURN_STEP_US             30U

#define TRACK_DEFAULT_ERROR_FILTER_ALPHA            0.45f
#define TRACK_DEFAULT_SERVO_KP                      45.0f
#define TRACK_DEFAULT_SERVO_KD                      28.0f
#define TRACK_DEFAULT_SERVO_EDGE_GAIN_SCALE         3.0f
#define TRACK_DEFAULT_WHEEL_KP                      7.5f
#define TRACK_DEFAULT_WHEEL_KD                      5.0f
#define TRACK_DEFAULT_SERVO_DIFF_GAIN               0.03f
#define TRACK_DEFAULT_ERROR_BRAKE_GAIN              5.0f

#define TRACK_DEFAULT_BASE_SPEED                    30
#define TRACK_DEFAULT_LOST_BASE_SPEED               26
#define TRACK_DEFAULT_TARGET_SPEED_LIMIT            35
#define TRACK_DEFAULT_TARGET_SPEED_MIN              10
#define TRACK_DEFAULT_DIFF_LIMIT                    58

#define TRACK_DEFAULT_LOST_FORWARD_SPEED            20
#define TRACK_DEFAULT_LOST_REVERSE_SPEED            -6
#define TRACK_DEFAULT_LOST_ENTER_CONFIRM_TICKS      3U
#define TRACK_DEFAULT_CORNER_INNER_SPEED            -10
#define TRACK_DEFAULT_CORNER_OUTER_SPEED            30
#define TRACK_DEFAULT_CORNER_HOLD_TICKS             100U

typedef enum
{
    TRACK_STATE_NORMAL = 0,
    TRACK_STATE_LOST,
    TRACK_STATE_CORNER_LEFT,
    TRACK_STATE_CORNER_RIGHT
} TrackState;

#define TRACK_STATE_PASS TRACK_STATE_LOST
#define TRACK_STATE_RECOVER TRACK_STATE_NORMAL

typedef enum
{
    TRACK_PATTERN_LINE = 0,
    TRACK_PATTERN_ALL_WHITE,
    TRACK_PATTERN_ALL_BLACK
} TrackPatternClass;

typedef struct
{
    uint8_t channel[TRACK_SENSOR_CHANNEL_COUNT];
    uint8_t mask;
    uint8_t active_count;
} TrackSensorSample;

typedef struct
{
    float error_filter_alpha;
    float servo_kp;
    float servo_kd;
    float servo_edge_gain_scale;
    float wheel_kp;
    float wheel_kd;
    float servo_diff_gain;
    float error_brake_gain;
    int16_t base_speed;
    int16_t lost_base_speed;
    int16_t target_speed_min;
    int16_t target_speed_limit;
    int16_t diff_limit;
    int16_t lost_forward_speed;
    int16_t lost_reverse_speed;
    int16_t corner_inner_speed;
    int16_t corner_outer_speed;
    uint16_t servo_center_pulse_us;
    uint16_t servo_max_offset_us;
    uint16_t servo_slew_step_us;
    uint8_t lost_enter_confirm_ticks;
    uint16_t corner_hold_ticks;
} TrackParams;

typedef struct
{
    TrackState state;
    TrackPatternClass pattern;
    TrackSensorSample sensors;
    TrackParams params;
    float position_error;
    float filtered_error;
    float last_valid_error;
    float last_error_for_diff;
    float servo_last_error;
    float outer_integral;
    float wheel_diff_command;
    float servo_offset_command;
    int16_t base_speed;
    int16_t speed_diff;
    int16_t left_target_speed;
    int16_t right_target_speed;
    int8_t last_turn_direction;
    uint16_t servo_target_pulse_us;
    uint16_t servo_pulse_us;
    uint16_t state_ticks;
    uint8_t abnormal_confirm_ticks;
    uint8_t normal_confirm_ticks;
} TrackHandle;

void Track_Init(TrackHandle *handle);
void Track_Update(TrackHandle *handle);

#endif
