#ifndef TRACK_H
#define TRACK_H

#include "main.h"

/* ==================== 控制周期 ==================== */
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
#define TRACK_SENSOR_ACTIVE_LEVEL                   GPIO_PIN_RESET
#define TRACK_SENSOR_ADDRESS_SETTLE_NOP_COUNT       32U

/* ==================== 舵机参数 ==================== */
#define TRACK_SERVO_CENTER_PULSE_US                 1500U
#define TRACK_SERVO_TURN_OFFSET_US                  300U
#define TRACK_SERVO_SLEW_STEP_US                    20U

/* ==================== 外环默认参数 ==================== */
#define TRACK_DEFAULT_OUTER_KP                      16.0f
#define TRACK_DEFAULT_OUTER_KI                      0.06f
#define TRACK_DEFAULT_OUTER_KD                      4.0f
#define TRACK_DEFAULT_OUTER_INTEGRAL_LIMIT          8.0f
#define TRACK_DEFAULT_OUTER_INTEGRAL_BAND           1.0f

/* ==================== 位置误差映射 ==================== */
#define TRACK_DEFAULT_ERROR_OUTER                   4.0f
#define TRACK_DEFAULT_ERROR_MID_OUTER               3.2f
#define TRACK_DEFAULT_ERROR_MID_INNER               2.4f
#define TRACK_DEFAULT_ERROR_INNER                   1.5f
#define TRACK_DEFAULT_HARD_TURN_ERROR_THRESHOLD     2.5f
#define TRACK_DEFAULT_SERVO_ERROR_GAIN              75.0f

/* ==================== 速度规划参数 ==================== */
#define TRACK_DEFAULT_BASE_SPEED_NORMAL             25
#define TRACK_DEFAULT_BASE_SPEED_RECOVER            18
#define TRACK_DEFAULT_BASE_SPEED_PASS               22
#define TRACK_DEFAULT_BASE_SPEED_FLOOR              5
#define TRACK_DEFAULT_TARGET_SPEED_FLOOR            5
#define TRACK_DEFAULT_TARGET_SPEED_LIMIT            40
#define TRACK_DEFAULT_NORMAL_DIFF_LIMIT             55
#define TRACK_DEFAULT_RECOVER_DIFF_LIMIT            40
#define TRACK_DEFAULT_HARD_TURN_OUTER_SPEED         20
#define TRACK_DEFAULT_HARD_TURN_INNER_SPEED         5
#define TRACK_DEFAULT_ERROR_BRAKE_GAIN              4.5f
#define TRACK_DEFAULT_STRAIGHT_TRIM                 0

/* ==================== 状态机参数 ==================== */
#define TRACK_DEFAULT_PASS_ENTER_CONFIRM_TICKS      1U
#define TRACK_DEFAULT_PASS_EXIT_CONFIRM_TICKS       1U
#define TRACK_DEFAULT_RECOVER_HOLD_TICKS            8U

typedef enum
{
    TRACK_STATE_NORMAL = 0,
    TRACK_STATE_PASS,
    TRACK_STATE_RECOVER
} TrackState;

typedef enum
{
    TRACK_PATTERN_LINE = 0,
    TRACK_PATTERN_ALL_WHITE,
    TRACK_PATTERN_ALL_BLACK
} TrackPatternClass;

typedef enum
{
    TRACK_SENSOR_BIT_CH1 = (1U << 7),
    TRACK_SENSOR_BIT_CH2 = (1U << 6),
    TRACK_SENSOR_BIT_CH3 = (1U << 5),
    TRACK_SENSOR_BIT_CH4 = (1U << 4),
    TRACK_SENSOR_BIT_CH5 = (1U << 3),
    TRACK_SENSOR_BIT_CH6 = (1U << 2),
    TRACK_SENSOR_BIT_CH7 = (1U << 1),
    TRACK_SENSOR_BIT_CH8 = (1U << 0)
} TrackSensorBit;

typedef struct
{
    uint8_t channel[TRACK_SENSOR_CHANNEL_COUNT];
    uint8_t mask;
    uint8_t active_count;
} TrackSensorSample;

typedef struct
{
    float outer_kp;
    float outer_ki;
    float outer_kd;
    float outer_integral_limit;
    float outer_integral_band;
    float error_outer;
    float error_mid_outer;
    float error_mid_inner;
    float error_inner;
    float hard_turn_error_threshold;
    float servo_error_gain;
    float error_brake_gain;
    int16_t base_speed_normal;
    int16_t base_speed_recover;
    int16_t base_speed_pass;
    int16_t base_speed_floor;
    int16_t target_speed_floor;
    int16_t target_speed_limit;
    int16_t normal_diff_limit;
    int16_t recover_diff_limit;
    int16_t hard_turn_outer_speed;
    int16_t hard_turn_inner_speed;
    int16_t straight_trim;
    uint16_t servo_slew_step_us;
    uint8_t pass_enter_confirm_ticks;
    uint8_t pass_exit_confirm_ticks;
    uint8_t recover_hold_ticks;
} TrackParams;

typedef struct
{
    TrackState state;
    TrackPatternClass pattern;
    TrackSensorSample sensors;
    TrackParams params;
    float position_error;
    float last_valid_error;
    float last_outer_error;
    float outer_integral;
    int16_t base_speed;
    int16_t speed_diff;
    int16_t left_target_speed;
    int16_t right_target_speed;
    int8_t last_turn_direction;
    int8_t pass_servo_direction;
    uint16_t servo_target_pulse_us;
    uint16_t servo_pulse_us;
    uint16_t state_ticks;
    uint8_t abnormal_confirm_ticks;
    uint8_t normal_confirm_ticks;
} TrackHandle;

void Track_Init(TrackHandle *handle);
void Track_Update(TrackHandle *handle);

#endif
