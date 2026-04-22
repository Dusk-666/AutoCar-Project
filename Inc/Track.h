#ifndef TRACK_H
#define TRACK_H

#include "main.h"

/* ==================== 循迹外环参数 ==================== */
#define TRACK_KP                        90.0f
#define TRACK_KD                        38.0f
#define APPROACH_KP                     125.0f
#define APPROACH_KD                     48.0f

/* ==================== 速度内环参数 ==================== */
#define SPEED_KP                        0.5f
#define SPEED_KD                        0.1f
#define ENCODER_PPR                     13
#define WHEEL_DIAMETER                  0.065f
#define GEAR_RATIO                      48.0f
#define CONTROL_PERIOD_MS               5

/* ==================== 速度目标参数 ==================== */
#define BASE_SPEED                      400
#define APPROACH_BASE_SPEED_RATIO       82
#define FULL_BLACK_SPEED_RATIO          68
#define LOST_LINE_SPEED_RATIO           72
#define TURN_INNER_RATIO                0.40f
#define TURN_OUTER_RATIO                1.30f
#define TRACK_TARGET_LIMIT              600
#define SPEED_TARGET_LIMIT              800

/* ==================== 状态机判定参数 ==================== */
#define OUTER_TRIGGER_TURN_COUNT        3U
#define OUTER_TRIGGER_DECAY_STEP        1U
#define APPROACH_HOLD_TICKS             8U
#define TURN_TIMEOUT_TICKS              55U
#define TURN_EXIT_CONFIRM_TICKS         3U
#define TURN_EXIT_ERROR_THRESHOLD       1.4f
#define LOST_LINE_HOLD_ERROR            2.8f
#define FULL_BLACK_ERROR_BAND           0.0f

typedef enum
{
    TRACKING = 0,
    APPROACH,
    TURN_LEFT,
    TURN_RIGHT
} TrackState;

typedef struct
{
    uint8_t left_outer;
    uint8_t left_inner;
    uint8_t right_inner;
    uint8_t right_outer;
    uint8_t active_count;
    uint8_t all_black;
    uint8_t all_white;
} TrackSensorState;

typedef struct
{
    TrackState state;
    TrackSensorState sensors;
    float current_error;
    float last_error;
    float last_valid_error;
    float pd_turn_output;
    int16_t base_speed;
    int16_t left_target_speed;
    int16_t right_target_speed;
    int16_t left_actual_speed;
    int16_t right_actual_speed;
    int32_t left_encoder_count;
    int32_t right_encoder_count;
    int32_t last_left_encoder_count;
    int32_t last_right_encoder_count;
    float left_speed_error;
    float right_speed_error;
    float last_left_speed_error;
    float last_right_speed_error;
    uint8_t left_outer_counter;
    uint8_t right_outer_counter;
    uint8_t turn_exit_counter;
    uint16_t state_tick;
    int8_t pending_turn_direction;
} TrackHandle;

/**
 * @brief 初始化循迹状态机上下文
 * @param handle 循迹模块句柄
 */
void Track_Init(TrackHandle *handle);

/**
 * @brief 执行一次循迹外环更新并输出左右轮目标速度
 * @param handle 循迹模块句柄
 */
void Track_Update(TrackHandle *handle);

/**
 * @brief 更新编码器计数并计算实际速度
 * @param handle 循迹模块句柄
 */
void Track_UpdateEncoders(TrackHandle *handle);

/**
 * @brief 执行速度内环PD控制
 * @param handle 循迹模块句柄
 */
void Track_RunSpeedControl(TrackHandle *handle);

#endif
