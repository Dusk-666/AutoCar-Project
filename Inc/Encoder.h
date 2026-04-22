#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"

/* ==================== 编码器方向修正参数 ==================== */
#define LEFT_ENCODER_DIRECTION          1
#define RIGHT_ENCODER_DIRECTION         1

typedef struct
{
    TIM_HandleTypeDef *timer_handle;
    int16_t last_counter;
    int16_t delta_count;
    int16_t speed_feedback;
} EncoderChannel;

typedef struct
{
    EncoderChannel left_channel;
    EncoderChannel right_channel;
} EncoderHandle;

/**
 * @brief 初始化编码器模块并绑定左右编码器定时器
 * @param handle 编码器模块句柄
 * @param left_timer 左编码器定时器
 * @param right_timer 右编码器定时器
 */
void Encoder_Init(EncoderHandle *handle, TIM_HandleTypeDef *left_timer, TIM_HandleTypeDef *right_timer);

/**
 * @brief 启动左右编码器接口
 * @param handle 编码器模块句柄
 */
void Encoder_Start(EncoderHandle *handle);

/**
 * @brief 周期更新左右编码器增量与速度反馈
 * @param handle 编码器模块句柄
 */
void Encoder_Update(EncoderHandle *handle);

#endif
