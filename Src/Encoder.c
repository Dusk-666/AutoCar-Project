#include "Encoder.h"

/**
 * @brief 读取单路编码器当前计数值
 * @param channel 编码器通道句柄
 * @return 当前计数器值
 */
static int16_t Encoder_ReadCounter(const EncoderChannel *channel)
{
    return (int16_t)__HAL_TIM_GET_COUNTER(channel->timer_handle);
}

/**
 * @brief 初始化编码器模块并绑定定时器资源
 * @param handle 编码器模块句柄
 * @param left_timer 左编码器定时器
 * @param right_timer 右编码器定时器
 */
void Encoder_Init(EncoderHandle *handle, TIM_HandleTypeDef *left_timer, TIM_HandleTypeDef *right_timer)
{
    handle->left_channel.timer_handle = left_timer;
    handle->left_channel.last_counter = 0;
    handle->left_channel.delta_count = 0;
    handle->left_channel.speed_feedback = 0;

    handle->right_channel.timer_handle = right_timer;
    handle->right_channel.last_counter = 0;
    handle->right_channel.delta_count = 0;
    handle->right_channel.speed_feedback = 0;
}

/**
 * @brief 启动左右编码器接口并清零初始计数
 * @param handle 编码器模块句柄
 */
void Encoder_Start(EncoderHandle *handle)
{
    HAL_TIM_Encoder_Start(handle->left_channel.timer_handle, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(handle->right_channel.timer_handle, TIM_CHANNEL_ALL);

    __HAL_TIM_SET_COUNTER(handle->left_channel.timer_handle, 0);
    __HAL_TIM_SET_COUNTER(handle->right_channel.timer_handle, 0);

    handle->left_channel.last_counter = 0;
    handle->right_channel.last_counter = 0;
}

/**
 * @brief 更新单路编码器的本周期增量和速度反馈
 * @param channel 编码器通道句柄
 * @param direction_sign 方向修正系数，通常取 1 或 -1
 */
static void Encoder_UpdateChannel(EncoderChannel *channel, int8_t direction_sign)
{
    int16_t current_counter = Encoder_ReadCounter(channel);
    int16_t counter_delta = (int16_t)(current_counter - channel->last_counter);

    channel->last_counter = current_counter;
    channel->delta_count = (int16_t)(counter_delta * direction_sign);
    channel->speed_feedback = channel->delta_count;
}

/**
 * @brief 周期更新左右编码器速度反馈
 * @param handle 编码器模块句柄
 */
void Encoder_Update(EncoderHandle *handle)
{
    Encoder_UpdateChannel(&handle->left_channel, LEFT_ENCODER_DIRECTION);
    Encoder_UpdateChannel(&handle->right_channel, RIGHT_ENCODER_DIRECTION);
}
