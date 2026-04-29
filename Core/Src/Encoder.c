#include "Encoder.h"

static float Encoder_ClampFloat(float input_value, float lower_limit, float upper_limit)
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

static int16_t Encoder_RoundFloatToS16(float input_value)
{
    if (input_value >= 0.0f)
    {
        return (int16_t)(input_value + 0.5f);
    }

    return (int16_t)(input_value - 0.5f);
}

static int16_t Encoder_ReadCounter(const EncoderChannel *channel)
{
    return (int16_t)__HAL_TIM_GET_COUNTER(channel->timer_handle);
}

static void Encoder_UpdateChannel(EncoderChannel *channel, float filter_alpha)
{
    int16_t current_counter = Encoder_ReadCounter(channel);
    int16_t counter_delta = (int16_t)(current_counter - channel->last_counter);

    channel->last_counter = current_counter;
    channel->delta_count = (int16_t)(counter_delta * channel->forward_sign);
    channel->filtered_speed += filter_alpha * ((float)channel->delta_count - channel->filtered_speed);
    channel->speed_feedback = Encoder_RoundFloatToS16(channel->filtered_speed);
}

void Encoder_Init(EncoderHandle *handle, TIM_HandleTypeDef *left_timer, TIM_HandleTypeDef *right_timer)
{
    handle->params.filter_alpha = ENCODER_DEFAULT_FILTER_ALPHA;

    handle->left_channel.timer_handle = left_timer;
    handle->left_channel.forward_sign = ENCODER_LEFT_FORWARD_SIGN_DEFAULT;
    handle->left_channel.last_counter = 0;
    handle->left_channel.delta_count = 0;
    handle->left_channel.speed_feedback = 0;
    handle->left_channel.filtered_speed = 0.0f;

    handle->right_channel.timer_handle = right_timer;
    handle->right_channel.forward_sign = ENCODER_RIGHT_FORWARD_SIGN_DEFAULT;
    handle->right_channel.last_counter = 0;
    handle->right_channel.delta_count = 0;
    handle->right_channel.speed_feedback = 0;
    handle->right_channel.filtered_speed = 0.0f;
}

void Encoder_Start(EncoderHandle *handle)
{
    HAL_TIM_Encoder_Start(handle->left_channel.timer_handle, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(handle->right_channel.timer_handle, TIM_CHANNEL_ALL);

    __HAL_TIM_SET_COUNTER(handle->left_channel.timer_handle, 0);
    __HAL_TIM_SET_COUNTER(handle->right_channel.timer_handle, 0);

    handle->left_channel.last_counter = Encoder_ReadCounter(&handle->left_channel);
    handle->right_channel.last_counter = Encoder_ReadCounter(&handle->right_channel);
    handle->left_channel.delta_count = 0;
    handle->right_channel.delta_count = 0;
    handle->left_channel.speed_feedback = 0;
    handle->right_channel.speed_feedback = 0;
    handle->left_channel.filtered_speed = 0.0f;
    handle->right_channel.filtered_speed = 0.0f;
}

void Encoder_Update(EncoderHandle *handle)
{
    float filter_alpha = Encoder_ClampFloat(handle->params.filter_alpha, 0.0f, 1.0f);

    Encoder_UpdateChannel(&handle->left_channel, filter_alpha);
    Encoder_UpdateChannel(&handle->right_channel, filter_alpha);
}
