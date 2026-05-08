#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"

/* ==================== 编码器默认参数 ==================== */
#define ENCODER_LEFT_FORWARD_SIGN_DEFAULT            -1
#define ENCODER_RIGHT_FORWARD_SIGN_DEFAULT           1
#define ENCODER_DEFAULT_FILTER_ALPHA                 0.45f

typedef struct
{
    TIM_HandleTypeDef *timer_handle;
    int8_t forward_sign;
    int16_t last_counter;
    int16_t speed_feedback;
    float filtered_speed;
} EncoderChannel;

typedef struct
{
    float filter_alpha;
} EncoderParams;

typedef struct
{
    EncoderChannel left_channel;
    EncoderChannel right_channel;
    EncoderParams params;
} EncoderHandle;

void Encoder_Init(EncoderHandle *handle, TIM_HandleTypeDef *left_timer, TIM_HandleTypeDef *right_timer);
void Encoder_Start(EncoderHandle *handle);
void Encoder_Update(EncoderHandle *handle);

#endif
