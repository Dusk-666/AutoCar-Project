#include "Servo.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

extern TIM_HandleTypeDef htim2;

void Servo_Init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, SERVO_CENTER);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, SERVO_CENTER);
}

void Servo_DrawSquare(void)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, SERVO_CENTER);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, SERVO_CENTER);
    HAL_Delay(SERVO_DELAY_TIME);
    
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, SERVO_CENTER + SERVO_SQUARE_SIZE);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, SERVO_CENTER);
    HAL_Delay(SERVO_DELAY_TIME);
    
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, SERVO_CENTER + SERVO_SQUARE_SIZE);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, SERVO_CENTER + SERVO_SQUARE_SIZE);
    HAL_Delay(SERVO_DELAY_TIME);
    
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, SERVO_CENTER);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, SERVO_CENTER + SERVO_SQUARE_SIZE);
    HAL_Delay(SERVO_DELAY_TIME);
}

void Servo_DrawCircle(void)
{
    const int circle_radius = 30;    // 圆的半径（PWM值）
    const double angle_step = 0.05;  // 角度步进（弧度）
    double angle = 0.0;
    
    while (1)
    {
        int x = SERVO_CENTER + (int)(circle_radius * cos(angle));
        int y = SERVO_CENTER + (int)(circle_radius * sin(angle));
        
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, x);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, y);
        
        angle += angle_step;
        if (angle >= 2 * M_PI)
        {
            angle = 0.0;
        }
        
        HAL_Delay(10);  // 控制画圆速度
    }
}
