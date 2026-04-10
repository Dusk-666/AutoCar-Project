#ifndef SERVO_H
#define SERVO_H

#include "main.h"

/* ==================== 云台控制参数 ==================== */
#define SERVO_MIN               50    // 舵机最小位置（0度）
#define SERVO_MAX               250   // 舵机最大位置（180度）
#define SERVO_CENTER            150   // 舵机中心位置（90度）
#define SERVO_SQUARE_SIZE       30    // 正方形边长（PWM值）
#define SERVO_DELAY_TIME        500   // 每个顶点停留时间（毫秒）

/* ==================== 函数声明 ==================== */
void Servo_Init(void);
void Servo_DrawSquare(void);
void Servo_TestRange(void);
void Servo_DrawCircle(void);

#endif
