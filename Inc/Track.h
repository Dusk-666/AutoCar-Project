#ifndef TRACK_H
#define TRACK_H

#include "main.h"

/* ==================== 基础速度参数 ==================== */
#define TRACK_RIGHT_BASE_SPEED    250 右轮基础速度
#define TRACK_SPEED_RATIO         1778  // 速度比例（左轮速度 = 右轮速度 × 1.778）

/* ==================== 速度限制参数 ==================== */
#define MIN_SPEED                 50    // 最小速度，确保电机始终转动
#define MAX_SPEED                 999   // 最大速度（PWM上限）

/* ==================== PD控制参数 ==================== */
#define TRACK_KP                  40    // 比例系数：响应当前误差
#define TRACK_KD                  25    // 微分系数：响应误差变化趋势
#define TRACK_MAX_STEER           200   // 最大转向量，限制PD输出

/* ==================== 转弯参数 ==================== */
#define TURN_INNER_RATIO          40    // 内轮速度比例（百分比，40表示40%）
#define TURN_OUTER_RATIO          130   // 外轮速度比例（百分比，130表示130%）
#define SPEED_RAMP_STEP           20    // 速度渐变步长，每次调整的量

/* ==================== 状态机参数 ==================== */
#define OUTER_SENSOR_CONFIRM      3     // 外侧传感器确认次数，防止误触发
#define MAX_TURN_TIME             100   // 最大转弯时间（控制周期数，100×5ms=500ms）

/* ==================== 十字路口参数 ==================== */
#define CROSS_SPEED_RATIO        100  // 十字路口速度比例（百分比，60表示60%）

/* ==================== 传感器状态定义 ==================== */
#define SENSOR_BLACK              0     // 检测到黑线
#define SENSOR_WHITE              1     // 检测到白底

/* ==================== 状态定义 ==================== */
typedef enum {
    TRACKING,      // 正常循迹状态（使用PD控制）
    TURN_LEFT,     // 大角度左转状态（使用差速转向）
    TURN_RIGHT,    // 大角度右转状态（使用差速转向）
    STOP           // 停车状态
} TrackState;

/* ==================== 函数声明 ==================== */
void Track_Init(void);
void Track_ControlLoop(void);

#endif
