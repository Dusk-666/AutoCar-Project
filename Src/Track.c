#include "Track.h"

extern TIM_HandleTypeDef htim4;

/* ==================== 传感器读取函数 ==================== */

/**
 * @brief 读取左外传感器状态
 * @return SENSOR_BLACK(0)表示检测到黑线，SENSOR_WHITE(1)表示检测到白底
 */
static uint8_t Track_ReadLeftOut(void)
{
    return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
}

/**
 * @brief 读取左内传感器状态
 * @return SENSOR_BLACK(0)表示检测到黑线，SENSOR_WHITE(1)表示检测到白底
 */
static uint8_t Track_ReadLeftIn(void)
{
    return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
}

/**
 * @brief 读取右内传感器状态
 * @return SENSOR_BLACK(0)表示检测到黑线，SENSOR_WHITE(1)表示检测到白底
 */
static uint8_t Track_ReadRightIn(void)
{
    return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
}

/**
 * @brief 读取右外传感器状态
 * @return SENSOR_BLACK(0)表示检测到黑线，SENSOR_WHITE(1)表示检测到白底
 */
static uint8_t Track_ReadRightOut(void)
{
    return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
}

/* ==================== 工具函数 ==================== */

/**
 * @brief 限制速度在有效范围内
 * @param speed 输入速度值
 * @return 限制后的速度值，范围[MIN_SPEED, MAX_SPEED]
 */
static uint16_t Track_ClampSpeed(int speed)
{
    if (speed < MIN_SPEED) return MIN_SPEED;
    if (speed > MAX_SPEED) return MAX_SPEED;
    return (uint16_t)speed;
}

/**
 * @brief 将右轮速度转换为左轮速度（应用速度比例）
 * @param right_speed 右轮速度
 * @return 左轮速度 = 右轮速度 × 1.778
 */
static uint16_t Track_RightToLeftSpeed(uint16_t right_speed)
{
    return (uint16_t)((right_speed * TRACK_SPEED_RATIO) / 1000);
}

/**
 * @brief 速度渐变函数，实现平滑的速度变化
 * @param current 当前速度
 * @param target 目标速度
 * @param step 每次调整的步长
 * @return 渐变后的速度
 */
static uint16_t Track_RampSpeed(uint16_t current, uint16_t target, uint16_t step)
{
    if (current < target)
    {
        current += step;
        if (current > target) current = target;
    }
    else if (current > target)
    {
        if (current - step < target) current = target;
        else current -= step;
    }
    return current;
}

/* ==================== 电机控制函数 ==================== */

/**
 * @brief 设置左电机状态和速度
 * @param forward 1表示前进，0表示后退
 * @param speed PWM速度值，范围[0, 999]
 */
static void Track_SetLeftMotor(uint8_t forward, uint16_t speed)
{
    if (forward)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
    }
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, speed);
}

/**
 * @brief 设置右电机状态和速度
 * @param forward 1表示前进，0表示后退
 * @param speed PWM速度值，范围[0, 999]
 */
static void Track_SetRightMotor(uint8_t forward, uint16_t speed)
{
    if (forward)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
    }
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, speed);
}

/* ==================== 状态机上下文 ==================== */

static struct {
    TrackState state;              // 当前状态
    int error;                     // 当前误差
    int last_error;                // 上次误差（用于PD控制的D项）
    uint16_t turn_count;           // 转弯时间计数器
    uint8_t left_out_count;        // 左外传感器触发计数
    uint8_t right_out_count;       // 右外传感器触发计数
    uint16_t left_actual_speed;    // 左轮实际速度
    uint16_t right_actual_speed;   // 右轮实际速度
} ctx;

/* ==================== 初始化函数 ==================== */

/**
 * @brief 初始化循迹模块
 * @note 使能电机驱动芯片，初始化状态机上下文
 */
void Track_Init(void)
{
    // 使能电机驱动芯片（PB6和PB7置高）
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    
    // 初始化电机为停止状态
    Track_SetLeftMotor(1, 0);
    Track_SetRightMotor(1, 0);
    
    // 初始化状态机上下文
    ctx.state = TRACKING;
    ctx.error = 0;
    ctx.last_error = 0;
    ctx.turn_count = 0;
    ctx.left_out_count = 0;
    ctx.right_out_count = 0;
    ctx.left_actual_speed = 0;
    ctx.right_actual_speed = 0;
}

/* ==================== 主控制循环 ==================== */

/**
 * @brief 循迹控制主循环
 * @note 每次调用执行一次控制周期，建议在main循环中以5ms周期调用
 */
void Track_ControlLoop(void)
{
    // ========== 1. 读取所有传感器状态 ==========
    uint8_t left_out = Track_ReadLeftOut();   // 左外传感器
    uint8_t left_in = Track_ReadLeftIn();     // 左内传感器
    uint8_t right_in = Track_ReadRightIn();   // 右内传感器
    uint8_t right_out = Track_ReadRightOut(); // 右外传感器
    
    // ========== 2. 检测特殊情况：全黑或全白 ==========
    uint8_t all_black = (left_out == SENSOR_BLACK && left_in == SENSOR_BLACK && 
                         right_in == SENSOR_BLACK && right_out == SENSOR_BLACK);
    uint8_t all_white = (left_out == SENSOR_WHITE && left_in == SENSOR_WHITE && 
                         right_in == SENSOR_WHITE && right_out == SENSOR_WHITE);
    
    // ========== 3. 状态机主逻辑 ==========
    switch (ctx.state)
    {
        /* ---------- TRACKING状态：使用PD控制 ---------- */
        case TRACKING:
        {
            // 处理十字路口（全黑）：减速直行通过
            if (all_black)
            {
                // 计算目标速度：降低速度通过十字路口
                int left_target = (TRACK_RIGHT_BASE_SPEED * CROSS_SPEED_RATIO) / 100;
                int right_target = (TRACK_RIGHT_BASE_SPEED * CROSS_SPEED_RATIO) / 100;
                
                // 应用速度比例
                left_target = Track_RightToLeftSpeed(left_target);
                
                // 限制速度
                left_target = Track_ClampSpeed(left_target);
                right_target = Track_ClampSpeed(right_target);
                
                // 速度渐变
                ctx.left_actual_speed = Track_RampSpeed(ctx.left_actual_speed, left_target, SPEED_RAMP_STEP);
                ctx.right_actual_speed = Track_RampSpeed(ctx.right_actual_speed, right_target, SPEED_RAMP_STEP);
                
                // 设置电机速度
                Track_SetLeftMotor(1, ctx.left_actual_speed);
                Track_SetRightMotor(1, ctx.right_actual_speed);
                
                // 保持TRACKING状态，等待传感器恢复正常
                break;
            }
            
            // 处理脱线（全白）：保持当前速度继续前进
            if (all_white)
            {
                // 保持当前速度，继续前进
                // 小车会按照惯性继续前进，直到找到黑线
                Track_SetLeftMotor(1, ctx.left_actual_speed);
                Track_SetRightMotor(1, ctx.right_actual_speed);
                
                // 保持TRACKING状态，等待传感器恢复正常
                break;
            }
            
            // 计算误差值
            // 误差定义：负值表示偏右，正值表示偏左
            if (left_in == SENSOR_BLACK && right_in == SENSOR_BLACK)
            {
                ctx.error = 0;  // 完美居中
            }
            else if (left_in == SENSOR_BLACK && right_in == SENSOR_WHITE)
            {
                ctx.error = -2;  // 偏右，需要向左修正
            }
            else if (left_in == SENSOR_WHITE && right_in == SENSOR_BLACK)
            {
                ctx.error = +2;  // 偏左，需要向右修正
            }
            else
            {
                ctx.error = 0;  // 默认情况
            }
            
            // PD控制计算
            // 转向量 = Kp × 误差 + Kd × 误差变化率
            int p_term = TRACK_KP * ctx.error;
            int d_term = TRACK_KD * (ctx.error - ctx.last_error);
            int steer = p_term + d_term;
            
            // 限制转向量，防止过大的转向
            if (steer > TRACK_MAX_STEER) steer = TRACK_MAX_STEER;
            if (steer < -TRACK_MAX_STEER) steer = -TRACK_MAX_STEER;
            
            // 计算目标速度
            // 左轮速度 = 基础速度 - 转向量
            // 右轮速度 = 基础速度 + 转向量
            int left_target = TRACK_RIGHT_BASE_SPEED - steer;
            int right_target = TRACK_RIGHT_BASE_SPEED + steer;
            
            // 限制速度在有效范围内
            left_target = Track_ClampSpeed(left_target);
            right_target = Track_ClampSpeed(right_target);
            
            // 应用速度比例：左轮速度 = 右轮速度 × 1.778
            left_target = Track_RightToLeftSpeed(left_target);
            
            // 速度渐变，实现平滑过渡
            ctx.left_actual_speed = Track_RampSpeed(ctx.left_actual_speed, left_target, SPEED_RAMP_STEP);
            ctx.right_actual_speed = Track_RampSpeed(ctx.right_actual_speed, right_target, SPEED_RAMP_STEP);
            
            // 设置电机速度
            Track_SetLeftMotor(1, ctx.left_actual_speed);
            Track_SetRightMotor(1, ctx.right_actual_speed);
            
            // 检查是否需要进入转弯状态
            // 外侧传感器触发计数，防止误触发
            if (left_out == SENSOR_BLACK)
            {
                ctx.left_out_count++;
                if (ctx.left_out_count >= OUTER_SENSOR_CONFIRM)
                {
                    ctx.state = TURN_LEFT;
                    ctx.turn_count = 0;
                    ctx.left_out_count = 0;
                }
            }
            else
            {
                ctx.left_out_count = 0;
            }
            
            if (right_out == SENSOR_BLACK)
            {
                ctx.right_out_count++;
                if (ctx.right_out_count >= OUTER_SENSOR_CONFIRM)
                {
                    ctx.state = TURN_RIGHT;
                    ctx.turn_count = 0;
                    ctx.right_out_count = 0;
                }
            }
            else
            {
                ctx.right_out_count = 0;
            }
            
            // 保存当前误差用于下次计算
            ctx.last_error = ctx.error;
            break;
        }
        
        /* ---------- TURN_LEFT状态：大角度左转 ---------- */
        case TURN_LEFT:
        {
            // 差速转向：左轮减速，右轮加速
            // 内轮（左轮）速度 = 基础速度 × 40%
            // 外轮（右轮）速度 = 基础速度 × 130%
            int left_target = (TRACK_RIGHT_BASE_SPEED * TURN_INNER_RATIO) / 100;
            int right_target = (TRACK_RIGHT_BASE_SPEED * TURN_OUTER_RATIO) / 100;
            
            // 确保速度不小于最小值
            if (left_target < MIN_SPEED) left_target = MIN_SPEED;
            right_target = Track_ClampSpeed(right_target);
            
            // 应用速度比例
            left_target = Track_RightToLeftSpeed(left_target);
            
            // 速度渐变
            ctx.left_actual_speed = Track_RampSpeed(ctx.left_actual_speed, left_target, SPEED_RAMP_STEP);
            ctx.right_actual_speed = Track_RampSpeed(ctx.right_actual_speed, right_target, SPEED_RAMP_STEP);
            
            // 设置电机速度
            Track_SetLeftMotor(1, ctx.left_actual_speed);
            Track_SetRightMotor(1, ctx.right_actual_speed);
            
            // 转弯时间计数
            ctx.turn_count++;
            // 
            // 检查退出条件
            // 条件：中间两个传感器都检测到黑线，说明已经恢复居中
            if (left_in == SENSOR_BLACK && right_in == SENSOR_BLACK)
            {
                ctx.state = TRACKING;
                ctx.turn_count = 0;
                ctx.last_error = 0;  // 重置误差
            }
            break;
        }
        
        /* ---------- TURN_RIGHT状态：大角度右转 ---------- */
        case TURN_RIGHT:
        {
            // 差速转向：左轮加速，右轮减速
            // 外轮（左轮）速度 = 基础速度 × 130%
            // 内轮（右轮）速度 = 基础速度 × 40%
            int left_target = (TRACK_RIGHT_BASE_SPEED * TURN_OUTER_RATIO) / 100;
            int right_target = (TRACK_RIGHT_BASE_SPEED * TURN_INNER_RATIO) / 100;
            
            // 应用速度比例
            left_target = Track_RightToLeftSpeed(left_target);
            
            // 确保速度不小于最小值
            left_target = Track_ClampSpeed(left_target);
            if (right_target < MIN_SPEED) right_target = MIN_SPEED;
            
            // 速度渐变
            ctx.left_actual_speed = Track_RampSpeed(ctx.left_actual_speed, left_target, SPEED_RAMP_STEP);
            ctx.right_actual_speed = Track_RampSpeed(ctx.right_actual_speed, right_target, SPEED_RAMP_STEP);
            
            // 设置电机速度
            Track_SetLeftMotor(1, ctx.left_actual_speed);
            Track_SetRightMotor(1, ctx.right_actual_speed);
            
            // 转弯时间计数
            ctx.turn_count++;
            
            // 检查退出条件
            // 条件：中间两个传感器都检测到黑线，说明已经恢复居中
            if (left_in == SENSOR_BLACK && right_in == SENSOR_BLACK)
            {
                ctx.state = TRACKING;
                ctx.turn_count = 0;
                ctx.last_error = 0;  // 重置误差
            }
            break;
        }
        
        /* ---------- STOP状态：停车 ---------- */
        case STOP:
        {
            // 停止所有电机
            Track_SetLeftMotor(1, 0);
            Track_SetRightMotor(1, 0);
            
            // 重置实际速度
            ctx.left_actual_speed = 0;
            ctx.right_actual_speed = 0;
            
            // 检查恢复条件：不是全黑且不是全白
            if (!all_black && !all_white)
            {
                ctx.state = TRACKING;
                ctx.last_error = 0;
            }
            break;
        }
    }
}
