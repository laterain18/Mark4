#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

#define PWM_TIMER_PERIOD 19999  // ARR值，对应50Hz频率
#define MAX_PWM_DUTY     100.0f // 最大占空比100%

// 电机索引定义
typedef enum {
    MOTOR_LF = 0,  // 左前电机
    MOTOR_RF,      // 右前电机  
    MOTOR_LB,      // 左后电机
    MOTOR_RB,      // 右后电机
    MOTOR_NUM      // 电机总数
} Motor_TypeDef;

// 函数声明
void Set_Motor_Speed(Motor_TypeDef motor, float duty_cycle);
void Motors_Stop_All(void);

#endif