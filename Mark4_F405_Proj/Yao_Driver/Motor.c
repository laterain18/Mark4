#include "Motor.h"

// 外部定时器句柄声明 (需要在main.c中定义)
// extern TIM_HandleTypeDef htim1;  // TIM1用于电机控制
// extern TIM_HandleTypeDef htim4;  // TIM4用于电机控制

/**
 * @brief 设置单个电机速度
 * @param motor: 电机选择联合体
 * @param duty_cycle: 占空比 (0.0 - 100.0)
 * @retval None
 */
void Set_Motor_Speed(Motor_TypeDef motor_id, float duty_cycle)
{
    uint32_t ccr_value;
    
    // 参数边界检查
    if (duty_cycle < 0.0f) duty_cycle = 0.0f;
    if (duty_cycle > MAX_PWM_DUTY) duty_cycle = MAX_PWM_DUTY;
    
    // 获取电机ID
    if (motor_id >= MOTOR_NUM) return;
    
    // 计算CCR值: CCR = (duty_cycle / 100) * (ARR + 1)
    ccr_value = (uint32_t)((duty_cycle / 100.0f) * (PWM_TIMER_PERIOD + 1));
    
    // 根据电机ID设置对应的定时器通道
    switch (motor_id) {
        case MOTOR_LF:  // 左前电机 - TIM4_CH1 (PB6)
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ccr_value);
            break;
            
        case MOTOR_RF:  // 右前电机 - TIM1_CH1 (PA8)  
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_value);
            break;
            
        case MOTOR_LB:  // 左后电机 - TIM4_CH2 (PB7)
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, ccr_value);
            break;
            
        case MOTOR_RB:  // 右后电机 - TIM1_CH4 (PA11)
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, ccr_value);
            break;
            
        default:
            break;
    }
}

/**
 * @brief 停止所有电机
 * @param None
 * @retval None
 */
void Motors_Stop_All(void)
{
    Set_Motor_Speed(MOTOR_LF, 0.0f);
    Set_Motor_Speed(MOTOR_RF, 0.0f);
    Set_Motor_Speed(MOTOR_LB, 0.0f);
    Set_Motor_Speed(MOTOR_RB, 0.0f);
}
