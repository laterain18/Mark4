#include "interrupt.h"
#include <stdio.h>

void Task_1ms(void)
{

}

float speed_LF = 0.0f;
float speed_RF = 0.0f;
float speed_LB = 0.0f;
float speed_RB = 0.0f;
void Task_5ms(void)
{
    // 角速度环计算
    PID_Calculate_Position(&Drone_Pid.roll_gyro,  imuData.gyro.gyroX, -Drone_Pid.roll_angle.info.output);
    PID_Calculate_Position(&Drone_Pid.pitch_gyro, imuData.gyro.gyroY, -Drone_Pid.pitch_angle.info.output);
    PID_Calculate_Position(&Drone_Pid.yaw_gyro,   imuData.gyro.gyroZ, Drone_Pid.yaw_angle.info.output);

    // 设置速度
    if(BT_Flag == STATE_TAKEOFF)
    {
        Set_Motor_Speed(MOTOR_LF, lr_limit_ab((speed_LF - Drone_Pid.roll_gyro.info.output - Drone_Pid.pitch_gyro.info.output), 0.0f, 50.0f));
        Set_Motor_Speed(MOTOR_LB, lr_limit_ab((speed_LF - Drone_Pid.roll_gyro.info.output + Drone_Pid.pitch_gyro.info.output), 0.0f, 50.0f));
        Set_Motor_Speed(MOTOR_RF, lr_limit_ab((speed_LF + Drone_Pid.roll_gyro.info.output - Drone_Pid.pitch_gyro.info.output), 0.0f, 50.0f));
        Set_Motor_Speed(MOTOR_RB, lr_limit_ab((speed_LF + Drone_Pid.roll_gyro.info.output + Drone_Pid.pitch_gyro.info.output), 0.0f, 50.0f));

        // Set_Motor_Speed(MOTOR_LF, lr_limit_ab(speed_LF, 0.0f, 50.0f));
        // Set_Motor_Speed(MOTOR_LB, lr_limit_ab(speed_LB, 0.0f, 50.0f));
        // Set_Motor_Speed(MOTOR_RF, lr_limit_ab(speed_RF, 0.0f, 50.0f));
        // Set_Motor_Speed(MOTOR_RB, lr_limit_ab(speed_RB, 0.0f, 50.0f));
    }
    else
    {
        Set_Motor_Speed(MOTOR_LF, lr_limit_ab(0, 0.0f, 50.0f));
        Set_Motor_Speed(MOTOR_LB, lr_limit_ab(0, 0.0f, 50.0f));
        Set_Motor_Speed(MOTOR_RF, lr_limit_ab(0, 0.0f, 50.0f));
        Set_Motor_Speed(MOTOR_RB, lr_limit_ab(0, 0.0f, 50.0f));
    }
}

void Task_10ms(void)
{
    // 角度环计算
    PID_Calculate_Position(&Drone_Pid.roll_angle,  imuData.angle.roll,  0);
    PID_Calculate_Position(&Drone_Pid.pitch_angle, imuData.angle.pitch, 0);
    PID_Calculate_Position(&Drone_Pid.yaw_angle,   imuData.angle.yaw,   offset_yaw);
}

void Task_20ms(void)
{

}





