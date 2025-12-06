#include "pid.h"

Drone_PIDControllers Drone_Pid;

// 零点
float offset_roll  = -0.33;
float offset_pitch = -0.23;
float offset_yaw   = -0;

// 初始化所有PID控制器
void init_drone_pids(Drone_PIDControllers *pids)
{
    // 平衡 角速度环参数
    pids->roll_gyro.params = (PID_Params){0.0f, 0.0f, 0.0f, 0.0f,  0.5f};
    pids->pitch_gyro.params = (PID_Params){0.0f, 0.0f, 0.0f, 0.0f,  0.5f};
    pids->yaw_gyro.params = (PID_Params){0.0f, 0.0f, 0.0f, 0.0f,  0.5f};

    // 平衡 角度环参数
    pids->roll_angle.params = (PID_Params){0.0f, 0.0f, 0.0f, 0.0f,  0.5f};
    pids->pitch_angle.params = (PID_Params){0.0f, 0.0f, 0.0f, 0.0f,  0.5f};
    pids->yaw_angle.params = (PID_Params){0.0f, 0.0f, 0.0f, 0.0f,  0.5f};
    
}

// 位置式PID
void PID_Calculate_Position(PID_Controller *pid, float NowPoint, float SetPoint)
{
    pid->info.iError = NowPoint - SetPoint;
    
    // 滤波
    pid->info.iError = pid->params.filter_param * pid->info.iError + 
                      (1 - pid->params.filter_param) * pid->info.LastError;
    pid->info.SumError += pid->info.iError;
    
    // 积分限幅
    if (pid->params.integral_limit > 0.001f)  // 避免浮点数比较问题
    {
        pid->info.SumError = lr_limit(pid->info.SumError, pid->params.integral_limit);
    }
    
    // PID计算
    pid->info.output = pid->params.Kp * pid->info.iError +
             pid->params.Ki * pid->info.SumError +
             pid->params.Kd * (pid->info.iError - pid->info.LastError);
    
    pid->info.LastError = pid->info.iError;
}



