#ifndef PID_H
#define PID_H

#include "main.h"

#define lr_limit(a,b)   ((a) < -(b) ? -(b) : ((a) > (b) ? (b) : (a)))
#define lr_limit_ab(x, a, b) ((x) < (a) ? (a) : ((x) > (b) ? (b) : (x)))

// PID参数结构体
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral_limit;
    float filter_param;
} PID_Params;

// PID信息结构体
typedef struct {
    float LastError;
    float SumError;
    float iError;
    float output;
} PID_Info;

// 完整的PID控制器结构体
typedef struct {
    PID_Params params;  // 参数
    PID_Info info;      // 运行时信息
    const char *name;   // 控制器名称（调试用）
} PID_Controller;

// 无人机PID控制器
typedef struct {
    // 角速度环
    PID_Controller roll_gyro;
    PID_Controller pitch_gyro;
    PID_Controller yaw_gyro;

    // 角度环
    PID_Controller roll_angle;
    PID_Controller pitch_angle;
    PID_Controller yaw_angle;

} Drone_PIDControllers;

extern Drone_PIDControllers Drone_Pid;
extern float offset_roll;
extern float offset_pitch;
extern float offset_yaw;

void init_drone_pids(Drone_PIDControllers *pids);
void PID_Calculate_Position(PID_Controller *pid, float NowPoint, float SetPoint);

#endif