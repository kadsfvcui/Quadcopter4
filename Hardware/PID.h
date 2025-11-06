#ifndef __PID_H__
#define __PID_H__

#include "madgwick.h"
#include "pwm.h"
#include "ppm.h"
#include "math.h"

// PID结构体定义
typedef struct {
    float Kp;          // 比例系数
    float Ki;          // 积分系数
    float Kd;          // 微分系数
    float target;      // 目标值
    float output;      // PID输出
    float error;       // 当前误差
    float lastError;   // 上次误差
    float integral;    // 积分项
    float maxOutput;   // 输出限幅
    float maxIntegral; // 积分限幅
} PID_TypeDef;

// 四轴飞行器控制结构体
typedef struct {
    // 外环-姿态角PID
    struct {
        PID_TypeDef roll;   // 横滚角PID
        PID_TypeDef pitch;  // 俯仰角PID
        PID_TypeDef yaw;    // 偏航角PID
    } OuterPID;
    
    // 内环-角速度PID
    struct {
        PID_TypeDef roll;   // 横滚角速度PID
        PID_TypeDef pitch;  // 俯仰角速度PID
        PID_TypeDef yaw;    // 偏航角速度PID
    } InnerPID;
    
    // 飞行器姿态数据
    float roll;      // 当前横滚角
    float pitch;     // 当前俯仰角
    float yaw;       // 当前偏航角
    float rollRate;  // 当前横滚角速度
    float pitchRate; // 当前俯仰角速度
    float yawRate;   // 当前偏航角速度

    // 目标角速度数据
    float targetRollRate;  // 目标横滚角速度
    float targetPitchRate; // 目标俯仰角速度
    float targetYawRate;   // 目标偏航角速度
    
    // 遥控器输入数据
    float rcRoll;    // 遥控器横滚输入
    float rcPitch;   // 遥控器俯仰输入
    float rcYaw;     // 遥控器偏航输入
    float rcThrottle;// 遥控器油门输入
    
    // 控制输出
    float motor1;    // 电机1输出值
    float motor2;    // 电机2输出值
    float motor3;    // 电机3输出值
    float motor4;    // 电机4输出值
} Quad_TypeDef;

// 函数声明
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float maxOutput, float maxIntegral);
float PID_Calculate(PID_TypeDef *pid, float current, float target, float dt);
void PID_Reset(PID_TypeDef *pid);

void QuadPID_Init(Quad_TypeDef *quad);
void QuadPID_Update(Quad_TypeDef *quad, float roll, float pitch, float yaw, 
                   float rollRate, float pitchRate, float yawRate, 
                   uint16_t *ppm, float dt);
void OuterLoop_Update(Quad_TypeDef *quad, float roll, float pitch, float yaw, uint16_t *ppm, float dt);
void InnerLoop_Update(Quad_TypeDef *quad, MPU6050_DataTypeDef *gyro, float dt);
void QuadPID_MotorControl(Quad_TypeDef *quad);

#endif
