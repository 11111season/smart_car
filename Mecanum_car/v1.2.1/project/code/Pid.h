#ifndef CODE_PID_H_
#define CODE_PID_H_

#include "zf_common_headfile.h"
/*----------------------- 结构体变量声明 --------------------------*/
typedef struct
{
    // 控制器参数
    float Kp;
    float Ki;
    float Kd;

    // 设定值和反馈值
    float Target;   // 目标值 (SP)
    float FeedBack; // 反馈值 (FB)

    // 历史状态（必须的）
    float err_k_1;        // 上次误差 err[k-1]
    float err_int_k_1;    // 上次积分值 err_int[k-1]
    uint64_t t_k_1;       // 上次时间 t[k-1]

    // 输出和限幅
    float Output;         // 当前输出
    float UpperLimit;     // 输出上限
    float LowerLimit;     // 输出下限
    float IntegralMax;    // 积分限幅

    // 使能标志
    uint8_t Enable;
} PID;

/*----------------------- 函数接口声明 --------------------------*/
// 函数声明
void PID_Init(PID *pid, float Kp, float Ki, float Kd);
void PID_SetLimit(PID *pid, float Upper, float Lower);
void PID_SetIntegralLimit(PID *pid, float IntegralMax);
void PID_SetTarget(PID *pid, float target);
void PID_Enable(PID *pid, uint8_t enable);
float PID_Calculate(PID *pid, float FeedBack);
void PID_Reset(PID *pid);

#endif /* CODE_PID_H_ */
