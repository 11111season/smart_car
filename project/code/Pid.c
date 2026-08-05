#include "Pid.h"

// 外部时间基准
extern volatile uint64_t time_us;

/*****************************************************************************
 * @name       : PID_Init
 * @date       : 2026-03-10
 * @function   : PID的初始化函数
 * @parameters : 参数Kp,Ki,Kd
 * @retvalue   : 无
 * @note       : 无
******************************************************************************/
void PID_Init(PID *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->Target = 0.0f;
    pid->FeedBack = 0.0f;

    // 历史状态初始化
    pid->err_k_1 = 0.0f;
    pid->err_int_k_1 = 0.0f;
    pid->t_k_1 = 0;

    // 输出初始化
    pid->Output = 0.0f;

    // 默认限幅, 可自行修改
    pid->UpperLimit = 1800.0f;
    pid->LowerLimit = -1800.0f;
    pid->IntegralMax = 800.0f;

    pid->Enable = 0;
}

/*****************************************************************************
 * @name       : PID_SetLimit
 * @date       : 2026-03-10
 * @function   : PID输出限幅函数
 * @parameters : 参数PID结构体, 上下限以及积分限
 * @retvalue   : 无
 * @note       : 无
******************************************************************************/
void PID_SetLimit(PID *pid, float Upper, float Lower)
{
    pid->UpperLimit = Upper;
    pid->LowerLimit = Lower;
}


/*****************************************************************************
 * @name       : PID_SetIntegralLimit
 * @date       : 2026-03-10
 * @function   : PID的积分限幅函数
 * @parameters : 参数PID结构体, 积分限幅
 * @retvalue   : 无
 * @note       : 无
******************************************************************************/
void PID_SetIntegralLimit(PID *pid, float IntegralMax)
{
    pid->IntegralMax = IntegralMax;
}

/*****************************************************************************
 * @name       : PID_SetTarget
 * @date       : 2026-03-10
 * @function   : PID设置目标值函数
 * @parameters : 参数PID结构体, 目标值
 * @retvalue   : 无
 * @note       : 无
******************************************************************************/
void PID_SetTarget(PID *pid, float target)
{
    pid->Target = target;
}

/*****************************************************************************
 * @name       : PID_Enable
 * @date       : 2026-03-10
 * @function   : PID使能函数
 * @parameters : 参数PID结构体, 使能/关闭
 * @retvalue   : 无
 * @note       : 无
******************************************************************************/
void PID_Enable(PID *pid, uint8_t enable)
{
    pid->Enable = enable;
    if (!enable) {
        PID_Reset(pid);
    }
}

/*****************************************************************************
 * @name       : PID_Calculate
 * @date       : 2026-03-10
 * @function   : PID的计算函数
 * @parameters : 参数PID结构体, 反馈值
 * @retvalue   : 计算PID输出的pwm值
 * @note       : 无
******************************************************************************/
float PID_Calculate(PID *pid, float FeedBack)
{
    if (!pid->Enable) {
        return 0.0f;
    }

    pid->FeedBack = FeedBack;

    // 1. 计算当前误差
    float err = pid->Target - pid->FeedBack;

    // 2. 获取时间（微秒）
    uint64_t t_k = time_us;
    float deltaT = (pid->t_k_1 == 0) ? 0.01f : (t_k - pid->t_k_1) * 1e-6f;

    // 防止时间差异常
    if (deltaT <= 0 || deltaT > 0.1f) {
        deltaT = 0.01f;  // 默认10ms
    }

    // 3. 计算各个分量
    float COp = 0.0f;  // 比例输出量
    float COi = 0.0f;  // 积分输出量
    float COd = 0.0f;  // 微分输出量

    // 3.1 比例量
    COp = pid->Kp * err;

    // 3.2 积分量（梯形积分）
    float err_int = pid->err_int_k_1;  // 从上次积分值开始

    if (pid->t_k_1 != 0) {  // 不是第一次计算
        // 梯形积分, 面积 = (上底+下底) × 高 ÷ 2
        float trapezoidal_area = (err + pid->err_k_1) * deltaT * 0.5f;
        err_int += trapezoidal_area;
    } else {
        // 第一次运算, 使用矩形积分
        err_int += err * deltaT;
    }

    // 积分限幅, 防止积分饱和
    if (err_int > pid->IntegralMax) {
        err_int = pid->IntegralMax;
    }
    if (err_int < -pid->IntegralMax) {
        err_int = -pid->IntegralMax;
    }

    COi = pid->Ki * err_int;

    // 3.3 微分项
    if (pid->t_k_1 != 0 && deltaT > 1e-6f) {
        // 标准微分, 斜率 = Δerr / Δt
        COd = pid->Kd * (err - pid->err_k_1) / deltaT;
    }

    // 4. 总输出量
    float CO = COp + COi + COd;

    // 5. 输出限幅
    if (CO > pid->UpperLimit) {
        CO = pid->UpperLimit;
    }
    if (CO < pid->LowerLimit) {
        CO = pid->LowerLimit;
    }

    // 6. 更新历史状态
    pid->err_k_1 = err;          // 保存当前误差, 下次运算为err[k-1]
    pid->err_int_k_1 = err_int;  // 保存当前积分值
    pid->t_k_1 = t_k;            // 保存当前时间
    pid->Output = CO;            // 保存输出值

    return CO;
}

/*****************************************************************************
 * @name       : PID_Reset
 * @date       : 2026-03-10
 * @function   : PID的复位函数
 * @parameters : 参数PID结构体
 * @retvalue   : 无
 * @note       : 无
******************************************************************************/
void PID_Reset(PID *pid)
{
    pid->err_k_1 = 0.0f;
    pid->err_int_k_1 = 0.0f;
    pid->t_k_1 = 0;
    pid->Output = 0.0f;
}
