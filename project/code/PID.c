#include "zf_common_headfile.h"

// PID参数初始化
// 注意：这些参数需要根据实际硬件调参，这里提供的是典型的四旋翼初始值

void PID_param_Init(void)
{
    //============ 内环角速度PID (需要PD控制，积分可能不需要) ============
    // Roll 内环 - 控制横滚角速度
    PIDRateX.kp = 0.0f; //1.2f;          // 比例增益 (根据实际调整)
    PIDRateX.ki = 0.0f;          // 积分增益 (内环通常不需要)
    PIDRateX.kd = 0.0f; //0.05f;         // 微分增益 (提供阻尼)
    PIDRateX.Integ_LimitHigh = 0;
    PIDRateX.Integ_LimitLow = 0;
    PIDRateX.Out_LimitHigh = 400; // 内环输出限幅
    PIDRateX.Out_LimitLow = -400;
    
    // Pitch 内环 - 控制俯仰角速度
    PIDRateY.kp = 0.0f; //1.2f;
    PIDRateY.ki = 0.0f;
    PIDRateY.kd = 0.0f; //0.05f;
    PIDRateY.Integ_LimitHigh = 0;
    PIDRateY.Integ_LimitLow = 0;
    PIDRateY.Out_LimitHigh = 400;
    PIDRateY.Out_LimitLow = -400;
    
    // Yaw 内环 - 控制偏航角速度
    PIDRateZ.kp = 0.0f; //1.5f;          // 偏航通常需要更大的增益
    PIDRateZ.ki = 0.0f;
    PIDRateZ.kd = 0.0f; //0.02f;
    PIDRateZ.Integ_LimitHigh = 0;
    PIDRateZ.Integ_LimitLow = 0;
    PIDRateZ.Out_LimitHigh = 400;
    PIDRateZ.Out_LimitLow = -400;
    
    //============ 外环角度PID (只需要P控制) ============
    // Roll 外环 - 控制横滚角度
    PIDRoll.kp = 0.0f; //6.0f;           // 角度P增益
    PIDRoll.ki = 0.0f;
    PIDRoll.kd = 0.0f;
    PIDRoll.Integ_LimitHigh = 0;
    PIDRoll.Integ_LimitLow = 0;
    PIDRoll.Out_LimitHigh = 200;  // 外环输出限幅 (作为内环目标)
    PIDRoll.Out_LimitLow = -200;
    
    // Pitch 外环 - 控制俯仰角度
    PIDPitch.kp = 0.0f; //6.0f;
    PIDPitch.ki = 0.0f;
    PIDPitch.kd = 0.0f;
    PIDPitch.Integ_LimitHigh = 0;
    PIDPitch.Integ_LimitLow = 0;
    PIDPitch.Out_LimitHigh = 200;
    PIDPitch.Out_LimitLow = -200;
    
    // Yaw 外环 - 控制偏航角度
    PIDYaw.kp = 0.0f; //5.0f;            // 偏航角度P增益
    PIDYaw.ki = 0.0f;
    PIDYaw.kd = 0.0f;
    PIDYaw.Integ_LimitHigh = 50; // 可以适当加入积分消除稳态误差
    PIDYaw.Integ_LimitLow = -50;
    PIDYaw.Out_LimitHigh = 200;
    PIDYaw.Out_LimitLow = -200;
}


//-------------PID数据复位-------------
void PID_Rest(_PID_param_st **pid, const uint8_t len)
{
    uint8_t i;
    for(i = 0; i < len; i++)
    {
        pid[i]->integ = 0;
        pid[i]->pre_Error = 0;
        pid[i]->out = 0;
        pid[i]->target = 0;
        pid[i]->measured = 0;
    }
}


//-------------PID计算-------------
void PID_Update(_PID_param_st* pid, const float dt)
{
    float error;
    float deriv;
    float output;
    
    if(dt <= 0) return;
    
    error = pid->target - pid->measured; //当前角度与实际角度的误差

    pid->integ += error * dt;     //误差积分累加值
    
    // ===== 微分（抗噪）=====
    deriv = -(pid->measured - pid->prev_measured) / dt;
    
    // ===== P + D =====
    pid->out = pid->kp * error + pid->kd * deriv;  
    
    // ===== 抗积分饱和 =====
    if(output < pid->Out_LimitHigh && output > pid->Out_LimitLow)
    {
        pid->integ += error * dt;
    }
    
    // 积分限幅 
    pid->integ = LIMIT(pid->integ, pid->Integ_LimitLow, pid->Integ_LimitHigh);
    
    //输出
    pid->out = pid->kp * error + pid->ki * pid->integ + pid->kd * deriv; //PID输出
    
    // 输出限幅
    pid->out = LIMIT(pid->out, pid->Out_LimitLow, pid->Out_LimitHigh);
    
    //更新上次的误差
    pid->pre_Error = error;  
    pid->prev_measured = pid->measured;
}


//-----------串级PID------------
void Cascade_PID(_PID_param_st* pid_rate, _PID_param_st* pid_eular, const float dt)
{	 
    PID_Update(pid_eular, dt);    //先计算外环
    pid_rate->target = pid_eular->out;
    PID_Update(pid_rate, dt);    //再计算内环	
}
