#include "zf_common_headfile.h"

// PID参数初始化
void PID_param_Init(void)
{
    //============ 内环角速度PID (需要PD控制，积分可能不需要) ============
    // Roll 内环 
    PIDVelX.kp = 1.0047f; //1.2f;          
    PIDVelX.ki = 2.8163f;          
    PIDVelX.kd = 0.001f; //0.05f;         
    PIDVelX.Integ_LimitHigh = 800;
    PIDVelX.Integ_LimitLow = -800;
    PIDVelX.Out_LimitHigh = 200; 
    PIDVelX.Out_LimitLow = -200;
       
    // Pitch 内环 
    PIDVelY.kp = 0.0f; //1.2f;
    PIDVelY.ki = 0.0f;
    PIDVelY.kd = 0.0f; //0.05f;
    PIDVelY.Integ_LimitHigh = 0;
    PIDVelY.Integ_LimitLow = 0;
    PIDVelY.Out_LimitHigh = 400;
    PIDVelY.Out_LimitLow = -400;
    
    // Yaw 内环 
    PIDVelZ.kp = 4.0f; //1.5f;         
    PIDVelZ.ki = 0.0f;
    PIDVelZ.kd = 0.0f; //0.02f;
    PIDVelZ.Integ_LimitHigh = 0;
    PIDVelZ.Integ_LimitLow = 0;
    PIDVelZ.Out_LimitHigh = 400;
    PIDVelZ.Out_LimitLow = -400;
    
    //============ 外环角度PID (只需要P控制) ============
    // Roll 外环 
    PIDRoll.kp = 1.0f; //6.0f;           
    PIDRoll.ki = 0.0f;
    PIDRoll.kd = 0.0f;
    PIDRoll.Integ_LimitHigh = 0;
    PIDRoll.Integ_LimitLow = 0;
    PIDRoll.Out_LimitHigh = 200;  
    PIDRoll.Out_LimitLow = -200;
    
    // Pitch 外环 
    PIDPitch.kp = 0.0f; //6.0f;
    PIDPitch.ki = 0.0f;
    PIDPitch.kd = 0.0f;
    PIDPitch.Integ_LimitHigh = 0;
    PIDPitch.Integ_LimitLow = 0;
    PIDPitch.Out_LimitHigh = 200;
    PIDPitch.Out_LimitLow = -200;
    
    // Yaw 外环 
    PIDYaw.kp = 0.0f; //5.0f;          
    PIDYaw.ki = 0.0f;
    PIDYaw.kd = 0.0f;
    PIDYaw.Integ_LimitHigh = 50; 
    PIDYaw.Integ_LimitLow = -50;
    PIDYaw.Out_LimitHigh = 200;
    PIDYaw.Out_LimitLow = -200;
    
    //============ 外环角度PID (只需要P控制) ============
    
    
    

}


//-------------PID数据复位-------------
void PID_Rest_Init(_PID_param_st **pid, const uint8_t len)
{
    uint8_t i;
    for(i = 0; i < len; i++)
    {        
        pid[i]->error = 0;
        pid[i]->last_error = 0;
        
        pid[i]->measured = 0;
        pid[i]->last_measured = 0;
        
        pid[i]->deriv = 0;
        pid[i]->last_deriv = 0;
       
        //上电置0，别的不置
        pid[i]->target = 0;
        
        pid[i]->integ = 0;
        pid[i]->out = 0;
    }
}

void PID_Rest_Change(_PID_param_st **pid, const uint8_t len)
{
    uint8_t i;
    for(i = 0; i < len; i++)
    {        
        pid[i]->error = 0;
        pid[i]->last_error = 0;
        pid[i]->integ = 0;
        pid[i]->out = 0;
        
//        pid[i]->measured = 0;
//        pid[i]->last_measured = 0;
//        
//        pid[i]->deriv = 0;
//        pid[i]->last_deriv = 0;
    }
}

//-------------PID计算-------------
void PID_Update(_PID_param_st* pid, float target, float measured, const float dt)
{
//    float temp_output;
    
    if(dt <= 0) return;
    
    //传参
    pid->target=target;
    pid->measured=measured;
    //当前角度与实际角度的误差
    pid->error = pid->target - pid->measured; 
    
    // 误差微分（抗噪）
    pid->deriv = -(pid->error - pid->last_error) / dt;
//    // 测量微分（抗噪）
//    pid->deriv = -(pid->measured - pid->last_measured) / dt;

//    // 微分滤波
//    pid->deriv = 0.7f * pid->last_deriv + 0.3f * pid->deriv;
//    pid->last_deriv = pid->deriv;
    
    // 预测输出
//    temp_output = pid->kp * pid->error + pid->ki * pid->integ + pid->kd * pid->deriv;  
    
//    // 抗积分饱和 
//    if(temp_output < pid->Out_LimitHigh && temp_output > pid->Out_LimitLow)
//    {
        pid->integ += pid->error * dt;
//    }
    
    // 积分限幅 
    if(pid->integ >= 200)pid->integ=200;
    if(pid->integ <= -200)pid->integ=-200;

//    pid->integ = LIMIT(pid->integ, pid->Integ_LimitLow, pid->Integ_LimitHigh);
    
    //输出
    pid->out = pid->kp * pid->error + pid->ki * pid->integ + pid->kd * pid->deriv; //PID输出
    
    // 输出限幅
    pid->out = LIMIT(pid->out, pid->Out_LimitLow, pid->Out_LimitHigh);
    
    //更新上次的误差
    pid->last_error = pid->error;  //误差微分，待定
//    pid->last_measured = pid->measured;//测量微分
}



////-----------串级PID------------
//void Cascade_PID(_PID_param_st* pid_rate, _PID_param_st* pid_eular, const float dt)
//{	 
//    PID_Update(pid_eular, dt);    //先计算外环
//    pid_rate->target = pid_eular->out;
//    PID_Update(pid_rate, dt);    //再计算内环	
//}
