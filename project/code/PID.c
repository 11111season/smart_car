#include "zf_common_headfile.h"

// PID参数初始化
void PID_param_Init(void)
{
    //============ 内环角速度PID (需要PD控制，积分可能不需要) ============
    // Roll 内环 
    PIDVelX.kp = 1.6f; //1.2f;          
    PIDVelX.ki = PIDVelX.kp * 2.8;          
 //   PIDVelX.kd = PIDVelX.kp * 0.01; //0.05f;      0.005抖动没有明显改善 ，几乎不影响  
    PIDVelX.Integ_LimitHigh = 800;
    PIDVelX.Integ_LimitLow = -800;
    PIDVelX.Out_LimitHigh = 400; 
    PIDVelX.Out_LimitLow = -400;
       
    // Pitch 内环 
    PIDVelY.kp = 1.6f; //1.2f;
    PIDVelY.ki = PIDVelY.kp * 2.8f;
 //   PIDVelY.kd = 0.03f; //0.05f;
    PIDVelY.Integ_LimitHigh = 0;
    PIDVelY.Integ_LimitLow = 0;
    PIDVelY.Out_LimitHigh = 400;
    PIDVelY.Out_LimitLow = -400;
    
    // Yaw 内环 
    PIDVelZ.kp = 1.6f; //1.5f;         
    PIDVelZ.ki = 0.0f;
    PIDVelZ.kd = 0.0f; //0.02f;
    PIDVelZ.Integ_LimitHigh = 0;
    PIDVelZ.Integ_LimitLow = 0;
    PIDVelZ.Out_LimitHigh = 400;
    PIDVelZ.Out_LimitLow = -400;
    
    //============ 外环角度PID (只需要P控制) ============
    // Roll 外环 
    PIDRoll.kp = 1.5f; //6.0f;           
    PIDRoll.ki = 0.0f;
    PIDRoll.kd = 0.01f;
    PIDRoll.Integ_LimitHigh = 0;
    PIDRoll.Integ_LimitLow = 0;
    PIDRoll.Out_LimitHigh = 200;  
    PIDRoll.Out_LimitLow = -200;
    
    // Pitch 外环 
    PIDPitch.kp = 1.5f; //6.0f;
    PIDPitch.ki = 0.0f;
    PIDPitch.kd = 0.01f;
    PIDPitch.Integ_LimitHigh = 0;
    PIDPitch.Integ_LimitLow = 0;
    PIDPitch.Out_LimitHigh = 200;
    PIDPitch.Out_LimitLow = -200;
    
    // Yaw 外环 
    PIDYaw.kp = 1.0f; //5.0f;          
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
    pid->target = target;
    
    pid->measured=measured;
    //目标值减去当前值
    pid->error = pid->target - pid->measured; 
    
    // 误差微分（抗噪）
    pid->deriv = -(pid->error - pid->last_error) / dt;

    pid->deriv = PT1Filter_Apply(&filter_imu_gyro_x,pid->deriv);
  
    pid->integ += pid->ki * pid->error * dt;
    
    if(pid->integ>200)
      pid->integ = 200;
    else if(pid->integ<-200)
      pid->integ = -200;
    
    pid->out = pid->kp * pid->error +  pid->integ ; //PID输出

    pid->out += pid->kd * pid->deriv;

    // 输出限幅
    pid->out = LIMIT(pid->out, pid->Out_LimitLow, pid->Out_LimitHigh);
    
    //更新上次的误差
    pid->last_error = pid->error;  //误差微分，待定

}

void PID_Update_d_measure(_PID_param_st* pid, float target, float measured, const float dt)
{
//    float temp_output;
    
    if(dt <= 0) return;
    
    //传参
    pid->target=target;
    
    pid->measured=measured;
    //目标值减去当前值
    pid->error = pid->target - pid->measured; 
    
    // 测量微分
    pid->deriv = (measured - pid->last_measured) / dt;

  

    pid->integ += pid->error * dt;
    
    
    pid->out = pid->kp * pid->error + pid->ki * pid->integ + pid->kd * pid->deriv; //PID输出
    
    // 输出限幅
   // pid->out = LIMIT(pid->out, pid->Out_LimitLow, pid->Out_LimitHigh);
    
    //更新上次的误差
    pid->last_error = pid->error;  //误差微分，待定
    pid->last_measured = measured;
}

////-----------串级PID------------
//void Cascade_PID(_PID_param_st* pid_rate, _PID_param_st* pid_eular, const float dt)
//{	 
//    PID_Update(pid_eular, dt);    //先计算外环
//    pid_rate->target = pid_eular->out;
//    PID_Update(pid_rate, dt);    //再计算内环	
//}
