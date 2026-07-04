// =============================================================================
// PID.c - PID控制实现
// =============================================================================
#include "zf_common_headfile.h"

float anti_gravity_boost = 0.0f;
static PT1Filter_t anti_gravity_filter;

// =============================================================================
// PID参数初始化
// =============================================================================
void PID_param_Init(void)
{
    // ---------------------------- 内环角速度PID参数 (PD控制) ----------------------------
    // Roll 内环
    PIDVelX.kp = VELX_KP;
    PIDVelX.ki = VELX_KI;
    PIDVelX.kd = VELX_KD;
    PIDVelX.Integ_LimitHigh = VEL_INTEG_LIMIT_HIGH;
    PIDVelX.Integ_LimitLow = VEL_INTEG_LIMIT_LOW;
    PIDVelX.Out_LimitHigh = VELX_OUT_LIMIT_HIGH;
    PIDVelX.Out_LimitLow = VELX_OUT_LIMIT_LOW;

    // Pitch 内环
    PIDVelY.kp = VELY_KP;
    PIDVelY.ki = VELY_KI;
    PIDVelY.kd = VELY_KD;
    PIDVelY.Integ_LimitHigh = VEL_INTEG_LIMIT_HIGH;
    PIDVelY.Integ_LimitLow = VEL_INTEG_LIMIT_LOW;
    PIDVelY.Out_LimitHigh = VELY_OUT_LIMIT_HIGH;
    PIDVelY.Out_LimitLow = VELY_OUT_LIMIT_LOW;

    // Yaw 内环
    PIDVelZ.kp = VELZ_KP;
    PIDVelZ.ki = VELZ_KI;
    PIDVelZ.kd = VELZ_KD;
    PIDVelZ.Integ_LimitHigh = VEL_INTEG_LIMIT_HIGH;
    PIDVelZ.Integ_LimitLow = VEL_INTEG_LIMIT_LOW;
    PIDVelZ.Out_LimitHigh = VELZ_OUT_LIMIT_HIGH;
    PIDVelZ.Out_LimitLow = VELZ_OUT_LIMIT_LOW;

    // ---------------------------- 外环角度PID参数 (P控制) ----------------------------
    // Roll 外环
    PIDRoll.kp = ROLL_KP;
    PIDRoll.ki = ROLL_KI;
    PIDRoll.kd = ROLL_KD;
    PIDRoll.Integ_LimitHigh = ROLL_INTEG_LIMIT_HIGH;
    PIDRoll.Integ_LimitLow = ROLL_INTEG_LIMIT_LOW;
    PIDRoll.Out_LimitHigh = ROLL_OUT_LIMIT_HIGH;
    PIDRoll.Out_LimitLow = ROLL_OUT_LIMIT_LOW;

    // Pitch 外环
    PIDPitch.kp = PITCH_KP;
    PIDPitch.ki = PITCH_KI;
    PIDPitch.kd = PITCH_KD;
    PIDPitch.Integ_LimitHigh = PITCH_INTEG_LIMIT_HIGH;
    PIDPitch.Integ_LimitLow = PITCH_INTEG_LIMIT_LOW;
    PIDPitch.Out_LimitHigh = PITCH_OUT_LIMIT_HIGH;
    PIDPitch.Out_LimitLow = PITCH_OUT_LIMIT_LOW;

    // Yaw 外环
    PIDYaw.kp = YAW_KP;
    PIDYaw.ki = YAW_KI;
    PIDYaw.kd = YAW_KD;
    PIDYaw.Integ_LimitHigh = YAW_INTEG_LIMIT_HIGH;
    PIDYaw.Integ_LimitLow = YAW_INTEG_LIMIT_LOW;
    PIDYaw.Out_LimitHigh = YAW_OUT_LIMIT_HIGH;
    PIDYaw.Out_LimitLow = YAW_OUT_LIMIT_LOW;

    // ---------------------------- 微分滤波器初始化 ----------------------------
    // 内环微分滤波器 - 角速度环需要较快的响应
    PT1Filter_InitWithFreq(&PIDVelX.deriv_filter, DERIV_FILTER_VELX, 200);
    PT1Filter_InitWithFreq(&PIDVelY.deriv_filter, DERIV_FILTER_VELY, 200);
    PT1Filter_InitWithFreq(&PIDVelZ.deriv_filter, DERIV_FILTER_VELZ, 200);

    // 外环微分滤波器 - 角度环需要更平滑的控制
    PT1Filter_InitWithFreq(&PIDRoll.deriv_filter, DERIV_FILTER_ROLL, 200);
    PT1Filter_InitWithFreq(&PIDPitch.deriv_filter, DERIV_FILTER_PITCH, 200);
    PT1Filter_InitWithFreq(&PIDYaw.deriv_filter, DERIV_FILTER_YAW, 200);

    // ---------------------------- 高度环PID参数 ----------------------------
    // 外环: 高度误差 → 目标垂直速度
    PIDHeight.kp = HEIGHT_KP;
    PIDHeight.ki = HEIGHT_KI;
    PIDHeight.kd = HEIGHT_KD;
    PIDHeight.Integ_LimitHigh = HEIGHT_INTEG_LIMIT_HIGH;
    PIDHeight.Integ_LimitLow = HEIGHT_INTEG_LIMIT_LOW;
    PIDHeight.Out_LimitHigh = HEIGHT_OUT_LIMIT_HIGH;
    PIDHeight.Out_LimitLow = HEIGHT_OUT_LIMIT_LOW;

    // 内环: 垂直速度误差 → 油门修正量
    PIDVelH.kp = VELH_KP;
    PIDVelH.ki = VELH_KI;
    PIDVelH.kd = VELH_KD;
    PIDVelH.Integ_LimitHigh = VELH_INTEG_LIMIT_HIGH;
    PIDVelH.Integ_LimitLow = VELH_INTEG_LIMIT_LOW;
    PIDVelH.Out_LimitHigh = VELH_OUT_LIMIT_HIGH;
    PIDVelH.Out_LimitLow = VELH_OUT_LIMIT_LOW;

    PT1Filter_InitWithFreq(&PIDHeight.deriv_filter, DERIV_FILTER_HEIGHT, 200);
    PT1Filter_InitWithFreq(&PIDVelH.deriv_filter, DERIV_FILTER_VELH, 200);

    // ---------------------------- 光流定点PID参数 (预留，暂未启用) ----------------------------
    // X轴: 位置外环 + 速度内环
    PIDPosX.kp = POS_KP;
    PIDPosX.ki = POS_KI;
    PIDPosX.kd = POS_KD;
    PIDPosX.Integ_LimitHigh = POS_INTEG_LIMIT_HIGH;
    PIDPosX.Integ_LimitLow = POS_INTEG_LIMIT_LOW;
    PIDPosX.Out_LimitHigh = POS_OUT_LIMIT_HIGH;
    PIDPosX.Out_LimitLow = POS_OUT_LIMIT_LOW;

    PIDPosX_Vel.kp = POS_VELX_KP;
    PIDPosX_Vel.ki = POS_VEL_KI;
    PIDPosX_Vel.kd = POS_VEL_KD;
    PIDPosX_Vel.Integ_LimitHigh = POS_VEL_INTEG_LIMIT_HIGH;
    PIDPosX_Vel.Integ_LimitLow = POS_VEL_INTEG_LIMIT_LOW;
    PIDPosX_Vel.Out_LimitHigh = POS_VEL_OUT_LIMIT_HIGH;
    PIDPosX_Vel.Out_LimitLow = POS_VEL_OUT_LIMIT_LOW;

    // Y轴: 位置外环 + 速度内环
    PIDPosY.kp = POS_KP;
    PIDPosY.ki = POS_KI;
    PIDPosY.kd = POS_KD;
    PIDPosY.Integ_LimitHigh = POS_INTEG_LIMIT_HIGH;
    PIDPosY.Integ_LimitLow = POS_INTEG_LIMIT_LOW;
    PIDPosY.Out_LimitHigh = POS_OUT_LIMIT_HIGH;
    PIDPosY.Out_LimitLow = POS_OUT_LIMIT_LOW;

    PIDPosY_Vel.kp = POS_VELY_KP;
    PIDPosY_Vel.ki = POS_VEL_KI;
    PIDPosY_Vel.kd = POS_VEL_KD;
    PIDPosY_Vel.Integ_LimitHigh = POS_VEL_INTEG_LIMIT_HIGH;
    PIDPosY_Vel.Integ_LimitLow = POS_VEL_INTEG_LIMIT_LOW;
    PIDPosY_Vel.Out_LimitHigh = POS_VEL_OUT_LIMIT_HIGH;
    PIDPosY_Vel.Out_LimitLow = POS_VEL_OUT_LIMIT_LOW;
    
    PT1Filter_InitWithFreq(&PIDPosX.deriv_filter, DERIV_FILTER_POS, 200);
    PT1Filter_InitWithFreq(&PIDPosX_Vel.deriv_filter, DERIV_FILTER_POS_VEL, 200);
    PT1Filter_InitWithFreq(&PIDPosY.deriv_filter, DERIV_FILTER_POS, 200);
    PT1Filter_InitWithFreq(&PIDPosY_Vel.deriv_filter, DERIV_FILTER_POS_VEL, 200);

    PT1Filter_InitWithFreq(&anti_gravity_filter, ANTI_GRAVITY_FILTER_FREQ, 200);

    
}

// =============================================================================
// Anti-Gravity 油门前馈更新
// =============================================================================
// THR是PWM占空比(3000~6500)，归一化到0~1后按BF算法计算I项boost
// =============================================================================
void anti_gravity_update(float throttle, float dt)
{
    float t = (throttle - MOTOR_MIN_DUTY) / (MOTOR_MAX_DUTY - MOTOR_MIN_DUTY);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    static float prev_t = 0.0f;
    float raw = fabsf(t - prev_t) / dt;
    uint8_t is_increasing = (t > prev_t);
    prev_t = t;

    float inv = 1.0f - t;
    float boost = raw * inv * inv;
    if (is_increasing) {
        boost *= inv * 0.5f;
    }

    boost = PT1Filter_Apply(&anti_gravity_filter, boost);
    anti_gravity_boost = boost * ANTI_GRAVITY_GAIN;
}

// =============================================================================
// PID数据复位
// =============================================================================
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
        pid[i]->target = 0;
        pid[i]->integ = 0;
        pid[i]->out = 0;
    }
}

// =============================================================================
// PID计算 - 误差微分 (适用于内环角速度控制)
// =============================================================================
void PID_Update(_PID_param_st *pid, float target, float measured, const float dt)
{
    if(dt <= 0) return;

    pid->target = target;
    pid->measured = measured;
    pid->error = pid->target - pid->measured;

    // 误差微分 + PT1滤波
    float raw_deriv = -(pid->error - pid->last_error) / dt;
    pid->deriv = PT1Filter_Apply(&pid->deriv_filter, raw_deriv);

    // 积分累加
    pid->integ += pid->ki * pid->error * dt * (1.0f + anti_gravity_boost);
    pid->integ = LIMIT(pid->integ, pid->Integ_LimitLow, pid->Integ_LimitHigh);

    // PID输出
    pid->out = pid->kp * pid->error + pid->integ;
    pid->out += pid->kd * pid->deriv;

    // 输出限幅
    pid->out = LIMIT(pid->out, pid->Out_LimitLow, pid->Out_LimitHigh);

    // 更新历史值
    pid->last_error = pid->error;
}

// =============================================================================
// PID计算 - 测量值微分 (适用于外环角度控制，微分先行)
// =============================================================================
void PID_Update_d_measure(_PID_param_st *pid, float target, float measured, const float dt)
{
    if(dt <= 0) return;

    pid->target = target;
    pid->measured = measured;
    pid->error = pid->target - pid->measured;

    // 测量值微分 + PT1滤波 (微分先行，对噪声不敏感)
    float raw_deriv = -(measured - pid->last_measured) / dt;
    pid->deriv = PT1Filter_Apply(&pid->deriv_filter, raw_deriv);

    // 积分累加
    pid->integ += pid->error * dt;
    pid->integ = LIMIT(pid->integ, pid->Integ_LimitLow, pid->Integ_LimitHigh);

    // PID输出
    pid->out = pid->kp * pid->error + pid->ki * pid->integ + pid->kd * pid->deriv;
    
    // 输出限幅
    pid->out = LIMIT(pid->out, pid->Out_LimitLow, pid->Out_LimitHigh);
    
    // 更新历史值
    pid->last_error = pid->error;
    pid->last_measured = measured;
}

// =============================================================================
// 角度差计算 - 处理角度环绕问题
// =============================================================================
static float angle_diff_180(float target, float measured)
{
    float diff = target - measured;
    while(diff > 180.0f)  diff -= 360.0f;
    while(diff < -180.0f) diff += 360.0f;
    return diff;
}

// =============================================================================
// Yaw专用PID - 自动处理角度环绕
// =============================================================================
void PID_Update_Yaw(_PID_param_st *pid, float target, float measured, const float dt)
{
    if(dt <= 0) return;

    pid->target = target;
    pid->measured = measured;

    // 最短路径误差
    pid->error = angle_diff_180(target, measured);

    // 误差微分
    pid->deriv = -(pid->error - pid->last_error) / dt;

    // 积分累加
    pid->integ += pid->ki * pid->error * dt * (1.0f + anti_gravity_boost);
    pid->integ = LIMIT(pid->integ, pid->Integ_LimitLow, pid->Integ_LimitHigh);

    // PID输出
    pid->out = pid->kp * pid->error + pid->integ + pid->kd * pid->deriv;

    // 输出限幅
    pid->out = LIMIT(pid->out, pid->Out_LimitLow, pid->Out_LimitHigh);

    // 更新历史值
    pid->last_error = pid->error;
}
//{
//    uint8_t i;
//    for(i = 0; i < len; i++)
//    {        
//        pid[i]->error = 0;
//        pid[i]->last_error = 0;
//        pid[i]->integ = 0;
//        pid[i]->out = 0;
//        
////        pid[i]->measured = 0;
////        pid[i]->last_measured = 0;
////        
////        pid[i]->deriv = 0;
////        pid[i]->last_deriv = 0;
//    }
//}