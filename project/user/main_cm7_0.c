/*********************************************************************************************************************
* CYT4BB Opensourec Library
* Copyright (c) 2022 SEEKFREE
*
* 文件名称          main_cm7_0
* 模式说明          正常业务模式
*                   KEY_1: 缓启动  KEY_2: 发送PID指令  KEY_3: 停止
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "init.h"
#include "key_task.h"
#include "HC06_Driver.h"
#include "inertial_nav.h"
#include "test.h"

// **************************** 代码区域 ****************************

extern volatile uint8_t send_vel_flag;

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    debug_init();
    Init_all();
    KeyTask_Init();
//    pwm_init(MotorL1_Pwm, 10000, 400);
    while(true)
    {
        KeyTask_Handler();
        InertialNav_Update();
        HC06_Task();

        // ISR每10ms置位 → 主循环消费: 发速度前馈 + 打印无人机数据
        if (send_vel_flag) {
            send_vel_flag = 0;
            // 实测车身速度: 编码器脉冲→麦轮运动学→m/s
            float s1=motor_L1.encoder_speed, s2=motor_L2.encoder_speed;
            float s3=motor_R1.encoder_speed, s4=motor_R2.encoder_speed;
            float vx_enc = ( s1+ s2+ s3+ s4)*0.25f;   // 前进m/s
            float vy_enc = (-s1+ s2+ s3- s4)*0.25f;   // 左移为正
            float vx_act = EncoderPulses_To_LinearVelocity((int)vx_enc);
            float vy_act = EncoderPulses_To_LinearVelocity((int)vy_enc);
            HC06_SendVelocity(vx_act, vy_act);
        }
    }
}
