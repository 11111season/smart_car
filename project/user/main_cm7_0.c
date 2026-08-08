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
#include "App_lora3a22.h"
#include "QMC5883L.h"   // 磁力计电机干扰测试task
#include "App_Menu.h" // 菜单显示

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
        //W25Q64_Test();               // 掉电存储通路测试 (写读回比对). 掉电持久化测试期间注释 — 它每轮会先擦扇区0, 会把要验证的数据冲掉!
        //W25Q64_Persist_Test();       // 2026-08-08: 存储模块已接管 (w25q64_storage 两个分区), 且0x000000现为航点地图区, 旧测试读它会误报
        App_Menu_Task();               // 菜单显示任务
        //HC06_Task();                 // 485/蓝牙接收暂用 (LoRa模拟无人机期间注释)
#if !MAG_CALIB_MODE
        App_Lora_Task();               // LoRa遥控器模拟无人机 (校准期间关闭, 避免污染磁力计数据)

        // 导航模式调试: 限频打印 陀螺积分航向 (50ms) — 2026-08-07 注释: 干扰遥控S4实测
        // 2026-08-07: 磁力计融合暂关, 用陀螺仪+零漂学习, 故只打陀螺yaw
        // {
        //     static uint64_t dbg_t = 0;
        //     if (time_us - dbg_t >= 50000) {
        //         dbg_t = time_us;
        //         printf("G:%.1f\n", My_Imu660ra_GetYaw());   // 陀螺积分 (含零漂补偿)
        //     }
        // }
#endif
#if MAG_CALIB_MODE && MAG_CALIB_MOTOR_TEST == 1
        qmc5883l_motor_test_task();    // 电机磁场干扰测试: 消费ISR标志并打印
#elif MAG_CALIB_MODE && MAG_CALIB_MOTOR_TEST == 2
        Mag_Yaw_Verify();              // 航向验证: 连续打印三路yaw
#endif

        // 发送侧暂缓 (LoRa占用UART_1, 避免与接收冲突)
        // if (send_vel_flag) {
        //     send_vel_flag = 0;
        //     float s1=motor_L1.encoder_speed, s2=motor_L2.encoder_speed;
        //     float s3=motor_R1.encoder_speed, s4=motor_R2.encoder_speed;
        //     float vx_enc = ( s1+ s2+ s3+ s4)*0.25f;
        //     float vy_enc = (-s1+ s2+ s3- s4)*0.25f;
        //     float vx_act = EncoderPulses_To_LinearVelocity((int)vx_enc);
        //     float vy_act = EncoderPulses_To_LinearVelocity((int)vy_enc);
        //     HC06_SendVelocity(vx_act, vy_act);
        // }
    }
}
