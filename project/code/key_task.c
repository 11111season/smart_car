/*********************************************************************************************************************
* CYT4BB Opensourec Library
* Copyright (c) 2022 SEEKFREE
*
* 文件名称          key_task
* 功能描述          按键业务逻辑 + IMU就绪自动启动
*                   IMU零漂完成 → 自动使能角度环+速度环
*                   KEY_1: 发送无人机指令1
*                   KEY_2: 发送无人机指令2
*                   KEY_3: 停止 (关PID + 电机刹车 + 发指令3)
*
* 修改记录
* 2026-08-07        按键处理整体移交 App_Menu 菜单 (重构控制链路)
*                   KeyTask_Handler 内按键逻辑用 #if 0 关闭, 由菜单统一收发任务
********************************************************************************************************************/

#include "key_task.h"
#include "motor.h"
#include "My_imu660ra.h"
#include "HC06_Driver.h"
#include "inertial_nav.h"
#include "QMC5883L.h"

//static uint8_t pid_started = 0;   // 2026-08-07: 按键逻辑关闭后无读取方, 注释避免"set but never used"警告

void KeyTask_Init(void)
{
    // 2026-08-09 下地实测惯导: 恢复闭环 (架高验证时注释的PID使能全部恢复)
    Motor_Enable_PID(1);
    PID_Reset(&angle_pid_yaw);   PID_Enable(&angle_pid_yaw, 1);
    PID_Reset(&angle_pid_gyro);  PID_Enable(&angle_pid_gyro, 1);
    My_Imu660ra_ResetYaw();
    MagYaw_Reset();   // 磁力计融合零点: 当前方向=0° (记录/导航统一坐标系)
    angle_target = 0.0f;
    target_vx  = 0.0f;
    target_vy  = 0.0f;
    //pid_started = 1;              // 2026-08-07: 同 pid_started 变量, 一并注释
}

void KeyTask_Handler(void)
{
#if MAG_CALIB_MODE && MAG_CALIB_MOTOR_TEST == 1
    // 电机磁场干扰测试模式: KEY4 开始/结束 PWM缓变 (菜单控制链路下, 测试需要独立按键入口)
    qmc5883l_motor_test_key_handler();
    return;
#elif MAG_CALIB_MODE && MAG_CALIB_MOTOR_TEST == 0
    // 椭圆标定模式: KEY4 开始/结束采集 (菜单控制链路下, 标定需要独立按键入口)
    qmc5883l_calibration_key_handler();
    return;
#endif
#if 0
    // 2026-08-07: 按键逻辑整体关闭, 控制链路重构到 App_Menu 菜单, 所有按键由菜单统一处理
    // 2026-08-08: InertialNav_KeyHandler 已删除 (KEY_4 记录流程迁移到菜单 STATE_INR_WP_START/REC)
    // 恢复方法: 把下面 #if 0 改成 #if 1, 并和 App_Menu 协商好按键分工 (不要两边同时处理)
    // 注意: key_scanner() 在 cm7_0_isr.c 的 PIT 中断里调用, 与本函数无关, 不受影响
    if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
    {
        HC06_SendDroneCmd(1);
        key_clear_state(KEY_1);
    }

    if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
    {
        HC06_SendDroneCmd(2);
        key_clear_state(KEY_2);
    }

#if MAG_CALIB_MODE && MAG_CALIB_MOTOR_TEST == 1
    // 电机磁场干扰测试: KEY4 开始/停止 PWM缓变
    qmc5883l_motor_test_key_handler();
#elif MAG_CALIB_MODE && MAG_CALIB_MOTOR_TEST == 2
    // 航向验证模式: KEY4 无特殊功能
#elif MAG_CALIB_MODE
    // 校准模式: KEY4 开始/结束磁力计采集 (慢速自转≥2圈)
    qmc5883l_calibration_key_handler();
#else
    // InertialNav_KeyHandler();   // 2026-08-08 已删除: 记录流程迁移到菜单, 此处保留注释占位
#endif

    if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
    {
        Motor_Enable_PID(0);
        PID_Enable(&angle_pid_yaw, 0);
        PID_Enable(&angle_pid_gyro, 0);
        pid_started = 0;

        Motor_SetSpeed(&motor_L1, 0, MotorL1_Turn, MotorL1_Pwm);
        Motor_SetSpeed(&motor_L2, 0, MotorL2_Turn, MotorL2_Pwm);
        Motor_SetSpeed(&motor_R1, 0, MotorR1_Turn, MotorR1_Pwm);
        Motor_SetSpeed(&motor_R2, 0, MotorR2_Turn, MotorR2_Pwm);

        target_vx  = 0.0f;
        target_vy  = 0.0f;

        HC06_SendDroneCmd(3);
        key_clear_state(KEY_3);
    }
#endif
}
