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
********************************************************************************************************************/

#include "key_task.h"
#include "motor.h"
#include "My_imu660ra.h"
#include "HC06_Driver.h"
#include "inertial_nav.h"
#include "QMC5883L.h"

static uint8_t pid_started = 0;

void KeyTask_Init(void)
{
    Motor_Enable_PID(1);
    PID_Reset(&angle_pid_yaw);   PID_Enable(&angle_pid_yaw, 1);
    PID_Reset(&angle_pid_gyro);  PID_Enable(&angle_pid_gyro, 1);
    My_Imu660ra_ResetYaw();
    //MagYaw_Reset();                      // 磁力计零点 (2026-08-07: 磁力计暂关, 用陀螺仪+零漂)
    angle_target = 0.0f;
    target_vx  = 0.0f;
    target_vy  = 0.0f;
    pid_started = 1;
}

void KeyTask_Handler(void)
{
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
    InertialNav_KeyHandler();
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
}
