/*********************************************************************************************************************
* CYT4BB Opensourec Library
* Copyright (c) 2022 SEEKFREE
*
* 文件名称          test
* 功能描述          测试函数 + 单信标记录与导航
********************************************************************************************************************/

#include "test.h"

//-------------------------------------------------------------------------------------------------------------------
// 原有测试函数 (保留, 已注释调用)
//-------------------------------------------------------------------------------------------------------------------
static uint8_t angle_test_started = 0;
static uint8_t pid_started = 0;
static uint8_t square_started = 0;
static enum { FWD, RIGHT, BACK, LEFT } square_state = FWD;
static uint64_t square_start_time = 0;
static uint8_t figure8_started = 0;
static enum { STAGE0, STAGE1, STAGE2, STAGE3 } figure8_stage = STAGE0;
static uint64_t figure8_start_time = 0;
int change_flag = 0;

void Angle_Test(float vx,float vy)
{
    if(Start_Pid_Flag == 1 && angle_test_started == 0) {
        My_Imu660ra_ResetYaw();
        Motor_Enable_PID(1);
        PID_Reset(&angle_pid_yaw);
        PID_Enable(&angle_pid_yaw, 1);
        target_vx = vx;
        target_vy = vy;
        angle_test_started = 1;
        Start_Pid_Flag = 0;
    }
    if(Stop_Pid_Flag == 1 && angle_test_started == 1) {
        PID_Enable(&angle_pid_yaw, 0);
        Motor_Enable_PID(0);
        Motor_SetSpeed(&motor_L1,0, MotorL1_Turn, MotorL1_Pwm);
        Motor_SetSpeed(&motor_L2,0, MotorL2_Turn, MotorL2_Pwm);
        Motor_SetSpeed(&motor_R1,0, MotorR1_Turn, MotorR1_Pwm);
        Motor_SetSpeed(&motor_R2,0, MotorR2_Turn, MotorR2_Pwm);
        angle_test_started = 0;
        Stop_Pid_Flag = 0;
        target_vx = 0.0f;
        target_vy = 0.0f;
    }
    if(angle_test_started) {
        printf("Yaw=%.1f, Pitch=%.1f, Roll=%.1f, Omega_des=%.1f\n",
               My_Imu660ra_GetYaw(), My_Imu660ra_GetPitch(),
               My_Imu660ra_GetRoll(), angle_pid_yaw.Output);
    }
}

void Horizon_test(int direction, float target_speed)
{
    if(Start_Pid_Flag == 1 && pid_started == 0) {
        Motor_Enable_PID(1);
        PID_Reset(&angle_pid_yaw);
        PID_Enable(&angle_pid_yaw, 1);
        target_vx = 0.0f;
        target_vy = (direction > 0) ? target_speed : -target_speed;
        pid_started = 1;
        Start_Pid_Flag = 0;
    }
    if(Stop_Pid_Flag == 1 && pid_started == 1) {
        PID_Enable(&angle_pid_yaw, 0);
        Motor_Enable_PID(0);
        Motor_SetSpeed(&motor_L1,0, MotorL1_Turn, MotorL1_Pwm);
        Motor_SetSpeed(&motor_L2,0, MotorL2_Turn, MotorL2_Pwm);
        Motor_SetSpeed(&motor_R1,0, MotorR1_Turn, MotorR1_Pwm);
        Motor_SetSpeed(&motor_R2,0, MotorR2_Turn, MotorR2_Pwm);
        pid_started = 0;
        Stop_Pid_Flag = 0;
        target_vx = 0.0f;
        target_vy = 0.0f;
    }
}

void Square_Test(float speed_mps, uint32_t duration_ms)
{
    if (Start_Pid_Flag == 1 && square_started == 0) {
        Motor_Enable_PID(1);
        PID_Reset(&angle_pid_yaw);
        PID_Enable(&angle_pid_yaw, 1);
        square_state = FWD;
        square_start_time = 0;
        target_vx = speed_mps;
        target_vy = 0.0f;
        square_started = 1;
        Start_Pid_Flag = 0;
    }
    if (Stop_Pid_Flag == 1 && square_started == 1) {
        PID_Enable(&angle_pid_yaw, 0);
        Motor_Enable_PID(0);
        Motor_SetSpeed(&motor_L1,0, MotorL1_Turn, MotorL1_Pwm);
        Motor_SetSpeed(&motor_L2,0, MotorL2_Turn, MotorL2_Pwm);
        Motor_SetSpeed(&motor_R1,0, MotorR1_Turn, MotorR1_Pwm);
        Motor_SetSpeed(&motor_R2,0, MotorR2_Turn, MotorR2_Pwm);
        square_started = 0;
        Stop_Pid_Flag = 0;
        target_vx = 0.0f;
        target_vy = 0.0f;
    }
    if (!square_started) return;
    if (!angle_pid_yaw.Enable) return;
    uint64_t now_ms = time_us / 1000;
    if (square_start_time == 0) { square_start_time = now_ms; return; }
    if (now_ms - square_start_time >= duration_ms) {
        switch (square_state) {
            case FWD:  square_state = RIGHT; target_vx = 0.0f; target_vy = speed_mps; break;
            case RIGHT: square_state = BACK;  target_vx = -speed_mps; target_vy = 0.0f; break;
            case BACK:  square_state = LEFT;  target_vx = 0.0f; target_vy = -speed_mps; break;
            case LEFT:  square_state = FWD;   target_vx = speed_mps; target_vy = 0.0f; break;
        }
        square_start_time = now_ms;
    }
}

void Figure8_Test(float speed_mps, uint32_t duration_ms)
{
    if (Start_Pid_Flag == 1 && figure8_started == 0) {
        Motor_Enable_PID(1);
        motor_L1.pid.Enable = 1; motor_L2.pid.Enable = 1;
        motor_R1.pid.Enable = 1; motor_R2.pid.Enable = 1;
        PID_Reset(&angle_pid_yaw);
        PID_Enable(&angle_pid_yaw, 1);
        target_vx = 0.0f; target_vy = speed_mps;
        figure8_stage = STAGE0; figure8_start_time = 0;
        figure8_started = 1; Start_Pid_Flag = 0;
    }
    if (Stop_Pid_Flag == 1 && figure8_started == 1) {
        PID_Enable(&angle_pid_yaw, 0);
        Motor_Enable_PID(0);
        Motor_SetSpeed(&motor_L1,0, MotorL1_Turn, MotorL1_Pwm);
        Motor_SetSpeed(&motor_L2,0, MotorL2_Turn, MotorL2_Pwm);
        Motor_SetSpeed(&motor_R1,0, MotorR1_Turn, MotorR1_Pwm);
        Motor_SetSpeed(&motor_R2,0, MotorR2_Turn, MotorR2_Pwm);
        figure8_started = 0; Stop_Pid_Flag = 0;
        target_vx = 0.0f; target_vy = 0.0f;
    }
    if (!figure8_started) return;
    if (!angle_pid_yaw.Enable) return;
    uint64_t now_ms = time_us / 1000;
    if (figure8_start_time == 0) { figure8_start_time = now_ms; return; }
    if (now_ms - figure8_start_time >= duration_ms) {
        switch (figure8_stage) {
            case STAGE0: target_vx = -speed_mps; target_vy = -speed_mps; figure8_stage = STAGE1; break;
            case STAGE1: target_vx = 0.0f; target_vy = speed_mps; figure8_stage = STAGE2; break;
            case STAGE2: target_vx = speed_mps; target_vy = -speed_mps; figure8_stage = STAGE3; break;
            case STAGE3: target_vx = 0.0f; target_vy = speed_mps; figure8_stage = STAGE0; break;
        }
        figure8_start_time = now_ms;
    }
}

void IMU660RA_Test(void)
{
    imu660ra_get_acc();
    imu660ra_get_gyro();
    static uint32_t print_cnt = 0;
    if (print_cnt++ % 10 == 0) {
        printf("ax=%d,ay=%d,az=%d\n", imu660ra_acc_x,imu660ra_acc_y,imu660ra_acc_z);
        printf("gx=%d,gy=%d,gz=%d\n", imu660ra_gyro_x,imu660ra_gyro_y,imu660ra_gyro_z);
    }
    system_delay_ms(10);
}

void My_IMU660RA_Test(void)
{
    static uint32_t print_cnt = 0;
    if (print_cnt++ % 10 == 0) {
        printf("%.1f,%.1f,%.1f\n",
               My_Imu660ra_GetYaw(), My_Imu660ra_GetPitch(), My_Imu660ra_GetRoll());
    }
    system_delay_ms(10);
}


/*==================================================== 磁力计测试 ====================================================*/
void Mag_Test(void)
{
    qmc5883l_get_all();
    float imu_yaw = My_Imu660ra_GetYaw();

    if (key_get_state(KEY_3) == KEY_SHORT_PRESS) {
        printf("MAG:%.1f IMU:%.1f raw(%d,%d)\n",
               qmc5883l_heading, imu_yaw,
               qmc5883l_mag_x, qmc5883l_mag_y);
        key_clear_state(KEY_3);
    }
}
