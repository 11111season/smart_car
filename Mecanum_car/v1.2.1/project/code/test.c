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

/*==================================================== 单信标记录与导航 ====================================================*/
// 编码器脉冲 -> 米 (含实测校准系数, 1m地砖标定)
#define BCN_ENC_CALIB  4.17f    // 实测: 实际距离 / 编码器计算距离 = 1m / 0.24m
static float bcn_enc2m(int32_t p) {
    return (float)p / ENCODER_PULSES_PER_REV * ENCODER_GEAR_RATIO * WHEEL_CIRCUMFERENCE * BCN_ENC_CALIB;
}

// 四轮脉冲差绝对值取平均
static int32_t bcn_enc_delta(int32_t e0[4]) {
    int32_t d1 = motor_L1.total_encoder - e0[0];
    int32_t d2 = motor_L2.total_encoder - e0[1];
    int32_t d3 = motor_R1.total_encoder - e0[2];
    int32_t d4 = motor_R2.total_encoder - e0[3];
    return (int32_t)((fabsf((float)d1)+fabsf((float)d2)+fabsf((float)d3)+fabsf((float)d4))*0.25f);
}

// 状态: IDLE -> RECORD -> DONE -> GO
enum { BCN_IDLE, BCN_RECORD, BCN_DONE, BCN_GO };
static uint8_t  bcn_state    = BCN_IDLE;
static float    bcn_yaw      = 0.0f;   // 记录的平均yaw(度)
static float    bcn_dist_m   = 0.0f;   // 记录的距离(m)
static int32_t  bcn_enc0[4]  = {0};    // 编码器快照
static uint16_t bcn_debounce = 0;
static float    bcn_rem_i   = 0.0f;   // 导航积分残差
static uint8_t  bcn_go_first = 1;     // 导航首次标志

// ISR读取: 导航期间的目标速度与角度
uint8_t bcn_nav_on   = 0;
float   bcn_nav_vx   = 0.0f;
float   bcn_nav_vy   = 0.0f;
float   bcn_nav_angle = 0.0f;

//-------------------------------------------------------------------------------------------------------------------
// KEY_4 按键处理 (在 key_task 的 Handler 中调用)
// 第1次按: 开始记录 (编码器+yaw), 不碰PID
// 第2次按: 结束记录, 输出距离和平均yaw
// 第3次按: 启动PID(锁0度), 走向信标
// 第4次按: 强制停止
//-------------------------------------------------------------------------------------------------------------------
void BeaconSingle_KeyHandler(void)
{
    if (key_get_state(KEY_4) != KEY_SHORT_PRESS) return;
    if (bcn_debounce > 0) return;          // 消抖中, 忽略
    bcn_debounce = 50;                     // 500ms消抖 (主循环~10ms周期)

    switch (bcn_state) {

    case BCN_IDLE:
        // 第1次: 开始记录编码器和yaw (PID保持之前的状态, 不动)
        bcn_enc0[0] = motor_L1.total_encoder;
        bcn_enc0[1] = motor_L2.total_encoder;
        bcn_enc0[2] = motor_R1.total_encoder;
        bcn_enc0[3] = motor_R2.total_encoder;
        bcn_yaw = My_Imu660ra_GetYaw();       // 直接取当前yaw, 不平均
        bcn_state = BCN_RECORD;
        printf("BCN: recording start, yaw=%.1f\n", bcn_yaw);
        break;

    case BCN_RECORD:
        // 第2次: 结束记录, 计算结果
        {
            int32_t avg = bcn_enc_delta(bcn_enc0);
            bcn_dist_m = bcn_enc2m(avg);
            bcn_state = BCN_DONE;
            bcn_nav_on = 0;
            printf("BCN: dist=%.2fm, yaw=%.1f deg\n", bcn_dist_m, bcn_yaw);
        }
        break;

    case BCN_DONE:
        // 第3次: 启动PID, 锁绝对0度, 斜向走去信标
        My_Imu660ra_ResetYaw();
        angle_target = 0.0f;
        target_vx = 0.0f;
        target_vy = 0.0f;
        Motor_Enable_PID(1);
        PID_Reset(&angle_pid_yaw);   PID_Enable(&angle_pid_yaw, 1);
        PID_Reset(&angle_pid_gyro);  PID_Enable(&angle_pid_gyro, 1);
        // 导航起点快照
        bcn_enc0[0] = motor_L1.total_encoder;
        bcn_enc0[1] = motor_L2.total_encoder;
        bcn_enc0[2] = motor_R1.total_encoder;
        bcn_enc0[3] = motor_R2.total_encoder;
        bcn_state = BCN_GO;
        printf("BCN: GO dist=%.2fm yaw=%.1f\n", bcn_dist_m, bcn_yaw);
        break;

    case BCN_GO:
        // 第4次: 强制停止
        Motor_Enable_PID(0);
        PID_Enable(&angle_pid_yaw, 0);
        PID_Enable(&angle_pid_gyro, 0);
        target_vx = 0.0f; target_vy = 0.0f;
        bcn_nav_on = 0;
        bcn_rem_i = 0.0f; bcn_go_first = 1;
        bcn_state = BCN_IDLE;
        printf("BCN: force stop\n");
        break;
    }
    key_clear_state(KEY_4);    // 消费本次按键, 避免重复触发
}

//-------------------------------------------------------------------------------------------------------------------
// 主循环调用: 记录时累加yaw, 导航时更新目标速度
//-------------------------------------------------------------------------------------------------------------------
void BeaconSingle_Test(void)
{
    // 消抖计数递减
    if (bcn_debounce > 0) bcn_debounce--;

    // 导航期间: 计算剩余距离, 生成斜向速度
    if (bcn_state != BCN_GO) {
        bcn_nav_on = 0;
        return;
    }

    int32_t traveled = bcn_enc_delta(bcn_enc0);
    float   traveled_m = bcn_enc2m(traveled);
    float   remain = bcn_dist_m - traveled_m;

    if (remain <= 0.03f) {
        // 到达, 停车
        bcn_nav_vx = 0.0f;
        bcn_nav_vy = 0.0f;
        bcn_nav_angle = 0.0f;
        bcn_nav_on = 0;
        target_vx = 0.0f; target_vy = 0.0f;
        bcn_rem_i = 0.0f; bcn_go_first = 1;   // 重置导航积分
        bcn_state = BCN_DONE;
        bcn_debounce = 50;
        printf("BCN: arrived! %.2fm\n", traveled_m);
        return;
    }

    // 导航首次进入时清零积分
    if (bcn_go_first) { bcn_rem_i = 0.0f; bcn_go_first = 0; }
    bcn_rem_i += remain * 0.01f;
    if (bcn_rem_i > 0.15f) bcn_rem_i = 0.15f;
    if (bcn_rem_i < -0.15f) bcn_rem_i = -0.15f;
    float spd = 1.5f * remain + bcn_rem_i;
    if (spd > 0.3f) spd = 0.3f;
    if (spd < 0.08f && remain > 0.02f) spd = 0.08f;

    // 用记录的yaw分解为vx/vy, 角度环锁0度不变
    float rad = bcn_yaw * (PI / 180.0f);
    bcn_nav_vx   = spd * cosf(rad);
    bcn_nav_vy   = spd * sinf(rad);
    bcn_nav_angle = 0.0f;
    bcn_nav_on   = 1;
}
