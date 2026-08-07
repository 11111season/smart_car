/*********************************************************************************************************************
* CYT4BB Opensourec Library
* Copyright (c) 2022 SEEKFREE
*
* 文件名称          test
* 功能描述          测试函数 + 单信标记录与导航
********************************************************************************************************************/

#include "test.h"
#include "inertial_nav.h"   // fused_yaw

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

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       Mag_Yaw_Verify
// 函数描述       连续打印三个航向角, 用于验证椭圆校准效果
//                每行: 磁力计绝对角, 融合角, 陀螺积分角
//                转动时观察三者增量是否一致, 磁力计是否跟随
// 调用位置       主循环连续调用(限频打印)
//-------------------------------------------------------------------------------------------------------------------
void Mag_Yaw_Verify(void)
{
    static uint64_t last_t = 0;
    if (time_us - last_t < 50000) return;   // 50ms打印一次
    last_t = time_us;

    qmc5883l_get_all();
    printf("Y:%.1f,F:%.1f,G:%.1f,mx:%d,my:%d\n",
           qmc5883l_heading,   // 磁力计绝对航向 (0-360)
           fused_yaw,          // 磁力计+陀螺互补融合 (相对零点)
           My_Imu660ra_GetYaw(), // 陀螺纯积分 (含零漂补偿)
           qmc5883l_mag_x,     // 原始X counts (抓尖峰诊断用)
           qmc5883l_mag_y      // 原始Y counts (抓尖峰诊断用)
    );
}

//-------------------------------------------------------------------------------------------------------------------
// 闭环正方形测试: 锁定0度角, 右→后→左→前 四边平移
// 边长 SQ_SIDE_M, 编码器里程判到达, PID位置闭环控速
// 调用: Square_ClosedLoop_Init() 一次, 然后 Square_ClosedLoop_Update() 在10ms ISR中
//-------------------------------------------------------------------------------------------------------------------
#define SQ_SIDE_M  0.5f   // 边长 0.5m

static enum { SQ_IDLE, SQ_RUN, SQ_DONE } sq_cl_state = SQ_IDLE;
static uint8_t  sq_cl_leg = 0;       // 当前边: 0=右 1=后 2=左 3=前
static int32_t  sq_cl_enc0[4];       // 每段起点编码器
static PID      sq_pid_vx, sq_pid_vy;

// 编码器平均增量 → 米
static float sq_enc2m(int32_t e0[4]) {
    int32_t d = 0;
    d += abs(motor_L1.total_encoder - e0[0]);
    d += abs(motor_L2.total_encoder - e0[1]);
    d += abs(motor_R1.total_encoder - e0[2]);
    d += abs(motor_R2.total_encoder - e0[3]);
    return (float)(d/4) / ENCODER_PULSES_PER_REV * ENCODER_GEAR_RATIO * WHEEL_CIRCUMFERENCE;
}

void Square_ClosedLoop_Init(void)
{
    // 两轴位置PID: Kp=1.0, 限幅±0.12m/s
    PID_Init(&sq_pid_vx, 1.0f, 0.0f, 0.0f);
    PID_SetLimit(&sq_pid_vx, 0.12f, -0.12f);
    PID_SetIntegralLimit(&sq_pid_vx, 10.0f);
    PID_Enable(&sq_pid_vx, 1);

    PID_Init(&sq_pid_vy, 1.0f, 0.0f, 0.0f);
    PID_SetLimit(&sq_pid_vy, 0.12f, -0.12f);
    PID_SetIntegralLimit(&sq_pid_vy, 10.0f);
    PID_Enable(&sq_pid_vy, 1);

    // 记录起点, 锁0度
    sq_cl_enc0[0] = motor_L1.total_encoder;
    sq_cl_enc0[1] = motor_L2.total_encoder;
    sq_cl_enc0[2] = motor_R1.total_encoder;
    sq_cl_enc0[3] = motor_R2.total_encoder;
    My_Imu660ra_ResetYaw();
    Motor_Enable_PID(1);
    PID_Reset(&angle_pid_yaw);   PID_Enable(&angle_pid_yaw, 1);
    PID_Reset(&angle_pid_gyro);  PID_Enable(&angle_pid_gyro, 1);
    angle_target = 0.0f;

    sq_cl_leg  = 0;
    sq_cl_state = SQ_RUN;
    printf("SQ: start, side=%.2fm\n", SQ_SIDE_M);
}

void Square_ClosedLoop_Update(void)
{
    if (sq_cl_state != SQ_RUN) return;

    // 当前已走距离 (米)
    float traveled = sq_enc2m(sq_cl_enc0);

    // 到达当前边的终点 → 切换下一段
    if (SQ_SIDE_M - traveled <= 0.02f) {
        sq_cl_leg++;
        if (sq_cl_leg >= 4) {
            sq_cl_state = SQ_DONE;
            target_vx = 0.0f; target_vy = 0.0f;
            printf("SQ: done!\n");
            return;
        }
        // 重置编码器起点
        sq_cl_enc0[0] = motor_L1.total_encoder;
        sq_cl_enc0[1] = motor_L2.total_encoder;
        sq_cl_enc0[2] = motor_R1.total_encoder;
        sq_cl_enc0[3] = motor_R2.total_encoder;
        PID_Reset(&sq_pid_vx);
        PID_Reset(&sq_pid_vy);
        printf("SQ: leg %d start\n", sq_cl_leg);
    }

    // 每段只有一条轴在动, 另一轴目标=反馈=0保持静止
    float vx = 0.0f, vy = 0.0f;
    switch (sq_cl_leg) {
        case 0: // 右移: Y+, X 不动
            PID_SetTarget(&sq_pid_vy,  SQ_SIDE_M);  vy = PID_Calculate(&sq_pid_vy, traveled);
            PID_SetTarget(&sq_pid_vx, 0.0f);        vx = PID_Calculate(&sq_pid_vx, 0.0f);
            break;
        case 1: // 后退: X-, Y 不动
            PID_SetTarget(&sq_pid_vx, -SQ_SIDE_M);  vx = PID_Calculate(&sq_pid_vx, traveled);
            PID_SetTarget(&sq_pid_vy, 0.0f);        vy = PID_Calculate(&sq_pid_vy, 0.0f);
            break;
        case 2: // 左移: Y-, X 不动
            PID_SetTarget(&sq_pid_vy, -SQ_SIDE_M);  vy = PID_Calculate(&sq_pid_vy, traveled);
            PID_SetTarget(&sq_pid_vx, 0.0f);        vx = PID_Calculate(&sq_pid_vx, 0.0f);
            break;
        case 3: // 前进: X+, Y 不动
            PID_SetTarget(&sq_pid_vx,  SQ_SIDE_M);  vx = PID_Calculate(&sq_pid_vx, traveled);
            PID_SetTarget(&sq_pid_vy, 0.0f);        vy = PID_Calculate(&sq_pid_vy, 0.0f);
            break;
    }
    target_vx = vx;
    target_vy = vy;
}

//-------------------------------------------------------------------------------------------------------------------
// 闭环圆形测试: 恒定切向速度 + 恒定角速度, yaw转够360°停
// 调用: Circle_ClosedLoop_Init(R, dir) 一次, 然后 Circle_ClosedLoop_Update() 在10ms ISR中
// R 米, dir=1逆时针 dir=-1顺时针, 角速度=v/R 自动算
//-------------------------------------------------------------------------------------------------------------------
#define CIRCLE_SPEED  0.12f   // 切向线速度 m/s

static enum { CIR_IDLE, CIR_RUN, CIR_DONE } cir_state = CIR_IDLE;
static float cir_omega = 0.0f;     // 计算得出的目标角速度 rad/s
static float cir_yaw0  = 0.0f;     // 起始yaw (度)

void Circle_ClosedLoop_Init(float radius_m, int dir)
{
    // omega = v / R, 限幅防止过快转向
    cir_omega = CIRCLE_SPEED / radius_m;
    if (dir < 0) cir_omega = -cir_omega;
    if (fabsf(cir_omega) > 1.5f) cir_omega = (cir_omega > 0) ? 1.5f : -1.5f;

    cir_yaw0 = My_Imu660ra_GetYaw();
    My_Imu660ra_ResetYaw();        // 清零, 转满360停
    Motor_Enable_PID(1);
    cir_state = CIR_RUN;
    printf("CIR: R=%.2f omega=%.2f dir=%d\n", radius_m, cir_omega, dir);
}

void Circle_ClosedLoop_Update(void)
{
    if (cir_state != CIR_RUN) return;

    float dyaw = fabsf(My_Imu660ra_GetYaw());
    if (dyaw >= 360.0f) {
        cir_state = CIR_DONE;
        target_vx = 0.0f; target_vy = 0.0f;
        printf("CIR: done! dyaw=%.1f\n", dyaw);
        return;
    }
    // 切向恒速, 角速度恒定 → 圆轨迹
    float omega_rad = cir_omega;
    Mecanum_Move(CIRCLE_SPEED, 0.0f, omega_rad);
}

//-------------------------------------------------------------------------------------------------------------------
// 闭环八字测试: 锁定0度角, 4段对角线平移 (利用麦轮运动学分解编码器到XY)
// 调用: Figure8_ClosedLoop_Init() 一次, 然后 Figure8_ClosedLoop_Update() 在10ms ISR中
//-------------------------------------------------------------------------------------------------------------------
#define F8_SIDE_M  0.5f   // 半对角线长 0.5m (X和Y各走0.5m)

static enum { F8_IDLE, F8_RUN, F8_DONE } f8_cl_state = F8_IDLE;
static uint8_t  f8_cl_leg = 0;
static int32_t  f8_enc0[4];
static PID      f8_pid_vx, f8_pid_vy;

// 利用麦轮运动学: 编码器增量 → X位移(m) + Y位移(m)
// 复用 InertialNav_PosUpdate 的公式: vx=(L1+L2+R1+R2)/4, vy=(L1-L2-R1+R2)/4
static void f8_enc2xy(float *dx, float *dy)
{
    float s1 = (float)(motor_L1.total_encoder - f8_enc0[0]);
    float s2 = (float)(motor_L2.total_encoder - f8_enc0[1]);
    float s3 = (float)(motor_R1.total_encoder - f8_enc0[2]);
    float s4 = (float)(motor_R2.total_encoder - f8_enc0[3]);

    float vx_p = ( s1 + s2 + s3 + s4) * 0.25f;   // 前进为正
    float vy_p = ( s1 - s2 - s3 + s4) * 0.25f;   // 左移为正

    *dx =  vx_p / ENCODER_PULSES_PER_REV * ENCODER_GEAR_RATIO * WHEEL_CIRCUMFERENCE;
    *dy =  vy_p / ENCODER_PULSES_PER_REV * ENCODER_GEAR_RATIO * WHEEL_CIRCUMFERENCE;
}

void Figure8_ClosedLoop_Init(void)
{
    PID_Init(&f8_pid_vx, 1.0f, 0.0f, 0.0f);
    PID_SetLimit(&f8_pid_vx, 0.12f, -0.12f);
    PID_SetIntegralLimit(&f8_pid_vx, 10.0f);
    PID_Enable(&f8_pid_vx, 1);

    PID_Init(&f8_pid_vy, 1.0f, 0.0f, 0.0f);
    PID_SetLimit(&f8_pid_vy, 0.12f, -0.12f);
    PID_SetIntegralLimit(&f8_pid_vy, 10.0f);
    PID_Enable(&f8_pid_vy, 1);

    f8_enc0[0] = motor_L1.total_encoder;
    f8_enc0[1] = motor_L2.total_encoder;
    f8_enc0[2] = motor_R1.total_encoder;
    f8_enc0[3] = motor_R2.total_encoder;
    My_Imu660ra_ResetYaw();
    Motor_Enable_PID(1);
    PID_Reset(&angle_pid_yaw);   PID_Enable(&angle_pid_yaw, 1);
    PID_Reset(&angle_pid_gyro);  PID_Enable(&angle_pid_gyro, 1);
    angle_target = 0.0f;

    f8_cl_leg  = 0;
    f8_cl_state = F8_RUN;
    printf("F8: start, side=%.2fm\n", F8_SIDE_M);
}

void Figure8_ClosedLoop_Update(void)
{
    if (f8_cl_state != F8_RUN) return;

    float tx = 0.0f, ty = 0.0f;
    // 八字4段: 后左→右→前左→右 (和原开环版一致)
    switch (f8_cl_leg) {
        case 0: tx = -F8_SIDE_M; ty = -F8_SIDE_M; break;   // 后左对角线
        case 1: tx =  0.0f;      ty =  F8_SIDE_M; break;   // 纯右移
        case 2: tx =  F8_SIDE_M; ty = -F8_SIDE_M; break;   // 前左对角线
        case 3: tx =  0.0f;      ty =  F8_SIDE_M; break;   // 纯右移
    }

    float traveled_x, traveled_y;
    f8_enc2xy(&traveled_x, &traveled_y);

    // 麦轮分解后直接喂给两轴PID，斜边自动处理
    PID_SetTarget(&f8_pid_vx, tx);
    PID_SetTarget(&f8_pid_vy, ty);
    float vx = PID_Calculate(&f8_pid_vx, traveled_x);
    float vy = PID_Calculate(&f8_pid_vy, traveled_y);
    target_vx = vx;
    target_vy = vy;

    // 到达判断: 两轴误差都 <2cm
    float ex = tx - traveled_x;
    float ey = ty - traveled_y;
    if (fabsf(ex) <= 0.02f && fabsf(ey) <= 0.02f) {
        f8_cl_leg++;
        if (f8_cl_leg >= 4) {
            f8_cl_state = F8_DONE;
            target_vx = 0.0f; target_vy = 0.0f;
            printf("F8: done!\n");
            return;
        }
        f8_enc0[0] = motor_L1.total_encoder;
        f8_enc0[1] = motor_L2.total_encoder;
        f8_enc0[2] = motor_R1.total_encoder;
        f8_enc0[3] = motor_R2.total_encoder;
        PID_Reset(&f8_pid_vx);
        PID_Reset(&f8_pid_vy);
        printf("F8: leg %d start\n", f8_cl_leg);
    }
}