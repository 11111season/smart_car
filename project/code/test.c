/*********************************************************************************************************************
* CYT4BB Opensourec Library
* Copyright (c) 2022 SEEKFREE
*
* 文件名称          test
* 功能描述          测试函数 + 单信标记录与导航
********************************************************************************************************************/

#include "test.h"
#include "inertial_nav.h"   // fused_yaw
#include "HC06_Driver.h"    // 2026-08-15 链路测试: HC06_Init / HC06_Task

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
    // D = 磁航向Y − 陀螺航向G (±180环绕): 静止时反映安装方向固定偏差(应恒定),
    // 转动中D的波动 = 磁力计受扰/架体晃动幅度 (2026-08-09架高验证新增)
    float g = My_Imu660ra_GetYaw();
    float d = qmc5883l_heading - g;
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    printf("Y:%.1f,F:%.1f,G:%.1f,D:%+.1f,mx:%d,my:%d\n",
           qmc5883l_heading,   // 磁力计绝对航向 (0-360)
           fused_yaw,          // 磁力计+陀螺互补融合 (相对零点)
           g,                  // 陀螺纯积分 (含零漂补偿)
           d,                  // 磁-陀偏差: 固定偏差=安装关系, 波动=干扰
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

//-------------------------------------------------------------------------------------------------------------------
// W25Q64 掉电存储测试通路
// 1. 读 JEDEC ID (判断硬件通不通)
// 2. 擦除扇区0 (4KB)
// 3. 写入 256 字节递增模式
// 4. 读回比对
// 结果打印到默认串口 (UART_0, 115200)
// 调用位置: 主循环连续调用, 内部用 done 标志只测一轮 (避免反复擦写损耗 Flash)
//-------------------------------------------------------------------------------------------------------------------
#define W25Q64_TEST_ADDR    (0x000000)   // 测试扇区起始地址 (扇区0)

void W25Q64_Test(void)
{
    static uint8 done = 0;
    static uint8 state = 0;
    static uint32 test_addr = W25Q64_TEST_ADDR;
    // 注意: 状态机跨多次主循环调用, 缓冲区必须 static, 否则比对时是未初始化的新栈数组
    static uint8 tx_buf[W25Q64_PAGE_SIZE];
    static uint8 rx_buf[W25Q64_PAGE_SIZE];

    if (done) return;   // 只测一轮

    // 上电先确认ID正常 (init.c 已打印过, 这里再确认一次当前值)
    if (w25q64_chip_id == 0 || (w25q64_chip_id & 0xFFFFFF) == 0xFFFFFF)
    {
        printf("W25Q64: no device (ID=0x%06lX)\n", (unsigned long)w25q64_chip_id);
        done = 1;
        return;
    }

    switch (state)
    {
        case 0:  // 擦除扇区
            printf("W25Q64: erase sector @0x%06lX...\n", (unsigned long)test_addr);
            w25q64_sector_erase(test_addr);
            state = 1;
            break;

        case 1:  // 填充数据并写入
            for (uint16 i = 0; i < W25Q64_PAGE_SIZE; i++)
            {
                tx_buf[i] = (uint8)i;   // 递增模式: 0x00,0x01,...,0xFF
            }
            printf("W25Q64: write %d bytes...\n", W25Q64_PAGE_SIZE);
            w25q64_write(tx_buf, test_addr, W25Q64_PAGE_SIZE);
            state = 2;
            break;

        case 2:  // 读回
            w25q64_read(rx_buf, test_addr, W25Q64_PAGE_SIZE);
            state = 3;
            break;

        case 3:  // 比对
        {
            uint8 pass = 1;
            for (uint16 i = 0; i < W25Q64_PAGE_SIZE; i++)
            {
                if (tx_buf[i] != rx_buf[i])
                {
                    if (pass) printf("W25Q64: FAIL @0x%04X exp=0x%02X got=0x%02X\n",
                                     i, tx_buf[i], rx_buf[i]);
                    pass = 0;
                    break;
                }
            }
            if (pass)
            {
                printf("W25Q64: PASS, %d bytes @0x%06lX verified\n",
                       W25Q64_PAGE_SIZE, (unsigned long)test_addr);
            }
            else
            {
                printf("W25Q64: FAIL (see above)\n");
            }
            done = 1;   // 测试结束
            break;
        }
        default:
            state = 0;
            break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// W25Q64 掉电持久化测试 (验证 NorFlash 掉电不丢数据)
// 流程:  写入模式写入 "Hello W25Q64!" → 掉电 → 重启后把 W25Q64_PERSIST_WRITE 改为 0
//        重新编译烧录 = 读取模式, 验证能原样读回 → 掉电存储打通
// 切换:  改下面 W25Q64_PERSIST_WRITE 宏即可 (1=写, 0=读), 无需动 main.c
// 调用:  主循环连续调用, done 标志只执行一轮 (避免反复擦写损耗 Flash)
// 注意:  读取验证前必须禁用 W25Q64_Test() — 它会先擦除扇区0, 把要验证的数据冲掉!
//-------------------------------------------------------------------------------------------------------------------
#define W25Q64_PERSIST_ADDR     (0x000000)          // 存放位置 (扇区0起始)
#define W25Q64_PERSIST_MSG      "Hello W25Q64!"     // 标志性内容 (随便写, 掉电后能原样读回即通过)
#define W25Q64_PERSIST_MAX_LEN  (32)                // 读取缓冲长度 (读 MAX-1 字节, 末尾手动补 \0)
#define W25Q64_PERSIST_WRITE    (0)                 // 1=本次写入; 掉电后改为 0 重新烧录 = 读取验证

void W25Q64_Persist_Test(void)
{
    static uint8 done = 0;
    static uint8 state = 0;
    static uint8 msg[W25Q64_PERSIST_MAX_LEN];

    if (done) return;

    // 上电先确认ID正常
    if (w25q64_chip_id == 0 || (w25q64_chip_id & 0xFFFFFF) == 0xFFFFFF)
    {
        printf("W25Q64: no device (ID=0x%06lX)\n", (unsigned long)w25q64_chip_id);
        done = 1;
        return;
    }

#if W25Q64_PERSIST_WRITE
    // ================= 写入模式 (掉电前运行一次) =================
    switch (state)
    {
        case 0:  // 擦除扇区 (先清干净, 保证能写出可识别内容)
            printf("W25Q64: P-WRITE erase sector @0x%06lX...\n", (unsigned long)W25Q64_PERSIST_ADDR);
            w25q64_sector_erase(W25Q64_PERSIST_ADDR);
            state = 1;
            break;

        case 1:  // 写入字符串 (sizeof 含结尾 \0, memcpy 一并拷入)
        {
            uint8 len = (uint8)sizeof(W25Q64_PERSIST_MSG);
            memcpy(msg, W25Q64_PERSIST_MSG, len);
            printf("W25Q64: P-WRITE \"%s\" (%uB) @0x%06lX...\n",
                   (char*)msg, len, (unsigned long)W25Q64_PERSIST_ADDR);
            w25q64_write(msg, W25Q64_PERSIST_ADDR, len);
            state = 2;
            break;
        }

        case 2:  // 立即读回, 先确认本次写入本身没问题
            w25q64_read(msg, W25Q64_PERSIST_ADDR, W25Q64_PERSIST_MAX_LEN - 1);
            msg[W25Q64_PERSIST_MAX_LEN - 1] = 0;   // 手动补 \0, 防 strcmp 越界
            if (strcmp((char*)msg, W25Q64_PERSIST_MSG) == 0)
            {
                printf("W25Q64: P-WRITE verified OK.\n");
                printf("W25Q64: now POWER OFF -> set W25Q64_PERSIST_WRITE=0 and rebuild -> power on to verify\n");
            }
            else
            {
                printf("W25Q64: P-WRITE verify FAIL (read=\"%s\")\n", (char*)msg);
            }
            done = 1;
            break;

        default:
            state = 0;
            break;
    }
#else
    // ================= 读取模式 (掉电重启后运行, 验证数据还在) =================
    switch (state)
    {
        case 0:  // 读回
            w25q64_read(msg, W25Q64_PERSIST_ADDR, W25Q64_PERSIST_MAX_LEN - 1);
            msg[W25Q64_PERSIST_MAX_LEN - 1] = 0;   // 手动补 \0
            state = 1;
            break;

        case 1:
            // 先查 0xFF (扇区未写入/被擦过) 再做 strcmp, 防止读到无 \0 的脏数据越界
            if (msg[0] == 0xFF)
            {
                printf("W25Q64: P-READ empty (all 0xFF) — nothing written yet?\n");
            }
            else if (strcmp((char*)msg, W25Q64_PERSIST_MSG) == 0)
            {
                printf("W25Q64: P-READ PASS! \"%s\" survived power-off\n", (char*)msg);
            }
            else
            {
                printf("W25Q64: P-READ mismatch, got \"%s\"\n", (char*)msg);
            }
            done = 1;
            break;

        default:
            state = 0;
            break;
    }
#endif
}

//-------------------------------------------------------------------------------------------------------------------
// 零漂测量实验: 悬空架车跑惯导, CSV 抓原始陀螺漂移
// 前提:   cm7_0_isr.c DRIFT_LEARN_ENABLE=0 (禁用 ISR 侧零漂学习, 仅保留 IMU 静态学习)
// 用法:   上电 → 静止几秒 (静态学习收敛) → 菜单发车 (悬空/桌面) → 跑 4~5 分钟
//         Python 抓串口, 过滤以 "DBG," 开头的行即可
// 数据:   每 100ms 一行 (主循环限频, 不在 ISR, 不阻塞中断)
//         DBG,<t_ms>,<yaw>,<z_off>,<err_int>,<comp>,<gz>
//         yaw    = 陀螺积分航向 (发车 ResetYaw 归零, 偏离量=纯漂移)   [核心]
//         z_off  = 在线零偏 gyro_z_offset (运动期应冻结)             [理论验证]
//         err_int= 角度环积分 (comp 原本会吸收的量, 诊断用)           [诊断]
//         comp   = yaw_drift_comp (禁用后应恒为 0, 验证生效)          [诊断]
//         gz     = 滤波前 Z 角速度 (°/s)                             [诊断]
// 调用:   主循环连续调用 (内部门控于 mission_armed + 限频)
//-------------------------------------------------------------------------------------------------------------------
#define DRIFT_PRINT_PERIOD_US  100000UL   // 100ms 一条

void Drift_Measure_Test(void)
{
    static uint64_t last_t = 0;
    if (!mission_armed) return;                          // 仅发车后打印
    if (time_us - last_t < DRIFT_PRINT_PERIOD_US) return;
    last_t = time_us;

    printf("DBG,%lu,%.2f,%.3f,%.1f,%.2f,%.2f\n",
           (unsigned long)(time_us / 1000),
           My_Imu660ra_GetYaw(),        // 陀螺积分航向 (无ISR补偿)
           gyro_z_offset,               // 在线零偏
           angle_pid_yaw.err_int_k_1,   // 角度环积分
           yaw_drift_comp,              // 漂移补偿 (应恒0)
           imu660_gz);                  // Z角速度 (°/s)
}

//-------------------------------------------------------------------------------------------------------------------
// 零漂测量实验: KEY_1 绕过菜单直接发车
// 前提:   App_Menu_Init / App_Menu_Task 已注释 (屏幕未接, 烧录器占用)
// 行为:   按下 KEY_1 → 从 W25Q64 重读地图 (幂等) → Inav_Launch 武装发车
//         mission_armed=1 后 3.5s, 巡逻自动触发 (InertialNav_Update 内部判断)
//         Drift_Measure_Test 同步开始打 DBG
// 调用:   主循环连续调用 (发车后 mission_armed=1, 自动忽略后续按键)
//-------------------------------------------------------------------------------------------------------------------
void Test_QuickLaunch(void)
{
    if (mission_armed) return;                     // 已发车, 忽略后续按键
    if (key_get_state(KEY_1) == KEY_SHORT_PRESS) {
        key_clear_state(KEY_1);
        Inav_LoadMap();                            // 从 W25Q64 重读地图 (幂等)
        if (Inav_Launch()) {
            printf("TEST: KEY1 launch, 3.5s to patrol\n");
        } else {
            printf("TEST: launch denied (map not ready?)\n");
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 2026-08-15 通信链路最小测试: UART1 接收+回显 (只走 HC06 中断路径, 其他子系统全不初始化)
// 前提:   其他逻辑全部注释, 主循环只调本函数 (内部死循环)
// 接线:   USB-TTL → 小车 UART1: USB_TX→P04_0(小车RX), USB_RX→P04_1(小车TX), 共GND
// 波特率: 115200 8N1
// 行为:   电脑发 #x,y,flag$ 帧 → 小车中断收字节 → 整帧解析 → UART0打印 + UART1回显
//         注意: 本库 uart_query_byte 读的是软件缓冲, 必须走接收中断 (uart_isr_mask 填充), 纯轮询不可用
//-------------------------------------------------------------------------------------------------------------------
void Uart1_EchoTest(void)
{
    HC06_Init(115200);      // FIFO + UART1 @115200 + RX中断 (引脚 P04_1/P04_0), ISR 由 uart1_isr 调 HC06_UART_RX_Handler
    printf("ECHO-TEST: UART1 @115200 rx-irq ready, send #x,y,flag$ frames...\n");
    while (1) {
        HC06_Task();        // 解析 #...$ → ParseFrameData → UART0打印 DRONE-RX + UART1回显 RX: [...]
    }
}