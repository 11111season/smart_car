/*********************************************************************************************************************
* CYT4BB Opensourec Library
* Copyright (c) 2022 SEEKFREE
*
* 文件名称          inertial_nav
* 功能描述          惯性导航: 链式多信标记录 + 连续航点导航
*                   记录: B1=原点→B1, B2=B1→B2 (链式)
*                   导航: 原点→B1→B2→几何中心 (连续, 不中断)
********************************************************************************************************************/

#include "inertial_nav.h"
#include "QMC5883L.h"

/*==================================================== 编译开关 ====================================================*/
#define INAV_MODE  1   // 0=五信标链式+航点  1=直录几何中心

/*==================================================== 编码器 ====================================================*/
#define BCN_ENC_CALIB  4.30f

static float enc2m(int32_t p) {
    return (float)p / ENCODER_PULSES_PER_REV * ENCODER_GEAR_RATIO * WHEEL_CIRCUMFERENCE * BCN_ENC_CALIB;
}
static int32_t enc_delta(int32_t e0[4]) {
    int32_t d1 = motor_L1.total_encoder - e0[0];
    int32_t d2 = motor_L2.total_encoder - e0[1];
    int32_t d3 = motor_R1.total_encoder - e0[2];
    int32_t d4 = motor_R2.total_encoder - e0[3];
    return (int32_t)((fabsf((float)d1)+fabsf((float)d2)+fabsf((float)d3)+fabsf((float)d4))*0.25f);
}

/*==================================================== 信标坐标 ====================================================*/
typedef struct { float x, y; } pt_t;

#if INAV_MODE == 0
  #define BCN_MAX  5
  #define WP_MAX   (BCN_MAX + 1)   // 5信标 + 1中心
#else
  #define BCN_MAX  1               // 只记录1次(几何中心)
  #define WP_MAX   1               // 1个航点
#endif

static pt_t    bcn_abs[BCN_MAX];
static pt_t    bcn_pos;
static uint8_t bcn_idx = 0;
static pt_t    bcn_center;

static pt_t    wp_abs[WP_MAX];
static float   wp_dist[WP_MAX];
static float   wp_yaw[WP_MAX];
static uint8_t wp_idx = 0;
static pt_t    seg_start;

#define POS_KP  1.0f    // 位置闭环P增益
#define POS_MAX 0.25f   // 位置闭环最大速度(m/s)

/*==================================================== 状态机 ====================================================*/
enum { BCN_IDLE, BCN_RECORD, BCN_DONE, BCN_GO };
static uint8_t  bcn_state    = BCN_IDLE;
static float    bcn_yaw      = 0.0f;
static float    bcn_dist_m   = 0.0f;
static int32_t  bcn_enc0[4]  = {0};
uint16_t bcn_debounce = 0;
static uint16_t bcn_pause    = 0;
static float    bcn_rem_i    = 0.0f;
static uint8_t  bcn_go_first = 1;

uint8_t bcn_nav_on   = 0;
float   bcn_nav_vx   = 0.0f;
float   bcn_nav_vy   = 0.0f;
float   bcn_nav_angle = 0.0f;

float   g_pos_x = 0.0f, g_pos_y = 0.0f;
uint8_t go_center = 0;
float   fused_yaw = 0.0f;

/*==================================================== 磁力计融合 ====================================================*/
static float mag_offset = 0.0f;    // yaw零点偏移

void MagYaw_Reset(void) {          // 外部调用: 设当前方向为0度
    qmc5883l_get_all();
    mag_offset = qmc5883l_heading;
}

// 互补滤波: α=0.98 陀螺仪, (1-α)=0.02 磁力计
void MagYaw_Update(void)
{
    static float fy = 0.0f;
    static uint8_t init = 0;

    float gz = imu660_gz;
    fy += gz * 0.01f;

    qmc5883l_get_all();
    float mag = qmc5883l_heading;

    if (!init) { fy = mag; init = 1; }

    float diff = mag - fy;
    while (diff >  180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    fy += diff * 0.005f;

    while (fy >  360.0f) fy -= 360.0f;
    while (fy <    0.0f) fy += 360.0f;

    // 减零点偏移 → 相对角度
    fused_yaw = fy - mag_offset;
    while (fused_yaw >  180.0f) fused_yaw -= 360.0f;
    while (fused_yaw < -180.0f) fused_yaw += 360.0f;
}

/*==================================================== 初始化 ====================================================*/
void InertialNav_Init(void)
{
    bcn_state = BCN_IDLE;
    bcn_idx   = 0;
    bcn_pos.x = 0.0f; bcn_pos.y = 0.0f;
    bcn_nav_on = 0;
    g_pos_x = 0.0f; g_pos_y = 0.0f;
    go_center = 0;
}

/*==================================================== 实时位置追踪 (ISR 10ms调用) ====================================================*/
void InertialNav_PosUpdate(void)
{
    // 麦轮正运动学: encoder_speed均已统一符号(正=前进)
    // vx = (L1+L2+R1+R2)/4,  vy = (-L1+L2+R1-R2)/4
    float s1 = motor_L1.encoder_speed;
    float s2 = motor_L2.encoder_speed;
    float s3 = motor_R1.encoder_speed;
    float s4 = motor_R2.encoder_speed;

    float vx = ( s1 + s2 + s3 + s4) * 0.25f;   // 前进: 四轮同向
    float vy = ( s1 - s2 - s3 + s4) * 0.25f;   // 左移: L1正 R2正 L2反 R1反

    float dx = enc2m((int32_t)fabsf(vx)); if (vx < 0.0f) dx = -dx;
    float dy = enc2m((int32_t)fabsf(vy)); if (vy < 0.0f) dy = -dy;

    // 车体系 → 世界系 (记录bcn_center的坐标系)
    float yaw_rad = My_Imu660ra_GetYaw() * (PI / 180.0f);
    float c = cosf(yaw_rad), s = sinf(yaw_rad);
    g_pos_x += dx * c - dy * s;
    g_pos_y += dx * s + dy * c;
}

/*==================================================== 计算中心+航点 ====================================================*/
static void bcn_build_waypoints(void)
{
    // 打印所有信标
    for (int i = 0; i < BCN_MAX; i++) {
        printf("INAV: B%d(%.2f,%.2f)\n", i+1, bcn_abs[i].x, bcn_abs[i].y);
    }

#if INAV_MODE == 0
    // 模式0: 五信标 → 几何中心 + 链式航点
    float sx = 0.0f, sy = 0.0f;
    for (int i = 0; i < BCN_MAX; i++) { sx += bcn_abs[i].x; sy += bcn_abs[i].y; }
    bcn_center.x = sx / (float)BCN_MAX;
    bcn_center.y = sy / (float)BCN_MAX;
    printf("INAV: CENTER(%.2f,%.2f)\n", bcn_center.x, bcn_center.y);
    for (int i = 0; i < BCN_MAX; i++) wp_abs[i] = bcn_abs[i];
    wp_abs[BCN_MAX] = bcn_center;
#else
    // 模式1: 直录中心, 仅1个航点
    bcn_center = bcn_abs[0];  // 中心=录制点
    wp_abs[0] = bcn_center;
    printf("INAV: CENTER(%.2f,%.2f)\n", bcn_center.x, bcn_center.y);
#endif

    // 预计算每段距离和方向
    pt_t prev = {0.0f, 0.0f};
    for (int i = 0; i < WP_MAX; i++) {
        float dx = wp_abs[i].x - prev.x;
        float dy = wp_abs[i].y - prev.y;
        wp_dist[i] = sqrtf(dx * dx + dy * dy);
        wp_yaw[i]  = atan2f(dy, dx) * (180.0f / PI);
        if (wp_yaw[i] < 0.0f) wp_yaw[i] += 360.0f;
        printf("INAV: SEG%d d=%.2fm a=%.1f\n", i, wp_dist[i], wp_yaw[i]);
        prev = wp_abs[i];
    }
}

static void bcn_start_nav(void);
static void bcn_start_idle(void);

/*==================================================== KEY_4 ====================================================*/
void InertialNav_KeyHandler(void)
{
    if (key_get_state(KEY_4) != KEY_SHORT_PRESS) return;
    if (bcn_debounce > 0) return;
    bcn_debounce = 80;

    switch (bcn_state) {

    case BCN_IDLE:
        if (bcn_idx >= BCN_MAX) {
            if (!mission_armed) {
                // 第3次按键4: 武装任务, 启动PID, 清零位置, 等待无人机标志位
                mission_armed = 1;
                angle_target = 0.0f;
                target_vx = 0.0f; target_vy = 0.0f;
                Motor_Enable_PID(1);
                PID_Reset(&angle_pid_yaw);   PID_Enable(&angle_pid_yaw, 1);
                PID_Reset(&angle_pid_gyro);  PID_Enable(&angle_pid_gyro, 1);
                g_pos_x = 0.0f; g_pos_y = 0.0f;
                go_center = 0;
                My_Imu660ra_ResetYaw();
                MagYaw_Reset();
                bcn_debounce = 80;
                printf("INAV: Mission armed, waiting for drone flag...\n");
            }
            // 已武装: 不再重复操作
        } else {
            // 开始记录信标: PID全关 + PWM清零
            Motor_Enable_PID(0);
            PID_Enable(&angle_pid_yaw, 0);
            PID_Enable(&angle_pid_gyro, 0);
            bcn_nav_on = 0;
            Motor_SetSpeed(&motor_L1,0,MotorL1_Turn,MotorL1_Pwm);
            Motor_SetSpeed(&motor_L2,0,MotorL2_Turn,MotorL2_Pwm);
            Motor_SetSpeed(&motor_R1,0,MotorR1_Turn,MotorR1_Pwm);
            Motor_SetSpeed(&motor_R2,0,MotorR2_Turn,MotorR2_Pwm);
            bcn_enc0[0] = motor_L1.total_encoder;
            bcn_enc0[1] = motor_L2.total_encoder;
            bcn_enc0[2] = motor_R1.total_encoder;
            bcn_enc0[3] = motor_R2.total_encoder;
            bcn_yaw = My_Imu660ra_GetYaw();   // 与PosUpdate同用陀螺仪yaw
            bcn_state = BCN_RECORD;
            printf("INAV: B%d record start, yaw=%.1f\n", bcn_idx + 1, bcn_yaw);
        }
        break;

    case BCN_RECORD:
        bcn_dist_m = enc2m(enc_delta(bcn_enc0));
        {
            // 链式累积绝对坐标
            float rad = bcn_yaw * (PI / 180.0f);
            bcn_pos.x += bcn_dist_m * cosf(rad);
            bcn_pos.y += bcn_dist_m * sinf(rad);
            bcn_abs[bcn_idx] = bcn_pos;
        }
        printf("INAV: B%d dist=%.2fm abs(%.2f,%.2f)\n",
               bcn_idx + 1, bcn_dist_m, bcn_abs[bcn_idx].x, bcn_abs[bcn_idx].y);

        bcn_idx++;
        if (bcn_idx >= BCN_MAX) {
            bcn_build_waypoints();
            printf("INAV: press KEY_4 to GO\n");
        }
        bcn_state = BCN_IDLE;
        bcn_nav_on = 0;
        bcn_debounce = 20;   // 缩短消抖
        break;

    case BCN_GO:
        // 已在导航中: 强制停止
        Motor_Enable_PID(0);
        PID_Enable(&angle_pid_yaw, 0);
        PID_Enable(&angle_pid_gyro, 0);
        target_vx = 0.0f; target_vy = 0.0f;
        bcn_nav_on = 0;
        bcn_rem_i = 0.0f; bcn_go_first = 1;
        bcn_state = BCN_IDLE;
        bcn_idx = 0;
        bcn_pos.x = 0.0f; bcn_pos.y = 0.0f;
        printf("INAV: force stop\n");
        break;

    case BCN_DONE:
        break;
    }
    key_clear_state(KEY_4);
}

// 启动导航 (从KEY_4 DONE状态触发, 或由update自动续段)
// 仅启动PID(不导航), 清零实时位置 → 等待蓝牙M触发
static void bcn_start_idle(void)
{
    // MagYaw_Reset已移至KeyTask_Init, 不再重复调用
    angle_target = 0.0f;
    target_vx = 0.0f; target_vy = 0.0f;
    Motor_Enable_PID(1);
    PID_Reset(&angle_pid_yaw);   PID_Enable(&angle_pid_yaw, 1);
    PID_Reset(&angle_pid_gyro);  PID_Enable(&angle_pid_gyro, 1);
    g_pos_x = 0.0f; g_pos_y = 0.0f;  // 清零实时位置追踪
    go_center = 0;
    printf("INAV: PID on, pos zeroed, waiting M key...\n");
}

static void bcn_start_nav(void)
{
    wp_idx = 0;
    bcn_state = BCN_GO;
    seg_start.x = 0.0f; seg_start.y = 0.0f;

    angle_target = 0.0f;
    target_vx = 0.0f; target_vy = 0.0f;
    Motor_Enable_PID(1);
    PID_Reset(&angle_pid_yaw);   PID_Enable(&angle_pid_yaw, 1);
    PID_Reset(&angle_pid_gyro);  PID_Enable(&angle_pid_gyro, 1);
    bcn_enc0[0] = motor_L1.total_encoder;
    bcn_enc0[1] = motor_L2.total_encoder;
    bcn_enc0[2] = motor_R1.total_encoder;
    bcn_enc0[3] = motor_R2.total_encoder;
    bcn_rem_i = 0.0f; bcn_go_first = 1;
    printf("INAV: SEG%d GO d=%.2fm a=%.1f\n",
           wp_idx, wp_dist[wp_idx], wp_yaw[wp_idx]);
}

/*==================================================== 主循环更新 ====================================================*/
void InertialNav_Update(void)
{
    if (bcn_debounce > 0) bcn_debounce--;

    // ---- 标志位2触发去中心 (替代蓝牙M键) ----
    // 条件: 已武装 + 无人机发2(丢信标) + 未在导航 + 消抖结束 + 距中心>15cm
    if (mission_armed && drone_beacon_flag == 2 && !go_center && bcn_debounce == 0) {
        float dx = bcn_center.x - g_pos_x;
        float dy = bcn_center.y - g_pos_y;
        if (sqrtf(dx * dx + dy * dy) > 0.15f) {
            go_center = 1;
            printf("INAV: flag=2, go center (%.2f,%.2f)\n", bcn_center.x, bcn_center.y);
        }
    }

    // ---- 去中心导航消抖保护 ----
    if (go_center && bcn_debounce > 0) {
        bcn_nav_on = 0;  // 消抖期, 强制关nav
        return;
    }
    if (go_center) {
        float err_x = bcn_center.x - g_pos_x;
        float err_y = bcn_center.y - g_pos_y;
        float err   = sqrtf(err_x * err_x + err_y * err_y);
        if (err <= 0.15f || (fabsf(err_x) < 0.02f && fabsf(err_y) < 0.15f)) {
            bcn_nav_vx = 0.0f; bcn_nav_vy = 0.0f; bcn_nav_angle = 0.0f;
            bcn_nav_on = 0;
            target_vx = 0.0f; target_vy = 0.0f;
            go_center = 0;
            bcn_nav_vx = 0.0f; bcn_nav_vy = 0.0f; bcn_nav_angle = 0.0f;
            bcn_nav_on = 0;
            target_vx = 0.0f; target_vy = 0.0f;
            bcn_debounce = 50;   // 500ms强制消抖, 禁止任何重入
            Motor_Enable_PID(0); Motor_Enable_PID(1);
            printf("INAV: back to center! err=%.3fm\n", err);
            return;
        }
        float vx = POS_KP * err_x;
        float vy = POS_KP * err_y;
        float spd = sqrtf(vx * vx + vy * vy);
        if (spd > POS_MAX) { vx *= POS_MAX / spd; vy *= POS_MAX / spd; }
        bcn_nav_vx = vx; bcn_nav_vy = vy; bcn_nav_angle = 0.0f;
        bcn_nav_on = 1;
        return;
    }

    // 段间暂停: 倒计时, 到零时加载下一段
    if (bcn_pause > 0) {
        bcn_pause--;
        if (bcn_pause == 0) {
            seg_start = wp_abs[wp_idx - 1];   // 新段起点 = 上一航点
            bcn_enc0[0] = motor_L1.total_encoder;
            bcn_enc0[1] = motor_L2.total_encoder;
            bcn_enc0[2] = motor_R1.total_encoder;
            bcn_enc0[3] = motor_R2.total_encoder;
            bcn_rem_i = 0.0f; bcn_go_first = 1;
            bcn_debounce = 80;
            printf("INAV: SEG%d GO d=%.2fm a=%.1f [auto]\n",
                   wp_idx, wp_dist[wp_idx], wp_yaw[wp_idx]);
        }
        return;
    }

    if (bcn_state != BCN_GO) {
        return;   // 不在此处清零bcn_nav_on, 由蓝牙/go_center自行管理
    }

    // ---- 导航中: 编码器位置闭环 ----
    int32_t traveled = enc_delta(bcn_enc0);
    float   traveled_m = enc2m(traveled);

    // 当前位置 = 段起点 + 编码器位移沿段方向投影
    float rad = wp_yaw[wp_idx] * (PI / 180.0f);
    pt_t cur;
    cur.x = seg_start.x + traveled_m * cosf(rad);
    cur.y = seg_start.y + traveled_m * sinf(rad);

    // 误差 = 目标 - 当前
    pt_t *tgt = &wp_abs[wp_idx];
    float err_x = tgt->x - cur.x;
    float err_y = tgt->y - cur.y;
    float err_dist = sqrtf(err_x * err_x + err_y * err_y);

    if (err_dist <= 0.02f) {
        printf("INAV: SEG%d arrived! err=%.3fm pos(%.2f,%.2f)\n",
               wp_idx, err_dist, cur.x, cur.y);
        wp_idx++;

        if (wp_idx >= WP_MAX) {
            bcn_nav_vx = 0.0f; bcn_nav_vy = 0.0f; bcn_nav_angle = 0.0f;
            bcn_nav_on = 0;
            target_vx = 0.0f; target_vy = 0.0f;
            bcn_state = BCN_IDLE;
            bcn_idx = 0;
            bcn_pos.x = 0.0f; bcn_pos.y = 0.0f;
            bcn_debounce = 80;
            printf("INAV: all done!\n");
            return;
        }

        bcn_pause = 50;
        bcn_nav_vx = 0.0f; bcn_nav_vy = 0.0f; bcn_nav_angle = 0.0f;
        bcn_nav_on = 0;
        target_vx = 0.0f; target_vy = 0.0f;
        printf("INAV: SEG%d arrived, pause 500ms...\n", wp_idx - 1);
        return;
    }

    // 位置PID → vx/vy
    float vx = POS_KP * err_x;
    float vy = POS_KP * err_y;
    float spd = sqrtf(vx * vx + vy * vy);
    if (spd > POS_MAX) { vx *= POS_MAX / spd; vy *= POS_MAX / spd; }

    bcn_nav_vx   = vx;
    bcn_nav_vy   = vy;
    bcn_nav_angle = 0.0f;
    bcn_nav_on   = 1;
}
