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

/*==================================================== 编码器 ====================================================*/
#define BCN_ENC_CALIB  4.30f   // 微调: 4.17→4.30, 补偿~3%系统性欠量
#define BCN_MAX        5   // 5信标

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

/*==================================================== 信标坐标 (绝对, 累积) ====================================================*/
typedef struct { float x, y; } pt_t;

static pt_t    bcn_abs[BCN_MAX];     // 各信标绝对坐标 (m)
static pt_t    bcn_pos;              // 当前累积位置 (记录时逐步叠加)
static uint8_t bcn_idx = 0;
static pt_t    bcn_center;

// 航点: B1, B2, ..., Center (共 BCN_MAX+1 个)
#define WP_MAX  (BCN_MAX + 1)  // 5信标 + 1中心 = 6航点
static pt_t    wp_abs[WP_MAX];       // 航点绝对坐标 (m)
static float   wp_dist[WP_MAX];      // 每段距离 (m, 仅打印)
static float   wp_yaw[WP_MAX];       // 每段角度 (度, 仅打印)
static uint8_t wp_idx = 0;
static pt_t    seg_start;            // 当前段起点 (m)

/*==================================================== 状态机 ====================================================*/
enum { BCN_IDLE, BCN_RECORD, BCN_DONE, BCN_GO };
static uint8_t  bcn_state    = BCN_IDLE;
static float    bcn_yaw      = 0.0f;
static float    bcn_dist_m   = 0.0f;
static int32_t  bcn_enc0[4]  = {0};
static uint16_t bcn_debounce = 0;
static uint16_t bcn_pause    = 0;
static float    bcn_rem_i    = 0.0f;
static uint8_t  bcn_go_first = 1;

uint8_t bcn_nav_on   = 0;
float   bcn_nav_vx   = 0.0f;
float   bcn_nav_vy   = 0.0f;
float   bcn_nav_angle = 0.0f;

/*==================================================== 初始化 ====================================================*/
void InertialNav_Init(void)
{
    bcn_state = BCN_IDLE;
    bcn_idx   = 0;
    bcn_pos.x = 0.0f; bcn_pos.y = 0.0f;
    bcn_nav_on = 0;
}

/*==================================================== 计算中心+航点 ====================================================*/
static void bcn_build_waypoints(void)
{
    // 打印所有信标
    for (int i = 0; i < BCN_MAX; i++) {
        printf("INAV: B%d(%.2f,%.2f)\n", i+1, bcn_abs[i].x, bcn_abs[i].y);
    }

    // 几何中心 = 所有信标平均值
    float sx = 0.0f, sy = 0.0f;
    for (int i = 0; i < BCN_MAX; i++) { sx += bcn_abs[i].x; sy += bcn_abs[i].y; }
    bcn_center.x = sx / (float)BCN_MAX;
    bcn_center.y = sy / (float)BCN_MAX;
    printf("INAV: CENTER(%.2f,%.2f)\n", bcn_center.x, bcn_center.y);

    // 航点: B1~B5 + Center
    for (int i = 0; i < BCN_MAX; i++) wp_abs[i] = bcn_abs[i];
    wp_abs[BCN_MAX] = bcn_center;

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

static void bcn_start_nav(void);  // 前置声明

/*==================================================== KEY_4 ====================================================*/
void InertialNav_KeyHandler(void)
{
    if (key_get_state(KEY_4) != KEY_SHORT_PRESS) return;
    if (bcn_debounce > 0) return;
    bcn_debounce = 80;

    switch (bcn_state) {

    case BCN_IDLE:
        if (bcn_idx >= BCN_MAX) {
            // 全部信标已记录, 启动连续导航
            bcn_start_nav();
        } else {
            // 开始记录下一个信标
            bcn_enc0[0] = motor_L1.total_encoder;
            bcn_enc0[1] = motor_L2.total_encoder;
            bcn_enc0[2] = motor_R1.total_encoder;
            bcn_enc0[3] = motor_R2.total_encoder;
            bcn_yaw = My_Imu660ra_GetYaw();
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
static void bcn_start_nav(void)
{
    wp_idx = 0;
    bcn_state = BCN_GO;
    seg_start.x = 0.0f; seg_start.y = 0.0f;  // 从原点出发

    My_Imu660ra_ResetYaw();
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
        // 检查: 是否已记录完且按下KEY_4启动导航?
        // 由KEY_4在IDLE状态下检测: 如果bcn_idx>=BCN_MAX则启动
        if (bcn_state == BCN_IDLE && bcn_idx >= BCN_MAX) {
            // KEY_4已在KeyHandler中处理过, 这里由update触发启动
            // 实际上由KeyHandler中的额外逻辑处理
        }
        bcn_nav_on = 0;
        return;
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

    // 位置PID → vx/vy (Kp降为0.8防震荡, 限幅0.25m/s)
    #define POS_KP  1.0f
    #define POS_MAX 0.25f
    float vx = POS_KP * err_x;
    float vy = POS_KP * err_y;
    float spd = sqrtf(vx * vx + vy * vy);
    if (spd > POS_MAX) { vx *= POS_MAX / spd; vy *= POS_MAX / spd; }

    bcn_nav_vx   = vx;
    bcn_nav_vy   = vy;
    bcn_nav_angle = 0.0f;
    bcn_nav_on   = 1;
}
