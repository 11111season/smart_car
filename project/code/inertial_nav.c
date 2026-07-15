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
#define INAV_MODE  1   // 0=五信标链式循环  1=双航点巡逻

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
  #define BCN_MAX  2               // 记录2个航点 (WP3硬编码, 不需记录)
  #define WP_MAX   3               // 3个航点: WP1+WP2(录制) + WP3(0,0.5)
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
#define POS_MAX 0.17f   // 位置闭环最大速度(m/s)

/*==================================================== 状态机 ====================================================*/
enum { BCN_IDLE, BCN_RECORD, BCN_DONE, BCN_GO };
static uint8_t  bcn_state    = BCN_IDLE;
static float    bcn_yaw      = 0.0f;
static float    bcn_dist_m   = 0.0f;
static int32_t  bcn_enc0[4]  = {0};
uint16_t bcn_debounce = 0;
static uint64_t bcn_pause_start_us = 0;  // 段间暂停开始时刻 (微秒)
#define BCN_PAUSE_US  2000000            // 段间等待: 2秒
static float    bcn_rem_i    = 0.0f;
static uint8_t  bcn_go_first = 1;

uint8_t bcn_nav_on   = 0;
float   bcn_nav_vx   = 0.0f;
float   bcn_nav_vy   = 0.0f;
float   bcn_nav_angle = 0.0f;

float   g_pos_x = 0.0f, g_pos_y = 0.0f;
uint8_t go_center = 0;
float   fused_yaw = 0.0f;

/*==================================================== 巡逻状态 ====================================================*/
static uint8_t  patrol_wp = 0;          // 当前巡逻目标航点索引 (0 或 1)
static uint64_t patrol_arrive_us = 0;   // 到达航点时的 time_us 时间戳
uint8_t patrol_active = 0;              // 巡逻模式激活标志, ISR可清除
#define PATROL_WAIT_US  2000000         // 航点等待超时: 3秒 (微秒)

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
    // 消除 mode-conditional 变量的编译警告
    (void)bcn_rem_i; (void)bcn_go_first;  // 仅被写入从未读取
#if INAV_MODE == 1
    (void)bcn_center;                     // mode 1 不使用几何中心
#endif
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
    // 模式1: 双录制航点 + 硬编码WP3 (发车区正前方0.5m)
    wp_abs[0] = bcn_abs[0];
    wp_abs[1] = bcn_abs[1];
    wp_abs[2].x = 0.5f; wp_abs[2].y = 0.0f;   // WP3: 发车区正前方0.5m
    printf("INAV: WP1(%.2f,%.2f) WP2(%.2f,%.2f) WP3(%.2f,%.2f)\n",
           wp_abs[0].x, wp_abs[0].y,
           wp_abs[1].x, wp_abs[1].y,
           wp_abs[2].x, wp_abs[2].y);
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

/*==================================================== 模式0: 启动五信标循环导航 ====================================================*/
#if INAV_MODE == 0
static void bcn_start_cycle(void)
{
    mission_armed = 1;
    mission_arm_time = time_us;     // 记录发车时刻, 等4s后执行
    wp_idx = 0;
    bcn_state = BCN_GO;
    seg_start.x = 0.0f; seg_start.y = 0.0f;

    angle_target = 0.0f;
    target_vx = 0.0f; target_vy = 0.0f;
    Motor_Enable_PID(1);
    PID_Reset(&angle_pid_yaw);   PID_Enable(&angle_pid_yaw, 1);
    PID_Reset(&angle_pid_gyro);  PID_Enable(&angle_pid_gyro, 1);
    g_pos_x = 0.0f; g_pos_y = 0.0f;
    My_Imu660ra_ResetYaw();
    MagYaw_Reset();

    bcn_enc0[0] = motor_L1.total_encoder;
    bcn_enc0[1] = motor_L2.total_encoder;
    bcn_enc0[2] = motor_R1.total_encoder;
    bcn_enc0[3] = motor_R2.total_encoder;
    bcn_pause_start_us = 0;
    bcn_debounce = 80;
    printf("INAV: Cycle start, SEG%d d=%.2fm a=%.1f\n",
           wp_idx, wp_dist[wp_idx], wp_yaw[wp_idx]);
}
#endif

/*==================================================== KEY_4 ====================================================*/
void InertialNav_KeyHandler(void)
{
    if (key_get_state(KEY_4) != KEY_SHORT_PRESS) return;
    if (bcn_debounce > 0) return;
    bcn_debounce = 80;

    switch (bcn_state) {

    case BCN_IDLE:
        if (bcn_idx >= BCN_MAX) {
#if INAV_MODE == 0
            if (!mission_armed) {
                bcn_start_cycle();       // 五信标记录完毕, 开始循环导航
            }
#else
            if (!mission_armed) {
                // 2个航点已记录完毕, 武装任务, 等4s无人机就位后执行
                mission_armed = 1;
                mission_arm_time = time_us;
                printf("INAV: Mission armed, waiting 4s for drone...\n");
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
#endif
            // 已武装: 不再重复操作
        } else {
            // 开始记录信标: PID全关 + PWM清零
#if INAV_MODE == 1
            bcn_pos.x = 0.0f; bcn_pos.y = 0.0f;  // mode 1: 独立坐标, 每个航点从原点开始
#endif
            // mode 0: 链式累积, bcn_pos 不重置 (B1=原点→B1, B2=B1→B2, ...)
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

/*==================================================== 主循环更新 ====================================================*/
void InertialNav_Update(void)
{
    if (bcn_debounce > 0) bcn_debounce--;

    // ---- 巡逻: flag=2 连续10帧才确认丢信标, 在3个航点间循环 ----
    // 触发条件: 已武装 + 丢信标确认 + 未在巡逻 + 消抖结束
    if (mission_armed && time_us - mission_arm_time >= 4000000
        && flag2_count >= FLAG2_DEBOUNCE && !patrol_active && bcn_debounce == 0) {
        float d_min = 1e9f;
        for (int i = 0; i < WP_MAX; i++) {
            float d = sqrtf((wp_abs[i].x-g_pos_x)*(wp_abs[i].x-g_pos_x) +
                            (wp_abs[i].y-g_pos_y)*(wp_abs[i].y-g_pos_y));
            if (d < d_min) { d_min = d; patrol_wp = (uint8_t)i; }
        }
        patrol_arrive_us = 0;
        patrol_active = 1;
        printf("INAV: Patrol start, closer=wp%d (%.2f,%.2f) d=%.2f\n",
               patrol_wp+1, wp_abs[patrol_wp].x, wp_abs[patrol_wp].y, d_min);
    }

    if (patrol_active) {
        pt_t *tgt = &wp_abs[patrol_wp];
        float err_x = tgt->x - g_pos_x;
        float err_y = tgt->y - g_pos_y;
        float err = sqrtf(err_x*err_x + err_y*err_y);

        if (err <= 0.15f || (fabsf(err_x) < 0.02f && fabsf(err_y) < 0.15f)) {
            // 已到达航点，原地等待3秒
            bcn_nav_vx = 0.0f; bcn_nav_vy = 0.0f; bcn_nav_angle = 0.0f;
            bcn_nav_on = 0;
            target_vx = 0.0f; target_vy = 0.0f;

            if (patrol_arrive_us == 0) {
                patrol_arrive_us = time_us;   // 记录到达时刻
                printf("INAV: Arrived wp%d, waiting 3s...\n", patrol_wp+1);
            }
            if (time_us - patrol_arrive_us >= PATROL_WAIT_US) {
                patrol_wp = (patrol_wp + 1) % WP_MAX;   // 顺序循环 0→1→2→0
                patrol_arrive_us = 0;
                printf("INAV: No beacon, switch to wp%d\n", patrol_wp+1);
            }
            return;
        }

        // 离开航点范围: 重置到达时间戳 (防止中途短暂触发计时)
        patrol_arrive_us = 0;

        // 导航去航点
        float vx = POS_KP * err_x;
        float vy = POS_KP * err_y;
        float spd = sqrtf(vx*vx + vy*vy);
        if (spd > POS_MAX) { vx *= POS_MAX/spd; vy *= POS_MAX/spd; }
        bcn_nav_vx = vx; bcn_nav_vy = vy; bcn_nav_angle = 0.0f;
        bcn_nav_on = 1;
        return;
    }

    // ---- 段间暂停: time_us计时, 2秒等待 (mode 0 使用) ----
    if (bcn_pause_start_us > 0) {
        if (time_us - bcn_pause_start_us < BCN_PAUSE_US) {
            return;   // 等待中
        }
        // 暂停结束, 开始下一段
        bcn_pause_start_us = 0;
        if (wp_idx == 0) {
            seg_start.x = 0.0f; seg_start.y = 0.0f;   // 循环重启: 从原点开始
        } else {
            seg_start = wp_abs[wp_idx - 1];
        }
        bcn_enc0[0] = motor_L1.total_encoder;
        bcn_enc0[1] = motor_L2.total_encoder;
        bcn_enc0[2] = motor_R1.total_encoder;
        bcn_enc0[3] = motor_R2.total_encoder;
        bcn_rem_i = 0.0f; bcn_go_first = 1;
        bcn_debounce = 80;
        printf("INAV: SEG%d GO d=%.2fm a=%.1f [auto]\n",
               wp_idx, wp_dist[wp_idx], wp_yaw[wp_idx]);
    }

    if (bcn_state != BCN_GO) {
        return;
    }

    // ---- 导航中: 编码器位置闭环 (mode 0 使用) ----
    int32_t traveled = enc_delta(bcn_enc0);
    float   traveled_m = enc2m(traveled);

    float rad = wp_yaw[wp_idx] * (PI / 180.0f);
    pt_t cur;
    cur.x = seg_start.x + traveled_m * cosf(rad);
    cur.y = seg_start.y + traveled_m * sinf(rad);

    pt_t *tgt = &wp_abs[wp_idx];
    float err_x = tgt->x - cur.x;
    float err_y = tgt->y - cur.y;
    float err_dist = sqrtf(err_x * err_x + err_y * err_y);

    if (err_dist <= 0.02f) {
        printf("INAV: SEG%d arrived! err=%.3fm pos(%.2f,%.2f)\n",
               wp_idx, err_dist, cur.x, cur.y);
        wp_idx++;

        if (wp_idx >= WP_MAX) {
            // 循环: 回到第一个航点, 等2秒后继续
            wp_idx = 0;
            seg_start.x = 0.0f; seg_start.y = 0.0f;
            bcn_pause_start_us = time_us;
            bcn_nav_vx = 0.0f; bcn_nav_vy = 0.0f; bcn_nav_angle = 0.0f;
            bcn_nav_on = 0;
            target_vx = 0.0f; target_vy = 0.0f;
            bcn_debounce = 80;
            printf("INAV: Cycle restart, back to WP1\n");
            return;
        }

        bcn_pause_start_us = time_us;   // 到达航点, 开始2秒等待
        bcn_nav_vx = 0.0f; bcn_nav_vy = 0.0f; bcn_nav_angle = 0.0f;
        bcn_nav_on = 0;
        target_vx = 0.0f; target_vy = 0.0f;
        printf("INAV: SEG%d arrived, pause 2s...\n", wp_idx - 1);
        return;
    }

    float vx = POS_KP * err_x;
    float vy = POS_KP * err_y;
    float spd = sqrtf(vx * vx + vy * vy);
    if (spd > POS_MAX) { vx *= POS_MAX / spd; vy *= POS_MAX / spd; }

    bcn_nav_vx   = vx;
    bcn_nav_vy   = vy;
    bcn_nav_angle = 0.0f;
    bcn_nav_on   = 1;
}
