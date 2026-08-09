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
#include "App_menu.h"     // 菜单全局: wp_set/launch_enable/launch_off_x/y (W25Q64掉电存储覆盖)

/*==================================================== 编译开关 ====================================================*/
#define INAV_MODE  1   // 0=五信标链式循环  1=航点巡逻

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
typedef flash_wp_t pt_t;   // 航点坐标 (与 W25Q64 存储 flash_wp_t 同布局)

/* 航点上限为运行时变量 (原编译期宏 BCN_MAX/WP_MAX 废弃):
 *   bcn_max = 菜单设置的航点数量 (不含发车区) = wp_set
 *   wp_max  = bcn_max + launch_enable  (启用发车区则 +1 发车区航点)
 * 上电从 W25Q64 读设置覆盖, 数组按物理上限 FLASH_WP_MAX 开 */
uint8_t bcn_max = 2;                          // 记录航点数 (不含发车区)
uint8_t wp_max  = 2;                          // 总航点数 (含发车区)

static pt_t    bcn_abs[FLASH_WP_MAX];         // 录制航点 (最多9)
static pt_t    bcn_pos;
static uint8_t bcn_idx = 0;
static pt_t    bcn_center;

static pt_t    wp_abs[FLASH_WP_MAX + 1];      // 最终导航航点 (录制 + 发车区)
static float   wp_dist[FLASH_WP_MAX + 1];
static float   wp_yaw[FLASH_WP_MAX + 1];
static uint8_t wp_idx = 0;
static pt_t    seg_start;

#define POS_KP  1.0f    // 位置闭环P增益
#define POS_MAX 0.30f   // 位置闭环最大速度(m/s)

/*==================================================== 状态机 ====================================================*/
enum { BCN_IDLE, BCN_RECORD, BCN_DONE, BCN_GO };
static uint8_t  bcn_state    = BCN_IDLE;
static float    bcn_yaw      = 0.0f;
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
static float mag_fy = 0.0f;        // 融合绝对航向(0-360), 与 mag_offset 配套

void MagYaw_Reset(void) {          // 外部调用: 设当前方向为0度 (发车/记录起点)
    qmc5883l_get_all();
    // 同时对齐融合内部状态与磁力计 → fused_yaw 立即精确为0
    // (只改 mag_offset 会留下 fy 与 mag 之间的残余差, 发车瞬间世界系有残差)
    mag_offset = qmc5883l_heading;
    mag_fy     = qmc5883l_heading;
}

// 互补滤波: α=0.98 陀螺仪, (1-α)=0.02 磁力计 (2026-08-09 架构: 磁力计=实时绝对基准)
// 磁力计权重恒定: 架高20cm/距电机15cm动态实测 EM中心偏移-1.9°(可忽略), 全程信任磁力计
// 不再有运动门控 — 需要实时绝对基准持续修正陀螺积分, 航点停靠锚定已废弃
#define MAG_YAW_WEIGHT  0.02f   // 每20ms磁力计修正权重: 修正时间常数≈1s
void MagYaw_Update(void)
{
    static uint8_t init = 0;
    static uint8_t skip_cnt = 0;

    float gz = imu660_gz;
    mag_fy += gz * 0.01f;

    // 磁力计每20ms读一次 (QMC5883L 100Hz采样 + 512过采样, 10ms读可能读到新旧混合)
    // 融合计算仍每10ms执行, 磁力计修正隔帧更新
    skip_cnt++;
    if (skip_cnt >= 2) {
        skip_cnt = 0;
        qmc5883l_get_all();
        float mag = qmc5883l_heading;

        if (!init) { mag_fy = mag; init = 1; }

        // 磁力计跳变保护: 相邻采样角差超过阈值视为异常尖峰
        // 用±180环绕后的最短角差, 区分过零(359→1的diff=2)与尖峰(57→6的diff=-50)
        // 连续 MAG_SPIKE_ACCEPT 次异常才信任新值(防真实快速转动被误杀)
        static float mag_prev = -1.0f;
        static uint8_t mag_spike_cnt = 0;
        #define MAG_SPIKE_THRESH   20.0f
        #define MAG_SPIKE_ACCEPT   3
        if (mag_prev >= 0.0f) {
            float d = mag - mag_prev;
            while (d >  180.0f) d -= 360.0f;
            while (d < -180.0f) d += 360.0f;
            if (fabsf(d) > MAG_SPIKE_THRESH) {
                if (mag_spike_cnt < MAG_SPIKE_ACCEPT) {
                    mag_spike_cnt++;
                    mag = mag_prev;   // 异常: 用上一次值
                } else {
                    mag_spike_cnt = 0;  // 连续多次异常: 信任新值(可能真在快速转)
                }
            } else {
                mag_spike_cnt = 0;
            }
        }
        mag_prev = mag;

        float diff = mag - mag_fy;
        while (diff >  180.0f) diff -= 360.0f;
        while (diff < -180.0f) diff += 360.0f;

        // ---- 磁力计实时修正: 恒定权重 (2026-08-09 架构: 实时绝对基准) ----
        // 旧门控(运动时关磁力计)已废弃: 架高动态实测电机EM干扰仅-1.9°, 无需关闭
        // 运动/静止都信任磁力计修正陀螺积分 → 不再需要零漂学习
        mag_fy += diff * MAG_YAW_WEIGHT;

        while (mag_fy >  360.0f) mag_fy -= 360.0f;
        while (mag_fy <    0.0f) mag_fy += 360.0f;
    }

    // 减零点偏移 → 相对角度
    fused_yaw = mag_fy - mag_offset;
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
    // 用磁力计融合航向 fused_yaw: 地图坐标系跟随磁力计绝对基准, 不随陀螺积分漂移
    // (2026-08-09 架构: 地图信任磁力计, 实时绝对修正)
    float yaw_rad = fused_yaw * (PI / 180.0f);
    float c = cosf(yaw_rad), s = sinf(yaw_rad);
    g_pos_x += dx * c - dy * s;
    g_pos_y += dx * s + dy * c;
}

/*==================================================== 计算中心+航点 ====================================================*/
static void bcn_build_waypoints(void)
{
    // 打印所有录制航点
    for (int i = 0; i < bcn_max; i++) {
        printf("INAV: B%d(%.2f,%.2f)\n", i+1, bcn_abs[i].x, bcn_abs[i].y);
    }

#if INAV_MODE == 0
    // 模式0: 五信标 → 几何中心 + 链式航点
    float sx = 0.0f, sy = 0.0f;
    for (int i = 0; i < bcn_max; i++) { sx += bcn_abs[i].x; sy += bcn_abs[i].y; }
    bcn_center.x = sx / (float)bcn_max;
    bcn_center.y = sy / (float)bcn_max;
    printf("INAV: CENTER(%.2f,%.2f)\n", bcn_center.x, bcn_center.y);
    for (int i = 0; i < bcn_max; i++) wp_abs[i] = bcn_abs[i];
    wp_abs[bcn_max] = bcn_center;
    wp_max = bcn_max + 1;
#else
    // 模式1: 录制航点 + 可选发车区航点 (WP_MAX = bcn_max + launch_enable)
    for (int i = 0; i < bcn_max; i++) wp_abs[i] = bcn_abs[i];
    if (launch_enable) {
        // 发车区航点 = 起点(世界原点) + 菜单偏移; 世界系 x=前 y=左 (与 g_pos 一致)
        wp_abs[bcn_max].x = (float)launch_off_x * 0.1f;
        wp_abs[bcn_max].y = (float)launch_off_y * 0.1f;
        printf("INAV: LAUNCH wp(%.2f,%.2f)\n", wp_abs[bcn_max].x, wp_abs[bcn_max].y);
    }
    printf("INAV: %d wp:", wp_max);
    for (int i = 0; i < wp_max; i++) printf(" (%.2f,%.2f)", wp_abs[i].x, wp_abs[i].y);
    printf("\n");
#endif

    // 预计算每段距离和方向
    pt_t prev = {0.0f, 0.0f};
    for (int i = 0; i < wp_max; i++) {
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
    HC06_SendDroneCmd(4);           // 通知无人机: 小车已发车 #D$
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
    MagYaw_Reset();   // 2026-08-09: 发车归零磁力计融合 (与陀螺复位同步)

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

/*==================================================== 菜单控制接口 ====================================================*/
// 说明: 原 InertialNav_KeyHandler (KEY_4 记录流程) 已由菜单接管, 废弃删除
// KEY_3 始终为退回主菜单, 故取消闭环放在记录页首次 KEY_4 (见 App_Menu STATE_INR_WP_START)

// 取消闭环: PID全关 + PWM清零 (记录前让小车可自由推动车头)
void Inav_DisableControl(void)
{
    Motor_Enable_PID(0);
    PID_Enable(&angle_pid_yaw, 0);
    PID_Enable(&angle_pid_gyro, 0);
    bcn_nav_on = 0;
    target_vx = 0.0f; target_vy = 0.0f;
    Motor_SetSpeed(&motor_L1,0,MotorL1_Turn,MotorL1_Pwm);
    Motor_SetSpeed(&motor_L2,0,MotorL2_Turn,MotorL2_Pwm);
    Motor_SetSpeed(&motor_R1,0,MotorR1_Turn,MotorR1_Pwm);
    Motor_SetSpeed(&motor_R2,0,MotorR2_Turn,MotorR2_Pwm);
    printf("INAV: closed-loop off, car free to push\n");
}

// 开始记录当前航点: 记编码器起点 + 当前航向
// mode 1: 每个航点从原点(发车区)开始独立记录, bcn_pos 清零
void Inav_RecStart(void)
{
    Motor_Enable_PID(0);
    PID_Enable(&angle_pid_yaw, 0);
    PID_Enable(&angle_pid_gyro, 0);
    bcn_nav_on = 0;
    Motor_SetSpeed(&motor_L1,0,MotorL1_Turn,MotorL1_Pwm);
    Motor_SetSpeed(&motor_L2,0,MotorL2_Turn,MotorL2_Pwm);
    Motor_SetSpeed(&motor_R1,0,MotorR1_Turn,MotorR1_Pwm);
    Motor_SetSpeed(&motor_R2,0,MotorR2_Turn,MotorR2_Pwm);
#if INAV_MODE == 1
    bcn_pos.x = 0.0f; bcn_pos.y = 0.0f;   // mode 1: 独立坐标, 每个航点从原点开始
#endif
    // mode 0: 链式累积, bcn_pos 不重置 (B1=原点→B1, B2=B1→B2, ...)
    bcn_enc0[0] = motor_L1.total_encoder;
    bcn_enc0[1] = motor_L2.total_encoder;
    bcn_enc0[2] = motor_R1.total_encoder;
    bcn_enc0[3] = motor_R2.total_encoder;
    bcn_yaw = fused_yaw;   // 与PosUpdate同用磁力计融合航向 (记录/导航同一坐标系)
    printf("INAV: B%d record start, yaw=%.1f\n", bcn_idx + 1, bcn_yaw);
}

// 结束记录当前航点: 按编码器位移 + 记录起始航向累积绝对坐标 → 存 bcn_abs[bcn_idx++]
void Inav_RecEnd(void)
{
    if (bcn_idx >= bcn_max) return;   // 超出设置数量: 忽略
    float dist = enc2m(enc_delta(bcn_enc0));
    float rad  = bcn_yaw * (PI / 180.0f);
    bcn_pos.x += dist * cosf(rad);
    bcn_pos.y += dist * sinf(rad);
    bcn_abs[bcn_idx] = bcn_pos;
    printf("INAV: B%d dist=%.2fm abs(%.2f,%.2f)\n",
           bcn_idx + 1, dist, bcn_abs[bcn_idx].x, bcn_abs[bcn_idx].y);
    bcn_idx++;
}

// 全部录完: bcn_abs → wp_abs (含发车区航点) + 预计算每段距离/方向
void Inav_BuildMap(void)
{
    Inav_UpdateMax();
    bcn_build_waypoints();
}

// 读当前录制航点地图 (用于存 W25Q64)
const flash_wp_t* Inav_GetMap(void) { return bcn_abs; }

// 重算 bcn_max/wp_max (菜单设置变化后调用)
void Inav_UpdateMax(void)
{
    if (wp_set < 1)   wp_set = 1;
    if (wp_set > FLASH_WP_MAX) wp_set = FLASH_WP_MAX;
    bcn_max = wp_set;
    wp_max  = bcn_max + (launch_enable ? 1 : 0);
    if (wp_max > FLASH_WP_MAX + 1) wp_max = FLASH_WP_MAX + 1;
}

// 上电载入: 从 W25Q64 读设置(init.c 已写入菜单全局) + 航点地图 → 设上限 + 构建导航航点
void Inav_LoadMap(void)
{
    flash_wp_t wp[FLASH_WP_MAX];
    uint8_t n = FlashStore_LoadWaypoints(wp, FLASH_WP_MAX);

    Inav_UpdateMax();     // bcn_max = wp_set, wp_max = bcn_max + launch_enable

    bcn_idx = 0;
    for (uint8_t i = 0; i < n; i++) {
        if (i >= bcn_max) break;   // 只装载设置数量以内
        bcn_abs[i] = wp[i];
        bcn_idx++;
    }
    bcn_pos.x = 0.0f; bcn_pos.y = 0.0f;
    if (bcn_idx > 0) bcn_build_waypoints();
    printf("INAV: map loaded (%d wp), bcn_max=%d wp_max=%d\n", bcn_idx, bcn_max, wp_max);
}

// 清除历史: 清空内存航点 (W25Q64 由调用方 FlashStore_ClearAll)
void Inav_ResetMap(void)
{
    bcn_idx = 0;
    bcn_pos.x = 0.0f; bcn_pos.y = 0.0f;
    bcn_state = BCN_IDLE;
    patrol_active = 0;
    bcn_nav_on = 0;
    printf("INAV: map reset\n");
}

// 航点是否录齐 (达到设置数量)
uint8_t Inav_CanLaunch(void)
{
    return (bcn_idx >= bcn_max) ? 1 : 0;
}

// 武装发车: mission_armed=1 + 记发车时刻 + 开闭环 + 复位原点/航向
// 3.5s 延迟由 PositionControl_Update / 巡逻触发条件处理; 发车区航点为 wp_abs[bcn_max]
uint8_t Inav_Launch(void)
{
    if (!Inav_CanLaunch()) {
        printf("INAV: launch denied, only %d/%d wp recorded\n", bcn_idx, bcn_max);
        return 0;
    }
    mission_armed = 1;
    mission_arm_time = time_us;
    printf("INAV: Mission armed, waiting 3.5s for drone...\n");
    angle_target = 0.0f;
    target_vx = 0.0f; target_vy = 0.0f;
    Motor_Enable_PID(1);
    PID_Reset(&angle_pid_yaw);   PID_Enable(&angle_pid_yaw, 1);
    PID_Reset(&angle_pid_gyro);  PID_Enable(&angle_pid_gyro, 1);
    g_pos_x = 0.0f; g_pos_y = 0.0f;
    go_center = 0;
    bcn_nav_on = 0;
    patrol_active = 0;
    My_Imu660ra_ResetYaw();
    MagYaw_Reset();   // 2026-08-09: 发车归零磁力计融合, 车头=世界0° (与陀螺复位同步)
    return 1;
}

// 停止所有控制任务: 关闭环 + 零速 + 停桨(发3) + 完赛级锁定
// 与 race_done 同级: 后续不执行任何指令, 断电重启刷新状态
void Inav_StopAll(void)
{
    Motor_Enable_PID(0);
    PID_Enable(&angle_pid_yaw, 0);
    PID_Enable(&angle_pid_gyro, 0);
    target_vx = 0.0f; target_vy = 0.0f;
    bcn_nav_on = 0;
    patrol_active = 0;
    Motor_SetSpeed(&motor_L1,0,MotorL1_Turn,MotorL1_Pwm);
    Motor_SetSpeed(&motor_L2,0,MotorL2_Turn,MotorL2_Pwm);
    Motor_SetSpeed(&motor_R1,0,MotorR1_Turn,MotorR1_Pwm);
    Motor_SetSpeed(&motor_R2,0,MotorR2_Turn,MotorR2_Pwm);
    HC06_SendDroneCmd(3);       // 停桨指令 (占位: 当前遥控模拟, 恢复无人机通信后生效)
    race_done = 1;              // 与完赛同级: 小车将不执行任何指令, 断电重启刷新
    printf("INAV: STOP ALL, locked (race_done=1)\n");
}

/*==================================================== 主循环更新 ====================================================*/
void InertialNav_Update(void)
{
    if (race_done) return;   // 完赛: 不触发巡逻/导航
    if (bcn_debounce > 0) bcn_debounce--;

    // ---- 巡逻: flag=2 连续10帧才确认丢信标, 在3个航点间循环 ----
    // 触发条件: 已武装 + 丢信标确认 + 未在巡逻 + 消抖结束
    if (mission_armed && time_us - mission_arm_time >= 3500000
        && flag2_count >= FLAG2_DEBOUNCE && !patrol_active && bcn_debounce == 0) {
        float d_min = 1e9f;
        for (int i = 0; i < wp_max; i++) {
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
                patrol_wp = (patrol_wp + 1) % wp_max;   // 顺序循环
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

        if (wp_idx >= wp_max) {
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
