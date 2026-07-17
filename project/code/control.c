// =============================================================================
// control.c - 飞行控制实现
// =============================================================================
// 功能说明：
//   1. 飞行状态机管理 (LOCK/UNLOCK/TAKEOFF/FLY/LAND)
//   2. 姿态控制 (串级PID: 角度外环 + 角速度内环)
//   3. 电机混控 (X型四旋翼布局)
//   4. 起飞、降落、悬停逻辑
//   5. 磁力计融合切换策略 (地面禁用，空中启用)
// =============================================================================

#include "zf_common_headfile.h"
#include "camera.h"
#include "battery.h"    // 电池电压油门补偿

// 视觉共享内存 —— 定义在此处，CM7_1 (camera.c) 通过相同地址访问
#pragma location = VISION_SHARE_ADDR
vision_share_t g_vision_share;

// 滤波后的小车坐标 (cm7_0_isr.c 中 EMA 平滑, 供位置环使用)
extern float get_filtered_car_x(void);
extern float get_filtered_car_y(void);

// 定义一个视觉像素误差到期望速度的转换系数（根据实际调试缩放，初始可以给个小值）
#define PIXEL_TO_VEL_SCALE       0.000f   // 像素误差 → 速度目标 (废弃，改用动态)
#define FF_SCALE_MAX             0.00f   // 前馈最大比例 (低速时)
#define FF_SPEED_CEIL            0.4f     // 前馈截止速度 (m/s)，超过后前馈=0
#define BEACON_HOLD_FRAMES_MAX   100     // 丢失后保持帧数 (约500ms@200Hz)
#define BEACON_FADE_DECAY        0.93f   // 超时后每帧衰减系数 (越接近1衰减越慢)
#define PIXELS_PER_DEG           1.3f  // 像素/度 (实测 50px/16°=3.125)



// =============================================================================
// 电机布局说明 (X型四旋翼，俯视图)
// =============================================================================
//     1     2
//         X
//     4     3
//
// 电机1: 右前 (CCW, 逆时针)
// 电机2: 左前 (CW,  顺时针)
// 电机3: 左后 (CCW, 逆时针)
// 电机4: 右后 (CW,  顺时针)
//
// 姿态修正方向:
//   右倾 roll  > 0  →  左侧电机(2,3)加速，右侧电机(1,4)减速
//   前倾 pitch > 0  →  后方电机(3,4)加速，前方电机(1,2)减速
//   逆时针 yaw > 0  →  CCW电机(1,3)加速，CW电机(2,4)减速

// =============================================================================
// 控制环频率参考 (实际频率由中断决定，此处为推荐值)
// =============================================================================
// | 控制层               | 频率        | dt          |
// |---------------------|-------------|-------------|
// | 角速度环(PIDVelX/Y/Z)| 250~500Hz   | 0.002~0.004 |
// | 姿态环(Roll/Pitch/Yaw)| 100~200Hz | 0.005~0.01  |
// | 高度环               | 50~100Hz    | 0.01~0.02   |
// | 位置环               | 50Hz        | 0.02        |

uint8_t out_flag;   // 电机输出使能标志: 1=输出, 0=停转

float takeoff_base_thr = HOVER_THRUST;  // 起飞阶段基础油门，从3900平滑上升到HOVER_THRUST

// 无信标自动降落计时器
float g_no_beacon_timer = 0.0f;
uint8_t g_no_beacon_flag3_sent = 0;
volatile uint8_t g_car_flag3_pending = 0;    // 主循环检测此标志发送 "#0,0,3$"

// flight_control() — 完整状态机，当前未使用（已被 stabilization@200Hz 直接替代）
// 保留 flight_state 变量及枚举供 future use


// =============================================================================
// 姿态稳定控制 (自稳模式)
// =============================================================================
// 功能: 串级PID姿态控制 + 电机混控输出
//   - 外环(角度环): 角度误差 → 期望角速度
//   - 内环(角速度环): 角速度误差 → 电机修正量
//   - 混控: 将油门+PID修正量分配给4个电机
// =============================================================================
void stabilization(float dt)
{
    // 前馈值 (由 POSITION_HOLD==2 @40Hz 块写入，姿态控制块读取)
    static float pid_ff_roll = 0.0f, pid_ff_pitch = 0.0f;

    // ==============================================
    // out_flag 输出使能检查 (前置)
    // ==============================================
    if (out_flag == 0)
    {
        m1 = 0; m2 = 0; m3 = 0; m4 = 0;
        return;
    }

    // ==============================================
    // 姿态保护: Roll/Pitch超过90度立即停浆
    // ==============================================
    if (fabsf(eulerAngle.roll) > 90.0f || fabsf(eulerAngle.pitch) > 90.0f)
    {
        m1 = 0; m2 = 0; m3 = 0; m4 = 0;
        out_flag = 0;
        flight_state = STATE_LOCK;
        return;
    }

    // ---------- 高度控制 ----------
    // 注: alt.target_height 由 flight_control() 状态机管理
    float THR;
#if HEIGHT == 1
    if (flag.height_vel_only) {
        // 降落阶段1: 跳过高度外环，直接用速度环控速
        PID_Update(&PIDVelH, PIDVelH.target, world_data.vz, dt);
    } else {
        PID_Update(&PIDHeight, alt.target_height, world_data.pz, dt);
        PID_Update(&PIDVelH, PIDHeight.out, world_data.vz, dt);
    }
#endif
    THR = takeoff_base_thr + PIDVelH.out;

    // ---------- 电池电压油门补偿 ----------
    // 电压下降时增大基础油门，高度环输出不参与补偿
#if THROTTLE_ATTENUATION == 1
    {
        static uint8_t comp_inited = 0;
        if (!comp_inited) {
            Battery_GetThrottleComp();  // 首次初始化PT1
            comp_inited = 1;
        }
        float comp = Battery_GetThrottleComp();
        THR = takeoff_base_thr * comp + PIDVelH.out;
    }
#endif

    // ---------- 起飞阶段: 基础油门 3500→HOVER_THRUST 线性增加 (5秒) ----------
    // 只抬基础油门，高度 PID 输出不受限，电机仍有完整控制权
    {
        static float thr_ramp_timer = 0.0f;
        static uint8_t prev_phase = 0;
        if (flag.takeoff_phase) {
            if (!prev_phase) thr_ramp_timer = 0.0f;
            thr_ramp_timer += dt;
            float ramp_ratio = thr_ramp_timer / 1.5f;
            if (ramp_ratio > 1.0f) ramp_ratio = 1.0f;
            takeoff_base_thr = 3500.0f + (float)(HOVER_THRUST - 3500) * ramp_ratio;
        }
        prev_phase = flag.takeoff_phase;
    }

    // ---------- 位置/速度环 → 角度目标 (统一40Hz分频) ----------
    // 控制模式由 POSITION_HOLD 宏选择:
    //   0 = 禁用速度环, 目标角度=0(纯自稳)
    //   1 = 视觉追车, 位置外环+速度内环串级, 40Hz
    //   2 = 纯速度环悬停, 目标速度=0, 40Hz
    // PID每40Hz更新一次(与光流/视觉同步), 前馈平铺到200Hz
    static uint8_t ctrl_div = 0;
    static float last_roll_tgt = 0.0f, last_pitch_tgt = 0.0f;
    pid_ff_roll = 0.0f; pid_ff_pitch = 0.0f;

#if POSITION_HOLD == 0
    PIDRoll.target  = 0.0f;
    PIDPitch.target = 0.0f;
#elif POSITION_HOLD == 1
    if (++ctrl_div >= 5) {
        ctrl_div = 0;
        if (g_vision_share.car_found && flag.pos_ctrl_enabled) {
            float car_x = get_filtered_car_x();
            float car_y = get_filtered_car_y();
            // 姿态补偿: 前倾→car_x↑, 右倾→car_y↓
            car_x -= eulerAngle.pitch * PIXELS_PER_DEG;
            car_y += eulerAngle.roll  * PIXELS_PER_DEG;
            // 像素速度前馈: 计算小车在图像中的移动速度，叠加到速度环目标
            static float prev_car_x = 0, prev_car_y = 0;
            static uint8_t ff_inited = 0;
            float ff_vel_x = 0.0f, ff_vel_y = 0.0f;
            if (!ff_inited) {
                prev_car_x = car_x; prev_car_y = car_y;
                ff_inited = 1;
            } else {
                float dx = car_x - prev_car_x;
                float dy = car_y - prev_car_y;
                prev_car_x = car_x; prev_car_y = car_y;
                // 非真实检测(衰减值)或跳变>20px, 清零前馈
                if (!g_vision_share.car_fresh || fabsf(dx) > 20.0f) dx = 0.0f;
                if (!g_vision_share.car_fresh || fabsf(dy) > 20.0f) dy = 0.0f;
                // 动态前馈比例: 飞机速度>0.8m/s时线性衰减到0
                float speed = sqrtf(world_data.vx*world_data.vx + world_data.vy*world_data.vy);
                float ff_scale = FF_SCALE_MAX * (1.0f - speed / FF_SPEED_CEIL);
                if (ff_scale < 0.0f) ff_scale = 0.0f;
                ff_vel_x = dx * 40.0f * ff_scale;
                ff_vel_y = dy * 40.0f * ff_scale;
            }
            PIDPosX.target = car_x;
            PIDPosY.target = car_y;
            PID_Update(&PIDPosX, PIDPosX.target, (float)(CAMERA_W / 2), dt);
            PID_Update(&PIDPosY, PIDPosY.target, (float)(CAMERA_H / 2), dt);
            PID_Update(&PIDPosX_Vel, PIDPosX.out + ff_vel_x, world_data.vx, dt);
            PID_Update(&PIDPosY_Vel, PIDPosY.out + ff_vel_y, world_data.vy, dt);
            // 写共享内存供上位机显示
            g_vision_share.vel_tgt_x = PIDPosX_Vel.target;
            g_vision_share.vel_tgt_y = PIDPosY_Vel.target;
            g_vision_share.ff_vel_x = ff_vel_x;
            g_vision_share.ff_vel_y = ff_vel_y;
            last_roll_tgt  = PIDRoll.target;
            last_pitch_tgt = PIDPitch.target;
            PIDRoll.target  = PIDPosY_Vel.out;
            PIDPitch.target = PIDPosX_Vel.out;
        } else {
            // 无小车/未使能: 速度环 V=0 保持定点 (不跑位置环)
            PID_Update(&PIDPosX_Vel, 0, world_data.vx, dt);
            PID_Update(&PIDPosY_Vel, 0, world_data.vy, dt);
            last_roll_tgt  = PIDRoll.target;
            last_pitch_tgt = PIDPitch.target;
            PIDRoll.target  = PIDPosY_Vel.out;
            PIDPitch.target = PIDPosX_Vel.out;
        }
    }
    pid_ff_roll  = (PIDRoll.target  - last_roll_tgt)  * ANG_RATE_FF_GAIN / (5.0f * dt);
    pid_ff_pitch = (PIDPitch.target - last_pitch_tgt) * ANG_RATE_FF_GAIN / (5.0f * dt);
#elif POSITION_HOLD == 2
    if (++ctrl_div >= 5) {
        ctrl_div = 0;
        PID_Update(&PIDPosX_Vel, 0, world_data.vx, dt);
        PID_Update(&PIDPosY_Vel, 0, world_data.vy, dt);
        last_roll_tgt  = PIDRoll.target;
        last_pitch_tgt = PIDPitch.target;
        PIDRoll.target  = PIDPosY_Vel.out;
        PIDPitch.target = PIDPosX_Vel.out;
    }
    pid_ff_roll  = (PIDRoll.target  - last_roll_tgt)  * ANG_RATE_FF_GAIN / (5.0f * dt);
    pid_ff_pitch = (PIDPitch.target - last_pitch_tgt) * ANG_RATE_FF_GAIN / (5.0f * dt);
#endif

    // ---------- 姿态控制 (外环+内环) ----------
    // 可通过宏 ATTITUDE 开关: 1=启用, 0=禁用(只输出基础油门)
#if ATTITUDE == 1

    // ----- 角速度前馈: 角度目标变化率(40Hz)平铺到200Hz -----
    // 所有模式的前馈已在位置/速度环段统一计算(pid_ff_roll/pitch)
    float roll_ff  = pid_ff_roll;
    float pitch_ff = pid_ff_pitch;
    if (flag.takeoff_phase) {
        roll_ff = 0.0f;     // 起飞阶段禁用角速度前馈，
        pitch_ff = 0.0f;    // 避免速度环Ki清零后前馈干扰姿态
    }
    PIDPosX_Vel.test = pitch_ff;
    PIDPosY_Vel.test = roll_ff;

    // Roll轴: 角度环(测量微分) + 角速度环(误差微分)
    PID_Update_d_measure(&PIDRoll, PIDRoll.target, eulerAngle.roll, dt);
    PID_Update(&PIDVelX, PIDRoll.out + roll_ff, imu_data.gyro_x_pt1, dt);

    // Pitch轴: 角度环(测量微分) + 角速度环(误差微分)
    PID_Update_d_measure(&PIDPitch, PIDPitch.target, eulerAngle.pitch, dt);
    PID_Update(&PIDVelY, PIDPitch.out + pitch_ff, imu_data.gyro_y_pt1, dt);

    // Yaw轴: 由 flight_control() 状态机管理 flag.yaw_angle_enabled
    //   0 = 起飞阶段: 仅角速度环维持偏航角速度为0, 防磁力计干扰导致自旋
    //   1 = 正常飞行: 角度环(带最短路径) + 角速度环
    if (flag.yaw_angle_enabled) {
        PID_Update_Yaw(&PIDYaw, PIDYaw.target, eulerAngle.yaw, dt);
        PID_Update(&PIDVelZ, PIDYaw.out, imu_data.gyro_z_pt1, dt);
    } else {
        PID_Update(&PIDVelZ, 0.0f, imu_data.gyro_z_pt1, dt);
    }
#endif

    // ==============================================
    // 电机混控: 将油门+姿态修正量分配给4个电机
    // ==============================================

#if BASE_SPEED == 1
    // ----- 模式1: 基础油门 + PID修正 -----
    // Roll修正: 左侧(2,3)为正，右侧(1,4)为负
    // Pitch修正: 后方(3,4)为正，前方(1,2)为负
    // Yaw修正:  CCW(1,3)为正，CW(2,4)为负
    // 注意: 先在有符号域计算，避免 uint16_t 转型负值溢出导致尖峰
    {
        int16_t mix1 = (int16_t)THR + (int16_t)PIDVelX.out - (int16_t)PIDVelY.out - (int16_t)PIDVelZ.out;
        int16_t mix2 = (int16_t)THR - (int16_t)PIDVelX.out - (int16_t)PIDVelY.out + (int16_t)PIDVelZ.out;
        int16_t mix3 = (int16_t)THR - (int16_t)PIDVelX.out + (int16_t)PIDVelY.out - (int16_t)PIDVelZ.out;
        int16_t mix4 = (int16_t)THR + (int16_t)PIDVelX.out + (int16_t)PIDVelY.out + (int16_t)PIDVelZ.out;
        m1 = (uint16_t)LIMIT(mix1, (int16_t)MOTOR_MIN_DUTY, (int16_t)MOTOR_MAX_DUTY);
        m2 = (uint16_t)LIMIT(mix2, (int16_t)MOTOR_MIN_DUTY, (int16_t)MOTOR_MAX_DUTY);
        m3 = (uint16_t)LIMIT(mix3, (int16_t)MOTOR_MIN_DUTY, (int16_t)MOTOR_MAX_DUTY);
        m4 = (uint16_t)LIMIT(mix4, (int16_t)MOTOR_MIN_DUTY, (int16_t)MOTOR_MAX_DUTY);
    }
#elif BASE_SPEED == 2
    // ----- 模式2: 纯基础油门(调试用) -----
    m1 = (uint16_t)THR;
    m2 = (uint16_t)THR;
    m3 = (uint16_t)THR;
    m4 = (uint16_t)THR;
#endif

    m1 = LIMIT(m1, MOTOR_MIN_DUTY, MOTOR_MAX_DUTY);
    m2 = LIMIT(m2, MOTOR_MIN_DUTY, MOTOR_MAX_DUTY);
    m3 = LIMIT(m3, MOTOR_MIN_DUTY, MOTOR_MAX_DUTY);
    m4 = LIMIT(m4, MOTOR_MIN_DUTY, MOTOR_MAX_DUTY);

    // 注: 电机实际输出通过 small_driver_set_duty 在外部调用
}


// position_control() 已移除 — 功能由 stabilization() 内 POSITION_HOLD 编译宏统一管理


// =============================================================================
// 悬停锁点
// =============================================================================
// 功能: 在第一次进入悬停时，锁定当前位置和高度作为控制目标
// 说明: 这样飞机会保持在解锁时所在的位置，而不是飞到坐标原点
// =============================================================================
void hover_lock(void)
{
    if(!flag.hover_lock)                    // 尚未锁点
    {
        PIDPosX.target = world_data.px;     // 锁当前X位置
        PIDPosY.target = world_data.py;     // 锁当前Y位置
        alt.target_height = world_data.pz;  // 锁当前高度

        flag.hover_lock = 1;                // 标记已锁点
    }
}


// hover_control() 已移除 — 等价于 hover_lock() + stabilization(dt)


// =============================================================================
// 飞行状态机 (取代旧的 take_off)
// =============================================================================
// 功能: 管理整个飞行过程的状态切换
// 状态:
//   STATE_LOCK    — 停浆，复位所有标志
//   STATE_UNLOCK  — 电机测试 (通过 #1$ 指令进入)
//   STATE_TAKEOFF — 起飞爬升 (通过 #2$ 指令进入)
//   STATE_FLY     — 正常飞行 (完成起飞后自动切换)
//
// 解耦设计:
//   flight_control() 管理飞行阶段、设置 PID 目标和标志
//   stabilization()  是纯控制器，不含任何状态逻辑
// =============================================================================
void flight_control(float dt)
{
    switch(flight_state)
    {
        // ============ STATE_LOCK: 停浆 ============
        case STATE_LOCK:
            out_flag = 0;
            m1 = 0; m2 = 0; m3 = 0; m4 = 0;
            // 复位所有飞行阶段标志
            flag.yaw_angle_enabled  = 0;
            flag.pos_ctrl_enabled   = 0;
            flag.takeoff_phase      = 0;
            flag.hover_lock         = 0;
            flag.height_vel_only    = 0;
            g_no_beacon_timer       = 0.0f;
            g_no_beacon_flag3_sent  = 0;
            // 恢复角速度 Kp 到配置默认值 (降落阶段2可能增加了0.5)
            PIDVelX.kp              = VELX_KP;
            PIDVelY.kp              = VELY_KP;
            PIDVelZ.kp              = VELZ_KP;
            // 恢复水平速度环 Kp (降落阶段2可能增加了1)
            PIDPosX_Vel.kp          = POS_VELX_KP;
            PIDPosY_Vel.kp          = POS_VELY_KP;
            takeoff_base_thr        = HOVER_THRUST;
            break;

        // ============ STATE_UNLOCK: 电机测试 ============
        case STATE_UNLOCK:
            out_flag = 1;
            stabilization(dt);      // 运行姿态控制，但主循环中 out_flag==1 会覆盖为固定占空比
            break;

        // ============ STATE_TAKEOFF: 起飞爬升 ============
        case STATE_TAKEOFF:
        {
            out_flag = 2;
            flag.yaw_angle_enabled  = 0;     // 起飞阶段: 偏航仅角速度环 (目标=0)，防止磁力计干扰导致自旋
            flag.pos_ctrl_enabled   = 0;     // 起飞阶段: 垂直爬升, 不追车
            flag.takeoff_phase      = 1;     // 光流无积分

            static uint8_t step = 0;

            // Step 0: 初始化 (仅在进入 TAKEOFF 时执行一次)
            if (step == 0) {
                PIDYaw.target       = eulerAngle.yaw;             // 锁初始偏航
                alt.target_height   = 0.0f;                       // 从0开始线性升高
                PIDPosX.target      = world_data.px;              // 锁当前位置
                PIDPosY.target      = world_data.py;

                // 速度环去积分，避免起飞时累积 (到达目标高度后恢复)
                PIDPosX_Vel.ki      = 0.0f;
                PIDPosX_Vel.integ   = 0.0f;
                PIDPosY_Vel.ki      = 0.0f;
                PIDPosY_Vel.integ   = 0.0f;

                step = 1;
            }

            // Step 1: 线性爬升
            // 注: takeoff_base_thr 由 stabilization() 内 ramp 代码控制 (3500→HOVER_THRUST 1.5s)
            // 此处不赋值，避免覆盖 ramp 结果
            alt.target_height += (TAKEOFF_TARGET_HEIGHT / 1.5f) * dt;

            // 到达1m实际高度: 锁定当前偏航，恢复角度环
            if (!flag.yaw_angle_enabled && world_data.pz >= 1.0f) {
                flag.yaw_angle_enabled = 1;
                PIDYaw.target = eulerAngle.yaw;   // 锁当前真实物理偏航
            }

            // 到达目标高度: 结束起飞，进入正常飞行
            if (alt.target_height >= TAKEOFF_TARGET_HEIGHT) {
                alt.target_height = TAKEOFF_TARGET_HEIGHT;
                flag.takeoff_phase = 0;

                // 恢复速度环积分 (直接用宏常量，无需保存)
                PIDPosX_Vel.ki = POS_VEL_KI;
                PIDPosY_Vel.ki = POS_VEL_KI;

                flag.hover_lock = 1;
                hover_lock();

                step = 0;                           // 复位 substep，支持再次起飞
                flight_state = STATE_FLY;
                break;
            }

            stabilization(dt);      // 所有控制由 stabilization 统一处理 (速度环 V=0 / 追车)
            break;
        }

        // ============ STATE_FLY: 正常飞行 ============
        case STATE_FLY:
            out_flag = 2;
            flag.pos_ctrl_enabled = 1;                    // 开启位置环追车
            alt.target_height = TAKEOFF_TARGET_HEIGHT;    // 维持目标高度

            // 若还未使能偏航角度环（理论上已完成，但兜底处理）
            if (!flag.yaw_angle_enabled && world_data.pz >= 1.0f) {
                flag.yaw_angle_enabled = 1;
                PIDYaw.target = eulerAngle.yaw;
            }

            // ---------- 10秒自动降落 ----------
            g_no_beacon_timer += dt;
            if (g_no_beacon_timer >= NO_BEACON_LAND_TIMEOUT && !g_no_beacon_flag3_sent) {
                g_no_beacon_flag3_sent = 1;
                g_car_flag3_pending = 1;             // 主循环发送 flag=3
                printf("[AUTO] 10s timeout, landing...\r\n");
                flight_state = STATE_LAND;
            }

            stabilization(dt);
            break;

        // ============ STATE_LAND: 降落 ============
        case STATE_LAND:
            land(dt);
            break;
    }
}


// ============ land(): 降落函数 ============
// 阶段1: 高度速度环 0.1m/s 下降至 0.3m
// 阶段2: 锁定当前油门，4s线性衰减至0 + 近地锁降判定
void land(float dt)
{
    static uint8_t step = 0;
    static uint8_t ground_detect_cnt = 0;
    static float lock_thr = 0.0f;
    static float decay_timer = 0.0f;
    static float saved_kp[3] = {0};  // VelX, VelY, VelZ
    static float saved_posvel_kp[2] = {0};  // PosX_Vel, PosY_Vel

    switch (step) {
        case 0:     // ---------- 初始化 ----------
            flag.yaw_angle_enabled = 0;        // 偏航仅角速度环，不自旋
            flag.height_vel_only   = 1;        // 高度速度环直控
            takeoff_base_thr = HOVER_THRUST;

            alt.target_height = world_data.pz;  // 保持当前高度目标不变

            // 保存原始角速度 Kp 和速度环 Kp，阶段2需要增加
            saved_kp[0] = PIDVelX.kp;
            saved_kp[1] = PIDVelY.kp;
            saved_kp[2] = PIDVelZ.kp;
            saved_posvel_kp[0] = PIDPosX_Vel.kp;
            saved_posvel_kp[1] = PIDPosY_Vel.kp;

            // 设速度环目标 -0.2m/s (向下)
            PIDVelH.target = -0.2f;
            PIDVelH.integ  = 0.0f;

            // 水平速度环去积分
            PIDPosX_Vel.ki    = 0.0f;
            PIDPosX_Vel.integ = 0.0f;
            PIDPosY_Vel.ki    = 0.0f;
            PIDPosY_Vel.integ = 0.0f;
            PIDPosX_Vel.target = 0.0f;
            PIDPosY_Vel.target = 0.0f;

            PIDYaw.target = eulerAngle.yaw;
            ground_detect_cnt = 0;
            decay_timer = 0.0f;

            step = 1;
            break;

        case 1:     // ---------- 阶段1: 速度环-0.1m/s下降至0.3m ----------
            // stabilization() 中 flag.height_vel_only 生效，直接跟踪 PIDVelH.target = -0.1f

            // 水平速度保持 (纯 Kp)
            PID_Update(&PIDPosX_Vel, 0.0f, world_data.vx, dt);
            PID_Update(&PIDPosY_Vel, 0.0f, world_data.vy, dt);
            PIDRoll.target  = PIDPosX_Vel.out;
            PIDPitch.target = PIDPosY_Vel.out;

            stabilization(dt);

            // 实际高度到达 0.3m → 冻结油门，进入衰减
            if (world_data.pz <= 0.3f) {
                flag.height_vel_only = 0;       // 恢复串级模式
                // 角速度 Kp 增加 0.5，增强近地抗风能力
                PIDVelX.kp = saved_kp[0] + 0.7f;
                PIDVelY.kp = saved_kp[1] + 0.7f;
                PIDVelZ.kp = saved_kp[2] + 0.7f;
                // 水平速度环 Kp 增加 1，近地抗风更强
                PIDPosX_Vel.kp = saved_posvel_kp[0] + 1.0f;
                PIDPosY_Vel.kp = saved_posvel_kp[1] + 1.0f;
                lock_thr = takeoff_base_thr + PIDVelH.out;
                takeoff_base_thr = lock_thr;
                PIDHeight.integ = 0.0f;
                PIDVelH.integ   = 0.0f;
                alt.target_height = world_data.pz;
                step = 2;
            }
            break;

        case 2:     // ---------- 阶段2: 2s油门衰减至0 + 近地锁降 ----------
            decay_timer += dt;
            takeoff_base_thr = lock_thr * (1.0f - decay_timer / 2.0f);
            if (takeoff_base_thr < 0.0f) takeoff_base_thr = 0.0f;

            // 冻结高度环，让高度环不反向修正
            alt.target_height = world_data.pz;
            PIDHeight.integ = 0.0f;
            PIDVelH.integ   = 0.0f;

            // 水平速度保持 (纯 Kp)
            PID_Update(&PIDPosX_Vel, 0.0f, world_data.vx, dt);
            PID_Update(&PIDPosY_Vel, 0.0f, world_data.vy, dt);
            PIDRoll.target  = PIDPosX_Vel.out;
            PIDPitch.target = PIDPosY_Vel.out;

            stabilization(dt);

            // 接地检测: |vz| < 0.1m/s 且 ToF 高度 < 0.2m → 锁桨
            if (fabsf(world_data.vz) < 0.1f && of.height < 0.2f) {
                ground_detect_cnt++;
                if (ground_detect_cnt > 20) {   // 100ms 防抖
                    out_flag = 0;
                    flight_state = STATE_LOCK;
                    step = 0;
                }
            } else {
                ground_detect_cnt = 0;
            }
            break;
    }
}

// pos_hold_control() 已移除 — 等价于 hover_lock() + stabilization(dt)