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
#define PIXEL_TO_VEL_SCALE       0.005f   // 像素误差 → 速度目标
#define BEACON_HOLD_FRAMES_MAX   100     // 丢失后保持帧数 (约500ms@200Hz)
#define BEACON_FADE_DECAY        0.92f   // 超时后每帧衰减系数 (越接近1衰减越慢)
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
    float THR;
   alt.target_height = TAKEOFF_TARGET_HEIGHT;
#if HEIGHT == 1
    PID_Update(&PIDHeight, alt.target_height, world_data.pz, dt);
    PID_Update(&PIDVelH, PIDHeight.out, world_data.vz, dt);
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
    // 安全起飞锁: 首次到达 1m 高度后才启用速度/位置环
    static uint8_t pos_ctrl_enabled = 0;
    if (!pos_ctrl_enabled && world_data.pz >= 1.0f) {
        pos_ctrl_enabled = 1;
    }
 
    if (++ctrl_div >= 5) {
        ctrl_div = 0;
        if (g_vision_share.car_found && pos_ctrl_enabled) {
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
                float vel_x = (car_x - prev_car_x) * 40.0f;  // 像素/帧→像素/秒
                float vel_y = (car_y - prev_car_y) * 40.0f;
                prev_car_x = car_x; prev_car_y = car_y;
                ff_vel_x = vel_x * PIXEL_TO_VEL_SCALE;  // 像素/秒→期望速度(m/s)
                ff_vel_y = vel_y * PIXEL_TO_VEL_SCALE;
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
            last_roll_tgt  = PIDRoll.target;
            last_pitch_tgt = PIDPitch.target;
            PIDRoll.target  = PIDPosY_Vel.out;
            PIDPitch.target = PIDPosX_Vel.out;
        } else {
            // 无小车: 保持水平0度，不跑位置环
            last_roll_tgt  = PIDRoll.target;
            last_pitch_tgt = PIDPitch.target;
            PIDRoll.target  = 0.0f;
            PIDPitch.target = 0.0f;
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
    PIDPosX_Vel.test = pitch_ff;
    PIDPosY_Vel.test = roll_ff;

    // Roll轴: 角度环(测量微分) + 角速度环(误差微分)
    PID_Update_d_measure(&PIDRoll, PIDRoll.target, eulerAngle.roll, dt);
    PID_Update(&PIDVelX, PIDRoll.out + roll_ff, imu_data.gyro_x_pt1, dt);

    // Pitch轴: 角度环(测量微分) + 角速度环(误差微分)
    PID_Update_d_measure(&PIDPitch, PIDPitch.target, eulerAngle.pitch, dt);
    PID_Update(&PIDVelY, PIDPitch.out + pitch_ff, imu_data.gyro_y_pt1, dt);

    // Yaw轴: 角度环(带最短路径处理) + 角速度环
    PID_Update_Yaw(&PIDYaw, PIDYaw.target, eulerAngle.yaw, dt);
    PID_Update(&PIDVelZ, PIDYaw.out, imu_data.gyro_z_pt1, dt);
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
// 起飞控制
// =============================================================================
// 功能: 自动起飞至目标高度
// 策略:
//   - 起飞前: 禁用磁力计融合 (避免地面磁场干扰)
//   - 离地阶段(step=1): 角度控制，仅保持水平
//   - 上升阶段(step=2): 逐渐升至目标高度，同时检测磁力计融合条件
//   - 完成阶段(step=3): 切换至定点悬停模式
// =============================================================================
void take_off(float dt)
{
    static uint8_t step = 0;
    static float vel_ki_saved = 0.0f;          // 保存原始速度环KI，起飞后恢复

    switch(step)
    {
        // ============ Step 0: 起飞前初始化 ============
        case 0:
            PIDYaw.target = eulerAngle.yaw;           // 锁当前偏航
            alt.target_height = TAKEOFF_TARGET_HEIGHT;   // 直接设为宏定义的目标高度
            PIDPosX.target = world_data.px;            // 锁当前X位置(起飞即定点)
            PIDPosY.target = world_data.py;            // 锁当前Y位置
            flag.takeoff_phase = 1;                    // 标记起飞阶段(光流无I)

            // 起飞阶段速度环纯P：清零积分避免起飞时累积
            vel_ki_saved = PIDPosX_Vel.ki;
            PIDPosX_Vel.ki = 0.0f;
            PIDPosX_Vel.integ = 0.0f;
            PIDPosY_Vel.ki = 0.0f;
            PIDPosY_Vel.integ = 0.0f;

            step = 1;
            stabilization(dt);                         // step 0也执行姿态控制
        break;

        // ============ Step 1: 5秒线性升至目标高度 ============
        case 1:
        {
            // 基础油门直接使用悬停油门，不做平滑
            takeoff_base_thr = HOVER_THRUST;

            // 线性增加目标高度：TAKEOFF_TARGET_HEIGHT/8 = 0.15m/s
            alt.target_height += (TAKEOFF_TARGET_HEIGHT / 8.0f) * dt;

            // 到达或超过目标高度 → 钳位并结束起飞
            if(alt.target_height >= TAKEOFF_TARGET_HEIGHT)
            {
                alt.target_height = TAKEOFF_TARGET_HEIGHT;
                flag.takeoff_phase = 0;                // 结束起飞阶段

                // 恢复速度环PI：稳定后加入积分消除静差
                PIDPosX_Vel.ki = vel_ki_saved;
                PIDPosY_Vel.ki = vel_ki_saved;

                flag.hover_lock = 1;                   // 预锁悬停点
                hover_lock();
                stabilization(dt);                     // 进入定点悬停
                return;
            }

            // 起飞即定点: 纯速度环，目标速度=0
            PIDPosX_Vel.target = 0.0f;
            PIDPosY_Vel.target = 0.0f;
            PID_Update(&PIDPosX_Vel, PIDPosX_Vel.target, world_data.vx, dt);
            PID_Update(&PIDPosY_Vel, PIDPosY_Vel.target, world_data.vy, dt);
            PIDRoll.target  = PIDPosX_Vel.out;
            PIDPitch.target = PIDPosY_Vel.out;
            
            stabilization(dt);
        }
        break;
    }
}


// pos_hold_control() 已移除 — 等价于 hover_lock() + stabilization(dt)