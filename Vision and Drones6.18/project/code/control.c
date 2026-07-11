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

// 定义一个视觉像素误差到期望速度的转换系数（根据实际调试缩放，初始可以给个小值）
#define PIXEL_TO_VEL_SCALE       0.02f   // 像素误差 → 速度目标
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

// =============================================================================
// 飞行状态机 - 顶层调度函数
// =============================================================================
// 功能: 根据当前飞行状态，调度相应的控制函数
// 调用频率: 200Hz (由PIT定时中断触发)
// 
// 状态流转:
//   LOCK → (解锁命令) → UNLOCK
//   UNLOCK → (上锁命令) → LOCK
//   UNLOCK → (起飞命令) → TAKEOFF
//   TAKEOFF → (飞到目标高度) → FLY
//   FLY → (降落命令) → LAND
//   LAND → (落地检测) → LOCK
// =============================================================================
void flight_control(float dt)
{
    static flight_state_e last_flight_state = STATE_LOCK;
    
    // ---------- 状态变化时重置PID ----------
    // 每个飞行状态有不同的控制目标，切换时必须清空PID历史值
    // 防止上一状态的积分累积在新状态中产生冲击
    // 注意: 保留高度环PID(PIDHeight[6], PIDVelH[7])，防止切换后掉高
    if(flight_state != last_flight_state)
    {
        _PID_param_st height_save[2];
        height_save[0] = *pPidObject[6];     // 保存PIDHeight
        height_save[1] = *pPidObject[7];     // 保存PIDVelH

        PID_Rest_Init (pPidObject, 12);      // 重置全部12个PID控制器

        *pPidObject[6] = height_save[0];     // 恢复PIDHeight
        *pPidObject[7] = height_save[1];     // 恢复PIDVelH

        flag.hover_lock = 0;              // 清除悬停锁点标记
        flag.take_off_yaw = 0;            // 清除起飞偏航锁标记

        last_flight_state = flight_state;
    }

    // ---------- 紧急停止处理 ----------
    // 最高优先级：遥控器紧急指令立即停车
    if(rc.emergency_cmd)
    {
        small_driver_set_duty(0, 0, 0, 0);  // 全部电机停转
        flight_state = STATE_LOCK;
        return;
    }

    // ---------- 状态机分发 ----------
    switch(flight_state)
    {
        // ===================================
        // STATE_LOCK: 上锁状态
        // 功能: 电机停转，等待解锁命令
        // ===================================
        case STATE_LOCK:
        {
            flag.lock = 1;
            flag.unlock = 0;
            flag.mag_fusion_enabled = 1;
            m1 = 0; m2 = 0; m3 = 0; m4 = 0;   // 清零避免main loop覆盖
            small_driver_set_duty(0, 0, 0, 0);
        }
        break;
            

        // ===================================
        // STATE_UNLOCK: 解锁状态
        // 功能: 电机怠速，等待起飞命令
        // ===================================
        case STATE_UNLOCK:
            flag.lock = 0;
            flag.unlock = 1;
            
            PIDRoll.target = 0;
            PIDPitch.target = 0;
            
            rc.unlock_cmd = 0;                 // 清除解锁命令
            
            // 遥控器控制已关闭，状态切换由命令#1~#3控制
            // if(rc.lock_cmd)                    // 收到上锁命令 → 回到上锁状态
            // {
            //     flight_state = STATE_LOCK;
            // }
            // if(rc.takeoff_cmd)                 // 收到起飞命令 → 进入起飞状态
            // {
            //     flight_state = STATE_TAKEOFF;
            // }
            
            stabilization(dt);                 // 计算电机输出 (out_flag=1时强制3900)
            break;
            
        // ===================================
        // STATE_TAKEOFF: 起飞状态
        // 功能: 自动起飞至目标高度
        // ===================================
        case STATE_TAKEOFF:
            take_off(dt);
            break;
    }
}


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
     
    // ---------- DEBUG模式: 纯角速度控制(目标=0) ----------
    // 用于测试角速度环是否正常工作
#if DEBUG    
//    PID_Update(&PIDVelX, 0, imu_data.gyro_x, dt);
//    PID_Update(&PIDVelY, 0, imu_data.gyro_y, dt);
//    PID_Update(&PIDVelY, 0, imu_data.gyro_z, dt);

#endif

    // ---------- 高度控制 ----------
    float THR;
    
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

    // ---------- 视觉位置控制：小车坐标=目标，屏幕中心=测量值 ----------
    // 串级：位置误差 → 位置PID → 期望速度 → 速度PID → Roll/Pitch目标角
#if POSITION_HOLD == 1
    {
        // 测量值：屏幕中心 = 无人机自身位置
     
        // 目标值：视觉识别的小车坐标
        if(g_vision_share.car_found)
        {
            PIDPosX.target = (float)g_vision_share.car_x;
            PIDPosY.target = (float)g_vision_share.car_y;
        }
        else
        {
            // 未检测到小车：目标=测量值(保持原位)
            PIDPosX.target = world_data.px;
            PIDPosY.target = world_data.py;
        }

        // 位置外环：位置误差 → 期望速度
        PID_Update(&PIDPosX, PIDPosX.target, (float)(CAMERA_W / 2), dt);
        PID_Update(&PIDPosY, PIDPosY.target, (float)(CAMERA_H / 2), dt);

        // 速度内环：期望速度 → 期望角度
        PID_Update(&PIDPosX_Vel, PIDPosX.out, world_data.vx, dt);
        PID_Update(&PIDPosY_Vel, PIDPosY.out, world_data.vy, dt);

        PIDRoll.target  = PIDPosY_Vel.out;
        PIDPitch.target = PIDPosX_Vel.out;
    }
#endif

    // ---------- 姿态控制 (外环+内环) ----------
    // 可通过宏 ATTITUDE 开关: 1=启用, 0=禁用(只输出基础油门)
#if ATTITUDE == 1   
    
    // ----- 角速度前馈: 目标角度变化率直接加到角速度环 -----
    // 跳过角度环延迟，提升响应速度
    static float last_roll_target = 0.0f;
    static float last_pitch_target = 0.0f;
    float roll_ff = (PIDRoll.target - last_roll_target) / dt * ANG_RATE_FF_GAIN;
    float pitch_ff = (PIDPitch.target - last_pitch_target) / dt * ANG_RATE_FF_GAIN;
    last_roll_target = PIDRoll.target;
    last_pitch_target = PIDPitch.target;
    
    // Roll轴: 角度环(测量微分) + 角速度环(误差微分)
    PID_Update_d_measure(&PIDRoll, PIDRoll.target, eulerAngle.roll, dt);           
    PID_Update(&PIDVelX, PIDRoll.out + roll_ff, imu_data.gyro_x_pt1, dt);      
    
    // Pitch轴: 角度环(测量微分) + 角速度环(误差微分)
    PID_Update_d_measure(&PIDPitch, PIDPitch.target, eulerAngle.pitch, dt);         
    PID_Update(&PIDVelY, PIDPitch.out + pitch_ff, imu_data.gyro_y_pt1, dt);        
    anti_gravity_boost = 0.0f;
    
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

//     m1 = (uint16_t)THR + (uint16_t)PIDVelX.out - (uint16_t)PIDVelY.out - (uint16_t)PIDVelZ.out; // 右前
//    m2 = (uint16_t)THR - (uint16_t)PIDVelX.out - (uint16_t)PIDVelY.out + (uint16_t)PIDVelZ.out; // 左前
//    m3 = (uint16_t)THR - (uint16_t)PIDVelX.out + (uint16_t)PIDVelY.out - (uint16_t)PIDVelZ.out; // 左后
//    m4 = (uint16_t)THR + (uint16_t)PIDVelX.out + (uint16_t)PIDVelY.out + (uint16_t)PIDVelZ.out;
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
    
    // ---------- 输出使能控制 ----------
    // out_flag=0时强制输出0(停转)，out_flag=1时正常输出混控值
    if(out_flag == 1)
    {
          m1=3500;
          m2=3500;
          m3=3500;
          m4=3500;  
    }
    else if(out_flag==2)
    {
        // 正常输出 (由混控公式计算)
    }
    else if(out_flag == 0)
    {
        m1 = 0;
        m2 = 0;
        m3 = 0;
        m4 = 0;
    }

    // 注: 电机实际输出通过 small_driver_set_duty 在外部调用
}


// =============================================================================
// 位置控制 (串级PID)
// =============================================================================
// 功能: 将位置误差转换为姿态目标角
//   - 外环(位置环): 位置误差 → 期望速度
//   - 中环(速度环): 速度误差 → 期望角度
// =============================================================================
void position_control(float dt)
{   
    // ----- 位置外环: XY位置 → 期望速度 -----
    PID_Update(&PIDPosX, PIDPosX.target, world_data.px, dt);
    PID_Update(&PIDPosY, PIDPosY.target, world_data.py, dt);
    
    // ----- 速度内环: 期望速度 → 期望角度 -----
    // 注意: X方向移动对应Roll倾斜，Y方向移动对应Pitch倾斜
    PID_Update(&PIDPosX_Vel, PIDPosX.out, world_data.vx, dt);  // X速度 → Roll目标
    PID_Update(&PIDPosY_Vel, PIDPosY.out, world_data.vy, dt);  // Y速度 → Pitch目标
    
    // ----- 将速度环输出赋给姿态环目标 -----
    PIDRoll.target  = PIDPosX_Vel.out;
    PIDPitch.target = PIDPosY_Vel.out;
}


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


// =============================================================================
// 悬停控制
// =============================================================================
// 功能: 位置保持 + 姿态自稳
// =============================================================================
void hover_control(float dt)
{
    hover_lock();              // 锁定悬停点
    position_control(dt);      // 位置 → 姿态目标
    stabilization(dt);         // 姿态 → 电机输出
}


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
            alt.target_height = 0.0f;                  // 从0开始线性增长到目标高度
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

            // 线性增加目标高度：TAKOFF_TARGET_HEIGHT/5 = 0.2m/s
            alt.target_height += (TAKEOFF_TARGET_HEIGHT / 5.0f) * dt;

            // 到达或超过目标高度 → 钳位并结束起飞
            if(alt.target_height >= TAKEOFF_TARGET_HEIGHT)
            {
                alt.target_height = TAKEOFF_TARGET_HEIGHT;
                flag.takeoff_phase = 0;                // 结束起飞阶段

                // 恢复速度环PI：稳定后加入积分消除静差
                PIDPosX_Vel.ki = vel_ki_saved;
                PIDPosY_Vel.ki = vel_ki_saved;

                flag.hover_lock = 1;                   // 预锁悬停点
                pos_hold_control(dt);                  // 进入定点悬停
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


// =============================================================================
// 定点位置保持控制 (视觉追踪小车)
// =============================================================================
// 功能: 无人机识别小车后追踪
// 策略:
//   - 首次进入: 锁定高度
//   - 位置环由 stabilization 内 POSITION_HOLD 段统一处理
//   - 小车坐标=目标值，屏幕中心=测量值
// =============================================================================
void pos_hold_control(float dt)
{
    // ----- 首次进入锁点(高度锁) -----
    hover_lock();

    // 位置环已移入 stabilization 的 POSITION_HOLD 段处理
    stabilization(dt);
}