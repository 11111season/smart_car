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
extern uint8_t key_val1;
void flight_control(float dt)
{
    static flight_state_e last_flight_state = STATE_LOCK;
    
    // ---------- 状态变化时重置PID ----------
    // 每个飞行状态有不同的控制目标，切换时必须清空PID历史值
    // 防止上一状态的积分累积在新状态中产生冲击
    if(flight_state != last_flight_state)
    {
        PID_Rest_Init (pPidObject, 12);   // 重置全部12个PID控制器
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
            small_driver_set_duty(0, 0, 0, 0);

            // 按键检测 (例如按键1，即你之前的 key_val1)
            if(key_val1 == 1)
            {
                out_flag = 1;                    // 使能电机输出
                flight_state = STATE_TAKEOFF;    // 直接进入起飞状态
                key_val1 = 0;                    // 清除按键标志
                PID_Rest_Init(pPidObject, 12);   // 初始化PID
            }
        }
        break;
            

        // ===================================
        // STATE_UNLOCK: 解锁状态
        // 功能: 电机怠速，等待起飞命令
        // ===================================
        case STATE_UNLOCK:
            flag.lock = 0;
            flag.unlock = 1;
            
            rc.unlock_cmd = 0;                 // 清除解锁命令
            
            if(rc.lock_cmd)                    // 收到上锁命令 → 回到上锁状态
            {
                flight_state = STATE_LOCK;
            }
            if(rc.takeoff_cmd)                 // 收到起飞命令 → 进入起飞状态
            {
                flight_state = STATE_TAKEOFF;
            }
            break;
            
        // ===================================
        // STATE_TAKEOFF: 起飞状态
        // 功能: 自动起飞至目标高度
        // ===================================
        case STATE_TAKEOFF:
            take_off(dt);
            break;
            
        // ===================================
        // STATE_FLY: 飞行状态
        // 功能: 定点悬停 + 遥控器微调位置
        // ===================================
        case STATE_FLY:
            pos_hold_control(dt);              // 定点位置保持控制
            
            if(rc.land_cmd)                    // 收到降落命令 → 进入降落状态
            {
                flight_state = STATE_LAND;
            }
            break;
            
        // ===================================
        // STATE_LAND: 降落状态
        // 功能: 自动降落至地面并上锁
        // ===================================
        case STATE_LAND:
            land(dt);
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
     
    // ---------- DEBUG模式: 纯角速度控制(目标=0) ----------
    // 用于测试角速度环是否正常工作
#if DEBUG    
   // PID_Update(&PIDVelX, 0, imu_data.gyro_x, dt);
   PID_Update(&PIDVelH,0,world_data.vz,dt);
#endif

    // ---------- 高度控制 ----------
    // 可通过宏 HEIGHT 开关: 1=启用, 0=禁用(使用固定油门)
    float THR;
#if HEIGHT == 1 
    PIDHeight.target = TAKEOFF_TARGET_HEIGHT;
    PID_Update(&PIDHeight, PIDHeight.target, world_data.pz, dt);
    PID_Update(&PIDVelH, PIDHeight.out, world_data.vz, dt);
#endif
    THR = HOVER_THRUST + PIDVelH.out;

    // ---------- 位置控制 (视觉伺服调试) ----------
    // 可通过宏 POSITION_HOLD 开关: 1=启用, 0=禁用
#if POSITION_HOLD == 1
    {
//        float error_x = (float)CAR_TARGET_X - (CAMERA_W / 2.0f);
//        float error_y = (float)CAR_TARGET_Y - (CAMERA_H / 2.0f);
        PIDPosX.target = 0.0f;
        PIDPosY.target = 0.0f;
        PID_Update(&PIDPosX, PIDPosX.target, world_data.px, dt);
        PID_Update(&PIDPosY, PIDPosY.target, world_data.py, dt);
        PID_Update(&PIDPosX_Vel, PIDPosX.out, world_data.vx, dt);
        PID_Update(&PIDPosY_Vel, PIDPosY.out, world_data.vy, dt);
        PIDRoll.target  = PIDPosX_Vel.out;
        PIDPitch.target = PIDPosY_Vel.out;
    }
#endif

    // ---------- 姿态控制 (外环+内环) ----------
    // 可通过宏 ATTITUDE 开关: 1=启用, 0=禁用(只输出基础油门)
#if ATTITUDE == 1   
    // Roll轴: 角度环(测量微分) + 角速度环(误差微分)
    anti_gravity_update(THR, dt);
    PID_Update_d_measure(&PIDRoll, PIDRoll.target, eulerAngle.roll, dt);           
    PID_Update(&PIDVelX, PIDRoll.out, imu_data.gyro_x, dt);      
    
    // Pitch轴: 角度环(测量微分) + 角速度环(误差微分)
    PID_Update_d_measure(&PIDPitch, PIDPitch.target, eulerAngle.pitch, dt);         
    PID_Update(&PIDVelY, PIDPitch.out, imu_data.gyro_y, dt);        
    anti_gravity_boost = 0.0f;
    
    // Yaw轴: 角度环(带最短路径处理) + 角速度环
    PID_Update_Yaw(&PIDYaw, PIDYaw.target, eulerAngle.yaw, dt);            
    PID_Update(&PIDVelZ, PIDYaw.out, imu_data.gyro_z, dt);  
#endif    

    // ==============================================
    // 电机混控: 将油门+姿态修正量分配给4个电机
    // ==============================================

#if BASE_SPEED == 1     
    // ----- 模式1: 基础油门 + PID修正 -----
    // Roll修正: 左侧(2,3)为正，右侧(1,4)为负
    // Pitch修正: 后方(3,4)为正，前方(1,2)为负
    // Yaw修正:  CCW(1,3)为正，CW(2,4)为负
    m1 = (uint16_t)((int16_t)THR + (int16_t)PIDVelX.out - (int16_t)PIDVelY.out - (int16_t)PIDVelZ.out);
    m2 = (uint16_t)((int16_t)THR - (int16_t)PIDVelX.out - (int16_t)PIDVelY.out + (int16_t)PIDVelZ.out);
    m3 = (uint16_t)((int16_t)THR - (int16_t)PIDVelX.out + (int16_t)PIDVelY.out - (int16_t)PIDVelZ.out);
    m4 = (uint16_t)((int16_t)THR + (int16_t)PIDVelX.out + (int16_t)PIDVelY.out + (int16_t)PIDVelZ.out);

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
        m1 = 3900;
        m2 = 3900;
        m3 = 3900;
        m4 = 3900;
    }
    else if(out_flag == 2)
    {
      
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
// 磁力计融合切换检测
// =============================================================================
// 功能: 检测飞机是否已飞离地面并稳定，决定是否启用磁力计融合
// 策略:
//   - 地面阶段: 禁用磁力计融合 (避免地面磁场畸变干扰)
//   - 起飞后: 达到一定高度且姿态稳定 → 启用磁力计融合
// 调用: 在起飞过程中周期性调用
// =============================================================================
void check_mag_switch(float dt)
{
    static float stable_timer = 0;      // 稳定计时器(秒)
    static uint8_t switched = 0;        // 是否已切换标志

    // 已切换，不再重复
    if(switched) return;

    // 条件1: 高度超过阈值
    if(world_data.pz > TAKEOFF_ALT_SWITCH)
    {
        // 条件2: 三轴角速度都在安全范围内(飞机姿态稳定)
        if(fabs(imu_data.gyro_x) < 30.0f &&
           fabs(imu_data.gyro_y) < 30.0f &&
           fabs(imu_data.gyro_z) < 30.0f)
        {
            // 累计稳定时间
            stable_timer += dt;
            
            // 条件3: 稳定时间超过阈值
            if(stable_timer > TAKEOFF_STABLE_TIME)
            {
                flag.mag_fusion_enabled = 1;

                // 锁定当前yaw角为目标 (磁力计融合后yaw估计可能跳变)
                PIDYaw.target = eulerAngle.yaw;

                // 只重置Yaw相关PID，Roll/Pitch保持运行避免失稳
                PIDYaw.error = 0;
                PIDYaw.last_error = 0;
                PIDYaw.integ = 0;
                PIDYaw.out = 0;
                PIDVelZ.error = 0;
                PIDVelZ.last_error = 0;
                PIDVelZ.integ = 0;
                PIDVelZ.out = 0;

                switched = 1;
                stable_timer = 0;
            }
        }
        else
        {
            // 不稳定，清零计时器重新开始
            stable_timer = 0;
        }
    }
    else
    {
        // 高度不够，清零计时器
        stable_timer = 0;
    }
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
        PIDHeight.target = world_data.pz;   // 锁当前高度

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
// 一键降落
// =============================================================================
// 功能: 自动匀速下降至地面，检测到近地后自动上锁
// 流程:
//   1. 高度目标逐步下降 (0.3m/s)
//   2. Roll/Pitch保持水平 (目标0°)
//   3. Yaw锁定当前方向
//   4. 近地保护: 减缓控制防止触地反弹
//   5. 检测到长时间低高度+低速度 → 自动上锁
// =============================================================================
void land(float dt)
{
    // ----- 高度目标: 以0.3m/s速度下降 -----
    PIDHeight.target -= 0.3f * dt; 
    
    // 最小高度限制: 不低于0.05m (防止高度目标变负)
    if(PIDHeight.target < 0.05f)
        PIDHeight.target = 0.05f; 
    
    // ----- 姿态: 保持水平 -----
    PIDRoll.target = 0;
    PIDPitch.target = 0;
    
    // ----- Yaw: 锁定当前方向 -----
    if(!flag.take_off_yaw)
    {
        PIDYaw.target = eulerAngle.yaw;     // 第一次进入时锁当前偏航
        flag.take_off_yaw = 1;
    }
     
    // ----- 近地保护: 高度<15cm时减弱控制 -----
    // 目的: 防止触地瞬间PID过猛导致弹跳
    if(world_data.pz < 0.15f)
    {
        PIDVelH.out *= 0.5f;                // 减弱高度控制力度
    }
    
    // ----- 姿态控制 -----
    stabilization(dt);
    
    // ----- 自动上锁检测 -----
    // 条件: 高度<0.1m 且 垂直速度<0.1m/s，持续50次(约0.5秒@100Hz)
    static uint16_t land_cnt = 0;
    if(world_data.pz < 0.1f && fabs(world_data.vz) < 0.1f)
    {
        land_cnt++;
        
        if(land_cnt > 50)
        {
            flight_state = STATE_LOCK;      // 触地稳定，自动上锁
        }
    }
    else
    {
        land_cnt = 0;                       // 条件不满足，清零计数
    }
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
    static uint8_t step = 0;               // 起飞步骤计数器
    static float takeoff_z = TAKEOFF_TARGET_HEIGHT;

    switch(step)
    {
        // ============ Step 0: 起飞前初始化 ============
        case 0:
            PIDYaw.target = eulerAngle.yaw;         // 锁当前偏航
            PIDHeight.target = world_data.pz;        // 高度目标从当前开始

            flag.mag_fusion_enabled = 0;             // 禁用磁力计融合(地面磁场不可靠)
            step = 1;
        break;

        // ============ Step 1: 离地阶段 ============
        // 目标: 平稳离开地面
        case 1:
            PIDHeight.target += 0.3f * dt;           // 高度目标缓慢增加(0.3m/s)

            PIDRoll.target = 0;                       // Roll保持水平
            PIDPitch.target = 0;                      // Pitch保持水平
            
            stabilization(dt);                        // 姿态控制
            check_mag_switch(dt);                     // 检测是否满足磁力计融合条件

            // 条件: 高度超过0.15m → 进入上升阶段
            if(world_data.pz > 0.15f)
            {
                step = 2;
            }
        break;

        // ============ Step 2: 上升至目标高度 ============
        case 2:
            PIDHeight.target += 0.5f * dt;           // 加速上升(0.5m/s)

            PIDRoll.target = 0;
            PIDPitch.target = 0;
            
            stabilization(dt);
            check_mag_switch(dt);                     // 继续检测磁力计融合条件

            // 条件: 接近目标高度且垂直速度小 → 起飞完成
            if(fabs(world_data.pz - takeoff_z) < 0.1f && fabs(world_data.vz) < 0.2f)
            {
                step = 0;                             // 重置step计数器(供下次起飞使用)
                flag.hover_lock = 0;                  // 清除悬停锁点(让hover重新锁当前位置)
                step = 3;                             // 进入完成阶段
            }
        break;
        
        // ============ Step 3: 起飞完成 → 转悬停 ============
        case 3:
            hover_control(dt);                        // 执行悬停控制(含位置锁)
            flight_state = STATE_FLY;                 // 转入正常飞行状态
        break;
    }
}


// =============================================================================
// 定点位置保持控制
// =============================================================================
// 功能: 定点悬停 + 遥控器微调
// 说明: 当前位置由 hover_lock() 锁定，遥控器可微调偏移
// =============================================================================
void pos_hold_control(float dt)
{
    // ----- 首次进入锁点 -----
    hover_lock();

    // ----- 遥控器微调(当前乘以0禁用，防止遥控器误操作) -----
    PIDPosX.target = rc.roll  * 0.0f;
    PIDPosY.target = rc.pitch * 0.0f;
    PIDHeight.target = rc.thr * 0.0f;
    PIDYaw.target = rc.yaw * 0.0f;

    // ----- 位置 → 姿态 → 电机 -----
    position_control(dt);
    stabilization(dt);
}