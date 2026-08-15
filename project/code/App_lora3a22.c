#include "App_lora3a22.h"
#include "App_menu.h"      // 位置环速度限幅 pos_limit_x/y (动态映射左杆缩放用, v1.4.7)

/*********************************************************************************************************************
 * 文件名称          App_lora3a22
 * 功能描述          LoRa遥控器模拟无人机通信
 *                   用遥控器摇杆模拟无人机发送的像素误差, 驱动小车位置环/角度环
 *
 * 信道映射:
 *   joystick[0] 左杆X → 左右平移像素误差 (pos_error_y)
 *   joystick[1] 左杆Y → 前后平移像素误差 (pos_error_x)
 *   joystick[2] 右杆X → 自转 (angle_target 增量式)
 *
 * 模拟方式:
 *   位置环需要 drone_beacon_flag==1 才执行, 故模拟时强制置1(模拟无人机一直发现信标)
 *   自转用增量式: 右杆推着就累加目标角度, 回中自然锁住当前朝向
 ********************************************************************************************************************/

/*==================================================== 配置参数 ====================================================*/
// 摇杆死区: 小于该值视为回中(对应遥控器内部 threshold=50, 此处适当放宽)
#define APP_JOYSTICK_DEADZONE    80

// 左摇杆 → 像素误差缩放 (v1.4.7 起动态映射到位置环限幅):
//   满推(摇杆±2000)时像素误差使位置环输出 = 当前位置环限幅 (pos_limit*0.05 m/s)
//   位置环输出 = pos_pid.Kp(0.01) × err → err_full = (pos_limit*0.05)/0.01 = 5*pos_limit → K = 5*pos_limit/2000
//   默认6(0.30m/s) → K=0.015 → 满推0.30m/s (原固定0.010仅0.20); 限幅调大遥控器满推同步变快,
//   用于模拟"限幅放大后 无人机控制小车跑更快"的效果 (真实无人机像素误差不受此缩放影响)
// 位置环 err_x = -GetPositionErrorX(), 符号标定后应保证"前推=前进"
#define APP_STICK_MAX        2000.0f
#define APP_POS_KP           0.01f
#define APP_STICK_K_FULL(pos_lim)  (((float)(pos_lim) * 0.05f) / APP_POS_KP / APP_STICK_MAX)
#define APP_STICK_K_REF      0.010f   // 死带标定参考缩放 (旧固定K), 有效命令死带按 K/REF 等比放大保持机械残留容忍不变

// 符号标定: 若方向反, 改为 -1.0f (位置环内部有 -GetPositionError 的视觉翻转负号)
#define APP_SIGN_FWD             1.0f     // 前后方向: +1=前推前进, -1=反
#define APP_SIGN_LAT            -1.0f     // 左右方向: +1=右推右移, -1=反(实测左打右移, 取反)

// 右摇杆 → 自转角速度: 目标角度增量 (度/秒/满量程)
// 满量程±2047 → 全程打满时约 ±90°/s, 可通过该系数调整
#define APP_YAW_K                0.015f   // = 30°/s ÷ 2047 (满推自转≈30°/s, 可调)
#define APP_YAW_SIGN            -1.0f     // 自转方向: +1=右推顺时针, -1=反(实测左打反, 取反)

// 有效命令死带: 摇杆弹簧回中不完美, 残留原始值(±80~几百) × K 产生的命令其实 <0.02m/s 几乎不动车.
// 若按原始值判"有操作"会一直占住 flag=1 → flag2_count 不累加 → 永不转惯导巡逻 (发车不走的根因).
// 故改用"有效命令大小"判有操作, 残留视为回中. 换算容差: 位置≈原始残留±280, 自转≈±200.
// v1.4.7: K 随位置环限幅动态变化, 实际死带 = APP_CMD_DEADBAND × (K/APP_STICK_K_REF) 在任务内逐帧计算,
//         保持"等价摇杆机械残留容忍"不变 (K 放大时残留映射像素误差同步放大, 死带须等比放大防误判).
#define APP_CMD_DEADBAND      2.0f    // 位置误差命令死带基准(像素, K=0.010时): <2px → 位置环<0.02m/s, 视为回中
#define APP_YAW_DEADBAND_DPS  3.0f    // 自转角速度死带(°/s): <3°/s 不累加目标角度, 视为回中

/*==================================================== 内部状态 ====================================================*/
static uint8_t app_lora_inited = 0;
static uint64_t app_last_tick_us = 0;    // 上次Task调用的时间戳(微秒)
static uint64_t app_last_frame_us = 0;   // 最近一次收到遥控器新帧的时间戳(微秒)
static uint64_t app_last_active_us = 0;  // 最近一次遥控器有操作(摇杆/按键)的时间戳(微秒)
static uint64_t app_disc_count_us = 0;   // 断连路径 flag2_count 累加节拍 (每50ms+1, 与空闲路径帧频一致, 2026-08-13)

// 遥控器数据超时: 超过该时间没收到新帧, 视为断连/休眠, 强制清零误差
#define APP_FRAME_TIMEOUT_US  200000    // 200ms (遥控器50ms一帧, 4帧未收到即判超时)
// 遥控器空闲判定: 收到帧但无操作(摇杆回中+无按键)持续该时长, 视为无人机认为无信标
// 2026-08-07: 200ms→2s, 与比赛"无人机一段时间没发现信标才转入惯导"的模拟一致
#define APP_IDLE_TIMEOUT_US   2000000   // 2s

/*==================================================== 内部函数 ====================================================*/

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       app_stick_raw
// 函数描述       读取摇杆原始值并做死区处理, 回中/死区内返回0
//-------------------------------------------------------------------------------------------------------------------
static float app_stick_raw(int16 raw)
{
    if (raw >  APP_JOYSTICK_DEADZONE) return (float)(raw - APP_JOYSTICK_DEADZONE);
    if (raw < -APP_JOYSTICK_DEADZONE) return (float)(raw + APP_JOYSTICK_DEADZONE);
    return 0.0f;
}

/*==================================================== 接口函数 ====================================================*/

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       App_Lora_Init
// 函数描述       初始化LoRa遥控器, 并模拟无人机"一直发现信标"
//               位置环 PositionControl_Update 要求 drone_beacon_flag==1 才执行
//-------------------------------------------------------------------------------------------------------------------
void App_Lora_Init(void)
{
    if (app_lora_inited) return;

    lora3a22_init();          // 初始化UART_1 + 挂载 lora3a22_uart_callback

    // 模拟无人机链路: 强制认为发现信标, 使位置环可以执行
    // (真实无人机是通过 #x,y,1$ 帧解析设置此标志)
    drone_beacon_flag = 1;
    flag2_count = 0;

    app_lora_inited = 1;
    //printf("App_Lora: init ok, simulating drone (flag=1)\n");   // 2026-08-08 暂注释 (W25Q64测试期间串口干净)
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       App_Lora_Task
// 函数描述       主循环轮询: 读取摇杆 → 写入位置误差/目标角度 → 驱动小车
//               左摇杆: 模拟无人机像素误差 (前后/左右平移)
//               右摇杆: 增量式自转, 回中锁角度
//-------------------------------------------------------------------------------------------------------------------
void App_Lora_Task(void)
{
    static uint8_t armed_prev = 0;

    if (!app_lora_inited) return;

    // ---- 调试: 每500ms打印一次数据链路状态 ----
    // k=key[0..3]摇杆/副机按键, sw=switch_key[0..3]拨杆, jy=joystick[0..3]摇杆轴
    // 按遥控器 S4 观察哪个 k/sw 字段跳1, 确定完赛触发的映射
    // 2026-08-08: 暂时注释 (W25Q64 测试期间干扰串口判断)
#if 0
    static uint64_t last_dbg_us = 0;
    if (time_us - last_dbg_us > 500000) {
        last_dbg_us = time_us;
        printf("LORA: k=%d,%d,%d,%d sw=%d,%d,%d,%d jy=%d,%d,%d,%d state=%d\n",
               lora3a22_uart_transfer.key[0],
               lora3a22_uart_transfer.key[1],
               lora3a22_uart_transfer.key[2],
               lora3a22_uart_transfer.key[3],
               lora3a22_uart_transfer.switch_key[0],
               lora3a22_uart_transfer.switch_key[1],
               lora3a22_uart_transfer.switch_key[2],
               lora3a22_uart_transfer.switch_key[3],
               lora3a22_uart_transfer.joystick[0],
               lora3a22_uart_transfer.joystick[1],
               lora3a22_uart_transfer.joystick[2],
               lora3a22_uart_transfer.joystick[3],
               lora3a22_state_flag);
    }
#endif

    // ---- 完赛触发: 最左边拨杆 switch_key[0] 1→0 = 完赛 (模拟 flag=3) ----
    // 拨杆默认位置1, 完赛时拨到位置0. 边沿检测(1→0)只触发一次, race_done 自锁由 ISR 处理.
    // 两道保险:
    //   1) 首帧(sw0_prev=0xff)只记录拨杆状态不判定 → 上电时拨杆若停在0不会误触发锁定
    //   2) 仅 mission_armed 后生效 → 记录航点阶段(未武装)误碰拨杆不锁车
    static uint8_t sw0_prev = 0xff;                    // 0xff=首帧尚未初始化
    uint8_t sw0 = lora3a22_uart_transfer.switch_key[0];
    if (sw0_prev != 0xff && mission_armed && sw0_prev == 1 && sw0 == 0) {
        race_done = 1;
        printf("App_Lora: RACE DONE via switch1 (flag=3)\n");
    }
    sw0_prev = sw0;

    // 读取摇杆原始值 → 死区(80) → 有效命令 (供下面 flag 判定与输出共用)
    int16 joy0 = lora3a22_uart_transfer.joystick[0];   // 左杆X
    int16 joy1 = lora3a22_uart_transfer.joystick[1];   // 左杆Y
    int16 joy2 = lora3a22_uart_transfer.joystick[2];   // 右杆X

    float raw_y   = app_stick_raw(joy1);   // 前后
    float raw_x   = app_stick_raw(joy0);   // 左右
    float raw_yaw = app_stick_raw(joy2);   // 自转

    // 有效命令: 位置环 err = -GetPositionErrorX(), 符号标定见 APP_SIGN_FWD/LAT
    // 左杆缩放动态取自位置环限幅 (v1.4.7): 前后杆随 pos_limit_x, 左右杆随 pos_limit_y
    float k_fwd = APP_STICK_K_FULL(pos_limit_x);
    float k_lat = APP_STICK_K_FULL(pos_limit_y);
    float pos_err_x = raw_y   * k_fwd * APP_SIGN_FWD;
    float pos_err_y = raw_x   * k_lat * APP_SIGN_LAT;
    float yaw_rate  = raw_yaw * APP_YAW_K       * APP_YAW_SIGN;   // °/s (与dt无关)
    // 有效命令死带随 K 等比放大 (v1.4.7): K 变大后同尺寸摇杆机械残留映射更多像素误差,
    // 若死带固定 2.0px 会把回中残留误判为"有操作" → 卡死不转巡逻. 按 K/REF 放大保持等价容忍.
    float db_fwd = APP_CMD_DEADBAND * (k_fwd / APP_STICK_K_REF);
    float db_lat = APP_CMD_DEADBAND * (k_lat / APP_STICK_K_REF);

    // 遥控器链路状态判定 — 两级判"丢信标":
    //   遥控器开机后每50ms必发一帧(即使不动), 不能只靠"收不到帧"判丢信标.
    //   A. 断连: 完全收不到帧 (遥控关机)  >200ms → flag=2
    //   B. 空闲: 收到帧但摇杆全回中+无按键 >200ms → flag=2  (模拟无人机认为此时没有信标)
    //   两者都累加 flag2_count, 连续达到阈值才确认丢信标 (FLAG2_DEBOUNCE=10)
    if (lora3a22_finsh_flag) {
        lora3a22_finsh_flag = 0;
        app_last_frame_us = time_us;

        // 有操作判定: 用"有效命令大小"而非摇杆原始值 或 任一按键按下
        // (摇杆弹簧回中不完美的残留产生的命令其实<0.02m/s 几乎不动车, 应视为回中)
        uint8_t remote_active = 0;
        if (fabsf(pos_err_x) > db_fwd || fabsf(pos_err_y) > db_lat) remote_active = 1;
        if (fabsf(yaw_rate)  > APP_YAW_DEADBAND_DPS) remote_active = 1;
        for (int i = 0; i < 4; i++) {
            if (lora3a22_uart_transfer.key[i] != 0) remote_active = 1;
        }

        if (remote_active) {
            // 有操作: 模拟无人机重新发现信标
            app_last_active_us = time_us;
            if (drone_beacon_flag != 1) {
                drone_beacon_flag = 1;
                flag2_count = 0;
                //printf("App_Lora: link OK, beacon found (flag=1)\n");   // 2026-08-08 暂注释 (W25Q64测试)
            }
        } else if (time_us - app_last_active_us > APP_IDLE_TIMEOUT_US) {
            // 空闲超时: 清误差 + 模拟丢信标 (flag=2) → 触发惯导巡逻
            SetPositionError(0.0f, 0.0f);
            if (drone_beacon_flag != 2) {
                drone_beacon_flag = 2;
                flag2_count = 0;
                //printf("App_Lora: remote idle, beacon lost (flag=2)\n");   // 2026-08-08 暂注释
            }
            if (flag2_count < 255) flag2_count++;   // 连续空闲帧计数 (uint8上限保护)
        }
    } else if (time_us - app_last_frame_us > APP_FRAME_TIMEOUT_US) {
        // 断连(遥控关机/休眠): 清误差 + 模拟丢信标. flag2_count 每50ms+1 (与空闲路径帧频一致),
        // 连续10次(≈500ms)才确认丢信标 → 巡逻重触发 ≈0.2s+0.5s=0.7s (2026-08-13: 原每主循环+1,
        // 遥控器若不满"每50ms必发一帧", 松开摇杆即闪断判丢信标, <0.5s就进巡逻)
        // 2026-08-14: 曾试"统一2s门限"(与空闲路径一致) — 用户实测打杆不顺畅, 已回退到本版 (0.7s).
        SetPositionError(0.0f, 0.0f);
        if (drone_beacon_flag != 2) {
            drone_beacon_flag = 2;
            flag2_count = 0;
            app_disc_count_us = time_us;   // 重置累加节拍 (首个计数从50ms后开始)
            //printf("App_Lora: link lost, beacon lost (flag=2)\n");   // 2026-08-08 暂注释 (W25Q64测试)
        }
        if (time_us - app_disc_count_us >= 50000) {   // 按遥控器帧频50ms累加, 不再每主循环+1
            app_disc_count_us = time_us;
            if (flag2_count < 255) flag2_count++;
        }
        return;   // 断连期间不更新角度/误差
    }

    // 计算距上次调用的真实时间差 (主循环周期不固定, 用时间戳算)
    uint64_t now_us = time_us;
    float dt_sec;
    if (app_last_tick_us == 0) {
        dt_sec = 0.01f;                  // 首次: 假设10ms
    } else {
        dt_sec = (float)(now_us - app_last_tick_us) * 1e-6f;
        if (dt_sec <= 0.0f || dt_sec > 0.1f) dt_sec = 0.01f;   // 防异常
    }
    app_last_tick_us = now_us;

    // ---- 左摇杆 → 像素误差 (模拟无人机发送) ----
    // 前推(joy1>0) 应使 pos_error_x 最终产生 vx>0 前进; 死带内残留输出0
    SetPositionError((fabsf(pos_err_x) > db_fwd) ? pos_err_x : 0.0f,
                     (fabsf(pos_err_y) > db_lat) ? pos_err_y : 0.0f);

    // ---- 右摇杆 → 增量式自转 ----
    // 摇杆推着就累加 angle_target, 回中锁朝向; 死带内残留不累加(不会缓慢漂移)
    if (fabsf(yaw_rate) > APP_YAW_DEADBAND_DPS)
        angle_target += yaw_rate * dt_sec;

    // 角度归一化到 ±180, 防止长期自转浮点溢出
    if (angle_target >  180.0f) angle_target -= 360.0f;
    if (angle_target < -180.0f) angle_target += 360.0f;

    // ---- 发车状态变化检测: 打印提示 ----
    if (mission_armed != armed_prev) {
        armed_prev = mission_armed;
        if (mission_armed) printf("App_Lora: mission armed, control active\n");
        else               printf("App_Lora: mission disarmed\n");
    }
}
