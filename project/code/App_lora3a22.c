#include "App_lora3a22.h"

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

// 左摇杆 → 像素误差缩放: 摇杆满量程约±2000
// 位置环 Kp=0.01 限幅0.30m/s, 满推像素误差≈20 → 位置环输出≈0.20m/s
// K = 20 / 2000 = 0.010  (2026-08-07: 0.005→0.016→0.010, 满推0.20m/s, 不吃满留余量)
// 位置环 err_x = -GetPositionErrorX(), 符号标定后应保证"前推=前进"
#define APP_STICK_K_FWD          0.010f   // 左杆Y → pos_error_x 缩放 (满推≈20→0.20m/s)
#define APP_STICK_K_LAT          0.010f   // 左杆X → pos_error_y 缩放 (满推≈20→0.20m/s)

// 符号标定: 若方向反, 改为 -1.0f (位置环内部有 -GetPositionError 的视觉翻转负号)
#define APP_SIGN_FWD             1.0f     // 前后方向: +1=前推前进, -1=反
#define APP_SIGN_LAT            -1.0f     // 左右方向: +1=右推右移, -1=反(实测左打右移, 取反)

// 右摇杆 → 自转角速度: 目标角度增量 (度/秒/满量程)
// 满量程±2047 → 全程打满时约 ±90°/s, 可通过该系数调整
#define APP_YAW_K                0.015f   // = 30°/s ÷ 2047 (满推自转≈30°/s, 可调)
#define APP_YAW_SIGN            -1.0f     // 自转方向: +1=右推顺时针, -1=反(实测左打反, 取反)

/*==================================================== 内部状态 ====================================================*/
static uint8_t app_lora_inited = 0;
static uint64_t app_last_tick_us = 0;    // 上次Task调用的时间戳(微秒)
static uint64_t app_last_frame_us = 0;   // 最近一次收到遥控器新帧的时间戳(微秒)
static uint64_t app_last_active_us = 0;  // 最近一次遥控器有操作(摇杆/按键)的时间戳(微秒)

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
    printf("App_Lora: init ok, simulating drone (flag=1)\n");
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

    // 遥控器链路状态判定 — 两级判"丢信标":
    //   遥控器开机后每50ms必发一帧(即使不动), 不能只靠"收不到帧"判丢信标.
    //   A. 断连: 完全收不到帧 (遥控关机)  >200ms → flag=2
    //   B. 空闲: 收到帧但摇杆全回中+无按键 >200ms → flag=2  (模拟无人机认为此时没有信标)
    //   两者都累加 flag2_count, 连续达到阈值才确认丢信标 (FLAG2_DEBOUNCE=10)
    if (lora3a22_finsh_flag) {
        lora3a22_finsh_flag = 0;
        app_last_frame_us = time_us;

        // 有操作判定: 任一摇杆离开死区 或 任一按键按下
        uint8_t remote_active = 0;
        for (int i = 0; i < 4; i++) {
            if (lora3a22_uart_transfer.joystick[i] >  APP_JOYSTICK_DEADZONE ||
                lora3a22_uart_transfer.joystick[i] < -APP_JOYSTICK_DEADZONE) remote_active = 1;
        }
        for (int i = 0; i < 4; i++) {
            if (lora3a22_uart_transfer.key[i] != 0) remote_active = 1;
        }

        if (remote_active) {
            // 有操作: 模拟无人机重新发现信标
            app_last_active_us = time_us;
            if (drone_beacon_flag != 1) {
                drone_beacon_flag = 1;
                flag2_count = 0;
                printf("App_Lora: link OK, beacon found (flag=1)\n");
            }
        } else if (time_us - app_last_active_us > APP_IDLE_TIMEOUT_US) {
            // 空闲超时: 清误差 + 模拟丢信标 (flag=2) → 触发惯导巡逻
            SetPositionError(0.0f, 0.0f);
            if (drone_beacon_flag != 2) {
                drone_beacon_flag = 2;
                flag2_count = 0;
                printf("App_Lora: remote idle, beacon lost (flag=2)\n");
            }
            if (flag2_count < 255) flag2_count++;   // 连续空闲帧计数 (uint8上限保护)
        }
    } else if (time_us - app_last_frame_us > APP_FRAME_TIMEOUT_US) {
        // 断连(遥控关机): 清误差 + 模拟丢信标. 每次超时累加 flag2_count, 连续达到阈值才确认丢信标
        // (与 HC06 真实帧的 flag2_count++ 逻辑一致, FLAG2_DEBOUNCE=10)
        SetPositionError(0.0f, 0.0f);
        if (drone_beacon_flag != 2) {
            drone_beacon_flag = 2;
            flag2_count = 0;
            printf("App_Lora: link lost, beacon lost (flag=2)\n");
        }
        if (flag2_count < 255) flag2_count++;   // 连续超时帧计数 (uint8上限保护)
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

    // 读取摇杆原始值
    int16 joy0 = lora3a22_uart_transfer.joystick[0];   // 左杆X
    int16 joy1 = lora3a22_uart_transfer.joystick[1];   // 左杆Y
    int16 joy2 = lora3a22_uart_transfer.joystick[2];   // 右杆X

    float raw_y = app_stick_raw(joy1);   // 前后
    float raw_x = app_stick_raw(joy0);   // 左右
    float raw_yaw = app_stick_raw(joy2); // 自转

    // ---- 左摇杆 → 像素误差 (模拟无人机发送) ----
    // 位置环 err_x = -GetPositionErrorX(), 符号标定见 APP_SIGN_FWD/LAT
    // 前推(joy1>0) 应使 pos_error_x 最终产生 vx>0 前进
    SetPositionError(raw_y * APP_STICK_K_FWD * APP_SIGN_FWD,
                     raw_x * APP_STICK_K_LAT * APP_SIGN_LAT);

    // ---- 右摇杆 → 增量式自转 ----
    // 摇杆推着就累加 angle_target, 回中自然锁住当前朝向
    angle_target += raw_yaw * APP_YAW_K * APP_YAW_SIGN * dt_sec;

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
