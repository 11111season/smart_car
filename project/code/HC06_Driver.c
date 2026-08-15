#include "HC06_Driver.h"

/*****************************************************************全局定义*****************************************************************/
// 自定义 FIFO 缓冲区
#define RX_FIFO_SIZE 64
//静态缓冲数组
static uint8_t rx_buffer[RX_FIFO_SIZE];
// 命令行缓冲区
static char cmd_line[32];
static uint8_t cmd_index = 0;
static volatile char last_cmd = 0;   // 最近有效指令
static fifo_struct rx_fifo;          // 存原始字节的 FIFO
// 方向前速度
static float speed_fwd = 0.0f;
static float speed_bwd = 0.0f;
static float speed_left = 0.0f;
static float speed_right = 0.0f;
// 方向键按下时间
static uint64_t time_fwd = 0;
static uint64_t time_bwd = 0;
static uint64_t time_left = 0;
static uint64_t time_right = 0;
static void Update_TargetFromButtons(void);  // 声明原型

// 速度步进参数
#define SPEED_STEP 0.01f      // 每次按键0.01 m/s
#define SPEED_MAX 0.15f        // 最大速度 0.2
#define SPEED_U 0.7                   //衰减系数,让竖直方向的速度分量更平滑
#define CMD_TIMEOUT_US 300000 // 超时时间100ms
#define HC06_SCB   (SCB2)
// 模式切换
static uint8_t mode = 0;      // 0:平移模式, 1:旋转模式
static float rot_speed = 0.5f; // 旋转速度 0.5(rad/s)
static float manual_omega = 0.0f; // 手动旋转角速度
volatile uint32_t uart_rx_irq_cnt = 0;

// 接收到的位姿误差（来自无人机）
static float pos_error_x = 0.0f;
static float pos_error_y = 0.0f;
volatile uint8_t drone_beacon_flag = 0;  // 1=beacon found, 2=beacon lost
uint8_t flag2_count = 0;                 // flag=2 连续帧计数, 达到阈值才确认丢信标
volatile uint8_t race_done = 0;          // 完赛标志: 0=比赛中, 1=完赛

// 帧解析状态机
enum FrameState {
    FRAME_WAIT_START,   // 等待 '#'
    FRAME_RECV_DATA,    // 正在接收数据
};
static enum FrameState frame_state = FRAME_WAIT_START;
static char frame_buffer[32];   // 存储帧的原始字符串
static uint8_t frame_idx = 0;

/*****************************************************************全局定义*****************************************************************/

/*****************************************************************************
 * @name       : HC06_UART_RX_Handler
 * @date       : 2026-02-01
 * @function   : 串口接收中断处理函数
 * @parameters : 无
 * @retvalue   : 返回接收字节数/10ms
 * @note       : 会被 isr.c 调用
******************************************************************************/
//
void HC06_UART_RX_Handler(void)
{
    uart_rx_irq_cnt++;   // 每进入一次中断, 计数加1
    uint8_t data;
    // 从串口硬件取一个字节, 无需轮询, 因为已经进入中断
    if (uart_query_byte(UART_1, &data)) {
        // 写入 FIFO
        fifo_write_buffer(&rx_fifo, &data, 1);
    }
}

/*****************************************************************************
 * @name       : HC06_Init
 * @date       : 2026-02-01
 * @function   : 串口初始化函数
 * @parameters : baudrate:波特率, 一般建议使用9600
 * @retvalue   : 无
 * @note       : 在init_all中被调用
******************************************************************************/
// 初始化串口
void HC06_Init(uint32 baudrate)
{
    // 1. 初始化 FIFO
    fifo_init(&rx_fifo, FIFO_DATA_8BIT, rx_buffer, RX_FIFO_SIZE);

    // 2. 初始化串口 1, 引脚固定为 P10_5 TX, P10_6 RX
    uart_init(UART_1, baudrate, UART1_TX_P04_1, UART1_RX_P04_0);

    // 3. 使能接收中断
    uart_rx_interrupt(UART_1, 1);

    //4.使能发送中断
    //uart_tx_interrupt(UART_1,1);

    // 5. 复位状态
    cmd_index = 0;
    last_cmd = 0;
}

void ParseFrameData(const char *data, uint16_t len)
{
    // 预期数据格式: x,y,flag (flag: 1=发现信标, 2=丢失信标, 3=完赛)
    float x, y;
    int flag;
    if (sscanf(data, "%f,%f,%d", &x, &y, &flag) == 3) {
        pos_error_x = x;
        pos_error_y = y;
        drone_beacon_flag = (uint8_t)flag;
        if (flag == 3) race_done = 1;        // flag=3 → 完赛
        if (flag == 2) flag2_count++;
        else           flag2_count = 0;
        printf("DRONE: x=%.1f y=%.1f flag=%d f2cnt=%d race=%d\n", x, y, flag, flag2_count, race_done);
    } else {
        printf("DRONE: parse err: [%s]\n", data);
    }
}

float GetPositionErrorX(void) { return pos_error_x; }
float GetPositionErrorY(void) { return pos_error_y; }
uint8_t GetDroneBeaconFlag(void) { return drone_beacon_flag; }

void ResetPositionError(void) { pos_error_x = 0.0f; pos_error_y = 0.0f; }
void SetPositionError(float x, float y) { pos_error_x = x; pos_error_y = y; }

/*****************************************************************************
 * @name       : HC06_Task
 * @date       : 2026-02-01
 * @function   : 接收数据后的处理, 包括解析无人机指令
 * @parameters : 无
 * @retvalue   : 无
 * @note       : 需要在主循环中调用
******************************************************************************/
//void HC06_Task(void)
//{
//    uint8_t data;
//    while (fifo_read_element(&rx_fifo, &data, FIFO_READ_AND_CLEAN) == FIFO_SUCCESS) {
//        // 可选, 测试用回显
//        //uart_write_byte(UART_1, data);
//
//        if (data == '!') {
//            // 收到终止符, 处理之前累积的命令
//            if (cmd_index > 0) {
//                cmd_line[cmd_index] = '\0';
//                // 目前命令只使用单字符命令, 所以取第一个字符
//                last_cmd = cmd_line[0];
//                // 清空缓冲区, 准备下一条命令
//                cmd_index = 0;
//            }
//        } else {
//            // 普通字符, 写入缓冲区并防止溢出
//            if (cmd_index < sizeof(cmd_line) - 1) {
//                cmd_line[cmd_index++] = data;
//            }
//        }
//    }
//    // 根据按键状态更新目标速度
//    Update_TargetFromButtons();
//    static uint64_t last_print_time = 0;
//    if (angle_pid_yaw.Enable) {
//        if (time_us - last_print_time > 100000) { // 每100ms打印一次
//            //float yaw = My_MPU6050_GetYaw();
//            float yaw = My_Imu660ra_GetYaw();
//            // 注意: angle_pid_yaw.Output 是上次PID计算输出值, 单位°/s
//            //printf("yaw=%.2f, omega_des=%.2f\n", yaw, angle_pid_yaw.Output);
//            last_print_time = time_us;
//        }
//    }
//}
void HC06_Task(void)
{
    uint8_t data;
    while (fifo_read_element(&rx_fifo, &data, FIFO_READ_AND_CLEAN) == FIFO_SUCCESS) {
        // ========== 帧协议解析 #...$ ==========
        if (frame_state == FRAME_WAIT_START && data == '#') {
            frame_state = FRAME_RECV_DATA;
            frame_idx = 0;
            continue;
        }
        if (frame_state == FRAME_RECV_DATA) {
            if (data == '$') {
                // 一帧接收完成
                if (frame_idx > 0) {
                    frame_buffer[frame_idx] = '\0';
                    ParseFrameData(frame_buffer, frame_idx);
                }
                frame_state = FRAME_WAIT_START;
                continue;
            } else {
                // 存储数据, 防止溢出
                if (frame_idx < sizeof(frame_buffer)-1) {
                    frame_buffer[frame_idx++] = data;
                }
                continue;
            }
        }

        // 蓝牙单字符: 跳过!终止符, 其余传Handler
        if (data == '!') continue;
        Bluetooth_Command_Handler((char)data);
        continue;
//        // ========== 原有的单字符指令集以 '!' 为终止符 ==========
//        if (data == '!') {
//            if (cmd_index > 0) {
//                cmd_line[cmd_index] = '\0';
//                last_cmd = cmd_line[0];
//                cmd_index = 0;
//            }
//        } else {
//            if (cmd_index < sizeof(cmd_line)-1) {
//                cmd_line[cmd_index++] = data;
//            }
//        }
    }

    // 无人机控制模式: 跳过手机蓝牙速度计算, target_vx/vy 由 ISR 中 PositionControl_Update() 设置
    // Update_TargetFromButtons();

    // ---- 调试: 每500ms打印一次状态 ----
//    static uint64_t last_dbg_time = 0;
//    if (time_us - last_dbg_time > 500000) {
//        printf("DBG: armed=%d flag=%d px=%.1f py=%.1f vx=%.3f vy=%.3f go_center=%d bcn_nav=%d\n",
//               mission_armed, drone_beacon_flag,
//               GetPositionErrorX(), GetPositionErrorY(),
//               target_vx, target_vy,
//               go_center, bcn_nav_on);
//        last_dbg_time = time_us;
//    }
}

void HC06_SendDroneCmd(uint8_t cmd)
{
    uart_write_byte(UART_1, '#');
    uart_write_byte(UART_1, 'A' + cmd - 1);    // cmd=1→A, cmd=2→B, cmd=3→C
    uart_write_byte(UART_1, '$');
}

// 发送小车目标速度给无人机作为前馈 (#vx,vy$)
void HC06_SendVelocity(float vx, float vy)
{
    char buf[24];
    int len = snprintf(buf, sizeof(buf), "#%.1f,%.1f$", vx, vy);
    for (int i = 0; i < len && i < sizeof(buf); i++) {
        uart_write_byte(UART_1, (uint8_t)buf[i]);
    }
}
//
//// 底层发送一个字节
//static void hc06_tx_byte(uint8 data)
//{
//    volatile stc_SCB_t *scb = HC06_SCB;
//    uint32 timeout = 5000;
//    while (Cy_SCB_GetNumInTxFifo(scb) >= 16)  // 等待 FIFO 有空位
//    {
//        if (--timeout == 0) return;
//    }
//    Cy_SCB_WriteTxFifo(scb, data);
//}
//
//// 发送字符串
//static void hc06_tx_string(const char *str)
//{
//    while (*str)
//    {
//        hc06_tx_byte(*str++);
//    }
//}
//
//// 兼容旧的发送指令函数, 改用 hc06_tx_string
//void HC06_SendDroneCmd(uint8_t cmd)
//{
//    // 格式:#1$  #2$  #3$
//    char buf[4] = {'#', '0' + cmd, '$', '\0'};   // 构造字符串
//    hc06_tx_string(buf);
//}

/*****************************************************************************
 * @name       : HC06_GetCmd
 * @date       : 2026-02-01
 * @function   : 获取串口指令（获取后自动清零）
 * @parameters : 无
 * @retvalue   : 无
 * @note       : 可在主循环中使用
******************************************************************************/
char HC06_GetCmd(void)
{
    char cmd = last_cmd;
    last_cmd = 0;
    return cmd;
}

/*****************************************************************************
 * @name       : Process_Bluetooth_Command
 * @date       : 2026-02-01
 * @function   : 处理蓝牙指令函数, 根据接收到的指令控制小车的运动
 * @parameters : cmd:接收到的蓝牙指令字符
 * @retvalue   : 无
 * @note       : 可在主循环中使用
******************************************************************************/
void Bluetooth_Command_Handler(char cmd)
{
    uint64_t now = time_us;

    if (cmd == 'F') { // 前进
        // 无人机控制: 禁用F键
        // speed_fwd += SPEED_STEP;
        // if (speed_fwd > SPEED_U * SPEED_MAX) speed_fwd = SPEED_U * SPEED_MAX;
        // time_fwd = now;
    }
    else if (cmd == 'B') { // 后退
        // 无人机控制: 禁用B键
        // speed_bwd += SPEED_STEP;
        // if (speed_bwd > SPEED_MAX) speed_bwd = SPEED_MAX;
        // time_bwd = now;
    }
    else if (cmd == 'L') { // 左移（右转指令）
        // 无人机控制: 禁用L键
        // speed_left += SPEED_STEP;
        // if (speed_left > SPEED_U * SPEED_MAX) speed_left = SPEED_U * SPEED_MAX;
        // time_left = now;
    }
    else if (cmd == 'R') { // 右移（左转指令）
        // 无人机控制: 禁用R键
        // speed_right += SPEED_STEP;
        // if (speed_right > SPEED_MAX) speed_right = SPEED_MAX;
        // time_right = now;
    }
//    else if (cmd == 'M') { speed_fwd = speed_bwd = speed_left = speed_right = 0.0f; go_center = 1; printf("GO CENTER!\n"); }  // 废弃: 去中心改为由标志位2触发
//    else if (cmd == 'M') {   // 原模式切换已废弃, 改为上面go_center
//        mode = !mode;
//        if (mode == 0) {
//            // 切换平移模式时, 将当前yaw设为新的零点
////            angle_target = My_MPU6050_GetYaw();
//            angle_target = My_Imu660ra_GetYaw();
//        }
//        // 切换模式时, 清零目标速度（可选）
//        speed_fwd = speed_bwd = speed_left = speed_right = 0.0f;
    }
//    else if (cmd == 'X') {
//        balance_mode = !balance_mode;
//        target_vx = 0; target_vy = 0;
//        manual_omega = 0;
//        if (balance_mode) {
//            pitch_offset = My_Imu660ra_GetPitch();
//            roll_offset = My_Imu660ra_GetRoll();
////            pitch_offset = My_MPU6050_GetPitch();
////            roll_offset = My_MPU6050_GetRoll();
//            // 用 printf 打印偏移值确认是否正常
//            //printf("pitch_offset=%.2f, roll_offset=%.2f\n", pitch_offset, roll_offset);
//            //uart_write_string(UART_2, "Balance mode ON\r\n");
//        } else {
//            //uart_write_string(UART_2, "Balance mode OFF\r\n");
//        }
//    }

/*****************************************************************************
 * @name       : Get_ManualOmega
 * @date       : 2026-02-01
 * @function   : 获取手动控制的角速度
 * @parameters : 无
 * @retvalue   : 无
 * @note       : 在中断函数中的目标角度控制加上该角度
******************************************************************************/
float Get_ManualOmega(void)
{
    return manual_omega;
}
/*****************************************************************内部调用函数*****************************************************************/

/*****************************************************************************
 * @name       : Check_Speed_Timeout
 * @date       : 2026-02-01
 * @function   : 超时检测函数, 检测到超时的按键速度保持为0
 * @parameters : 无
 * @retvalue   : 无
 * @note       : 在本文件内部使用
******************************************************************************/
static void Check_Speed_Timeout(void)
{
    uint64_t now = time_us;
    if (now - time_fwd > CMD_TIMEOUT_US) speed_fwd = 0.0f;
    if (now - time_bwd > CMD_TIMEOUT_US) speed_bwd = 0.0f;
    if (now - time_left > CMD_TIMEOUT_US) speed_left = 0.0f;
    if (now - time_right > CMD_TIMEOUT_US) speed_right = 0.0f;
}

/*****************************************************************************
 * @name       : Update_TargetFromButtons
 * @date       : 2026-02-01
 * @function   : 根据指令映射后的按键指令转换为目标速度/角度
 * @parameters : 无
 * @retvalue   : 无
 * @note       : 在本文件内部使用
******************************************************************************/
static void Update_TargetFromButtons(void)
{
    Check_Speed_Timeout();

    if (mode == 0) { // 平移模式
        // 根据前后合成速度 vx
        if (speed_fwd > speed_bwd) {
            target_vx = speed_fwd - speed_bwd;
        } else {
            target_vx = - (speed_bwd - speed_fwd);
        }
        // 根据左右合成速度 vy
        if (speed_right > speed_left) {
            target_vy = speed_right - speed_left;
        } else {
            target_vy = - (speed_left - speed_right);
        }
        manual_omega = 0.0f; // 平移模式无手动旋转
    }
    else { // 旋转模式
        target_vx = 0.0f;
        target_vy = 0.0f;

        // 动态更新角度环目标为当前yaw, 使角度环不干预
        angle_target = My_Imu660ra_GetYaw();
        //angle_target = My_MPU6050_GetYaw();   // 备用
        // 根据左右速度差计算旋转角速度
        float diff = speed_right - speed_left;
        manual_omega = diff * rot_speed / SPEED_MAX; // 映射到旋转速度
        // 限幅
        if (manual_omega > rot_speed) manual_omega = rot_speed;
        if (manual_omega < -rot_speed) manual_omega = -rot_speed;
    }
}
//
//void Process_Bluetooth_Command(char cmd)
//{
//    switch (cmd) {
//        case 'e': // 使能角度环
//            if (mpu6050_ready) {
//                Motor_Enable_PID(1);
//                PID_Reset(&angle_pid_yaw);
//                PID_Enable(&angle_pid_yaw, 1);
//                My_MPU6050_ResetYaw();
//                target_vx = 0.0f; target_vy = 0.0f;
//                printf("Angle control enabled\n");
//            } else {
//                // 提示无人机未就绪
//                printf("MPU6050 not ready\n");
//            }
//            break;
//        case 'v':
//            Motor_Enable_PID(1);
//            // 不使能角度环
//            PID_Enable(&angle_pid_yaw, 0);
//            target_vx = 0.2f; target_vy = 0.0f;
//            printf("speed mode: vx=0.2\n");
//            break;
//        case 'd': // 关闭角度环
//            Motor_Enable_PID(0);
//            PID_Enable(&angle_pid_yaw, 0);
//            target_vx = 0.0f; target_vy = 0.0f;
//            // 强制停车, 参考原来的停止方式
//            Motor_SetSpeed(&motor_L1, 0, MotorL1_Turn, MotorL1_Pwm);
//            Motor_SetSpeed(&motor_L2, 0, MotorL2_Turn, MotorL2_Pwm);
//            Motor_SetSpeed(&motor_R1, 0, MotorR1_Turn, MotorR1_Pwm);
//            Motor_SetSpeed(&motor_R2, 0, MotorR2_Turn, MotorR2_Pwm);
//            printf("PID disabled, motors stopped\n");
//            break;
//        case 'f': // 前进
//            target_vx = 0.2f; target_vy = 0.0f;
//            printf("target: vx=0.2, vy=0\n");
//            break;
//        case 'b': // 后退
//            target_vx = -0.2f; target_vy = 0.0f; break;
//        case 'l': // 左移
//            target_vx = 0.0f; target_vy = -0.2f; break;
//        case 'r': // 右移
//            target_vx = 0.0f; target_vy = 0.2f; break;
//        case 's': // 停止
//            target_vx = 0.0f; target_vy = 0.0f;
//            Motor_SetSpeed(&motor_L1, 0, MotorL1_Turn, MotorL1_Pwm);
//            Motor_SetSpeed(&motor_L2, 0, MotorL2_Turn, MotorL2_Pwm);
//            Motor_SetSpeed(&motor_R1, 0, MotorR1_Turn, MotorR1_Pwm);
//            Motor_SetSpeed(&motor_R2, 0, MotorR2_Turn, MotorR2_Pwm);
//            printf("target: (0,0)\n");
//            break;
//        default:
//            break;
//    }
//}
