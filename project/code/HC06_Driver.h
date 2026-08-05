#ifndef CODE_HC06_DRIVER_H_
#define CODE_HC06_DRIVER_H_

#include "zf_common_headfile.h"
#include "motor.h" // 需要用到 target_vx, target_vy
#include "pid.h"
#include "My_imu660ra.h"
#include "inertial_nav.h"
/*----------------------- 函数接口声明 --------------------------*/
// 初始化串口模块（通常由debug_init完成, 此函数可直接调用）
void HC06_Init(uint32 baudrate);
float Get_ManualOmega(void);
// 接收数据并处理, 需要在主循环中调用
void HC06_Task(void);
void HC06_UART_RX_Handler(void);
// 获取蓝牙有效指令（可主动轮询）
char HC06_GetCmd(void);
void Bluetooth_Command_Handler(char cmd);


// 解析无人机数据帧（# ... $携带位置误差）
void ParseFrameData(const char *data, uint16_t len);
// 获取当前位姿误差（可选, 用于调试）
float GetPositionErrorX(void);
float GetPositionErrorY(void);
uint8_t GetDroneBeaconFlag(void);
void ResetPositionError(void);
// 发送无人机指令（#A$ 起桨, #B$ 起飞, #C$ 急停, #D$ 已发车）
void HC06_SendDroneCmd(uint8_t cmd);
void HC06_SendVelocity(float vx, float vy);   // 发送目标速度给无人机前馈

/*----------------------- 外部变量声明 -----------------------*/
extern volatile uint64_t time_us;
extern volatile uint8_t mpu6050_read;
extern volatile uint32_t uart_rx_irq_cnt;
extern volatile uint8_t drone_beacon_flag;
extern uint8_t flag2_count;            // flag=2 连续帧计数, 防抖用
extern volatile uint8_t race_done;     // 完赛标志: 0=比赛中, 1=完赛
#define FLAG2_DEBOUNCE  10              // 连续收到N帧flag=2才确认丢信标

#endif /* CODE_HC06_DRIVER_H_ */