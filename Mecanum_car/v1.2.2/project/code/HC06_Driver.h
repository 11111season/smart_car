#ifndef CODE_HC06_DRIVER_H_
#define CODE_HC06_DRIVER_H_

#include "zf_common_headfile.h"
#include "motor.h" // 需要用到 target_vx, target_vy
#include "pid.h"
#include "My_imu660ra.h"
/*----------------------- 函数接口声明 --------------------------*/
// 初始化蓝牙模块（调用逐飞库的debug_init即可，它会根据上面的宏配置）
void HC06_Init(uint32 baudrate);
float Get_ManualOmega(void);
// 蓝牙数据处理函数，需要在主循环中周期调用
void HC06_Task(void);
void HC06_UART_RX_Handler(void);
// 获取最新有效的指令（如果有的话）
char HC06_GetCmd(void);
void Bluetooth_Command_Handler(char cmd);


// 新增：解析帧数据（# ... $）并更新位置误差
void ParseFrameData(const char *data, uint16_t len);
// 新增：获取当前位置误差（可选，用于调试）
float GetPositionErrorX(void);
float GetPositionErrorY(void);
// 发送无人机命令（#1$ 起浆，#2$ 启动PID，#3$ 急停）
void HC06_SendDroneCmd(uint8_t cmd);

/*----------------------- 外部参数引用 -----------------------*/
extern volatile uint64_t time_us;
extern volatile uint8_t mpu6050_read;
extern volatile uint32_t uart_rx_irq_cnt;

#endif /* CODE_HC06_DRIVER_H_ */