#ifndef CODE_APP_LORA3A22_H_
#define CODE_APP_LORA3A22_H_

#include "zf_common_headfile.h"
#include "motor.h"
#include "HC06_Driver.h"

/*----------------------- 控制信号来源切换 (v1.4.9) --------------------------*/
// CONTROL_SRC_DRONE: 1=真实无人机/视觉  0=LoRa遥控器模拟
//   1: HC06 解析 #x,y,flag$ 帧 (原RS485/蓝牙路径). 2026-08-15 视觉双摄验证期间,
//      遥控器接线换无线模块接视觉, 视觉发送与无人机一致 → 走这条, 接收逻辑不变.
//   0: App_Lora 摇杆→像素误差 (LoRa遥控器模拟无人机, 台架测试用).
#define CONTROL_SRC_DRONE   1

/*----------------------- 接口函数 --------------------------*/
void App_Lora_Init(void);   // 初始化LoRa遥控器 + 模拟无人机链路
void App_Lora_Task(void);   // 主循环轮询: 摇杆→像素误差→小车运动

#endif /* CODE_APP_LORA3A22_H_ */
