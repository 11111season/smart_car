#ifndef CODE_APP_LORA3A22_H_
#define CODE_APP_LORA3A22_H_

#include "zf_common_headfile.h"
#include "motor.h"
#include "HC06_Driver.h"

/*----------------------- 接口函数 --------------------------*/
void App_Lora_Init(void);   // 初始化LoRa遥控器 + 模拟无人机链路
void App_Lora_Task(void);   // 主循环轮询: 摇杆→像素误差→小车运动

#endif /* CODE_APP_LORA3A22_H_ */
