#ifndef CODE_KEY_TASK_H_
#define CODE_KEY_TASK_H_

#include "zf_common_headfile.h"

// 磁力计椭圆校准模式: 1=KEY4用于校准(采集原始数据), 0=KEY4用于导航(记录航点/发车)
// 校准时置1, 校准完成置0并重新编译
// 置1时: 主循环不跑App_Lora_Task, KEY4走磁力计采集, 避免调试打印污染数据
// 2026-08-09: 架高标定+动态验证全部完成 → 置0 恢复正常导航 (下地实测惯导)
#define MAG_CALIB_MODE  0

// 磁力计校准子模式: 0=椭圆拟合(静态, 转圈采集), 1=电机磁场干扰测试(PWM扫频)
// 2=航向验证(连续打印磁力计/融合/陀螺三路yaw)
// 仅在 MAG_CALIB_MODE=1 时有意义
// 2026-08-09: 标定完成(硬铁-145/-521+软铁1.091), 动态验证完成(L1 EM偏移-1.9°)
// 置0: 无校准任务 (MAG_CALIB_MODE=0 时本宏无效果)
#define MAG_CALIB_MOTOR_TEST  0

void KeyTask_Init(void);
void KeyTask_Handler(void);

#endif /* CODE_KEY_TASK_H_ */
