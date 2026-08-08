#ifndef __APP_MENU_H_
#define __APP_MENU_H_

#include "zf_common_headfile.h"   // RGB565_* 颜色宏定义在 zf_common_font.h, 总头文件一并引入

void App_Menu_Init(void);
void App_Menu_Task(void);

/* 发车区航点设置配置 (菜单运行时写入, 后续控制逻辑读取; 实际总航点数 = wp_set + launch_enable) */
extern uint8_t launch_enable;     // 1=启用 0=禁用
extern int16   launch_off_x;      // 偏移x (单位0.1m, 范围 -20~+20, 前正后负, 对应小车 vx)
extern int16   launch_off_y;      // 偏移y (单位0.1m, 范围 -20~+20, 左正右负, 对应小车 vy)

#endif
