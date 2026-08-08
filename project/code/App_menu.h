#ifndef __APP_MENU_H_
#define __APP_MENU_H_

#include "zf_common_headfile.h"   // RGB565_* 颜色宏定义在 zf_common_font.h, 总头文件一并引入

void App_Menu_Init(void);
void App_Menu_Task(void);

/* 惯导设置全局 (菜单写入, 掉电存储经 W25Q64 覆盖, 惯性导航读取; 实际总航点数 = wp_set + launch_enable) */
extern uint8_t wp_set;            // 已确认航点总数 (不含发车区) = 惯导 BCN_MAX
extern uint8_t launch_enable;     // 发车区航点启用: 1=启用 0=禁用 (决定 WP_MAX 是否 +1)
extern int16   launch_off_x;      // 发车区偏移x (单位0.1m, 范围 -20~+20, 前正后负, 对应小车 vx)
extern int16   launch_off_y;      // 发车区偏移y (单位0.1m, 范围 -20~+20, 左正右负, 对应小车 vy)

#endif
