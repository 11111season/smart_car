#ifndef __APP_MENU_H_
#define __APP_MENU_H_

#include "zf_common_headfile.h"   // RGB565_* 颜色宏定义在 zf_common_font.h, 总头文件一并引入

void App_Menu_Init(void);
void App_Menu_Task(void);

/* 惯导设置全局 (菜单写入, 掉电存储经 W25Q64 覆盖, 惯性导航读取; 实际总航点数 = wp_set + launch_enable) */
extern uint8_t wp_set;            // 已确认航点总数 (不含发车区, 0~9; v1.4.9 起可设0=不记录惯导, 仅发车区偏移点) = 惯导 BCN_MAX
extern uint8_t launch_enable;     // 发车区航点启用: 1=启用 0=禁用 (决定 WP_MAX 是否 +1)
extern uint8_t launch_step;       // 发车区偏移步长 (单位0.05m, 范围 1~10 → 0.05~0.50m, 默认4=0.20m; 菜单"设置步长"写, 掉电存储)
extern uint8_t coord_step;        // 坐标模式航点输入步长 (单位0.05m, 范围 1~10 → 0.05~0.50m, 默认4=0.20m; 菜单"设置步长(坐标模式)"写, 掉电存储)
extern int16   launch_off_x;      // 发车区偏移x (单位0.01m, 范围 -200~+200, 前正后负, 对应小车 vx)
extern int16   launch_off_y;      // 发车区偏移y (单位0.01m, 范围 -200~+200, 左正右负, 对应小车 vy)
extern uint8_t spd_limit_x;       // 惯导速度限幅x (单位0.05m/s, 范围 0~10 → 0~0.5m/s, 对应 target_vx 限幅)
extern uint8_t spd_limit_y;       // 惯导速度限幅y (单位0.05m/s, 范围 0~10 → 0~0.5m/s, 对应 target_vy 限幅)
extern uint8_t pos_limit_x;       // 位置环速度限幅x (单位0.05m/s, 范围 0~10 → 0~0.5m/s, 位置环 target_vx 限幅)
extern uint8_t pos_limit_y;       // 位置环速度限幅y (单位0.05m/s, 范围 0~10 → 0~0.5m/s, 位置环 target_vy 限幅)
extern uint8_t rec_mode;          // 航点记录模式: 0=手动推车记录 1=坐标输入 (菜单"设置航点记录模式"写, 掉电存储)

#endif
