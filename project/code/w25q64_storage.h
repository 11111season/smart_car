/*********************************************************************************************************************
* CYT4BB Opensourec Library W25Q64 掉电存储分区模块 (类 STM32 内部flash区域划分)
* Copyright (c) 2026 YourName
*
* 文件名称   w25q64_storage
* 版本信息   v1.4.4
* 开发环境   IAR 9.40.1
* 适用平台   CYT4BB
*
* 存储区域划分 (扇区对齐, 人为分区便于管理):
*   Region 1 @ 0x000000 (扇区0): 惯导地图数据 — 航点地图
*       格式: [1字节数量][数量×8字节坐标], 最多 FLASH_WP_MAX=9 个航点, 有多少航点记录多少
*   Region 2 @ 0x001000 (扇区1): 设置参数 — 航点数量 + 发车区设置 + 惯导速度限幅 + 位置环速度限幅
*       格式: 12字节 flash_settings_t (magic 校验)
*
* 上电流程: init.c 读 Region2 → 覆盖菜单全局 (wp_set/launch_enable/launch_off_x/y/spd_limit_x/y/pos_limit_x/y)
*           → 再由 Inav_LoadMap() 读 Region1 → 设 bcn_max/wp_max + 构建导航航点
* 修改记录
* 日期        作者        备注
* 2026-08-08  YourName    first version
********************************************************************************************************************/

#ifndef CODE_W25Q64_STORAGE_H_
#define CODE_W25Q64_STORAGE_H_

#include "zf_common_typedef.h"

// ====================================================区域与常量====================================================
#define FLASH_WP_MAX        9            // 航点物理上限 (与惯性导航数组上限一致)
#define FLASH_MAGIC         0x5A         // 设置区有效标志 (读回非该值 = 未写入/已清除)

#define FLASH_MAP_ADDR      0x000000     // Region 1: 航点地图 (扇区0)
#define FLASH_SETTINGS_ADDR 0x001000     // Region 2: 设置参数 (扇区1)

// ====================================================数据类型====================================================
// 航点坐标 (m), 与惯性导航 pt_t 同布局 (float x, float y)
typedef struct { float x, y; } flash_wp_t;

// Region 2 设置结构体 (12 字节)
typedef struct {
    uint8  magic;          // 0x5A = 已写入
    uint8  wp_set;         // 航点总数 (不含发车区) = 惯导 BCN_MAX
    uint8  launch_enable;  // 发车区航点启用 (1/0), 决定 WP_MAX 是否 +1
    uint8  _pad;           // 对齐填充
    int16  launch_off_x;   // 发车区偏移 x (0.1m, 菜单步进, 范围 ±2m)
    int16  launch_off_y;   // 发车区偏移 y (0.1m, 菜单步进, 范围 ±2m)
    uint8  spd_limit_x;    // 惯导速度限幅 x (0.05m/s, 菜单步进, 范围 0~0.5m/s → 存0~10)
    uint8  spd_limit_y;    // 惯导速度限幅 y (0.05m/s, 菜单步进, 范围 0~0.5m/s → 存0~10)
    uint8  pos_limit_x;    // 位置环速度限幅 x (0.05m/s, 菜单步进, 范围 0~0.5m/s → 存0~10)
    uint8  pos_limit_y;    // 位置环速度限幅 y (0.05m/s, 菜单步进, 范围 0~0.5m/s → 存0~10)
} flash_settings_t;

// ====================================================函数声明====================================================
// 保存设置区 (Region2): 擦除扇区1 + 写入12字节
void    FlashStore_SaveSettings(const flash_settings_t *s);

// 读取设置区 (Region2): 返回 1=读到有效数据(magic匹配), 0=空/未初始化
uint8   FlashStore_LoadSettings(flash_settings_t *s);

// 保存航点地图 (Region1): 擦除扇区0 + [1字节数量][n×8字节坐标]
void    FlashStore_SaveWaypoints(const flash_wp_t *wp, uint8 n);

// 读取航点地图 (Region1): 返回读出航点个数 (0=空), 上限 max_n
uint8   FlashStore_LoadWaypoints(flash_wp_t *wp, uint8 max_n);

// 清除历史数据: 航点地图 + 航点数量 + 发车区启用标志 全部清零 (两扇区擦除)
void    FlashStore_ClearAll(void);

#endif /* CODE_W25Q64_STORAGE_H_ */
