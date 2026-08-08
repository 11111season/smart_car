#include "w25q64_storage.h"
#include "W25Q64.h"

/*****************************************************************************
 * @name       : FlashStore_SaveSettings
 * @date       : 2026-08-08
 * @function   : 保存设置区 (Region2 @0x001000): 擦除扇区1 + 写入8字节
 * @parameters : s          设置结构体指针
 * @retvalue   : 无
 * @note       : 写入前擦除整个扇区 (4KB), 期间阻塞主循环但ISR照常
******************************************************************************/
void FlashStore_SaveSettings(const flash_settings_t *s)
{
    w25q64_sector_erase(FLASH_SETTINGS_ADDR);
    w25q64_write((const uint8*)s, FLASH_SETTINGS_ADDR, sizeof(flash_settings_t));
}

/*****************************************************************************
 * @name       : FlashStore_LoadSettings
 * @date       : 2026-08-08
 * @function   : 读取设置区 (Region2): 校验 magic
 * @parameters : s          接收缓冲区
 * @retvalue   : 1=读到有效数据  0=空/未初始化
 * @note       : 无
******************************************************************************/
uint8 FlashStore_LoadSettings(flash_settings_t *s)
{
    w25q64_read((uint8*)s, FLASH_SETTINGS_ADDR, sizeof(flash_settings_t));
    return (s->magic == FLASH_MAGIC) ? 1 : 0;
}

/*****************************************************************************
 * @name       : FlashStore_SaveWaypoints
 * @date       : 2026-08-08
 * @function   : 保存航点地图 (Region1 @0x000000): [1字节数量][n×8字节坐标]
 * @parameters : wp         航点坐标数组
 * @parameters : n          航点个数 (上限 FLASH_WP_MAX)
 * @retvalue   : 无
 * @note       : 只记录已设置数量的航点
******************************************************************************/
void FlashStore_SaveWaypoints(const flash_wp_t *wp, uint8 n)
{
    if (n > FLASH_WP_MAX) n = FLASH_WP_MAX;
    w25q64_sector_erase(FLASH_MAP_ADDR);
    w25q64_write(&n, FLASH_MAP_ADDR, 1);                                   // 数量头
    if (n > 0)
        w25q64_write((const uint8*)wp, FLASH_MAP_ADDR + 1, n * sizeof(flash_wp_t));
}

/*****************************************************************************
 * @name       : FlashStore_LoadWaypoints
 * @date       : 2026-08-08
 * @function   : 读取航点地图 (Region1)
 * @parameters : wp         接收缓冲区
 * @parameters : max_n      缓冲区容量 (FLASH_WP_MAX)
 * @retvalue   : 读出航点个数 (0=空)
 * @note       : 擦除后全 0xFF, 首字节 0xFF 视为空
******************************************************************************/
uint8 FlashStore_LoadWaypoints(flash_wp_t *wp, uint8 max_n)
{
    uint8 cnt = 0;
    w25q64_read(&cnt, FLASH_MAP_ADDR, 1);
    if (cnt == 0xFF || cnt == 0 || cnt > max_n) return 0;
    w25q64_read((uint8*)wp, FLASH_MAP_ADDR + 1, cnt * sizeof(flash_wp_t));
    return cnt;
}

/*****************************************************************************
 * @name       : FlashStore_ClearAll
 * @date       : 2026-08-08
 * @function   : 清除历史数据: 航点地图 + 设置 (航点数量/发车区标志) 两扇区全部擦除
 * @parameters : 无
 * @retvalue   : 无
 * @note       : 擦除后两区全 0xFF, magic 不再匹配, 上电按默认值
******************************************************************************/
void FlashStore_ClearAll(void)
{
    w25q64_sector_erase(FLASH_MAP_ADDR);
    w25q64_sector_erase(FLASH_SETTINGS_ADDR);
}
