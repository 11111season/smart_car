/*********************************************************************************************************************
* CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          VL53L5CX
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2026-07-11       Trae              first version (ST ULD API v2.0.1)
********************************************************************************************************************/
/*********************************************************************************************************************
* 接线定义：
*                   ------------------------------------
*                   模块管脚            单片机管脚
*                   SCL                查看 VL53L5CX.h 中 VL53L5CX_SCL_PIN 宏定义
*                   SDA                查看 VL53L5CX.h 中 VL53L5CX_SDA_PIN 宏定义
*                   PWR_EN             查看 VL53L5CX.h 中 VL53L5CX_PWR_EN_PIN 宏定义
*                   LPn                查看 VL53L5CX.h 中 VL53L5CX_LPn_PIN 宏定义
*                   INT                查看 VL53L5CX.h 中 VL53L5CX_INT_PIN 宏定义
*                   VCC                5V电源
*                   GND                电源地
*                   ------------------------------------
********************************************************************************************************************/

#include "zf_common_debug.h"
#include <stdio.h>

#include "zf_driver_delay.h"
#include "zf_driver_exti.h"
#include "zf_driver_soft_iic.h"
#include "zf_driver_gpio.h"

#include "VL53L5CX.h"
#include "zf_device_config.h"
#include "zf_device_type.h"

/*
 * ST VL53L5CX ULD API
 * Note: Conflicting functions are renamed with _st prefix (see vl53l5cx_api.h).
 * We #undef the macros here so our wrapper functions keep their original names.
 */
#include "vl53l5cx_api.h"
#undef vl53l5cx_init
#undef vl53l5cx_start_ranging
#undef vl53l5cx_stop_ranging
#undef vl53l5cx_set_resolution

//================================================全局变量定义================================================
uint8               vl53l5cx_init_flag          = 0;                                // 模块完成初始化标志位
uint8               vl53l5cx_finsh_flag         = 0;                                // 数据更新完成标志位
uint16              vl53l5cx_distance_mm        = 8192;                             // 首区（Zone 0）距离（毫米）
vl53l5cx_result_t   vl53l5cx_zone_result;                                           // 多区测距结果
vl53l5cx_config_t   vl53l5cx_config;                                                // 当前配置参数
//================================================全局变量定义================================================

#if VL53L5CX_USE_SOFT_IIC
static soft_iic_info_struct vl53l5cx_iic_struct;
#endif

/* ST API 设备配置和结果结构体 */
static VL53L5CX_Configuration vl53l5cx_dev;     /* ST API 传感器配置结构体（含 84KB 固件数据） */
static VL53L5CX_ResultsData  vl53l5cx_results;  /* ST API 测距结果结构体 */


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 VL53L5CX
// 参数说明     void
// 返回参数     uint8           1-初始化失败 0-初始化成功
// 使用示例     vl53l5cx_init();
// 备注信息     初始化包括 GPIO 初始化、I2C 初始化、硬件复位、固件下载、传感器探测及配置
//-------------------------------------------------------------------------------------------------------------------
uint8 vl53l5cx_init(void)
{
    uint8 status = 1;
    uint8 is_alive = 0;

    // 1. I2C 初始化
#if VL53L5CX_USE_SOFT_IIC
    soft_iic_init(&vl53l5cx_iic_struct, VL53L5CX_DEV_ADDR, VL53L5CX_SOFT_IIC_DELAY, VL53L5CX_SCL_PIN, VL53L5CX_SDA_PIN);
#endif

    // 2. GPIO 初始化（PWR_EN 初始拉低、LPn 初始拉低）
    gpio_init(VL53L5CX_PWR_EN_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(VL53L5CX_LPn_PIN,    GPO, GPIO_LOW, GPO_PUSH_PULL);

#if VL53L5CX_INT_ENABLE
    gpio_init(VL53L5CX_INT_PIN, GPI, GPIO_LOW, GPO_PUSH_PULL);
#endif

    // 3. 关联平台层 I2C 接口到 ST API 结构体
    vl53l5cx_dev.platform.iic = &vl53l5cx_iic_struct;

    // 4. 硬件复位（PWR_EN + LPn 时序）
    VL53L5CX_Reset_Sensor(&(vl53l5cx_dev.platform));
//    printf("  [VL53L5CX] Reset done\r\n");

    // 5. 检查传感器是否在线（ST API）
    status = vl53l5cx_is_alive(&vl53l5cx_dev, &is_alive);
//    printf("  [VL53L5CX] Is alive: %d (status=%d)\r\n", is_alive, status);
    if (!is_alive || status)
    {
//        printf("  [VL53L5CX] Sensor not detected!\r\n");
        return 1;
    }

    // 6. 初始化传感器（加载固件+配置）— 这是最关键的一步！
    //    耗时约几百毫秒，包含 84KB 固件下载、NVM 校准、串扰补偿等
    status = st_vl53l5cx_init(&vl53l5cx_dev);
//    printf("  [VL53L5CX] ULD init: %d\r\n", status);
    if (status)
    {
//        printf("  [VL53L5CX] ULD init failed!\r\n");
        return 1;
    }
//    printf("  [VL53L5CX] ULD ready! (version: %s)\r\n", VL53L5CX_API_REVISION);

    // 7. 设置默认配置参数
    vl53l5cx_config.resolution           = VL53L5CX_RESOLUTION_4X4;             // 默认 4x4 分辨率
    vl53l5cx_config.timing_budget_ms     = 30;                                   // 默认 30ms 积分时间
    vl53l5cx_config.ranging_frequency_hz = 25;                                   // 默认 25Hz 测距频率

    vl53l5cx_zone_result.zone_count      = VL53L5CX_RESOLUTION_4X4;

    // 8. 应用配置到传感器
    {
        // 使用 ST API 设置分辨率
        status = st_vl53l5cx_set_resolution(&vl53l5cx_dev, VL53L5CX_RESOLUTION_4X4);
//        printf("  [VL53L5CX] Set resolution: %d\r\n", status);

        // 使用 ST API 设置测距频率
        status = vl53l5cx_set_ranging_frequency_hz(&vl53l5cx_dev, vl53l5cx_config.ranging_frequency_hz);
//        printf("  [VL53L5CX] Set frequency: %d\r\n", status);

        // 使用 ST API 设置积分时间（需要切换到 Autonomous 模式）
        status = vl53l5cx_set_integration_time_ms(&vl53l5cx_dev, vl53l5cx_config.timing_budget_ms);
//        printf("  [VL53L5CX] Set integration time: %d\r\n", status);
    }

    // 9. 标记初始化完成
    vl53l5cx_init_flag = 1;

#if VL53L5CX_INT_ENABLE
    exti_init(VL53L5CX_INT_PIN, EXTI_TRIGGER_FALLING);
    vl53l5cx_finsh_flag = 0;
#endif

    // 注册到逐飞外设类型系统
    set_tof_type(TOF_VL53L5CX, vl53l5cx_int_handler);

//    printf("  [VL53L5CX] Init OK\r\n");
    return 0;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动测距
// 参数说明     void
// 返回参数     void
// 使用示例     vl53l5cx_start_ranging();
// 备注信息     使用 ST API 启动连续测距模式
//-------------------------------------------------------------------------------------------------------------------
void vl53l5cx_start_ranging(void)
{
    uint8 status;

    if (0 == vl53l5cx_init_flag)
    {
        return;
    }

    // 使用 ST API 启动测距（内部自动配置固件 + 清除中断 + 写 RANGE_START）
    status = st_vl53l5cx_start_ranging(&vl53l5cx_dev);

//    printf("  [VL53L5CX] Start ranging: %d\r\n", status);
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     停止测距
// 参数说明     void
// 返回参数     void
// 使用示例     vl53l5cx_stop_ranging();
// 备注信息     使用 ST API 停止传感器测距
//-------------------------------------------------------------------------------------------------------------------
void vl53l5cx_stop_ranging(void)
{
    if (0 == vl53l5cx_init_flag)
    {
        return;
    }

    st_vl53l5cx_stop_ranging(&vl53l5cx_dev);
    vl53l5cx_finsh_flag = 0;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取测距数据
// 参数说明     void
// 返回参数     void
// 使用示例     vl53l5cx_get_distance();
// 备注信息     使用 ST API 检查并读取测距数据 更新到 vl53l5cx_zone_result
//              状态码 5 或 9 表示有效测距
//-------------------------------------------------------------------------------------------------------------------
void vl53l5cx_get_distance(void)
{
    uint8 i;
    uint8 is_ready = 0;
    uint8 status;
    uint8 zone_count;

    if (0 == vl53l5cx_init_flag)
    {
        return;
    }

    // 1. 使用 ST API 检查数据是否就绪（通过 streamcount 检测）
    status = vl53l5cx_check_data_ready(&vl53l5cx_dev, &is_ready);

    if (is_ready && (0 == status))
    {
        // 2. 使用 ST API 读取测距数据（自动解析块头格式）
        status = vl53l5cx_get_ranging_data(&vl53l5cx_dev, &vl53l5cx_results);

        if (0 == status)
        {
            // 3. 将 ST API 结果转换到用户结果结构体
            zone_count = vl53l5cx_zone_result.zone_count;
            if (0 == zone_count)
            {
                zone_count = VL53L5CX_RESOLUTION_4X4;
            }

            for (i = 0; i < zone_count; i++)
            {
                // 从 ST API 结果中提取距离和状态
                // 注意：VL53L5CX_NB_TARGET_PER_ZONE = 1，所以 target index = i
                int16_t dist = vl53l5cx_results.distance_mm[i];
                uint8_t st  = vl53l5cx_results.target_status[i];

                // 有效性检查（ST 定义：5 或 9 = 有效）
                if ((5 != st && 9 != st) || dist <= 0)
                {
                    vl53l5cx_zone_result.distance_mm[i] = 8192;                 // 无效数据标记
                }
                else
                {
                    vl53l5cx_zone_result.distance_mm[i] = (uint16)dist;
                }
                vl53l5cx_zone_result.status[i] = st;
            }

            // 更新首区距离
            vl53l5cx_distance_mm = vl53l5cx_zone_result.distance_mm[0];

            // 标记数据更新完成
            vl53l5cx_finsh_flag = 1;
        }
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     VL53L5CX INT 中断响应函数
// 参数说明     void
// 返回参数     void
// 使用示例     在 VL53L5CX_INT_PIN 对应的外部中断处理函数中调用
// 备注信息     中断触发后自动调用 vl53l5cx_get_distance() 读取数据
//-------------------------------------------------------------------------------------------------------------------
void vl53l5cx_int_handler(void)
{
#if VL53L5CX_INT_ENABLE
    if (0 == vl53l5cx_init_flag)
    {
        return;
    }

    // 中断模式下直接读取数据（ST API 会自动处理中断清除）
    vl53l5cx_get_distance();
#else
    (void)vl53l5cx_init_flag;
#endif
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置传感器分辨率
// 参数说明     uint8 mode      分辨率模式 VL53L5CX_RESOLUTION_4X4=16 或 VL53L5CX_RESOLUTION_8X8=64
// 返回参数     uint8           1-设置失败 0-设置成功
// 使用示例     vl53l5cx_set_resolution(VL53L5CX_RESOLUTION_4X4);
// 备注信息     使用 ST API 设置分辨率 需在停止测距状态下调用
//-------------------------------------------------------------------------------------------------------------------
uint8 vl53l5cx_set_resolution(uint8 mode)
{
    uint8 st;

    if (0 == vl53l5cx_init_flag)
    {
        return 1;
    }

    if (VL53L5CX_RESOLUTION_4X4 != mode && VL53L5CX_RESOLUTION_8X8 != mode)
    {
        return 1;
    }

    // 使用 ST API 设置分辨率
    if (VL53L5CX_RESOLUTION_4X4 == mode)
    {
        st = st_vl53l5cx_set_resolution(&vl53l5cx_dev, VL53L5CX_RESOLUTION_4X4);
    }
    else
    {
        st = st_vl53l5cx_set_resolution(&vl53l5cx_dev, VL53L5CX_RESOLUTION_8X8);
    }

    if (0 == st)
    {
        vl53l5cx_config.resolution = mode;
        vl53l5cx_zone_result.zone_count = mode;
        return 0;
    }

    return 1;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新配置到传感器
// 参数说明     void
// 返回参数     uint8           1-配置失败 0-配置成功
// 使用示例     vl53l5cx_update_config();
// 备注信息     使用 ST API 将 vl53l5cx_config 中的参数写入传感器
//-------------------------------------------------------------------------------------------------------------------
uint8 vl53l5cx_update_config(void)
{
    uint8 st;

    if (0 == vl53l5cx_init_flag)
    {
        return 1;
    }

    // 1. 分辨率
    if (VL53L5CX_RESOLUTION_4X4 == vl53l5cx_config.resolution)
    {
        st = st_vl53l5cx_set_resolution(&vl53l5cx_dev, VL53L5CX_RESOLUTION_4X4);
    }
    else
    {
        st = st_vl53l5cx_set_resolution(&vl53l5cx_dev, VL53L5CX_RESOLUTION_8X8);
    }
    if (st) return st;

    // 2. 积分时间
    st = vl53l5cx_set_integration_time_ms(&vl53l5cx_dev,
            (uint32_t)vl53l5cx_config.timing_budget_ms);
    if (st) return st;

    // 3. 测距频率
    st = vl53l5cx_set_ranging_frequency_hz(&vl53l5cx_dev,
            vl53l5cx_config.ranging_frequency_hz);
    if (st) return st;

    // 4. 更新结果结构体区数量
    vl53l5cx_zone_result.zone_count = vl53l5cx_config.resolution;

    return 0;
}
