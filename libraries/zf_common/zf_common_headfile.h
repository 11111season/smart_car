/*********************************************************************************************************************
* CYT4BB Opensourec Library 即 CYT4BB 开源库, 一个位于官方 SDK 接口上的单片机开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 该文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是自由软件
* 你可以根据自由软件基金会发布的 GPL(GNU General Public License, GNU通用公共许可证)
* 的 GPL 的第3版(或 GPL3.0), 或任何你选择的更高版本, 来重新分发或修改它
*
* 这个库的分发是希望它能有用, 但不提供任何保证
* 甚至没有任何适销性或适用于特定用途的保证
* 更详细的说明请参见 GPL
*
* 你应该已经收到了这个库的副本, 和一份 GPL 的副本
* 如果没有, 请访问 <https://www.gnu.org/licenses/>
*
* 版权注意:
* 这个库使用了 GPL3.0 开源协议, 所有产品化均为非开源版本
* 如需商用英译见 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件
* 版权见 libraries 文件夹内 各文件夹下的 LICENSE 文件
* 欢迎各位使用并反馈问题, 修改或发布时请保留逐飞科技的相关版权信息
*
* 文件名称          zf_common_headfile
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹中的 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 技术支持          https://seekfree.taobao.com/
*
* 修改记录
* 日期             作者               备注
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#ifndef _zf_common_headfile_h_
#define _zf_common_headfile_h_


#include "stdio.h"
#include "stdint.h"
#include "string.h"

//===================================================芯片 SDK 底层===================================================
#include "cy_project.h"
#include "cy_device_headers.h"
#include "arm_math.h"
//===================================================芯片 SDK 底层===================================================

//====================================================开源库公共层====================================================
#include "zf_common_typedef.h"
#include "zf_common_clock.h"
#include "zf_common_debug.h"
#include "zf_common_fifo.h"
#include "zf_common_font.h"
#include "zf_common_function.h"
#include "zf_common_interrupt.h"
//====================================================开源库公共层====================================================

//===================================================芯片底层驱动层===================================================
#include "zf_driver_adc.h"
#include "zf_driver_delay.h"
#include "zf_driver_dma.h"
#include "zf_driver_encoder.h"
#include "zf_driver_exti.h"
#include "zf_driver_flash.h"
#include "zf_driver_gpio.h"
#include "zf_driver_ipc.h"
#include "zf_driver_pit.h"
#include "zf_driver_pwm.h"
#include "zf_driver_soft_iic.h"
#include "zf_driver_soft_spi.h"
#include "zf_driver_spi.h"
#include "zf_driver_timer.h"
#include "zf_driver_uart.h"
//===================================================芯片底层驱动层===================================================

//===================================================外设设备驱动层===================================================
#include "zf_device_ble6a20.h"
#include "zf_device_dl1a.h"
#include "zf_device_dl1b.h"
#include "zf_device_gnss.h"
#include "zf_device_icm20602.h"
#include "zf_device_imu660ra.h"
#include "zf_device_imu660rb.h"
#include "zf_device_imu660rc.h"
#include "zf_device_imu963ra.h"
#include "zf_device_ips114.h"
#include "zf_device_ips200.h"
#include "zf_device_ips200pro.h"
#include "zf_device_key.h"
#include "zf_device_lora3a22.h"
#include "zf_device_menc15a.h"
#include "zf_device_oled.h"
#include "zf_device_mt9v03x.h"
#include "zf_device_pmw3901.h"
#include "zf_device_tft180.h"
#include "zf_device_tsl1401.h"
#include "zf_device_type.h"
#include "zf_device_upflow302.h"
#include "zf_device_uart_receiver.h"
#include "zf_device_wifi_spi.h"
#include "zf_device_wifi_uart.h"
#include "zf_device_wireless_uart.h"
//===================================================外设设备驱动层===================================================

//=====================================================逐飞应用层=====================================================
#include "seekfree_assistant.h"
#include "seekfree_assistant_interface.h"
//=====================================================逐飞应用层=====================================================
#endif
