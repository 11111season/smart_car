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
* 文件名称          cm7_1_isr
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹中的 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 技术支持          https://seekfree.taobao.com/
*
* 修改记录
* 日期             作者               备注
* 2024-1-9      pudding            first version
* 2024-5-14     pudding            新增12个pit定时器中断 添加注释说明
* 2025-2-4      pudding            优化定时器中断逻辑, 防止长时间调用导致的错误问题, 优化在线采集初始化逻辑
* 2025-2-4      pudding            新增在线调试接口
********************************************************************************************************************/

#include "zf_common_headfile.h"

// **************************** PIT中断函数 ****************************
void pit0_ch0_isr()                     // 定时器通道 0 的中断处理函数
{
    pit_isr_flag_clear(PIT_CH0);



}

void pit0_ch1_isr()                     // 定时器通道 1 的中断处理函数
{
    pit_isr_flag_clear(PIT_CH1);

}

void pit0_ch2_isr()                     // 定时器通道 2 的中断处理函数
{
    pit_isr_flag_clear(PIT_CH2);

}

void pit0_ch10_isr()                    // 定时器通道 10 的中断处理函数
{
    pit_isr_flag_clear(PIT_CH10);

}

void pit0_ch11_isr()                    // 定时器通道 11 的中断处理函数
{
    pit_isr_flag_clear(PIT_CH11);

}

void pit0_ch12_isr()                    // 定时器通道 12 的中断处理函数
{
    pit_isr_flag_clear(PIT_CH12);

}

void pit0_ch13_isr()                    // 定时器通道 13 的中断处理函数
{
    pit_isr_flag_clear(PIT_CH13);

}

void pit0_ch14_isr()                    // 定时器通道 14 的中断处理函数
{
    pit_isr_flag_clear(PIT_CH14);

}

void pit0_ch15_isr()                    // 定时器通道 15 的中断处理函数
{
    pit_isr_flag_clear(PIT_CH15);

}

void pit0_ch16_isr()                    // 定时器通道 16 的中断处理函数
{
    pit_isr_flag_clear(PIT_CH16);

}

void pit0_ch17_isr()                    // 定时器通道 17 的中断处理函数
{
    pit_isr_flag_clear(PIT_CH17);

}

void pit0_ch18_isr()                    // 定时器通道 18 的中断处理函数
{
    pit_isr_flag_clear(PIT_CH18);

}

void pit0_ch19_isr()                    // 定时器通道 19 的中断处理函数
{
    pit_isr_flag_clear(PIT_CH19);

}

void pit0_ch20_isr()                    // 定时器通道 20 的中断处理函数
{
    pit_isr_flag_clear(PIT_CH20);

}

void pit0_ch21_isr()                    // 定时器通道 21 的中断处理函数
{
    pit_isr_flag_clear(PIT_CH21);
    tsl1401_collect_pit_handler();
}
// **************************** PIT中断函数 ****************************


// **************************** 串口中断函数 ****************************
// 串口0默认为调试串口
void uart0_isr (void)
{
    if(uart_isr_mask(UART_0))            // 串口0接收中断
    {

#if DEBUG_UART_USE_INTERRUPT             // 使能调试串口接收中断
        debug_interrupr_handler();       // 处理 debug 串口接收的数据, 数据会被 debug 缓存后被获取
#endif                                   // 如果想修改 DEBUG_UART_INDEX 或其他位置请放到对应的串口中断里去

    }
    else                                 // 串口0发送中断
    {



    }
}

void uart1_isr (void)
{
    if(uart_isr_mask(UART_1))            // 串口1接收中断
    {

        wireless_module_uart_handler();

    }
    else                                // 串口1发送中断
    {



    }
}

void uart2_isr (void)
{
    if(uart_isr_mask(UART_2))            // 串口2接收中断
    {

        gnss_uart_callback();

    }
    else                                // 串口2发送中断
    {



    }
}

void uart3_isr (void)
{
    if(uart_isr_mask(UART_3))            // 串口3接收中断
    {



    }
    else                                // 串口3发送中断
    {



    }
}

void uart4_isr (void)
{
    if(uart_isr_mask(UART_4))            // 串口4接收中断
    {

        uart_receiver_handler();                                                                // 串口接收回调函数

    }
    else                                // 串口4发送中断
    {



    }
}

void uart5_isr (void)
{
    if(uart_isr_mask(UART_5))            // 串口5接收中断
    {



    }
    else                                // 串口5发送中断
    {



    }
}

void uart6_isr (void)
{
    if(uart_isr_mask(UART_6))            // 串口6接收中断
    {



    }
    else                                // 串口6发送中断
    {



    }
}
// **************************** 串口中断函数 ****************************

// **************************** 外部中断函数 ****************************
void gpio_0_exti_isr()                  // 外部 GPIO_0 中断处理函数
{



}

void gpio_1_exti_isr()                  // 外部 GPIO_1 中断处理函数
{
    if(exti_flag_get(P01_0))		// 示例P1_0端口外部中断判断
    {




    }
    if(exti_flag_get(P01_1))
    {


    }
}

void gpio_2_exti_isr()                  // 外部 GPIO_2 中断处理函数
{
    if(exti_flag_get(P02_0))
    {


    }
    if(exti_flag_get(P02_4))
    {


    }

}

void gpio_3_exti_isr()                  // 外部 GPIO_3 中断处理函数
{



}

void gpio_4_exti_isr()                  // 外部 GPIO_4 中断处理函数
{



}

void gpio_5_exti_isr()                  // 外部 GPIO_5 中断处理函数
{



}


void gpio_6_exti_isr()                  // 外部 GPIO_6 中断处理函数
{



}

void gpio_7_exti_isr()                  // 外部 GPIO_7 中断处理函数
{



}

void gpio_8_exti_isr()                  // 外部 GPIO_8 中断处理函数
{



}

void gpio_9_exti_isr()                  // 外部 GPIO_9 中断处理函数
{



}

void gpio_10_exti_isr()                  // 外部 GPIO_10 中断处理函数
{



}

void gpio_11_exti_isr()                  // 外部 GPIO_11 中断处理函数
{



}

void gpio_12_exti_isr()                  // 外部 GPIO_12 中断处理函数
{



}

void gpio_13_exti_isr()                  // 外部 GPIO_13 中断处理函数
{



}

void gpio_14_exti_isr()                  // 外部 GPIO_14 中断处理函数
{



}

void gpio_15_exti_isr()                  // 外部 GPIO_15 中断处理函数
{



}

void gpio_16_exti_isr()                  // 外部 GPIO_16 中断处理函数
{



}

void gpio_17_exti_isr()                  // 外部 GPIO_17 中断处理函数
{



}

void gpio_18_exti_isr()                  // 外部 GPIO_18 中断处理函数
{



}

void gpio_19_exti_isr()                  // 外部 GPIO_19 中断处理函数
{



}

void gpio_20_exti_isr()                  // 外部 GPIO_20 中断处理函数
{



}

void gpio_21_exti_isr()                  // 外部 GPIO_21 中断处理函数
{



}

void gpio_22_exti_isr()                  // 外部 GPIO_22 中断处理函数
{



}

void gpio_23_exti_isr()                  // 外部 GPIO_23 中断处理函数
{



}
// **************************** 外部中断函数 ****************************
