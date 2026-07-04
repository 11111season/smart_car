#include "small_driver_uart_control.h"

small_device_value_struct motor_value;      // 驱动通讯数据结构体


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 串口接收回调函数
// 参数说明     void
// 返回参数     void
// 使用示例     uart_control_callback(1000, -1000);
// 备注信息     用于接收返回的速度信息  该函数需要在对应的串口接收中断中调用
//-------------------------------------------------------------------------------------------------------------------
void uart_control_callback(void)
{
    uint8 receive_data;                                                                     // 接收临时变量

    if(uart_query_byte(SMALL_DRIVER_UART, &receive_data))                                   // 检测接收缓冲区
    {
        if(receive_data == 0xA5 && motor_value.receive_data_buffer[0] != 0xA5)              // 判断是否收到帧头 且 当前帧头位置是否正确(防粘包)
        {
            motor_value.receive_data_count = 0;                                             // 未收到帧头或未正确接收帧头则重新计数
        }

        motor_value.receive_data_buffer[motor_value.receive_data_count ++] = receive_data;  // 存储串口数据

        if(motor_value.receive_data_count >= 11)                                            // 判断是否收到指定长度的数据
        {
            if(motor_value.receive_data_buffer[0] == 0xA5)                                  // 判断帧头是否正确
            {
                motor_value.sum_check_data = 0;                                             // 清校验和变量

                for(int i = 0; i < 10; i ++)
                {
                    motor_value.sum_check_data += motor_value.receive_data_buffer[i];       // 重新计算校验和
                }

                if(motor_value.sum_check_data == motor_value.receive_data_buffer[10])       // 校验正确
                {
                    for(int i = 0; i < 4; i ++)
                    {
                        motor_value.receive_speed_data[i]   = (((int)motor_value.receive_data_buffer[i * 2 + 2] << 8) | (int)motor_value.receive_data_buffer[i * 2 + 3]);
                    }

                    motor_value.receive_data_count = 0;                                     // 清零接收计数变量

                    memset(motor_value.receive_data_buffer, 0, 11);                         // 清零接收缓存
                }
                else
                {
                    motor_value.receive_data_count = 0;                                     // 清零接收计数变量

                    memset(motor_value.receive_data_buffer, 0, 11);                         // 清零接收缓存
                }
            }
            else
            {
                motor_value.receive_data_count = 0;                                         // 清零接收计数变量

                memset(motor_value.receive_data_buffer, 0, 11);                             // 清零接收缓存
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 设置电机占空比
// 参数说明     left_duty       左侧占空比  范围 -10000 ~ 10000  正值为正转
// 参数说明     right_duty      右侧占空比  范围 -10000 ~ 10000  正值为正转
// 返回参数     void
// 使用示例     small_driver_set_duty(1000, -1000, 1000, -1000);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void small_driver_set_duty(int16 motor_duty_1, int16 motor_duty_2, int16 motor_duty_3, int16 motor_duty_4)
{
    motor_value.send_data_buffer[0] = 0xA5;                                         // 设置帧头

    motor_value.send_data_buffer[1] = 0X01;                                         // 设置功能码

    motor_value.send_data_buffer[2] = (uint8)((motor_duty_1 & 0xFF00) >> 8);        // 发送 电机1占空比 的高八位

    motor_value.send_data_buffer[3] = (uint8)(motor_duty_1 & 0x00FF);               // 发送 电机1占空比 的低八位

    motor_value.send_data_buffer[4] = (uint8)((motor_duty_2 & 0xFF00) >> 8);        // 发送 电机2占空比 的高八位

    motor_value.send_data_buffer[5] = (uint8)(motor_duty_2 & 0x00FF);               // 发送 电机2占空比 的低八位

    motor_value.send_data_buffer[6] = (uint8)((motor_duty_4 & 0xFF00) >> 8);        // 发送 电机3占空比 的高八位

    motor_value.send_data_buffer[7] = (uint8)(motor_duty_4 & 0x00FF);               // 发送 电机3占空比 的低八位

    motor_value.send_data_buffer[8] = (uint8)((motor_duty_3 & 0xFF00) >> 8);        // 发送 电机4占空比 的高八位

    motor_value.send_data_buffer[9] = (uint8)(motor_duty_3 & 0x00FF);               // 发送 电机4占空比 的低八位

    motor_value.send_data_buffer[10] = 0;                                           // 清校验和

    for(int i = 0; i < 10; i ++)
    {
        motor_value.send_data_buffer[10] += motor_value.send_data_buffer[i];        // 计算校验位
    }

    uart_write_buffer(SMALL_DRIVER_UART, motor_value.send_data_buffer, 11);                     // 发送电机占空比的 字节包 出去
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 获取速度信息
// 参数说明     void
// 返回参数     void
// 使用示例     small_driver_get_speed();
// 备注信息     只需发送一次 驱动会循环反馈速度信息(默认10ms)
//-------------------------------------------------------------------------------------------------------------------
void small_driver_get_speed(void)
{
    motor_value.send_data_buffer[0] = 0xA5;                                         // 设置帧头

    motor_value.send_data_buffer[1] = 0X02;                                         // 设置功能码

    motor_value.send_data_buffer[2] = 0x00;                                         // 填充数据

    motor_value.send_data_buffer[3] = 0x00;                                         // 填充数据

    motor_value.send_data_buffer[4] = 0x00;                                         // 填充数据

    motor_value.send_data_buffer[5] = 0x00;                                         // 填充数据

    motor_value.send_data_buffer[6] = 0x00;                                         // 填充数据

    motor_value.send_data_buffer[7] = 0x00;                                         // 填充数据

    motor_value.send_data_buffer[8] = 0x00;                                         // 填充数据

    motor_value.send_data_buffer[9] = 0x00;                                         // 填充数据

    motor_value.send_data_buffer[10] = 0xA7;                                        // 设置校验和

    uart_write_buffer(SMALL_DRIVER_UART, motor_value.send_data_buffer, 11);         // 发送获取转速数据的 字节包 出去
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 数据结构初始化
// 参数说明     void
// 返回参数     void
// 使用示例     small_driver_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void small_driver_init(void)
{
    memset(motor_value.send_data_buffer, 0, 11);                             // 清零发送缓存

    memset(motor_value.receive_data_buffer, 0, 11);                          // 清零接收缓存

    memset(motor_value.receive_speed_data, 0, 8);                            // 清零接收速度

    motor_value.receive_data_count          = 0;

    motor_value.sum_check_data              = 0;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 串口通讯初始化
// 参数说明     void
// 返回参数     void
// 使用示例     small_driver_uart_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void small_driver_uart_init(void)
{
    uart_init(SMALL_DRIVER_UART, SMALL_DRIVER_BAUDRATE, SMALL_DRIVER_RX, SMALL_DRIVER_TX);      // 串口初始化

    uart_rx_interrupt(SMALL_DRIVER_UART, 1);                                                    // 使能串口接收中断

    small_driver_init();                                                                        // 结构体变量初始化

    small_driver_set_duty(0, 0, 0, 0);                                                          // 设置0占空比

    small_driver_get_speed();                                                                   // 获取实时速度数据
}
