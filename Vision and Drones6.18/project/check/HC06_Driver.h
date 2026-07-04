// =============================================================================
// HC06_Driver.h - HC06 蓝牙串口驱动 (帧协议 "#N$")
// =============================================================================
#ifndef HC06_DRIVER_H_
#define HC06_DRIVER_H_

#include "zf_common_typedef.h"
#include "zf_common_fifo.h"

void HC06_Init(uint32 baudrate);
void HC06_SendCmd(const char *str);
void HC06_SendVisionError(int16 err_x, int16 err_y);
void HC06_Task(void);
uint8_t HC06_GetCmd(void);

// UART 中断处理函数（需在 uart1 中断中调用）
void HC06_UART_RX_Handler(void);

// HC06 RX FIFO（允许外部直接访问）
extern fifo_struct hc06_rx_fifo;

#endif /* HC06_DRIVER_H_ */
