#ifndef CODE_HC06_DRIVER_H_
#define CODE_HC06_DRIVER_H_

#include "zf_common_headfile.h"

void HC06_Init(uint32 baudrate);
void HC06_SendCmd(const char *cmd);
void HC06_SendVisionError(int16 err_x, int16 err_y, uint8_t flag);
void HC06_UART_RX_Handler(void);
uint8_t HC06_GetCmd(void);

extern fifo_struct hc06_rx_fifo;

#endif /* CODE_HC06_DRIVER_H_ */
