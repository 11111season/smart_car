#include "HC06_Driver.h"
#include "zf_driver_uart.h"
#include "zf_common_fifo.h"

#define HC06_SCB  get_scb_module(UART_1)

// HC06 专用 RX FIFO
static uint8_t hc06_rx_buf[32];
fifo_struct hc06_rx_fifo;

// 帧解析状态
typedef enum {
    FRAME_WAIT_START,   // 等待 '#'
    FRAME_RECV_DATA,    // 正在接收数字
} frame_state_t;

static frame_state_t frame_state = FRAME_WAIT_START;
static uint8_t frame_buffer[8];
static uint8_t frame_idx = 0;

// 命令输出
static volatile uint8_t hc06_cmd_num = 0;
static volatile uint8_t hc06_cmd_ready = 0;

// 解析完整帧 "#N$" → 提取数值 N
static void ParseFrameData(uint8_t *buf, uint8_t len)
{
    uint8_t val = 0;
    for(uint8_t i = 0; i < len; i++)
    {
        if(buf[i] >= '0' && buf[i] <= '9')
            val = val * 10 + (buf[i] - '0');
        else
            break;
    }
    hc06_cmd_num = val;
    hc06_cmd_ready = 1;
}

// 非阻塞发送
static void hc06_tx_byte(uint8 data)
{
    volatile stc_SCB_t *scb = HC06_SCB;
    uint32 timeout = 5000;
    while(Cy_SCB_GetNumInTxFifo(scb) >= 16)
    {
        if(--timeout == 0) return;
    }
    Cy_SCB_WriteTxFifo(scb, data);
}

static void hc06_tx_string(const char *str)
{
    while(*str)
    {
        hc06_tx_byte(*str++);
    }
}

void HC06_Init(uint32 baudrate)
{
    // 初始化 RX FIFO
    fifo_init(&hc06_rx_fifo, FIFO_DATA_8BIT, hc06_rx_buf, 32);

    // 初始化 UART
    uart_init(UART_1, baudrate, UART1_TX_P04_1, UART1_RX_P04_0);

    // 使能 UART1 接收中断（否则中断处理函数 HC06_UART_RX_Handler 不会被调用）
    uart_rx_interrupt(UART_1, 1);

    // 状态初始化
    frame_state = FRAME_WAIT_START;
    frame_idx = 0;
    hc06_cmd_num = 0;
    hc06_cmd_ready = 0;
}

void HC06_SendCmd(const char *str)
{
    hc06_tx_string(str);
}

void HC06_SendVisionError(int16 err_x, int16 err_y, uint8_t flag)
{
    char buf[32];
    sprintf(buf, "#%d,%d,%d$", err_x, err_y, flag);
    hc06_tx_string(buf);
}

// 中断处理：仅将字节写入 FIFO
void HC06_UART_RX_Handler(void)
{
    uint8 byte;
    while(uart_query_byte(UART_1, &byte))
    {
        fifo_write_buffer(&hc06_rx_fifo, &byte, 1);
    }
}

// 主循环任务：从 FIFO 读取并解析 "#N$" 帧
void HC06_Task(void)
{
    uint8_t data;

    while(fifo_read_element(&hc06_rx_fifo, &data, FIFO_READ_AND_CLEAN) == FIFO_SUCCESS)
    {
        if(frame_state == FRAME_WAIT_START)
        {
            if(data == '#')
            {
                frame_state = FRAME_RECV_DATA;
                frame_idx = 0;
            }
        }
        else if(frame_state == FRAME_RECV_DATA)
        {
            if(data == '$')
            {
                if(frame_idx > 0)
                {
                    frame_buffer[frame_idx] = '\0';
                    ParseFrameData(frame_buffer, frame_idx);
                }
                frame_state = FRAME_WAIT_START;
            }
            else
            {
                if(frame_idx < sizeof(frame_buffer) - 1)
                {
                    frame_buffer[frame_idx++] = data;
                }
            }
        }
    }
}

// 获取 HC06 命令（取一次后自动清零）
uint8_t HC06_GetCmd(void)
{
    if(hc06_cmd_ready)
    {
        hc06_cmd_ready = 0;
        return hc06_cmd_num;
    }
    return 0;
}
