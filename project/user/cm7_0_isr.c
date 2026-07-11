/*********************************************************************************************************************
* CYT4BB Opensourec Library ���� CYT4BB ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� CYT4BB ��Դ���һ����
*
* CYT4BB ��Դ�� ���������
* �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          cm7_0_isr
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 9.40.1
* ����ƽ̨          CYT4BB
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2024-1-9      pudding            first version
* 2024-5-14     pudding            ����12��pit�����ж� ���Ӳ���ע��˵��
* 2025-2-4      pudding            �Ż������ж��߼�����ֹ������ŵ��µĿ������⣬�Ż����ڲ����ʼ����߼�
* 2025-2-4      pudding            �����������ڽӿ�
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "HC06_Driver.h"
#include "VL53L5CX.h"

// 共享内存（实际定义在 control.c）
extern vision_share_t g_vision_share;

static uint32 last_frame_id = 0;

// 视觉坐标 EMA 平滑（降低逐帧抖动）
#define VISION_SMOOTH_ALPHA  0.3f   // 0=完全平滑, 1=无平滑, 越小越平滑

static float filt_err_x = 0, filt_err_y = 0;
static float filt_car_x = 0, filt_car_y = 0;
static uint8 filt_inited = 0;

static void car_send_by_vision(void)
{
    HC06_SendVisionError((int16)filt_err_x, (int16)filt_err_y);
}

// 获取滤波后的小车坐标（供 control.c 位置环使用）
float get_filtered_car_x(void) { return filt_car_x; }
float get_filtered_car_y(void) { return filt_car_y; }

static void process_vision_frame(void)
{
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)&g_vision_share, sizeof(g_vision_share));

    if(g_vision_share.frame_id == last_frame_id)
        return;

    last_frame_id = g_vision_share.frame_id;

    // 首次初始化
    if (!filt_inited) {
        filt_err_x = g_vision_share.err_x;
        filt_err_y = g_vision_share.err_y;
        filt_car_x = g_vision_share.car_x;
        filt_car_y = g_vision_share.car_y;
        filt_inited = 1;
    }

    // EMA 低通平滑
    float a = VISION_SMOOTH_ALPHA;
    float b = 1.0f - a;
    filt_err_x = filt_err_x * b + g_vision_share.err_x * a;
    filt_err_y = filt_err_y * b + g_vision_share.err_y * a;
    filt_car_x = filt_car_x * b + g_vision_share.car_x * a;
    filt_car_y = filt_car_y * b + g_vision_share.car_y * a;

    car_send_by_vision();
}

// **************************** PIT 中断函数 ****************************
void pit0_ch0_isr()                     // 定时器通道 0 的中断服务函数 (200Hz)
{
    pit_isr_flag_clear(PIT_CH0);
    flight_control(0.005);              // 由状态机分发到 take_off / hover_control 等
}

void pit0_ch1_isr()                     // 定时器通道 1 的中断服务函数
{
    pit_isr_flag_clear(PIT_CH1);
    qmc5883l_get_all();
    Mahony_Mag_Update();
}

void pit0_ch2_isr()                     // 定时器通道 2 的中断服务函数
{
    pit_isr_flag_clear(PIT_CH2);

    height_data_deal(0.040);    // VL53L5CX + IMU 高度融合（25Hz，主循环已更新传感器数据）

    // 25Hz 写显示数据到共享内存（CM7_1 读取并驱屏）
    g_vision_share.disp_roll     = eulerAngle.roll;
    g_vision_share.disp_pitch    = eulerAngle.pitch;
    g_vision_share.disp_yaw      = eulerAngle.yaw;
    g_vision_share.disp_m1       = m1;
    g_vision_share.disp_m2       = m2;
    g_vision_share.disp_m3       = m3;
    g_vision_share.disp_m4       = m4;
    g_vision_share.disp_mag_x    = qmc5883l_heading;
    g_vision_share.disp_of_dx    = of.dx;
    g_vision_share.disp_world_vx = world_data.vx;
    g_vision_share.disp_imu_gx   = imu_data.gyro_x;
    g_vision_share.disp_of_height = of.height;
    g_vision_share.disp_target   = PIDYaw.target;
    g_vision_share.disp_volt     = Battery_GetVoltage();
    g_vision_share.disp_dirty    = 1;
}

void pit0_ch10_isr()                    // 定时器通道 10 的中断服务函数
{
    pit_isr_flag_clear(PIT_CH10);

    process_vision_frame();             // 100Hz 读视觉帧 + HC06 发送

    fifo_data_count = fifo_used(&uart_data_fifo);
    if(fifo_data_count != 0)
    {
        fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);
        buff_value = atof((char*)fifo_get_data);
    }
}

void pit0_ch11_isr()                    // 定时器通道 11 的中断服务函数
{
    pit_isr_flag_clear(PIT_CH11);

    OF_data_deal(0.025);
    velocity_mahony_fusion(0.025);
}

void pit0_ch12_isr()                    // 定时器通道 12 的中断服务函数
{
    pit_isr_flag_clear(PIT_CH12);

}

void pit0_ch13_isr()                    // 定时器通道 13 的中断服务函数
{
    pit_isr_flag_clear(PIT_CH13);
}

void pit0_ch14_isr()                    // 定时器通道 14 的中断服务函数
{
    pit_isr_flag_clear(PIT_CH14);
}

void pit0_ch15_isr()
{
    pit_isr_flag_clear(PIT_CH15);
}
void pit0_ch16_isr()
{
    pit_isr_flag_clear(PIT_CH16);
}
void pit0_ch17_isr()
{
    pit_isr_flag_clear(PIT_CH17);
}
void pit0_ch18_isr()
{
    pit_isr_flag_clear(PIT_CH18);
}
void pit0_ch19_isr()
{
    pit_isr_flag_clear(PIT_CH19);
}
void pit0_ch20_isr()
{
    pit_isr_flag_clear(PIT_CH20);
}
void pit0_ch21_isr()
{
    pit_isr_flag_clear(PIT_CH21);
}

// 串口中断桩
void uart0_isr(void)
{
    if(uart_isr_mask(UART_0))
        uart_rx_interrupt_handler();
}
void uart1_isr(void)
{
    if(uart_isr_mask(UART_1))
        HC06_UART_RX_Handler();
}
void uart2_isr(void)
{
    if(uart_isr_mask(UART_2))
        upflow302_receive_callback();
}
void uart3_isr(void) { }
void uart4_isr(void)
{
    if(uart_isr_mask(UART_4))
        uart_control_callback();
}
void uart5_isr(void) { }
void uart6_isr(void) { }

// GPIO 外部中断桩
void gpio_0_exti_isr()  { }
void gpio_1_exti_isr()
{
    if(exti_flag_get(P01_0)) { }
    if(exti_flag_get(P01_1)) { }
}
void gpio_2_exti_isr()
{
    if(exti_flag_get(P02_0)) { }
    if(exti_flag_get(P02_4)) { }
}
void gpio_3_exti_isr()  { }
void gpio_4_exti_isr()  { }
void gpio_5_exti_isr()  { }
void gpio_6_exti_isr()
{
    if(exti_flag_get(P06_7))
    {
        imu660rc_callback();
        imu660rc_get_data();
        quarternion_to_rotation_matrix();
    }
}
void gpio_7_exti_isr()  { }
void gpio_8_exti_isr()  { }
void gpio_9_exti_isr()  { }
void gpio_10_exti_isr() { }
void gpio_11_exti_isr() { }
void gpio_12_exti_isr() { }
void gpio_13_exti_isr() { }
void gpio_14_exti_isr() { }
void gpio_15_exti_isr() { }
void gpio_16_exti_isr() { }
void gpio_17_exti_isr() { }
void gpio_18_exti_isr() { }
void gpio_19_exti_isr() { }
void gpio_20_exti_isr() { }
void gpio_21_exti_isr() { }
void gpio_22_exti_isr() { }
void gpio_23_exti_isr() { }
