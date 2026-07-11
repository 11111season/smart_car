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
#include "Encoder.h"
#include "My_imu660ra.h"
#include "HC06_Driver.h"
#include "inertial_nav.h"   // 暂时注释
volatile int count = 0;
volatile uint64_t time_us = 0;
volatile int Start_Pid_Flag = 0;
volatile int Stop_Pid_Flag = 0;
volatile uint8_t send_uart_cmd_flag = 0;
volatile uint8_t send_uart_cmd_value = 0;

// **************************** PIT�жϺ��� ****************************
void pit0_ch0_isr()
{
    pit_isr_flag_clear(PIT_CH0);
    count++;

    Encoder_Data_Get();
    My_Imu660ra_Update();
    MagYaw_Update();                                    // 磁力计融合
    InertialNav_PosUpdate();

    PositionControl_Update();

    if (bcn_nav_on) {
        target_vx = bcn_nav_vx;
        target_vy = bcn_nav_vy;
        angle_target = bcn_nav_angle;
    }

    if (angle_pid_yaw.Enable) {
        float yaw = My_Imu660ra_GetYaw();
        PID_SetTarget(&angle_pid_yaw, angle_target);
        float omega_des = PID_Calculate(&angle_pid_yaw, yaw);   // 角度PID直接输出 deg/s
        float omega_rad = omega_des * (PI / 180.0f);
        Mecanum_Move(target_vx, target_vy, omega_rad);
    } else {
        Mecanum_Move(target_vx, target_vy, 0.0f);
    }

    //AntiSlipControl();
    Motor_PID_Control_All();
}

void pit0_ch1_isr()                     // ��ʱ��ͨ�� 1 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH1);
    key_scanner();
//    if(key_get_state(KEY_1)== KEY_SHORT_PRESS)//����̰��м䰴��P21.5
//    {
//          send_uart_cmd_flag = 1;
//          send_uart_cmd_value = 1;
//          printf("send_uart_cmd_value:%d\n", send_uart_cmd_value);
//        if(imu660ra_ready)
//        //{
//            Start_Pid_Flag = 1;
//            Stop_Pid_Flag = 0;  // ���ֹͣ��־
//        }
//    }
//    if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
//    {
//          send_uart_cmd_flag = 1;
//          send_uart_cmd_value = 2;
//          printf("send_uart_cmd_value:%d\n", send_uart_cmd_value);
//    }
//    
//    if(key_get_state(KEY_3) == KEY_SHORT_PRESS)//����̰����
//    {
//        send_uart_cmd_flag = 1;
//        send_uart_cmd_value = 3;
//        printf("send_uart_cmd_value:%d\n", send_uart_cmd_value);
//        Start_Pid_Flag = 0;
//        Stop_Pid_Flag = 1;
//    }
}

void pit0_ch2_isr()                     // ��ʱ��ͨ�� 2 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH2);
    time_us++;
    
}

void pit0_ch10_isr()                    // ��ʱ��ͨ�� 10 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH10);
    
}

void pit0_ch11_isr()                    // ��ʱ��ͨ�� 11 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH11);
    
}

void pit0_ch12_isr()                    // ��ʱ��ͨ�� 12 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH12);
    
}

void pit0_ch13_isr()                    // ��ʱ��ͨ�� 13 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH13);
    
}

void pit0_ch14_isr()                    // ��ʱ��ͨ�� 14 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH14);
    
}

void pit0_ch15_isr()                    // ��ʱ��ͨ�� 15 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH15);
    
}

void pit0_ch16_isr()                    // ��ʱ��ͨ�� 16 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH16);
    
}

void pit0_ch17_isr()                    // ��ʱ��ͨ�� 17 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH17);
    
}

void pit0_ch18_isr()                    // ��ʱ��ͨ�� 18 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH18);
    
}

void pit0_ch19_isr()                    // ��ʱ��ͨ�� 19 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH19);
    
}

void pit0_ch20_isr()                    // ��ʱ��ͨ�� 20 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH20);
    
}

void pit0_ch21_isr()                    // ��ʱ��ͨ�� 21 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH21);
    tsl1401_collect_pit_handler();
}
// **************************** PIT�жϺ��� ****************************


// **************************** �����жϺ��� ****************************
// ����0Ĭ����Ϊ���Դ���
void uart0_isr (void)
{
    if(uart_isr_mask(UART_0))            // ����0�����ж�
    {
        
#if DEBUG_UART_USE_INTERRUPT             // ������� debug �����ж�
        debug_interrupr_handler();       // ���� debug ���ڽ��մ������� ���ݻᱻ debug ���λ�������ȡ
#endif                                   // ����޸��� DEBUG_UART_INDEX ����δ�����Ҫ�ŵ���Ӧ�Ĵ����ж�ȥ
      
    }
    else                                 // ����0�����ж�
    {           
        
        
        
    }
}

void uart1_isr (void)
{
    if(uart_isr_mask(UART_1))            // ����1�����ж�
    {
        
        //wireless_module_uart_handler();  // ����ģ��ͳһ�ص�����
      HC06_UART_RX_Handler();
      
    }
    else                                // ����1�����ж�
    {
      
        
        
    }
}

void uart2_isr (void)
{
    if(uart_isr_mask(UART_2))            // ����2�����ж�
    {
        
        gnss_uart_callback();            // GPSģ��ص�����      
        
    }
    else                                // ����2�����ж�
    {
        
        
       
    }
}

void uart3_isr (void)
{
    if(uart_isr_mask(UART_3))            // ����3�����ж�
    {
        
        
        
    }
    else                                // ����3�����ж�
    {
      
        
        
    }
}

void uart4_isr (void)
{
    if(uart_isr_mask(UART_4))            // ����4�����ж�
    {

        uart_receiver_handler();                                                                // ���ڽ��ջ��ص�����
       
    }
    else                                // ����4�����ж�
    {
      
        
        
    }
}

void uart5_isr (void)
{
    if(uart_isr_mask(UART_5))            // ����5�����ж�
    {
        
        
       
    }
    else                                // ����5�����ж�
    {
      
        
        
    }
}

void uart6_isr (void)
{
    if(uart_isr_mask(UART_6))            // ����6�����ж�
    {

        
       
    }
    else                                // ����6�����ж�
    {
      
        
        
    }
}
// **************************** �����жϺ��� ****************************

// **************************** �ⲿ�жϺ��� ****************************
void gpio_0_exti_isr()                  // �ⲿ GPIO_0 �жϷ�����     
{
    
  
  
}

void gpio_1_exti_isr()                  // �ⲿ GPIO_1 �жϷ�����     
{
    if(exti_flag_get(P01_0))		// ʾ��P1_0�˿��ⲿ�ж��ж�
    {

      
      
            
    }
    if(exti_flag_get(P01_1))
    {

            
            
    }
}

void gpio_2_exti_isr()                  // �ⲿ GPIO_2 �жϷ�����     
{
    if(exti_flag_get(P02_0))
    {
            
            
    }
    if(exti_flag_get(P02_4))
    {
            
            
    }

}

void gpio_3_exti_isr()                  // �ⲿ GPIO_3 �жϷ�����     
{



}

void gpio_4_exti_isr()                  // �ⲿ GPIO_4 �жϷ�����     
{



}

void gpio_5_exti_isr()                  // �ⲿ GPIO_5 �жϷ�����     
{



}


void gpio_6_exti_isr()                  // �ⲿ GPIO_6 �жϷ�����     
{



}

void gpio_7_exti_isr()                  // �ⲿ GPIO_7 �жϷ�����     
{



}

void gpio_8_exti_isr()                  // �ⲿ GPIO_8 �жϷ�����     
{



}

void gpio_9_exti_isr()                  // �ⲿ GPIO_9 �жϷ�����     
{



}

void gpio_10_exti_isr()                  // �ⲿ GPIO_10 �жϷ�����     
{



}

void gpio_11_exti_isr()                  // �ⲿ GPIO_11 �жϷ�����     
{



}

void gpio_12_exti_isr()                  // �ⲿ GPIO_12 �жϷ�����     
{



}

void gpio_13_exti_isr()                  // �ⲿ GPIO_13 �жϷ�����     
{



}

void gpio_14_exti_isr()                  // �ⲿ GPIO_14 �жϷ�����     
{



}

void gpio_15_exti_isr()                  // �ⲿ GPIO_15 �жϷ�����     
{



}

void gpio_16_exti_isr()                  // �ⲿ GPIO_16 �жϷ�����     
{



}

void gpio_17_exti_isr()                  // �ⲿ GPIO_17 �жϷ�����     
{



}

void gpio_18_exti_isr()                  // �ⲿ GPIO_18 �жϷ�����     
{



}

void gpio_19_exti_isr()                  // �ⲿ GPIO_19 �жϷ�����     
{



}

void gpio_20_exti_isr()                  // �ⲿ GPIO_20 �жϷ�����     
{



}

void gpio_21_exti_isr()                  // �ⲿ GPIO_21 �жϷ�����     
{



}

void gpio_22_exti_isr()                  // �ⲿ GPIO_22 �жϷ�����     
{



}

void gpio_23_exti_isr()                  // �ⲿ GPIO_23 �жϷ�����     
{



}
// **************************** �ⲿ�жϺ��� ****************************
