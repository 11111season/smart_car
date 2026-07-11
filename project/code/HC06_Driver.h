#ifndef CODE_HC06_DRIVER_H_
#define CODE_HC06_DRIVER_H_

#include "zf_common_headfile.h"
#include "motor.h" // ��Ҫ�õ� target_vx, target_vy
#include "pid.h"
#include "My_imu660ra.h"
#include "inertial_nav.h"
/*----------------------- �����ӿ����� --------------------------*/
// ��ʼ������ģ�飨������ɿ��debug_init���ɣ������������ĺ����ã�
void HC06_Init(uint32 baudrate);
float Get_ManualOmega(void);
// �������ݴ�����������Ҫ����ѭ�������ڵ���
void HC06_Task(void);
void HC06_UART_RX_Handler(void);
// ��ȡ������Ч��ָ�����еĻ���
char HC06_GetCmd(void);
void Bluetooth_Command_Handler(char cmd);


// ����������֡���ݣ�# ... $��������λ�����
void ParseFrameData(const char *data, uint16_t len);
// ��������ȡ��ǰλ������ѡ�����ڵ��ԣ�
float GetPositionErrorX(void);
float GetPositionErrorY(void);
void ResetPositionError(void);
// �������˻����#1$ �𽬣�#2$ ����PID��#3$ ��ͣ��
void HC06_SendDroneCmd(uint8_t cmd);

/*----------------------- �ⲿ�������� -----------------------*/
extern volatile uint64_t time_us;
extern volatile uint8_t mpu6050_read;
extern volatile uint32_t uart_rx_irq_cnt;

#endif /* CODE_HC06_DRIVER_H_ */