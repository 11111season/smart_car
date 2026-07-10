#include "Encoder.h"

/*****************************************************************************
 * @name       : Encoder_Init
 * @date       : 2026-03-10
 * @function   : ���ڱ������ĳ�ʼ������
 * @parameters : ��
 * @retvalue   : ��
 * @note       : ��
******************************************************************************/
void Encoder_Init(void)
{
    //��ǰ�ֱ�����
    encoder_dir_init(TC_CH27_ENCODER,TC_CH27_ENCODER_CH1_P19_2,TC_CH27_ENCODER_CH2_P19_3);
    //encoder_dir_init(TC_CH20_ENCODER,TC_CH20_ENCODER_CH1_P08_1,TC_CH20_ENCODER_CH2_P08_2);
    //��ǰ�ֱ�����
    //encoder_dir_init(TC_CH07_ENCODER, TC_CH07_ENCODER_CH1_P07_6, TC_CH07_ENCODER_CH2_P07_7);
    encoder_dir_init(TC_CH58_ENCODER,TC_CH58_ENCODER_CH1_P17_3,TC_CH58_ENCODER_CH2_P17_4);
    //����ֱ�����
    encoder_dir_init(TC_CH07_ENCODER, TC_CH07_ENCODER_CH1_P07_6, TC_CH07_ENCODER_CH2_P07_7);
    //encoder_dir_init(TC_CH58_ENCODER,TC_CH58_ENCODER_CH1_P17_3,TC_CH58_ENCODER_CH2_P17_4);
    //�Һ��ֱ�����
    encoder_dir_init(TC_CH20_ENCODER,TC_CH20_ENCODER_CH1_P08_1,TC_CH20_ENCODER_CH2_P08_2);
    //encoder_dir_init(TC_CH27_ENCODER,TC_CH27_ENCODER_CH1_P19_2,TC_CH27_ENCODER_CH2_P19_3);
}

/*****************************************************************************
 * @name       : Encoder_Data_Get
 * @date       : 2026-03-10
 * @function   : ���ڱ����������ݶ�ȡ����
 * @parameters : ��
 * @retvalue   : ��
 * @note       : ��
******************************************************************************/
void Encoder_Data_Get(void)
{
    //��ǰ�ֱ��������ݻ�ȡ
    motor_L1.encoder_raw = encoder_get_count(TC_CH27_ENCODER);
    //��ǰ�ֵ�ͨ�����˲�
    motor_L1.encoder_speed = 0.5f * motor_L1.encoder_speed + 0.5f * motor_L1.encoder_raw;//��������ȡ�ٶ�
    //��ȡ�ñ�����֪�������˶�Զ�ľ���
    motor_L1.total_encoder += motor_L1.encoder_raw;
    //��ȡ��ת��֮����м������
    encoder_clear_count(TC_CH27_ENCODER);
    
    //����ֱ��������ݻ�ȡ
    motor_L2.encoder_raw = encoder_get_count(TC_CH07_ENCODER);
    //����ֵ�ͨ�����˲�
    motor_L2.encoder_speed = 0.5f * motor_L2.encoder_speed + 0.5f * motor_L2.encoder_raw;
    //��ȡ�ñ�����֪�������˶�Զ�ľ���
    motor_L2.total_encoder += motor_L2.encoder_raw;
    //��ȡ��ת��֮����м������
    encoder_clear_count(TC_CH07_ENCODER);
    
    //��ǰ�ֱ��������ݻ�ȡ
    motor_R1.encoder_raw = -encoder_get_count(TC_CH58_ENCODER);
    //��ǰ�ֵ�ͨ�����˲�
    motor_R1.encoder_speed = 0.5f * motor_R1.encoder_speed + 0.5f * motor_R1.encoder_raw;//��������ȡ�ٶ�
    //��ȡ�ñ�����֪�������˶�Զ�ľ���
    motor_R1.total_encoder += motor_R1.encoder_raw;
    //��ȡ��ת��֮����м������
    encoder_clear_count(TC_CH58_ENCODER);
    
    //�Һ��ֱ��������ݻ�ȡ
    motor_R2.encoder_raw = -encoder_get_count(TC_CH20_ENCODER);
    //�Һ��ֵ�ͨ�����˲�
    motor_R2.encoder_speed = 0.5f * motor_R2.encoder_speed + 0.5f * motor_R2.encoder_raw;
    //��ȡ�ñ�����֪�������˶�Զ�ľ���
    motor_R2.total_encoder += motor_R2.encoder_raw;
    //��ȡ��ת��֮����м������
    encoder_clear_count(TC_CH20_ENCODER);
}
