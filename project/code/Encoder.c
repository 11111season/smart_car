#include "Encoder.h"

/*****************************************************************************
 * @name       : Encoder_Init
 * @date       : 2026-03-10
 * @function   : 编码器初始化函数
 * @parameters : 无
 * @retvalue   : 无
 * @note       : 无
******************************************************************************/
void Encoder_Init(void)
{
    //左前轮编码器
    encoder_dir_init(TC_CH27_ENCODER,TC_CH27_ENCODER_CH1_P19_2,TC_CH27_ENCODER_CH2_P19_3);
    //encoder_dir_init(TC_CH20_ENCODER,TC_CH20_ENCODER_CH1_P08_1,TC_CH20_ENCODER_CH2_P08_2);
    //左前轮编码器
    //encoder_dir_init(TC_CH07_ENCODER, TC_CH07_ENCODER_CH1_P07_6, TC_CH07_ENCODER_CH2_P07_7);
    encoder_dir_init(TC_CH58_ENCODER,TC_CH58_ENCODER_CH1_P17_3,TC_CH58_ENCODER_CH2_P17_4);
    //左后轮编码器
    encoder_dir_init(TC_CH07_ENCODER, TC_CH07_ENCODER_CH1_P07_6, TC_CH07_ENCODER_CH2_P07_7);
    //encoder_dir_init(TC_CH58_ENCODER,TC_CH58_ENCODER_CH1_P17_3,TC_CH58_ENCODER_CH2_P17_4);
    //右后轮编码器
    encoder_dir_init(TC_CH20_ENCODER,TC_CH20_ENCODER_CH1_P08_1,TC_CH20_ENCODER_CH2_P08_2);
    //encoder_dir_init(TC_CH27_ENCODER,TC_CH27_ENCODER_CH1_P19_2,TC_CH27_ENCODER_CH2_P19_3);
}

/*****************************************************************************
 * @name       : Encoder_Data_Get
 * @date       : 2026-03-10
 * @function   : 编码器数据读取函数
 * @parameters : 无
 * @retvalue   : 无
 * @note       : 无
******************************************************************************/
void Encoder_Data_Get(void)
{
    //左前轮编码器数据获取
    motor_L1.encoder_raw = encoder_get_count(TC_CH27_ENCODER);
    //左前轮通道低通滤波
    motor_L1.encoder_speed = 0.5f * motor_L1.encoder_speed + 0.5f * motor_L1.encoder_raw;//滤波后获取速度
    //累加总脉冲, 用于计算运动距离
    motor_L1.total_encoder += motor_L1.encoder_raw;
    //读取计数器后清空计数
    encoder_clear_count(TC_CH27_ENCODER);

    //左后轮编码器数据获取
    motor_L2.encoder_raw = encoder_get_count(TC_CH07_ENCODER);
    //左后轮通道低通滤波
    motor_L2.encoder_speed = 0.5f * motor_L2.encoder_speed + 0.5f * motor_L2.encoder_raw;
    //累加总脉冲, 用于计算运动距离
    motor_L2.total_encoder += motor_L2.encoder_raw;
    //读取计数器后清空计数
    encoder_clear_count(TC_CH07_ENCODER);

    //右前轮编码器数据获取
    motor_R1.encoder_raw = -encoder_get_count(TC_CH58_ENCODER);
    //右前轮通道低通滤波
    motor_R1.encoder_speed = 0.5f * motor_R1.encoder_speed + 0.5f * motor_R1.encoder_raw;//滤波后获取速度
    //累加总脉冲, 用于计算运动距离
    motor_R1.total_encoder += motor_R1.encoder_raw;
    //读取计数器后清空计数
    encoder_clear_count(TC_CH58_ENCODER);

    //右后轮编码器数据获取
    motor_R2.encoder_raw = -encoder_get_count(TC_CH20_ENCODER);
    //右后轮通道低通滤波
    motor_R2.encoder_speed = 0.5f * motor_R2.encoder_speed + 0.5f * motor_R2.encoder_raw;
    //累加总脉冲, 用于计算运动距离
    motor_R2.total_encoder += motor_R2.encoder_raw;
    //读取计数器后清空计数
    encoder_clear_count(TC_CH20_ENCODER);
}
