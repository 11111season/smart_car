#include "My_imu660ra.h"

// �����˲�ϵ�������ݾ��������0.95��ʾ����������95%�����ٶȼ�5%��
#define FILTER_COEFF 0.95f
//�ⲿʱ�����
extern volatile uint64_t time_us;
//���ٶ�������
static float imu660_ax,imu660_ay,imu660_az;
//���ٶ�������
float imu660_gx,imu660_gy,imu660_gz;
// ��������ƫ����/s��
static float gx_bias = 0.0f;
static float gy_bias = 0.0f;
static float gz_bias = 0.0f;
// �Ƕȣ��ȣ�
static float imu660_yaw = 0.0f;
static float imu660_pitch = 0.0f;
static float imu660_roll = 0.0f;
// ȫ�ֱ���
static float gyro_z_lpf = 0.0f;        // �˲����Z����ٶ�
static float gyro_z_offset = 0.0f;     // ��ƫ������������/s��
static uint16_t still_cnt = 0;
// �ϴθ���ʱ�䣨΢�룩
static uint64_t last_time = 0;
volatile uint8_t imu660ra_ready = 0;

/*****************************************************************************
 * @name       : My_IMU660RA_Calibrate
 * @date       : 2026-03-10
 * @function   : imu660raУ׼����
 * @parameters : ��
 * @retvalue   : ��
 * @note       : ��
******************************************************************************/
void My_IMU660RA_Calibrate(void)
{
    int i;
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    const int samples = 100;

    for (i = 0; i < samples; i++)
    {
        // ��ȡ������ԭʼ���ݲ�ת��Ϊ������
        imu660ra_get_gyro();
        sum_gx += imu660ra_gyro_transition(imu660ra_gyro_x);
        sum_gy += imu660ra_gyro_transition(imu660ra_gyro_y);
        sum_gz += imu660ra_gyro_transition(imu660ra_gyro_z);
        system_delay_ms(5);
    }

    gx_bias = sum_gx / samples;
    gy_bias = sum_gy / samples;
    gz_bias = sum_gz / samples;
}

/*****************************************************************************
 * @name       : My_Imu660ra_Init
 * @date       : 2026-03-10
 * @function   : �����Լ���imu660ra��ʼ������
 * @parameters : ��
 * @retvalue   : ����ֵΪ0��1��1��ʼ��ʧ�ܣ�0��ʼ���ɹ�
 * @note       : ��
******************************************************************************/
uint8_t My_Imu660ra_Init(void)
{
  uint8_t ret = imu660ra_init();
  if(ret != 0)
  {
        printf("IMU660RA init failed!\n");
        return ret;
  }
    system_delay_ms(500);
    My_IMU660RA_Calibrate();   // �������� delay������ʼ���׶���������
    last_time = time_us;
    // ���ýǶ�
    imu660_yaw = 0.0f;
    imu660_pitch = 0.0f;
    imu660_roll = 0.0f;

    imu660ra_ready = 1;        // У׼��ɣ���������
    printf("IMU660RA ready\n");
    return 0;
}

/*****************************************************************************
 * @name       : My_Imu660ra_Update
 * @date       : 2026-03-10
 * @function   : imu660ra�������ݺ���
 * @parameters : ��
 * @retvalue   : ��
 * @note       : ��
******************************************************************************/
void My_Imu660ra_Update(void)
{
    uint64_t now = time_us;
    float dt = (now - last_time) * 1e-6f;    // ΢��ת��
    if (dt <= 0.0f || dt > 0.1f) dt = 0.01f; // ���������100ms

    // �ر��жϣ����� I2C ʱ������ I2C ���ж����У�
    interrupt_global_disable();

    // ��ȡ����ԭʼ����
    imu660ra_get_acc();
    imu660ra_get_gyro();

    interrupt_global_enable(0);               // ���¿����ж�

    // ת��Ϊ��������������Ϊ���ǵ�imu660ra�İ�װ�����Ǻ�֮ǰ�����ǵ����෴�ģ��������ǽ���ȡ��
    imu660_ax = imu660ra_acc_transition(imu660ra_acc_x);
    imu660_ay = imu660ra_acc_transition(imu660ra_acc_y);
    imu660_az = -imu660ra_acc_transition(imu660ra_acc_z);

    // �����Ǽ�ȥ��ƫ
    imu660_gx = imu660ra_gyro_transition(imu660ra_gyro_x) - gx_bias;
    imu660_gy = imu660ra_gyro_transition(imu660ra_gyro_y) - gy_bias;
    imu660_gz = imu660ra_gyro_transition(imu660ra_gyro_z) - gz_bias;

    // ��ͨ�˲���ϵ��0.1���ɸ��ݲ������ڵ�����
    gyro_z_lpf = 0.9f * gyro_z_lpf + 0.1f * imu660_gz;

    // ��ֹ��⣺�����˲�ֵ��С��0.5��/s �ҳ���50�����ڣ�����10ms���ڣ���500ms��
    const float threshold = 0.5f;
    if (fabsf(gyro_z_lpf) < threshold && fabsf(imu660_gy) < threshold && fabsf(imu660_gx) < threshold)
    {
        if (still_cnt < 50) still_cnt++;
        if (still_cnt >= 50)
        {
            // ��ֹ�㹻�ã�������ƫ��ѧϰ��0.01��
            gyro_z_offset += (gyro_z_lpf - gyro_z_offset) * 0.01f;
        }
    }
    else
    {
        still_cnt = 0;
    }

    // ����ƫ�������ٶ�
    float gz_corrected = imu660_gz - gyro_z_offset;
//    // --- ��ֹ����붯̬Ư�Ʋ���
//    static float drift_rate = 0.0f;          // ���Ƶ�Ư���� (��/s)
//    static uint16_t stationary_count = 0;
//    const float movement_threshold = 0.1f;    // �˶���ֵ
//    const uint16_t stationary_samples = 100;  // ��ֹ�ж���������10ms������Լ1�룩
//    const float drift_learning_rate = 0.005f; // ѧϰ����
//
//    if (fabsf(imu660_gx) < movement_threshold && fabsf(imu660_gy) < movement_threshold && fabsf(imu660_gz) < movement_threshold)
//    {
//        stationary_count++;
//        if (stationary_count >= stationary_samples)
//        {
//            // ��ֹ�㹻��ʱ�䣬�õ�ǰ gz ����Ư���ʣ���Ϊ��ʵ�˶�Ϊ0��
//            drift_rate += (imu660_gz - drift_rate) * drift_learning_rate;
//            if (drift_rate > 2.0f) drift_rate = 2.0f;
//            if (drift_rate < -2.0f) drift_rate = -2.0f;
//        }
//    }
//    else
//    {
//        stationary_count = 0;
//    }
//
//    // ��Ư�����������ٶȣ���ѡ����У׼�㹻��Ҳ���Բ�������
//    float imu_gz_corrected = imu660_gz - drift_rate;

    // --- �����ǻ��ֵõ��Ƕ����� ---
    float imu660_yaw_g   = imu660_yaw   + gz_corrected * dt;
    float imu660_pitch_g = imu660_pitch + imu660_gx * dt;          // ���Ÿ���ʵ���ᶨ�����
    float imu660_roll_g  = imu660_roll  - imu660_gy * dt;          // ƽ�⳵���÷���

    // --- �ɼ��ٶȼ��㸩���ͺ���ǣ�����ģ����ƾ�ֹ�������˶�ʱ���ţ�---
    float imu660_pitch_a = imu660_pitch_g;  // Ĭ�ϱ���ԭֵ
    float imu660_roll_a  = imu660_roll_g;
    float norm = sqrtf(imu660_ax*imu660_ax + imu660_ay*imu660_ay + imu660_az*imu660_az);
    if (norm > 0.8f && norm < 1.2f)
    {   // �����жϼ��ٶȷ�ֵ�ӽ�1g����Ϊ�˶�������
        // ���ݼ��ٶȼƼ���Ƕȣ����������MPU6050��ͬ�Ĺ�ʽ
        // ע�⣺IMU660RA�����������MPU6050��ͬ����������෴���������
        imu660_pitch_a = atan2f(-imu660_ax, sqrtf(imu660_ay*imu660_ay + imu660_az*imu660_az)) * 57.29578f;  // ����ת��
        imu660_roll_a  = atan2f(imu660_ay, imu660_az) * 57.29578f;
    }

    // --- �����˲��ں� ---
    imu660_pitch = FILTER_COEFF * imu660_pitch_g + (1 - FILTER_COEFF) * imu660_pitch_a;
    imu660_roll  = FILTER_COEFF * imu660_roll_g  + (1 - FILTER_COEFF) * imu660_roll_a;
    imu660_yaw   = imu660_yaw_g;   // ƫ�����޷��ü��ٶ�������ֱ�ӻ���

    // ���Ƕ������� [-180, 180]
    if (imu660_yaw > 180.0f)   imu660_yaw -= 360.0f;
    if (imu660_yaw < -180.0f)  imu660_yaw += 360.0f;
    
    //imu660_yaw = -imu660_yaw;

    last_time = now;
}

void My_Imu660ra_ResetYaw(void)   {imu660_yaw = 0.0f;}
float My_Imu660ra_GetAx(void)   { return imu660_ax; }
float My_Imu660ra_GetAy(void)   { return imu660_ay; }
float My_Imu660ra_GetAz(void)   { return imu660_az; }
float My_Imu660ra_GetGx(void)   { return imu660_gx; }
float My_Imu660ra_GetGy(void)   { return imu660_gy; }
float My_Imu660ra_GetGz(void)   { return imu660_gz; }
float My_Imu660ra_GetYaw(void)  { return imu660_yaw; }
float My_Imu660ra_GetPitch(void) { return imu660_pitch; }
float My_Imu660ra_GetRoll(void)  { return imu660_roll; }
