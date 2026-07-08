#include "My_imu660ra.h"

// 互补滤波系数（根据经验调整，0.95表示信任陀螺仪95%，加速度计5%）
#define FILTER_COEFF 0.95f
//外部时间变量
extern volatile uint64_t time_us;
//加速度物理量
static float imu660_ax,imu660_ay,imu660_az;
//角速度物理量
float imu660_gx,imu660_gy,imu660_gz;
// 陀螺仪零偏（°/s）
static float gx_bias = 0.0f;
static float gy_bias = 0.0f;
static float gz_bias = 0.0f;
// 角度（度）
static float imu660_yaw = 0.0f;
static float imu660_pitch = 0.0f;
static float imu660_roll = 0.0f;
// 全局变量
static float gyro_z_lpf = 0.0f;        // 滤波后的Z轴角速度
static float gyro_z_offset = 0.0f;     // 零偏（物理量，°/s）
static uint16_t still_cnt = 0;
// 上次更新时间（微秒）
static uint64_t last_time = 0;
volatile uint8_t imu660ra_ready = 0;

/*****************************************************************************
 * @name       : My_IMU660RA_Calibrate
 * @date       : 2026-03-10
 * @function   : imu660ra校准函数
 * @parameters : 无
 * @retvalue   : 无
 * @note       : 无
******************************************************************************/
void My_IMU660RA_Calibrate(void)
{
    int i;
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    const int samples = 100;

    for (i = 0; i < samples; i++)
    {
        // 获取陀螺仪原始数据并转换为物理量
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
 * @function   : 我们自己的imu660ra初始化函数
 * @parameters : 无
 * @retvalue   : 返回值为0和1，1初始化失败，0初始化成功
 * @note       : 无
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
    My_IMU660RA_Calibrate();   // 这里用了 delay，但初始化阶段是允许的
    last_time = time_us;
    // 重置角度
    imu660_yaw = 0.0f;
    imu660_pitch = 0.0f;
    imu660_roll = 0.0f;

    imu660ra_ready = 1;        // 校准完成，允许启动
    printf("IMU660RA ready\n");
    return 0;
}

/*****************************************************************************
 * @name       : My_Imu660ra_Update
 * @date       : 2026-03-10
 * @function   : imu660ra更新数据函数
 * @parameters : 无
 * @retvalue   : 无
 * @note       : 无
******************************************************************************/
void My_Imu660ra_Update(void)
{
    uint64_t now = time_us;
    float dt = (now - last_time) * 1e-6f;    // 微秒转秒
    if (dt <= 0.0f || dt > 0.1f) dt = 0.01f; // 防错，最大100ms

    // 关闭中断，保护 I2C 时序（软件 I2C 对中断敏感）
    interrupt_global_disable();

    // 读取最新原始数据
    imu660ra_get_acc();
    imu660ra_get_gyro();

    interrupt_global_enable(0);               // 重新开放中断

    // 转换为物理量：这里因为我们的imu660ra的安装方向是和之前陀螺仪的轴相反的，所以我们进行取反
    imu660_ax = imu660ra_acc_transition(imu660ra_acc_x);
    imu660_ay = imu660ra_acc_transition(imu660ra_acc_y);
    imu660_az = -imu660ra_acc_transition(imu660ra_acc_z);

    // 陀螺仪减去零偏
    imu660_gx = imu660ra_gyro_transition(imu660ra_gyro_x) - gx_bias;
    imu660_gy = imu660ra_gyro_transition(imu660ra_gyro_y) - gy_bias;
    imu660_gz = imu660ra_gyro_transition(imu660ra_gyro_z) - gz_bias;

    // 低通滤波（系数0.1，可根据采样周期调整）
    gyro_z_lpf = 0.9f * gyro_z_lpf + 0.1f * imu660_gz;

    // 静止检测：三轴滤波值均小于0.5°/s 且持续50个周期（假设10ms周期，即500ms）
    const float threshold = 0.5f;
    if (fabsf(gyro_z_lpf) < threshold && fabsf(imu660_gy) < threshold && fabsf(imu660_gx) < threshold)
    {
        if (still_cnt < 50) still_cnt++;
        if (still_cnt >= 50)
        {
            // 静止足够久，更新零偏（学习率0.01）
            gyro_z_offset += (gyro_z_lpf - gyro_z_offset) * 0.01f;
        }
    }
    else
    {
        still_cnt = 0;
    }

    // 用零偏修正角速度
    float gz_corrected = imu660_gz - gyro_z_offset;
//    // --- 静止检测与动态漂移补偿
//    static float drift_rate = 0.0f;          // 估计的漂移率 (°/s)
//    static uint16_t stationary_count = 0;
//    const float movement_threshold = 0.1f;    // 运动阈值
//    const uint16_t stationary_samples = 100;  // 静止判定样本数（10ms周期下约1秒）
//    const float drift_learning_rate = 0.005f; // 学习速率
//
//    if (fabsf(imu660_gx) < movement_threshold && fabsf(imu660_gy) < movement_threshold && fabsf(imu660_gz) < movement_threshold)
//    {
//        stationary_count++;
//        if (stationary_count >= stationary_samples)
//        {
//            // 静止足够长时间，用当前 gz 更新漂移率（认为真实运动为0）
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
//    // 用漂移率修正角速度（可选，若校准足够好也可以不修正）
//    float imu_gz_corrected = imu660_gz - drift_rate;

    // --- 陀螺仪积分得到角度增量 ---
    float imu660_yaw_g   = imu660_yaw   + gz_corrected * dt;
    float imu660_pitch_g = imu660_pitch + imu660_gx * dt;          // 符号根据实际轴定义调整
    float imu660_roll_g  = imu660_roll  - imu660_gy * dt;          // 平衡车常用符号

    // --- 由加速度计算俯仰和横滚角（仅当模块近似静止或匀速运动时可信）---
    float imu660_pitch_a = imu660_pitch_g;  // 默认保持原值
    float imu660_roll_a  = imu660_roll_g;
    float norm = sqrtf(imu660_ax*imu660_ax + imu660_ay*imu660_ay + imu660_az*imu660_az);
    if (norm > 0.8f && norm < 1.2f)
    {   // 粗略判断加速度幅值接近1g，认为运动不剧烈
        // 根据加速度计计算角度，这里采用与MPU6050相同的公式
        // 注意：IMU660RA的轴向可能与MPU6050相同，如果方向相反请调整符号
        imu660_pitch_a = atan2f(-imu660_ax, sqrtf(imu660_ay*imu660_ay + imu660_az*imu660_az)) * 57.29578f;  // 弧度转度
        imu660_roll_a  = atan2f(imu660_ay, imu660_az) * 57.29578f;
    }

    // --- 互补滤波融合 ---
    imu660_pitch = FILTER_COEFF * imu660_pitch_g + (1 - FILTER_COEFF) * imu660_pitch_a;
    imu660_roll  = FILTER_COEFF * imu660_roll_g  + (1 - FILTER_COEFF) * imu660_roll_a;
    imu660_yaw   = imu660_yaw_g;   // 偏航角无法用加速度修正，直接积分

    // 将角度限制在 [-180, 180]
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
