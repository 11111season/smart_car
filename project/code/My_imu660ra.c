#include "My_imu660ra.h"

// 互补滤波系数: 数据经过0.95加权, 表示陀螺仪占95%, 加速度计占5%
#define FILTER_COEFF 0.95f
// 外部时间基准
extern volatile uint64_t time_us;
// 加速度原始量
static float imu660_ax,imu660_ay,imu660_az;
// 角速度原始量
float imu660_gx,imu660_gy,imu660_gz;
// 三轴陀螺零偏 (°/s)
static float gx_bias = 0.0f;
static float gy_bias = 0.0f;
static float gz_bias = 0.0f;
float gyro_z_offset = 0.0f;     // Z轴在线零偏 (非static, ISR可读写), 限幅±2°/s
float yaw_drift_comp = 0.0f;    // 漂移补偿虚拟量 (非static, ISR累加, 控制环+惯导共用读取)
// 姿态角 (度)
static float imu660_yaw = 0.0f;
static float imu660_pitch = 0.0f;
static float imu660_roll = 0.0f;
// 全局变量
static float gyro_z_lpf = 0.0f;        // 滤波后的Z轴角速度
static uint16_t still_cnt = 0;
// 上次更新时间 (微秒)
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
        // 读取陀螺原始数据并转换为角速度
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
 * @function   : 封装imu660ra初始化函数
 * @parameters : 无
 * @retvalue   : 返回值为0或1, 1初始化失败, 0初始化成功
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
    My_IMU660RA_Calibrate();   // 若开机会 delay, 初始化阶段无需重复校准
    last_time = time_us;
    // 设置角度
    imu660_yaw = 0.0f;
    imu660_pitch = 0.0f;
    imu660_roll = 0.0f;

    imu660ra_ready = 1;        // 校准完成, 初始化成功
    printf("IMU660RA ready\n");
    return 0;
}

/*****************************************************************************
 * @name       : My_Imu660ra_Update
 * @date       : 2026-03-10
 * @function   : imu660ra数据更新函数
 * @parameters : 无
 * @retvalue   : 无
 * @note       : 无
******************************************************************************/
void My_Imu660ra_Update(void)
{
    uint64_t now = time_us;
    float dt = (now - last_time) * 1e-6f;    // 微秒转秒
    if (dt <= 0.0f || dt > 0.1f) dt = 0.01f; // 限制采样间隔不超过100ms

    // 关闭中断, 防止 I2C 时序被打断导致 I2C 数据出错
    interrupt_global_disable();

    // 读取陀螺原始数据
    imu660ra_get_acc();
    imu660ra_get_gyro();

    interrupt_global_enable(0);               // 重新开启中断

    // 转换为加速度值, 因为我们的imu660ra的安装方式是和之前正相反, 所以这里我们取负值
    imu660_ax = imu660ra_acc_transition(imu660ra_acc_x);
    imu660_ay = imu660ra_acc_transition(imu660ra_acc_y);
    imu660_az = -imu660ra_acc_transition(imu660ra_acc_z);

    // 角速度减去零偏
    imu660_gx = imu660ra_gyro_transition(imu660ra_gyro_x) - gx_bias;
    imu660_gy = imu660ra_gyro_transition(imu660ra_gyro_y) - gy_bias;
    imu660_gz = imu660ra_gyro_transition(imu660ra_gyro_z) - gz_bias;

    // 低通滤波系数0.1, 可根据采样周期进行调整
    gyro_z_lpf = 0.9f * gyro_z_lpf + 0.1f * imu660_gz;

    // 静止检测: 三轴角速度均 < 0.2°/s 且持续 15 周期 (0.15s), 学习陀螺零偏
    const float threshold = 0.2f;
    if (fabsf(gyro_z_lpf) < threshold && fabsf(imu660_gy) < threshold && fabsf(imu660_gx) < threshold)
    {
        if (still_cnt < 15) still_cnt++;
        if (still_cnt >= 15)
        {
            // 静止足够久, 当前滤波读数即为残余零偏, 激进学习 (12%/周期)
            gyro_z_offset += (gyro_z_lpf - gyro_z_offset) * 0.12f;
            if (gyro_z_offset >  2.0f) gyro_z_offset =  2.0f;
            if (gyro_z_offset < -2.0f) gyro_z_offset = -2.0f;
        }
    }
    else
    {
        still_cnt = 0;
    }

    // 扣除在线零偏后的角速度
    float gz_corrected = imu660_gz - gyro_z_offset;

    // --- 陀螺仪积分得到的角度值 ---
    float imu660_yaw_g   = imu660_yaw   + gz_corrected * dt;
    float imu660_pitch_g = imu660_pitch + imu660_gx * dt;          // 俯仰根据实际轴定义调整
    float imu660_roll_g  = imu660_roll  - imu660_gy * dt;          // 平衡车方向使用方式

    // --- 由加速度计算俯仰和横滚角, 用于修正静止状态或缓速运动时的漂移 ---
    float imu660_pitch_a = imu660_pitch_g;  // 默认保持原值
    float imu660_roll_a  = imu660_roll_g;
    float norm = sqrtf(imu660_ax*imu660_ax + imu660_ay*imu660_ay + imu660_az*imu660_az);
    if (norm > 0.8f && norm < 1.2f)
    {   // 通过判断加速度幅值接近1g, 认为是缓速运动状态
        // 根据加速度计计算角度, 使用与MPU6050相同的公式
        // 注意: IMU660RA加速度轴与MPU6050不同, 需要取反修正
        imu660_pitch_a = atan2f(-imu660_ax, sqrtf(imu660_ay*imu660_ay + imu660_az*imu660_az)) * 57.29578f;  // 角度转换
        imu660_roll_a  = atan2f(imu660_ay, imu660_az) * 57.29578f;
    }

    // --- 互补滤波融合 ---
    imu660_pitch = FILTER_COEFF * imu660_pitch_g + (1 - FILTER_COEFF) * imu660_pitch_a;
    imu660_roll  = FILTER_COEFF * imu660_roll_g  + (1 - FILTER_COEFF) * imu660_roll_a;
    imu660_yaw   = imu660_yaw_g;   // 偏航角无法用加速度计修正, 直接积分

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

// 漂移补偿后的航向: 控制环与惯导共用 (惯导地图坐标系随此实时修正, 不再相对车头旋转)
float My_Imu660ra_GetYawComp(void) { return imu660_yaw - yaw_drift_comp; }
float My_Imu660ra_GetPitch(void) { return imu660_pitch; }
float My_Imu660ra_GetRoll(void)  { return imu660_roll; }
