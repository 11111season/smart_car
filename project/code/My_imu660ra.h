#ifndef CODE_MY_IMU660RA_H_
#define CODE_MY_IMU660RA_H_

#include "zf_common_headfile.h"

/*----------------------- 外部变量声明 -----------------------*/
extern volatile uint8_t imu660ra_ready;
extern float imu660_gx,imu660_gy,imu660_gz; // 三轴角速度 (单位: °/s)
extern float gyro_z_offset;               // Z轴在线零偏 (°/s), ISR可更新, 限幅±2
extern float yaw_drift_comp;              // 漂移补偿虚拟量 (ISR累加, 控制环+惯导共用)

/*----------------------- 函数接口声明 --------------------------*/
void My_IMU660RA_Calibrate(void);
uint8_t My_Imu660ra_Init(void);
void My_Imu660ra_Update(void);
// 简化的获取函数
float My_Imu660ra_GetYaw(void);
float My_Imu660ra_GetYawComp(void);   // 漂移补偿后的航向 (GetYaw - yaw_drift_comp)
float My_Imu660ra_GetPitch(void);
float My_Imu660ra_GetRoll(void);
float My_Imu660ra_GetAx(void);
float My_Imu660ra_GetAy(void);
float My_Imu660ra_GetAz(void);
float My_Imu660ra_GetGx(void);
float My_Imu660ra_GetGy(void);
float My_Imu660ra_GetGz(void);
void My_Imu660ra_ResetYaw(void);

#endif /* CODE_MY_IMU660RA_H_ */