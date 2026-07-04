#ifndef CODE_IMU_H_
#define CODE_IMU_H_


void imu660rc_get_data(void);
void quarternion_to_rotation_matrix(void);

void quat_to_euler(float q0 ,float q1 ,float q2 ,float q3 ,_euler_param_st* angle);

void Mahony_Mag_Update(void);        // 带磁力计融合的姿态更新
void Mahony_Update_NoMag(void);     // 无磁力计融合 (起飞阶段)
void mag_apply_calibration(float mx_raw, float my_raw, float mz_raw,
                      float *mx_cal, float *my_cal, float *mz_cal);
void mag_set_calibration(float offset[3], float scale[3][3]);

// 全局变量
extern _euler_param_st eulerAngle;
extern float rotation_matrix[3][3];

// 磁力计校准调试变量
extern float mag_raw_gauss[3];
extern float mag_calib[3];

#endif 
