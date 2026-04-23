#ifndef CODE_IMU_H_
#define CODE_IMU_H_


void imu660rc_get_data(void);
void quarternion_to_rotation_matrix(float quat[4]);

void quat_to_euler(float q[4], _euler_param_st * angle);

void Mahony_Mag_Update() ;
void mag_apply_calibration(float mx_raw, float my_raw, float mz_raw,
                      float *mx_cal, float *my_cal, float *mz_cal);
void mag_set_calibration(float offset[3], float scale[3][3]);



#endif 
