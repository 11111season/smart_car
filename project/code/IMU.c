#include "zf_common_headfile.h"




void imu660rc_get_data(void)
{
      
    imu_data.gyro_x = imu660rc_gyro_x;  
    imu_data.gyro_y = imu660rc_gyro_y;  
    imu_data.gyro_z = imu660rc_gyro_z;  
    
    imu_data.acc_x = imu660rc_acc_x;  
    imu_data.acc_y = imu660rc_acc_y;  
    imu_data.acc_z = imu660rc_acc_z;  

    eulerAngle.roll = imu660rc_roll;
    eulerAngle.pitch = imu660rc_pitch;
    eulerAngle.yaw = imu660rc_yaw;


}