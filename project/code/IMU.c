#include "zf_common_headfile.h"

/*
标准世界坐标系转换
  Roll，绕 X轴旋转，右倾为正（右边机翼下）
  Pitch，绕 Y轴旋转，抬头为正
  Yaw，绕 Z轴旋转逆时针为正（从上往下看）

ax_w =  cp*cy * ax + (sr*sp*cy - cr*sy) * ay + (cr*sp*cy + sr*sy) * az;

ay_w =  cp*sy * ax + (sr*sp*sy + cr*cy) * ay + (cr*sp*sy - sr*cy) * az;
      
az_w = -sp * ax + sr*cp * ay + cr*cp * az;
      抬头投影-x，右倾投影y，Z轴本身就是主要重力方向乘 cos 是“姿态修正”

*/


void imu660rc_get_data(void)
{
    //单位度每秒
    imu_data.gyro_x = imu660rc_gyro_x/14.2857;  
    imu_data.gyro_y = imu660rc_gyro_y/14.2857;  
    imu_data.gyro_z = imu660rc_gyro_z/14.2857;  
    
    //单位m/s
    imu_data.acc_x = imu660rc_acc_x/4098.36 * 9.8f;  
    imu_data.acc_y = imu660rc_acc_y/4098.36 * 9.8f;  
    imu_data.acc_z = imu660rc_acc_z/4098.36 * 9.8f;  

    eulerAngle.roll = imu660rc_roll;
    eulerAngle.pitch = imu660rc_pitch;
    eulerAngle.yaw = imu660rc_yaw;


}


//void height_data_deal(void)
//{
//    float last_height;
//    //获取高度数据
//    float height= dl1b_distance_mm/1000.0f;
//    //滤波
//      
//    //激光高度差分速度加滤波
//    float vz_laser = (height - last_height) / dt;
//    
//    //---------加速度转换世界坐标系-----------
//    //单精度sinf，且只算一次
//    float cr = cosf(eulerAngle.roll);//标准抬头>0
//    float sr = sinf(eulerAngle.roll);
//    float cp = cosf(eulerAngle.pitch);
//    float sp = sinf(eulerAngle.pitch);
//    float cy = cosf(eulerAngle.yaw);
//    float sy = -sinf(eulerAngle.yaw);
//    
//    
//
//
//    world_data.ax= (cp*cy)*imu_data.acc_x + (sr*sp*cy - cr*sy)*imu_data.acc_y + (cr*sp*cy + sr*sy)*imu_data.acc_z;
//
//    world_data.ay = (cp*sy)*imu_data.acc_x + (sr*sp*sy + cr*cy)*imu_data.acc_y + (cr*sp*sy - sr*cy)*imu_data.acc_z;
//
//    world_data.az = (-sp)*imu_data.acc_x + (sr*cp)*imu_data.acc_y + (cr*cp)*imu_data.acc_z;
//    //减去重力（？只在Z轴转换吗）
//
//    //梯形积分得到速度
//  
//   //积分得到位置
//
////----------融合速度，互补滤波，权重参数
//    //融合速度：加速度积分，高度差分
//    //融合高度：测量高度和速度积分
//    
//
//
//
//
//
//
//}
//
//
//
//
//
//
//
//
//// 推荐做法：总是转换到世界坐标系
//typedef struct {
//    float vx, vy, vz;  // 世界坐标系速度
//    float px, py, pz;  // 世界坐标系位置
//} WorldState_t;
//
//WorldState_t UpdateWorldState(float ax, float ay, float az,
//                               float pitch, float roll, float yaw,
//                               float dt)
//{
//    static WorldState_t state = {0};
//    static float last_ax_w = 0, last_ay_w = 0, last_az_w = 0;
//    
//    // 1. 转换到世界坐标系
//    float ax_w, ay_w, az_w;
//    BodyToWorld(ax, ay, az, pitch, roll, yaw, &ax_w, &ay_w, &az_w);
//    
//    // 2. 减去重力（只在Z轴）
//    az_w -= 9.81f;
//    
//    // 3. 梯形积分得到速度
//    state.vx += (ax_w + last_ax_w) / 2.0f * dt;
//    state.vy += (ay_w + last_ay_w) / 2.0f * dt;
//    state.vz += (az_w + last_az_w) / 2.0f * dt;
//    
//    // 4. 积分得到位置
//    state.px += state.vx * dt;
//    state.py += state.vy * dt;
//    state.pz += state.vz * dt;
//    
//    // 5. 保存值
//    last_ax_w = ax_w;
//    last_ay_w = ay_w;
//    last_az_w = az_w;
//    
//    return state;
//}
//
//
//
//
//void Height_Fusion_Update(float tof_height,   // 激光高度（m）
//                          float acc_x,
//                          float acc_y,
//                          float acc_z,
//                          float roll,
//                          float pitch,
//                          float dt)
//{
//    // ===== 1. 激光高度 =====
//    float z_laser = tof_height;
//
//    // ===== 2. 激光差分速度 =====
//    float vz_laser = (z_laser - last_z) / dt;
//
//    // 低通滤波（防抖）
//    vz_laser = 0.7f * vz_laser_last + 0.3f * vz_laser;
//    vz_laser_last = vz_laser;
//
//    // ===== 3. 加速度 → 世界坐标 =====
//    float az_world = acc_x * sinf(pitch)
//                   - acc_y * sinf(roll) * cosf(pitch)
//                   + acc_z * cosf(roll) * cosf(pitch);
//
//    // 去重力
//    az_world -= 9.8f;
//
//    // ===== 4. 融合速度 =====
//    vz = 0.98f * (vz + az_world * dt) + 0.02f * vz_laser;
//
//    // ===== 5. 融合高度 =====
//    z = 0.98f * (z + vz * dt) + 0.02f * z_laser;
//
//    // ===== 6. 更新 =====
//    last_z = z_laser;
//}