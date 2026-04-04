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

/*
测量
acc_z ≈ +9.8
world_data.az ≈ +9.8

如果
world_data.az -= 9.8f;   // 如果Z轴朝上
world_data.az += 9.8f;   // 如果Z轴朝下（NED系）
*/
#ifndef M_PI
#define M_PI 3.1415926f
#endif

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


/*
高度：tof+imu积分高度
高度速度 = IMU积分 + 高度差分
位置：光流和速度积分
水平速度：vx = 光流为主 + IMU辅助
vy = 光流为主 + IMU辅助
*/






// 从 imu660rc_quarternion 构建旋转矩阵
// 输入: quat[4] = {qx, qy, qz, qw}
// 输出: R[3][3] 旋转矩阵
//imu660rc_quarternion
//IMU → 四元数 → 旋转矩阵 → 世界坐标
void quarternion_to_rotation_matrix(float quat[4])
{
    float qx = quat[0];
    float qy = quat[1];
    float qz = quat[2];
    float qw = quat[3];
    
    // 计算平方值
    float qx2 = qx * qx;
    float qy2 = qy * qy;
    float qz2 = qz * qz;
    float qw2 = qw * qw;
    
    // 计算乘积
    float qx_qy = qx * qy;
    float qx_qz = qx * qz;
    float qx_qw = qx * qw;
    float qy_qz = qy * qz;
    float qy_qw = qy * qw;
    float qz_qw = qz * qw;
    
    // 旋转矩阵元素 (标准公式，适用于 qx,qy,qz,qw 顺序)
    float R11 = 1.0f - 2.0f * (qy2 + qz2);
    float R12 = 2.0f * (qx_qy - qz_qw);
    float R13 = 2.0f * (qx_qz + qy_qw);
    
    float R21 = 2.0f * (qx_qy + qz_qw);
    float R22 = 1.0f - 2.0f * (qx2 + qz2);
    float R23 = 2.0f * (qy_qz - qx_qw);
    
    float R31 = 2.0f * (qx_qz - qy_qw);
    float R32 = 2.0f * (qy_qz + qx_qw);
    float R33 = 1.0f - 2.0f * (qx2 + qy2);
    
    // 加速度转化
    world_data.ax = R11*imu_data.acc_x + R12*imu_data.acc_y + R13*imu_data.acc_z;
    world_data.ay = R21*imu_data.acc_x + R22*imu_data.acc_y + R23*imu_data.acc_z;
    world_data.az = R31*imu_data.acc_x + R32*imu_data.acc_y + R33*imu_data.acc_z;

    // 去重力
    world_data.az -= 9.8f;
//    if(world_data.az > 0)
//    {
//        world_data.az -= 9.8f;
//    }
//    else
//    {
//        world_data.az += 9.8f;
//    }
}





////IMU → 四元数 → 欧拉角（角度） → sin/cos → 世界坐标
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
//
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
//}
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