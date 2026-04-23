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

//    eulerAngle.roll = imu660rc_roll;
//    eulerAngle.pitch = imu660rc_pitch;
//    eulerAngle.yaw = imu660rc_yaw;


}



/*
高度：tof+imu积分高度
高度速度 = IMU积分 + 高度差分
位置：光流和速度积分
水平速度：vx = 光流为主 + IMU辅助
vy = 光流为主 + IMU辅助
*/

static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;



// 快速平方根倒数
static float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/**
 * @brief 磁力计融合四元数更新（Mahony算法，无校准简化版）
 * @param gx, gy, gz  陀螺仪角速度 (rad/s) —— 对应 imu_data.gyro_x, _y, _z
 * @param mx, my, mz  磁力计原始值（直接传原始高斯值或任意单位）
 * @param dt          采样周期 (秒)
 * @param q            四元数数组，按你的顺序：q[0]=qx, q[1]=qy, q[2]=qw(实部), q[3]=qz
 */
void Mahony_Mag_Update() {
  
    float gx = imu_data.gyro_x * (M_PI / 180.0f);
    float gy = imu_data.gyro_y * (M_PI / 180.0f); 
    float gz = imu_data.gyro_z * (M_PI / 180.0f);
                       
//    float mx = PT1Filter_Apply(&filter_qcm5883l,qmc5883l_mag_x_gauss); 
//    float my = PT1Filter_Apply(&filter_qcm5883l,qmc5883l_mag_y_gauss);
//    float mz = PT1Filter_Apply(&filter_qcm5883l,qmc5883l_mag_z_gauss);
    float mx = qmc5883l_mag_x_gauss;
    float my = qmc5883l_mag_y_gauss;
    float mz = qmc5883l_mag_z_gauss;

    
    mag_apply_calibration(mx, my, mz, &mx, &my, &mz);
    
    float dt = 0.005; 
    float q[4];
      
      
      
    float recipNorm;
    float hx, hy, hz, bx, bz;
    float halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;

    // 提取四元数分量（按你的顺序）
    float q0 = imu660rc_quarternion[0]; // qx
    float q1 = imu660rc_quarternion[1]; // qy
    float q2 = imu660rc_quarternion[2]; // qw (实部)
    float q3 = imu660rc_quarternion[3]; // qz

    float kp= 1.0;
    float ki =0.001;
    // ----- 1. 归一化磁力计-----
    if (!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))) {
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;
    }

    // ----- 2. 使用当前四元数将机体磁场旋转到世界坐标系 -----
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    // 旋转矩阵元素（仅用于磁场变换）
    hx = mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3)       + mz * (q1q3 + q0q2);
    hy = mx * (q1q2 + q0q3)       + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1);
    hz = mx * (q1q3 - q0q2)       + my * (q2q3 + q0q1)       + mz * (0.5f - q1q1 - q2q2);

    // 计算参考地磁方向（假设地磁北在水平面上，东向分量为0）
    bx = sqrtf(hx * hx + hy * hy);
    bz = hz;

    // ----- 3. 将参考磁场旋转回机体坐标系，得到理论测量值 -----
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3)       + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3)       + bz * (0.5f - q1q1 - q2q2);

    // ----- 4. 计算磁力计误差（叉乘）-----
    halfex = (my * halfwz - mz * halfwy);
    halfey = (mz * halfwx - mx * halfwz);
    halfez = (mx * halfwy - my * halfwx);

    // ----- 5. PI控制器修正陀螺仪 -----
    if (ki > 0.0f) {
        integralFBx += halfex * ki * dt;
        integralFBy += halfey * ki * dt;
        integralFBz += halfez * ki * dt;
    }
    gx += kp * halfex + integralFBx;
    gy += kp * halfey + integralFBy;
    gz += kp * halfez + integralFBz;
//    printf("%3f,%3f,%3f\r\n",integralFBx ,integralFBy,integralFBz);
    // ----- 6. 用修正后的角速度更新四元数（一阶积分）-----
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    float qa = q2; // 实部
    float qb = q0;
    float qc = q1;
    float qd = q3;
    q2 += (-qb * gx - qc * gy - qd * gz);
    q0 += ( qa * gx + qc * gz - qd * gy);
    q1 += ( qa * gy - qb * gz + qd * gx);
    q3 += ( qa * gz + qb * gy - qc * gx);

    // ----- 7. 四元数归一化 -----
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    // 写回数组（保持原顺序）
    imu660rc_quarternion[0] = q0;
    imu660rc_quarternion[1] = q1;
    imu660rc_quarternion[2] = q2; // 实部
    imu660rc_quarternion[3] = q3;
    float norm = sqrtf(mx*mx + my*my + mz*mz);
//    printf("Mag norm: %.3f\r\n", norm);
    printf("%6f,%6f,%6f\r\n", halfwx,halfwy,halfwz);  
    quat_to_euler(imu660rc_quarternion, &eulerAngle);
}

/**
 * @brief 将你的四元数转换为欧拉角（角度）
 * @param q    四元数数组（你的顺序）
 * @param roll, pitch, yaw  输出欧拉角 (角度)
 */
void quat_to_euler(float q[4],_euler_param_st* angle) {
    float q0 = q[0]; // qx
    float q1 = q[1]; // qy
    float q2 = q[2]; // qw
    float q3 = q[3]; // qz

    // Roll
    float sinr_cosp = 2.0f * (q2 * q0 + q1 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q0 * q0 + q1 * q1);
    angle-> roll = atan2f(sinr_cosp, cosr_cosp) / PI *180;

    // Pitch
    float sinp = 2.0f * (q2 * q1 - q3 * q0);
    if (fabsf(sinp) >= 1.0f)
    angle->pitch = copysignf(M_PI / 2.0f, sinp)/ PI *180;
    else
    angle->pitch = asinf(sinp)/ PI *180;

    // Yaw
    float siny_cosp = 2.0f * (q2 * q3 + q0 * q1);
    float cosy_cosp = 1.0f - 2.0f * (q1 * q1 + q3 * q3);
    angle->yaw = atan2f(siny_cosp, cosy_cosp)/ PI *180;
}

/**
 * @brief 应用椭球校准到磁力计数据
 * @param mx_raw, my_raw, mz_raw  原始磁力计读数（高斯或 μT）
 * @param mx_cal, my_cal, mz_cal  输出校准后的数据
 */
void mag_apply_calibration(float mx_raw, float my_raw, float mz_raw,
                           float *mx_cal, float *my_cal, float *mz_cal) {
    // 1. 减去硬铁偏移
    float x_off = mx_raw - mag_cal.offset[0];
    float y_off = my_raw - mag_cal.offset[1];
    float z_off = mz_raw - mag_cal.offset[2];

    // 2. 应用软铁校正矩阵乘法
    *mx_cal = mag_cal.scale[0][0] * x_off + mag_cal.scale[0][1] * y_off + mag_cal.scale[0][2] * z_off;
    *my_cal = mag_cal.scale[1][0] * x_off + mag_cal.scale[1][1] * y_off + mag_cal.scale[1][2] * z_off;
    *mz_cal = mag_cal.scale[2][0] * x_off + mag_cal.scale[2][1] * y_off + mag_cal.scale[2][2] * z_off;
}

/**
 * @brief 设置校准参数（可从上位机或Flash加载后调用）
 * @param offset  硬铁偏移数组 [x, y, z]
 * @param scale   软铁矩阵 3x3（按行存储）
 */
void mag_set_calibration(float offset[3], float scale[3][3]) {
    for (int i = 0; i < 3; i++) {
        mag_cal.offset[i] = offset[i];
        for (int j = 0; j < 3; j++) {
            mag_cal.scale[i][j] = scale[i][j];
        }
    }
}





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