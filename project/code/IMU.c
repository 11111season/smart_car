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

float volatile acc_roll;
float volatile acc_pitch;  
    
void imu660rc_get_data(void)
{
    //单位度每秒
    imu_data.gyro_x = imu660rc_gyro_x/14.2857;  
    imu_data.gyro_y = imu660rc_gyro_y/14.2857;  
    imu_data.gyro_z = -imu660rc_gyro_z/14.2857;  
;
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
void Mahony_Mag_Update(void) {
    // 陀螺仪数据：度/秒 → 弧度/秒
    float gx = imu_data.gyro_x * (M_PI / 180.0f);
    float gy = imu_data.gyro_y * (M_PI / 180.0f);
    float gz = imu_data.gyro_z * (M_PI / 180.0f);
 
    // 加速度计数据：读取并归一化
    float ax = imu_data.acc_x;
    float ay = imu_data.acc_y;
    float az = imu_data.acc_z;

    
    float acc_norm = sqrtf(ax*ax + ay*ay + az*az);
    if (acc_norm > 0.0f) {
        ax /= acc_norm;
        ay /= acc_norm;
        az /= acc_norm;
    }

    // 磁力计数据：读取、校准、归一化
    float mx_raw = qmc5883l_mag_x_gauss;
    float my_raw = qmc5883l_mag_y_gauss;
    float mz_raw = qmc5883l_mag_z_gauss;
    float mx_cal, my_cal, mz_cal;
    mag_apply_calibration(mx_raw, my_raw, mz_raw, &mx_cal, &my_cal, &mz_cal);
    float mx = mx_cal, my = my_cal, mz = mz_cal;

    float mag_norm = sqrtf(mx*mx + my*my + mz*mz);
    if (mag_norm > 0.0f) {
        mx /= mag_norm;
        my /= mag_norm;
        mz /= mag_norm;
    } else {
        return;
    }

    const float dt = 0.005f;
    const float Kp = 4.5f;      // 比例增益（磁力计和加速度计共用，或分别调）
    const float Ki = 0.05f;    // 积分增益

    // 提取当前四元数（你的顺序：q0=qx, q1=qy, q2=qw, q3=qz）
    float qx = imu_data.q0;
    float qy = imu_data.q1;
    float qw = imu_data.q2;   // 实部
    float qz = imu_data.q3;

    // 预计算四元数乘积项
    float qwqw = qw*qw, qxqx = qx*qx, qyqy = qy*qy, qzqz = qz*qz;
    float qwqx = qw*qx, qwqy = qw*qy, qwqz = qw*qz;
    float qxqy = qx*qy, qxqz = qx*qz, qyqz = qy*qz;

    // ================= 1. 重力向量误差（加速度计修正 Roll/Pitch）=================
    // 理论重力方向（世界坐标系 [0,0,1] 转至机体坐标系）
    float vx = 2.0f * (qxqz - qwqy);
    float vy = 2.0f * (qwqx + qyqz);
    float vz = qwqw - qxqx - qyqy + qzqz;

    // 加速度计测量值 ax, ay, az 已经是机体坐标系下的单位向量
    // 叉乘得到误差
    float halfex_acc = (ay*vz - az*vy);
    float halfey_acc = (az*vx - ax*vz);
    float halfez_acc = (ax*vy - ay*vx);

    // ================= 2. 磁场向量误差（磁力计修正 Yaw）=================
    // 将机体磁场旋转到世界坐标系
    float hx = mx*(0.5f - qyqy - qzqz) + my*(qxqy - qwqz)       + mz*(qxqz + qwqy);
    float hy = mx*(qxqy + qwqz)       + my*(0.5f - qxqx - qzqz) + mz*(qyqz - qwqx);
    float hz = mx*(qxqz - qwqy)       + my*(qyqz + qwqx)       + mz*(0.5f - qxqx - qyqy);

    float bx = sqrtf(hx*hx + hy*hy);
    float bz = hz;

    // 将参考磁场旋转回机体坐标系
    float halfwx = bx*(0.5f - qyqy - qzqz) + bz*(qxqz - qwqy);
    float halfwy = bx*(qxqy - qwqz)       + bz*(qwqx + qyqz);
    float halfwz = bx*(qwqy + qxqz)       + bz*(0.5f - qxqx - qyqy);

    // 磁力计误差
    float halfex_mag = (my*halfwz - mz*halfwy);
    float halfey_mag = (mz*halfwx - mx*halfwz);
    float halfez_mag = (mx*halfwy - my*halfwx);

    // ================= 3. 总误差 = 加速度计误差 + 磁力计误差 =================
    float halfex = halfex_acc + halfex_mag;
    float halfey = halfey_acc + halfey_mag;
    float halfez = halfez_acc + halfez_mag;

    // ================= 4. PI 修正陀螺仪 =================
    static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
    if (Ki > 0.0f) {
        integralFBx += halfex * Ki * dt;
        integralFBy += halfey * Ki * dt;
        integralFBz += halfez * Ki * dt;
        // 积分限幅
        const float limit = 0.2f;
        if (integralFBx >  limit) integralFBx =  limit;
        if (integralFBx < -limit) integralFBx = -limit;
        if (integralFBy >  limit) integralFBy =  limit;
        if (integralFBy < -limit) integralFBy = -limit;
        if (integralFBz >  limit) integralFBz =  limit;
        if (integralFBz < -limit) integralFBz = -limit;
    }

    gx += Kp * halfex + integralFBx;
    gy += Kp * halfey + integralFBy;
    gz += Kp * halfez + integralFBz;

    // ================= 5. 四元数一阶积分更新 =================
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    float qa = qw;
    float qb = qx;
    float qc = qy;
    float qd = qz;
    qw += (-qb*gx - qc*gy - qd*gz);
    qx += ( qa*gx + qc*gz - qd*gy);
    qy += ( qa*gy - qb*gz + qd*gx);
    qz += ( qa*gz + qb*gy - qc*gx);

    // 归一化
    float norm = sqrtf(qw*qw + qx*qx + qy*qy + qz*qz);
    if (norm > 0.0f) {
        qw /= norm; qx /= norm; qy /= norm; qz /= norm;
    }

    // 写回
    imu_data.q0 = qx;
    imu_data.q1 = qy;
    imu_data.q2 = qw;
    imu_data.q3 = qz;
    
    // 转欧拉角（调试）
    quat_to_euler(imu_data.q0, imu_data.q1, imu_data.q2, imu_data.q3, &eulerAngle);
    // 磁力计直接计算的航向（用于对比）
// 融合后的 Roll, Pitch（来自 Mahony 更新后的 eulerAngle）

    float mag_yaw = atan2f(-my, mx) * 180.0f / M_PI;
    if (mag_yaw < 0) mag_yaw += 360.0f;

    acc_roll = atan2f(ay, az) * 180.0f / M_PI;
    acc_pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI;

}

/**
 * @brief 将你的四元数转换为欧拉角（角度）
 * @param q    四元数数组（你的顺序）
 * @param roll, pitch, yaw  输出欧拉角 (角度)
 */
void quat_to_euler(float q0 ,float q1 ,float q2 ,float q3 ,_euler_param_st* angle) {
 

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
void quarternion_to_rotation_matrix()
{
    float qx = imu_data.q0;
    float qy = imu_data.q1;
    float qz = imu_data.q3;
    float qw = imu_data.q2;
    
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
    world_data.ax = R11*imu_data.acc_x + R12*imu_data.acc_y + R13*imu_data.acc_z; //世界坐标系，用于测速
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