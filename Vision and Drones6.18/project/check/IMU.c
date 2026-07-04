// =============================================================================
// IMU.c - 惯性测量单元
// =============================================================================
#include "zf_common_headfile.h"

/*
 * 标准惯性坐标系旋转
 *   Roll:  绕 X 轴旋转 (右边往下为正)
 *   Pitch: 绕 Y 轴旋转 (抬头为正)
 *   Yaw:   绕 Z 轴旋转 (逆时针为正, 从上往下看)
 *
 * ax_w =  cp*cy*ax + (sr*sp*cy - cr*sy)*ay + (cr*sp*cy + sr*sy)*az;
 * ay_w =  cp*sy*ax + (sr*sp*sy + cr*cy)*ay + (cr*sp*sy - sr*cy)*az;
 * az_w = -sp*ax + sr*cp*ay + cr*cp*az;
 */

/*
 * 加速度约定:
 *   acc_z = +9.8  world_data.az = +9.8
 *   Z轴朝上: world_data.az -= 9.8f;
 *   Z轴朝下: world_data.az += 9.8f;  (NED系)
 */
#ifndef M_PI
#define M_PI 3.1415926f
#endif

float volatile acc_roll;
float volatile acc_pitch;
float rotation_matrix[3][3];

void imu660rc_get_data(void)
{
    // 单位: 度/秒
    imu_data.gyro_x = imu660rc_gyro_x / 14.2857;
    imu_data.gyro_y = imu660rc_gyro_y / 14.2857;
    imu_data.gyro_z = -imu660rc_gyro_z / 14.2857;

    // 单位: m/s^2
    imu_data.acc_x = imu660rc_acc_x / 4098.36 * 9.8f;
    imu_data.acc_y = imu660rc_acc_y / 4098.36 * 9.8f;
    imu_data.acc_z = imu660rc_acc_z / 4098.36 * 9.8f;
}

/*
 * 高度: TOF + IMU 积分高度
 * 高度速度 = IMU积分 + 高度差分
 * 位置: 加速度积分
 * 水平速度: vx = 光流 + IMU积分
 *           vy = 光流 + IMU积分
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
 * @brief Mahony姿态更新 - 无磁力计融合 (起飞阶段使用)
 * @note  仅使用加速度计+陀螺仪，避免地面磁场干扰
 */
void Mahony_Update_NoMag(void)
{
    float gx = imu_data.gyro_x * (M_PI / 180.0f);
    float gy = imu_data.gyro_y * (M_PI / 180.0f);
    float gz = imu_data.gyro_z * (M_PI / 180.0f);

    float ax = imu_data.acc_x;
    float ay = imu_data.acc_y;
    float az = imu_data.acc_z;

    float acc_norm = sqrtf(ax*ax + ay*ay + az*az);
    if (acc_norm > 0.0f) {
        ax /= acc_norm; ay /= acc_norm; az /= acc_norm;
    }

    const float dt = 0.005f;
    const float Kp = 4.5f;
    const float Ki = 0.05f;

    float qx = imu_data.q0;
    float qy = imu_data.q1;
    float qw = imu_data.q2;
    float qz = imu_data.q3;

    float qwqw = qw*qw, qxqx = qx*qx, qyqy = qy*qy, qzqz = qz*qz;
    float qwqx = qw*qx, qwqy = qw*qy, qwqz = qw*qz;
    float qxqy = qx*qy, qxqz = qx*qz, qyqz = qy*qz;

    // 加速度计校正 (仅校正 Roll/Pitch，无磁力计)
    float vx = 2.0f * (qxqz - qwqy);
    float vy = 2.0f * (qwqx + qyqz);
    float vz = qwqw - qxqx - qyqy + qzqz;

    float halfex = (ay*vz - az*vy);
    float halfey = (az*vx - ax*vz);
    float halfez = (ax*vy - ay*vx);

    static float integralFBx_nm = 0.0f, integralFBy_nm = 0.0f, integralFBz_nm = 0.0f;
    if (Ki > 0.0f) {
        integralFBx_nm += halfex * Ki * dt;
        integralFBy_nm += halfey * Ki * dt;
        integralFBz_nm += halfez * Ki * dt;
        const float limit = 0.2f;
        if (integralFBx_nm >  limit) integralFBx_nm =  limit;
        if (integralFBx_nm < -limit) integralFBx_nm = -limit;
        if (integralFBy_nm >  limit) integralFBy_nm =  limit;
        if (integralFBy_nm < -limit) integralFBy_nm = -limit;
        if (integralFBz_nm >  limit) integralFBz_nm =  limit;
        if (integralFBz_nm < -limit) integralFBz_nm = -limit;
    }

    gx += Kp * halfex + integralFBx_nm;
    gy += Kp * halfey + integralFBy_nm;
    gz += Kp * halfez + integralFBz_nm;

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    float qa = qw, qb = qx, qc = qy, qd = qz;
    qw += (-qb*gx - qc*gy - qd*gz);
    qx += ( qa*gx + qc*gz - qd*gy);
    qy += ( qa*gy - qb*gz + qd*gx);
    qz += ( qa*gz + qb*gy - qc*gx);

    float norm = sqrtf(qw*qw + qx*qx + qy*qy + qz*qz);
    if (norm > 0.0f) {
        qw /= norm; qx /= norm; qy /= norm; qz /= norm;
    }

    imu_data.q0 = qx;
    imu_data.q1 = qy;
    imu_data.q2 = qw;
    imu_data.q3 = qz;

    quat_to_euler(imu_data.q0, imu_data.q1, imu_data.q2, imu_data.q3, &eulerAngle);
}

/**
 * @brief Mahony姿态更新 - 带磁力计融合 (正常飞行使用)
 * @note  陀螺仪 + 加速度计 + 磁力计 九轴融合
 */
void Mahony_Mag_Update(void)
{
    // 起飞阶段不融合磁力计，避免地面磁场干扰
    if(!flag.mag_fusion_enabled)
    {
        Mahony_Update_NoMag();
        return;
    }

    // 陀螺仪数据: 度/秒 -> 弧度/秒
    float gx = imu_data.gyro_x * (M_PI / 180.0f);
    float gy = imu_data.gyro_y * (M_PI / 180.0f);
    float gz = imu_data.gyro_z * (M_PI / 180.0f);

    // 加速度计数据: 归一化
    float ax = imu_data.acc_x;
    float ay = imu_data.acc_y;
    float az = imu_data.acc_z;

    float acc_norm = sqrtf(ax*ax + ay*ay + az*az);
    if (acc_norm > 0.0f) {
        ax /= acc_norm;
        ay /= acc_norm;
        az /= acc_norm;
    }

    // 磁力计数据: 校准后归一化
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
    const float Kp = 4.5f;       // 比例增益 (加速度计和陀螺仪共用)
    const float Ki = 0.05f;      // 积分增益

    // 获取当前四元数 (存储顺序: q0=qx, q1=qy, q2=qw, q3=qz)
    float qx = imu_data.q0;
    float qy = imu_data.q1;
    float qw = imu_data.q2;      // 实部
    float qz = imu_data.q3;

    // 预计算四元数乘积
    float qwqw = qw*qw, qxqx = qx*qx, qyqy = qy*qy, qzqz = qz*qz;
    float qwqx = qw*qx, qwqy = qw*qy, qwqz = qw*qz;
    float qxqy = qx*qy, qxqz = qx*qz, qyqz = qy*qz;

    // ================= 1. 加速度计校正 (校正 Roll/Pitch) =================
    float vx = 2.0f * (qxqz - qwqy);
    float vy = 2.0f * (qwqx + qyqz);
    float vz = qwqw - qxqx - qyqy + qzqz;

    float halfex_acc = (ay*vz - az*vy);
    float halfey_acc = (az*vx - ax*vz);
    float halfez_acc = (ax*vy - ay*vx);

    // ================= 2. 磁力计校正 (校正 Yaw) =================
    float hx = mx*(0.5f - qyqy - qzqz) + my*(qxqy - qwqz)       + mz*(qxqz + qwqy);
    float hy = mx*(qxqy + qwqz)       + my*(0.5f - qxqx - qzqz) + mz*(qyqz - qwqx);
    float hz = mx*(qxqz - qwqy)       + my*(qyqz + qwqx)       + mz*(0.5f - qxqx - qyqy);

    float bx = sqrtf(hx*hx + hy*hy);
    float bz = hz;

    float halfwx = bx*(0.5f - qyqy - qzqz) + bz*(qxqz - qwqy);
    float halfwy = bx*(qxqy - qwqz)       + bz*(qwqx + qyqz);
    float halfwz = bx*(qwqy + qxqz)       + bz*(0.5f - qxqx - qyqy);

    float halfex_mag = (my*halfwz - mz*halfwy);
    float halfey_mag = (mz*halfwx - mx*halfwz);
    float halfez_mag = (mx*halfwy - my*halfwx);

    // ================= 3. 总误差 = 加速度计 + 磁力计 =================
    float halfex = halfex_acc + halfex_mag;
    float halfey = halfey_acc + halfey_mag;
    float halfez = halfez_acc + halfez_mag;

    // ================= 4. PI 控制器修正 =================
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

    // 转欧拉角
    quat_to_euler(imu_data.q0, imu_data.q1, imu_data.q2, imu_data.q3, &eulerAngle);

    // 参考值 (调试用)
    float mag_yaw = atan2f(-my, mx) * 180.0f / M_PI;
    if (mag_yaw < 0) mag_yaw += 360.0f;

    acc_roll = atan2f(ay, az) * 180.0f / M_PI;
    acc_pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI;
}

/**
 * @brief 四元数转欧拉角 (角度制)
 * @param q    四元数 (顺序: q0=qx, q1=qy, q2=qw, q3=qz)
 * @param angle 输出的欧拉角 (度)
 */
void quat_to_euler(float q0, float q1, float q2, float q3, _euler_param_st *angle)
{
    // Roll
    float sinr_cosp = 2.0f * (q2 * q0 + q1 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q0 * q0 + q1 * q1);
    angle->roll = atan2f(sinr_cosp, cosr_cosp) / M_PI * 180.0f;

    // Pitch
    float sinp = 2.0f * (q2 * q1 - q3 * q0);
    if (fabsf(sinp) >= 1.0f)
        angle->pitch = copysignf(M_PI / 2.0f, sinp) / M_PI * 180.0f;
    else
        angle->pitch = asinf(sinp) / M_PI * 180.0f;

    // Yaw
    float siny_cosp = 2.0f * (q2 * q3 + q0 * q1);
    float cosy_cosp = 1.0f - 2.0f * (q1 * q1 + q3 * q3);
    angle->yaw = atan2f(siny_cosp, cosy_cosp) / M_PI * 180.0f;
}

/**
 * @brief 应用磁力计校准参数
 */
void mag_apply_calibration(float mx_raw, float my_raw, float mz_raw,
                           float *mx_cal, float *my_cal, float *mz_cal)
{
    // 1. 减去硬磁偏移
    float x_off = mx_raw - mag_cal.offset[0];
    float y_off = my_raw - mag_cal.offset[1];
    float z_off = mz_raw - mag_cal.offset[2];

    // 2. 应用软磁校准矩阵
    *mx_cal = mag_cal.scale[0][0] * x_off + mag_cal.scale[0][1] * y_off + mag_cal.scale[0][2] * z_off;
    *my_cal = mag_cal.scale[1][0] * x_off + mag_cal.scale[1][1] * y_off + mag_cal.scale[1][2] * z_off;
    *mz_cal = mag_cal.scale[2][0] * x_off + mag_cal.scale[2][1] * y_off + mag_cal.scale[2][2] * z_off;
}

/**
 * @brief 设置磁力计校准参数
 */
void mag_set_calibration(float offset[3], float scale[3][3])
{
    for (int i = 0; i < 3; i++) {
        mag_cal.offset[i] = offset[i];
        for (int j = 0; j < 3; j++) {
            mag_cal.scale[i][j] = scale[i][j];
        }
    }
}

// =============================================================================
// IMU六轴 → 旋转矩阵 → 世界坐标系加速度/速度/位置
// =============================================================================

/*
 * 输入: quat[4] = {qx, qy, qz, qw}
 * 输出: R[3][3] 旋转矩阵
 */
void quarternion_to_rotation_matrix()
{
    float qx = imu_data.q0;
    float qy = imu_data.q1;
    float qz = imu_data.q3;
    float qw = imu_data.q2;

    float qx2 = qx * qx;
    float qy2 = qy * qy;
    float qz2 = qz * qz;
    float qw2 = qw * qw;

    rotation_matrix[0][0] = qw2 + qx2 - qy2 - qz2;
    rotation_matrix[0][1] = 2.0f * (qx*qy - qw*qz);
    rotation_matrix[0][2] = 2.0f * (qx*qz + qw*qy);

    rotation_matrix[1][0] = 2.0f * (qx*qy + qw*qz);
    rotation_matrix[1][1] = qw2 - qx2 + qy2 - qz2;
    rotation_matrix[1][2] = 2.0f * (qy*qz - qw*qx);

    rotation_matrix[2][0] = 2.0f * (qx*qz - qw*qy);
    rotation_matrix[2][1] = 2.0f * (qy*qz + qw*qx);
    rotation_matrix[2][2] = qw2 - qx2 - qy2 + qz2;

    // 体坐标系加速度 → 世界坐标系 (ar = R * ab)
    float ax = imu_data.acc_x;
    float ay = imu_data.acc_y;
    float az = imu_data.acc_z;
    world_data.ax = rotation_matrix[0][0]*ax + rotation_matrix[0][1]*ay + rotation_matrix[0][2]*az;
    world_data.ay = rotation_matrix[1][0]*ax + rotation_matrix[1][1]*ay + rotation_matrix[1][2]*az;
    world_data.az = rotation_matrix[2][0]*ax + rotation_matrix[2][1]*ay + rotation_matrix[2][2]*az - 9.8f;
}