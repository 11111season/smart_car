// =============================================================================
// config.h - 飞控配置文件
// =============================================================================
#ifndef CONFIG_H_
#define CONFIG_H_

// =============================================================================
// 功能模块开关 (1=启用, 0=禁用)
// =============================================================================
#define ATTITUDE                    (1)     // 姿态控制
#define HEIGHT                      (0)     // 高度控制
#define POSITION_HOLD               (0)     // 位置保持
#define POSITION_TARGET_ZERO        (0)     // 1=定点悬停(只用速度环), 0=视觉追踪小车
#define THROTTLE_ATTENUATION        (0)     // 油门衰减
#define BASE_SPEED                  (1)     // 基础速度模式

// =============================================================================
// 调试模式开关
// =============================================================================
#define DEBUG                       (0)     // 调试模式

// =============================================================================
// VOFA+ 调试分类 (1=输出, 0=禁用，每类独立控制)
// =============================================================================
#define VOFA_RAW_IMU               (0)     // 原始IMU: gyro_x/y/z, acc_x/y/z
#define VOFA_FUSED_EULER           (0)     // 融合姿态角: roll, pitch, yaw
#define VOFA_PID_ANGLE             (0)     // 角度环PID: target/error/out (Roll+Pitch)
#define VOFA_PID_RATE              (0)     // 角速度环PID: out (VelX/Y/Z)
#define VOFA_PID_HEIGHT            (0)     // 高度环PID: target/current/out
#define VOFA_MOTOR_OUT             (0)     // 电机PWM: m1~m4
#define VOFA_WORLD_POS             (0)     // 世界坐标: px,py,vx,vy

// =============================================================================
// VOFA+ 输出宏 (每条输出一行中的若列，主循环末尾补 \r\n)
// =============================================================================
#if VOFA_RAW_IMU
#define VOFA_RAW_IMU_PRINT() \
    printf("%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,", \
           imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z, \
           imu_data.acc_x, imu_data.acc_y, imu_data.acc_z)
#else
#define VOFA_RAW_IMU_PRINT()
#endif

#if VOFA_FUSED_EULER
#define VOFA_FUSED_EULER_PRINT() \
    printf("%.2f,%.2f,%.2f,", \
           eulerAngle.roll, eulerAngle.pitch, eulerAngle.yaw)
#else
#define VOFA_FUSED_EULER_PRINT()
#endif

#if VOFA_PID_ANGLE
#define VOFA_PID_ANGLE_PRINT() \
    printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,", \
           PIDRoll.target, PIDRoll.error, PIDRoll.out, \
           PIDPitch.target, PIDPitch.error, PIDPitch.out)
#else
#define VOFA_PID_ANGLE_PRINT()
#endif

#if VOFA_PID_RATE
#define VOFA_PID_RATE_PRINT() \
    printf("%.2f,%.2f,%.2f,", \
           PIDVelX.out, PIDVelY.out, PIDVelZ.out)
#else
#define VOFA_PID_RATE_PRINT()
#endif

#if VOFA_PID_HEIGHT
#define VOFA_PID_HEIGHT_PRINT() \
    printf("%.3f,%.3f,%.2f,", \
           PIDHeight.target, world_data.pz, PIDVelH.out)
#else
#define VOFA_PID_HEIGHT_PRINT()
#endif

#if VOFA_MOTOR_OUT
#define VOFA_MOTOR_OUT_PRINT() \
    printf("%d,%d,%d,%d,", m1, m2, m3, m4)
#else
#define VOFA_MOTOR_OUT_PRINT()
#endif

#if VOFA_WORLD_POS
#define VOFA_WORLD_POS_PRINT() \
    printf("%.2f,%.2f,%.2f,%.2f,", \
           world_data.px, world_data.py, world_data.vx, world_data.vy)
#else
#define VOFA_WORLD_POS_PRINT()
#endif

// =============================================================================
// 电机配置
// =============================================================================
#define MOTOR_MIN_DUTY              (2800)  // 电机最小占空比
#define MOTOR_MAX_DUTY              (7500)  // 电机最大占空比
#define HOVER_THRUST                (5250)  // 悬停油门

// =============================================================================
// PID配置
// =============================================================================
#define CONTROL_DT                  (0.005f) // 控制周期 (s)

// ---------- 内环角速度PID积分限幅 ----------
#define VEL_INTEG_LIMIT_HIGH        (800.0f)
#define VEL_INTEG_LIMIT_LOW         (-800.0f)

// ---------- 内环角速度PID输出限幅 ----------
#define VELX_OUT_LIMIT_HIGH         (2000.0f)
#define VELX_OUT_LIMIT_LOW          (-2000.0f)
#define VELY_OUT_LIMIT_HIGH         (2000.0f)
#define VELY_OUT_LIMIT_LOW          (-2000.0f)
#define VELZ_OUT_LIMIT_HIGH         (1500.0f)
#define VELZ_OUT_LIMIT_LOW          (-1500.0f)

// ---------- 外环角度PID积分/输出限幅 ----------
#define ROLL_INTEG_LIMIT_HIGH       (0.0f)
#define ROLL_INTEG_LIMIT_LOW        (0.0f)
#define ROLL_OUT_LIMIT_HIGH         (200.0f)
#define ROLL_OUT_LIMIT_LOW          (-200.0f)

#define PITCH_INTEG_LIMIT_HIGH      (0.0f)
#define PITCH_INTEG_LIMIT_LOW       (0.0f)
#define PITCH_OUT_LIMIT_HIGH        (200.0f)
#define PITCH_OUT_LIMIT_LOW         (-200.0f)

#define YAW_INTEG_LIMIT_HIGH        (0.0f)
#define YAW_INTEG_LIMIT_LOW         (0.0f)
#define YAW_OUT_LIMIT_HIGH          (150.0f)
#define YAW_OUT_LIMIT_LOW           (-150.0f)

// ---------- 高度环PID积分/输出限幅 ----------
#define HEIGHT_INTEG_LIMIT_HIGH     (150.0f)
#define HEIGHT_INTEG_LIMIT_LOW      (-150.0f)
#define HEIGHT_OUT_LIMIT_HIGH       (0.4f)
#define HEIGHT_OUT_LIMIT_LOW        (-0.4f)

#define VELH_INTEG_LIMIT_HIGH       (700.0f)
#define VELH_INTEG_LIMIT_LOW        (-700.0f)
#define VELH_OUT_LIMIT_HIGH         (1500.0f)
#define VELH_OUT_LIMIT_LOW          (-1500.0f)

// ---------- 位置环PID积分/输出限幅 ----------
#define POS_INTEG_LIMIT_HIGH        (0.0f)
#define POS_INTEG_LIMIT_LOW         (-0.0f)
#define POS_OUT_LIMIT_HIGH          (2.0f)
#define POS_OUT_LIMIT_LOW           (-2.0f)

#define POS_VEL_INTEG_LIMIT_HIGH    (100.0f)
#define POS_VEL_INTEG_LIMIT_LOW     (-100.0f)
#define POS_VEL_OUT_LIMIT_HIGH      (30.0f)
#define POS_VEL_OUT_LIMIT_LOW       (-30.0f)

// =============================================================================
// 滤波器配置
// =============================================================================
#define GYRO_FILTER_FREQ            (50.0f)  // 陀螺仪一阶低通 (Hz)，增强以压制噪声侧D生效
#define GYRO_NOTCH_CENTER           (80.0f)  // 陀螺仪陷波中心频率 (Hz)，目标电机振动
#define GYRO_NOTCH_Q                (3.0f)   // 陷波品质因数，越高越窄
#define HEIGHT_FILTER_FREQ          (50.0f)  // 高度数据滤波 (Hz)，平衡响应与平滑
#define HEIGHT_VZ_FILTER_FREQ       (5.0f)   // 垂直速度滤波 (Hz)，加大滤波抗干扰
#define OF_VX_FILTER_FREQ           (30.0f)  // 光流VX滤波 (Hz)
#define OF_VY_FILTER_FREQ           (30.0f)  // 光流VY滤波 (Hz)
#define ANGLE_DERIV_FILTER_FREQ     (15.0f)  // 角度微分滤波截止频率 (Hz)
#define RATE_DERIV_FILTER_FREQ      (30.0f)  // 角速度微分滤波截止频率 (Hz)

// =============================================================================
// Mahony姿态融合自适应Kp
// =============================================================================
// 核心思路: 水平加速时降低加速度计修正权重，避免角度被加速度"拽歪"
// acc_dev = |加速度模长 - 1g|  →  acc_dev越大说明机动越剧烈，Kp降得越低
#define MAHONY_KP_STARTUP           (7.0f)   // 开机前5秒Kp上限 (快速收敛)
#define MAHONY_KP_NORMAL            (0.4f)   // 5秒后Kp上限 (稳态收敛)
#define MAHONY_KP_MIN               (0.05f)  // 剧烈机动时Kp (靠陀螺仪独立积分)
#define MAHONY_ACC_DEV_THRESH       (0.15f)  // 加速度偏离阈值(g)，超过后线性衰减Kp
#define MAHONY_STARTUP_TIME         (6.0f)   // 开机加速收敛时长 (秒)

// =============================================================================
// 光流传感器安装偏移 (相对于重心，机体坐标系，单位: 米)
// =============================================================================
// X前+Y右+Z下，传感器Y轴偏心4.5cm
#define OF_SENSOR_X                 (0.0f)   // 前后 (前为正)
#define OF_SENSOR_Y                 (-0.045f) // 左右 (右为正)，4.5cm偏左

// =============================================================================
// Anti-Gravity 油门前馈
// =============================================================================
// 推油门瞬间临时放大内环I项增益，防止急加速时掉头
#define ANTI_GRAVITY_GAIN           (3.0f)   // I项临时放大倍数 (2~6)
#define ANTI_GRAVITY_FILTER_FREQ    (6.0f)   // 油门变化率低通截止 (Hz, 4~8)

// =============================================================================
// 外环角度PID参数 (P控制)
// =============================================================================
#define ROLL_KP                     (3.0f)  // Roll角度环 P
#define ROLL_KI                     (0.0f)  // Roll角度环 I
#define ROLL_KD                     (0.00f) // Roll角度环 D

#define PITCH_KP                    (3.0f)  // Pitch角度环 P
#define PITCH_KI                    (0.0f)  // Pitch角度环 I
#define PITCH_KD                    (0.00f) // Pitch角度环 D

#define YAW_KP                      (4.0f)  // Yaw角度环 P
#define YAW_KI                      (0.0f)  // Yaw角度环 I
#define YAW_KD                      (0.00f) // Yaw角度环 D
                     
// =============================================================================
// 高度环PID参数
// =============================================================================
// 外环(高度->速度): 将高度误差转换为期望垂直速度
#define HEIGHT_KP                   (1.0f)  // 高度外环 P，降低以提高稳定性
#define HEIGHT_KI                   (0.0f) // 高度外环 I
#define HEIGHT_KD                   (0.0f)  // 高度外环 D

// 内环(垂直速度->油门): 将垂直速度误差转换为油门修正量
#define VELH_KP                     (700.0f)  // 垂直速度内环 P
#define VELH_KI                     (400.0f)  // 垂直速度内环 I
#define VELH_KD                     (0.00f) // 垂直速度内环 D

// =============================================================================
// 光流定点PID参数 
// =============================================================================
// 外环(位置->速度): 将位置误差转换为期望水平速度
#define POS_KP                      (0.6f)  // 位置外环 P
#define POS_KI                      (0.0f) // 位置外环 I
#define POS_KD                      (0.00f)  // 位置外环 D

// 内环(水平速度->角度): 将速度误差转换为期望角度
#define POS_VELX_KP                 (6.0f) // X方向速度内环 P (前后→Pitch)，原7.0
#define POS_VELY_KP                 (6.0f) // Y方向速度内环 P (左右→Roll)，原6.0
#define POS_VEL_KI                  (0.8f)  // 水平速度内环 I
#define POS_VEL_KD                  (0.0f)  // 水平速度内环 D  

// =============================================================================
// 内环角速度PID参数 (PD控制)
// =============================================================================
#define VELX_KP                     (5.5f)  // Roll角速度环 P //6.5
#define VELX_KI                     (2.6f)  // Roll角速度环 I  // 2.6
#define VELX_KD                    (0.6f)   // Roll角速度环 D (阻尼抑制P震荡) // 0.6

#define VELY_KP                     (5.5f)  // Pitch角速度环 P  
#define VELY_KI                     (2.6f)  // Pitch角速度环 I
#define VELY_KD                    (0.6f)   // Pitch角速度环 D (阻尼抑制P震荡) //0.6

#define VELZ_KP                     (10.0f)  // Yaw角速度环 P
#define VELZ_KI                     (3.0f)  // Yaw角速度环 I
#define VELZ_KD                     (0.0f) // Yaw角速度环 D

// =============================================================================
// 微分滤波器截止频率配置
// =============================================================================
#define DERIV_FILTER_VELX           (1.0f)  // Roll角速度微分滤波 (Hz)
#define DERIV_FILTER_VELY           (1.0f)  // Pitch角速度微分滤波 (Hz)
#define DERIV_FILTER_VELZ           (10.0f)  // Yaw角速度微分滤波 (Hz)
#define DERIV_FILTER_ROLL           (30.0f)  // Roll角度微分滤波 (Hz)
#define DERIV_FILTER_PITCH          (30.0f)  // Pitch角度微分滤波 (Hz)
#define DERIV_FILTER_YAW            (30.0f)  // Yaw角度微分滤波 (Hz)
#define DERIV_FILTER_HEIGHT         (10.0f)  // 高度外环微分滤波 (Hz)
#define DERIV_FILTER_VELH           (20.0f)  // 垂直速度内环微分滤波 (Hz)
#define DERIV_FILTER_POS            (1.0f)   // 位置外环微分滤波 (Hz)
#define DERIV_FILTER_POS_VEL        (1.0f)   // 水平速度内环微分滤波 (Hz)

// =============================================================================
// 起飞阶段控制配置
// =============================================================================
#define TAKEOFF_TARGET_HEIGHT       (0.9f)  // 起飞目标高度 (m)
#define TAKEOFF_ALT_SWITCH          (0.3f)  // 启用磁力计融合的高度阈值 (m)
#define TAKEOFF_STABLE_TIME         (2.0f)  // 稳定判定时间 (s)

#endif /* CONFIG_H_ */
