#include "zf_common_headfile.h"
/*
 * =============================================================================
 * of.c - 光流传感器数据处理与视觉-惯导融合
 * =============================================================================
 *
 * 【原理说明】
 *   光流传感器 (UPFLOW302) 检测地面图像的运动，输出像素位移 (Δx, Δy)。
 *   结合传感器安装高度 (h) 和相机焦距参数 (K)，可计算出水平速度：
 *
 *     角速度 ω = Δx / (dt * K)         // 由像素位移推算角速度
 *     线速度 v = ω * h                  // 由角速度和高度推算线速度
 *     即：v = Δx * h / (dt * K)        // 合并公式
 *
 * 【融合策略 - Mahony互补滤波】
 *   光流速度 (高频但易受光照/纹理影响) 与 IMU加速度积分速度 (低频不漂但噪声大)
 *   通过类Mahony互补滤波融合：
 *
 *     v_fusion = Kp * (v_of - v_imu) + Ki * ∫(v_of - v_imu)dt
 *
 *   融合结果经一阶低通滤波 (PT1) 平滑后输出。
 *
 * 【对外接口】
 *   world_data.vx, world_data.vy  → 融合后的世界坐标系速度
 *   world_data.px, world_data.py  → 融合后的累计位置
 *   of.vx_pt1, of.vy_pt1         → 滤波后的光流速度 (用于观测校正)
 * =============================================================================
 */

/*
 * =============================================================================
 * OF_init - 光流传感器初始化
 * =============================================================================
 * 功能: 初始化光流传感器 (UPFLOW302) 并清零所有相关状态变量
 * 说明:
 *   1. 调用 upflow302_receive_init() 启动传感器数据接收
 *   2. 清零速度 (vx, vy)、位置 (px, py)、像素位移 (dx, dy)
 *   3. 首次初始化时 (flag.of_init==0)，将滤波后的光流速度赋给世界坐标系，
 *      作为速度融合的初始值
 * =============================================================================
 */
void OF_init(void)
{
    upflow302_receive_init();                  // 初始化UPFLOW302传感器数据接收
    world_data.vx = 0;                         // 世界坐标系X方向速度清零
    world_data.vy = 0;                         // 世界坐标系Y方向速度清零
    world_data.px = 0;                         // X方向累计位置清零
    world_data.py = 0;                         // Y方向累计位置清零
    
    of.vx = 0;                                 // 光流原始X速度清零
    of.vy = 0;                                 // 光流原始Y速度清零
    of.vx_pt1 = 0;                             // 一阶滤波后X速度清零
    of.vy_pt1 = 0;                             // 一阶滤波后Y速度清零

    of.dx = 0;                                 // X方向像素位移清零
    of.dy = 0;                                 // Y方向像素位移清零
    of.dx_i = 0;                               // X方向累计像素位移清零
    of.dy_i = 0;                               // Y方向累计像素位移清零
    
    if(!flag.of_init)
    {
        world_data.vx = of.vx_pt1;             // 首次初始化：用光流速度填充世界速度
        world_data.vy = of.vy_pt1;
        flag.of_init = 1;                      // 标记光流已初始化
        return;
    }
}

/*
 * =============================================================================
 * OF_data_deal - 光流数据解算与预处理
 * =============================================================================
 * 功能: 将光流传感器原始像素位移解算为水平速度
 * 
 * 处理流程:
 *   1. 获取当前高度 (来自超声波/激光测距)
 *   2. 读取UPFLOW302的像素位移 (dx, dy)，修正方向
 *   3. 根据像素位移 ÷ (dt × K) 推算角速度 (optical_ang)
 *   4. 陀螺仪角速度补偿 → 去除机体旋转带来的虚假光流
 *   5. 角速度 × 高度 → 线速度
 *   6. 一阶低通滤波 (PT1) 平滑输出
 * 
 * 注意:
 *   - K 为光流传感器角度-像素换算系数，需实际标定
 *   - 陀螺仪补偿公式: ω_true = ω_optical + ω_gyro
 *     当无人机旋转时，光流会测到额外的"虚假"运动，需用陀螺仪数据抵消
 * =============================================================================
 */
void OF_data_deal(float dt)
{
    // 1. 获取当前滤波后的高度 (由超声波/激光测距模块提供)
    of.height = alt.height;
    
    // 2. 读取原始像素位移 (UPFLOW302坐标系到机体坐标系的映射)
    of.dx = -upflow302_receive.upflow302_y;     // X方向位移 (取反修正)
    of.dy = upflow302_receive.upflow302_x;      // Y方向位移
    
    // 3. 陀螺仪数据单位转换: 度/秒 → 弧度/秒
    imu_data.gyro_rad_y = imu_data.gyro_y * (PI / 180.0f);
    imu_data.gyro_rad_x = imu_data.gyro_x * (PI / 180.0f);

    // 4. 像素位移 → 角速度: ω = Δpixel / (dt × K)
    of.optical_ang_x = of.dx / (dt * of.K);     // 光流推算的X轴角速度 (rad/s)
    of.optical_ang_y = of.dy / (dt * of.K);     // 光流推算的Y轴角速度 (rad/s)

    // 5. 陀螺仪补偿: 去除机体旋转分量
    //    光流视角速度 = 真实运动角速度 - 机体旋转角速度
    //    因此: 真实运动角速度 = 光流视角速度 + 机体旋转角速度
    of.rotation_ex = of.optical_ang_x + imu_data.gyro_rad_y;   // 补偿后X轴运动角速度
    of.rotation_ey = of.optical_ang_y + imu_data.gyro_rad_x;   // 补偿后Y轴运动角速度

    // 6. 角速度 → 线速度: v = ω × h
    of.vx = of.rotation_ex * of.height;         // X方向水平速度 (m/s)
    of.vy = of.rotation_ey * of.height;         // Y方向水平速度 (m/s)
    
    // 7. 一阶低通滤波 (PT1): 抑制高频噪声
    of.vx_pt1 = PT1Filter_Apply(&filter_pwm3901_vx, of.vx);
    of.vy_pt1 = PT1Filter_Apply(&filter_pwm3901_vy, of.vy);
}




/*
 * =============================================================================
 * velocity_mahony_fusion - 光流-惯导速度融合 (Mahony互补滤波)
 * =============================================================================
 * 功能: 将光流测量速度与IMU加速度积分速度进行融合
 * 
 * 【算法原理】
 *   借鉴Mahony姿态融合的思路，将光流速度作为"观测值"，
 *   加速度积分速度作为"预测值"，通过PI反馈校正：
 *
 *     预测: v_pred += a_imu * dt          (加速度积分)
 *     误差: e = v_of - v_pred             (光流观测 - 积分预测)
 *     校正: v_fusion = v_pred + Kp*e + Ki*∫e*dt
 *
 * 【参数说明】
 *   Kp = 0.8  → 比例增益，决定光流置信度
 *   Ki = 0.2  → 积分增益，消除加速度计漂移引起的累积误差
 *   起飞阶段 Ki = 0 (高度未到目标值，光流速度不可靠)
 *
 * 【输出】
 *   world_data.vx, world_data.vy  → 融合后的速度
 *   world_data.px, world_data.py  → 位置 (速度积分)
 * =============================================================================
 */
void velocity_mahony_fusion(float dt) 
{
    static float Kp, Ki, ex, ey;
    static float integralX = 0.0f, integralY = 0.0f;

    Kp = 0.8f;                          // 比例增益: 光流观测置信度
    Ki = flag.takeoff_phase ? 0.0f : 0.2f;  // 起飞阶段不用I (高度不够时水平速度不可靠)
    
    // 预测步: 加速度积分得到速度 (imu_data.ax/ay 由加速度计测得)
    world_data.vx += world_data.ax * dt;    // X方向速度预测 (积分)
    world_data.vy += world_data.ay * dt;    // Y方向速度预测 (积分)

    // 观测误差: 光流速度 - 加速度积分速度
    ex = of.vx_pt1 - world_data.vx;         // X方向观测误差
    ey = of.vy_pt1 - world_data.vy;         // Y方向观测误差
    
    // 积分项累积: 消除加速度计零偏引起的漂移
    if (Ki > 0.0f) 
    {
        integralX += Ki * ex * dt;
        integralY += Ki * ey * dt;
    }
    
    // 校正步: PI反馈修正速度预测值
    world_data.vx += Kp * ex + integralX;   // X方向融合速度
    world_data.vy += Kp * ey + integralY;   // Y方向融合速度
    
    // 位置积分: 速度 → 位置
    world_data.px += world_data.vx * dt;    // X方向累计位置
    world_data.py += world_data.vy * dt;    // Y方向累计位置
}

