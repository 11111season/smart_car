#include "zf_common_headfile.h"

void TOF_init(void)
{
    // 硬件初始化
    dl1b_init();
    
    // 状态初始化
    alt.height       = 0;
    alt.last_height  = 0;
    alt.vz_laser_raw = 0;
    alt.vz_laser     = 0;
    alt.vz_acc       = 0;
    alt.error        = 0;
    world_data.vz    = 0;
    world_data.pz    = 0;
}

/**
 * ===========================================================================
 * height_data_deal - 垂直速度与高度融合（Mahony 风格 PI 修正 + 多层保护）
 * ===========================================================================
 * 
 * 【核心思想】
 *   - 加速度计积分提供"预测值"（高频响应好，但会漂移）
 *   - 激光差分速度提供"观测值"（绝对准确，但有噪声和异常尖峰）
 *   - PI 控制器动态修正加速度积分，激光越可靠修正越强，越不可靠修正越弱
 * 
 * 【多层保护策略】
 *   第一层：对激光原始距离做单帧限幅（防止地面突变导致的高度跳变）
 *   第二层：激光差分速度低通滤波（抑制差分运算放大的高频噪声）
 *   第三层：激光质量因子平滑过渡（避免硬开关导致的速度突变）
 *   第四层：高度层面缓慢修正（防止速度层长期漂移导致高度偏离）
 *   第五层：积分限幅（防止PI修正器饱和失控）
 * 
 * 【调用频率】建议 50~200Hz
 * 【输入】 world_data.az: 世界坐标系下的垂直加速度（已去重力，m/s²）
 *         dl1b_distance_mm: 激光原始斜距（毫米）——不是对地高度！
 * 【输出】 world_data.vz: 融合后的垂直速度（m/s）
 *         world_data.pz: 融合后的对地高度（m）
 * ===========================================================================
 */
void height_data_deal(float dt)
{
    static uint8_t laser_was_invalid = 0;

    // =========================================================================
    // 第一环节：加速度计纯积分速度（预测值）
    // =========================================================================
    alt.vz_acc += world_data.az * dt;


    // =========================================================================
    // 第二环节：激光斜距 → 姿态补偿 → 对地高度
    // =========================================================================
    // 激光安装在机体下方，沿机体-Z方向发射，测的是斜距 d。
    // 当飞机倾斜时（Roll=φ, Pitch=θ），斜距 > 对地高度。
    //
    // 用旋转矩阵的 R[2][2] 投影：
    //   R[2][2] = cos(φ)×cos(θ)  （四元数直接算，无需三角函数）
    //   h_true = d × R[2][2]
    
    // 2.1 哨兵值检测：dl1b_distance_mm=8192 表示传感器无有效数据
    if (dl1b_distance_mm > 4000) {
        laser_was_invalid = 1;
        world_data.vz = alt.vz_acc;
        alt.height_acc += alt.vz_acc * dt;
        world_data.pz = alt.height_acc;
        alt.height_limited = alt.height_acc;
        return;
    }
    
    // 2.2 读取激光原始斜距，毫米→米
    float raw_height = dl1b_distance_mm / 1000.0f;
    
    // 2.3 姿态角补偿：斜距投影为对地高度
    float cos_factor = rotation_matrix[2][2];
    if (cos_factor < 0.3f)  cos_factor = 0.2f;   // 极端倾斜保护（约78°）
    if (cos_factor > 1.0f)  cos_factor = 1.0f;
    float true_height_raw = raw_height * cos_factor;

    // 2.4 激光刚恢复有效：直接跳到当前激光值，消除滞后跳变
    if (laser_was_invalid) {
        laser_was_invalid = 0;
        PT1Filter_InitWithFreq(&filter_height, HEIGHT_FILTER_FREQ, 100);
        PT1Filter_InitWithFreq(&filter_height_vz, HEIGHT_VZ_FILTER_FREQ, 100);
        alt.height = true_height_raw;
        alt.last_height = true_height_raw;
        alt.height_limited = true_height_raw;
        alt.height_acc = true_height_raw;
        world_data.pz = true_height_raw;
        return;
    }
    
    // 2.5 对补偿后的对地高度做低通滤波
    alt.height = PT1Filter_Apply(&filter_height, true_height_raw);

    // 2.3 【关键保护】单帧高度变化限幅
    // 目的：当飞机剧烈晃动时，激光测距点在地面快速移动，
    //       可能扫描到地面上的坑洼或障碍物，导致高度读数瞬间跳变。
    //       限制单帧高度变化不超过 20cm，可以砍掉这些离群值。
    // 注意：20cm/帧，在200Hz下对应最大40m/s的垂直速度，
    //       远超正常飞行范围，因此不会影响正常的快速升降。
    float dz = alt.height - alt.last_height;
    dz = LIMIT(dz, -0.2f, 0.2f);
    
    // 2.4 用限幅后的增量重建高度（保证高度曲线的连续性）
    alt.height_limited = alt.last_height + dz;
    world_data.pz = alt.height_limited;
    
    // 2.5 初始化处理（上电首帧）
    if (!flag.height_init) {
        alt.last_height = alt.height_limited;  // 初始化历史高度
        flag.height_init = 1;                   // 标记已初始化
    }


    // =========================================================================
    // 第三环节：激光差分速度计算（观测值）
    // =========================================================================
    
    // 3.1 用限幅后的高度增量计算差分速度（原始值）
    // 数学本质：v = Δh / Δt
    // 注意：这里用限幅后的 dz 而非原始高度差，已经过滤了突变
    alt.vz_laser_raw = dz / dt;
    
    // 3.2 对差分速度做低通滤波
    // 目的：差分运算会放大高频噪声（相当于高通滤波），
    //       必须紧跟一个低通滤波器来压制噪声。
    // 截止频率建议 20~30Hz（α=0.2~0.4 @100Hz）
    alt.vz_laser = PT1Filter_Apply(&filter_height_vz, alt.vz_laser_raw);
    
    // 3.3 更新历史高度（供下一帧差分使用）
    alt.last_height = alt.height_limited;


    // =========================================================================
    // 第四环节：激光质量因子计算（平滑过渡的关键）
    // =========================================================================
    // 核心思想：不搞非黑即白的硬开关，而是根据激光差分速度的波动幅度，
    //          动态计算一个 0.0~1.0 的"信任度"因子。
    //          - 1.0：激光完全可信，全强度修正
    //          - 0.0：激光完全不可信，几乎不修正
    //          - 中间值：平滑过渡，避免突变
    
    alt.laser_quality = 1.0f;           // 默认完全信任激光
    float vz_abs = fabsf(alt.vz_laser);   // 激光差分速度的绝对值
    
    const float VZ_NORM = 0.10f;   // 悬停噪声约 0.05，取 1.5 倍
    // 意义：悬停时激光差分速度噪声通常在 0.02~0.05m/s 以内
    //       低于此值认为是正常数据，完全信任激光
    const float VZ_MAX  = 1.0f;    // 从 0.2 放宽到 0.8
    // 意义：激光差分速度超过 0.2m/s 通常意味着：
    //       - 飞机剧烈晃动导致测距点在地面扫过
    //       - 激光数据出现异常尖峰
    //       - 地面不平整或遇到障碍物
    //       此时激光数据完全不可信，应几乎完全依赖加速度计
    
    if (vz_abs > VZ_NORM) {
        // 线性衰减：从 VZ_NORM 到 VZ_MAX，质量因子从 1.0 线性降到 0.0
        alt.laser_quality = 1.0f - (vz_abs - VZ_NORM) / (VZ_MAX - VZ_NORM);
        // 钳位到 [0.0, 1.0]
        if (alt.laser_quality < 0.0f) alt.laser_quality = 0.0f;
    }


    // =========================================================================
    // 第五环节：Mahony 风格 PI 速度修正（动态增益）
    // =========================================================================
    
    // 5.1 计算误差：激光速度（观测值）- 加速度积分速度（预测值）
    alt.error = alt.vz_laser - alt.vz_acc;
    
    // 5.2 积分器（用于消除加速度计零偏的长期影响）
    static float integral_vz = 0.0f;     // 静态变量，跨帧保持
    
    // 5.3 基础增益
    const float Kp_base = 0.6f;   // 原 0.2
    const float Ki_base = 0.02f;  // 原 0.02
    
    // 5.4 动态增益：基础增益 × 激光质量因子
    // 核心：激光不可靠时，自动削弱修正强度，转由加速度积分主导
    float Kp_eff = Kp_base * alt.laser_quality;
    float Ki_eff = Ki_base * alt.laser_quality;
    
//    if (Kp_eff < 0.15f) Kp_eff = 0.15f;  // 最低保持 15% 修正力度
//    if (Ki_eff < 0.01f) Ki_eff = 0.01f;
    // 5.5 积分累加（用动态 Ki）
    integral_vz += alt.error * Ki_eff * dt;
    
    // 5.6 积分限幅（防止饱和失控）
    // ±1.0 m/s² 对应约 0.1g 的加速度偏置补偿
    // 正常加速度计零偏在 0.02~0.05g 以内，1.0 已足够覆盖
    integral_vz = LIMIT(integral_vz, -2.0f, 2.0f);
    
    // 5.7 修正加速度积分速度
    alt.vz_acc += Kp_eff * alt.error + integral_vz;
    
    // 5.8 轻微泄漏（极低频，仅防止纯积分发散）
    // 0.9995@100Hz = 每秒衰减5%，仅在激光长时间失效时起保护
    alt.vz_acc *= 0.9995f;



    // =========================================================================
    // 第七环节：最终输出
    // =========================================================================
    world_data.vz = alt.vz_acc;   // 垂直速度 = 融合后的速度
    // world_data.pz 已在前面赋值（限幅后的激光高度 + 高度修正）
//printf("vz_laser=%.3f, quality=%.2f, Kp_eff=%.3f, error=%.3f, vz_acc=%.3f\r\n",
//       alt.vz_laser, laser_quality, Kp_eff, alt.error, alt.vz_acc);

}