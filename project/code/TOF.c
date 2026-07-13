#include "zf_common_headfile.h"
#include "VL53L5CX.h"

// 4x4 模式下，中心 4 个 Zone 索引（边缘 Zone 可能扫到障碍物）
#define CENTER_ZONE_0    5
#define CENTER_ZONE_1    6
#define CENTER_ZONE_2    9
#define CENTER_ZONE_3   10

void TOF_init(void)
{
    // VL53L5CX 初始化（替代原 DL1B，两者 I2C 地址均为 0x29 冲突）
    uint8_t ret = vl53l5cx_init();
    if (0 == ret)
    {
        vl53l5cx_start_ranging();
        printf("  [TOF] VL53L5CX started at 30Hz\r\n");
    }
    else
    {
        printf("  [TOF] VL53L5CX init FAILED!\r\n");
    }
    
    // 状态初始化
    alt.height       = 0;
    alt.last_height  = 0;
    alt.vz_laser_raw = 0;
    alt.vz_laser     = 0;
    alt.vz_acc       = 0;
    alt.error        = 0;
    alt.target_height= 0;
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
 *   - VL53L5CX 差分速度提供"观测值"（绝对准确，但有噪声和异常尖峰）
 *   - 使用中心 4 区（Zone 5/6/9/10）平均值作为高度源，比单 Zone 更鲁棒
 *   - PI 控制器动态修正加速度积分，激光越可靠修正越强，越不可靠修正越弱
 * 
 * 【多层保护策略】
 *   第一层：中心 4 区无效时回退到 Zone 0
 *   第二层：激光差分速度低通滤波（抑制差分运算放大的高频噪声）
 *   第三层：激光质量因子平滑过渡（避免硬开关导致的速度突变）
 *   第四层：高度层面缓慢修正（防止速度层长期漂移导致高度偏离）
 *   第五层：积分限幅（防止PI修正器饱和失控）
 * 
 * 【调用频率】建议 50Hz（与 PIT_CH2 一致）
 * 【输入】 world_data.az: 世界坐标系下的垂直加速度（已去重力，m/s²）
 *         vl53l5cx_zone_result / vl53l5cx_distance_mm: 多区测距结果
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
    // 第二环节：VL53L5CX 多区数据 → 中心 4 区平均 → 对地高度
    // =========================================================================
    // VL53L5CX 朝下安装，4×4 模式共 16 个 Zone：
    //     0   1   2   3
    //     4   5   6   7
    //     8   9  10  11
    //    12  13  14  15
    // 中心 4 区 (5,6,9,10) 最不易受地面边缘或障碍物影响

    // 2.1 从中心 4 区计算平均高度
    float raw_height_sum = 0.0f;
    uint8  valid_count   = 0;
    const uint8 center_idx[4] = {CENTER_ZONE_0, CENTER_ZONE_1,
                                 CENTER_ZONE_2, CENTER_ZONE_3};

    for (uint8 i = 0; i < 4; i++)
    {
        uint8  idx = center_idx[i];
        uint8  st  = vl53l5cx_zone_result.status[idx];
        uint16 d   = vl53l5cx_zone_result.distance_mm[idx];

        // 有效性检查（ST 定义：status=5 或 9 为有效测距）
        if ((5 == st || 9 == st) && d > 0 && d < 8000)
        {
            raw_height_sum += (float)d / 1000.0f;   // mm → m
            valid_count++;
        }
    }

    // 2.2 中心区有效数 ≥ 1 则取最高值（最远距离），否则回退到 Zone 0
    float raw_height;
    if (valid_count > 0)
    {
        // 取最高值（最远距离），使高度定在地面最高点
        float max_h = 0.0f;
        for (uint8 i = 0; i < 4; i++)
        {
            uint8  idx = center_idx[i];
            uint8  st  = vl53l5cx_zone_result.status[idx];
            uint16 d   = vl53l5cx_zone_result.distance_mm[idx];
            if ((5 == st || 9 == st) && d > 0 && d < 8000)
            {
                float h = (float)d / 1000.0f;
                if (h > max_h) max_h = h;
            }
        }
        raw_height = max_h;
    }
    else
    {
        // 回退到 Zone 0（首区）
        float d0 = (float)vl53l5cx_distance_mm / 1000.0f;
        if (d0 > 0.001f && d0 < 8.0f)
            raw_height = d0;
        else
            raw_height = 8.192f;   // 完全无效时的哨兵值
    }

    // 2.3 哨兵值检测：> 4m 或 ≤ 0 表示传感器无有效数据
    if (raw_height > 4.0f || raw_height <= 0.001f)
    {
        laser_was_invalid = 1;
        world_data.vz = alt.vz_acc;
        alt.height_acc += alt.vz_acc * dt;
        world_data.pz = alt.height_acc;
        alt.height_limited = alt.height_acc;
        return;
    }

    // 2.4 姿态角补偿：斜距投影为对地高度
    // VL53L5CX 安装在机体下方，测的是斜距。当飞机倾斜时（Roll=φ, Pitch=θ），
    // 斜距 > 对地高度。用旋转矩阵 R[2][2] = cos(φ)×cos(θ) 投影。
    static PT1Filter_t filter_cos;
    static uint8_t cos_filter_init = 0;
    if(!cos_filter_init)
    {
        PT1Filter_InitWithFreq(&filter_cos, 30.0f, 25);
        cos_filter_init = 1;
    }
    float cos_factor = PT1Filter_Apply(&filter_cos, rotation_matrix[2][2]);
    if (cos_factor < 0.3f)  cos_factor = 0.3f;   // 极端倾斜保护（约 72°）
    if (cos_factor > 1.0f)  cos_factor = 1.0f;
    float true_height_raw = raw_height * cos_factor;

    // 2.5 激光刚恢复有效：直接跳到当前激光值，消除滞后跳变
    if (laser_was_invalid)
    {
        laser_was_invalid = 0;
        PT1Filter_InitWithFreq(&filter_height, HEIGHT_FILTER_FREQ, 25);
        PT1Filter_InitWithFreq(&filter_height_vz, HEIGHT_VZ_FILTER_FREQ, 25);
        alt.height = true_height_raw;
        alt.last_height = true_height_raw;
        alt.height_limited = true_height_raw;
        alt.height_acc = true_height_raw;
        world_data.pz = true_height_raw;
        return;
    }

    // 2.6 对补偿后的对地高度做低通滤波
    alt.height = PT1Filter_Apply(&filter_height, true_height_raw);

    // 2.7 【关键保护】单帧高度变化限幅
    // 目的：当飞机剧烈晃动时，激光测距点在地面快速移动，
    //       可能扫描到坑洼或障碍物，导致高度读数瞬间跳变。
    //       限制单帧高度变化不超过 15cm，砍掉离群值。
    float dz = alt.height - alt.last_height;
    dz = LIMIT(dz, -0.15f, 0.15f);

    // 2.8 用限幅后的增量重建高度（保证高度曲线的连续性）
    alt.height_limited = alt.last_height + dz;
    world_data.pz = alt.height_limited;

    // 2.9 初始化处理（上电首帧）
    if (!flag.height_init)
    {
        alt.last_height = alt.height_limited;
        flag.height_init = 1;
    }


    // =========================================================================
    // 第三环节：激光差分速度计算（观测值）
    // =========================================================================

    // 3.1 用限幅后的高度增量计算差分速度
    alt.vz_laser_raw = dz / dt;

    // 3.2 对差分速度做低通滤波（差分放大高频噪声，必须低通压制）
    alt.vz_laser = PT1Filter_Apply(&filter_height_vz, alt.vz_laser_raw);

    // 3.3 更新历史高度（供下一帧差分使用）
    alt.last_height = alt.height_limited;


    // =========================================================================
    // 第四环节：激光质量因子计算（平滑过渡）
    // =========================================================================
    // 根据激光差分速度的波动幅度，动态计算 0.0~1.0 的"信任度"因子
    //   - 1.0：激光完全可信，全强度修正
    //   - 0.0：激光完全不可信，几乎不修正
    //   - 中间值：平滑过渡，避免突变

    alt.laser_quality = 1.0f;
    float vz_abs = fabsf(alt.vz_laser);

    const float VZ_NORM = 0.30f;   // 悬停噪声约 0.05，放宽到 0.30
    const float VZ_MAX  = 1.00f;   // 超过此值视为完全不可信

    if (vz_abs > VZ_NORM)
    {
        alt.laser_quality = 1.0f - (vz_abs - VZ_NORM) / (VZ_MAX - VZ_NORM);
        if (alt.laser_quality < 0.0f) alt.laser_quality = 0.0f;
    }


    // =========================================================================
    // 第五环节：Mahony 风格 PI 速度修正（动态增益）
    // =========================================================================

    // 5.1 误差：激光速度（观测值）- 加速度积分速度（预测值）
    alt.error = alt.vz_laser - alt.vz_acc;

    // 5.2 积分器（用于消除加速度计零偏的长期影响）
    static float integral_vz = 0.0f;

    // 5.3 基础增益
    const float Kp_base = 0.6f;
    const float Ki_base = 0.02f;

    // 5.4 动态增益 = 基础增益 × 激光质量因子
    float Kp_eff = Kp_base * alt.laser_quality;
    float Ki_eff = Ki_base * alt.laser_quality;

    // 5.5 积分累加（用动态 Ki）
    integral_vz += alt.error * Ki_eff * dt;

    // 5.6 积分限幅（防止饱和失控）
    integral_vz = LIMIT(integral_vz, -2.0f, 2.0f);

    // 5.7 修正加速度积分速度
    alt.vz_acc += Kp_eff * alt.error + integral_vz;

    // 5.8 轻微泄漏（极低频，防止纯积分发散）
    alt.vz_acc *= 0.9995f;


    // =========================================================================
    // 第七环节：最终输出
    // =========================================================================
    world_data.vz = alt.vz_acc;   // 垂直速度 = 融合后的速度
}
