#include "battery.h"
#include "filter.h"     // PT1Filter

// ADC 初始化标志
static uint8_t battery_adc_inited = 0;

// 油门补偿PT1滤波
static PT1Filter_t battery_comp_filter;
static uint8_t battery_comp_inited = 0;

// =============================================================================
// 电压 → RPM 线性映射表
//   电压(V)    RPM
//   16.8      5150
//   16.3      5200
//   15.8      5250
//   15.3      5550
//   15.0      5650
//   14.8      5800
// =============================================================================
const VoltageRPMPoint_t g_voltage_rpm_table[VOLTAGE_RPM_TABLE_SIZE] = {
    {16.8f, 5150.0f},
    {16.3f, 5200.0f},
    {15.8f, 5250.0f},
    {15.3f, 5550.0f},
    {15.0f, 5650.0f},
    {14.8f, 5800.0f},
};

//-------------------------------------------------------------------
// 函数:    Battery_Init
// 功能:    初始化 P07_5(ADC0_CH21) 为 12位 ADC
// 说明:    只需调用一次，重复调用不会重复初始化
//-------------------------------------------------------------------
void Battery_Init(void)
{
    if(battery_adc_inited) return;

    adc_init(BAT_ADC_CHANNEL, ADC_12BIT);
    battery_adc_inited = 1;
}

//-------------------------------------------------------------------
// 函数:    Battery_GetVoltage
// 功能:    读取电池电压
// 返回:    实际电池电压(V)
// 原理:    ADC原始值 → 引脚电压(V) → 分压比还原 → 电池电压(V)
//-------------------------------------------------------------------
float Battery_GetVoltage(void)
{
    // 如果未初始化，自动初始化
    if(!battery_adc_inited)
        Battery_Init();

    // 均值滤波读取 ADC 原始值
    uint16_t adc_raw = adc_mean_filter_convert(BAT_ADC_CHANNEL, BAT_FILTER_COUNT);

    // 计算实际电压
    // 引脚电压 = adc_raw / 4095 * 3.3
    // 电池电压 = 引脚电压 * 11
    float pin_voltage = (float)adc_raw / BAT_ADC_RESOLUTION * BAT_ADC_REF_VOLTAGE;
    float battery_voltage = pin_voltage * BAT_DIVIDER_RATIO + BAT_VOLTAGE_OFFSET;

    return battery_voltage;
}

//-------------------------------------------------------------------
// 函数:    Battery_VoltageToRPM
// 功能:    电压 → RPM 线性插值
// 参数:    voltage - 电池电压 (V)
// 返回:    对应RPM (越界时返回边界值)
// 原理:    查表线性插值，低于14.8V/高于16.8V时钳位
//-------------------------------------------------------------------
float Battery_VoltageToRPM(float voltage)
{
    // 高于最高电压 → 返回表头值
    if (voltage >= g_voltage_rpm_table[0].voltage)
        return g_voltage_rpm_table[0].rpm;

    // 低于最低电压 → 返回表尾值
    if (voltage <= g_voltage_rpm_table[VOLTAGE_RPM_TABLE_SIZE - 1].voltage)
        return g_voltage_rpm_table[VOLTAGE_RPM_TABLE_SIZE - 1].rpm;

    // 线性插值: 找到电压所在区间
    for (uint8_t i = 0; i < VOLTAGE_RPM_TABLE_SIZE - 1; i++)
    {
        if (voltage <= g_voltage_rpm_table[i].voltage &&
            voltage >= g_voltage_rpm_table[i + 1].voltage)
        {
            float v_high = g_voltage_rpm_table[i].voltage;
            float v_low  = g_voltage_rpm_table[i + 1].voltage;
            float t = (voltage - v_low) / (v_high - v_low);
            return g_voltage_rpm_table[i + 1].rpm +
                   t * (g_voltage_rpm_table[i].rpm - g_voltage_rpm_table[i + 1].rpm);
        }
    }

    return g_voltage_rpm_table[VOLTAGE_RPM_TABLE_SIZE - 1].rpm;
}

//-------------------------------------------------------------------
// 函数:    Battery_GetThrottleComp
// 功能:    获取油门补偿系数
// 返回:    补偿系数 (满电16.8V时=1.0，低电时>1.0)
// 原理:    对电压做PT1平滑后查表 → RPM(当前电压)/RPM(16.8V基准)
//          0.2Hz低通滤除电压噪声，补偿值稳定不抖动
// 示例:    14.8V时 5800/5150=1.126，油门需提升12.6%
//-------------------------------------------------------------------
float Battery_GetThrottleComp(void)
{
    float raw = Battery_GetVoltage();

    // 首次初始化PT1滤波器 (fc=0.2Hz, 控制周期200Hz)
    if (!battery_comp_inited)
    {
        PT1Filter_InitWithFreq(&battery_comp_filter, 0.2f, 200);
        battery_comp_inited = 1;
        return raw / g_voltage_rpm_table[0].voltage;  // 简单的比例近似，待下次调用稳定
    }

    // 对电压做PT1滤波后再查表，避免噪声导致补偿抖动
    float voltage = PT1Filter_Apply(&battery_comp_filter, raw);
    float rpm = Battery_VoltageToRPM(voltage);
    float comp = rpm / g_voltage_rpm_table[0].rpm;   // ÷5150

    return comp;
}
