#ifndef CODE_BATTERY_H_
#define CODE_BATTERY_H_

#include "zf_common_headfile.h"

// P07_5 ADC 分压参数
#define BAT_DIVIDER_RATIO       (11.0f)     // 1/11 分压
#define BAT_ADC_REF_VOLTAGE     (3.3f)      // ADC 参考电压(V)
#define BAT_ADC_RESOLUTION      (4095.0f)   // 12位ADC
#define BAT_ADC_CHANNEL         ADC0_CH21_P07_5
#define BAT_VOLTAGE_OFFSET      (0.1f)      // 电压校准偏移 (测量值比实际低0.1V)

// 电池电压滤波次数
#define BAT_FILTER_COUNT        (8)

// =============================================================================
// 电压 → RPM 线性映射表 (电压越低，维持相同推力所需RPM越高)
// =============================================================================
#define VOLTAGE_RPM_TABLE_SIZE  6
typedef struct {
    float voltage;  // 电池电压 (V)
    float rpm;      // 对应RPM
} VoltageRPMPoint_t;

extern const VoltageRPMPoint_t g_voltage_rpm_table[VOLTAGE_RPM_TABLE_SIZE];

void    Battery_Init(void);
float   Battery_GetVoltage(void);       // 返回实际电池电压(V)
float   Battery_VoltageToRPM(float voltage);  // 电压 → RPM 线性插值
float   Battery_GetThrottleComp(void);  // 获取油门补偿系数 (1.0 = 满电基准)

#endif /* CODE_BATTERY_H_ */
