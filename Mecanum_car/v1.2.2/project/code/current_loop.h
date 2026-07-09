#ifndef CODE_CURRENT_LOOP_H_
#define CODE_CURRENT_LOOP_H_

#include "zf_common_headfile.h"
#include "pid.h"

/*------------------------ 电流环配置宏定义 ------------------------*/
#define CURRENT_LOOP_SAMPLE_TIME_MS   10.0f       // 采样周期 (ms)，与10ms定时中断同步
#define CURRENT_LOOP_SAMPLE_TIME_S    (CURRENT_LOOP_SAMPLE_TIME_MS / 1000.0f)

#define CURRENT_ADC_FILTER_COUNT      8           // ADC均值滤波采样次数
#define CURRENT_ADC_MAX               4095        // 12位ADC最大值

#define CURRENT_RAMP_STEP            0.02f        // 电流斜坡步长 (A/周期)，防止阶跃冲击
#define CURRENT_MAX_DEFAULT          2.0f         // 默认最大电流限制 (A)
#define CURRENT_MIN_DEFAULT         -2.0f         // 默认最小电流限制 (A)

/*------------------------ 电流环结构体定义 -----------------------*/
typedef struct
{
    // ---- 硬件配置 ----
    adc_channel_enum    adc_channel;        // 电流采样ADC通道
    pwm_channel_enum    pwm_channel;        // 电机驱动PWM通道
    gpio_pin_enum       dir_pin;            // 电机方向控制GPIO

    // ---- 电流采样标定参数 ----
    float               adc_offset;         // ADC零电流偏置 (无电流时的ADC值)
    float               amp_gain;           // 电流放大器增益 (V/V)
    float               shunt_resistance;   // 采样电阻阻值 (Ω)
    float               vref;               // ADC参考电压 (V)

    // 直接标定系数 (与上面二选一，优先使用)
    float               current_scale;      // 电流比例系数 (A/ADC)，若>0则忽略上面的公式计算

    // ---- 运行状态 ----
    float               target_current;     // 目标电流 (A)，正=前进方向
    float               ramp_current;       // 斜坡缓变后的目标电流 (A)
    float               measured_current;   // 实测电流 (A)，经滤波
    float               adc_raw;            // ADC原始值

    int                 output_duty;        // 输出PWM占空比 (0~10000)

    // ---- 保护参数 ----
    float               current_max;        // 电流上限 (A)
    float               current_min;        // 电流下限 (A)
    int                 pwm_max;            // PWM最大输出 (默认3000)
    int                 pwm_min;            // PWM最小输出 (默认-3000)

    // ---- PI控制器 ----
    PID                 pi;                 // 电流PI控制器 (仅用Kp和Ki)

    // ---- 使能与状态 ----
    uint8_t             enable;             // 电流环使能
    uint8_t             motor_index;        // 电机编号 (调试用)

} current_loop_t;

/*------------------------ 外部变量声明 -----------------------*/
extern current_loop_t current_loop_m1;   // 电机1电流环
extern current_loop_t current_loop_m2;   // 电机2电流环
extern current_loop_t current_loop_m3;   // 电机3电流环
extern current_loop_t current_loop_m4;   // 电机4电流环

/*------------------------ 公有接口函数 -----------------------*/

// 初始化电流环
void CurrentLoop_Init(current_loop_t *cl,
                      adc_channel_enum adc_ch,
                      pwm_channel_enum pwm_ch,
                      gpio_pin_enum dir_pin,
                      float current_scale,
                      float kp, float ki);

// 简化初始化（使用公式计算标定系数）
void CurrentLoop_InitEx(current_loop_t *cl,
                        adc_channel_enum adc_ch,
                        pwm_channel_enum pwm_ch,
                        gpio_pin_enum dir_pin,
                        float adc_offset,
                        float amp_gain,
                        float shunt_resistance,
                        float vref,
                        float kp, float ki);

// 设置目标电流
void CurrentLoop_SetTarget(current_loop_t *cl, float current_a);

// 电流环更新函数 (在10ms定时中断中调用)
void CurrentLoop_Update(current_loop_t *cl);

// 全部四个电机电流环批量更新
void CurrentLoop_UpdateAll(void);

// 使能/失能电流环
void CurrentLoop_Enable(current_loop_t *cl, uint8_t enable);

// 设置电流限制
void CurrentLoop_SetLimit(current_loop_t *cl, float max_current, float min_current);

// 设置PWM限制
void CurrentLoop_SetPWMLimit(current_loop_t *cl, int pwm_max, int pwm_min);

// 紧急停止 (立即关闭PWM输出)
void CurrentLoop_EmergencyStop(current_loop_t *cl);
void CurrentLoop_EmergencyStopAll(void);

// 获取当前实测电流
float CurrentLoop_GetCurrent(current_loop_t *cl);

// 校准ADC零偏 (电机未通电时调用)
void CurrentLoop_CalibrateOffset(current_loop_t *cl);

#endif /* CODE_CURRENT_LOOP_H_ */
