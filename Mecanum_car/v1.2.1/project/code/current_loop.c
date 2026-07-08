/*********************************************************************************************************************
* CYT4BB Opensourec Library
* Copyright (c) 2022 SEEKFREE
*
* 文件名称          current_loop
* 功能描述          有刷直流电机电流环控制
*                   基于分流电阻 + 放大器 + ADC采样实现电流闭环控制
*                   电流环为最内层控制环，实现对电机转矩的精确控制
* 适用平台          CYT4BB
*
* 修改记录
* 日期              作者               备注
* 2026-06-30       user               first version
********************************************************************************************************************/

#include "current_loop.h"

/*****************************************************************全局变量*****************************************************************/
// 四个电机的电流环实例
current_loop_t current_loop_m1;
current_loop_t current_loop_m2;
current_loop_t current_loop_m3;
current_loop_t current_loop_m4;

// 低通滤波后的ADC值 (一阶IIR滤波)
static float adc_filtered_m1 = 0.0f;
static float adc_filtered_m2 = 0.0f;
static float adc_filtered_m3 = 0.0f;
static float adc_filtered_m4 = 0.0f;

#define ADC_FILTER_ALPHA    0.3f    // 低通滤波系数 (0~1)，越小滤波越强

/*****************************************************************内部辅助函数*****************************************************************/

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       adc_to_current
// 函数描述       将ADC原始值转换为实际电流值 (A)
// 传入参数       cl            电流环结构体指针
//               adc_value     ADC原始值 (0~4095)
// 传出参数       float         实际电流值 (A)，正值表示正向/前进电流
// 使用示例       float current = adc_to_current(&current_loop_m1, adc_value);
//-------------------------------------------------------------------------------------------------------------------
static float adc_to_current(current_loop_t *cl, uint16 adc_value)
{
    float current;

    if (cl->current_scale > 0.0f)
    {
        // 方式1：使用直接标定系数 (简化的线性转换)
        // current = (adc_value - adc_offset) * current_scale
        current = ((float)adc_value - cl->adc_offset) * cl->current_scale;
    }
    else
    {
        // 方式2：使用物理公式计算
        // 1. ADC值 → 电压: V_adc = adc_value / ADC_MAX * Vref
        // 2. 放大器输入电压 = V_adc / amp_gain
        // 3. 电流 = 放大器输入电压 / 分流电阻
        float v_adc = ((float)adc_value / (float)CURRENT_ADC_MAX) * cl->vref;
        float v_shunt = v_adc / cl->amp_gain;
        float i_total = v_shunt / cl->shunt_resistance;
        // 减去零偏得到实际电流
        float i_offset = (cl->adc_offset / (float)CURRENT_ADC_MAX * cl->vref)
                       / cl->amp_gain / cl->shunt_resistance;
        current = i_total - i_offset;
    }

    return current;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       apply_ramp
// 函数描述       对目标电流施加斜坡缓变，防止阶跃冲击
// 传入参数       target        目标电流 (A)
//               current       当前斜坡输出电流 (A)
//               step          每周期最大变化步长 (A)
// 传出参数       float         缓变后的目标值
//-------------------------------------------------------------------------------------------------------------------
static float apply_ramp(float target, float current, float step)
{
    float diff = target - current;

    if (diff > step)
    {
        return current + step;
    }
    else if (diff < -step)
    {
        return current - step;
    }
    else
    {
        return target;
    }
}

/*****************************************************************公有接口函数*****************************************************************/

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       CurrentLoop_Init
// 函数描述       初始化电流环 (使用直接标定系数方式)
// 传入参数       cl            电流环结构体指针
//               adc_ch        ADC采样通道
//               pwm_ch        PWM输出通道
//               dir_pin       方向控制GPIO引脚
//               current_scale 电流标定系数 (A/ADC)，可由实验测定
//               kp            PI比例系数
//               ki            PI积分系数
// 传出参数       void
// 使用示例       CurrentLoop_Init(&current_loop_m1, ADC0_CH00_P06_0,
//                                 TCPWM_CH09_P05_0, P05_2, 0.002f, 3.0f, 0.5f);
//-------------------------------------------------------------------------------------------------------------------
void CurrentLoop_Init(current_loop_t *cl,
                      adc_channel_enum adc_ch,
                      pwm_channel_enum pwm_ch,
                      gpio_pin_enum dir_pin,
                      float current_scale,
                      float kp, float ki)
{
    // 1. 清零结构体
    memset(cl, 0, sizeof(current_loop_t));

    // 2. 硬件配置
    cl->adc_channel  = adc_ch;
    cl->pwm_channel  = pwm_ch;
    cl->dir_pin      = dir_pin;

    // 3. 标定参数
    cl->current_scale = current_scale;
    cl->adc_offset    = 0.0f;      // 初始化后需调用 CurrentLoop_CalibrateOffset 校准

    // 4. 初始化ADC
    adc_init(adc_ch, ADC_12BIT);

    // 5. 初始化方向GPIO
    gpio_init(dir_pin, GPO, 0, GPO_PUSH_PULL);

    // 6. 初始化PWM (默认1kHz, 0%占空比)
    pwm_init(pwm_ch, 1000, 0);

    // 7. 初始化PI控制器 (电流环通常只需PI，Kd=0)
    PID_Init(&cl->pi, kp, ki, 0.0f);
    PID_SetLimit(&cl->pi, (float)CURRENT_MAX_DEFAULT * 5.0f, -(float)CURRENT_MAX_DEFAULT * 5.0f);
    PID_SetIntegralLimit(&cl->pi, 200.0f);
    PID_Enable(&cl->pi, 0);        // 默认不使能

    // 8. 默认保护参数
    cl->current_max = CURRENT_MAX_DEFAULT;
    cl->current_min = CURRENT_MIN_DEFAULT;
    cl->pwm_max     = 3000;
    cl->pwm_min     = -3000;

    // 9. 默认状态
    cl->enable = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       CurrentLoop_InitEx
// 函数描述       初始化电流环 (使用物理公式计算方式)
// 传入参数       cl            电流环结构体指针
//               adc_ch        ADC采样通道
//               pwm_ch        PWM输出通道
//               dir_pin       方向控制GPIO引脚
//               adc_offset    ADC零电流偏置值
//               amp_gain      电流放大器增益 (V/V)
//               shunt_resistance  采样电阻阻值 (Ω)
//               vref          ADC参考电压 (V)
//               kp            PI比例系数
//               ki            PI积分系数
// 使用示例       CurrentLoop_InitEx(&cl, ADC0_CH00_P06_0, TCPWM_CH09_P05_0,
//                                   P05_2, 2047.0f, 50.0f, 0.01f, 3.3f, 3.0f, 0.5f);
//-------------------------------------------------------------------------------------------------------------------
void CurrentLoop_InitEx(current_loop_t *cl,
                        adc_channel_enum adc_ch,
                        pwm_channel_enum pwm_ch,
                        gpio_pin_enum dir_pin,
                        float adc_offset,
                        float amp_gain,
                        float shunt_resistance,
                        float vref,
                        float kp, float ki)
{
    // 1. 调用基础初始化，scale=0 表示使用公式计算模式
    CurrentLoop_Init(cl, adc_ch, pwm_ch, dir_pin, 0.0f, kp, ki);

    // 2. 覆盖物理参数
    cl->adc_offset      = adc_offset;
    cl->amp_gain        = amp_gain;
    cl->shunt_resistance = shunt_resistance;
    cl->vref            = vref;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       CurrentLoop_CalibrateOffset
// 函数描述       校准ADC零偏 (电机未通电时调用，采集多组ADC值取平均作为零电流偏置)
// 传入参数       cl            电流环结构体指针
//-------------------------------------------------------------------------------------------------------------------
void CurrentLoop_CalibrateOffset(current_loop_t *cl)
{
    uint32 sum = 0;
    const int samples = 100;

    for (int i = 0; i < samples; i++)
    {
        sum += adc_convert(cl->adc_channel);
        system_delay_ms(1);
    }

    cl->adc_offset = (float)sum / (float)samples;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       CurrentLoop_SetTarget
// 函数描述       设置目标电流
// 传入参数       cl            电流环结构体指针
//               current_a     目标电流 (A)，正值=前进方向，负值=后退方向
//-------------------------------------------------------------------------------------------------------------------
void CurrentLoop_SetTarget(current_loop_t *cl, float current_a)
{
    cl->target_current = current_a;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       CurrentLoop_Enable
// 函数描述       使能/失能电流环
// 传入参数       cl            电流环结构体指针
//               enable        1=使能，0=失能（失能时PWM输出清零）
//-------------------------------------------------------------------------------------------------------------------
void CurrentLoop_Enable(current_loop_t *cl, uint8_t enable)
{
    cl->enable = enable;
    PID_Enable(&cl->pi, enable);

    if (!enable)
    {
        // 失能时关闭输出
        pwm_set_duty(cl->pwm_channel, 0);
        gpio_set_level(cl->dir_pin, 0);
        cl->output_duty = 0;
        cl->ramp_current = 0.0f;
        PID_Reset(&cl->pi);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       CurrentLoop_SetLimit
// 函数描述       设置电流限制
//-------------------------------------------------------------------------------------------------------------------
void CurrentLoop_SetLimit(current_loop_t *cl, float max_current, float min_current)
{
    cl->current_max = max_current;
    cl->current_min = min_current;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       CurrentLoop_SetPWMLimit
// 函数描述       设置PWM输出限制
//-------------------------------------------------------------------------------------------------------------------
void CurrentLoop_SetPWMLimit(current_loop_t *cl, int pwm_max, int pwm_min)
{
    cl->pwm_max = pwm_max;
    cl->pwm_min = pwm_min;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       CurrentLoop_GetCurrent
// 函数描述       获取当前实测电流值
//-------------------------------------------------------------------------------------------------------------------
float CurrentLoop_GetCurrent(current_loop_t *cl)
{
    return cl->measured_current;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       CurrentLoop_EmergencyStop
// 函数描述       紧急停止本电机
//-------------------------------------------------------------------------------------------------------------------
void CurrentLoop_EmergencyStop(current_loop_t *cl)
{
    // 关闭PID控制
    PID_Enable(&cl->pi, 0);
    PID_Reset(&cl->pi);

    // 清零输出
    pwm_set_duty(cl->pwm_channel, 0);
    gpio_set_level(cl->dir_pin, 0);

    // 复位状态
    cl->enable         = 0;
    cl->target_current = 0.0f;
    cl->ramp_current   = 0.0f;
    cl->output_duty    = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       CurrentLoop_EmergencyStopAll
// 函数描述       紧急停止全部四个电机
//-------------------------------------------------------------------------------------------------------------------
void CurrentLoop_EmergencyStopAll(void)
{
    CurrentLoop_EmergencyStop(&current_loop_m1);
    CurrentLoop_EmergencyStop(&current_loop_m2);
    CurrentLoop_EmergencyStop(&current_loop_m3);
    CurrentLoop_EmergencyStop(&current_loop_m4);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       CurrentLoop_Update
// 函数描述       电流环主更新函数 (需在10ms定时中断中调用)
// 传入参数       cl            电流环结构体指针
// 核心算法:
//   1. ADC采样 → 电流实测值 (含低通滤波)
//   2. 目标电流斜坡缓变
//   3. PI控制器计算 → PWM占空比
//   4. 输出PWM + 方向控制
//   5. 过流保护检测
//-------------------------------------------------------------------------------------------------------------------
void CurrentLoop_Update(current_loop_t *cl)
{
    if (!cl->enable) return;

    // ===== 第一步：ADC采样与电流计算 =====

    // 均值滤波采样
    uint16 adc_raw = adc_mean_filter_convert(cl->adc_channel, CURRENT_ADC_FILTER_COUNT);
    cl->adc_raw = (float)adc_raw;

    // 一阶低通滤波
    float *adc_filtered = NULL;
    if      (cl == &current_loop_m1) adc_filtered = &adc_filtered_m1;
    else if (cl == &current_loop_m2) adc_filtered = &adc_filtered_m2;
    else if (cl == &current_loop_m3) adc_filtered = &adc_filtered_m3;
    else if (cl == &current_loop_m4) adc_filtered = &adc_filtered_m4;

    if (adc_filtered != NULL)
    {
        *adc_filtered = ADC_FILTER_ALPHA * cl->adc_raw
                      + (1.0f - ADC_FILTER_ALPHA) * (*adc_filtered);

        // 将滤波后的ADC值转换为电流
        cl->measured_current = adc_to_current(cl, (uint16)(*adc_filtered));
    }
    else
    {
        cl->measured_current = adc_to_current(cl, adc_raw);
    }

    // ===== 第二步：过流保护检测 =====
    if (cl->measured_current > cl->current_max * 1.2f ||
        cl->measured_current < cl->current_min * 1.2f)
    {
        // 过流120%：立即关闭输出
        CurrentLoop_EmergencyStop(cl);
        return;
    }

    // ===== 第三步：目标电流斜坡缓变 =====
    cl->ramp_current = apply_ramp(cl->target_current, cl->ramp_current, CURRENT_RAMP_STEP);

    // 目标电流限幅
    if (cl->ramp_current > cl->current_max)  cl->ramp_current = cl->current_max;
    if (cl->ramp_current < cl->current_min)  cl->ramp_current = cl->current_min;

    // ===== 第四步：PI控制器计算 =====

    // 设定PI目标值
    PID_SetTarget(&cl->pi, cl->ramp_current);

    // 计算PI输出
    float pi_output = PID_Calculate(&cl->pi, cl->measured_current);

    // ===== 第五步：PWM输出与方向控制 =====

    // 根据PI输出符号决定方向和PWM占空比
    if (pi_output >= 0)
    {
        // 正向电流：设置方向引脚为高电平
        gpio_set_level(cl->dir_pin, 1);

        // PI输出映射到PWM占空比 (0~10000)
        // 这里假设PI输出近似对应PWM值，实际需根据系统调整
        int duty = (int)pi_output;

        // PWM限幅
        if (duty > cl->pwm_max)  duty = cl->pwm_max;
        if (duty < 0)            duty = 0;

        cl->output_duty = duty;
        pwm_set_duty(cl->pwm_channel, (uint32)duty);
    }
    else
    {
        // 反向电流：设置方向引脚为低电平
        gpio_set_level(cl->dir_pin, 0);

        // 取绝对值映射到PWM
        int duty = (int)(-pi_output);

        // PWM限幅
        if (duty > cl->pwm_max)  duty = cl->pwm_max;
        if (duty < 0)            duty = 0;

        cl->output_duty = duty;
        pwm_set_duty(cl->pwm_channel, (uint32)duty);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       CurrentLoop_UpdateAll
// 函数描述       批量更新全部四个电机电流环 (推荐在10ms定时中断中调用)
//-------------------------------------------------------------------------------------------------------------------
void CurrentLoop_UpdateAll(void)
{
    CurrentLoop_Update(&current_loop_m1);
    CurrentLoop_Update(&current_loop_m2);
    CurrentLoop_Update(&current_loop_m3);
    CurrentLoop_Update(&current_loop_m4);
}
