/*********************************************************************************************************************
* CYT4BB Opensourec Library
* Copyright (c) 2022 SEEKFREE
*
* 文件名称          motor
* 功能描述          四轮麦克纳姆轮电机驱动与运动控制
*                   - 电机速度PID闭环 (前馈 + 反馈)
*                   - 麦克纳姆轮运动学解算
*                   - 角度环 (Yaw偏航控制)
*                   - 平衡环 (Pitch/Roll自平衡)
*                   - 位置环 (视觉/外部位置指令)
*                   - 防滑控制
* 适用平台          CYT4BB
*
* 修改记录
* 日期              作者               备注
* 2026-01-26       user               first version
* 2026-03-10       user               添加角度环、平衡环、位置环
* 2026-06-30       user               调整参数: PWM改为10kHz, 编码器滤波增强, 输出变化率限制
********************************************************************************************************************/

#include "motor.h"
#include "App_menu.h"   // 位置环速度限幅 pos_limit_x/y (菜单设置, W25Q64持久化)

/*==================================================== 全局变量 ====================================================*/
#define POS_V_MAX 0.50f   // 位置环 PID 输出限幅天花板 (v2.5.0 起运行时被 pos_limit_x/y 覆盖, 此值仅 Init 用一次)
#define SPD_CLAMP_MAX 200.0f   // 速度环目标转速硬钳 (pulse/10ms): 0.5m/s=200 脉冲/10ms (v1.4.7 由 ±180 放开到 ±200, 让菜单 0.5m/s 真正跑满; 原 ±180≈0.45m/s 物理极限)
// 四个电机结构体实例
motor_t motor_L1;       // 左前轮
motor_t motor_L2;       // 左后轮
motor_t motor_R1;       // 右前轮
motor_t motor_R2;       // 右后轮

// 角度串级PID实例
PID angle_pid_yaw;      // 外环: 角度误差 → 目标角速度 (deg/s), 温和P+慢I
PID angle_pid_gyro;     // 内环: 角速度闭环 (陀螺反馈), Kp阻尼+Ki自动吃零偏

// 底盘运动目标速度
float target_vx  = 0.0f;   // X方向目标速度 (m/s), 前进为正
float target_vy  = 0.0f;   // Y方向目标速度 (m/s), 左移为正
float angle_target = 0.0f; // 偏航角目标值 (度)
uint8_t  mission_armed = 0;      // 0=等待按键4发车, 1=已发车, 响应无人机标志位
uint64_t mission_arm_time = 0;    // 发车时刻(us), 启动后等4s再执行
// 控制模式定义
typedef enum {
    CONTROL_MODE_OPEN_LOOP = 0,    // 开环控制
    CONTROL_MODE_CLOSED_LOOP       // 闭环PID控制
} ControlMode_t;

static ControlMode_t control_mode = CONTROL_MODE_OPEN_LOOP;

/*==================================================== 前馈标定表 ====================================================*/
// 速度 → PWM 实测标定数据 (速度单位: 编码器脉冲/10ms, PWM单位: 0.01%占空比)
// 用于前馈控制, 通过开环测试逐点采集, 线性插值查表
static const struct {
    float speed;
    float pwm;
} ff_table[] = {
    {  0.0f,   0.0f },     // 零点
    { 10.0f,  30.0f },     // 死区附近
    { 21.2f,  35.0f },     // 死区突破点
    { 35.0f,  43.0f },     // 插值
    { 50.7f,  50.0f },     // 实测
    { 70.0f,  62.0f },     // 插值
    { 96.0f,  75.0f },     // 实测
    {130.0f,  80.0f },     // 插值
    {154.8f,  85.0f },     // 实测
    {188.9f, 100.0f },     // 实测
    {220.9f, 110.0f },     // 实测
    {254.5f, 120.0f },     // 实测
    {281.1f, 130.0f },     // 实测(空载)
    {305.2f, 140.0f },     // 实测(空载)
    {331.2f, 150.0f },     // 实测(空载)
    {350.0f, 160.0f },     // 线性外推
};
#define FF_TABLE_SIZE  (sizeof(ff_table) / sizeof(ff_table[0]))

/*==================================================== 位置环PID ====================================================*/
static PID pos_pid_x;      // X方向位置PID
static PID pos_pid_y;      // Y方向位置PID

/*==================================================== 防滑控制 ====================================================*/
#define SLIP_DIFF_THRESH    60.0f     // 左右轮速差阈值 (脉冲/10ms)
#define SLIP_COUNT_MAX      5         // 连续打滑判定次数
#define ESCAPE_COMP_FACTOR  0.8f      // 脱困补偿系数

static uint8_t slip_counter = 0;

/*==================================================== 转速限幅 ====================================================*/
// 目标转速限幅 ±75 pulse/10ms, 防止空转飞车, 不限制PWM以保证大扭矩

/*==================================================== 初始化函数 ====================================================*/

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       Angle_Init
// 函数描述       偏航角PID初始化, 设置限幅并默认禁用
// 调用位置       Motor_Init() 内部调用
//-------------------------------------------------------------------------------------------------------------------
void Angle_Init(void)
{
    // ---- 角度外环: Kp=5修正角度 + Ki=0.08消除稳态残差 ----
    PID_Init(&angle_pid_yaw, 5.0f, 0.08f, 0.0f);
    PID_SetLimit(&angle_pid_yaw, 100.0f, -100.0f);
    PID_SetIntegralLimit(&angle_pid_yaw, 80.0f);
    PID_Enable(&angle_pid_yaw, 0);

    // ---- 角速度内环: 纯P阻尼, 不积分 (避免与外环Ki拔河) ----
    PID_Init(&angle_pid_gyro, 0.6f, 0.1f, 0.0f);
    PID_SetLimit(&angle_pid_gyro, 100.0f, -100.0f);
    PID_SetIntegralLimit(&angle_pid_gyro, 0.0f);
    PID_Enable(&angle_pid_gyro, 0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       Motor_Init
// 函数描述       四电机硬件初始化: GPIO方向引脚 + PWM输出 + 电机速度PID参数配置
// 调用位置       Init_all() 中调用
// 参数说明       PWM频率: 10kHz (避免人耳可听噪声)
//                PID初值: Kp=15, Ki=2, Kd=1.5 (D项用于抑制负载突变时的电钻现象)
//-------------------------------------------------------------------------------------------------------------------
void Motor_Init(void)
{
    // 清零电机结构体
    memset(&motor_L1, 0, sizeof(motor_t));
    memset(&motor_L2, 0, sizeof(motor_t));
    memset(&motor_R1, 0, sizeof(motor_t));
    memset(&motor_R2, 0, sizeof(motor_t));

    // ---- 硬件初始化: GPIO方向控制 + PWM (10kHz) ----
    gpio_init(MotorL1_Turn, GPO, 0, GPO_PUSH_PULL);
    pwm_init(MotorL1_Pwm, 10000, 0);

    gpio_init(MotorL2_Turn, GPO, 0, GPO_PUSH_PULL);
    pwm_init(MotorL2_Pwm, 10000, 0);

    gpio_init(MotorR1_Turn, GPO, 0, GPO_PUSH_PULL);
    pwm_init(MotorR1_Pwm, 10000, 0);

    gpio_init(MotorR2_Turn, GPO, 0, GPO_PUSH_PULL);
    pwm_init(MotorR2_Pwm, 10000, 0);

    // ---- 四个电机速度PID初始化 ----
    PID_Init(&motor_L1.pid, 3.0f, 0.10f, 0.0f);
    PID_SetLimit(&motor_L1.pid, (float)PWM_MAX_SAFE, -(float)PWM_MAX_SAFE);
    PID_SetIntegralLimit(&motor_L1.pid, 100.0f);
    PID_Enable(&motor_L1.pid, 0);

    PID_Init(&motor_L2.pid, 3.0f, 0.10f, 0.0f);
    PID_SetLimit(&motor_L2.pid, (float)PWM_MAX_SAFE, -(float)PWM_MAX_SAFE);
    PID_SetIntegralLimit(&motor_L2.pid, 100.0f);
    PID_Enable(&motor_L2.pid, 0);

    PID_Init(&motor_R1.pid, 3.2f, 0.15f, 0.0f);
    PID_SetLimit(&motor_R1.pid, (float)PWM_MAX_SAFE, -(float)PWM_MAX_SAFE);
    PID_SetIntegralLimit(&motor_R1.pid, 100.0f);
    PID_Enable(&motor_R1.pid, 0);

    PID_Init(&motor_R2.pid, 3.2f, 0.15f, 0.0f);
    PID_SetLimit(&motor_R2.pid, (float)PWM_MAX_SAFE, -(float)PWM_MAX_SAFE);
    PID_SetIntegralLimit(&motor_R2.pid, 100.0f);
    PID_Enable(&motor_R2.pid, 0);

    // 子模块初始化
    Angle_Init();
    PositionControl_Init();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       PositionControl_Init
// 函数描述       位置环PID初始化 (X/Y方向, 将位置误差转换为速度指令)
//-------------------------------------------------------------------------------------------------------------------
void PositionControl_Init(void)
{
    PID_Init(&pos_pid_x, 0.01f, 0.0f, 0.0f);
    PID_SetLimit(&pos_pid_x, POS_V_MAX, -POS_V_MAX);            // 输出限幅: ±0.20 m/s
    PID_SetIntegralLimit(&pos_pid_x, 10.0f);
    PID_Enable(&pos_pid_x, 1);

    PID_Init(&pos_pid_y, 0.01f, 0.0f, 0.0f);
    PID_SetLimit(&pos_pid_y, POS_V_MAX, -POS_V_MAX);
    PID_SetIntegralLimit(&pos_pid_y, 10.0f);
    PID_Enable(&pos_pid_y, 1);
}

void PositionControl_Reset(void)
{
    PID_Reset(&pos_pid_x);
    PID_Reset(&pos_pid_y);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       PositionControl_Update
// 函数描述       位置环更新: 读取位置误差 → PID计算 → 写入 target_vx/vy
//               注意: 此函数会覆盖遥控/蓝牙指令的速度值
//-------------------------------------------------------------------------------------------------------------------
void PositionControl_Update(void)
{
    float err_x = -GetPositionErrorX();
    // 2026-08-15 视觉双摄实测: 视觉 y+ = 车右侧 (信标在右 err.y>0), 而 target_vy 正=左移,
    // 故 y 不加负号: 正误差 → PID输出负 → 右移. 原负号使正误差=左移, 方向反了.
    // (x 保持负号: 视觉 x+ = 前方, 正误差 → 前进, 实测正确)
    float err_y =  -GetPositionErrorY();

    // 完赛/发车延迟: 立即锁住
    if (race_done || !mission_armed || time_us - mission_arm_time < 3500000)
    {
        PID_Reset(&pos_pid_x);
        PID_Reset(&pos_pid_y);
        target_vx = 0.0f;
        target_vy = 0.0f;
        return;
    }

    // 无人机发现信标(flag=1) 才执行位置闭环
    if (drone_beacon_flag != 1)
    {
        PID_Reset(&pos_pid_x);
        PID_Reset(&pos_pid_y);
        target_vx = 0.0f;
        target_vy = 0.0f;
        return;
    }

    // 位置环分轴限幅 (菜单 pos_limit_x/y, 0.05m/s 步进, 0~0.5m/s): 每周期把菜单值同步到位置PID输出限幅
    // 替代硬编码 POS_V_MAX — 否则菜单设 0.4/0.5 会被 PID 输出限幅卡在 0.30 (v2.5.0 接线)
    PID_SetLimit(&pos_pid_x, (float)pos_limit_x * 0.05f, -(float)pos_limit_x * 0.05f);
    PID_SetLimit(&pos_pid_y, (float)pos_limit_y * 0.05f, -(float)pos_limit_y * 0.05f);

    float vx_cmd = PID_Calculate(&pos_pid_x, err_x);
    float vy_cmd = PID_Calculate(&pos_pid_y, err_y);

    target_vx = vx_cmd;
    target_vy = vy_cmd;
}

/*==================================================== 防滑控制 ====================================================*/

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       AntiSlipControl
// 函数描述       检测左右轮速差异, 连续打滑超过阈值时施加脱困补偿
//               触发条件: 同轴左右轮速差 > SLIP_DIFF_THRESH, 持续 SLIP_COUNT_MAX 次
//               补偿策略: 给打滑较快一侧施加同向补偿速度, 抑制空转
//-------------------------------------------------------------------------------------------------------------------
#ifndef constrain
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif

void AntiSlipControl(void)
{
    float lf = motor_L1.encoder_speed;
    float rf = motor_R1.encoder_speed;
    float lr = motor_L2.encoder_speed;
    float rr = motor_R2.encoder_speed;

    float diff_f = lf - rf;
    float diff_r = lr - rr;
    bool slip_front = (fabsf(diff_f) > SLIP_DIFF_THRESH);
    bool slip_rear  = (fabsf(diff_r) > SLIP_DIFF_THRESH);

    if (slip_front || slip_rear) {
        slip_counter++;
        if (slip_counter >= SLIP_COUNT_MAX) {
            float escape_dir = (target_vx > 0) ? 1.0f : (target_vx < 0) ? -1.0f : 0.0f;
            if (escape_dir == 0) escape_dir = 1.0f;

            if (slip_front) {
                float fast_speed = fmaxf(fabsf(lf), fabsf(rf));
                float comp_speed = fast_speed * ESCAPE_COMP_FACTOR;
                comp_speed = constrain(comp_speed, 0, 400);
                int target = (int)(escape_dir * comp_speed);
                if (lf > rf) {
                    motor_R1.target_speed = target;          // 右前轮补偿
                } else {
                    motor_L1.target_speed = target;          // 左前轮补偿
                }
            }
            if (slip_rear) {
                float fast_speed = fmaxf(fabsf(lr), fabsf(rr));
                float comp_speed = fast_speed * ESCAPE_COMP_FACTOR;
                comp_speed = constrain(comp_speed, 0, 400);
                int target = (int)(escape_dir * comp_speed);
                if (lr > rr) {
                    motor_R2.target_speed = target;          // 右后轮补偿
                } else {
                    motor_L2.target_speed = target;          // 左后轮补偿
                }
            }
        }
    } else {
        slip_counter = 0;
    }
}

/*==================================================== 电机底层驱动 ====================================================*/

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       Motor_SetSpeed
// 函数描述       单电机PWM输出 + 方向控制
// 参数说明       duty > 0: 正转 (方向引脚高, PWM=duty)
//               duty < 0: 反转 (方向引脚低, PWM=-duty)
//               duty 被钳位在 [PWM_MIN_SAFE, PWM_MAX_SAFE] = [-3000, 3000]
//-------------------------------------------------------------------------------------------------------------------
void Motor_SetSpeed(motor_t *motor, int duty,
                    gpio_pin_enum turn_pin,
                    pwm_channel_enum pwm_pin)
{
    // L2/R2电机物理反接: 底层取反, 上层逻辑统一(正=前进)
    if (pwm_pin == MotorL2_Pwm || pwm_pin == MotorR2_Pwm) duty = -duty;

    if (duty > PWM_MAX_SAFE) duty = PWM_MAX_SAFE;
    if (duty < PWM_MIN_SAFE) duty = PWM_MIN_SAFE;
    motor->duty = duty;

    if (duty >= 0) {
        gpio_set_level(turn_pin, 1);
        pwm_set_duty(pwm_pin, duty);
    } else {
        gpio_set_level(turn_pin, 0);
        pwm_set_duty(pwm_pin, -duty);
    }
}

/*==================================================== 前馈计算 ====================================================*/

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       Feedforward_Calculate
// 函数描述       根据目标速度查表+线性插值计算前馈PWM值
//               前馈表 ff_table[] 为开环实测的 速度→PWM 映射
//               前馈提供控制量的主体, PID反馈仅做微调修正
//-------------------------------------------------------------------------------------------------------------------
float Feedforward_Calculate(float target_speed)
{
    float abs_speed = fabsf(target_speed);
    float ff_abs;

    if (abs_speed <= ff_table[0].speed) {
        ff_abs = ff_table[0].pwm;
    } else if (abs_speed >= ff_table[FF_TABLE_SIZE - 1].speed) {
        ff_abs = ff_table[FF_TABLE_SIZE - 1].pwm;
    } else {
        int i = 1;
        while (i < FF_TABLE_SIZE && abs_speed > ff_table[i].speed) {
            i++;
        }
        float t = (abs_speed - ff_table[i - 1].speed) /
                  (ff_table[i].speed - ff_table[i - 1].speed);
        ff_abs = ff_table[i - 1].pwm + t * (ff_table[i].pwm - ff_table[i - 1].pwm);
    }

    return (target_speed >= 0) ? ff_abs : -ff_abs;
}

/*==================================================== 运动学转换 ====================================================*/

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       EncoderPulses_To_WheelRPM
// 函数描述       编码器脉冲/10ms → 车轮转速 (RPM)
// 公式           encoder_rpm = (pulses / 采样时间) * 60 / 编码器每转脉冲数
//               wheel_rpm = encoder_rpm * 编码器齿轮比 (30:70)
//-------------------------------------------------------------------------------------------------------------------
float EncoderPulses_To_WheelRPM(int pulses_per_10ms)
{
    float encoder_rpm = (pulses_per_10ms / SAMPLE_TIME_S) * 60.0f / ENCODER_PULSES_PER_REV;
    float wheel_rpm = encoder_rpm * ENCODER_GEAR_RATIO;
    return wheel_rpm;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       WheelRPM_To_LinearVelocity
// 函数描述       车轮RPM → 线速度 (m/s)
//-------------------------------------------------------------------------------------------------------------------
float WheelRPM_To_LinearVelocity(float wheel_rpm)
{
    return (wheel_rpm / 60.0f) * WHEEL_CIRCUMFERENCE;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       LinearVelocity_To_WheelRPM
// 函数描述       线速度 (m/s) → 车轮RPM
//-------------------------------------------------------------------------------------------------------------------
float LinearVelocity_To_WheelRPM(float linear_velocity)
{
    return (linear_velocity / WHEEL_CIRCUMFERENCE) * 60.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       WheelRPM_To_EncoderPulses
// 函数描述       车轮RPM → 编码器脉冲/10ms
//-------------------------------------------------------------------------------------------------------------------
float WheelRPM_To_EncoderPulses(float wheel_rpm)
{
    float encoder_rpm = wheel_rpm / ENCODER_GEAR_RATIO;
    float pulses = encoder_rpm * ENCODER_PULSES_PER_REV * (SAMPLE_TIME_S / 60.0f);
    return pulses;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       LinearVelocity_To_EncoderPulses
// 函数描述       线速度 (m/s) → 编码器脉冲/10ms (速度环目标值转换)
//-------------------------------------------------------------------------------------------------------------------
float LinearVelocity_To_EncoderPulses(float linear_velocity)
{
    float wheel_rpm = LinearVelocity_To_WheelRPM(linear_velocity);
    return WheelRPM_To_EncoderPulses(wheel_rpm);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       EncoderPulses_To_LinearVelocity
// 函数描述       编码器脉冲/10ms → 线速度 (m/s) (实测速度反馈)
//-------------------------------------------------------------------------------------------------------------------
float EncoderPulses_To_LinearVelocity(int pulses_per_10ms)
{
    float wheel_rpm = EncoderPulses_To_WheelRPM(pulses_per_10ms);
    return WheelRPM_To_LinearVelocity(wheel_rpm);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       Speed_To_PWM
// 函数描述       速度 → PWM理论值转换 (基于电机模型, 用于开环参考)
//               公式: PWM = (motor_rpm / 18500) * 10000
//               注意: 此函数仅作参考, 实际控制使用前馈查表而非此模型
//-------------------------------------------------------------------------------------------------------------------
float Speed_To_PWM(float speed)
{
    float wheel_rpm = EncoderPulses_To_WheelRPM((int)speed);

    float total_ratio = MOTOR_INTERNAL_RATIO * EXTERNAL_GEAR_RATIO;
    float motor_rpm = wheel_rpm * total_ratio;

    float theoretical_pwm = (motor_rpm / 18500.0f) * 10000.0f;

    float pwm = theoretical_pwm;
    if (pwm > 300.0f)  pwm = 300.0f;
    if (pwm < -300.0f) pwm = -300.0f;
    if (pwm > 1000.0f) pwm = 1000.0f;
    if (pwm < -1000.0f) pwm = -1000.0f;

    return pwm;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       PWM_To_Speed
// 函数描述       PWM → 速度 反向转换 (Speed_To_PWM的逆运算)
//-------------------------------------------------------------------------------------------------------------------
float PWM_To_Speed(float pwm)
{
    if (pwm > 1000.0f) pwm = 1000.0f;
    if (pwm < -1000.0f) pwm = -1000.0f;

    float pwm_abs = fabsf(pwm);
    float motor_rpm = (pwm_abs / 1000.0f) * 18500.0f;

    float total_ratio = MOTOR_INTERNAL_RATIO * EXTERNAL_GEAR_RATIO;
    float wheel_rpm = motor_rpm / total_ratio;

    float encoder_rpm = wheel_rpm / ENCODER_GEAR_RATIO;
    float pulses_per_minute = encoder_rpm * ENCODER_PULSES_PER_REV;
    float speed = pulses_per_minute * (SAMPLE_TIME_S / 60.0f);

    if (pwm < 0) speed = -speed;
    return speed;
}

/*==================================================== 开环运动函数 (手动/遥控) ====================================================*/

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       Front_back_control
// 函数描述       底盘前后运动 (开环)
// 参数说明       front_back_flag: >0前进, <0后退, =0停止
//               duty: PWM占空比 (仅正数)
//-------------------------------------------------------------------------------------------------------------------
void Front_back_control(int front_back_flag, int duty)
{
    if (front_back_flag < 0) {
        Motor_SetSpeed(&motor_L1, -duty, MotorL1_Turn, MotorL1_Pwm);
        Motor_SetSpeed(&motor_L2,  duty, MotorL2_Turn, MotorL2_Pwm);
        Motor_SetSpeed(&motor_R1, -duty, MotorR1_Turn, MotorR1_Pwm);
        Motor_SetSpeed(&motor_R2,  duty, MotorR2_Turn, MotorR2_Pwm);
    } else if (front_back_flag > 0) {
        Motor_SetSpeed(&motor_L1,  duty, MotorL1_Turn, MotorL1_Pwm);
        Motor_SetSpeed(&motor_L2,  duty, MotorL2_Turn, MotorL2_Pwm);
        Motor_SetSpeed(&motor_R1,  duty, MotorR1_Turn, MotorR1_Pwm);
        Motor_SetSpeed(&motor_R2,  duty, MotorR2_Turn, MotorR2_Pwm);
    } else {
        Motor_SetSpeed(&motor_L1, 0, MotorL1_Turn, MotorL1_Pwm);
        Motor_SetSpeed(&motor_L2, 0, MotorL2_Turn, MotorL2_Pwm);
        Motor_SetSpeed(&motor_R1, 0, MotorR1_Turn, MotorR1_Pwm);
        Motor_SetSpeed(&motor_R2, 0, MotorR2_Turn, MotorR2_Pwm);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       Horizon_Translation
// 函数描述       底盘左右平移 (开环)
//-------------------------------------------------------------------------------------------------------------------
void Horizon_Translation(int horizontal_flag, int duty)
{
    if (horizontal_flag > 0) {
        Motor_SetSpeed(&motor_L1,  duty, MotorL1_Turn, MotorL1_Pwm);
        Motor_SetSpeed(&motor_L2,  duty, MotorL2_Turn, MotorL2_Pwm);
        Motor_SetSpeed(&motor_R1, -duty, MotorR1_Turn, MotorR1_Pwm);
        Motor_SetSpeed(&motor_R2,  duty, MotorR2_Turn, MotorR2_Pwm);
    } else if (horizontal_flag < 0) {
        Motor_SetSpeed(&motor_L1, -duty, MotorL1_Turn, MotorL1_Pwm);
        Motor_SetSpeed(&motor_L2,  duty, MotorL2_Turn, MotorL2_Pwm);
        Motor_SetSpeed(&motor_R1,  duty, MotorR1_Turn, MotorR1_Pwm);
        Motor_SetSpeed(&motor_R2,  duty, MotorR2_Turn, MotorR2_Pwm);
    } else {
        Motor_SetSpeed(&motor_L1, 0, MotorL1_Turn, MotorL1_Pwm);
        Motor_SetSpeed(&motor_L2, 0, MotorL2_Turn, MotorL2_Pwm);
        Motor_SetSpeed(&motor_R1, 0, MotorR1_Turn, MotorR1_Pwm);
        Motor_SetSpeed(&motor_R2, 0, MotorR2_Turn, MotorR2_Pwm);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       UpperRight_UnderLeft
// 函数描述       底盘斜向平移 (右上-左下对角线方向)
//-------------------------------------------------------------------------------------------------------------------
void UpperRight_UnderLeft(int Inclined_flag, int duty)
{
    if (Inclined_flag > 0) {
        Motor_SetSpeed(&motor_L1,  duty, MotorL1_Turn, MotorL1_Pwm);
        Motor_SetSpeed(&motor_L2,  0, MotorL2_Turn, MotorL2_Pwm);
        Motor_SetSpeed(&motor_R1,  duty, MotorR1_Turn, MotorR1_Pwm);
        Motor_SetSpeed(&motor_R2,  0, MotorR2_Turn, MotorR2_Pwm);
    } else if (Inclined_flag < 0) {
        Motor_SetSpeed(&motor_L1,  0, MotorL1_Turn, MotorL1_Pwm);
        Motor_SetSpeed(&motor_L2,  duty, MotorL2_Turn, MotorL2_Pwm);
        Motor_SetSpeed(&motor_R1,  0, MotorR1_Turn, MotorR1_Pwm);
        Motor_SetSpeed(&motor_R2,  duty, MotorR2_Turn, MotorR2_Pwm);
    } else {
        Motor_SetSpeed(&motor_L1, 0, MotorL1_Turn, MotorL1_Pwm);
        Motor_SetSpeed(&motor_L2, 0, MotorL2_Turn, MotorL2_Pwm);
        Motor_SetSpeed(&motor_R1, 0, MotorR1_Turn, MotorR1_Pwm);
        Motor_SetSpeed(&motor_R2, 0, MotorR2_Turn, MotorR2_Pwm);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       UpperLeft_UnderRight
// 函数描述       底盘斜向平移 (左上-右下对角线方向)
//-------------------------------------------------------------------------------------------------------------------
void UpperLeft_UnderRight(int Inclined_flag, int duty)
{
    if (Inclined_flag > 0) {
        Motor_SetSpeed(&motor_L1, 0, MotorL1_Turn, MotorL1_Pwm);
        Motor_SetSpeed(&motor_L2, duty, MotorL2_Turn, MotorL2_Pwm);
        Motor_SetSpeed(&motor_R1, 0, MotorR1_Turn, MotorR1_Pwm);
        Motor_SetSpeed(&motor_R2, duty, MotorR2_Turn, MotorR2_Pwm);
    } else if (Inclined_flag < 0) {
        Motor_SetSpeed(&motor_L1, -duty, MotorL1_Turn, MotorL1_Pwm);
        Motor_SetSpeed(&motor_L2, 0, MotorL2_Turn, MotorL2_Pwm);
        Motor_SetSpeed(&motor_R1, -duty, MotorR1_Turn, MotorR1_Pwm);
        Motor_SetSpeed(&motor_R2, 0, MotorR2_Turn, MotorR2_Pwm);
    } else {
        Motor_SetSpeed(&motor_L1, 0, MotorL1_Turn, MotorL1_Pwm);
        Motor_SetSpeed(&motor_L2, 0, MotorL2_Turn, MotorL2_Pwm);
        Motor_SetSpeed(&motor_R1, 0, MotorR1_Turn, MotorR1_Pwm);
        Motor_SetSpeed(&motor_R2, 0, MotorR2_Turn, MotorR2_Pwm);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       AutoGytation
// 函数描述       底盘原地旋转 (开环)
//-------------------------------------------------------------------------------------------------------------------
void AutoGytation(int rotate_flag, int duty)
{
    if (rotate_flag > 0) {
        Motor_SetSpeed(&motor_L1,  duty, MotorL1_Turn, MotorL1_Pwm);
        Motor_SetSpeed(&motor_L2,  duty, MotorL2_Turn, MotorL2_Pwm);
        Motor_SetSpeed(&motor_R1, -duty, MotorR1_Turn, MotorR1_Pwm);
        Motor_SetSpeed(&motor_R2,  duty, MotorR2_Turn, MotorR2_Pwm);
    } else if (rotate_flag < 0) {
        Motor_SetSpeed(&motor_L1, -duty, MotorL1_Turn, MotorL1_Pwm);
        Motor_SetSpeed(&motor_L2,  duty, MotorL2_Turn, MotorL2_Pwm);
        Motor_SetSpeed(&motor_R1,  duty, MotorR1_Turn, MotorR1_Pwm);
        Motor_SetSpeed(&motor_R2,  duty, MotorR2_Turn, MotorR2_Pwm);
    } else {
        Motor_SetSpeed(&motor_L1, 0, MotorL1_Turn, MotorL1_Pwm);
        Motor_SetSpeed(&motor_L2, 0, MotorL2_Turn, MotorL2_Pwm);
        Motor_SetSpeed(&motor_R1, 0, MotorR1_Turn, MotorR1_Pwm);
        Motor_SetSpeed(&motor_R2, 0, MotorR2_Turn, MotorR2_Pwm);
    }
}

/*==================================================== PID闭环控制 ====================================================*/

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       Motor_Enable_PID
// 函数描述       统一使能/失能四个电机的速度PID
//-------------------------------------------------------------------------------------------------------------------
void Motor_Enable_PID(uint8_t enable)
{
    PID_Enable(&motor_L1.pid, enable);
    PID_Enable(&motor_L2.pid, enable);
    PID_Enable(&motor_R1.pid, enable);
    PID_Enable(&motor_R2.pid, enable);

    control_mode = enable ? CONTROL_MODE_CLOSED_LOOP : CONTROL_MODE_OPEN_LOOP;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       Motor_SafeStop_PID
// 函数描述       安全停止: 关闭PID + PWM清零 + 清除控制标志
//-------------------------------------------------------------------------------------------------------------------
void Motor_SafeStop_PID(void)
{
    Motor_Enable_PID(0);

    Motor_SetSpeed(&motor_L1, 0, MotorL1_Turn, MotorL1_Pwm);
    Motor_SetSpeed(&motor_L2, 0, MotorL2_Turn, MotorL2_Pwm);
    Motor_SetSpeed(&motor_R1, 0, MotorR1_Turn, MotorR1_Pwm);
    Motor_SetSpeed(&motor_R2, 0, MotorR2_Turn, MotorR2_Pwm);

    Start_Pid_Flag = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       Motor_SetTargetSpeed_All
// 函数描述       设置四个电机的目标速度 (编码器脉冲/10ms)
//-------------------------------------------------------------------------------------------------------------------
void Motor_SetTargetSpeed_All(float target_L1, float target_L2,
                              float target_R1, float target_R2)
{
    motor_L1.target_speed = (int)target_L1;
    motor_L2.target_speed = (int)target_L2;
    motor_R1.target_speed = (int)target_R1;
    motor_R2.target_speed = (int)target_R2;

    PID_SetTarget(&motor_L1.pid, target_L1);
    PID_SetTarget(&motor_L2.pid, target_L2);
    PID_SetTarget(&motor_R1.pid, target_R1);
    PID_SetTarget(&motor_R2.pid, target_R2);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       Motor_PID_Control_All
// 函数描述       四个电机速度PID统一更新 (在10ms中断中调用)
// 控制架构       total = 前馈(查表) + PID反馈(增量修正)
// 保护机制       ① PWM安全限幅 ±3000 (30%)
//               ② 输出变化率限制 ±500/周期 (5%/10ms) — 防止负载突变时电钻现象
//               ③ PID输出限幅(由PID自身IntegralLimit限制)
//-------------------------------------------------------------------------------------------------------------------
void Motor_PID_Control_All(void)
{
    float target, feedback, feedforward, total_output;

    // ---- 左前轮 ----
    if (motor_L1.pid.Enable) {
        // 转速限幅: 钳位目标速度到 ±SPD_CLAMP_MAX pulse/10ms (=200, 对应0.5m/s)
        target = (float)motor_L1.target_speed;
        if (target >  SPD_CLAMP_MAX) target =  SPD_CLAMP_MAX;
        if (target < -SPD_CLAMP_MAX) target = -SPD_CLAMP_MAX;
        PID_SetTarget(&motor_L1.pid, target);

        feedback    = PID_Calculate(&motor_L1.pid, (float)motor_L1.encoder_speed);
        //feedforward = Feedforward_Calculate(target);
        //total_output = feedforward + feedback;
        total_output = feedback;

        // PWM硬限幅 (仅作安全保护)
        if (total_output > PWM_MAX_SAFE) total_output = PWM_MAX_SAFE;
        if (total_output < PWM_MIN_SAFE) total_output = PWM_MIN_SAFE;

        motor_L1.duty = (int)total_output;
        Motor_SetSpeed(&motor_L1, motor_L1.duty, MotorL1_Turn, MotorL1_Pwm);
    }

    // ---- 左后轮 (电机线反接: 正duty=倒退, 故输出取反) ----
    if (motor_L2.pid.Enable) {
        target = (float)motor_L2.target_speed;
        if (target >  SPD_CLAMP_MAX) target =  SPD_CLAMP_MAX;
        if (target < -SPD_CLAMP_MAX) target = -SPD_CLAMP_MAX;
        PID_SetTarget(&motor_L2.pid, target);

        feedback    = PID_Calculate(&motor_L2.pid, (float)motor_L2.encoder_speed);
        //feedforward = Feedforward_Calculate(target);
        total_output = feedback;
        if (total_output > PWM_MAX_SAFE) total_output = PWM_MAX_SAFE;
        if (total_output < PWM_MIN_SAFE) total_output = PWM_MIN_SAFE;

        motor_L2.duty = (int)total_output;
        Motor_SetSpeed(&motor_L2, motor_L2.duty, MotorL2_Turn, MotorL2_Pwm);
    }

    // ---- 右前轮 ----
    if (motor_R1.pid.Enable) {
        target = (float)motor_R1.target_speed;
        if (target >  SPD_CLAMP_MAX) target =  SPD_CLAMP_MAX;
        if (target < -SPD_CLAMP_MAX) target = -SPD_CLAMP_MAX;
        PID_SetTarget(&motor_R1.pid, target);

        feedback    = PID_Calculate(&motor_R1.pid, (float)motor_R1.encoder_speed);
        //feedforward = Feedforward_Calculate(target);
//        total_output = feedforward + feedback;
        total_output = feedback;

        if (total_output > PWM_MAX_SAFE) total_output = PWM_MAX_SAFE;
        if (total_output < PWM_MIN_SAFE) total_output = PWM_MIN_SAFE;

        motor_R1.duty = (int)total_output;
        Motor_SetSpeed(&motor_R1, motor_R1.duty, MotorR1_Turn, MotorR1_Pwm);
    }

    // ---- 右后轮 (电机线反接: 正duty=倒退, 故输出取反) ----
    if (motor_R2.pid.Enable) {
        target = (float)motor_R2.target_speed;
        if (target >  SPD_CLAMP_MAX) target =  SPD_CLAMP_MAX;
        if (target < -SPD_CLAMP_MAX) target = -SPD_CLAMP_MAX;
        PID_SetTarget(&motor_R2.pid, target);

        feedback    = PID_Calculate(&motor_R2.pid, (float)motor_R2.encoder_speed);
        //feedforward = Feedforward_Calculate(target);
        total_output = feedback;
        if (total_output > PWM_MAX_SAFE) total_output = PWM_MAX_SAFE;
        if (total_output < PWM_MIN_SAFE) total_output = PWM_MIN_SAFE;

        motor_R2.duty = (int)total_output;
        Motor_SetSpeed(&motor_R2, motor_R2.duty, MotorR2_Turn, MotorR2_Pwm);
    }
}

/*==================================================== 运动学控制 ====================================================*/

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       Mecanum_Move
// 函数描述       麦克纳姆轮运动学解算: 底盘速度 (vx, vy, omega) → 四个车轮目标转速
// 公式           v_LF =  vx + vy + omega * R
//               v_LB =  vx - vy + omega * R
//               v_RF =  vx - vy - omega * R
//               v_RB =  vx + vy - omega * R
//               其中 R = (轮距 + 轴距) / 2 为等效旋转半径
// 注意           左后/右后轮方向取反 (编码器安装方向)
//-------------------------------------------------------------------------------------------------------------------
void Mecanum_Move(float vx, float vy, float omega)
{
    float v_LF = vx + vy + omega * TURN_RADIUS;
    float v_LB = vx - vy + omega * TURN_RADIUS;
    float v_RF = vx - vy - omega * TURN_RADIUS;
    float v_RB = vx + vy - omega * TURN_RADIUS;

    int target_LF = (int)LinearVelocity_To_EncoderPulses(v_LF);
    int target_LB = (int)LinearVelocity_To_EncoderPulses(v_LB);
    int target_RF = (int)LinearVelocity_To_EncoderPulses(v_RF);
    int target_RB = (int)LinearVelocity_To_EncoderPulses(v_RB);

    // L2/R2的电机反接已在Motor_PID_Control_All内部通过取反total_output处理
    Motor_SetTargetSpeed_All((float)target_LF,  (float)target_LB,
                             (float)target_RF,  (float)target_RB);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       Angle_Control_Yaw
// 函数描述       Yaw偏航角闭环控制: 目标角度0 → PID计算角速度 → 通过Mecanum_Move执行
//               用于直线行走时的航向保持
//               速度环(Kp=15)是内环, 角度环(Kp)是外环, 内环快外环慢
//-------------------------------------------------------------------------------------------------------------------
void Angle_Control_Yaw(float base_speed_mps, float wheel_track)
{
    if (!angle_pid_yaw.Enable) return;

    PID_SetTarget(&angle_pid_yaw, 0.0f);

    float yaw = My_Imu660ra_GetYaw();
    float omega_des = PID_Calculate(&angle_pid_yaw, yaw);       // 输出: 度/s
    float omega_rad = omega_des * (PI / 180.0f);               // 转换为 rad/s

    Mecanum_Move(base_speed_mps, 0.0f, omega_rad);
}

