#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_

#include "zf_common_headfile.h"
#include "pid.h"
#include "math.h"
#include "My_imu660ra.h"
#include "HC06_Driver.h"
/*------------------------ 电机引脚定义 ------------------------*/
#define MotorL1_Pwm TCPWM_CH10_P05_1  // 左前轮
//#define MotorL1_Pwm TCPWM_CH25_P09_1  // 左前轮
//#define MotorR1_Pwm TCPWM_CH09_P05_0  // 右前轮
#define MotorR1_Pwm TCPWM_CH24_P09_0  // 右前轮
#define MotorL2_Pwm TCPWM_CH09_P05_0  // 左后轮
#define MotorR2_Pwm TCPWM_CH25_P09_1  // 右后轮

#define MotorL1_Turn P05_3  // 左前轮
//#define MotorL1_Turn P10_3  // 左前轮
//#define MotorR1_Turn P05_2  // 右前轮
#define MotorR1_Turn P10_2  // 右前轮
#define MotorL2_Turn P05_2  // 左后轮
#define MotorR2_Turn P10_3  // 右后轮
/*------------------------ 系统限幅宏定义 ------------------------*/
#define PWM_MAX_SAFE     300       // 安全最大PWM (10kHz量程: 30%)      // 安全最大PWM（对应25%占空比，根据你的电源能力设定）2500
#define PWM_MIN_SAFE     -300      // 安全最小PWM     // 安全最小PWM
#define PWM_HARD_LIMIT   1000      // 硬件PWM量程上限     // 硬件PWM上限（库函数范围）
/*------------------------ 死区补偿宏定义 ------------------------*/
#define DEADZONE_FRONT   20        // 前轮死区PWM阈值 (10kHz量程)       // 前轮死区PWM（根据实测调整）
#define DEADZONE_REAR    20        // 后轮死区PWM阈值 (10kHz量程)       // 后轮死区PWM（也可统一用一个宏）
/*------------------------ 系统物理参数宏定义 ------------------------*/
//完整的传动系统参数
#define ENCODER_PPR 1024.0f //编码器线数（每转脉冲数）
#define ENCODER_QUAD_MULTIPLIER 4.0f //4倍频正交解码
#define ENCODER_PULSES_PER_REV (ENCODER_PPR * ENCODER_QUAD_MULTIPLIER) //4096脉冲/转

// 车体尺寸参数（单位：米）
#define WHEEL_TRACK     0.18f   // 轮距（左右轮中心距离）
#define WHEEL_BASE      0.20f   // 轴距（前后轮中心距离），需实测，这里先用相同值
#define TURN_RADIUS      ((WHEEL_TRACK + WHEEL_BASE) / 2.0f)  // 旋转半径

#define WHEEL_DIAMETER_CM 7.6f  //轮子直径
#define WHEEL_DIAMETER_M (WHEEL_DIAMETER_CM / 100.0f) //轮子直径（米）
#define WHEEL_CIRCUMFERENCE (PI * WHEEL_DIAMETER_M) //轮子周长（米）

#define EXTERNAL_GEAR_RATIO 5.0f //外部齿轮减速比（70:14 = 5:1）
#define ENCODER_GEAR_RATIO (30.0f/70.0f) //编码器齿轮比（30:70≈0.4286）
#define  MOTOR_INTERNAL_RATIO    1.0f        // 电机直驱(1:1)

#define SAMPLE_TIME_MS          10.0f        // 编码器采样时间（毫秒）
#define SAMPLE_TIME_S           (SAMPLE_TIME_MS / 1000.0f) // 采样时间（秒）

/*----------------------- 电机参数结构体定义 -----------------------*/
typedef struct motor
{
    // 目标值
    int target_speed;   // 目标速度（编码器脉冲数）

    // 控制值
    int duty;           // PWM占空比（-2000 ~ 2000）

    // 反馈值
    float encoder_speed;  // 当前编码器速度（滤波后）
    int16 encoder_raw;    // 原始编码器脉冲数
    int32 total_encoder; // 总编码器脉冲数（用于计算距离）

    // PID控制参数
    PID pid;            // 使用新PID结构体
} motor_t;

/*----------------------- 外部参数引用 -----------------------*/
extern motor_t motor_L1;  // 左前轮
extern motor_t motor_L2;  // 左后轮
extern motor_t motor_R1;  // 右前轮
extern motor_t motor_R2;  // 右后轮
extern volatile int Start_Pid_Flag;
extern volatile int Stop_Pid_Flag;
extern PID angle_pid_yaw;   // 角度外环: 角度误差→目标角速度
extern PID angle_pid_gyro;  // 角速度内环: 陀螺反馈闭环, Ki自动吃零偏
extern float target_vy;
extern float target_vx;   // 直线运动时的目标速度（前进为正，后退为负）
extern float angle_target;   // 角度环目标角度（度）
extern uint8_t  mission_armed;      // 任务武装状态: 0=等待发车, 1=已发车, 响应无人机指令
extern uint64_t mission_arm_time;    // 发车时刻(us), 发车后等4s再执行
extern volatile uint8_t drone_beacon_flag;  // 无人机标志位: 1=发现信标, 2=丢失信标

/*----------------------- 函数接口声明 --------------------------*/
//初始化类函数
void Motor_Init(void);
void Angle_Init(void);

//通用的电机控制函数
void Motor_SetSpeed(motor_t *motor, int duty,
                    gpio_pin_enum turn_pin,
                    pwm_channel_enum pwm_pin);
// 原有的运动控制函数（开环控制）
void Front_back_control(int front_back_flag, int duty);
void Horizon_Translation(int horizontal_flag, int duty);
void UpperRight_UnderLeft(int Inclined_flag, int duty);
void UpperLeft_UnderRight(int Inclined_flag, int duty);
void AutoGytation(int rotate_flag, int duty);
//物理数学关系转换类函数
float EncoderPulses_To_WheelRPM(int pulses_per_10ms);
float WheelRPM_To_LinearVelocity(float wheel_rpm);
float LinearVelocity_To_WheelRPM(float linear_velocity);
float WheelRPM_To_EncoderPulses(float wheel_rpm);
float LinearVelocity_To_EncoderPulses(float linear_velocity);
float EncoderPulses_To_LinearVelocity(int pulses_per_10ms);
float Speed_To_PWM(float speed);
float PWM_To_Speed(float pwm);
//前馈计算函数
float Feedforward_Calculate(float target_speed);
//闭环控制函数
void Motor_Enable_PID(uint8_t enable);
void Motor_SetTargetSpeed_All(float target_L1, float target_L2,
                              float target_R1, float target_R2);
void Motor_PID_Control_All(void);
//宏观系统控制类函数
void Angle_Control_Yaw(float base_speed_mps, float wheel_track);
void Mecanum_Move(float vx, float vy, float omega);

void PositionControl_Init(void);
void PositionControl_Reset(void);
//用于计算无人机传过来的目标位置
void PositionControl_Update(void);
void AntiSlipControl(void);   // 打滑检测与脱困补偿

#endif /* CODE_MOTOR_H_ */