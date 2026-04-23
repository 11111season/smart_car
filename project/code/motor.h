#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_

//pwm
#define PWM_CH1              (TCPWM_CH12_P05_3)                                // PWM输出端口，1号电机
#define PWM_CH2              (TCPWM_CH30_P10_2)                                // PWM输出端口，2号电机
#define PWM_CH3              (TCPWM_CH31_P10_3)                                // PWM输出端口，3号电机
#define PWM_CH4              (TCPWM_CH11_P05_2)                                // PWM输出端口，4号电机


//限幅    
#define MOTOR_MIN_DUTY   530     
#define MOTOR_MAX_DUTY   1125


//函数声明
void motor_init(void);
void motor_set(uint8 motor_num , uint16 speed);
void motor_set_all(uint16 m1, uint16 m2, uint16 m3, uint16 m4);


//函数
#define LIMIT(x,min,max) ((x)<(min)?(min):((x)>(max)?(max):(x)))
//用法speed = limit(speed, MOTOR_MIN_DUTY, MOTOR_MAX_DUTY);
#endif 
