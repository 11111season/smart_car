#include "zf_common_headfile.h"

//5%-10% ==1ms-2ms，500-1250
//电机50hz，占空比最高1250;  电调发挥90% = 1125 。最低最好5% =  500 ；1.6ms=800占空比=0.64 ；

void motor_init(void)
{
    pwm_init(PWM_CH1, 50, 0); // 无刷50Hz，1号电机
    pwm_init(PWM_CH2, 50, 0); // 无刷50Hz，2号电机
    pwm_init(PWM_CH3, 50, 0); // 无刷50Hz，3号电机
    pwm_init(PWM_CH4, 50, 0); // 无刷50Hz，4号电机

}


void motor_set(uint8 motor_num , uint16 speed)
{
    if(speed < 540) speed = 540;
    if(speed > 1125) speed = 1125;
    
    if(motor_num==1){pwm_set_duty(PWM_CH1, speed);}
    else if(motor_num==2){pwm_set_duty(PWM_CH2, speed);}
    else if(motor_num==3){pwm_set_duty(PWM_CH3, speed);}
    else if(motor_num==4){pwm_set_duty(PWM_CH4, speed);}
//    else{return 0;}
}



//四轴同控制
void motor_set_all(uint16 m1, uint16 m2, uint16 m3, uint16 m4)
{
    motor_set(1, m1);
    motor_set(2, m2);
    motor_set(3, m3);
    motor_set(4, m4);
}

    
    
//
////switch
//void motor_set(uint8 motor_num, uint16 speed)
//{
//    // -------- 限幅 --------
//    if(speed < MOTOR_MIN_DUTY) speed = MOTOR_MIN_DUTY;
//    if(speed > MOTOR_MAX_DUTY) speed = MOTOR_MAX_DUTY;
//
//    // -------- 电机映射 --------
//    switch(motor_num)
//    {
//        case 1: pwm_set_duty(PWM_CH1, speed); break;
//        case 2: pwm_set_duty(PWM_CH2, speed); break;
//        case 3: pwm_set_duty(PWM_CH3, speed); break;
//        case 4: pwm_set_duty(PWM_CH4, speed); break;
//        default: break; // 防止非法输入
//    }
//}
//
//
////数组
//
//pwm_channel_enum motor_pwm[4] = {
//    PWM_CH1,
//    PWM_CH2,
//    PWM_CH3,
//    PWM_CH4
//};
//
//void motor_set(uint8 motor_num, uint16 speed)
//{
//    // -------- 检查编号 --------
//    if(motor_num < 1 || motor_num > 4) return;
//
//    // -------- 限幅 --------
//    if(speed < MOTOR_MIN_DUTY) speed = MOTOR_MIN_DUTY;
//    if(speed > MOTOR_MAX_DUTY) speed = MOTOR_MAX_DUTY;
//
//    // -------- 输出 --------
//    pwm_set_duty(motor_pwm[motor_num - 1], speed);
//}