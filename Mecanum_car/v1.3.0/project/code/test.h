#ifndef CODE_TEST_H_
#define CODE_TEST_H_

#include "zf_common_headfile.h"
#include "Motor.h"
#include "My_Imu660ra.h"

/*----------------------- �ⲿ�������� -----------------------*/
extern volatile int count;
extern int change_flag;
extern volatile uint64_t time;
extern volatile int Start_Pid_Flag;
extern volatile int Stop_Pid_Flag;
extern volatile uint64_t time_us;
/*----------------------- �����ӿ����� -----------------------*/
void Angle_Test(float vx,float vy);
void Square_Test(float speed_mps, uint32_t duration_ms);
void Figure8_Test(float speed_mps, uint32_t duration_ms);
void IMU660RA_Test(void);
void My_IMU660RA_Test(void);

#endif /* CODE_TEST_H_ */