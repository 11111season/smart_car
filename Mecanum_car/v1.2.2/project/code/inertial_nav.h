#ifndef CODE_INERTIAL_NAV_H_
#define CODE_INERTIAL_NAV_H_

#include "zf_common_headfile.h"
#include "motor.h"
#include "My_imu660ra.h"

/*----------------------- ISR用: 信标导航期间覆盖target -----------------------*/
extern uint8_t bcn_nav_on;
extern float   bcn_nav_vx, bcn_nav_vy, bcn_nav_angle;

/*----------------------- 函数接口 -----------------------*/
void InertialNav_Init(void);
void InertialNav_KeyHandler(void);   // KEY_4 按键处理
void InertialNav_Update(void);       // 主循环调用, 记录yaw/导航更新

#endif /* CODE_INERTIAL_NAV_H_ */
