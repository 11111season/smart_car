#ifndef CODE_INERTIAL_NAV_H_
#define CODE_INERTIAL_NAV_H_

#include "zf_common_headfile.h"
#include "motor.h"
#include "My_imu660ra.h"

/*----------------------- ISR用: 信标导航期间覆盖target -----------------------*/
extern uint8_t bcn_nav_on;
extern float   bcn_nav_vx, bcn_nav_vy, bcn_nav_angle;

/*----------------------- 实时位置追踪 (m, 发车区=原点) -----------------------*/
extern float   g_pos_x, g_pos_y;
extern float   fused_yaw;            // 磁力计+陀螺仪互补融合yaw

void MagYaw_Update(void);
void MagYaw_Reset(void);             // 设当前方向为0度
extern uint8_t go_center;
extern uint8_t mission_armed;        // 任务武装: 按键4第3次置1, 开始响应无人机标志位
extern uint16_t bcn_debounce;        // 按键消抖/到达锁

/*----------------------- 函数接口 -----------------------*/
void InertialNav_Init(void);
void InertialNav_KeyHandler(void);
void InertialNav_Update(void);
void InertialNav_PosUpdate(void);    // ISR每10ms调用: 更新g_pos_x/y

#endif /* CODE_INERTIAL_NAV_H_ */
