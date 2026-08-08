#ifndef CODE_INERTIAL_NAV_H_
#define CODE_INERTIAL_NAV_H_

#include "zf_common_headfile.h"
#include "motor.h"
#include "My_imu660ra.h"
#include "w25q64_storage.h"      // flash_wp_t (航点地图存储类型)

/*----------------------- ISR用: 信标导航期间覆盖target -----------------------*/
extern uint8_t bcn_nav_on;
extern float   bcn_nav_vx, bcn_nav_vy, bcn_nav_angle;

/*----------------------- 实时位置追踪 (m, 发车区=原点) -----------------------*/
extern float   g_pos_x, g_pos_y;
extern float   fused_yaw;            // 磁力计+陀螺仪互补融合yaw

void MagYaw_Update(void);
void MagYaw_Reset(void);             // 设当前方向为0度
extern uint8_t go_center;
extern uint8_t mission_armed;        // 任务武装: 菜单发车时置1, 开始响应无人机标志位
extern uint8_t patrol_active;        // 巡逻模式激活: flag=2时在航点间循环
extern uint16_t bcn_debounce;        // 按键消抖/到达锁

/*----------------------- 运行时航点上限 (原编译期宏 BCN_MAX/WP_MAX) -----------------------*/
extern uint8_t bcn_max;              // 记录航点数 (不含发车区) = 菜单 wp_set
extern uint8_t wp_max;               // 总航点数 (含发车区) = bcn_max + launch_enable

/*----------------------- 函数接口 -----------------------*/
void InertialNav_Init(void);
void InertialNav_Update(void);
void InertialNav_PosUpdate(void);    // ISR每10ms调用: 更新g_pos_x/y

/*----------------------- 菜单控制接口 (KEY_4 记录流程由菜单接管) -----------------------*/
void Inav_DisableControl(void);      // 取消闭环: PID全关+PWM清零 (记录前让车可自由推动)
void Inav_RecStart(void);            // 开始记录当前航点
void Inav_RecEnd(void);              // 结束记录当前航点 (存 bcn_abs[bcn_idx++])
void Inav_BuildMap(void);            // 全部录完: 构建导航航点 (含发车区) + 预计算段参数
void Inav_LoadMap(void);             // 上电: 从 W25Q64 读地图 → 设上限 + 构建
void Inav_UpdateMax(void);           // 菜单设置变化后重算 bcn_max/wp_max
void Inav_ResetMap(void);            // 清除历史: 清空内存航点
uint8_t Inav_CanLaunch(void);        // 航点是否录齐
uint8_t Inav_Launch(void);           // 武装发车: mission_armed=1 + 3.5s
void Inav_StopAll(void);             // 停止所有控制: 关环+零速+停桨+完赛级锁定
const flash_wp_t* Inav_GetMap(void); // 读当前录制航点地图 (存 W25Q64)

#endif /* CODE_INERTIAL_NAV_H_ */
