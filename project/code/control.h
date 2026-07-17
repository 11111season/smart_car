#ifndef CODE_CONTROL_H_
#define CODE_CONTROL_H_

//----------函数声明------------

void flight_control(float dt);            // 飞行状态机 (LOCK/UNLOCK/TAKEOFF/FLY)
void land(float dt);                      // 降落 (状态机外独立函数，后续再接入)
void stabilization(float dt);             // 纯姿态控制器 (角度环+角速度环，不含状态逻辑)
void hover_lock(void);


#endif 
