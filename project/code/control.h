#ifndef CODE_CONTROL_H_
#define CODE_CONTROL_H_

//----------函数声明------------

void stabilization(float dt);             // 完整姿态控制 (角度环+角速度环)
void hover_lock(void);
void take_off(float dt);


#endif 
