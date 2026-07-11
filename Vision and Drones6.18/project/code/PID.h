#ifndef CODE_PID_H_
#define CODE_PID_H_

// =============================================================================
// PID参数初始化
// =============================================================================
void PID_param_Init(void);

// =============================================================================
// PID数据复位
// =============================================================================
void PID_Rest_Init(_PID_param_st **pid, const uint8_t len);

// =============================================================================
// PID计算函数
// =============================================================================
// 误差微分PID (适用于内环角速度控制)
void PID_Update(_PID_param_st *pid, float target, float measured, const float dt);

// 测量值微分PID (适用于外环角度控制，微分先行)
void PID_Update_d_measure(_PID_param_st *pid, float target, float measured, const float dt);

// Yaw专用PID (处理角度缠绕问题)
void PID_Update_Yaw(_PID_param_st *pid, float target, float measured, const float dt);

// 串级PID
void Cascade_PID(_PID_param_st *pid_rate, _PID_param_st *pid_eular, const float dt);

// =============================================================================
// Anti-Gravity 油门前馈
// =============================================================================
extern float anti_gravity_boost;
void anti_gravity_update(float throttle, float dt);

#endif /* CODE_PID_H_ */
