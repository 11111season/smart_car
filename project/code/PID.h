#ifndef CODE_PID_H_
#define CODE_PID_H_


void PID_param_Init(void);//PID参数初始化
void PID_Rest(_PID_param_st **pid,const uint8_t len);
void PID_Update(_PID_param_st* pid,const float dt);
void Cascade_PID(_PID_param_st* pid_rate,_PID_param_st* pid_eular,const float dt);



#endif 
