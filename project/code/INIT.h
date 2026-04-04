#ifndef CODE_INIT_H_
#define CODE_INIT_H_



//-----宏定义----
#undef DISABLE 
#define DISABLE 0
#undef ENABLE 
#define ENABLE 1
#undef REST
#define REST 0
#undef SET 
#define SET 1 
#undef EMERGENT
#define EMERGENT 0


//----------------------struct------------------------

//PID
typedef volatile struct
{
	float target;     
	float error;      
	float last_error;    
	float integ;        
        float deriv;
        float last_deriv;
        float measured;
        float last_measured;
	float kp;           
	float ki;           
	float kd;         
        float out;
	float Integ_LimitHigh;       
	float Integ_LimitLow;
	float Out_LimitHigh;
	float Out_LimitLow;
}_PID_param_st;

//IMU
typedef struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
} _imu_param_st;


//euler
typedef struct {
    float pitch;    
    float roll;     
    float yaw;       
} _euler_param_st;


//remote
typedef struct
{
	uint16_t roll;
	uint16_t pitch;
	uint16_t thr;
	uint16_t yaw;
	uint16_t AUX1;
	uint16_t AUX2;
	uint16_t AUX3;
	uint16_t AUX4;	
} _remote_param_st;


//标志位
typedef volatile struct
{
        uint8_t unlock;
	

} _flag_param_st;


//世界坐标系参数
typedef struct {
    float vx;    //世界x方向速度
    float vy;     
    float vz;     
    float ax;   //世界x方向加速度   
    float ay;     
    float az;    
    float px;    //世界坐标系位置
    float py;    
    float pz;    

} _world_param_st;


//高度
typedef struct {
    float height;  
    float last_height;
    float height_acc;
    float vz_acc;   
    float vz_deriv;
    float vz_last_deriv;
    
    
} _height_param_st;

//----------------------extern------------------------
//-------------struct----------
//PID
extern _PID_param_st PIDRateX; //内环PID数据,PID都要
extern _PID_param_st PIDRateY;
extern _PID_param_st PIDRateZ;

extern _PID_param_st PIDPitch; //外环PID数据,只要P
extern _PID_param_st PIDRoll;
extern _PID_param_st PIDYaw;

extern _PID_param_st PIDHeight;//外环
extern _PID_param_st PIDRateH;//内环

// 外部声明指针数组（在 control.c 中定义）
extern _PID_param_st *(pPidObject[]);

//IMU
extern _imu_param_st imu_data;
//euler
extern _euler_param_st eulerAngle;

//remote
extern _remote_param_st rc;

//flag
extern _flag_param_st flag;

//世界坐标系
extern _world_param_st world_data;

//高度
extern _height_param_st alt;


//---------------变量-----------

//电机变量
extern uint16_t m1;
extern uint16_t m2;
extern uint16_t m3;
extern uint16_t m4;

extern float buff_value;


void ALL_Init(void);//全部初始化



#endif 
