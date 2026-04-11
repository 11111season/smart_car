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
    uint8_t height_init;
    uint8_t of_init;
	

} _flag_param_st;//记得上电全给0


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
    float vz_deriv1;//激光差分速度滤波
    float vz_last_deriv;
    
    
} _height_param_st;

//光流
typedef struct {
    float dx;
    float dy;
    float dx_i;
    float dy_i;
    float height;  
    float vx;  
    float vy;  
    float vx1;
    float vy1;
    float vx_imu;
    float vy_imu;
    float px;  
    float py;  
    float px_imu;
    float py_imu;
    
} _of_param_st;

//----------------------extern------------------------
//-------------struct----------
//PID
extern _PID_param_st PIDVelX; //内环PID数据,PID都要
extern _PID_param_st PIDVelY;
extern _PID_param_st PIDVelZ;

extern _PID_param_st PIDPitch; //外环PID数据,只要P
extern _PID_param_st PIDRoll;
extern _PID_param_st PIDYaw;

extern _PID_param_st PIDHeight;//高度外环
extern _PID_param_st PIDVelH;//高度内环

extern _PID_param_st PIDPosX;//位置外环
extern _PID_param_st PIDPosY;

extern _PID_param_st PIDVelX;//位置内环
extern _PID_param_st PIDVelY;

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

//光流
extern _of_param_st of ; 

////滤波
// extern PT1Filter_t filter;
//---------------变量-----------

//电机变量
extern uint16_t m1;
extern uint16_t m2;
extern uint16_t m3;
extern uint16_t m4;

extern float buff_value;


void ALL_Init(void);//全部初始化



#endif 
