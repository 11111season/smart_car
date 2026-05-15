#ifndef CODE_INIT_H_
#define CODE_INIT_H_


//
////-----宏定义----
//#undef DISABLE 
//#define DISABLE 0
//#undef ENABLE 
//#define ENABLE 1
//#undef REST
//#define REST 0
//#undef SET 
//#define SET 1 
//#undef EMERGENT
//#define EMERGENT 0


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
typedef volatile struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
    float q0;
    float q1;
    float q2;
    float q3;
    float gyro_rad_y;
    float gyro_rad_x;
} _imu_param_st;


//euler
typedef volatile struct {
    float pitch;    
    float roll;     
    float yaw;       
} _euler_param_st;


////remote
//typedef struct
//{
//	uint16_t roll;
//	uint16_t pitch;
//	uint16_t thr;
//	uint16_t yaw;
//	uint16_t AUX1;
//	uint16_t AUX2;
//	uint16_t AUX3;
//	uint16_t AUX4;	
//} _remote_param_st;


//标志位
typedef volatile struct
{
    uint8_t unlock;
    uint8_t height_init;
    uint8_t of_init;
    uint8_t online;
    uint8_t pos_lock;
    uint8_t hover_lock;
    uint8_t take_off_yaw;
    uint8_t qmc5883l_init;

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
    float vx_pt1;
    float vy_pt1;
    float vx_imu;
    float vy_imu;
    float px;  
    float py;  
    float px_imu;
    float py_imu;
    
    
    float optical_ang_x;
    float optical_ang_y;   //光流感知到的角速度，需要消除 
    float rotation_ex;
    float rotation_ey;
    float K;
} _of_param_st;

//枚举状态
typedef enum{
    STATE_LOCK,
    STATE_UNLOCK,
    STATE_IDLE,
    STATE_TAKEOFF,
    STATE_HOVER,
    STATE_TASK,
    STATE_FLY,
    STATE_LAND,
    STATE_EMERGENCY
      
} flight_state_e;

typedef enum
{
    MODE_MANUAL = 0,     // 手飞
    MODE_ALT_HOLD,       // 定高
    MODE_POS_HOLD,       // 定点
    MODE_TASK            // 自动任务

}flight_mode_e;
//磁力计校准
typedef struct {
    float offset[3];        // 硬铁偏移 (bias)
    float scale[3][3];      // 软铁校正矩阵 (3x3)
} MagCalibration_t;


//rc
typedef struct {
   
    int16 thr;
    int16 roll;
    int16 yaw;
    int16 pitch;

    uint8 key1;
    uint8 key2;
    uint8 key3;
    uint8 key4;

    uint8 aux1;
    uint8 aux2;
    uint8 aux3;
    uint8 aux4;
  
    uint8 lock_cmd;
    uint8 unlock_cmd;
    uint8 task_cmd;
    uint8 takeoff_cmd;
    uint8 land_cmd;
    uint8 emergency_cmd;
  
} _rc_param_st;

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

extern _PID_param_st PIDPosX_Vel;//位置内环
extern _PID_param_st PIDPosY_Vel;

// 外部声明指针数组（在 control.c 中定义）
extern _PID_param_st *(pPidObject[]);

//IMU
extern _imu_param_st imu_data;

//euler
extern _euler_param_st eulerAngle;

//flag
extern _flag_param_st flag;

//世界坐标系
extern _world_param_st world_data;

//高度
extern _height_param_st alt;

//光流
extern _of_param_st of ; 

//磁力计
extern  MagCalibration_t mag_cal;

//remote
extern _rc_param_st rc;

//----------enum--------
extern flight_state_e flight_state ;
extern flight_mode_e flight_mode;






//---------------变量-----------

//电机变量
extern uint16_t m1;
extern uint16_t m2;
extern uint16_t m3;
extern uint16_t m4;

extern float buff_value;

//磁力计校准参数
extern float offset[3];
extern float scale[3][3];

//暂定
extern float THR;


void ALL_Init(void);//全部初始化



#endif 
