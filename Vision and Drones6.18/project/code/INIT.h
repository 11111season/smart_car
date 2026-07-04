#ifndef CODE_INIT_H_
#define CODE_INIT_H_

#include "filter.h"

//----------------------struct------------------------

//PID
typedef struct {
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
  float test;
    PT1Filter_t deriv_filter;  // 微分项专用PT1滤波器
}_PID_param_st;

//IMU
typedef volatile struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float gyro_x_pt1;
    float gyro_y_pt1;
    float gyro_z_pt1;
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


//��־λ
//标志位
typedef volatile struct
{
    uint8_t height_init;
    uint8_t of_init;
    uint8_t online;
    uint8_t pos_lock;
    uint8_t hover_lock;
    uint8_t take_off_yaw;
    uint8_t qmc5883l_init;

    // 起飞控制
    uint8_t mag_fusion_enabled;   // 磁力计融合使能
    uint8_t att_mode_full;        // 姿态模式 (1=完整姿态, 0=纯角速度)
    uint8_t takeoff_phase;        // 1=起飞阶段(5秒内线性升高中)

    //rc
    uint8_t unlock;
    uint8_t lock;
//    uint8_t
//    uint8_t
//    uint8_t
//    uint8_t
//    uint8_t
//    uint8_t
//    
    

    
    

} _flag_param_st;//�ǵ��ϵ�ȫ��0


//��������ϵ����
typedef struct {
    float vx;    //����x�����ٶ�
    float vy;     
    float vz;     
    float ax;   //����x������ٶ�   
    float ay;     
    float az;    
    float px;    //��������ϵλ��
    float py;    
    float pz;    

} _world_param_st;


//�߶�
typedef struct {
    float height;  
    float last_height;
    float height_acc;
    float vz_acc;   
    float vz_deriv;
    float vz_deriv1;//�������ٶ��˲�
    float vz_last_deriv;
    float vz_laser_raw;
    float vz_laser;
    float error ;
    float height_limited;
    float laser_quality;
    float target_height;          // 高度控制目标值(m)，可动态变化
} _height_param_st;

//����
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
    float optical_ang_y;   // 光流感知的角速度
    float rotation_ex;
    float rotation_ey;
    float K;
} _of_param_st;

// 枚举状态
typedef enum{
    STATE_LOCK,
    STATE_UNLOCK,
    STATE_TAKEOFF
} flight_state_e;

typedef enum
{
    MODE_MANUAL = 0,     // 手动
    MODE_ALT_HOLD,       // 定高
    MODE_POS_HOLD,       // 定点
    MODE_TASK            // 自动任务

}flight_mode_e;
//������У׼
typedef struct {
    float offset[3];        // Ӳ��ƫ�� (bias)
    float scale[3][3];      // ����У������ (3x3)
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
extern _PID_param_st PIDVelX; //�ڻ�PID����,PID��Ҫ
extern _PID_param_st PIDVelY;
extern _PID_param_st PIDVelZ;

extern _PID_param_st PIDPitch; //�⻷PID����,ֻҪP
extern _PID_param_st PIDRoll;
extern _PID_param_st PIDYaw;

extern _PID_param_st PIDHeight;//�߶��⻷
extern _PID_param_st PIDVelH;//�߶��ڻ�

extern _PID_param_st PIDPosX;//λ���⻷
extern _PID_param_st PIDPosY;

extern _PID_param_st PIDPosX_Vel;//λ���ڻ�
extern _PID_param_st PIDPosY_Vel;

// �ⲿ����ָ�����飨�� control.c �ж��壩
extern _PID_param_st *(pPidObject[]);

//IMU
extern _imu_param_st imu_data;

//euler
extern _euler_param_st eulerAngle;

//flag
extern _flag_param_st flag;

//��������ϵ
extern _world_param_st world_data;

//�߶�
extern _height_param_st alt;

//����
extern _of_param_st of ; 

//������
extern  MagCalibration_t mag_cal;

//remote
extern _rc_param_st rc;

//----------enum--------
extern flight_state_e flight_state ;
extern flight_mode_e flight_mode;






//---------------����-----------

//�������
extern uint16_t m1;
extern uint16_t m2;
extern uint16_t m3;
extern uint16_t m4;

extern float buff_value;

//������У׼����
extern float offset[3];
extern float scale[3][3];

//�ݶ�
extern float THR;


void ALL_Init(void);//ȫ����ʼ��



#endif 
