#include "zf_common_headfile.h"

//-------------struct----------
//pid
_PID_param_st PIDRateX; //内环PID数据,PID都要
_PID_param_st PIDRateY;
_PID_param_st PIDRateZ;

_PID_param_st PIDPitch; //外环PID数据,只要P
_PID_param_st PIDRoll;
_PID_param_st PIDYaw;

_PID_param_st PIDHeight;//外环
_PID_param_st PIDRateH;//内环





//imu
_imu_param_st imu_data;
//euler
_euler_param_st eulerAngle;

//remote
_remote_param_st rc;

//falg
_flag_param_st flag;

//TOF
_tof_param_st tof;

//世界坐标系
_world_param_st world_data;



//---------------变量-----------

//电机变量
uint16_t m1 = 0, m2 = 0, m3 = 0, m4 = 0;

//结构体数组，将每一个数组放一个pid结构体，这样就可以批量操作各个PID的数据了  比如解锁时批量复位pid控制数据，新手明白这句话的作用就可以了
_PID_param_st *(pPidObject[])={&PIDRateX,&PIDRateY,&PIDRateZ,&PIDRoll,&PIDPitch,&PIDYaw};

//传感器变量






void ALL_Init()
{
    //IPS初始化
    IPS_Init();       
    
    //陀螺仪初始化
    imu660rc_init(IMU660RC_QUARTERNION_120HZ);  
    
    //pwm初始化
    motor_init();     
    
    //PID参数
    PID_param_Init();
    
    //磁力计
    if(    qmc5883l_init()==0)
      printf("success");
    
    //TOF
    dl1b_init();
    //摄像头
//    camera_init();

    
    // 初始化时复位PID数据，防止随机值导致电机输出异常
    PID_Rest(pPidObject, 6);

//--------------pit-------------
    pit_ms_init(PIT_CH0, 5);    //5ms
      
    pit_ms_init(PIT_CH1, 5);    //5ms
    
    pit_ms_init(PIT_CH2, 20);    //5ms


}
 



