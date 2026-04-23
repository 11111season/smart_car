#include "zf_common_headfile.h"

//------------------------------------struct----------------------------
//pid
_PID_param_st PIDVelX; //内环PID数据,PID都要
_PID_param_st PIDVelY;
_PID_param_st PIDVelZ;

_PID_param_st PIDPitch; //外环PID数据,只要P
_PID_param_st PIDRoll;
_PID_param_st PIDYaw;

_PID_param_st PIDHeight;//外环
_PID_param_st PIDVelH;//内环

_PID_param_st PIDPosX;//位置外环
_PID_param_st PIDPosY;

_PID_param_st PIDPosX_Vel;//位置内环
_PID_param_st PIDPosY_Vel;


//imu
_imu_param_st imu_data;
//euler
_euler_param_st eulerAngle;

//remote
_remote_param_st rc;

//falg
_flag_param_st flag;

//世界坐标系
_world_param_st world_data;

//高度
_height_param_st alt;

//光流
_of_param_st of ;

//磁力计
MagCalibration_t mag_cal = {
    .offset = {0.0f, 0.0f, 0.0f},
    .scale  = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    }
};



//-------------------------------------enum------------------------
flight_state_e state ;


//-------------------------------------变量-----------------------
//电机变量
uint16_t m1 = 0, m2 = 0, m3 = 0, m4 = 0;

//结构体数组，将每一个数组放一个pid结构体，这样就可以批量操作各个PID的数据了  比如解锁时批量复位pid控制数据，新手明白这句话的作用就可以了
_PID_param_st *(pPidObject[])={
    &PIDRoll, &PIDPitch, &PIDYaw,
    &PIDVelX, &PIDVelY, &PIDVelZ,
    &PIDHeight, &PIDVelH,
    &PIDPosX, &PIDPosY,
    &PIDPosX_Vel, &PIDPosY_Vel
};

//串口数据
float buff_value;

//磁力计校准参数
float offset[3] = {
    -0.036896f, 0.048859f, 0.368527f};  
    
float scale[3][3] = {
    {1.024622f, 0.079763f, 0.060309f},
    {0.079763f, 1.006517f, 0.076973f},
    {0.060309f, 0.076973f, 0.984442f}
};

//-------------------------------------函数-------------------------------
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
    if(  qmc5883l_init()==0)
      printf("success");
    
    //磁力计设置校准参数
   mag_set_calibration(offset,scale);

    
    //TOF
    TOF_init();    
    //串口   
    fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 64);           // 初始化 fifo 挂载缓冲区   
    uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN);         // 初始化串口
    uart_rx_interrupt(UART_INDEX, 1);                                       // 开启 UART_INDEX 的接收中断
             
    //光流
    OF_init();    
    
    //滤波
    PT1Filter_InitWithFreq(&filter_height,100,100);
    PT1Filter_InitWithFreq(&filter_height_vz,20,100);//未调
    PT1Filter_InitWithFreq(&filter_pwm3901_vx,10,40);
    PT1Filter_InitWithFreq(&filter_pwm3901_vy,10,40);
   
    // 初始化时复位PID数据，防止随机值导致电机输出异常
    PID_Rest_Init(pPidObject, 12);
    
    
//-------------------------------------软件逻辑------------------------------
    state = STATE_LOCK;    

    
//-----------------------------------------pit-------------------------------
    pit_ms_init(PIT_CH0, 5);    //5ms
      
    pit_ms_init(PIT_CH1, 5);    
    
    pit_ms_init(PIT_CH2, 20);    
    
    pit_ms_init(PIT_CH10, 10);    

    pit_ms_init(PIT_CH11, 25);    


}
