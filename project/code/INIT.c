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
    .offset = {-0.0045f, -0.1195f, 0.2380f},
    .scale  = {
        {1.2860f, 0.0f, 0.0f},
        {0.0f, 1.1549f, 0.0f},
        {0.0f, 0.0f, 1.0000f}
    }
};

//rc
_rc_param_st rc;


//-------------------------------------enum------------------------
flight_state_e flight_state ;
flight_mode_e flight_mode;


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
    -0.0045f, -0.1195f, 0.2380f
}; 

float scale[3][3] = {
    {1.2860f, 0.0f, 0.0f},
    {0.0f, 1.1549f, 0.0f},
    {0.0f, 0.0f, 1.0000f}
};

//-------------------------------------函数-------------------------------
void  ALL_Init()
{
  
    interrupt_global_disable();
    
    key_init(10);
    //IPS初始化
    //IPS_Init(); // CM7_1 驱屏，CM7_0通过共享内存写显示数据
    
    //陀螺仪初始化
    imu660rc_init(IMU660RC_QUARTERNION_240HZ);  
    
    //电机初始化
//    motor_init();     
    small_driver_uart_init();
    
    //PID参数
    PID_param_Init();
    
    //磁力计
    if(  qmc5883l_init()==0)
    {
      flag.qmc5883l_init = 1;
      flag.mag_fusion_enabled = 1;  // 磁力计可用时默认开启融合
    }
    else
    {
      flag.mag_fusion_enabled = 0;
    }
    flag.att_mode_full = 1;  // 默认使用完整姿态控制
    
    //磁力计设置校准参数
    mag_set_calibration(offset,scale);
    
    //TOF
    TOF_init();         // VL53L5CX 初始化（已替换 DL1B，中心4区平均，30Hz）
    
    //串口   
    fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 64);           // 初始化 fifo 挂载缓冲区   
    uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN);         // 初始化串口
    uart_rx_interrupt(UART_INDEX, 1);                                       // 开启 UART_INDEX 的接收中断
             
    //光流
    OF_init();   
    
    //rc
 //   lora3a22_init();
    
    // =============================================================================
    // 滤波器初始化
    // =============================================================================
    PT1Filter_InitWithFreq(&filter_height, HEIGHT_FILTER_FREQ, 50);
    PT1Filter_InitWithFreq(&filter_height_vz, HEIGHT_VZ_FILTER_FREQ, 50);
    PT1Filter_InitWithFreq(&filter_pwm3901_vx, OF_VX_FILTER_FREQ, 40);
    PT1Filter_InitWithFreq(&filter_pwm3901_vy, OF_VY_FILTER_FREQ, 40);
    PT1Filter_InitWithFreq(&filter_imu_gyro_x, GYRO_FILTER_FREQ, 200);
    PT1Filter_InitWithFreq(&filter_imu_gyro_y, GYRO_FILTER_FREQ, 200);
    PT1Filter_InitWithFreq(&filter_imu_gyro_z, GYRO_FILTER_FREQ, 200);

    // 初始化时复位PID数据，防止随机值导致电机输出异常
    PID_Rest_Init(pPidObject, 12);
    
    //imu_data参数设置
    
    imu_data.q0 = 0;
    imu_data.q1 = 0;  
    imu_data.q2 = 1;
    imu_data.q3 = 0;
       
  
    
//-------------------------------------软件逻辑------------------------------
    flight_state = STATE_LOCK;    

    
//-----------------------------------------pit-------------------------------
    pit_ms_init(PIT_CH0, 5);    //5ms
      
    pit_ms_init(PIT_CH1, 5);    
    
    pit_ms_init(PIT_CH2, 40);           // 40ms → ~25Hz（与 VL53L5CX TOF 测距频率匹配）
    
    pit_ms_init(PIT_CH10, 10);    

    pit_ms_init(PIT_CH11, 25);    
    
    pit_ms_init(PIT_CH12, 10);  //rc                                 

    
    interrupt_global_enable(0);
}
