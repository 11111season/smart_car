#include "init.h"
#include "key_task.h"      // MAG_CALIB_MODE
#include "App_lora3a22.h"

/*****************************************************************************
 * @name       : Init_all
 * @date       : 2026-03-9
 * @function   : 总的的初始化函数
 * @parameters : 无
 * @retvalue   : 无
 * @note       : 无
******************************************************************************/
void Init_all(void)
{
    //电机初始化
    Motor_Init();
    //编码器初始化
    Encoder_Init();
    //按键初始化
    key_init(10);
    //陀螺仪初始化
    //My_MPU6050_Init();
    imu660ra_init();
    //磁力计初始化 (2026-08-07: 暂时注释, 导航用陀螺仪+零漂学习)
    //qmc5883l_init();
    //定时器初始化
    //蓝牙/485初始化 (LoRa模拟无人机期间注释)
    //HC06_Init(1000000);
#if !MAG_CALIB_MODE
    //LoRa遥控器模拟无人机初始化 (磁力计校准期间不初始化, 避免占用UART_1和打印)
    App_Lora_Init();
#endif
    pit_ms_init(PIT_CH0,10);//开启了10ms中断
    pit_ms_init(PIT_CH1,10);//开启了10ms中断
    pit_us_init(PIT_CH2, 1);//这样就代表定时器定时1us触发一次中断
    
    // 启用速度环和角度环
    //Motor_Enable_PID(1);     // 由按键启动
    //PID_Enable(&angle_pid_yaw, 1);  // 由按键启动
    // 可选：重置yaw，将当前方向作为0°
    My_Imu660ra_ResetYaw();
    angle_target = 0.0f;   // 角度环目标为0
  
}