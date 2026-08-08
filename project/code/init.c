#include "init.h"
#include "key_task.h"      // MAG_CALIB_MODE
#include "App_lora3a22.h"
#include "App_Menu.h"
#include "W25Q64.h"        // 掉电存储 (软件SPI, P06_2/3/4/5)
#include "w25q64_storage.h"   // 掉电存储分区: flash_settings_t / FlashStore_*
#include "inertial_nav.h"     // Inav_LoadMap (上电载入航点地图)

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
    //菜单初始化
    App_Menu_Init();
    //磁力计初始化 (2026-08-07: 暂时注释, 导航用陀螺仪+零漂学习)
    //qmc5883l_init();
    //W25Q64 掉电存储初始化 (软件SPI, P06_2/3/4/5)
    {
        uint8 w25_err = w25q64_init();
        printf("W25Q64: init %s, ID=0x%06lX\n",
               (w25_err ? "FAIL" : "OK"), (unsigned long)w25q64_chip_id);

        // 掉电存储加载: Region2 设置 (航点数量/发车区启用/偏移) → 覆盖菜单全局
        flash_settings_t st;
        if (FlashStore_LoadSettings(&st)) {
            wp_set        = st.wp_set;
            launch_enable = st.launch_enable;
            launch_off_x  = st.launch_off_x;
            launch_off_y  = st.launch_off_y;
            printf("FLASH: settings loaded (wp=%d en=%d off=%d,%d)\n",
                   wp_set, launch_enable, launch_off_x, launch_off_y);
        } else {
            printf("FLASH: no settings, use defaults (wp=%d en=%d)\n", wp_set, launch_enable);
        }
        // Region1 航点地图 → 设 bcn_max/wp_max (按菜单设置) + 构建导航航点
        Inav_LoadMap();
    }
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
    pit_ms_init(PIT_CH10,10);//菜单显示节拍器 (10ms, App_Menu 消费 1s 自动返回, 独立定时器不与其他任务共用)
    
    // 启用速度环和角度环
    //Motor_Enable_PID(1);     // 由按键启动
    //PID_Enable(&angle_pid_yaw, 1);  // 由按键启动
    // 可选：重置yaw，将当前方向作为0°
    My_Imu660ra_ResetYaw();
    angle_target = 0.0f;   // 角度环目标为0
  
}