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
    //菜单初始化 (2026-08-09 下地实测: 恢复屏幕和菜单控制)
    //2026-08-15 链路验证: 暂时注释屏幕, 单纯测通信通路 (发车逻辑不动)
    //2026-08-16: 通信问题已定位 (UART优先级), 恢复屏幕菜单
    App_Menu_Init();
    //磁力计初始化 (2026-08-07: 暂时注释, 导航用陀螺仪+零漂学习)
    qmc5883l_init();
    //W25Q64 掉电存储初始化 (软件SPI, P06_2/3/4/5)
    {
        uint8 w25_err = w25q64_init();
        // 2026-08-16: 不接烧录器验证期间, 注释一次性初始化打印 (UART0)
        //printf("W25Q64: init %s, ID=0x%06lX\n",
        //       (w25_err ? "FAIL" : "OK"), (unsigned long)w25q64_chip_id);

        // 掉电存储加载: Region2 设置 (航点数量/发车区启用/偏移/步长/速度限幅/记录模式) → 覆盖菜单全局
        // v2.7.0: magic 0x5A→0x5B, 老扇区 magic 不匹配 → 走 else 用默认值 (偏移单位 0.1m→0.01m 后老偏移作废)
        flash_settings_t st;
        if (FlashStore_LoadSettings(&st)) {
            wp_set        = st.wp_set;
            launch_enable = st.launch_enable;
            launch_off_x  = st.launch_off_x;
            launch_off_y  = st.launch_off_y;
            launch_step   = (st.launch_step >= 1 && st.launch_step <= 10) ? st.launch_step : 4;   // 步长钳位 1~10, 越界默认4=0.20m
            coord_step    = (st.coord_step  >= 1 && st.coord_step  <= 10) ? st.coord_step  : 4;   // 坐标模式步长钳位 (v2.8.0)
            // 速度限幅: v2.3.0 及以前结构体只有 8 字节, 新字段读到旧扇区残留(0xFF) → 超上限钳位默认
            spd_limit_x   = (st.spd_limit_x <= 10) ? st.spd_limit_x : 6;
            spd_limit_y   = (st.spd_limit_y <= 10) ? st.spd_limit_y : 6;
            pos_limit_x   = (st.pos_limit_x <= 10) ? st.pos_limit_x : 6;
            pos_limit_y   = (st.pos_limit_y <= 10) ? st.pos_limit_y : 6;
            rec_mode      = (st.rec_mode <= 1) ? st.rec_mode : 0;   // 记录模式 (旧填充字节读到0xFF → 0=手动)
            //printf("FLASH: settings loaded (wp=%d en=%d off=%d,%d step=%d cstep=%d spd=%d,%d pos=%d,%d rec=%d)\n",
            //       wp_set, launch_enable, launch_off_x, launch_off_y, launch_step, coord_step, spd_limit_x, spd_limit_y, pos_limit_x, pos_limit_y, rec_mode);
        } else {
            //printf("FLASH: no settings, use defaults (wp=%d en=%d step=%d cstep=%d spd=%d,%d pos=%d,%d rec=%d)\n",
            //       wp_set, launch_enable, launch_step, coord_step, spd_limit_x, spd_limit_y, pos_limit_x, pos_limit_y, rec_mode);
        }
        // Region1 航点地图 → 设 bcn_max/wp_max (按菜单设置) + 构建导航航点
        Inav_LoadMap();
    }
    //定时器初始化
    //控制输入: 真实无人机/视觉 = HC06 解析 #x,y,flag$ 帧 (原RS485/蓝牙路径); LoRa遥控模拟 = App_Lora
#if !MAG_CALIB_MODE
#if CONTROL_SRC_DRONE
    //真实无人机/视觉: UART_1 透传接收 #x,y,flag$ 帧 (视觉发送与无人机一致, 接收逻辑不变; 波特率与无线模块 115200 对齐)
    HC06_Init(115200);
#else
    //LoRa遥控器模拟无人机初始化 (磁力计校准期间不初始化, 避免占用UART_1和打印)
    App_Lora_Init();
#endif
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
