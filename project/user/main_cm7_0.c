#include "zf_common_headfile.h"
#include "HC06_Driver.h"
#include "control.h"
#include "battery.h"
#include "VL53L5CX.h"
#include "PID.h"
#include "INIT.h"

extern uint8_t out_flag;
extern flight_state_e flight_state;
extern _euler_param_st eulerAngle;
extern _world_param_st world_data;
extern _height_param_st alt;
extern uint16_t m1, m2, m3, m4;
extern _PID_param_st *(pPidObject[]);
extern volatile uint8_t g_car_flag3_pending;   // control.c 中定义，无信标自动降落通知小车

// 处理来自小车的指令: #1$=起浆, #2$=PID控制, #3$=停止
static void process_drone_command(uint8_t cmd)
{
    static uint8_t unlocked = 0;        // #1 解锁标记
    static uint8_t last_cmd = 0;        // 上一次指令

    printf("[CMD] recv #%d$ → ", cmd);

    switch(cmd)
    {
        case 1:     // #1$ → 起浆(解锁)
            unlocked = 1;
            out_flag = 1;
            flight_state = STATE_UNLOCK;
            printf("UNLOCK (out=1)\r\n");
            break;

        case 2:     // #2$ → 起飞; 若已在飞行则降落
            if (!unlocked) {
                printf("ignored (not unlocked yet)\r\n");
                break;
            }
            if (flight_state == STATE_FLY) {
                // 飞行中再次收到 #2$ → 降落
                flight_state = STATE_LAND;
                printf("LAND (from FLY)\r\n");
            } else if (flight_state == STATE_LAND || flight_state == STATE_TAKEOFF) {
                printf("ignored (in LAND/TAKEOFF already)\r\n");
            } else {
                PIDYaw.target = eulerAngle.yaw;  // 锁当前偏航角为零点
                printf("yaw locked=%.1f\r\n", eulerAngle.yaw);
                out_flag = 2;
                flight_state = STATE_TAKEOFF;
                printf("TAKEOFF (out=2)\r\n");
            }
            break;

        case 3:     // #3$ → 停止输出
            unlocked = 0;
            out_flag = 0;
            flight_state = STATE_LOCK;
            printf("LOCK (out=0, unlocked=0)\r\n");
            break;

        default:
            printf("unknown cmd=%d\r\n", cmd);
            break;
    }
    last_cmd = cmd;
}


int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	
    debug_init();                   
    SCB_DisableDCache();
    
    HC06_Init(115200);
    
    ALL_Init();
//    printf("CM7_0 init done\r\n");

    Battery_Init();                         // P07_5 ADC初始化，采样电池电压
    PIDRoll.target = 0;
    PIDPitch.target = 0;
    PIDYaw.target = 0;
    of.K = 5737;
    out_flag = 0;

    uint32 tof_poll = 0;
    uint32 flight_print = 0;             // 10Hz VOFA输出计数器

    while(true)  
    {
        // 每 ~20 次循环轮询 VL53L5CX（≈25Hz，I2C 操作必须在主循环，不能在 ISR）
        if (++tof_poll >= 20) {
            tof_poll = 0;
            vl53l5cx_get_distance();
        }

      HC06_Task();                        // 解析来自小车的 #N$ 指令

      {
          uint8_t cmd = HC06_GetCmd();    // 获取指令
          if(cmd) process_drone_command(cmd);
      }

      // 无信标自动降落: 在主循环发送 flag=3 给小车 (避免在 ISR 中调用 sprintf)
      if (g_car_flag3_pending) {
          g_car_flag3_pending = 0;
          HC06_SendVisionError(0, 0, 3);
      }

      // 10Hz VOFA+ 打印: 角速度期望,角速度测量,速度融合后,速度融合前,加速度,速度期望,高度
      if (++flight_print >= 50) {
          flight_print = 0;
          printf("%.2f,%.2f,%.2f, %.2f,%.2f,%.2f, "
                 "%.3f,%.3f,%.3f, %.3f,%.3f, "
                 "%.2f,%.2f,%.2f, %.2f,%.2f, "
                 "%.3f,%.3f\r\n",
                 (double)PIDVelX.target, (double)PIDVelY.target, (double)PIDVelZ.target,
                 (double)imu_data.gyro_x_pt1, (double)imu_data.gyro_y_pt1, (double)imu_data.gyro_z_pt1,
                 (double)world_data.vx, (double)world_data.vy, (double)world_data.vz,
                 (double)of.vx_pt1, (double)of.vy_pt1,
                 (double)imu_data.acc_x, (double)imu_data.acc_y, (double)imu_data.acc_z,
                 (double)PIDPosX_Vel.target, (double)PIDPosY_Vel.target,
                 (double)world_data.pz, (double)alt.target_height);
         // printf("%d,%d,%d,%d\r\n",m1,m2,m3,m4);
      }

      // out_flag=1 为电机测试模式：直接输出固定占空比，不经过 stabilization 混控
      if (out_flag == 1) {
          small_driver_set_duty(3000, 3000, 3000, 3000);
      } else {
          small_driver_set_duty(m1, m2, m3, m4);
      }
    }
}
