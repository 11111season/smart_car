#include "zf_common_headfile.h"
#include "HC06_Driver.h"
#include "control.h"
#include "battery.h"
#include "VL53L5CX.h"
#include "PID.h"
#include "INIT.h"

extern uint8_t out_flag;
extern flight_state_e flight_state;
extern _PID_param_st *(pPidObject[]);

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

        case 2:     // #2$ → 开启PID控制
            if (!unlocked) {
                printf("ignored (not unlocked yet)\r\n");
                break;
            }
            if (last_cmd == 2) {
                // 重复 #2：PID 全部清零，重新锁偏航
                PID_Rest_Init(pPidObject, 12);
                PIDYaw.target = eulerAngle.yaw;
                printf("PID reset, yaw=%.1f (re-trigger #2)\r\n", eulerAngle.yaw);
                break;
            }
            out_flag = 2;
            flight_state = STATE_TAKEOFF;
            printf("TAKEOFF (out=2)\r\n");
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
    uint32 tof_debug = 0;

    while(true)  
    {
        // 每 ~20 次循环轮询 VL53L5CX（≈25Hz，I2C 操作必须在主循环，不能在 ISR）
        if (++tof_poll >= 20) {
            tof_poll = 0;
            vl53l5cx_get_distance();
        }

        // [关闭] TOF 测试打印，避免干扰 DEBUG_SCORES
        /*
        if (++tof_debug >= 500) {
            tof_debug = 0;
            printf("TOF: z0=%4umm(st=%u) z5=%4umm(st=%u) z6=%4umm(st=%u) z9=%4umm(st=%u) z10=%4umm(st=%u)"
                   " | pz=%.2fm vz=%.2fm/s\r\n",
                   vl53l5cx_distance_mm,
                   vl53l5cx_zone_result.status[0],
                   vl53l5cx_zone_result.distance_mm[5], vl53l5cx_zone_result.status[5],
                   vl53l5cx_zone_result.distance_mm[6], vl53l5cx_zone_result.status[6],
                   vl53l5cx_zone_result.distance_mm[9], vl53l5cx_zone_result.status[9],
                   vl53l5cx_zone_result.distance_mm[10], vl53l5cx_zone_result.status[10],
                   (double)world_data.pz, (double)world_data.vz);
        }
        */

       // 每200次打印磁力计校准调试信息
//       if (++debug_cnt >= 200) {
//           debug_cnt = 0;
//           printf("mag_raw:%.3f,%.3f,%.3f | cal:%.3f,%.3f,%.3f\r\n",
//                  (double)mag_raw_gauss[0], (double)mag_raw_gauss[1], (double)mag_raw_gauss[2],
//                  (double)mag_calib[0], (double)mag_calib[1], (double)mag_calib[2]);
//       }
       //printf("%d,%d,%d\r\n", qmc5883l_mag_x, qmc5883l_mag_y, qmc5883l_mag_z);
      printf("%5d,%5d,%5d,%5d\r\n",m1,m2,m3,m4);
//    printf("%5f,%5f,%5f,%5f,%5f,%5f\r\n",PIDPosX.out,PIDPosY.out,world_data.vx,world_data.vy,(float)g_vision_share.car_x,(float)g_vision_share.car_y);
//    printf("%5f,%5f,%5f,%5f\r\n",imu_data.gyro_x_pt1,imu_data.gyro_y_pt1,eulerAngle.roll,eulerAngle.pitch);
      //printf("%5f,%5f,%5f,%5f\r\n",PIDHeight.out,PIDVelH.out,world_data.vz,alt.target_height);
  //    printf("%5f,%5f,%5f\r\n",qmc5883l_mag_x_gauss,qmc5883l_mag_y_gauss);
      HC06_Task();                        // 解析来自小车的 #N$ 指令

      {
          uint8_t cmd = HC06_GetCmd();    // 获取指令
          if(cmd) process_drone_command(cmd);
      }

      small_driver_set_duty(m1, m2, m3, m4);
    }
}
