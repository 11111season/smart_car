#include "zf_common_headfile.h"
#include "HC06_Driver.h"
#include "control.h"
#include "battery.h"
#include "VL53L5CX.h"

#define CAR_CMD_STOP         "S!"

extern uint8_t out_flag;
extern flight_state_e flight_state;
extern vision_share_t g_vision_share;

// HC06 帧解析状态
typedef enum {
    FRAME_WAIT_START,
    FRAME_RECV_DATA,
} frame_state_t;

static frame_state_t frame_state = FRAME_WAIT_START;
static uint8_t frame_buffer[8];
static uint8_t frame_idx = 0;

// 解析完整帧 "#N$" → 提取数值 N
static void ParseFrameData(uint8_t *buf, uint8_t len)
{
    uint8_t val = 0;
    for(uint8_t i = 0; i < len; i++)
    {
        if(buf[i] >= '0' && buf[i] <= '9')
            val = val * 10 + (buf[i] - '0');
        else
            break;
    }

    // 直接执行命令
    switch(val)
    {
        case 1:     // #1$ → 起浆(解锁)
            out_flag = 1;
            flight_state = STATE_UNLOCK;
            break;

        case 2:     // #2$ → 起飞
            out_flag = 2;
            flight_state = STATE_TAKEOFF;
            break;

        case 3:     // #3$ → 锁定/停止
            out_flag = 0;
            flight_state = STATE_LOCK;
            HC06_SendCmd(CAR_CMD_STOP);
            break;

        default:
            break;
    }
}

static void process_hc06_command(void)
{
    uint8_t data;

    // 从 HC06 FIFO 读取并解析
    while(fifo_read_element(&hc06_rx_fifo, &data, FIFO_READ_AND_CLEAN) == FIFO_SUCCESS)
    {
        if(frame_state == FRAME_WAIT_START && data == '#')
        {
            frame_state = FRAME_RECV_DATA;
            frame_idx = 0;
            continue;
        }

        if(frame_state == FRAME_RECV_DATA)
        {
            if(data == '$')
            {
                if(frame_idx > 0)
                {
                    frame_buffer[frame_idx] = '\0';
                    ParseFrameData(frame_buffer, frame_idx);
                }
                frame_state = FRAME_WAIT_START;
                continue;
            }
            else
            {
                if(frame_idx < sizeof(frame_buffer) - 1)
                {
                    frame_buffer[frame_idx++] = data;
                }
                continue;
            }
        }
    }
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
      //printf("%5d,%5d,%5d,%5d\r\n",m1,m2,m3,m4);
//    printf("%5f,%5f,%5f,%5f,%5f,%5f\r\n",PIDPosX.out,PIDPosY.out,world_data.vx,world_data.vy,(float)g_vision_share.car_x,(float)g_vision_share.car_y);
//    printf("%5f,%5f,%5f,%5f\r\n",imu_data.gyro_x_pt1,imu_data.gyro_y_pt1,eulerAngle.roll,eulerAngle.pitch);
      //printf("%5f,%5f,%5f,%5f\r\n",PIDHeight.out,PIDVelH.out,world_data.vz,alt.target_height);
  //    printf("%5f,%5f,%5f\r\n",qmc5883l_mag_x_gauss,qmc5883l_mag_y_gauss);
      process_hc06_command();
      small_driver_set_duty(m1, m2, m3, m4);
    }
}
