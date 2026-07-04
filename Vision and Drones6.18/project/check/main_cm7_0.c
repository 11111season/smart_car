#include "zf_common_headfile.h"
#include "HC06_Driver.h"
#include "control.h"

#define CAR_CMD_STOP         "S!"

extern uint8_t out_flag;

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
        case 1:     // #1$ → 解锁/起飞
            out_flag = 1;
            flag.takeoff_phase = 1;
            alt.target_height = world_data.pz;
            PIDYaw.target = eulerAngle.yaw;
            break;

        case 2:     // #2$ → 重置PID+飞控
            flag.takeoff_phase = 0;
            PID_Rest_Init(pPidObject, 12);
            PIDYaw.target = eulerAngle.yaw;
            out_flag = 2;
            break;

        case 3:     // #3$ → 锁定/停止
            out_flag = 0;
            flag.takeoff_phase = 0;
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
    printf("CM7_0 init done\r\n");

    PIDRoll.target = 0;
    PIDPitch.target = 0;
    PIDYaw.target = 0;
    of.K = 5737;
    out_flag = 0;

    uint32 ips_cnt = 0;
    while(true)  
    {
      printf("%5d,%5d,%5d,%5d\r\n",m1,m2,m3,m4);
      process_hc06_command();
      small_driver_set_duty(m1, m2, m3, m4);

      // IPS 屏幕显示（每 200 次循环刷新一次）
      if(++ips_cnt >= 200)
      {
          ips_cnt = 0;
          ips200_clear();
          ips200_set_color(RGB565_WHITE, RGB565_BLACK);

          ips200_show_string(16*0,  16*5,  "roll=:");  ips200_show_float(16*6, 16*5,  eulerAngle.roll,     3,5);
          ips200_show_string(16*0,  16*6,  "pitch=:"); ips200_show_float(16*6, 16*6,  eulerAngle.pitch,    3,5);
          ips200_show_string(16*0,  16*7,  "yaw=:");   ips200_show_float(16*6, 16*7,  eulerAngle.yaw,      3,5);
          ips200_show_string(16*0,  16*8,  "m1=:");    ips200_show_float(16*6, 16*8,  (double)m1,          4,5);
          ips200_show_string(16*0,  16*9,  "m2=:");    ips200_show_float(16*6, 16*9,  (double)m2,          4,5);
          ips200_show_string(16*0,  16*10, "m3=:");    ips200_show_float(16*6, 16*10, (double)m3,          4,5);
          ips200_show_string(16*0,  16*11, "m4=:");    ips200_show_float(16*6, 16*11, (double)m4,          4,5);
          ips200_show_string(16*0,  16*12, "mag_x=:"); ips200_show_float(16*6, 16*12, (double)qmc5883l_mag_x, 3,5);
          ips200_show_string(16*0,  16*13, "of.dx=:"); ips200_show_float(16*6, 16*13, of.dx,              3,5);
          ips200_show_string(16*0,  16*14, "w.vx=:");  ips200_show_float(16*6, 16*14, world_data.vx,       3,5);
          ips200_show_string(16*0,  16*15, "imu.gx=:");ips200_show_float(16*6, 16*15, imu_data.gyro_x,     3,5);
          ips200_show_string(16*0,  16*16, "of.h=:");  ips200_show_float(16*6, 16*16, of.height,           3,5);
          ips200_show_string(16*0,  16*17, "tgt=:");   ips200_show_float(16*6, 16*17, PIDRoll.target,      3,5);
      }
    }
}
