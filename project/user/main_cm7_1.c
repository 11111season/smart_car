#include "zf_common_headfile.h"
#include "camera.h"

#define SHARED  ((volatile vision_share_t *)VISION_SHARE_ADDR)

// ================= IPS 显示模式选择 =================
// 切换宏值来选择显示内容:
//   0 = 飞控数据模式 (姿态/电机/光流等, 原注释部分)
//   1 = 视觉调试模式 (视觉识别结果 + 霍夫参数)
//   2 = 静默模式 (二值化图像显示, 无文字覆盖)
#define IPS_DISPLAY_MODE    2   // ← 修改此值切换模式

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    debug_init();
     
    SCB_DisableDCache();
    camera_init();
    SCB_DisableDCache();

    // 设显示颜色：白字黑底，消除闪烁
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_clear();

    while(true)
    {
        camera_process();

#if IPS_DISPLAY_MODE == 0
        // ======== 模式0: 飞控数据模式 ========
//        ips200_show_string(16*0,  16*5,  "roll=:");  ips200_show_float(16*6, 16*5,  SHARED->disp_roll,   3,5);
//        ips200_show_string(16*0,  16*6,  "pitch=:"); ips200_show_float(16*6, 16*6,  SHARED->disp_pitch,  3,5);
//        ips200_show_string(16*0,  16*7,  "yaw=:");   ips200_show_float(16*6, 16*7,  SHARED->disp_yaw,    3,5);
        ips200_show_string(16*0,  16*8,  "m1=:");    ips200_show_float(16*6, 16*8,  SHARED->disp_m1,     4,5);
        ips200_show_string(16*0,  16*9,  "m2=:");    ips200_show_float(16*6, 16*9,  SHARED->disp_m2,     4,5);
        ips200_show_string(16*0,  16*10, "m3=:");    ips200_show_float(16*6, 16*10, SHARED->disp_m3,     4,5);
        ips200_show_string(16*0,  16*11, "m4=:");    ips200_show_float(16*6, 16*11, SHARED->disp_m4,     4,5);
        ips200_show_string(16*0,  16*12, "mag_x=:"); ips200_show_float(16*6, 16*12, SHARED->disp_mag_x,  3,5);
//        ips200_show_string(16*0,  16*13, "of.dx=:"); ips200_show_float(16*6, 16*13, SHARED->disp_of_dx,  3,5);
        ips200_show_string(16*0,  16*14, "w.vx=:");  ips200_show_float(16*6, 16*14, SHARED->disp_world_vx, 3,5);
        ips200_show_string(16*0,  16*15, "w.vy=:");  ips200_show_float(16*6, 16*15, SHARED->disp_world_vy, 3,5);
        ips200_show_string(16*0,  16*16, "tvx=:");   ips200_show_float(16*6, 16*16, SHARED->vel_tgt_x, 3,5);
        ips200_show_string(16*0,  16*17, "tvy=:");   ips200_show_float(16*6, 16*17, SHARED->vel_tgt_y, 3,5);
        ips200_show_string(16*0,  16*18, "imu.gx=:");ips200_show_float(16*6, 16*18, SHARED->disp_imu_gx, 3,5);
        ips200_show_string(16*0,  16*19, "tgt=:");   ips200_show_float(16*6, 16*19, SHARED->disp_target,  3,5);

#elif IPS_DISPLAY_MODE == 1
         // ======== 模式1: 视觉调试模式 (小车评分机制) ========
         // 注意: IPS200 宽240px, 8x16字体下 x + num*8 < 240
         ips200_show_string(16*0,  16*8,  "car=");
         ips200_show_int(16*4,  16*8, (int32)SHARED->car_found, 1);

         ips200_show_string(16*0,  16*9,  "id=");
         ips200_show_int(16*3,  16*9, (int32)SHARED->debug_car_track_id, 2);
         ips200_show_string(16*6,  16*9,  "sc=");
         ips200_show_int(16*9,  16*9, (int32)SHARED->debug_car_score, 3);

         ips200_show_string(16*0,  16*10,  "ang=");
         ips200_show_int(16*4,  16*10, (int32)SHARED->debug_car_angle, 3);
         ips200_show_string(16*9,  16*10,  "m=");
         ips200_show_int(16*11, 16*10, (int32)SHARED->debug_car_method, 1);

         ips200_show_string(16*0,  16*11,  "hd=");
         ips200_show_float(16*3,  16*11, SHARED->heading_angle * 57.29578f, 3, 2);

         ips200_show_string(16*0,  16*12,  "pos=");
         ips200_show_int(16*4,  16*12, (int32)SHARED->car_x, 4);
         ips200_show_string(16*9,  16*12,  ",");
         ips200_show_int(16*10, 16*12, (int32)SHARED->car_y, 4);
#endif
    }
}
