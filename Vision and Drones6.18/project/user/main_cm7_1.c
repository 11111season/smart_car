#include "zf_common_headfile.h"
#include "camera.h"

#define SHARED  ((volatile vision_share_t *)VISION_SHARE_ADDR)

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

        SCB_CleanInvalidateDCache_by_Addr((uint32_t *)SHARED, sizeof(vision_share_t));
        if(!SHARED->disp_dirty) continue;
        SHARED->disp_dirty = 0;

        ips200_show_string(16*0,  16*5,  "roll=:");  ips200_show_float(16*6, 16*5,  SHARED->disp_roll,   3,5);
        ips200_show_string(16*0,  16*6,  "pitch=:"); ips200_show_float(16*6, 16*6,  SHARED->disp_pitch,  3,5);
        ips200_show_string(16*0,  16*7,  "yaw=:");   ips200_show_float(16*6, 16*7,  SHARED->disp_yaw,    3,5);
        ips200_show_string(16*0,  16*8,  "m1=:");    ips200_show_float(16*6, 16*8,  SHARED->disp_m1,     4,5);
        ips200_show_string(16*0,  16*9,  "m2=:");    ips200_show_float(16*6, 16*9,  SHARED->disp_m2,     4,5);
        ips200_show_string(16*0,  16*10, "m3=:");    ips200_show_float(16*6, 16*10, SHARED->disp_m3,     4,5);
        ips200_show_string(16*0,  16*11, "m4=:");    ips200_show_float(16*6, 16*11, SHARED->disp_m4,     4,5);
        ips200_show_string(16*0,  16*12, "mag_x=:"); ips200_show_float(16*6, 16*12, SHARED->disp_mag_x,  3,5);
        ips200_show_string(16*0,  16*13, "of.dx=:"); ips200_show_float(16*6, 16*13, SHARED->disp_of_dx,  3,5);
        ips200_show_string(16*0,  16*14, "w.vx=:");  ips200_show_float(16*6, 16*14, SHARED->disp_world_vx, 3,5);
        ips200_show_string(16*0,  16*15, "imu.gx=:");ips200_show_float(16*6, 16*15, SHARED->disp_imu_gx, 3,5);
        ips200_show_string(16*0,  16*16, "of.h=:");  ips200_show_float(16*6, 16*16, SHARED->disp_of_height, 3,5);
        ips200_show_string(16*0,  16*17, "tgt=:");   ips200_show_float(16*6, 16*17, SHARED->disp_target,  3,5);
        ips200_show_string(16*0,  16*18, "volt=:");  ips200_show_float(16*6, 16*18, SHARED->disp_volt,    2,5);
    }
}
