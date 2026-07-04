#ifndef _CAMERA_H_
#define _CAMERA_H_
#include "zf_common_headfile.h"

// ================= 图像尺寸配置 =================
#define CAMERA_W            MT9V03X_W
#define CAMERA_H            MT9V03X_H

// ================= 识别算法参数 =================
#define IR_THRESHOLD        140
#define MIN_IR_PIXEL_COUNT  3
#define IR_CORE_THRESHOLD   200

// 分类由软判决评分系统动态决策

// 调试开关：0=关闭, 1=每帧每个blob打印分数
#define DEBUG_SCORES            0

// 新增：配对与姿态解算参数
#define CAR_MATCH_MIN_DIST2      9      // 最小头尾距离² ≈ 3px
#define CAR_MATCH_MAX_DIST2      3600   // 最大头尾距离² ≈ 60px
#define HEADING_FILTER_ALPHA     0.7f

// 误差衰减系数 (每帧衰减比例, 0~1, 越小衰减越快)
#define ERR_DECAY_FACTOR        0.70f

// ================= 控制参数配置 =================
#define DEADZONE_X          10
#define DEADZONE_Y          10
#define MARKER_POINT_NUM    5
#define CAMERA_STATUS_LED   P22_0   // 避开P19_0（与DL1B I2C SCL冲突）

// ================= 双核共享内存配置 =================
#define VISION_SHARE_ADDR   0x28001000
typedef struct
{
    uint32 frame_id;

    int16 err_x;          // 信标相对小车的 X 误差: beacon_x - car_x
    int16 err_y;          // 信标相对小车的 Y 误差: beacon_y - car_y
    uint16 target_found;  // 同时识别到信标和小车

    int16 beacon_err_x;   // 信标相对画面中心的 X 误差
    int16 beacon_err_y;   // 信标相对画面中心的 Y 误差
    uint16 beacon_found;

    int16 car_x;
    int16 car_y;
    uint16 car_found;
    uint16 reserved;

    float heading_angle;

    // === IPS 显示数据段 (CM7_0 写入, CM7_1 读取并驱动屏幕) ===
    float  disp_roll, disp_pitch, disp_yaw;
    uint16 disp_m1, disp_m2, disp_m3, disp_m4;
    float  disp_mag_x, disp_of_dx, disp_world_vx;
    float  disp_imu_gx, disp_of_height, disp_target;
    float  disp_volt;
    uint32 disp_dirty;   // 非0表示有新数据显示
} vision_share_t;

// ================= 接口函数声明 =================
void camera_init(void);
void camera_process(void);

#endif
