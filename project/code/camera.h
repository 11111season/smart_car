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

// 调试开关：0=关闭, 1=每帧打印 #err_x,err_y,n$ HC06 数据帧
#define DEBUG_HC06_FRAME        0
#define DEBUG_REALTIME_MONITOR  0   // 上位机可视化输出 (H, B, I行)

// 调试开关：0=关闭 (霍夫检测已废弃)
#define DEBUG_HOUGH             0
#define DEBUG_HOUGH_DIV         0

#define HEADING_FILTER_ALPHA     0.7f

// 误差衰减系数 (每帧衰减比例, 0~1, 越小衰减越快)
#define ERR_DECAY_FACTOR        0.96f

// ================= V形车标检测参数 =================
#define CAR_MARK_MIN_AREA          20      // 车标候选最小面积
#define CAR_MARK_MAX_AREA          900     // 车标候选最大面积
#define CAR_MARK_MIN_BASE_LEN      8       // 底边最小长度 (像素)
#define CAR_MARK_MIN_ARM_LEN       6       // 臂最小长度 (像素)
#define CAR_MARK_MIN_HEIGHT        5       // 顶点到底边最小垂距 (像素)
#define CAR_MARK_MIN_ANGLE_DEG     12      // 最小顶点角度 (度), 适应极端透视压缩 (原15)
#define CAR_MARK_MAX_ANGLE_DEG     90      // 最大顶点角度 (度)
#define CAR_MARK_ANGLE_CENTER_DEG  80      // 理想顶点角度 (直角≈90°)
#define CAR_MARK_MIN_SCORE         55      // 最低车标分类分数
#define CAR_MARK_DEBUG_DRAW        1       // 1=绘制V形调试叠加到图像

// ================= V形车标检测参数（霍夫变换已废弃） =================

// ================= 信标锁定参数 =================
#define BEACON_CONFIRM_FRAMES      3       // 信标锁定所需连续确认帧数

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
    uint16 car_fresh;       // 1=真实检测, 0=衰减保留值 (前馈用)

    float heading_angle;

    // === IPS 显示数据段 (CM7_0 写入, CM7_1 读取并驱动屏幕) ===
    float  disp_roll, disp_pitch, disp_yaw;
    uint16 disp_m1, disp_m2, disp_m3, disp_m4;
    float  disp_mag_x, disp_of_dx, disp_world_vx;
    float  disp_world_vy;              // 世界坐标系 Y 轴速度
    float  disp_imu_gx, disp_of_height, disp_target;
    float  disp_volt;
    float  vel_tgt_x, vel_tgt_y;       // 速度环目标 (CM7_0 写入, 上位机显示)
    float  ff_vel_x, ff_vel_y;         // 像素前馈量 (CM7_0 写入, 上位机显示)
    uint32 disp_dirty;   // 非0表示有新数据显示

    // === 视觉调试数据 (CM7_1 camera.c 写入, 主循环显示) ===
    uint16 debug_car_score;         // 最优小车标记的滤波分数
    int16  debug_car_angle;         // 最优小车标记的 V 形角度 (度)
    uint8  debug_car_method;        // 检测方式: 0=未检测到, 1=BFS, 2=霍夫后备
    uint8  debug_car_track_id;      // 最优小车标记的跟踪 ID
  } vision_share_t;

// ================= 接口函数声明 =================
void camera_init(void);
void camera_process(void);

#endif
