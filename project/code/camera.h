#ifndef _CAMERA_H_
#define _CAMERA_H_




typedef struct
{
    uint16 x;
    uint16 y;
} Point;




  
// ================= 核心配置 =================
#define CAMERA_W            MT9V03X_W
#define CAMERA_H            MT9V03X_H


// 固定二值化阈值，可根据环境亮度在 200~240 之间调整
#define IR_THRESHOLD        200     

// 最小连通域面积阈值，用于删除零散噪点
#define MIN_IR_PIXEL_COUNT  5       

// 第五步：宽高比约束，先用比较宽松的范围
#define BEACON_RATIO_MIN_X100  60
#define BEACON_RATIO_MAX_X100  160

// 第六步：亮核约束 + 目标锁定，减少手电筒/阳光误识别与双目标抖动
#define IR_CORE_THRESHOLD          245
#define BLOB_CORE_RATIO_MIN_X100   35
#define LOCK_SEARCH_RADIUS         30
#define LOCK_KEEP_RATIO_X100       80

#define MARKER_POINT_NUM    5       // 十字标记点数量

// 摄像头状态指示灯
#define CAMERA_STATUS_LED   P19_0


//------------------extern-----------

extern uint8  marker_x[MARKER_POINT_NUM];
extern uint8  marker_y[MARKER_POINT_NUM];

// ================= 接口函数 =================
void camera_init(void);    // 摄像头与显示模块初始化
void camera_process(void); // 摄像头主处理流程：采集、识别、显示

#endif