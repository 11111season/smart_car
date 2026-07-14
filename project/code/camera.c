/*
 * =============================================================================
 * camera.c - 视觉识别处理核心
 * =============================================================================
 * 功能:
 *   1. MT9V03x 红外摄像头图像采集与二值化
 *   2. 连通域提取 (BFS) 与 Blob 分析
 *   3. 软判决评分系统: 区分信标 (BLOB_BEACON) 和 V形车标 (BLOB_CAR_MARKER)
 *   4. V形车标几何特征提取与姿态解算 (车中心、航向角)
 *   5. 跨帧跟踪 (track_id 匹配 + 分数平滑)
 *   6. 双核共享内存 (vision_share_t) 刷新，供控制核 CM7_0 读取
 *
 * 工作流程 (每帧):
 *   DMA读取灰度图 → 二值化 → extract_blobs() → compute_raw_scores()
 *   → cross_frame_track() → smooth_and_decide() → 去重/分类
 *   → 姿态解算 → 共享内存刷新 → IPS显示/串口调试输出
 *
 * 硬件:
 *   - 摄像头: MT9V03x (通过 SmartIO + DMA 捕获到 0x28026024)
 *   - IPS屏幕: 可选显示二值化图像
 *   - 运行核: CM7_1 (视觉处理核)
 * =============================================================================
 */

#include "camera.h"
#include "HC06_Driver.h"
#include <math.h> // 使用 atan2f 需要包含 math 库

#define VISION_ENABLE_IPS_DISPLAY   1
#define VISION_SERIAL_DEBUG         0
#define VISION_SERIAL_DEBUG_DIV     1

// 通用数值钳位宏
#ifndef CLAMP
#define CLAMP(x, lo, hi)  (((x) < (lo)) ? (lo) : (((x) > (hi)) ? (hi) : (x)))
#endif

// 定义坐标点结构体
typedef struct
{
    uint16 x;
    uint16 y;
} Point;

// 新增：Blob 分类枚举
typedef enum
{
    BLOB_UNKNOWN = 0,
    BLOB_BEACON,
    BLOB_CAR_MARKER       // V形车标
} BlobType;

// 凸包面积比阈值等

// 新增：Blob 统一数据结构
typedef struct
{
    uint16 cx;
    uint16 cy;
    uint16 width;
    uint16 height;
    uint16 area;
    uint16 core_count;
    uint8 type;              // 用 uint8 而非 BlobType 避免 IAR 4 字节枚举对齐; 赋值时用 BLOB_UNKNOWN/BEACON/CAR_MARKER
    uint16 owidth;    // 带方向的最小外接矩形宽度
    uint16 oheight;   // 带方向的最小外接矩形高度

    // 软判决：原始置信度分数 (无上限)
    uint16 raw_beacon_score;
    uint16 raw_marker_score;

    // 跨帧跟踪
    uint8 track_id;                    // 跟踪 ID (0=未跟踪)
    uint16 filt_beacon_score;            // 滤波后信标分
    uint16 filt_marker_score;            // 滤波后车标分

    // V形车标几何特征
    uint16 marker_vertex_x;              // 角点 A (指向车头)
    uint16 marker_vertex_y;
    uint16 marker_base1_x;               // 底边端点 B
    uint16 marker_base1_y;
    uint16 marker_base2_x;               // 底边端点 C
    uint16 marker_base2_y;
    uint16 marker_base_mid_x;            // 底边中点 D (近似车中心)
    uint16 marker_base_mid_y;
    uint16 marker_angle_deg;             // 顶点夹角 (度)
    uint16 marker_height;                // 顶点到底边垂距
    uint16 marker_base_len;              // 底边长度 B-C
    uint16 marker_arm1_len;              // 臂长 A-B
    uint16 marker_arm2_len;              // 臂长 A-C
    float  marker_heading;               // 车头方向 (D→A)
    float  theta;                        // 主轴角度 (rad), 用于碎片配对
    uint8  marker_convex_ratio;          // 凸包面积比 (blob面积/凸包面积*100), 凹形<85
    uint8  marker_dist_std_ratio;        // 轮廓距离标准差比 (std/mean*100), V形>35, 圆形<20
    // 凸度统计 (debug): 每个轮廓点到凸包边的平均距离(0-100归一化)
    uint8  dbg_conv_mean_def;            // 平均凸度缺陷 (0-100)
    uint8  dbg_conv_max_def;             // 最大凸度缺陷 (0-100)
    uint8  dbg_conv_gt2_pct;             // 缺陷>2px的点占比(%)
    uint16 dbg_raw_mid_x;                // EMA 平滑前的底边中点 x (诊断)
    uint16 dbg_raw_mid_y;                // EMA 平滑前的底边中点 y (诊断)
} Blob;

// ======================== 双核共享内存区 ========================
#pragma location = VISION_SHARE_ADDR
vision_share_t g_vision_share = {0};

static uint8 image_buffer[CAMERA_H][CAMERA_W]; // 图像处理缓冲区
static uint8 image_copy[CAMERA_H][CAMERA_W];   // 图像发送备份区
static uint8 marker_x[MARKER_POINT_NUM];       // 标记点 X 坐标数组
static uint8 marker_y[MARKER_POINT_NUM];       // 标记点 Y 坐标数组
static uint8 visited[CAMERA_H][CAMERA_W];      // BFS 访问标记数组
static Point queue_buf[CAMERA_W * CAMERA_H];   // BFS 队列缓冲区


    
// Blob 数据库缓存池
#define MAX_BLOBS 20
static Blob blobs[MAX_BLOBS];
static uint8 blob_num = 0;

// 跨帧跟踪状态
#define MAX_TRACKS 20
#define SCORE_HISTORY_WEIGHT  0.65f   // 历史分数权重
#define SCORE_CURRENT_WEIGHT  0.35f   // 当前帧分数权重
#define TRACK_MATCH_MAX_DIST2 625     // 跟踪匹配最大距离² ≈ 25px
static Blob prev_tracks[MAX_TRACKS];  // 上一帧副本
static uint8 prev_track_num = 0;

// 分数平滑与判决
#define SCORE_MIN_CONFIDENCE   30     // 最低有效置信度
#define SCORE_MIN_MARGIN       8      // 最高分需领先第二高的最小差距

      
// 航向角平滑滤波器全局状态量
static float filtered_cos = 1.0f;
static float filtered_sin = 0.0f;
static uint8 is_first_heading = 1; // 用于初始化滤波器

static void update_marker(uint8 cx, uint8 cy, uint8 is_found)
{
    if (is_found)
    {
        marker_x[0] = cx;     marker_y[0] = cy;
        marker_x[1] = cx - 5; marker_y[1] = cy;
        marker_x[2] = cx + 5; marker_y[2] = cy;
        marker_x[3] = cx;     marker_y[3] = cy - 5;
        marker_x[4] = cx;     marker_y[4] = cy + 5;
    }
    else
    {
        for (uint8 i = 0; i < MARKER_POINT_NUM; i++)
        {
            marker_x[i] = 255;
            marker_y[i] = 0;
        }
    }

}


  
    // 绘制十字（用于信标灯和中心点）
void draw_black_cross(uint8 cx, uint8 cy)
{
    for(int i = -5; i <= 5; i++)
    {
        if(cy + i >= 0 && cy + i < CAMERA_H) image_buffer[cy + i][cx] = 0;
        if(cx + i >= 0 && cx + i < CAMERA_W) image_buffer[cy][cx + i] = 0;
    }
}



// 新增：画白框（用于小车车头车尾）
void draw_white_box(uint8 cx, uint8 cy, uint8 size)
{
    int16 left = cx - size;
    int16 right = cx + size;
    int16 top = cy - size;
    int16 bottom = cy + size;

    for(int16 x = left; x <= right; x++)
    {
        if(top >= 0 && top < CAMERA_H && x >= 0 && x < CAMERA_W)
            image_buffer[top][x] = 255; // 改成 255
        if(bottom >= 0 && bottom < CAMERA_H && x >= 0 && x < CAMERA_W)
            image_buffer[bottom][x] = 255; // 改成 255
    }
    for(int16 y = top; y <= bottom; y++)
    {
        if(left >= 0 && left < CAMERA_W && y >= 0 && y < CAMERA_H)
            image_buffer[y][left] = 255; // 改成 255
        if(right >= 0 && right < CAMERA_W && y >= 0 && y < CAMERA_H)
            image_buffer[y][right] = 255; // 改成 255
    }
}



// 新增：画白十字（用于小车中心点）
void draw_white_cross(uint8 cx, uint8 cy)
{
    for(int i = -5; i <= 5; i++)
    {
        if(cy + i >= 0 && cy + i < CAMERA_H) image_buffer[cy + i][cx] = 255;
        if(cx + i >= 0 && cx + i < CAMERA_W) image_buffer[cy][cx + i] = 255;
    }
}

// 计算两点距离平方
static uint32 point_dist2(Point a, Point b)
{
    int32 dx = (int32)a.x - (int32)b.x;
    int32 dy = (int32)a.y - (int32)b.y;
    return (uint32)(dx * dx + dy * dy);
}

// 点在三角形内检测 (重心坐标法)
// 返回 1=在三角形内部或边上, 0=外部
static uint8 point_in_triangle(int32 px, int32 py,
                                int32 ax, int32 ay,
                                int32 bx, int32 by,
                                int32 cx, int32 cy)
{
    // 以三角形边为法线, 用叉积符号判断
    int32 d1 = (bx - ax) * (py - ay) - (by - ay) * (px - ax);
    int32 d2 = (cx - bx) * (py - by) - (cy - by) * (px - bx);
    int32 d3 = (ax - cx) * (py - cy) - (ay - cy) * (px - cx);

    uint8 has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    uint8 has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

    return !(has_neg && has_pos);  // 全同号 → 内部
}

// ★★★ 凸包算法: Andrew 单调链, 返回凸包面积
// 输入 pts[0..count-1], 输出 convex_area, 需要至少 3 个点
static uint32 convex_hull_area(Point *pts, uint32 count)
{
    if (count < 3) return 0;
    // 简单排序: 按 x 升序, x 相同按 y 升序 (插入排序, count 很小)
    for (uint32 i = 1; i < count; i++) {
        Point key = pts[i];
        int32 j = (int32)i - 1;
        while (j >= 0 && (pts[j].x > key.x || (pts[j].x == key.x && pts[j].y > key.y))) {
            pts[j+1] = pts[j];
            j--;
        }
        pts[j+1] = key;
    }
    // 去重 (相邻重复点只保留一个)
    uint32 n = 0;
    for (uint32 i = 0; i < count; i++) {
        if (i == 0 || pts[i].x != pts[i-1].x || pts[i].y != pts[i-1].y) {
            pts[n++] = pts[i];
        }
    }
    if (n < 3) return 0;
    // Andrew monotone chain, 用 static 暂存 hull 点 (最大 256, 实际 < 100)
    static Point hull[256];
    uint32 h = 0;
    // 下凸壳
    for (uint32 i = 0; i < n; i++) {
        while (h >= 2) {
            int32 cx = (int32)hull[h-1].x - (int32)hull[h-2].x;
            int32 cy = (int32)hull[h-1].y - (int32)hull[h-2].y;
            int32 nx = (int32)pts[i].x - (int32)hull[h-2].x;
            int32 ny = (int32)pts[i].y - (int32)hull[h-2].y;
            if (cx*ny - cy*nx <= 0) h--; else break;
        }
        hull[h++] = pts[i];
    }
    // 上凸壳
    uint32 lower = h;
    for (uint32 i = n-1; i > 0; i--) {
        while (h > lower) {
            int32 cx = (int32)hull[h-1].x - (int32)hull[h-2].x;
            int32 cy = (int32)hull[h-1].y - (int32)hull[h-2].y;
            int32 nx = (int32)pts[i].x - (int32)hull[h-2].x;
            int32 ny = (int32)pts[i].y - (int32)hull[h-2].y;
            if (cx*ny - cy*nx <= 0) h--; else break;
        }
        hull[h++] = pts[i];
    }
    // shoelace 公式求凸包面积
    uint32 area_x2 = 0;
    for (uint32 i = 0, j = 1; i < h; i++, j = (j+1 < h) ? j+1 : 0) {
        area_x2 += (uint32)((int32)hull[i].x * (int32)hull[j].y);
        area_x2 -= (uint32)((int32)hull[i].y * (int32)hull[j].x);
    }
    return area_x2 / 2;
}

// 在 image_buffer 上用 Bresenham 画圆
static void draw_circle(int cx, int cy, int r, uint8 color)
{
    int x = r, y = 0;
    int err = 1 - r;
    while (x >= y)
    {
        int pts[][2] = {{cx+x,cy+y},{cx-x,cy+y},{cx+x,cy-y},{cx-x,cy-y},
                        {cx+y,cy+x},{cx-y,cy+x},{cx+y,cy-x},{cx-y,cy-x}};
        for (int i = 0; i < 8; i++)
        {
            int px = pts[i][0], py = pts[i][1];
            if (px >= 0 && px < CAMERA_W && py >= 0 && py < CAMERA_H)
                image_buffer[py][px] = color;
        }
        y++;
        if (err <= 0) err += 2*y + 1;
        else { x--; err += 2*(y-x) + 1; }
    }
}

// 在 image_buffer 上画一条线 (Bresenham)
static void draw_line(int x0, int y0, int x1, int y1, uint8 color)
{
    int dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
    int dy = (y1 > y0) ? (y1 - y0) : (y0 - y1);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    while (1)
    {
        if (x0 >= 0 && x0 < CAMERA_W && y0 >= 0 && y0 < CAMERA_H)
            image_buffer[y0][x0] = color;
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

// 沿 Bresenham 线采样 image_buffer, 返回 blob 像素占比 (0~100)
// blob 判断: image_buffer[y][x] > IR_THRESHOLD (140)
static uint8 line_blob_pct(Point a, Point b)
{
    int x = (int)a.x, y = (int)a.y;
    int x1 = (int)b.x, y1 = (int)b.y;
    int dx = (x1 > x) ? (x1 - x) : (x - x1);
    int dy = (y1 > y) ? (y1 - y) : (y - y1);
    int sx = (x < x1) ? 1 : -1;
    int sy = (y < y1) ? 1 : -1;
    int err = dx - dy;
    uint32 total = 0, filled = 0;

    while (1)
    {
        total++;
        if (x >= 0 && x < CAMERA_W && y >= 0 && y < CAMERA_H)
        {
            if (image_buffer[y][x] > (uint8)IR_THRESHOLD) filled++;
        }
        if (x == x1 && y == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x += sx; }
        if (e2 <  dx) { err += dx; y += sy; }
    }
    if (total == 0) return 0;
    return (uint8)((filled * 100) / total);
}

// V形车标记特征提取：从 blob 像素中找 3 个关键点 (角点A, 底边端点B,C)
static void compute_car_marker_feature(Blob *b, Point *pts, uint32 count)
{
    // 面积预过滤
    if(b->area < CAR_MARK_MIN_AREA || b->area > CAR_MARK_MAX_AREA) return;

    b->marker_angle_deg = 0;   // 清除上帧角度, 防止复用旧值

    // ===== 暴力搜索 V 形三点 =====
    // 枚举轮廓点采样对作为底边候选, 找最佳三角形
    {
        // 采样步长与轮廓点数成比例, 保证计算量可控
        uint8 step_s = (count > 80) ? 4u : (count > 40) ? 3u : 2u;
        if (step_s < 2) step_s = 2;

        uint32 best_score = 0;
        uint16 best_vertex_idx = 0, best_base1_idx = 0, best_base2_idx = 0;
        uint16 best_h_times10 = 0;  // 保存最佳高度×10

        for (uint32 i = 0; i < count; i += step_s)
        {
            for (uint32 j = i + step_s * 2; j < count; j += step_s)
            {
                int32 dx = (int32)pts[j].x - (int32)pts[i].x;
                int32 dy = (int32)pts[j].y - (int32)pts[i].y;
                uint32 d2 = (uint32)(dx*dx + dy*dy);
                uint32 min_d2 = (uint32)CAR_MARK_MIN_BASE_LEN * (uint32)CAR_MARK_MIN_BASE_LEN;
                if (d2 < min_d2) continue;  // 底边太短

                float bx_f = (float)dx;
                float by_f = (float)dy;
                float base_len_f = sqrtf(bx_f * bx_f + by_f * by_f);
                if (base_len_f < 0.5f) continue;

                // ===== 连线检验 → 快速滤除底边候选 =====
                // 底点间连线应跨越 V 形开口 (低 blob 占比)
                // 如果穿过 blob → 不是真正的底边, 跳过整个 (i,j)
                {
                    uint8 pct_base = line_blob_pct(pts[i], pts[j]);
                    if (pct_base > 45) continue;
                }

                // 找离线段 BC 最远的轮廓点作为顶点 A
                float best_h = 0;
                uint32 best_k = 0;
                for (uint32 k = 0; k < count; k += 1)  // 用步长1以获得最大精度
                {
                    if (k == i || k == j) continue;
                    float px = (float)((int32)pts[k].x - (int32)pts[i].x);
                    float py = (float)((int32)pts[k].y - (int32)pts[i].y);
                    float h = fabsf(px * by_f - py * bx_f) / base_len_f;
                    if (h > best_h) { best_h = h; best_k = k; }
                }

                if (best_h < (float)CAR_MARK_MIN_HEIGHT) continue;

                // 计算 V 形夹角
                float v1x = (float)((int32)pts[i].x - (int32)pts[best_k].x);
                float v1y = (float)((int32)pts[i].y - (int32)pts[best_k].y);
                float v2x = (float)((int32)pts[j].x - (int32)pts[best_k].x);
                float v2y = (float)((int32)pts[j].y - (int32)pts[best_k].y);
                float arm1 = sqrtf(v1x*v1x + v1y*v1y);
                float arm2 = sqrtf(v2x*v2x + v2y*v2y);
                float angle_deg = 0;
                if (arm1 > 0.5f && arm2 > 0.5f)
                {
                    float cos_a = (v1x*v2x + v1y*v2y) / (arm1 * arm2);
                    if (cos_a > 1.0f) cos_a = 1.0f;
                    if (cos_a < -1.0f) cos_a = -1.0f;
                    angle_deg = acosf(cos_a) * 57.2957795f;
                }
                if (angle_deg < (float)CAR_MARK_MIN_ANGLE_DEG ||
                    angle_deg > (float)CAR_MARK_MAX_ANGLE_DEG) continue;

                // 评分: base × height × 对称性(arm1/arm2 比值接近1)
                float sym = (arm1 < arm2) ? (arm1 / arm2) : (arm2 / arm1);
                if (sym < 0.3f) continue;  // 严重不对称, 跳过
                uint32 score = (uint32)(base_len_f * best_h * sym * 10.0f);

                if (score > best_score)
                {
                    best_score = score;
                    best_base1_idx = (uint16)i;
                    best_base2_idx = (uint16)j;
                    best_vertex_idx = (uint16)best_k;
                    best_h_times10 = (uint16)(best_h * 10.0f + 0.5f);
                }
            }
        }

        // 如果暴力搜索成功, 对最佳候选做连线检验
        if (best_score > 0)
        {
            Point bv = pts[best_vertex_idx];
            Point bb1 = pts[best_base1_idx];
            Point bb2 = pts[best_base2_idx];

            // 顶点→底点连线应在 V 形内部 (高 blob 占比)
            uint8 pct_v1 = line_blob_pct(bv, bb1);
            uint8 pct_v2 = line_blob_pct(bv, bb2);

            if (pct_v1 >= 35 && pct_v2 >= 35)
            {
                uint16 base_mid_x = ((uint16)bv.x + bb1.x + bb2.x) / 3;
                uint16 base_mid_y = ((uint16)bv.y + bb1.y + bb2.y) / 3;
                float bbase_len = sqrtf((float)((int32)bb2.x-(int32)bb1.x)*((int32)bb2.x-(int32)bb1.x) +
                                        (float)((int32)bb2.y-(int32)bb1.y)*((int32)bb2.y-(int32)bb1.y));
                float v1x_f = (float)((int32)bb1.x-(int32)bv.x);
                float v1y_f = (float)((int32)bb1.y-(int32)bv.y);
                float v2x_f = (float)((int32)bb2.x-(int32)bv.x);
                float v2y_f = (float)((int32)bb2.y-(int32)bv.y);
                float arm1_f = sqrtf(v1x_f*v1x_f+v1y_f*v1y_f);
                float arm2_f = sqrtf(v2x_f*v2x_f+v2y_f*v2y_f);
                float dot_f = v1x_f*v2x_f + v1y_f*v2y_f;
                float cos_a_f = 1.0f;
                if(arm1_f>0.5f && arm2_f>0.5f) { cos_a_f = dot_f/(arm1_f*arm2_f);
                    if(cos_a_f>1)cos_a_f=1;if(cos_a_f<-1)cos_a_f=-1; }
                float angle_deg_f = acosf(cos_a_f)*57.2957795f;
                float height_f = (float)best_h_times10 / 10.0f;

                b->marker_base1_x = bb1.x; b->marker_base1_y = bb1.y;
                b->marker_base2_x = bb2.x; b->marker_base2_y = bb2.y;
                b->marker_vertex_x = bv.x; b->marker_vertex_y = bv.y;
                b->marker_base_mid_x = base_mid_x; b->marker_base_mid_y = base_mid_y;
                b->marker_base_len = (uint16)(bbase_len+0.5f);
                b->marker_height = (uint16)(height_f+0.5f);
                b->marker_angle_deg = (uint16)(angle_deg_f+0.5f);
                b->marker_arm1_len = (uint16)(arm1_f+0.5f);
                b->marker_arm2_len = (uint16)(arm2_f+0.5f);
                b->marker_heading = atan2f((float)((int32)bv.y-(int32)base_mid_y),
                                           (float)((int32)bv.x-(int32)base_mid_x));
            }
        }
    }

    // 保存原始 (未平滑的) 坐标, 供 camera_process 在分类后统一做 EMA 平滑
    // EMA 放在 camera_process 中, 只对最终确定的 BLOB_CAR_MARKER 做平滑
    // 避免过早在 blob 循环中用 static 变量污染不同 blob 的坐标

    // ★★★ 凸包面积比: 凹形(<85) → V形, 凸形(>90) → 圆形/信标
    {
        uint32 hull_area = convex_hull_area(pts, count);
        uint32 cr = 100;
        if (hull_area > b->area) {
            cr = (uint32)b->area * 100 / hull_area;
            if (cr > 100) cr = 100;
        }
        b->marker_convex_ratio = (uint8)cr;
    }

    // ★★★ 轮廓距离标准差比: V形轮廓点到重心距离分散(>35), 圆形集中(<20)
    {
        float mean_d = 0.0f;
        for (uint32 p = 0; p < count; p++) {
            float dx = (float)((int32)pts[p].x - (int32)b->cx);
            float dy = (float)((int32)pts[p].y - (int32)b->cy);
            mean_d += sqrtf(dx*dx + dy*dy);
        }
        mean_d /= (float)count;
        float var_d = 0.0f;
        if (mean_d > 1.0f) {
            for (uint32 p = 0; p < count; p++) {
                float dx = (float)((int32)pts[p].x - (int32)b->cx);
                float dy = (float)((int32)pts[p].y - (int32)b->cy);
                float d = sqrtf(dx*dx + dy*dy);
                float diff = d - mean_d;
                var_d += diff * diff;
            }
            var_d /= (float)count;
            float std_d = sqrtf(var_d);
            uint8 sr = (uint8)(std_d / mean_d * 100.0f + 0.5f);
            if (sr > 100) sr = 100;
            b->marker_dist_std_ratio = sr;
        } else {
            b->marker_dist_std_ratio = 0;
        }
    }

    // ★★★ 轮廓凸度统计: 每个轮廓点到凸包边的距离
    {
        // pts 已被 convex_hull_area 排序, 用 Andrew monotone chain 求凸包
        uint32 n = count;
        // 去重
        uint32 nd = 0;
        for (uint32 i = 0; i < n; i++) {
            if (i == 0 || pts[i].x != pts[i-1].x || pts[i].y != pts[i-1].y)
                pts[nd++] = pts[i];
        }
        n = nd;
        static Point hull[256];
        uint32 h = 0;
        if (n >= 3) {
            for (uint32 i = 0; i < n; i++) {
                while (h >= 2) {
                    int32 cx = (int32)hull[h-1].x - (int32)hull[h-2].x;
                    int32 cy = (int32)hull[h-1].y - (int32)hull[h-2].y;
                    int32 nx = (int32)pts[i].x - (int32)hull[h-2].x;
                    int32 ny = (int32)pts[i].y - (int32)hull[h-2].y;
                    if (cx*ny - cy*nx <= 0) h--; else break;
                }
                hull[h++] = pts[i];
            }
            uint32 lower = h;
            for (uint32 i = n-1; i > 0; i--) {
                while (h > lower) {
                    int32 cx = (int32)hull[h-1].x - (int32)hull[h-2].x;
                    int32 cy = (int32)hull[h-1].y - (int32)hull[h-2].y;
                    int32 nx = (int32)pts[i].x - (int32)hull[h-2].x;
                    int32 ny = (int32)pts[i].y - (int32)hull[h-2].y;
                    if (cx*ny - cy*nx <= 0) h--; else break;
                }
                hull[h++] = pts[i];
            }
        }

        // 对每个轮廓点, 找最近凸包边的距离
        float sum_def = 0.0f;
        float max_def = 0.0f;
        uint32 gt2_cnt = 0;
        for (uint32 p = 0; p < count; p++) {
            float min_d = 999999.0f;
            for (uint32 e = 0, j = 1; e < h-1; e++, j = (j+1 < h-1) ? j+1 : 0) {
                float ax = (float)hull[e].x, ay = (float)hull[e].y;
                float bx = (float)hull[j].x, by = (float)hull[j].y;
                float px = (float)pts[p].x, py = (float)pts[p].y;
                float ex = bx - ax, ey = by - ay;
                float len2 = ex*ex + ey*ey;
                if (len2 < 0.5f) continue;
                float t = ((px-ax)*ex + (py-ay)*ey) / len2;
                if (t < 0.0f) t = 0.0f;
                if (t > 1.0f) t = 1.0f;
                float near_x = ax + t * ex, near_y = ay + t * ey;
                float dx = px - near_x, dy = py - near_y;
                float d = sqrtf(dx*dx + dy*dy);
                if (d < min_d) min_d = d;
            }
            sum_def += min_d;
            if (min_d > max_def) max_def = min_d;
            if (min_d > 2.0f) gt2_cnt++;
        }

        float mean_def = sum_def / (float)count;
        // 归一化到 0-100: max_def<1px→0, 否则按比例
        b->dbg_conv_mean_def = (uint8)((mean_def > 0.5f) ? (mean_def * 10.0f) : 0.0f);
        if (b->dbg_conv_mean_def > 100) b->dbg_conv_mean_def = 100;
        b->dbg_conv_max_def = (uint8)((max_def > 0.5f) ? (max_def * 10.0f) : 0.0f);
        if (b->dbg_conv_max_def > 100) b->dbg_conv_max_def = 100;
        b->dbg_conv_gt2_pct = (uint8)((uint32)gt2_cnt * 100 / count);
    }
}

// V形车标调试绘制：在图像上画出角点、底边端点、底边中点
static void draw_marker_debug(Blob *b)
{
#if CAR_MARK_DEBUG_DRAW
    int16 ax = (int16)b->marker_vertex_x;
    int16 ay = (int16)b->marker_vertex_y;
    int16 bx = (int16)b->marker_base1_x;
    int16 by = (int16)b->marker_base1_y;
    int16 cx = (int16)b->marker_base2_x;
    int16 cy = (int16)b->marker_base2_y;
    int16 dx = (int16)b->marker_base_mid_x;
    int16 dy = (int16)b->marker_base_mid_y;

    // 角点 A：白色方框 size 4
    draw_white_box((uint16)ax, (uint16)ay, 4);
    // 底边端点 B：白色方框 size 3
    draw_white_box((uint16)bx, (uint16)by, 3);
    // 底边端点 C：白色方框 size 3
    draw_white_box((uint16)cx, (uint16)cy, 3);
    // 底边中点 D：黑色十字
    draw_black_cross((uint16)dx, (uint16)dy);
#endif
}


/*
 * =============================================================================
 * extract_blobs - 连通域提取 (BFS) 与 Blob 基础属性计算
 * =============================================================================
 * 功能: 对二值图像进行4邻域BFS连通域标记，提取每个Blob的:
 *   - 质心 (cx, cy): 像素坐标均值
 *   - 外接矩形 (width, height): 轴对齐
 *   - 面积 (area): 像素计数
 *   - 核心高亮像素数 (core_count): 原始灰度图中超过IR_CORE_THRESHOLD的像素数
 *   - 方向矩 (owidth, oheight): 通过图像二阶矩计算主轴方向，
 *     再沿主轴方向投影得到旋转不变的外接矩形
 *
 * 该函数只做"提取"，不做分类，分类由后续软判决评分系统完成
 * =============================================================================
 */
static void extract_blobs(void)
{
    const int8 dx[4] = {1, -1, 0, 0};
    const int8 dy[4] = {0, 0, 1, -1};

    blob_num = 0;
    memset(visited, 0, sizeof(visited));

    for(uint16 y = 0; y < CAMERA_H; y++)
    {
        for(uint16 x = 0; x < CAMERA_W; x++)
        {
            if(blob_num >= MAX_BLOBS) return;
            if(visited[y][x] || image_buffer[y][x] == 0) continue;

            uint32 head = 0, tail = 1;
            uint32 x_sum = 0, y_sum = 0;
            uint16 min_x = x, max_x = x;
            uint16 min_y = y, max_y = y;
            uint16 core_count = 0;

            visited[y][x] = 1;
            queue_buf[0].x = x;
            queue_buf[0].y = y;

            while(head < tail)
            {
                uint16 cx = queue_buf[head].x;
                uint16 cy = queue_buf[head].y;
                head++;

                x_sum += cx;
                y_sum += cy;

                // 统计核心高亮像素（依靠原始灰度图）
                if(((uint8 (*)[MT9V03X_W])0x28026024)[cy][cx] >= IR_CORE_THRESHOLD)
                    core_count++;

                if(cx < min_x) min_x = cx;
                if(cx > max_x) max_x = cx;
                if(cy < min_y) min_y = cy;
                if(cy > max_y) max_y = cy;

                for(uint8 k = 0; k < 4; k++)
                {
                    int16 nx = cx + dx[k];
                    int16 ny = cy + dy[k];

                    if(nx < 0 || ny < 0 || nx >= CAMERA_W || ny >= CAMERA_H) continue;
                    if(visited[ny][nx] || image_buffer[ny][nx] == 0) continue;

                    visited[ny][nx] = 1;
                    queue_buf[tail].x = nx;
                    queue_buf[tail].y = ny;
                    tail++;
                }
            }

            if(tail < MIN_IR_PIXEL_COUNT) continue;
            
            // 边界剔除已移除，尾灯容易贴边被误删

            Blob *b = &blobs[blob_num];
            b->cx = x_sum / tail;
            b->cy = y_sum / tail;
            b->width = max_x - min_x + 1;
            b->height = max_y - min_y + 1;
            b->area = tail;
            b->core_count = core_count;
            b->type = BLOB_UNKNOWN;
            b->raw_beacon_score = 0;
            b->raw_marker_score = 0;
            b->filt_beacon_score = 0;
            b->filt_marker_score = 0;
            b->track_id = 0;

            if(b->height == 0) continue;

            // 带方向的最小外接矩形：用图像矩算主轴方向，投影求旋转不变的宽高比
            {
                float mx = (float)b->cx;
                float my = (float)b->cy;
                float Ixx = 0.0f, Iyy = 0.0f, Ixy = 0.0f;
                for(uint32 p = 0; p < tail; p++)
                {
                    float dx = (float)queue_buf[p].x - mx;
                    float dy = (float)queue_buf[p].y - my;
                    Ixx += dx * dx;
                    Iyy += dy * dy;
                    Ixy += dx * dy;
                }
                float theta = 0.5f * atan2f(2.0f * Ixy, Ixx - Iyy);
                b->theta = theta;           // 保存主轴角度供碎片配对
                float ct = cosf(theta);
                float st = sinf(theta);

                float min_u = 0.0f, max_u = 0.0f, min_v = 0.0f, max_v = 0.0f;
                for(uint32 p = 0; p < tail; p++)
                {
                    float dx = (float)queue_buf[p].x - mx;
                    float dy = (float)queue_buf[p].y - my;
                    float u = dx * ct + dy * st;
                    float v = -dx * st + dy * ct;
                    if(p == 0 || u < min_u) min_u = u;
                    if(p == 0 || u > max_u) max_u = u;
                    if(p == 0 || v < min_v) min_v = v;
                    if(p == 0 || v > max_v) max_v = v;
                }
                b->owidth  = (uint16)(max_u - min_u + 1.0f);
                b->oheight = (uint16)(max_v - min_v + 1.0f);
            }

            // 提取 V 形车标几何特征
            compute_car_marker_feature(b, queue_buf, tail);

            // 不在此处做硬分类，类型由软判决系统统一确定
            blob_num++;
        }
    }
}

// ==================== 软判决评分系统 ====================

// 为当前帧所有 blob 计算原始置信度分数
static void compute_raw_scores(void)
{
    for(uint8 i = 0; i < blob_num; i++)
    {
        // ★★★ 先把宽高比算出来, 供后续圆形判别用
        int32 oratio = (blobs[i].oheight > 0)
            ? (int32)blobs[i].owidth * 100 / (int32)blobs[i].oheight : 100;
        int32 ord = (oratio > 100) ? (oratio - 100) : (100 - oratio);

        int32 ratio = (blobs[i].height > 0)
            ? (int32)blobs[i].width * 100 / (int32)blobs[i].height
            : 100;
        int32 rd = (ratio > 100) ? (ratio - 100) : (100 - ratio);

        uint16 area = blobs[i].area;
        uint16 brightness = (area > 0) ? (blobs[i].core_count * 100 / area) : 0;

        // ---- 信标分：面积适中 + 圆形 ----
        int32 bs = 0;
        {   // 面积 (0~50)：峰值在 20~50，超过 100 大幅衰减
            int32 a = (area >= 10) ? (int32)area : 10;
            if(a > 150) a = 150;
            if(a <= 50)
                bs += a * 50 / 50;               // 10→10, 30→30, 50→50
            else if(a <= 100)
                bs += 50 - (a - 50) * 20 / 50;   // 50→50, 100→30
            else
                bs += 30 - (a - 100) * 30 / 50;  // 100→30, 150→0
        }
        {   // 圆度 (0~50)
            int32 rnd = 100 - rd;
            if(rnd < 10) rnd = 10;
            bs += (rnd - 10) * 50 / 90;
        }

        // 长条惩罚：ord>20 时信标分快速衰减，ord≥45 归零
        // 真正信标灯接近圆形，长条(如 V 形碎片)不可能是信标
        if(ord > 20)
        {
            int32 factor = (ord >= 45) ? 0 : (45 - ord);  // 20→25, 45→0 (原 60→0)
            bs = bs * factor / 25;                        // 衰减更陡 (原 /30)
        }

        // 亮度惩罚：低亮度不可能是信标，brightness<40 时线性衰减
        if(brightness < 40)
        {
            bs = bs * (int32)brightness / 40;
        }

        // 凹形惩罚: 凸包比<90 的 blob 有 V 形特征, 不可能是纯信标
        {
            uint8 cr = blobs[i].marker_convex_ratio;
            if (cr > 0 && cr < 90) {
                bs = bs * (int32)cr / 90;
            }
        }

        // ★★★ 距离标准差惩罚: V形轮廓点分散(>35), 不可能是纯信标
        {
            uint8 sr = blobs[i].marker_dist_std_ratio;
            if (sr > 35 && sr <= 100) {
                int32 factor = (100 - sr) * 100 / 65;   // 35→100, 100→0
                bs = bs * factor / 100;
            }
        }

        blobs[i].raw_beacon_score = (uint16)bs;

        // ---- 车标分 (V形) ----
        {
            uint16 ms = 0;

            // 面积分 (0~30)：峰值在理想面积区间
            if(area >= 80 && area <= 300)
                ms += 30;
            else if(area > 300 && area <= CAR_MARK_MAX_AREA)
                ms += 30 - (area - 300) * 30 / (CAR_MARK_MAX_AREA - 300);
            else if(area < 80 && area >= CAR_MARK_MIN_AREA)
                ms += area * 30 / 80;

            // 角度分 (0~30)：峰值在 CAR_MARK_ANGLE_CENTER_DEG (90°)
            {
                uint16 angle = blobs[i].marker_angle_deg;
                if(angle >= CAR_MARK_MIN_ANGLE_DEG && angle <= CAR_MARK_MAX_ANGLE_DEG)
                {
                    int32 diff = (angle > CAR_MARK_ANGLE_CENTER_DEG)
                        ? (angle - CAR_MARK_ANGLE_CENTER_DEG)
                        : (CAR_MARK_ANGLE_CENTER_DEG - angle);
                    ms += 30 - diff * 30 / 50;
                }
            }

            // 对称性分 (0~20)：两臂长度越接近分越高
            {
                uint16 arm1 = blobs[i].marker_arm1_len;
                uint16 arm2 = blobs[i].marker_arm2_len;
                if(arm1 > 0 && arm2 > 0)
                {
                    uint16 longer = (arm1 > arm2) ? arm1 : arm2;
                    uint16 shorter = (arm1 > arm2) ? arm2 : arm1;
                    uint16 ratio = shorter * 100 / longer;
                    if(ratio >= 70)      ms += 20;
                    else if(ratio >= 40) ms += (ratio - 40) * 20 / 30;
                }
            }

            // 高度分 (0~20)：顶点距离底边越远 V 形越明显
            {
                uint16 h = blobs[i].marker_height;
                if(h >= CAR_MARK_MIN_HEIGHT * 3)     ms += 20;
                else if(h >= CAR_MARK_MIN_HEIGHT)    ms += (h - CAR_MARK_MIN_HEIGHT) * 20 / (CAR_MARK_MIN_HEIGHT * 2);
            }

            // 凸包比加分 (0~20)：凹形(凸包比小) → V 形, 凸形(大) → 圆形
            {
                uint8 cr = blobs[i].marker_convex_ratio;
                if (cr > 0) {
                    if      (cr < 60) ms += 20;   // 深度凹形
                    else if (cr < 70) ms += 18;
                    else if (cr < 80) ms += 14;
                    else if (cr < 85) ms += 10;
                    else if (cr < 90) ms += 6;
                    else if (cr < 95) ms += 3;
                }
            }

            // ★★★ 距离标准差加分 (0~30)：轮廓点分散 → V 形, 集中 → 圆形
            {
                uint8 sr = blobs[i].marker_dist_std_ratio;
                if (sr > 0) {
                    if      (sr >= 70) ms += 30;   // 极分散: 明确V形(顶点近+底角远)
                    else if (sr >= 55) ms += 25;
                    else if (sr >= 40) ms += 15;
                    else if (sr >= 30) ms += 8;
                    else if (sr >= 20) ms += 3;
                }
            }

            blobs[i].raw_marker_score = ms;

            // 圆形惩罚：真圆形不可能是 V 形车标
            // 但如果 BFS/暴力搜索已经找到 V 形 (marker_angle_deg>0), 说明 blob 不是真圆,
            // 跳过惩罚, 避免已检出 V 形却被信标分反超
            if (blobs[i].marker_angle_deg == 0)
            {
                if(ord < 20)
                {
                    blobs[i].raw_marker_score = 0;
                }
                else if(ord < 40)
                {
                    blobs[i].raw_marker_score = (uint16)(ms * (ord - 20) / 20);
                }
            }
            // else: V 形已检出, 保留原始 ms, 不受圆形惩罚
        }

        // ★★★ 先判信标: 圆形(宽高比<1.25:1)且信标分≥40的 blob 强制清零 marker 分
        // 即使 BFS/霍夫找到了假 V 形(marker_angle_deg>0), 也不参与车标竞争
        if (ord < 25 && blobs[i].raw_beacon_score >= 40)
        {
            blobs[i].raw_marker_score = 0;
        }

    }
}

// 跨帧跟踪：将当前帧 blob 与上一帧匹配，继承历史分数
static void cross_frame_track(void)
{
    uint8 matched[MAX_BLOBS] = {0};
    static uint8 next_track_id = 1;

    for(uint8 j = 0; j < blob_num; j++)
        blobs[j].track_id = 0;

    if(blob_num == 0) { prev_track_num = 0; return; }

    for(uint8 i = 0; i < prev_track_num; i++)
    {
        uint8 best_j = 0xFF;
        uint32 best_d2 = TRACK_MATCH_MAX_DIST2 + 1;
        for(uint8 j = 0; j < blob_num; j++)
        {
            if(matched[j]) continue;
            int32 dx = (int32)blobs[j].cx - (int32)prev_tracks[i].cx;
            int32 dy = (int32)blobs[j].cy - (int32)prev_tracks[i].cy;
            uint32 d2 = dx * dx + dy * dy;
            if(d2 < best_d2) { best_d2 = d2; best_j = j; }
        }
        if(best_j != 0xFF)
        {
            blobs[best_j].track_id           = prev_tracks[i].track_id;
            blobs[best_j].filt_beacon_score  = prev_tracks[i].filt_beacon_score;
            blobs[best_j].filt_marker_score  = prev_tracks[i].filt_marker_score;
            matched[best_j] = 1;
        }
    }

    // 未匹配的新 blob：分配唯一递增 id，用 raw 分种子 filt
    for(uint8 j = 0; j < blob_num; j++)
    {
        if(!matched[j])
        {
            blobs[j].track_id = next_track_id++;
            if(next_track_id == 0) next_track_id = 1;
            blobs[j].filt_beacon_score = blobs[j].raw_beacon_score;
            blobs[j].filt_marker_score = blobs[j].raw_marker_score;
        }
    }
}

// 删除 seed_new_blobs，功能已合并到 cross_frame_track
#define seed_new_blobs() ((void)0)

// 保存当前帧到 prev_tracks（在类型判决后调用）
static void save_tracks(void)
{
    prev_track_num = (blob_num > MAX_TRACKS) ? MAX_TRACKS : blob_num;
    for(uint8 j = 0; j < prev_track_num; j++)
    {
        prev_tracks[j] = blobs[j];
    }
}

// 分数平滑与最终类型判决
 static void smooth_and_decide(void)
 {
     // 一阶滞后滤波 (保留给下游 EMA 平滑使用)
     for(uint8 i = 0; i < blob_num; i++)
     {
         blobs[i].filt_beacon_score = (uint16)(
             SCORE_HISTORY_WEIGHT * (float)blobs[i].filt_beacon_score +
             SCORE_CURRENT_WEIGHT * (float)blobs[i].raw_beacon_score);
         blobs[i].filt_marker_score = (uint16)(
             SCORE_HISTORY_WEIGHT * (float)blobs[i].filt_marker_score +
             SCORE_CURRENT_WEIGHT * (float)blobs[i].raw_marker_score);
     }

     // ★★★ 凸包法分类: 凸包面积比最小的 blob 就是 V 形车标
     //     圆形透镜/信标的凸包比接近 100%, V 形明显凹陷(<85%)
     {
         uint8 car_idx = 0xFF;
         uint8 min_cr = 100;

         for(uint8 i = 0; i < blob_num; i++)
         {
             uint8 cr = blobs[i].marker_convex_ratio;
             if(cr < min_cr && blobs[i].area >= CAR_MARK_MIN_AREA)
             {
                 min_cr = cr;
                 car_idx = i;
             }
         }

         // 标记类型: 凸包比<95 的最凹 blob → 车标, 其余都是信标
         // (信标凸包比=100%, V形=82~88%, 阈值95安全)
         for(uint8 i = 0; i < blob_num; i++)
         {
             if(i == car_idx && min_cr < 95)
                 blobs[i].type = BLOB_CAR_MARKER;
             else
                 blobs[i].type = BLOB_BEACON;
         }
     }
 }

// ==================== 旧逻辑保留区域 ====================

// V形车标姿态解算：从 BLOB_CAR_MARKER 获取车中心和航向
static uint8 find_car_pose_by_marker(uint8 *car_x, uint8 *car_y, float *heading, Blob **marker_blob)
{
    Blob *best = NULL;
    uint16 best_score = CAR_MARK_MIN_SCORE;

    for(uint8 i = 0; i < blob_num; i++)
    {
        if(blobs[i].type != BLOB_CAR_MARKER) continue;
        uint16 s = (blobs[i].track_id == 0)
            ? blobs[i].raw_marker_score : blobs[i].filt_marker_score;
        if(s > best_score)
        {
            best_score = s;
            best = &blobs[i];
        }
    }

    if(best == NULL) return 0;

    // 车中心 = 底边中点 D
    *car_x = (uint8)best->marker_base_mid_x;
    *car_y = (uint8)best->marker_base_mid_y;

    // 航向 = D → A（角点方向即车头方向）
    *heading = best->marker_heading;

    *marker_blob = best;
    return 1;
}

void camera_init(void)
{
    // ===== 初始化摄像头所有硬件引脚 =====
    // 数据引脚 P18_0 ~ P18_7（SmartIO并行数据捕获）
    gpio_init(P18_0, GPI, 0, GPI_FLOATING_IN);
    gpio_init(P18_1, GPI, 0, GPI_FLOATING_IN);
    gpio_init(P18_2, GPI, 0, GPI_FLOATING_IN);
    gpio_init(P18_3, GPI, 0, GPI_FLOATING_IN);
    gpio_init(P18_4, GPI, 0, GPI_FLOATING_IN);
    gpio_init(P18_5, GPI, 0, GPI_FLOATING_IN);
    gpio_init(P18_6, GPI, 0, GPI_FLOATING_IN);
    gpio_init(P18_7, GPI, 0, GPI_FLOATING_IN);
    // PCLK 和 VSYNC（TCPWM 捕获触发）
    gpio_init(P06_5, GPI, 0, GPI_FLOATING_IN);
    gpio_init(P06_6, GPI, 0, GPI_FLOATING_IN);

    gpio_init(CAMERA_STATUS_LED, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART); // 逐飞助手初始化
    
    while (mt9v03x_init())
    {
        gpio_toggle_level(CAMERA_STATUS_LED);
        system_delay_ms(500);
    }
    #if VISION_ENABLE_IPS_DISPLAY
    ips200_init(IPS200_TYPE_SPI);
    ips200_clear();
#endif
}


// V 形碎片配对: 将两条朝向相反的细长碎块拼回完整 V 形
// 场景: V 形头部被遮挡, 分裂为两个独立的长条碎片
// 每片单独检测不到 V 形, 但组合起来就是 V 形的两条臂
static void pair_v_fragments(void)
{
    // 碎片特征: 带方向外接矩形的长宽比 > 2.5 且不低于最小面积
    #define FRAG_MIN_ELONGATION 150    // 1.5x 长宽比 (原250→180，再降到150以匹配断裂V形残片)
    #define FRAG_MAX_PAIR_DIST  120    // 两碎片质心最大距离 (像素)
    #define FRAG_MIN_OVERLAP_Y   12    // 两碎片在 Y 方向的最小重叠 (原20, 降低以匹配短碎片)

    for(uint8 i = 0; i < blob_num; i++)
    {
        if(blobs[i].type != BLOB_UNKNOWN) continue;

        uint16 oh = (blobs[i].oheight > 0) ? blobs[i].oheight : 1;
        uint16 olen_ratio = (blobs[i].owidth > oh)
            ? (blobs[i].owidth * 100 / oh)
            : (oh * 100 / blobs[i].owidth);
        if(olen_ratio < FRAG_MIN_ELONGATION) continue;
        if(blobs[i].area < CAR_MARK_MIN_AREA) continue;

        for(uint8 j = i + 1; j < blob_num; j++)
        {
            if(blobs[j].type != BLOB_UNKNOWN) continue;

            uint16 oh2 = (blobs[j].oheight > 0) ? blobs[j].oheight : 1;
            uint16 olen_ratio2 = (blobs[j].owidth > oh2)
                ? (blobs[j].owidth * 100 / oh2)
                : (oh2 * 100 / blobs[j].owidth);
            if(olen_ratio2 < FRAG_MIN_ELONGATION) continue;
            if(blobs[j].area < CAR_MARK_MIN_AREA) continue;

            int32 dx = (int32)blobs[j].cx - (int32)blobs[i].cx;
            int32 dy = (int32)blobs[j].cy - (int32)blobs[i].cy;
            uint32 dist2 = dx * dx + dy * dy;
            if(dist2 > FRAG_MAX_PAIR_DIST * FRAG_MAX_PAIR_DIST) continue;

            int16 i_y1 = (int16)blobs[i].cy - (int16)(blobs[i].oheight/2);
            int16 i_y2 = (int16)blobs[i].cy + (int16)(blobs[i].oheight/2);
            int16 j_y1 = (int16)blobs[j].cy - (int16)(blobs[j].oheight/2);
            int16 j_y2 = (int16)blobs[j].cy + (int16)(blobs[j].oheight/2);
            if(i_y1 > j_y2 || j_y1 > i_y2) continue;
            int16 overlap_y = (i_y2 < j_y2 ? i_y2 : j_y2) - (i_y1 > j_y1 ? i_y1 : j_y1);
            if(overlap_y < (int16)FRAG_MIN_OVERLAP_Y) continue;

            float t1 = blobs[i].theta;
            float t2 = blobs[j].theta;
            if((t1 > 0 && t2 > 0) || (t1 < 0 && t2 < 0)) continue;

#if DEBUG_SCORES
            printf("PAIR: trying pair i=%d(%d,%d area=%d th=%.2f) j=%d(%d,%d area=%d th=%.2f)\n",
                   i, blobs[i].cx, blobs[i].cy, blobs[i].area, (double)t1,
                   j, blobs[j].cx, blobs[j].cy, blobs[j].area, (double)t2);
#endif

            float cos1 = cosf(t1), sin1 = sinf(t1);
            float cos2 = cosf(t2), sin2 = sinf(t2);
            float rho1 = (float)blobs[i].cx * cos1 + (float)blobs[i].cy * sin1;
            float rho2 = (float)blobs[j].cx * cos2 + (float)blobs[j].cy * sin2;

            float det = cos1 * sin2 - cos2 * sin1;
            if(fabsf(det) < 0.001f) continue;

            float ax = (rho1 * sin2 - rho2 * sin1) / det;
            float ay = (rho2 * cos1 - rho1 * cos2) / det;
            if(ax < 0 || ax >= (float)CAMERA_W) continue;
            if(ay < 0 || ay >= (float)CAMERA_H) continue;
            float min_y = (float)(blobs[i].cy < blobs[j].cy ? blobs[i].cy : blobs[j].cy);
            if(ay > min_y * 1.3f)  // 顶点必须显著高于 (y值更小) 两个碎片质心
            {
#if DEBUG_SCORES
                printf("PAIR: fail vertex_y=%.0f > min_cy*1.3=%.0f\n",
                       (double)ay, (double)(min_y*1.3));
#endif
                continue;
            }

            // 用两个碎片的底部均值作为 y_ref, 比固定用 CAMERA_H-1 更鲁棒
            float y_ref = (float)((blobs[i].cy + blobs[i].oheight/2) +
                                  (blobs[j].cy + blobs[j].oheight/2)) * 0.5f;
            if(y_ref <= ay) y_ref = ay + 2.0f;
            if(y_ref >= (float)CAMERA_H) y_ref = (float)CAMERA_H - 1.0f;

            float bx = rho1 / cos1 - y_ref * sin1 / cos1;
            float cx = rho2 / cos2 - y_ref * sin2 / cos2;
            float base_w = fabsf(bx - cx);
            if(base_w < 5.0f || base_w > (float)CAMERA_W * 0.8f)
            {
#if DEBUG_SCORES
                printf("PAIR: fail base_w=%.0f\n", (double)base_w);
#endif
                continue;
            }

            float arm1 = sqrtf((ax - bx)*(ax - bx) + (ay - y_ref)*(ay - y_ref));
            float arm2 = sqrtf((ax - cx)*(ax - cx) + (ay - y_ref)*(ay - y_ref));
            // 臂长检查 (暂时注释)
            // if(arm1 < (float)CAR_MARK_MIN_ARM_LEN || arm2 < (float)CAR_MARK_MIN_ARM_LEN)
            // {
            //     printf("PAIR: fail arm1=%.0f arm2=%.0f (min=%d)\n",
            //            (double)arm1, (double)arm2, CAR_MARK_MIN_ARM_LEN);
            //     continue;
            // }

            float dot = (bx - ax)*(cx - ax) + (y_ref - ay)*(y_ref - ay);
            float cos_a = dot / (arm1 * arm2);
            if(cos_a > 1.0f) cos_a = 1.0f; if(cos_a < -1.0f) cos_a = -1.0f;
            float angle_deg = acosf(cos_a) * 57.2957795f;
            if(angle_deg < (float)CAR_MARK_MIN_ANGLE_DEG ||
               angle_deg > (float)CAR_MARK_MAX_ANGLE_DEG)
            {
#if DEBUG_SCORES
                printf("PAIR: fail angle=%.0f [%d,%d]\n",
                       (double)angle_deg, CAR_MARK_MIN_ANGLE_DEG, CAR_MARK_MAX_ANGLE_DEG);
#endif
                continue;
            }

            // ===== 配对成功 =====
            blobs[i].type = BLOB_CAR_MARKER;
            blobs[j].type = BLOB_CAR_MARKER;

            float mid_x_f = (ax + bx + cx) / 3.0f;
            float mid_y_f = (ay + y_ref + y_ref) / 3.0f;
            float heading = atan2f(ay - mid_y_f, ax - mid_x_f);

            blobs[i].marker_vertex_x   = (uint16)(ax + 0.5f);
            blobs[i].marker_vertex_y   = (uint16)(ay + 0.5f);
            blobs[i].marker_base1_x    = (uint16)(bx + 0.5f);
            blobs[i].marker_base1_y    = (uint16)(y_ref + 0.5f);
            blobs[i].marker_base2_x    = (uint16)(cx + 0.5f);
            blobs[i].marker_base2_y    = (uint16)(y_ref + 0.5f);
            blobs[i].marker_base_mid_x = (uint16)(mid_x_f + 0.5f);
            blobs[i].marker_base_mid_y = (uint16)(mid_y_f + 0.5f);
            blobs[i].marker_base_len   = (uint16)(base_w + 0.5f);
            blobs[i].marker_height     = (uint16)(arm1 * sinf(acosf(cos_a)) + 0.5f);
            blobs[i].marker_angle_deg  = (uint16)(angle_deg + 0.5f);
            blobs[i].marker_arm1_len   = (uint16)(arm1 + 0.5f);
            blobs[i].marker_arm2_len   = (uint16)(arm2 + 0.5f);
            blobs[i].marker_heading    = heading;

            blobs[i].raw_beacon_score = 0;
            blobs[i].filt_beacon_score = 0;
            blobs[j].raw_beacon_score = 0;
            blobs[j].filt_beacon_score = 0;

            uint16 pair_score = 60;
            if(angle_deg >= 70 && angle_deg <= 100) pair_score += 20;
            blobs[i].raw_marker_score = pair_score;
            blobs[i].filt_marker_score = pair_score;

            break;
        }
    }
    #undef FRAG_MIN_ELONGATION
    #undef FRAG_MAX_PAIR_DIST
    #undef FRAG_MIN_OVERLAP_Y
}

// ===== 信标保护系统 =====
// 原理: 在小车 V 形外接矩形外的信标视为"真实信标", 加入保护列表
//       即使这些信标后来移动到 V 形内部, 也不会被误清除
#define MAX_PROTECTED_BEACONS 16
static uint8 protected_beacon_ids[MAX_PROTECTED_BEACONS];
static uint8 protected_beacon_count = 0;

static int is_beacon_protected(uint8 track_id)
{
    if (track_id == 0) return 0;
    for (uint8 i = 0; i < protected_beacon_count; i++)
        if (protected_beacon_ids[i] == track_id) return 1;
    return 0;
}

static void protect_beacon(uint8 track_id)
{
    if (track_id == 0) return;
    if (is_beacon_protected(track_id)) return;
    if (protected_beacon_count < MAX_PROTECTED_BEACONS)
        protected_beacon_ids[protected_beacon_count++] = track_id;
}

    
void camera_process(void)
{
    if (mt9v03x_finish_flag)
    {
        // 从DMA缓冲区读取原始图像数据（固定地址0x28026024，由zf_device_config.a中SmartIO/DMA写入）
        {
            uint8 (*dma_buf)[MT9V03X_W] = (uint8 (*)[MT9V03X_W])0x28026024;
            memcpy(image_buffer, dma_buf, MT9V03X_IMAGE_SIZE);
        }

        // 图像二值化
        for (uint16 y = 0; y < CAMERA_H; y++)
        {
            for (uint16 x = 0; x < CAMERA_W; x++)
            {
                image_buffer[y][x] = (image_buffer[y][x] >= IR_THRESHOLD) ? 255 : 0;
            }
        }

        // STEP 1: 提取连通域（不分类）
        extract_blobs();

        // STEP 2: 软判决管线
        compute_raw_scores();
        pair_v_fragments();   // ← 跨碎片 V 形配对 (在分数判决前清零信标分)
        cross_frame_track();
        seed_new_blobs();     // 无历史的用 raw 分
        smooth_and_decide();
        save_tracks();

        // V 形车标去重：至多保留一个最好的 marker
        // 优先保留分数高的；分数接近时（差 < 15），选更圆的（ord 更小，更像完整 V 形而非碎片）
        {
            uint8 best_m = 0xFF;
            uint16 best_ms = 0;
            int16 best_ord = 999;
            for(uint8 i = 0; i < blob_num; i++)
            {
                if(blobs[i].type == BLOB_CAR_MARKER)
                {
                    uint16 s = (blobs[i].track_id == 0) ? blobs[i].raw_marker_score : blobs[i].filt_marker_score;
                    int32 oratio = (blobs[i].oheight > 0)
                        ? (int32)blobs[i].owidth * 100 / (int32)blobs[i].oheight
                        : 100;
                    int32 ord = (oratio > 100) ? (oratio - 100) : (100 - oratio);
                    if(best_m == 0xFF ||
                       s > best_ms + 15 ||                              // 分数明显更高 → 替换
                       (s >= best_ms - 15 && (int16)ord < best_ord))    // 分数相近且更圆 → 替换
                    {
                        best_ms = s; best_m = i; best_ord = (int16)ord;
                    }
                }
            }
            if(best_m != 0xFF)
            {
                for(uint8 i = 0; i < blob_num; i++)
                {
                    if(blobs[i].type == BLOB_CAR_MARKER && i != best_m) blobs[i].type = BLOB_UNKNOWN;
                }
            }
        }

// ===== 帧间平滑 (在分类和去重之后, 只对最终确定的单个车标做 EMA) =====
        // 使用函数级 static 变量跨帧追踪: 检测丢失时重置 prev_init
        static float prev_vx = 0, prev_vy = 0;
        static float prev_b1x = 0, prev_b1y = 0;
        static float prev_b2x = 0, prev_b2y = 0;
        static float prev_mx = 0, prev_my = 0;
        static uint8 prev_init = 0;
        {
            Blob *mb = NULL;
            for (uint8 i = 0; i < blob_num; i++) {
                if (blobs[i].type == BLOB_CAR_MARKER) { mb = &blobs[i]; break; }
            }
            if (mb && mb->marker_angle_deg > 0)
            {
                #define BFS_EMA_ALPHA  0.85f
                #define BFS_MAX_DELTA  120.0f

                if (!prev_init) {
                    prev_vx = mb->marker_vertex_x; prev_vy = mb->marker_vertex_y;
                    prev_b1x = mb->marker_base1_x; prev_b1y = mb->marker_base1_y;
                    prev_b2x = mb->marker_base2_x; prev_b2y = mb->marker_base2_y;
                    prev_mx = mb->marker_base_mid_x; prev_my = mb->marker_base_mid_y;
                    prev_init = 1;
                } else {
                    float cvx = (float)mb->marker_vertex_x, cvy = (float)mb->marker_vertex_y;
                    float cb1x = (float)mb->marker_base1_x, cb1y = (float)mb->marker_base1_y;
                    float cb2x = (float)mb->marker_base2_x, cb2y = (float)mb->marker_base2_y;
                    float cmx = (float)mb->marker_base_mid_x, cmy = (float)mb->marker_base_mid_y;

                    cvx = prev_vx + CLAMP(cvx - prev_vx, -BFS_MAX_DELTA, BFS_MAX_DELTA);
                    cvy = prev_vy + CLAMP(cvy - prev_vy, -BFS_MAX_DELTA, BFS_MAX_DELTA);
                    cb1x = prev_b1x + CLAMP(cb1x - prev_b1x, -BFS_MAX_DELTA, BFS_MAX_DELTA);
                    cb1y = prev_b1y + CLAMP(cb1y - prev_b1y, -BFS_MAX_DELTA, BFS_MAX_DELTA);
                    cb2x = prev_b2x + CLAMP(cb2x - prev_b2x, -BFS_MAX_DELTA, BFS_MAX_DELTA);
                    cb2y = prev_b2y + CLAMP(cb2y - prev_b2y, -BFS_MAX_DELTA, BFS_MAX_DELTA);

                    // B/C 身份追踪
                    {
                        float d_same = (cb1x-prev_b1x)*(cb1x-prev_b1x) +
                                       (cb1y-prev_b1y)*(cb1y-prev_b1y) +
                                       (cb2x-prev_b2x)*(cb2x-prev_b2x) +
                                       (cb2y-prev_b2y)*(cb2y-prev_b2y);
                        float d_cross = (cb1x-prev_b2x)*(cb1x-prev_b2x) +
                                        (cb1y-prev_b2y)*(cb1y-prev_b2y) +
                                        (cb2x-prev_b1x)*(cb2x-prev_b1x) +
                                        (cb2y-prev_b1y)*(cb2y-prev_b1y);
                        if (d_same > d_cross)
                        {
                            float tmp = cb1x; cb1x = cb2x; cb2x = tmp;
                            tmp = cb1y; cb1y = cb2y; cb2y = tmp;
                            cmx = (cb1x + cb2x) * 0.5f;
                            cmy = (cb1y + cb2y) * 0.5f;
                        }
                    }
                    cmx = prev_mx + CLAMP(cmx - prev_mx, -BFS_MAX_DELTA, BFS_MAX_DELTA);
                    cmy = prev_my + CLAMP(cmy - prev_my, -BFS_MAX_DELTA, BFS_MAX_DELTA);

                    prev_vx = prev_vx * (1.0f - BFS_EMA_ALPHA) + cvx * BFS_EMA_ALPHA;
                    prev_vy = prev_vy * (1.0f - BFS_EMA_ALPHA) + cvy * BFS_EMA_ALPHA;
                    prev_b1x = prev_b1x * (1.0f - BFS_EMA_ALPHA) + cb1x * BFS_EMA_ALPHA;
                    prev_b1y = prev_b1y * (1.0f - BFS_EMA_ALPHA) + cb1y * BFS_EMA_ALPHA;
                    prev_b2x = prev_b2x * (1.0f - BFS_EMA_ALPHA) + cb2x * BFS_EMA_ALPHA;
                    prev_b2y = prev_b2y * (1.0f - BFS_EMA_ALPHA) + cb2y * BFS_EMA_ALPHA;
                    prev_mx = prev_mx * (1.0f - BFS_EMA_ALPHA) + cmx * BFS_EMA_ALPHA;
                    prev_my = prev_my * (1.0f - BFS_EMA_ALPHA) + cmy * BFS_EMA_ALPHA;

                    mb->marker_vertex_x = (uint16)(prev_vx + 0.5f);
                    mb->marker_vertex_y = (uint16)(prev_vy + 0.5f);
                    mb->marker_base1_x = (uint16)(prev_b1x + 0.5f);
                    mb->marker_base1_y = (uint16)(prev_b1y + 0.5f);
                    mb->marker_base2_x = (uint16)(prev_b2x + 0.5f);
                    mb->marker_base2_y = (uint16)(prev_b2y + 0.5f);
                    mb->marker_base_mid_x = (uint16)(prev_mx + 0.5f);
                    mb->marker_base_mid_y = (uint16)(prev_my + 0.5f);
                }
                #undef BFS_EMA_ALPHA
                #undef BFS_MAX_DELTA
            }
            else
            {
                // 没有车标被检出时重置 EMA, 下一帧检出时直接从新位置开始
                prev_init = 0;
            }
        }

        // 其余未知 blob 均视为信标
        {
            for(uint8 i = 0; i < blob_num; i++)
            {
                if(blobs[i].type == BLOB_UNKNOWN)
                {
                    blobs[i].type = BLOB_BEACON;
                }
            }
        }

        // 清除 V 形区域内的误识别信标 + 信标保护系统
        // 原理: 在 V 形外部的信标视为"真实", 加入保护列表
        //       内部信标如果不在保护列表中 → 误识别, 清除
        {
            // 记录小车自身 track_id, 防止灭灯后被误分类为 BEACON 时被清除
            static uint8 car_track_id = 0;

            Blob *car = 0;
            for(uint8 i = 0; i < blob_num; i++)
            {
                if(blobs[i].type == BLOB_CAR_MARKER)
                {
                    car = &blobs[i];
                    break;
                }
            }
            if(car)
            {
                car_track_id = car->track_id;
                if(car_track_id > 0)
                    protect_beacon(car_track_id);  // 小车 ID 永久受保护

                int margin = 15;
                int xmin = (car->cx > (uint16)margin) ? (int)(car->cx - car->width/2) - margin : 0;
                int xmax = (int)(car->cx + car->width/2) + margin;
                int ymin = (car->cy > (uint16)margin) ? (int)(car->cy - car->height/2) - margin : 0;
                int ymax = (int)(car->cy + car->height/2) + margin;

                // 第一遍: 保护外部的信标 (真实信标)
                for(uint8 i = 0; i < blob_num; i++)
                {
                    if(blobs[i].type == BLOB_BEACON)
                    {
                        int32 px = (int32)blobs[i].cx;
                        int32 py = (int32)blobs[i].cy;
                        // 在矩形外部 → 真实信标, 加入保护
                        if(px < xmin || px > xmax || py < ymin || py > ymax)
                        {
                            protect_beacon(blobs[i].track_id);
                        }
                    }
                }

                // 第二遍: 清除内部的信标, 但跳过受保护的、高分信标、或小车自身
                // 高分信标（beacon_score > 60）即使位置在内部也视为真实
                for(uint8 i = 0; i < blob_num; i++)
                {
                    if(blobs[i].type == BLOB_BEACON)
                    {
                        int32 px = (int32)blobs[i].cx;
                        int32 py = (int32)blobs[i].cy;
                        if(px >= xmin && px <= xmax &&
                           py >= ymin && py <= ymax)
                        {
                            // 受保护的 track_id → 跳过
                            if(is_beacon_protected(blobs[i].track_id))
                                continue;
                            // 小车自身 track_id → 跳过 (防灭灯误分类)
                            if(car_track_id > 0 && blobs[i].track_id == car_track_id)
                                continue;
                            // 高分信标 → 位置在内部也视为真实
                            uint16 bscore = (blobs[i].track_id == 0)
                                ? blobs[i].raw_beacon_score
                                : blobs[i].filt_beacon_score;
                            if(bscore > 60)
                                continue;
                            blobs[i].type = BLOB_UNKNOWN;
                        }
                    }
                }
            }
            else
            {
                car_track_id = 0;  // 未检测到小车, 重置
            }
        }



#if DEBUG_SCORES
        {
            // 单行紧凑输出，避免与 HC06 冲突
            printf("%d,%d", (int)g_vision_share.frame_id, (int)blob_num);
            for(uint8 d = 0; d < blob_num && d < 5; d++)
            {
                uint16 br = (blobs[d].area > 0) ? (blobs[d].core_count * 100 / blobs[d].area) : 0;
                printf(",%d,%d,%d,%d,%d,%d,%d,%d,%d",
                blobs[d].track_id, blobs[d].area, br,
                (blobs[d].oheight>0)?(int32)blobs[d].owidth*100/(int32)blobs[d].oheight:0,
                blobs[d].raw_beacon_score, blobs[d].raw_marker_score,
                blobs[d].filt_beacon_score, blobs[d].filt_marker_score,
                (int)blobs[d].type);
            }
            printf("\r\n");
        }
#endif
        
//#if DEBUG_SCORES
//        printf("%d,%d\r\n",
//            g_vision_share.err_x,
//            g_vision_share.err_y);
//#endif        

        // 跟踪 ID 用于锁住当前追踪的信标，防止在多信标间来回抽动
        static uint8 current_beacon_track_id = 0;
        
        Blob *best_beacon = NULL;
        uint8 beacon_count = 0;

        // 先统计信标数量
        for(uint8 i = 0; i < blob_num; i++)
        {
            if(blobs[i].type == BLOB_BEACON) beacon_count++;
        }

        if(beacon_count == 1)
        {
            // 单信标直接取，记住其 track_id
            for(uint8 i = 0; i < blob_num; i++)
            {
                if(blobs[i].type == BLOB_BEACON)
                {
                    best_beacon = &blobs[i];
                    current_beacon_track_id = blobs[i].track_id;
                    break;
                }
            }
        }
        else if(beacon_count > 1)
        {
            // 多信标：优先追踪已锁定 track_id 的信标，避免来回抽动
            if(current_beacon_track_id > 0)
            {
                for(uint8 i = 0; i < blob_num; i++)
                {
                    if(blobs[i].type == BLOB_BEACON && blobs[i].track_id == current_beacon_track_id)
                    {
                        best_beacon = &blobs[i];
                        break;
                    }
                }
            }
            
            // 锁定信标丢失或首次出现多信标：选最近画面中心的
            if(best_beacon == NULL)
            {
                int16 center_x = CAMERA_W / 2;
                int16 center_y = CAMERA_H / 2;
                uint32 best_d2 = 0xFFFFFFFF;
                for(uint8 i = 0; i < blob_num; i++)
                {
                    if(blobs[i].type != BLOB_BEACON) continue;
                    uint32 d2 = (int32)(blobs[i].cx - center_x) * (int32)(blobs[i].cx - center_x)
                              + (int32)(blobs[i].cy - center_y) * (int32)(blobs[i].cy - center_y);
                    if(d2 < best_d2) { best_d2 = d2; best_beacon = &blobs[i]; }
                }
                if(best_beacon != NULL)
                    current_beacon_track_id = best_beacon->track_id;
            }
        }

        // ★★★ 信标多帧确认: 防止突然出现的碎片被立即选为信标目标
        // 当车标被遮挡时, 断裂的 V 形碎片可能短暂成为 BLOB_BEACON
        // 这些碎片通常只出现 1-2 帧就被重新分类, 确认计数器可以过滤掉它们
        {
            static uint8 beacon_confirm_count = 0;
            static uint8 beacon_confirm_track_id = 0;

            if(best_beacon != NULL)
            {
                uint8 curr_tid = best_beacon->track_id;
                if(curr_tid == beacon_confirm_track_id && curr_tid > 0)
                {
                    // 同一信标持续出现 → 增加置信度
                    if(beacon_confirm_count < BEACON_CONFIRM_FRAMES)
                        beacon_confirm_count++;
                }
                else
                {
                    // 新信标出现 → 重置计数器
                    beacon_confirm_count = 0;
                    beacon_confirm_track_id = curr_tid;
                }

                // 未达到确认帧数时, 不使用该信标输出
                if(beacon_confirm_count < BEACON_CONFIRM_FRAMES)
                {
#if DEBUG_SCORES
                    printf("BEACON: track %d not confirmed (%d/%d)\n",
                           curr_tid, beacon_confirm_count, BEACON_CONFIRM_FRAMES);
#endif
                    best_beacon = NULL;
                }
            }
            else
            {
                // 当前帧无信标 → 衰减计数器
                if(beacon_confirm_count > 0) beacon_confirm_count--;
            }
        }

        // 给所有信标画十字
        for(uint8 i = 0; i < blob_num; i++)
        {
            if(blobs[i].type == BLOB_BEACON)
                draw_black_cross(blobs[i].cx, blobs[i].cy);
        }

        Blob *marker_blob = NULL;
        uint8 car_x = 0;
        uint8 car_y = 0;
        float raw_heading = 0.0f;

        // 绘制 V 形车标调试标记
        for(uint8 i = 0; i < blob_num; i++)
        {
            if(blobs[i].type == BLOB_CAR_MARKER)
            {
                draw_marker_debug(&blobs[i]);
            }
        }

        // 姿态解算：仅使用 V 形车标
        uint8 found_car = find_car_pose_by_marker(&car_x, &car_y, &raw_heading, &marker_blob);

        // 小车坐标衰减保留：检测到时更新，丢失时平滑衰减归零
        static float retain_car_x = 0.0f;
        static float retain_car_y = 0.0f;

        if(found_car)
        {
            // 绘制车中心标记
            draw_white_box(marker_blob->marker_vertex_x, marker_blob->marker_vertex_y, 4);
            draw_white_box(marker_blob->marker_base1_x, marker_blob->marker_base1_y, 3);
            draw_white_box(marker_blob->marker_base2_x, marker_blob->marker_base2_y, 3);
            draw_white_cross(car_x, car_y);
            update_marker(car_x, car_y, 1);

            float raw_cos = cosf(raw_heading);
            float raw_sin = sinf(raw_heading);

            if(is_first_heading)
            {
                filtered_cos = raw_cos;
                filtered_sin = raw_sin;
                is_first_heading = 0;
            }
            else
            {
                filtered_cos = HEADING_FILTER_ALPHA * filtered_cos + (1.0f - HEADING_FILTER_ALPHA) * raw_cos;
                filtered_sin = HEADING_FILTER_ALPHA * filtered_sin + (1.0f - HEADING_FILTER_ALPHA) * raw_sin;
            }

            g_vision_share.car_x = car_x;
            g_vision_share.car_y = car_y;
            g_vision_share.car_found = 1;
            g_vision_share.heading_angle = atan2f(filtered_sin, filtered_cos);

            // 检测到时更新保留坐标
            retain_car_x = (float)car_x;
            retain_car_y = (float)car_y;
        }
        else
        {
            // 丢失后平滑衰减归零
            retain_car_x *= CAR_RETAIN_DECAY;
            retain_car_y *= CAR_RETAIN_DECAY;
            if(fabsf(retain_car_x) < 1.0f) retain_car_x = 0.0f;
            if(fabsf(retain_car_y) < 1.0f) retain_car_y = 0.0f;

            g_vision_share.car_x = (int16)retain_car_x;
            g_vision_share.car_y = (int16)retain_car_y;
            g_vision_share.car_found = (retain_car_x != 0.0f || retain_car_y != 0.0f) ? 1 : 0;
            g_vision_share.heading_angle = 0.0f;
            is_first_heading = 1;

            if(!g_vision_share.car_found)
                update_marker(0, 0, 0);
        }

        // 评分调试数据: 写入最优小车的分数/角度/方法
        if(found_car && marker_blob != NULL)
        {
            g_vision_share.debug_car_score   = (marker_blob->track_id == 0) ?
                                                 marker_blob->raw_marker_score :
                                                 marker_blob->filt_marker_score;
            g_vision_share.debug_car_angle   = (int16)marker_blob->marker_angle_deg;
            g_vision_share.debug_car_method  = (marker_blob->raw_marker_score == 50) ? 2 : 1;
            g_vision_share.debug_car_track_id = marker_blob->track_id;
        }
        else
        {
            g_vision_share.debug_car_score   = 0;
            g_vision_share.debug_car_angle   = 0;
            g_vision_share.debug_car_method  = 0;
            g_vision_share.debug_car_track_id = 0;
        }

        // 误差衰减保留值 (帧间平滑过渡，避免丢失目标时突变)
        static int16 decay_beacon_err_x = 0;
        static int16 decay_beacon_err_y = 0;
        static int16 decay_err_x = 0;
        static int16 decay_err_y = 0;

        if(best_beacon != NULL)
        {
            g_vision_share.beacon_err_y = (CAMERA_H / 2) - (int16)best_beacon->cy;
            g_vision_share.beacon_err_x = (CAMERA_W / 2) - (int16)best_beacon->cx;
            decay_beacon_err_x = g_vision_share.beacon_err_x;
            decay_beacon_err_y = g_vision_share.beacon_err_y;
            g_vision_share.beacon_found = 1;
        }
        else
        {
            decay_beacon_err_x = (int16)((float)decay_beacon_err_x * ERR_DECAY_FACTOR);
            decay_beacon_err_y = (int16)((float)decay_beacon_err_y * ERR_DECAY_FACTOR);
            g_vision_share.beacon_err_x = decay_beacon_err_x;
            g_vision_share.beacon_err_y = decay_beacon_err_y;
            g_vision_share.beacon_found = 0;
        }

        if((best_beacon != NULL) && found_car)
        {
            // ===== 目标相对无人机的机体坐标系误差计算 =====
            // raw_dx, raw_dy: 信标到小车的像素矢量 (图像坐标系)
            int16 raw_dx = (int16)best_beacon->cx - (int16)car_x;
            int16 raw_dy = (int16)best_beacon->cy - (int16)car_y;

            float cos_h = cosf(g_vision_share.heading_angle);
            float sin_h = sinf(g_vision_share.heading_angle);

            // 将像素矢量旋转到机体坐标系:
            //   err_x = 小车→信标矢量 投影到 机头方向 (+) → 前后误差
            //   err_y = 小车→信标矢量 投影到 机头右方向 (+) → 左右误差
            // 这样控制核 CM7_0 可以直接用 err_x→Pitch, err_y→Roll 进行追踪
            g_vision_share.err_x = (int16)(raw_dx * cos_h + raw_dy * sin_h);
            g_vision_share.err_y = (int16)(-raw_dx * sin_h + raw_dy * cos_h);
            decay_err_x = g_vision_share.err_x;
            decay_err_y = g_vision_share.err_y;
            g_vision_share.target_found = 1;
        }
        else
        {
            decay_err_x = (int16)((float)decay_err_x * ERR_DECAY_FACTOR);
            decay_err_y = (int16)((float)decay_err_y * ERR_DECAY_FACTOR);
            g_vision_share.err_x = decay_err_x;
            g_vision_share.err_y = decay_err_y;
            g_vision_share.target_found = 0;
        }
        // 共享内存刷新: 双核数据同步
        // g_vision_share 位于 VISION_SHARE_ADDR (0x28001000) 的共享内存区
        // CM7_1 (视觉核) 写入后必须刷新 D-Cache，CM7_0 (控制核) 才能读到最新数据
        g_vision_share.frame_id++;
        SCB_CleanInvalidateDCache_by_Addr((uint32_t *)&g_vision_share, sizeof(g_vision_share));

        // 打印发送给小车的数据帧: #err_x,err_y,n$  (调试用, 默认关闭)
        #if DEBUG_HC06_FRAME
        printf("#%d,%d,%d$\r\n",
               g_vision_share.err_x,
               g_vision_share.err_y,
               g_vision_share.target_found ? 1 : 2);
        #endif

        // 上位机可视化输出 (默认关闭, 由 DEBUG_REALTIME_MONITOR 控制)
        #if DEBUG_REALTIME_MONITOR
        {
            // H行：V形车标 (frame_id,area,v_found,angle,arm1,arm2,base_len,
            //        base1_x,base1_y,base2_x,base2_y,mid_x,mid_y,vertex_x,vertex_y)
            for(uint8 hi = 0; hi < blob_num; hi++)
            {
                if(blobs[hi].type == BLOB_CAR_MARKER && blobs[hi].marker_angle_deg > 0)
                {
                    printf("H,%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
                           (unsigned long)g_vision_share.frame_id,
                           (unsigned int)blobs[hi].area,
                           (unsigned int)blobs[hi].marker_angle_deg,
                           (unsigned int)blobs[hi].marker_arm1_len,
                           (unsigned int)blobs[hi].marker_arm2_len,
                           (unsigned int)blobs[hi].marker_base_len,
                           (unsigned int)blobs[hi].marker_base1_x,
                           (unsigned int)blobs[hi].marker_base1_y,
                           (unsigned int)blobs[hi].marker_base2_x,
                           (unsigned int)blobs[hi].marker_base2_y,
                           (unsigned int)blobs[hi].marker_base_mid_x,
                           (unsigned int)blobs[hi].marker_base_mid_y,
                           (unsigned int)blobs[hi].marker_vertex_x,
                           (unsigned int)blobs[hi].marker_vertex_y);
                    break;  // 只输出第一个 V 形车标
                }
            }

            // B行：信标 (frame_id,cx,cy,filt_score)
            for(uint8 bi = 0; bi < blob_num; bi++)
            {
                if(blobs[bi].type == BLOB_BEACON)
                {
                    printf("B,%lu,%u,%u,%u\r\n",
                           (unsigned long)g_vision_share.frame_id,
                           (unsigned int)blobs[bi].cx,
                           (unsigned int)blobs[bi].cy,
                           (unsigned int)blobs[bi].filt_beacon_score);
                }
            }
        }
        #endif

        // 调试输出：小车与信标的像素误差（仅在有识别时输出）
       //if(g_vision_share.target_found || g_vision_share.car_found)
       //{
       //    printf("%d,%d,%d,%d,%d\r\n",
       //           g_vision_share.frame_id,
       //           (int)g_vision_share.target_found,
       //           (int)g_vision_share.car_found,
       //           (int)g_vision_share.err_x,
       //           (int)g_vision_share.err_y);
       //}
        
        // 刷新屏幕
        #if VISION_SERIAL_DEBUG
//        if((g_vision_share.frame_id % VISION_SERIAL_DEBUG_DIV) == 0)
//        {
//            printf("%lu,%u,%u,%u,%d,%d,%u,%d,%d,%d,%d,%d\r\n",
//                   (unsigned long)g_vision_share.frame_id,
//                   (unsigned int)blob_num,
//                   (unsigned int)g_vision_share.target_found,
//                   (unsigned int)g_vision_share.car_found,
//                   (int)g_vision_share.car_x,
//                   (int)g_vision_share.car_y,
//                   (unsigned int)g_vision_share.beacon_found,
//                   (int)g_vision_share.beacon_err_x,
//                   (int)g_vision_share.beacon_err_y,
//                   (int)g_vision_share.err_x,
//                   (int)g_vision_share.err_y,
//                   (int)(g_vision_share.heading_angle * 1000.0f));
//        }
#endif

#if VISION_ENABLE_IPS_DISPLAY
        // 在车头方向画圆标记
        {
            for(uint8 i = 0; i < blob_num; i++)
            {
                if(blobs[i].type == BLOB_CAR_MARKER && blobs[i].marker_angle_deg > 0)
                {
                    // 在顶点 (车头) 画一个实心圆
                    int vx = (int)blobs[i].marker_vertex_x;
                    int vy = (int)blobs[i].marker_vertex_y;
                    draw_circle(vx, vy, 8, 200);      // 外圆
                    draw_circle(vx, vy, 5, 255);      // 内圆
                    break;
                }
            }
        }
        ips200_displayimage03x(image_buffer[0], CAMERA_W, CAMERA_H);
#endif

        mt9v03x_finish_flag = 0;
    }
}