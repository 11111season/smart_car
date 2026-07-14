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

// 霍夫变换最大直线数 (需要在 Blob 定义之前)
#define HOUGH_MAX_LINES   8

// 新增：Blob 统一数据结构
typedef struct
{
    uint16 cx;
    uint16 cy;
    uint16 width;
    uint16 height;
    uint16 area;
    uint16 core_count;
    BlobType type;
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
    uint8  marker_hough_v;               // 1=霍夫变换确认的 V 形 (优先于软评分)
#if DEBUG_HOUGH
    uint8  dbg_hough_cnt;                // 霍夫直线数 (供延迟打印)
    uint8  dbg_hough_num_angles;         // 霍夫扫描角度数
    int16  dbg_hough_rho[HOUGH_MAX_LINES];
    int8   dbg_hough_theta[HOUGH_MAX_LINES];
    uint16 dbg_hough_votes[HOUGH_MAX_LINES];
#endif
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

/*
 * =============================================================================
 * 霍夫变换后备方案 - 用于 BFS 几何法因遮挡/分裂而失败的场景
 * 当 V 形被部分遮挡，分裂为 √ 形 + 小条时，霍夫可在图像级跨 blob 找到两条臂
 * =============================================================================
 */

// 边缘点缓存 (由 extract_edges_once 填充, 供 Hough 使用)
#define MAX_EDGE_POINTS   3000
static Point edge_pts[MAX_EDGE_POINTS];
static uint32 edge_cnt = 0;

// 霍夫直线结构
typedef struct {
    int16 rho;
    int8  theta_deg;
    uint16 votes;
} HoughLine;

// 霍夫 rho 直方图 (静态全局, 避免栈溢出)
#define RHO_HIST_SIZE     641     // rho 范围 [-320, +320], 偏移 +320 后索引 [0, 640]
#define RHO_HIST_OFFSET   320
static uint16 rho_hist[RHO_HIST_SIZE];

/*
 * -----------------------------------------------------------------------------
 * extract_edges_once - 全局边缘提取 (每帧调用一次)
 * -----------------------------------------------------------------------------
 * 功能: 在二值化后的 image_buffer 上检测 0↔255 的跳变，提取边缘点
 * 位置: 在 camera_process() 中二值化之后、extract_blobs() 之前调用
 * 输出: edge_pts[] 数组 + edge_cnt
 * -----------------------------------------------------------------------------
 */
static void extract_edges_once(void)
{
    edge_cnt = 0;
    for (uint16 y = 1; y < CAMERA_H - 1; y++)
    {
        for (uint16 x = 1; x < CAMERA_W - 1; x++)
        {
            // 检测水平或垂直方向上的二值跳变 (0→255 或 255→0)
            if (image_buffer[y][x] != image_buffer[y][x-1] ||
                image_buffer[y][x] != image_buffer[y-1][x])
            {
                // 只保留白色侧的边缘点 (V 形主体), 提高信噪比
                if (image_buffer[y][x] == 255)
                {
                    edge_pts[edge_cnt].x = x;
                    edge_pts[edge_cnt].y = y;
                    edge_cnt++;
                    if (edge_cnt >= MAX_EDGE_POINTS) return;
                }
            }
        }
    }
}

/*
 * -----------------------------------------------------------------------------
 * hough_transform_roi - 局部霍夫变换 (在指定 ROI 内找直线)
 * -----------------------------------------------------------------------------
 * 输入:
 *   cx, cy       - ROI 中心 (一般用 blob 质心)
 *   half_w, half_h - ROI 半宽半高 (以包含相邻碎片 blob)
 *   theta_list   - 要扫描的角度列表 (度)
 *   num_theta    - 角度个数
 *   vote_thresh  - 投票阈值
 * 输出:
 *   lines[]      - 检测到的直线数组
 *   返回值       - 检测到的直线数
 * -----------------------------------------------------------------------------
 */
static uint8 hough_transform_roi(uint16 cx, uint16 cy,
                                 uint16 half_w, uint16 half_h,
                                 const int8 *theta_list, uint8 num_theta,
                                 uint16 vote_thresh,
                                 HoughLine *lines, uint8 max_lines)
{
    // 计算 ROI 边界
    int16 x_min = (int16)cx - (int16)half_w;
    if (x_min < 0) x_min = 0;
    int16 x_max = (int16)cx + (int16)half_w;
    if (x_max >= CAMERA_W) x_max = CAMERA_W - 1;
    int16 y_min = (int16)cy - (int16)half_h;
    if (y_min < 0) y_min = 0;
    int16 y_max = (int16)cy + (int16)half_h;
    if (y_max >= CAMERA_H) y_max = CAMERA_H - 1;

    uint8 line_cnt = 0;

    for (uint8 a = 0; a < num_theta; a++)
    {
        int8 theta_deg = theta_list[a];
        float rad = (float)theta_deg * 0.0174532925f;  // π/180
        float cos_t = cosf(rad);
        float sin_t = sinf(rad);

        // 清零直方图 (只清零可能用到的 bin)
        memset(rho_hist, 0, sizeof(rho_hist));

        uint16 max_votes = 0;
        int16 best_rho = 0;

        // 遍历 ROI 内的边缘点
        for (uint32 i = 0; i < edge_cnt; i++)
        {
            uint16 ex = edge_pts[i].x;
            uint16 ey = edge_pts[i].y;
            if (ex < (uint16)x_min || ex > (uint16)x_max ||
                ey < (uint16)y_min || ey > (uint16)y_max) continue;

            int16 rho = (int16)((float)ex * cos_t + (float)ey * sin_t);
            int16 idx = rho + RHO_HIST_OFFSET;
            if (idx < 0 || idx >= RHO_HIST_SIZE) continue;

            rho_hist[idx]++;
            if (rho_hist[idx] > max_votes)
            {
                max_votes = rho_hist[idx];
                best_rho = rho;
            }
        }

        if (max_votes >= vote_thresh)
        {
            lines[line_cnt].rho = best_rho;
            lines[line_cnt].theta_deg = theta_deg;
            lines[line_cnt].votes = max_votes;
            line_cnt++;
            if (line_cnt >= max_lines) break;
        }
    }
    return line_cnt;
}

/*
 * -----------------------------------------------------------------------------
 * find_v_from_hough - 从霍夫直线中寻找 V 形配对
 * -----------------------------------------------------------------------------
 * 功能: 从检测到的直线中找一对斜率相反 (一正一负) 且交点合理的直线,
 *       作为 V 形的两条臂, 然后计算 V 形的几何特征.
 * 输入:
 *   lines[]      - 霍夫检测到的直线
 *   line_cnt     - 直线数
 * 输出:
 *   b            - 填充 Blob 的 V 形特征字段
 * 返回值: 1=成功, 0=失败
 * -----------------------------------------------------------------------------
 */
static uint8 find_v_from_hough(const HoughLine *lines, uint8 line_cnt, Blob *b)
{
    static float st_angle = 0.0f;     // 帧间追踪: 上一帧的 V 形角度
    static uint8  st_valid = 0;        // 0=无上一帧, 1=有上一帧

    float best_score = 0.0f;
    uint8 best_i = 0xFF, best_j = 0xFF;
    float b_ax, b_ay, b_bx, b_by, b_cx, b_cy;
    float b_arm1, b_arm2, b_angle_deg, b_height, b_base_w;

    // ★ 调试: 配对拒绝原因计数
    uint16 dbg_total = 0;       // 总配对尝试数
    uint16 dbg_angle_diff = 0;  // θ差异不满足V形范围
    uint16 dbg_parallel = 0;    // 平行线(行列式≈0)
    uint16 dbg_oob = 0;         // 交点超出图像边界
    uint16 dbg_nbhd = 0;        // 3×3邻域白像素不足
    uint16 dbg_bbox = 0;        // 端点超出blob外接矩形
    uint16 dbg_base_w = 0;      // 底边宽度不合理
    uint16 dbg_angle = 0;       // 顶点角度不满足范围
    uint16 dbg_height = 0;      // 高度不足
    uint16 dbg_arm = 0;         // 臂长<4(交叉配对)
    uint16 dbg_edge = 0;        // 靠近图像边界
    uint16 dbg_pass = 0;        // 通过所有过滤的配对

    for (uint8 i = 0; i < line_cnt; i++)
    {
        for (uint8 j = i + 1; j < line_cnt; j++)
        {
            // 要求两条直线夹角在 V 形范围内 (12°~90°)
            // θ差异太小→平行无法形成V；θ差异太大→过于尖锐不可能是车标
            {
                dbg_total++;
                int8 diff = lines[i].theta_deg - lines[j].theta_deg;
                if (diff < 0) diff = -diff;
                if (diff < (int8)CAR_MARK_MIN_ANGLE_DEG || diff > (int8)CAR_MARK_MAX_ANGLE_DEG)
                    { dbg_angle_diff++; continue; }
            }

            // 两条直线方程: x*cosθ + y*sinθ = ρ
            float cos1 = cosf((float)lines[i].theta_deg * 0.0174532925f);
            float sin1 = sinf((float)lines[i].theta_deg * 0.0174532925f);
            float cos2 = cosf((float)lines[j].theta_deg * 0.0174532925f);
            float sin2 = sinf((float)lines[j].theta_deg * 0.0174532925f);

            // 解交点 (顶点 A)
            float det = cos1 * sin2 - cos2 * sin1;
            if (fabsf(det) < 0.001f) { dbg_parallel++; continue; }   // 平行, 跳过

            float ax = ((float)lines[i].rho * sin2 - (float)lines[j].rho * sin1) / det;
            float ay = ((float)lines[j].rho * cos1 - (float)lines[i].rho * cos2) / det;

            // 交点必须在 ROI 内
            if (ax < 0 || ax >= (float)CAMERA_W || ay < 0 || ay >= (float)CAMERA_H)
                { dbg_oob++; continue; }

            // ★★★ 双向追踪: 沿直线两个方向追踪, 取找到白点多的那一侧
            // 消除"追踪方向翻转"导致的端点乱飘
            // 含间隙检测: 连续5黑像素即停止, 防止跨过间隙跳到远处噪声
            float dx1 = -sin1, dy1 = cos1;    // 线1正向
            float dx2 = -sin2, dy2 = cos2;    // 线2正向

            // --- 线1: 正向追踪 ---
            float bx_fwd = ax, by_fwd = ay;
            int cnt_fwd = 0, gap_fwd = 0;
            for (float t = 0; t < 200; t += 1.0f) {
                float x = ax + dx1 * t;
                float y = ay + dy1 * t;
                int ix = (int)x, iy = (int)y;
                if (ix < 0 || ix >= CAMERA_W || iy < 0 || iy >= CAMERA_H) break;
                if (image_buffer[iy][ix] == 255) { bx_fwd = x; by_fwd = y; cnt_fwd++; gap_fwd = 0; }
                else { gap_fwd++; if (gap_fwd >= 5) break; }
            }
            // --- 线1: 反向追踪 ---
            float bx_bwd = ax, by_bwd = ay;
            int cnt_bwd = 0, gap_bwd = 0;
            for (float t = 0; t < 200; t += 1.0f) {
                float x = ax - dx1 * t;
                float y = ay - dy1 * t;
                int ix = (int)x, iy = (int)y;
                if (ix < 0 || ix >= CAMERA_W || iy < 0 || iy >= CAMERA_H) break;
                if (image_buffer[iy][ix] == 255) { bx_bwd = x; by_bwd = y; cnt_bwd++; gap_bwd = 0; }
                else { gap_bwd++; if (gap_bwd >= 5) break; }
            }
            float bx = (cnt_fwd >= cnt_bwd) ? bx_fwd : bx_bwd;
            float by = (cnt_fwd >= cnt_bwd) ? by_fwd : by_bwd;

            // --- 线2: 正向追踪 ---
            float cx_fwd = ax, cy_fwd = ay;
            cnt_fwd = 0; gap_fwd = 0;
            for (float t = 0; t < 200; t += 1.0f) {
                float x = ax + dx2 * t;
                float y = ay + dy2 * t;
                int ix = (int)x, iy = (int)y;
                if (ix < 0 || ix >= CAMERA_W || iy < 0 || iy >= CAMERA_H) break;
                if (image_buffer[iy][ix] == 255) { cx_fwd = x; cy_fwd = y; cnt_fwd++; gap_fwd = 0; }
                else { gap_fwd++; if (gap_fwd >= 5) break; }
            }
            // --- 线2: 反向追踪 ---
            float cx_bwd = ax, cy_bwd = ay;
            cnt_bwd = 0; gap_bwd = 0;
            for (float t = 0; t < 200; t += 1.0f) {
                float x = ax - dx2 * t;
                float y = ay - dy2 * t;
                int ix = (int)x, iy = (int)y;
                if (ix < 0 || ix >= CAMERA_W || iy < 0 || iy >= CAMERA_H) break;
                if (image_buffer[iy][ix] == 255) { cx_bwd = x; cy_bwd = y; cnt_bwd++; gap_bwd = 0; }
                else { gap_bwd++; if (gap_bwd >= 5) break; }
            }
            float cx = (cnt_fwd >= cnt_bwd) ? cx_fwd : cx_bwd;
            float cy = (cnt_fwd >= cnt_bwd) ? cy_fwd : cy_bwd;

            // ★★★ 端点邻域噪点过滤: 3×3 窗口内至少 4 个白像素才接受
            // 孤立噪点周围没有足够白像素支撑, 不可能是 V 形臂端点
            {
                int ibx = (int)(bx + 0.5f), iby = (int)(by + 0.5f);
                int icx = (int)(cx + 0.5f), icy = (int)(cy + 0.5f);
                int sup_b = 0, sup_c = 0;
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        int sx, sy;
                        sx = ibx + dx; sy = iby + dy;
                        if (sx >= 0 && sx < CAMERA_W && sy >= 0 && sy < CAMERA_H &&
                            image_buffer[sy][sx] == 255) sup_b++;
                        sx = icx + dx; sy = icy + dy;
                        if (sx >= 0 && sx < CAMERA_W && sy >= 0 && sy < CAMERA_H &&
                            image_buffer[sy][sx] == 255) sup_c++;
                    }
                }
                if (sup_b < 2 || sup_c < 2) { dbg_nbhd++; continue; }
            }

            // 端点验证: B、C 都必须在 blob 外接矩形 ±15px 内
            {
                uint16 bbl = (b->cx > b->width/2)  ? b->cx - b->width/2  : 0;
                uint16 bbr = (b->cx + b->width/2  < CAMERA_W-1) ? b->cx + b->width/2  : CAMERA_W-1;
                uint16 bbt = (b->cy > b->height/2) ? b->cy - b->height/2 : 0;
                uint16 bbb = (b->cy + b->height/2 < CAMERA_H-1) ? b->cy + b->height/2 : CAMERA_H-1;
                if (bx < (int16)bbl - 15 || bx > (int16)bbr + 15 ||
                    by < (int16)bbt - 15 || by > (int16)bbb + 15 ||
                    cx < (int16)bbl - 15 || cx > (int16)bbr + 15 ||
                    cy < (int16)bbt - 15 || cy > (int16)bbb + 15)
                    { dbg_bbox++; continue; }
            }

            // 使用追踪到的实际端点位置
            float mid_x = (bx + cx) * 0.5f;
            float mid_y = (by + cy) * 0.5f;

            // 底边宽度检查 (太窄或太宽都不合理)
            float base_w = fabsf(bx - cx);
            if (base_w < 5.0f || base_w > (float)CAMERA_W * 0.8f) { dbg_base_w++; continue; }

            // 臂长检查: 顶点到实际追踪到的底边端点
            float arm1 = sqrtf((ax - bx) * (ax - bx) + (ay - by) * (ay - by));
            float arm2 = sqrtf((ax - cx) * (ax - cx) + (ay - cy) * (ay - cy));

            // 顶点夹角 (余弦定理) — 用实际追踪到的底边端点
            float dot = (bx - ax) * (cx - ax) + (by - ay) * (cy - ay);
            float cos_a = dot / (arm1 * arm2);
            if (cos_a > 1.0f) cos_a = 1.0f;
            if (cos_a < -1.0f) cos_a = -1.0f;
            float angle_deg = acosf(cos_a) * 57.2957795f;
            if (angle_deg < (float)CAR_MARK_MIN_ANGLE_DEG ||
                angle_deg > (float)CAR_MARK_MAX_ANGLE_DEG) { dbg_angle++; continue; }

            // 顶点到底边的垂距
            float height = arm1 * sinf(acosf(cos_a));
            if (height < (float)CAR_MARK_MIN_HEIGHT) { dbg_height++; continue; }

            // ★★★ 交叉配对过滤: 真V形两臂都长, 假V形(同臂内外缘)一臂极短
            // 放宽: 如果底边足够宽(>15px), 即使臂短也可能是薄臂的真V形
            if (arm1 < 4.0f || arm2 < 4.0f) {
                if (base_w < 15.0f) { dbg_arm++; continue; }
            }


            // ★★★ 边界保护: 拒绝顶点或底边中点太靠近图像边框的解
            #define HOUGH_EDGE_MARGIN 8
            if (ax < HOUGH_EDGE_MARGIN || ax >= ((float)CAMERA_W - HOUGH_EDGE_MARGIN) ||
                ay < HOUGH_EDGE_MARGIN || ay >= ((float)CAMERA_H - HOUGH_EDGE_MARGIN) ||
                mid_x < HOUGH_EDGE_MARGIN || mid_x >= ((float)CAMERA_W - HOUGH_EDGE_MARGIN) ||
                mid_y < HOUGH_EDGE_MARGIN || mid_y >= ((float)CAMERA_H - HOUGH_EDGE_MARGIN))
            {
                #undef HOUGH_EDGE_MARGIN
                dbg_edge++; continue;
            }
            #undef HOUGH_EDGE_MARGIN

            dbg_pass++;  // 通过所有过滤

            // ===== 评分: 选出最稳定的配对 =====
            // 臂长越长 → 白点追踪越稳定, 帧间可信度越高
            // 注意: 不惩罚不对称臂长, 透视畸变下真实V形也会不对称
            float score = (arm1 + arm2) * 0.5f;

            // 帧间角度稳定性加成: 优先选角度接近上一帧的配对
            // 转动时霍夫线索引会变, 但 V 形角度连续变化
            if (st_valid) {
                float ang_diff = fabsf(angle_deg - st_angle);
                if (ang_diff < 30.0f)
                    score += (20.0f - ang_diff);  // 角度越接近, 加成越高
            }

            if (score > best_score)
            {
                best_score = score;
                best_i = i; best_j = j;
                b_ax = ax;     b_ay = ay;
                b_bx = bx;     b_by = by;
                b_cx = cx;     b_cy = cy;
                b_arm1 = arm1; b_arm2 = arm2;
                b_angle_deg = angle_deg;
                b_height = height;
                b_base_w = base_w;
            }
        }
    }

    // ★ 调试: 打印配对拒绝原因分布
#if DEBUG_HOUGH
    printf("HDBG,%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
           (unsigned long)g_vision_share.frame_id,
           (unsigned int)dbg_total, (unsigned int)dbg_angle_diff,
           (unsigned int)dbg_parallel, (unsigned int)dbg_oob,
           (unsigned int)dbg_nbhd, (unsigned int)dbg_bbox,
           (unsigned int)dbg_base_w, (unsigned int)dbg_angle,
           (unsigned int)dbg_height, (unsigned int)dbg_arm,
           (unsigned int)dbg_edge, (unsigned int)dbg_pass);
#endif

    // ===== 填充最优配对 =====
    if (best_score > 0.0f)
    {
        st_angle = b_angle_deg;  // 更新帧间追踪
        st_valid = 1;
        uint16 uax = (uint16)(b_ax + 0.5f);
        uint16 uay = (uint16)(b_ay + 0.5f);
        uint16 ubx = (uint16)(b_bx + 0.5f);
        uint16 uby = (uint16)(b_by + 0.5f);
        uint16 ucx = (uint16)(b_cx + 0.5f);
        uint16 ucy = (uint16)(b_cy + 0.5f);
        uint16 umx = (uint16)((b_bx + b_cx) * 0.5f + 0.5f);
        uint16 umy = (uint16)((b_by + b_cy) * 0.5f + 0.5f);

        b->marker_vertex_x  = uax;
        b->marker_vertex_y  = uay;
        b->marker_base1_x   = ubx;
        b->marker_base1_y   = uby;
        b->marker_base2_x   = ucx;
        b->marker_base2_y   = ucy;
        b->marker_base_mid_x = umx;
        b->marker_base_mid_y = umy;
        b->marker_base_len   = (uint16)(b_base_w + 0.5f);
        b->marker_height     = (uint16)(b_height + 0.5f);
        b->marker_angle_deg  = (uint16)(b_angle_deg + 0.5f);
        b->marker_arm1_len   = (uint16)(b_arm1 + 0.5f);
        b->marker_arm2_len   = (uint16)(b_arm2 + 0.5f);
        // 验证朝向：V形底边（宽端）像素多，质心更靠近底边
        {
            float dvx = (float)uax - (float)b->cx;
            float dvy = (float)uay - (float)b->cy;
            float dbx = (float)umx - (float)b->cx;
            float dby = (float)umy - (float)b->cy;
            if ((dbx*dbx + dby*dby) > (dvx*dvx + dvy*dvy))
            {
                b->marker_heading = atan2f((float)((int32)umy - (int32)uay),
                                           (float)((int32)umx - (int32)uax));
            }
            else
            {
                b->marker_heading = atan2f((float)((int32)uay - (int32)umy),
                                           (float)((int32)uax - (int32)umx));
            }
        }

        if (b->raw_marker_score < 50)
            b->raw_marker_score = 50;

        return 1;
    }

    // 未检出: 重置帧间追踪
    st_valid = 0;
    return 0;
}

// 霍夫验证 BFS 结果: 用霍夫直线验证 BFS 找到的 V 形是否为真, 并精修端点
// 返回: 2=双臂验证通过, 1=单臂验证, 0=未验证(可能是圆形信标)
static uint8 hough_verify_bfs(HoughLine *lines, uint8 n, Blob *b)
{
    float ax = (float)b->marker_vertex_x;
    float ay = (float)b->marker_vertex_y;
    float bx = (float)b->marker_base1_x;
    float by = (float)b->marker_base1_y;
    float cx = (float)b->marker_base2_x;
    float cy = (float)b->marker_base2_y;

    float v1x = bx - ax, v1y = by - ay;
    float v2x = cx - ax, v2y = cy - ay;
    float a1 = sqrtf(v1x*v1x + v1y*v1y);
    float a2 = sqrtf(v2x*v2x + v2y*v2y);
    if (a1 < 2.0f || a2 < 2.0f) return 0;
    v1x /= a1; v1y /= a1;
    v2x /= a2; v2y /= a2;

    int8 best_i1 = -1, best_i2 = -1;
    float best_dot1 = 0.5f;  // cos(60°) — 方向偏差不超过60°
    float best_dot2 = 0.5f;

    for (uint8 i = 0; i < n; i++) {
        float theta = (float)lines[i].theta_deg * 0.0174533f;
        float lx = -sinf(theta), ly = cosf(theta);
        float dot1 = fabsf(lx * v1x + ly * v1y);
        float dot2 = fabsf(lx * v2x + ly * v2y);
        if (dot1 > best_dot1) { best_dot1 = dot1; best_i1 = (int8)i; }
        if (dot2 > best_dot2) { best_dot2 = dot2; best_i2 = (int8)i; }
    }

    if (best_i1 < 0 && best_i2 < 0) return 0;

    // 沿霍夫直线追踪, 精修端点
    if (best_i1 >= 0) {
        float theta = (float)lines[best_i1].theta_deg * 0.0174533f;
        float dx = -sinf(theta), dy = cosf(theta);
        float fwd_x = ax, fwd_y = ay, bwd_x = ax, bwd_y = ay;
        int cnt_fwd = 0, cnt_bwd = 0, gap_fwd = 0, gap_bwd = 0;
        for (float t = 0; t < 200; t += 1.0f) {
            float x = ax + dx * t, y = ay + dy * t;
            int ix = (int)x, iy = (int)y;
            if (ix < 0 || ix >= CAMERA_W || iy < 0 || iy >= CAMERA_H) break;
            if (image_buffer[iy][ix] == 255) { fwd_x = x; fwd_y = y; cnt_fwd++; gap_fwd = 0; }
            else { gap_fwd++; if (gap_fwd >= 5) break; }
        }
        for (float t = 0; t < 200; t += 1.0f) {
            float x = ax - dx * t, y = ay - dy * t;
            int ix = (int)x, iy = (int)y;
            if (ix < 0 || ix >= CAMERA_W || iy < 0 || iy >= CAMERA_H) break;
            if (image_buffer[iy][ix] == 255) { bwd_x = x; bwd_y = y; cnt_bwd++; gap_bwd = 0; }
            else { gap_bwd++; if (gap_bwd >= 5) break; }
        }
        if (cnt_fwd >= cnt_bwd && cnt_fwd > 0) { bx = fwd_x; by = fwd_y; }
        else if (cnt_bwd > 0) { bx = bwd_x; by = bwd_y; }
    }

    if (best_i2 >= 0) {
        float theta = (float)lines[best_i2].theta_deg * 0.0174533f;
        float dx = -sinf(theta), dy = cosf(theta);
        float fwd_x = ax, fwd_y = ay, bwd_x = ax, bwd_y = ay;
        int cnt_fwd = 0, cnt_bwd = 0, gap_fwd = 0, gap_bwd = 0;
        for (float t = 0; t < 200; t += 1.0f) {
            float x = ax + dx * t, y = ay + dy * t;
            int ix = (int)x, iy = (int)y;
            if (ix < 0 || ix >= CAMERA_W || iy < 0 || iy >= CAMERA_H) break;
            if (image_buffer[iy][ix] == 255) { fwd_x = x; fwd_y = y; cnt_fwd++; gap_fwd = 0; }
            else { gap_fwd++; if (gap_fwd >= 5) break; }
        }
        for (float t = 0; t < 200; t += 1.0f) {
            float x = ax - dx * t, y = ay - dy * t;
            int ix = (int)x, iy = (int)y;
            if (ix < 0 || ix >= CAMERA_W || iy < 0 || iy >= CAMERA_H) break;
            if (image_buffer[iy][ix] == 255) { bwd_x = x; bwd_y = y; cnt_bwd++; gap_bwd = 0; }
            else { gap_bwd++; if (gap_bwd >= 5) break; }
        }
        if (cnt_fwd >= cnt_bwd && cnt_fwd > 0) { cx = fwd_x; cy = fwd_y; }
        else if (cnt_bwd > 0) { cx = bwd_x; cy = bwd_y; }
    }

    // 更新坐标
    b->marker_base1_x = (uint16)(bx + 0.5f);
    b->marker_base1_y = (uint16)(by + 0.5f);
    b->marker_base2_x = (uint16)(cx + 0.5f);
    b->marker_base2_y = (uint16)(cy + 0.5f);
    b->marker_base_mid_x = (uint16)((bx + cx) * 0.5f + 0.5f);
    b->marker_base_mid_y = (uint16)((by + cy) * 0.5f + 0.5f);

    return (best_i1 >= 0 && best_i2 >= 0) ? 2 : 1;
}

// 计算两点距离平方
static uint32 point_dist2(Point a, Point b)
{
    int32 dx = (int32)a.x - (int32)b.x;
    int32 dy = (int32)a.y - (int32)b.y;
    return (uint32)(dx * dx + dy * dy);
}

// V形车标特征提取：从 blob 像素中找 3 个关键点 (角点A, 底边端点B,C)
static void compute_car_marker_feature(Blob *b, Point *pts, uint32 count)
{
    // 面积预过滤
    if(b->area < CAR_MARK_MIN_AREA || b->area > CAR_MARK_MAX_AREA) return;

    // 圆形度预过滤：近似圆形的 blob 不可能是 V 形（节省计算）
    if(b->oheight > 0)
    {
        uint16 ow = b->owidth;
        uint16 oh = b->oheight;
        uint16 ratio = (ow > oh) ? (ow * 100 / oh) : (oh * 100 / ow);
        if(ratio < 120) return;   // ~1:1.20 以内太圆，跳过 (原 135 收紧, 保留更多碎片候选)
    }

    b->marker_hough_v = 0;
    b->marker_angle_deg = 0;   // 防止复用旧值导致 angle=0 但 marker_hough_v=1 的 bug

    // DEBUG_HOUGH: 提升到函数作用域, 供末尾打印使用
    HoughLine hough_lines[HOUGH_MAX_LINES];
    uint8 hough_cnt = 0;
    uint8 hough_skip = 0;
    uint8 hough_num_angles = 0;

    // ===== 第一步：霍夫变换 (直线检测, 供 BFS 后验证用) =====
    // 在 blob ROI 内检测直线, 后续用于验证 BFS 结果并精修端点
    {
        #define CAR_MARK_EDGE_MARGIN  10
        hough_skip = 0;
        if (b->cx < CAR_MARK_EDGE_MARGIN ||
            b->cx > (CAMERA_W - CAR_MARK_EDGE_MARGIN) ||
            b->cy < CAR_MARK_EDGE_MARGIN ||
            b->cy > (CAMERA_H - CAR_MARK_EDGE_MARGIN))
        {
            hough_skip = 1;  // 太靠边, 跳过霍夫, 走 BFS 兜底
        }
        #undef CAR_MARK_EDGE_MARGIN

        if (!hough_skip)
        {
            uint16 roi_half_w = (b->width / 2) + HOUGH_ROI_MARGIN;
            uint16 roi_half_h = (b->height / 2) + HOUGH_ROI_MARGIN;

            int8 hough_angles[62];
            uint8 idx = 0;
            for(int8 t = -(int8)HOUGH_ANGLE_MAX; t <= -(int8)HOUGH_ANGLE_MIN; t += (int8)HOUGH_ANGLE_STEP)
                hough_angles[idx++] = t;
            for(int8 t = (int8)HOUGH_ANGLE_MIN; t <= (int8)HOUGH_ANGLE_MAX; t += (int8)HOUGH_ANGLE_STEP)
                hough_angles[idx++] = t;

            hough_num_angles = idx;

            hough_cnt = hough_transform_roi(
                b->cx, b->cy,
                roi_half_w, roi_half_h,
                hough_angles, idx,
                HOUGH_VOTE_THRESH,
                hough_lines, HOUGH_MAX_LINES);

#if DEBUG_SCORES
            printf("HOUGH: area=%d cx=%d cy=%d w=%d h=%d\n",
                   b->area, b->cx, b->cy, b->width, b->height);
            printf("HOUGH: lines=%d thresh=%d angles=%d\n",
                   hough_cnt, HOUGH_VOTE_THRESH, idx);
#endif

            // 霍夫直线保留供 BFS 后验证用, 不再作为独立检测器
        }
    }

    // ===== 第二步：BFS 拓扑法 (主检测) =====
    {
    // 第一步：找 8 个极值候选点
    Point candidates[8];
    uint32 best_metric[8];
    Point base1 = pts[0];
    Point base2 = pts[0];
    Point vertex = pts[0];
    uint32 best_base_d2 = 0;
    float best_height = 0.0f;

    for(uint8 i = 0; i < 8; i++)
    {
        candidates[i] = pts[0];
        best_metric[i] = (i & 1) ? 0 : 0xFFFFFFFF;
    }

    for(uint32 p = 0; p < count; p++)
    {
        uint32 x = pts[p].x;
        uint32 y = pts[p].y;
        uint32 metrics[8];
        metrics[0] = x;
        metrics[1] = x;
        metrics[2] = y;
        metrics[3] = y;
        metrics[4] = x + y;
        metrics[5] = x + y;
        metrics[6] = x + (uint32)(CAMERA_H - y);
        metrics[7] = x + (uint32)(CAMERA_H - y);

        for(uint8 i = 0; i < 8; i += 2)
        {
            if(metrics[i] < best_metric[i])
            {
                best_metric[i] = metrics[i];
                candidates[i] = pts[p];
            }
            if(metrics[i + 1] > best_metric[i + 1])
            {
                best_metric[i + 1] = metrics[i + 1];
                candidates[i + 1] = pts[p];
            }
        }
    }

    // 第二步：从候选点中找最远的一对 → B, C（底边两端）
    for(uint8 i = 0; i < 8; i++)
    {
        for(uint8 j = i + 1; j < 8; j++)
        {
            uint32 d2 = point_dist2(candidates[i], candidates[j]);
            if(d2 > best_base_d2)
            {
                best_base_d2 = d2;
                base1 = candidates[i];
                base2 = candidates[j];
            }
        }
    }

    // 第三步：全像素扫描，找离 BC 直线垂直距离最远的点 → A（角点）
    float bx = (float)((int32)base2.x - (int32)base1.x);
    float by = (float)((int32)base2.y - (int32)base1.y);
    float base_len = sqrtf(bx * bx + by * by);

    if(base_len > 0.5f)
    {
        for(uint32 p = 0; p < count; p++)
        {
            float px = (float)((int32)pts[p].x - (int32)base1.x);
            float py = (float)((int32)pts[p].y - (int32)base1.y);
            float height = fabsf(px * by - py * bx) / base_len;
            if(height > best_height)
            {
                best_height = height;
                vertex = pts[p];
            }
        }
    }

    // 第四步：验证 V 形几何
    float v1x = (float)((int32)base1.x - (int32)vertex.x);
    float v1y = (float)((int32)base1.y - (int32)vertex.y);
    float v2x = (float)((int32)base2.x - (int32)vertex.x);
    float v2y = (float)((int32)base2.y - (int32)vertex.y);
    float arm1 = sqrtf(v1x * v1x + v1y * v1y);
    float arm2 = sqrtf(v2x * v2x + v2y * v2y);

    // 余弦定理求夹角
    float dot = v1x * v2x + v1y * v2y;
    float cos_a = 1.0f;
    if(arm1 > 0.5f && arm2 > 0.5f)
    {
        cos_a = dot / (arm1 * arm2);
        if(cos_a > 1.0f) cos_a = 1.0f;
        if(cos_a < -1.0f) cos_a = -1.0f;
    }
    float angle_deg_f = acosf(cos_a) * 57.2957795f;

    // 验证通过条件
    uint8 bfs_ok = 1;
    if(base_len < (float)CAR_MARK_MIN_BASE_LEN) bfs_ok = 0;
    //  else if(arm1 < (float)CAR_MARK_MIN_ARM_LEN || arm2 < (float)CAR_MARK_MIN_ARM_LEN) bfs_ok = 0;  // 臂长检查 (暂时注释)
    else if(best_height < (float)CAR_MARK_MIN_HEIGHT) bfs_ok = 0;
    else if(angle_deg_f < (float)CAR_MARK_MIN_ANGLE_DEG || angle_deg_f > (float)CAR_MARK_MAX_ANGLE_DEG) bfs_ok = 0;

    if(bfs_ok)
    {
        // BFS 几何法验证通过, 存储特征
        uint16 base_mid_x = (base1.x + base2.x) / 2;
        uint16 base_mid_y = (base1.y + base2.y) / 2;

        b->marker_base1_x = base1.x;
        b->marker_base1_y = base1.y;
        b->marker_base2_x = base2.x;
        b->marker_base2_y = base2.y;
        b->marker_vertex_x = vertex.x;
        b->marker_vertex_y = vertex.y;
        b->marker_base_mid_x = base_mid_x;
        b->marker_base_mid_y = base_mid_y;
        b->marker_base_len = (uint16)(base_len + 0.5f);
        b->marker_height = (uint16)(best_height + 0.5f);
        b->marker_angle_deg = (uint16)(angle_deg_f + 0.5f);
        b->marker_arm1_len = (uint16)(arm1 + 0.5f);
        b->marker_arm2_len = (uint16)(arm2 + 0.5f);
        b->marker_heading = atan2f((float)((int32)vertex.y - (int32)base_mid_y),
                                   (float)((int32)vertex.x - (int32)base_mid_x));
    }
    }  // BFS 拓扑法结束

    // ===== 霍夫验证: 用霍夫直线验证 BFS 结果 =====
    // 如果霍夫直线与 BFS 两臂方向匹配, 精修端点
    // 如果不匹配, 可能是圆形信标 (降低置信度)
    if (b->marker_angle_deg > 0 && hough_cnt >= 2)
    {
        uint8 verified = hough_verify_bfs(hough_lines, hough_cnt, b);
        if (verified >= 2) {
            b->marker_hough_v = 1;  // 双臂都验证通过
        }
        // verified=1: 单臂验证, 不标记 hough_v 但端点已精修
        // verified=0: 未验证, 可能是圆形信标, 保持 BFS 结果
    }

    // ===== 第三步：轮廓暴力搜索 V 形 (兜底) =====
    // BFS 失败时, 直接枚举轮廓点采样对, 暴力搜索最佳三角形
    if (b->marker_angle_deg == 0)
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

        // 如果暴力搜索成功, 存储 V 形特征
        if (best_score > 0)
        {
            Point bv = pts[best_vertex_idx];
            Point bb1 = pts[best_base1_idx];
            Point bb2 = pts[best_base2_idx];

            uint16 base_mid_x = (bb1.x + bb2.x) / 2;
            uint16 base_mid_y = (bb1.y + bb2.y) / 2;
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

// ===== 帧间平滑: 所有检测结果统一做 EMA 低通滤波 =====
    // 霍夫/BFS/暴力搜索统一做 EMA 低通滤波, 减少帧间跳变
    // 含位移钳位: 单坐标每帧变化不超过 MAX_DELTA px, 抑制大幅抖动
    if (b->marker_angle_deg > 0)
    {
        #define BFS_EMA_ALPHA  0.60f  // 60% 新值, 40% 旧值 (快速收敛)
        #define BFS_MAX_DELTA  10.0f  // 单坐标每帧最大变化 (px)
        static float prev_vx = 0, prev_vy = 0;
        static float prev_b1x = 0, prev_b1y = 0;
        static float prev_b2x = 0, prev_b2y = 0;
        static float prev_mx = 0, prev_my = 0;
        static uint8 prev_init = 0;

        if (!prev_init) {
            prev_vx = b->marker_vertex_x; prev_vy = b->marker_vertex_y;
            prev_b1x = b->marker_base1_x; prev_b1y = b->marker_base1_y;
            prev_b2x = b->marker_base2_x; prev_b2y = b->marker_base2_y;
            prev_mx = b->marker_base_mid_x; prev_my = b->marker_base_mid_y;
            prev_init = 1;
        } else {
            float cvx = (float)b->marker_vertex_x, cvy = (float)b->marker_vertex_y;
            float cb1x = (float)b->marker_base1_x, cb1y = (float)b->marker_base1_y;
            float cb2x = (float)b->marker_base2_x, cb2y = (float)b->marker_base2_y;
            float cmx = (float)b->marker_base_mid_x, cmy = (float)b->marker_base_mid_y;

            // 位移钳位: 限制每帧最大变化, 防止大跳变
            cvx = prev_vx + CLAMP(cvx - prev_vx, -BFS_MAX_DELTA, BFS_MAX_DELTA);
            cvy = prev_vy + CLAMP(cvy - prev_vy, -BFS_MAX_DELTA, BFS_MAX_DELTA);
            cb1x = prev_b1x + CLAMP(cb1x - prev_b1x, -BFS_MAX_DELTA, BFS_MAX_DELTA);
            cb1y = prev_b1y + CLAMP(cb1y - prev_b1y, -BFS_MAX_DELTA, BFS_MAX_DELTA);
            cb2x = prev_b2x + CLAMP(cb2x - prev_b2x, -BFS_MAX_DELTA, BFS_MAX_DELTA);
            cb2y = prev_b2y + CLAMP(cb2y - prev_b2y, -BFS_MAX_DELTA, BFS_MAX_DELTA);

            // B/C 身份追踪: 检测 B 和 C 是否互换位置
            // 比较"同侧距离"和"交叉距离", 如果交叉距离更小说明互换了
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
                    // B 和 C 互换了, 交换回来
                    float tmp = cb1x; cb1x = cb2x; cb2x = tmp;
                    tmp = cb1y; cb1y = cb2y; cb2y = tmp;
                    // 中点不变 (avg 的交换不变性), 但重新计算更安全
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

            b->marker_vertex_x = (uint16)(prev_vx + 0.5f);
            b->marker_vertex_y = (uint16)(prev_vy + 0.5f);
            b->marker_base1_x = (uint16)(prev_b1x + 0.5f);
            b->marker_base1_y = (uint16)(prev_b1y + 0.5f);
            b->marker_base2_x = (uint16)(prev_b2x + 0.5f);
            b->marker_base2_y = (uint16)(prev_b2y + 0.5f);
            b->marker_base_mid_x = (uint16)(prev_mx + 0.5f);
            b->marker_base_mid_y = (uint16)(prev_my + 0.5f);
        }
    }
    // 清除宏定义, 避免跨函数污染
    #undef BFS_EMA_ALPHA
    #undef BFS_MAX_DELTA

#if DEBUG_HOUGH
    // 存储霍夫数据, 延迟到分类完成后打印 (避免信标被误记为 V 形检出)
    b->dbg_hough_cnt = hough_cnt;
    b->dbg_hough_num_angles = hough_num_angles;
    for (uint8 li = 0; li < hough_cnt && li < HOUGH_MAX_LINES; li++) {
        b->dbg_hough_rho[li]   = hough_lines[li].rho;
        b->dbg_hough_theta[li] = hough_lines[li].theta_deg;
        b->dbg_hough_votes[li] = hough_lines[li].votes;
    }
#endif
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
        // 霍夫确认的 V 形 → 直接锁定为车标, 跳过软评分
        if(blobs[i].marker_hough_v)
        {
            blobs[i].raw_marker_score = 100;
            blobs[i].raw_beacon_score = 0;
            continue;
        }

        int32 ratio = (blobs[i].height > 0)
            ? (int32)blobs[i].width * 100 / (int32)blobs[i].height
            : 100;
        int32 oratio = (blobs[i].oheight > 0)
            ? (int32)blobs[i].owidth * 100 / (int32)blobs[i].oheight
            : 100;
        int32 ord = (oratio > 100) ? (oratio - 100) : (100 - oratio);
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

        // ---- [关闭] 长条形保底车标分 ----
        // 先关掉保底逻辑，观察原始评分是否生效
        // 如果惩罚正确，长条信标分应自然低于车标分，不需要保底
        /*
        {
            if(blobs[i].raw_marker_score == 0 && ord > 30 && blobs[i].area >= CAR_MARK_MIN_AREA)
            {
                // 越细长保底分越高
                uint16 bonus = FRAG_MARKER_BONUS;  // 基础 20
                if(ord >= 60) bonus += 10;         // 特别细长的再加 10
                blobs[i].raw_marker_score = bonus;

                // 同时压低信标分: 细长条信标分直接清零
                blobs[i].raw_beacon_score = 0;
            }
        }
        */
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

// ==================== 车标丢失上下文信标抑制 ====================
// 保存上一帧的车标状态 (在 save_tracks 之前由 camera_process 更新)
static uint8  prev_frame_had_car      = 0;
static int16  prev_car_region_cx      = 0;  // 上一帧车标质心
static int16  prev_car_region_cy      = 0;
static uint16 prev_car_region_range   = 0;  // 搜索范围 = max(w,h) * 2

// 在 unknown→beacon 之后调用:
// 如果上一帧有高置信度车标而当前帧丢失, 将落入上一帧车标区域的信标抑制为 UNKNOWN
static void suppress_beacons_near_lost_marker(void)
{
    if(!prev_frame_had_car) return;             // 上一帧也没车, 不需要抑制

    // 检查当前帧是否有车标
    for(uint8 i = 0; i < blob_num; i++)
        if(blobs[i].type == BLOB_CAR_MARKER) return;   // 车标还在, 不需要抑制

    // 当前帧车标丢失: 抑制落入上一帧车标区域的信标
    int16 x1 = prev_car_region_cx - (int16)prev_car_region_range;
    int16 x2 = prev_car_region_cx + (int16)prev_car_region_range;
    int16 y1 = prev_car_region_cy - (int16)prev_car_region_range;
    int16 y2 = prev_car_region_cy + (int16)prev_car_region_range;

    for(uint8 i = 0; i < blob_num; i++)
    {
        if(blobs[i].type != BLOB_BEACON) continue;
        if((int16)blobs[i].cx >= x1 && (int16)blobs[i].cx <= x2 &&
           (int16)blobs[i].cy >= y1 && (int16)blobs[i].cy <= y2)
        {
#if DEBUG_SCORES
            printf("SUPPRESS: beacon %d (cx=%d,cy=%d) near lost car\n",
                   i, blobs[i].cx, blobs[i].cy);
#endif
            blobs[i].type = BLOB_UNKNOWN;
        }
    }
}

// 分数平滑与最终类型判决
 static void smooth_and_decide(void)
 {
     for(uint8 i = 0; i < blob_num; i++)
     {
         // 一阶滞后滤波
         blobs[i].filt_beacon_score = (uint16)(
             SCORE_HISTORY_WEIGHT * (float)blobs[i].filt_beacon_score +
             SCORE_CURRENT_WEIGHT * (float)blobs[i].raw_beacon_score);
         blobs[i].filt_marker_score = (uint16)(
             SCORE_HISTORY_WEIGHT * (float)blobs[i].filt_marker_score +
             SCORE_CURRENT_WEIGHT * (float)blobs[i].raw_marker_score);

         // 判决用分数：首帧用原始分，匹配帧用滤波分
         uint16 s_beacon = (blobs[i].track_id == 0)
             ? blobs[i].raw_beacon_score : blobs[i].filt_beacon_score;
         uint16 s_marker = (blobs[i].track_id == 0)
             ? blobs[i].raw_marker_score : blobs[i].filt_marker_score;

         // 两分类比较：车标分 > 信标分 → BLOB_CAR_MARKER，否则 BLOB_BEACON
         uint16 max_s, second_s;
         BlobType best_type;
         if(s_marker >= s_beacon)
         {
             max_s = s_marker; second_s = s_beacon;
             best_type = BLOB_CAR_MARKER;
         }
         else
         {
             max_s = s_beacon; second_s = s_marker;
             best_type = BLOB_BEACON;
         }

         // 首帧降低门槛：max≥15 且 margin≥8；匹配帧严格：max≥30 且 margin≥12
         uint8 min_conf = (blobs[i].track_id == 0) ? 15 : SCORE_MIN_CONFIDENCE;
         uint8 min_margin = (blobs[i].track_id == 0) ? 8  : SCORE_MIN_MARGIN;

         if(max_s >= min_conf && (max_s - second_s) >= min_margin)
         {
             blobs[i].type = best_type;
         }
         else if(blobs[i].track_id > 0)
         {
             // 继承历史类型，但仅当历史类型与当前最高分类一致
             BlobType hist_type = BLOB_UNKNOWN;
             for(uint8 p = 0; p < prev_track_num; p++)
             {
                 if(prev_tracks[p].track_id == blobs[i].track_id)
                 {
                     hist_type = prev_tracks[p].type;
                     break;
                 }
             }
             if(hist_type == best_type)
                 blobs[i].type = hist_type;
             else
                 blobs[i].type = best_type;
         }
         else
         {
             blobs[i].type = BLOB_UNKNOWN;
         }

        // Marker 类型额外校验：分数必须达最低阈值，角度必须在合理范围
        if(blobs[i].type == BLOB_CAR_MARKER)
        {
            uint16 ms = (blobs[i].track_id == 0) ? blobs[i].raw_marker_score : blobs[i].filt_marker_score;
            if(ms < CAR_MARK_MIN_SCORE ||
               blobs[i].marker_angle_deg < CAR_MARK_MIN_ANGLE_DEG ||
               blobs[i].marker_angle_deg > CAR_MARK_MAX_ANGLE_DEG)
            {
                blobs[i].type = BLOB_UNKNOWN;
            }
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

            float mid_x_f = (bx + cx) * 0.5f;
            float mid_y_f = y_ref;
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

        // 全局边缘提取 (供霍夫后备方案使用, 紧跟在二值化之后)
        extract_edges_once();

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

        // 更新上一帧车标状态 (供下一帧的 suppress_beacons_near_lost_marker 使用)
        // 必须在 save_tracks() 之后但仍在当前帧处理结束前更新
        {
            prev_frame_had_car = 0;
            for(uint8 i = 0; i < blob_num; i++)
            {
                if(blobs[i].type == BLOB_CAR_MARKER)
                {
                    prev_frame_had_car = 1;
                    prev_car_region_cx = (int16)blobs[i].cx;
                    prev_car_region_cy = (int16)blobs[i].cy;
                    uint16 sz = (blobs[i].width > blobs[i].height) ?
                                 blobs[i].width : blobs[i].height;
                    prev_car_region_range = (sz > 30) ? (sz * 2) : 60;
                    break;
                }
            }
        }

        // 其余未知 blob 均视为信标
        // 但当前帧有车标时，抑制车标附近的碎片变成信标（如 V 形断裂后的另一碎片）
        {
            uint8 has_car = 0;
            int16 car_cx = 0, car_cy = 0, car_range = 0;
            for(uint8 i = 0; i < blob_num; i++)
            {
                if(blobs[i].type == BLOB_CAR_MARKER)
                {
                    has_car = 1;
                    car_cx = (int16)blobs[i].cx;
                    car_cy = (int16)blobs[i].cy;
                    uint16 sz = (blobs[i].width > blobs[i].height) ?
                                 blobs[i].width : blobs[i].height;
                    car_range = (sz > 30) ? (sz * 2) : 60;
                    break;
                }
            }
            for(uint8 i = 0; i < blob_num; i++)
            {
                if(blobs[i].type == BLOB_UNKNOWN)
                {
                    if(has_car &&
                       (int16)blobs[i].cx >= car_cx - car_range &&
                       (int16)blobs[i].cx <= car_cx + car_range &&
                       (int16)blobs[i].cy >= car_cy - car_range &&
                       (int16)blobs[i].cy <= car_cy + car_range)
                    {
                        // 车标附近的碎片，不提升为信标
                        continue;
                    }
                    blobs[i].type = BLOB_BEACON;
                }
            }
        }

        // 车标丢失时抑制上一帧车标区域的信标 (防碎片被误选为信标目标)
        suppress_beacons_near_lost_marker();

#if DEBUG_HOUGH
        // 延迟打印 H 日志: 只对最终分类为 BLOB_CAR_MARKER 的 blob 打印
        // (之前是在 compute_car_marker_feature 中打印, 信标 blob 也会被误报 v_found=1)
        if ((g_vision_share.frame_id % DEBUG_HOUGH_DIV) == 0)
        {
            for (uint8 i = 0; i < blob_num; i++)
            {
                if (blobs[i].type != BLOB_CAR_MARKER) continue;
                
                Blob *b = &blobs[i];
                uint8 v_found = (b->marker_angle_deg >= CAR_MARK_MIN_ANGLE_DEG) ? 1u : 0u;
                uint8 hough_v = b->marker_hough_v;
                
                printf("H,%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u",
                       (unsigned long)g_vision_share.frame_id,
                       (unsigned int)b->area, (unsigned int)b->cx, (unsigned int)b->cy,
                       (unsigned int)b->width, (unsigned int)b->height,
                       (unsigned int)b->dbg_hough_cnt, (unsigned int)HOUGH_VOTE_THRESH,
                       (unsigned int)b->dbg_hough_num_angles,
                       (unsigned int)v_found,
                       (unsigned int)hough_v,
                       v_found ? (unsigned int)b->marker_angle_deg : 0u,
                       v_found ? (unsigned int)b->marker_height : 0u,
                       v_found ? (unsigned int)b->marker_arm1_len : 0u,
                       v_found ? (unsigned int)b->marker_arm2_len : 0u,
                       v_found ? (unsigned int)b->marker_base_len : 0u,
                       v_found ? (unsigned int)b->marker_base1_x : 0u,
                       v_found ? (unsigned int)b->marker_base1_y : 0u,
                       v_found ? (unsigned int)b->marker_base2_x : 0u,
                       v_found ? (unsigned int)b->marker_base2_y : 0u,
                       v_found ? (unsigned int)b->marker_base_mid_x : 0u,
                       v_found ? (unsigned int)b->marker_base_mid_y : 0u);
                for (uint8 li = 0; li < b->dbg_hough_cnt && li < HOUGH_MAX_LINES; li++) {
                    printf(",%d,%d,%u",
                           (int)b->dbg_hough_rho[li], (int)b->dbg_hough_theta[li],
                           (unsigned int)b->dbg_hough_votes[li]);
                }
                printf("\r\n");
            }
        }
#endif

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
        ips200_displayimage03x(image_buffer[0], CAMERA_W, CAMERA_H);
#endif

        mt9v03x_finish_flag = 0;
    }
}