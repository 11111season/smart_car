#include "camera.h"
#include "HC06_Driver.h"
#include <math.h> // 使用 atan2f 需要包含 math 库

#define VISION_ENABLE_IPS_DISPLAY   1
#define VISION_SERIAL_DEBUG         1
#define VISION_SERIAL_DEBUG_DIV     1

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
        if(ratio < 135) return;   // ~1:1.35 以内太圆，跳过
    }

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
    if(base_len < (float)CAR_MARK_MIN_BASE_LEN) return;
    if(arm1 < (float)CAR_MARK_MIN_ARM_LEN || arm2 < (float)CAR_MARK_MIN_ARM_LEN) return;
    if(best_height < (float)CAR_MARK_MIN_HEIGHT) return;
    if(angle_deg_f < (float)CAR_MARK_MIN_ANGLE_DEG || angle_deg_f > (float)CAR_MARK_MAX_ANGLE_DEG) return;//淘汰条件：底边太短、臂太短、高太低、角度不在50°~120°

    // 第五步：存储特征
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


// 新增核心函数 ：提取并分类所有连通域
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

        // 长条惩罚：ord>30 时信标分线性衰减，ord≥130 归零
        if(ord > 30)
        {
            int32 factor = (ord >= 130) ? 0 : (130 - ord);  // 30→100, 130→0
            bs = bs * factor / 100;
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
        cross_frame_track();
        seed_new_blobs();     // 无历史的用 raw 分
        smooth_and_decide();
        save_tracks();

        // V 形车标去重：至多保留一个最好的 marker
        {
            uint8 best_m = 0xFF;
            uint16 best_ms = 0;
            for(uint8 i = 0; i < blob_num; i++)
            {
                if(blobs[i].type == BLOB_CAR_MARKER)
                {
                    uint16 s = (blobs[i].track_id == 0) ? blobs[i].raw_marker_score : blobs[i].filt_marker_score;
                    if(s > best_ms) { best_ms = s; best_m = i; }
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

        // 其余未知 blob 均视为信标
        for(uint8 i = 0; i < blob_num; i++)
        {
            if(blobs[i].type == BLOB_UNKNOWN)
                blobs[i].type = BLOB_BEACON;
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
            int16 raw_dx = (int16)best_beacon->cx - (int16)car_x;
            int16 raw_dy = (int16)best_beacon->cy - (int16)car_y;

            float cos_h = cosf(g_vision_share.heading_angle);
            float sin_h = sinf(g_vision_share.heading_angle);

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
        // ================= 帧号更新与缓存刷新 =================
        g_vision_share.frame_id++;

        // 强刷 D-Cache，保证控制核 CM7_0 能够实时读取
        SCB_CleanInvalidateDCache_by_Addr((uint32_t *)&g_vision_share, sizeof(g_vision_share));

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
        if((g_vision_share.frame_id % VISION_SERIAL_DEBUG_DIV) == 0)
        {
            printf("%lu,%u,%u,%u,%d,%d,%u,%d,%d,%d,%d,%d\r\n",
                   (unsigned long)g_vision_share.frame_id,
                   (unsigned int)blob_num,
                   (unsigned int)g_vision_share.target_found,
                   (unsigned int)g_vision_share.car_found,
                   (int)g_vision_share.car_x,
                   (int)g_vision_share.car_y,
                   (unsigned int)g_vision_share.beacon_found,
                   (int)g_vision_share.beacon_err_x,
                   (int)g_vision_share.beacon_err_y,
                   (int)g_vision_share.err_x,
                   (int)g_vision_share.err_y,
                   (int)(g_vision_share.heading_angle * 1000.0f));
        }
#endif

#if VISION_ENABLE_IPS_DISPLAY
        ips200_displayimage03x(image_buffer[0], CAMERA_W, CAMERA_H);
#endif

        mt9v03x_finish_flag = 0;
    }
}