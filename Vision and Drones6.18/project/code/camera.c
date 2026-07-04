#include "camera.h"
#include "HC06_Driver.h"
#include <math.h> // 使用 atan2f 需要包含 math 库

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
    BLOB_CAR_HEAD,
    BLOB_CAR_TAIL
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
    uint16 raw_head_score;
    uint16 raw_tail_score;

    // 跨帧跟踪
    uint8 track_id;                    // 跟踪 ID (0=未跟踪)
    uint16 filt_beacon_score;            // 滤波后信标分
    uint16 filt_head_score;              // 滤波后车头分
    uint16 filt_tail_score;              // 滤波后车尾分
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

        // ---- 车头分：面积偏小 + 接近圆形 ----
        int32 hs = 0;
        {   // 面积 (0~120)：小面积高分，车头通常不大
            if(area >= 3 && area <= 40)
                hs += 120;
            else if(area > 40 && area <= 100)
                hs += 120 - (area - 40) * 120 / 60;   // 40→120, 100→0
            else if(area < 3 && area >= 1)
                hs += area * 120 / 3;                 // 1→40, 3→120
            else
                hs += 0;
        }
        {   // 圆度 (0~50)
            int32 rnd = 100 - rd;
            if(rnd < 10) rnd = 10;
            hs += (rnd - 10) * 50 / 90;
        }

        // 长条惩罚：车头必须是圆形，ord>20 开始衰减，ord≥100 归零
        if(ord > 20)
        {
            int32 factor = (ord >= 100) ? 0 : (100 - ord);  // 20→80, 100→0
            hs = hs * factor / 100;
        }

        blobs[i].raw_head_score = (uint16)hs;

        // ---- 车尾分：旋转不变长条为主，面积为辅 ----
        int32 ts = 0;

        // 面积贡献 (0~40)：只有足够大才给分，面积<120 得分很少
        if(area >= 200 && area <= 500)
            ts += 40;
        else if(area > 500)
            ts += 40 - (area - 500) * 40 / 200;   // 500→40, 700→0
        else if(area >= 120)
            ts += (area - 120) * 40 / 80;         // 120→0, 200→40
        else
            ts += 0;

        // 长条偏离度贡献 (0~100)：ord 越大得分越高，主导车尾分
        // 权重提升：长条形是车尾最核心特征，单独即可拉满车尾分
        if(ord < 20)
            ts += 0;
        else if(ord >= 100)
            ts += 100;
        else
            ts += (ord - 20) * 100 / 80;           // 20→0, 100→100

        blobs[i].raw_tail_score = (uint16)ts;
    }

    // 航向一致性惩罚：利用历史航向角，对方向偏离的车头候选降分
    {
        float ref_heading = g_vision_share.heading_angle;
        // 找出临时车尾（原始车尾分最高且超过阈值）
        int16 temp_tail_x = -1, temp_tail_y = -1;
        uint8 best_tail_score = 60;
        for(uint8 i = 0; i < blob_num; i++)
        {
            if(blobs[i].raw_tail_score > best_tail_score)
            {
                best_tail_score = blobs[i].raw_tail_score;
                temp_tail_x = blobs[i].cx;
                temp_tail_y = blobs[i].cy;
            }
        }
        // 检查上一帧是否有过小车（通过 prev_tracks 判断航向是否有效）
        uint8 prev_had_car = 0;
        for(uint8 p = 0; p < prev_track_num; p++)
        {
            if(prev_tracks[p].type == BLOB_CAR_TAIL || prev_tracks[p].type == BLOB_CAR_HEAD)
            {
                prev_had_car = 1;
                break;
            }
        }
        // 有临时车尾且上一帧有车（航向有效）时执行惩罚
        if(temp_tail_x >= 0 && prev_had_car)
        {
            for(uint8 i = 0; i < blob_num; i++)
            {
                if(blobs[i].cx == temp_tail_x && blobs[i].cy == temp_tail_y)
                    continue;
                float dx = (float)blobs[i].cx - temp_tail_x;
                float dy = (float)blobs[i].cy - temp_tail_y;
                float angle = atan2f(dy, dx);
                float diff = fabsf(angle - ref_heading);
                if(diff > 3.14159265f) diff = 6.2831853f - diff;
                float diff_deg = diff * 180.0f / 3.14159265f;
                // 减法惩罚：角度偏离越大扣分越多，>20°扣80分
                if(diff_deg > 20.0f)
                {
                    int32 deduct = 80;
                    blobs[i].raw_head_score = (blobs[i].raw_head_score > deduct) ? blobs[i].raw_head_score - deduct : 0;
                }
                else
                {
                    int32 deduct = (int32)(diff_deg / 20.0f * 80.0f);
                    blobs[i].raw_head_score = (blobs[i].raw_head_score > deduct) ? blobs[i].raw_head_score - deduct : 0;
                }
            }
        }
    }

    // ---------- 位置惩罚：远离上一帧车尾的 blob 降低车尾分 ----------
    {
        int16 prev_tail_x = -1, prev_tail_y = -1;
        for(uint8 p = 0; p < prev_track_num; p++)
        {
            if(prev_tracks[p].type == BLOB_CAR_TAIL)
            {
                prev_tail_x = (int16)prev_tracks[p].cx;
                prev_tail_y = (int16)prev_tracks[p].cy;
                break;
            }
        }
        if(prev_tail_x >= 0)
        {
            const uint16 MAX_MOVE_DIST = 60;
            const uint16 MAX_PENALTY_DIST = 120;
            for(uint8 i = 0; i < blob_num; i++)
            {
                uint16 dx = (blobs[i].cx > prev_tail_x) ? (blobs[i].cx - prev_tail_x) : (prev_tail_x - blobs[i].cx);
                uint16 dy = (blobs[i].cy > prev_tail_y) ? (blobs[i].cy - prev_tail_y) : (prev_tail_y - blobs[i].cy);
                uint16 dist = dx + dy;
                if(dist > MAX_MOVE_DIST)
                {
                    float factor = (dist > MAX_PENALTY_DIST) ? 0.0f : (1.0f - (float)(dist - MAX_MOVE_DIST) / (MAX_PENALTY_DIST - MAX_MOVE_DIST));
                    blobs[i].raw_tail_score = (uint16)((float)blobs[i].raw_tail_score * factor);
                }
            }
        }
    }

    // ---------- 车尾距离惩罚：远离车尾的 blob 降低车头分 ----------
    // 信标通常离车尾不远，远离车尾的车头候选极可能是误判
    {
        int16 tail_x = -1, tail_y = -1;
        uint8 best_tail_score = 60;
        for(uint8 i = 0; i < blob_num; i++)
        {
            if(blobs[i].raw_tail_score > best_tail_score)
            {
                best_tail_score = blobs[i].raw_tail_score;
                tail_x = (int16)blobs[i].cx;
                tail_y = (int16)blobs[i].cy;
            }
        }
        if(tail_x >= 0)
        {
            const uint16 TAIL_HEAD_MAX_DIST = 60;        // 超过此距离开始扣分
            const uint16 TAIL_HEAD_PENALTY_DIST = 120;   // 此距离扣满80分
            for(uint8 i = 0; i < blob_num; i++)
            {
                if(blobs[i].cx == tail_x && blobs[i].cy == tail_y) continue;
                uint16 dx = (blobs[i].cx > tail_x) ? (blobs[i].cx - tail_x) : (tail_x - blobs[i].cx);
                uint16 dy = (blobs[i].cy > tail_y) ? (blobs[i].cy - tail_y) : (tail_y - blobs[i].cy);
                uint16 dist = dx + dy;
                if(dist > TAIL_HEAD_MAX_DIST)
                {
                    // 减法惩罚：距离越远扣分越多，>120px扣80分
                    int32 deduct = (dist > TAIL_HEAD_PENALTY_DIST) ? 80
                        : (int32)((float)(dist - TAIL_HEAD_MAX_DIST) / (TAIL_HEAD_PENALTY_DIST - TAIL_HEAD_MAX_DIST) * 80.0f);
                    blobs[i].raw_head_score = (blobs[i].raw_head_score > deduct) ? blobs[i].raw_head_score - deduct : 0;
                }
            }
        }
    }

    // 空间邻近加分：当前帧原始车尾分最高的 blob → 车头分奖励
    {
        int16 temp_tail_x = -1, temp_tail_y = -1;
        uint8 best_tail_score = 60;
        for(uint8 i = 0; i < blob_num; i++)
        {
            if(blobs[i].raw_tail_score > best_tail_score)
            {
                best_tail_score = blobs[i].raw_tail_score;
                temp_tail_x = (int16)blobs[i].cx;
                temp_tail_y = (int16)blobs[i].cy;
            }
        }
        if(temp_tail_x >= 0)
        {
            for(uint8 i = 0; i < blob_num; i++)
            {
                if(blobs[i].cx == temp_tail_x && blobs[i].cy == temp_tail_y) continue;
                uint16 dx = (blobs[i].cx > temp_tail_x) ? (blobs[i].cx - temp_tail_x) : (temp_tail_x - blobs[i].cx);
                uint16 dy = (blobs[i].cy > temp_tail_y) ? (blobs[i].cy - temp_tail_y) : (temp_tail_y - blobs[i].cy);
                uint16 dist = dx + dy;
                if(dist < 80)
                {
                    uint8 bonus = 80 - dist;
                    if(bonus > 50) bonus = 50;
                    uint16 new_head = (uint16)blobs[i].raw_head_score + bonus;
                    blobs[i].raw_head_score = (uint16)new_head;
                }
            }
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
            blobs[best_j].filt_head_score    = prev_tracks[i].filt_head_score;
            blobs[best_j].filt_tail_score    = prev_tracks[i].filt_tail_score;
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
            blobs[j].filt_head_score   = blobs[j].raw_head_score;
            blobs[j].filt_tail_score   = blobs[j].raw_tail_score;
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
         blobs[i].filt_head_score = (uint16)(
             SCORE_HISTORY_WEIGHT * (float)blobs[i].filt_head_score +
             SCORE_CURRENT_WEIGHT * (float)blobs[i].raw_head_score);
         blobs[i].filt_tail_score = (uint16)(
             SCORE_HISTORY_WEIGHT * (float)blobs[i].filt_tail_score +
             SCORE_CURRENT_WEIGHT * (float)blobs[i].raw_tail_score);

         // 判决用分数：首帧用原始分，匹配帧用滤波分
         uint16 s1 = (blobs[i].track_id == 0)
             ? blobs[i].raw_beacon_score : blobs[i].filt_beacon_score;
         uint16 s2 = (blobs[i].track_id == 0)
             ? blobs[i].raw_head_score   : blobs[i].filt_head_score;
         uint16 s3 = (blobs[i].track_id == 0)
             ? blobs[i].raw_tail_score   : blobs[i].filt_tail_score;

         // 找最高分
         uint16 max_s = s1;
         uint16 second_s = 0;
         BlobType best_type = BLOB_BEACON;

         if(s2 > max_s) { second_s = max_s; max_s = s2; best_type = BLOB_CAR_HEAD; }
         else if(s2 > second_s) second_s = s2;

         if(s3 > max_s) { second_s = max_s; max_s = s3; best_type = BLOB_CAR_TAIL; }
         else if(s3 > second_s) second_s = s3;

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
                 blobs[i].type = best_type;   // 不一致则信任当前最高分
         }
         else
         {
             blobs[i].type = BLOB_UNKNOWN;
         }
    }

    // 强制修正：低亮度（br<40）且 raw_head_score > 80 的信标改为车头
    for(uint8 i = 0; i < blob_num; i++)
    {
        if(blobs[i].type == BLOB_BEACON)
        {
            uint8 br = (blobs[i].area > 0) ? (blobs[i].core_count * 100 / blobs[i].area) : 0;
            if(br < 40 && blobs[i].raw_head_score > 80)
            {
                blobs[i].type = BLOB_CAR_HEAD;
            }
        }
    }

    }

// ==================== 旧逻辑保留区域 ====================



// 新增核心函数 2：小车头尾配对与解算
static uint8 find_car_pose(uint8 *car_x, uint8 *car_y, float *heading, Blob **head_blob, Blob **tail_blob)
{
    uint32 best_dist2 = 0;
    Blob *best_head = NULL;
    Blob *best_tail = NULL;

    for(uint8 i = 0; i < blob_num; i++)
    {
        if(blobs[i].type != BLOB_CAR_HEAD) continue;

        for(uint8 j = 0; j < blob_num; j++)
        {
            if(blobs[j].type != BLOB_CAR_TAIL) continue;

            int32 dx = (int32)blobs[i].cx - blobs[j].cx;
            int32 dy = (int32)blobs[i].cy - blobs[j].cy;
            uint32 dist2 = dx * dx + dy * dy;

            if(dist2 < CAR_MATCH_MIN_DIST2 || dist2 > CAR_MATCH_MAX_DIST2) continue;

            if(dist2 > best_dist2)
            {
                best_dist2 = dist2;
                best_head = &blobs[i];
                best_tail = &blobs[j];
            }
        }
    }

    if(best_head == NULL || best_tail == NULL) return 0;

    *car_x = (best_head->cx + best_tail->cx) / 2;
    *car_y = (best_head->cy + best_tail->cy) / 2;
    
    *heading = atan2f((float)(best_head->cy - best_tail->cy), 
                      (float)(best_head->cx - best_tail->cx));

    *head_blob = best_head;
    *tail_blob = best_tail;

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
   ips200_init(IPS200_TYPE_SPI);
    ips200_clear();
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

        // 全场至多一组车头+车尾，取滤波分最高的
        {
            uint8 best_h = 0xFF, best_t = 0xFF;
            uint16 best_hs = 0, best_ts = 0;
            for(uint8 i = 0; i < blob_num; i++)
            {
                if(blobs[i].type == BLOB_CAR_HEAD && blobs[i].filt_head_score > best_hs)
                {
                    best_hs = blobs[i].filt_head_score;
                    best_h = i;
                }
                if(blobs[i].type == BLOB_CAR_TAIL && blobs[i].filt_tail_score > best_ts)
                {
                    best_ts = blobs[i].filt_tail_score;
                    best_t = i;
                }
            }
            for(uint8 i = 0; i < blob_num; i++)
            {
                if(blobs[i].type == BLOB_CAR_HEAD && i != best_h) blobs[i].type = BLOB_UNKNOWN;
                if(blobs[i].type == BLOB_CAR_TAIL && i != best_t) blobs[i].type = BLOB_UNKNOWN;
            }
        }

        // 去重后其余未知 blob 均视为信标
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
                printf(",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                blobs[d].track_id, blobs[d].area, br,
                (blobs[d].oheight>0)?(int32)blobs[d].owidth*100/(int32)blobs[d].oheight:0,
                blobs[d].raw_beacon_score, blobs[d].raw_head_score, blobs[d].raw_tail_score,
                blobs[d].filt_beacon_score, blobs[d].filt_head_score, blobs[d].filt_tail_score,
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

        Blob *head_blob = NULL;
        Blob *tail_blob = NULL;
        uint8 car_x = 0;
        uint8 car_y = 0;
        float raw_heading = 0.0f;

        // 车头车尾独立画框（无论配对是否成功）
        for(uint8 i = 0; i < blob_num; i++)
        {
            if(blobs[i].type == BLOB_CAR_TAIL)
            {
                draw_white_box(blobs[i].cx, blobs[i].cy, 5);
                draw_black_cross(blobs[i].cx, blobs[i].cy);
            }
            else if(blobs[i].type == BLOB_CAR_HEAD)
                draw_white_box(blobs[i].cx, blobs[i].cy, 3);
        }

        uint8 found_car = find_car_pose(&car_x, &car_y, &raw_heading, &head_blob, &tail_blob);

        // 小车坐标衰减保留：检测到时更新，丢失时平滑衰减归零
        static float retain_car_x = 0.0f;
        static float retain_car_y = 0.0f;

        if(found_car)
        {
            draw_white_box(head_blob->cx, head_blob->cy, 3);
            draw_white_box(tail_blob->cx, tail_blob->cy, 5);
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
        ips200_displayimage03x(image_buffer[0], CAMERA_W, CAMERA_H);

        mt9v03x_finish_flag = 0;
    }
}