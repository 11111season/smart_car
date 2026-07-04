#include "zf_common_headfile.h"

// 图像处理缓冲区与上位机标记点缓冲
static uint8 image_buffer[CAMERA_H][CAMERA_W];
uint8 marker_x[MARKER_POINT_NUM];
uint8 marker_y[MARKER_POINT_NUM];
static uint8 visited[CAMERA_H][CAMERA_W];
static Point queue_buf[CAMERA_W * CAMERA_H];
// 记录上一帧锁定目标，用于抑制双目标之间的频繁切换
static uint8 target_locked = 0;
static uint8 target_lock_x = 0;
static uint8 target_lock_y = 0;

/**
 * @brief 更新在上位机显示的十字标记
 * @param cx 重心 X 坐标
 * @param cy 重心 Y 坐标
 * @param is_found 是否找到目标
 */
static void update_marker(uint8 cx, uint8 cy, uint8 is_found)
{
    if (is_found)
    {
        // 中心
        marker_x[0] = cx;     marker_y[0] = cy;
        // 左
        marker_x[1] = cx - 5; marker_y[1] = cy;
        // 右
        marker_x[2] = cx + 5; marker_y[2] = cy;
        // 上
        marker_x[3] = cx;     marker_y[3] = cy - 5;
        // 下
        marker_x[4] = cx;     marker_y[4] = cy + 5;
    }
    else
    {
        // 没找到目标时，将标记点移出屏幕可见范围（x=255）
        for (uint8 i = 0; i < MARKER_POINT_NUM; i++)
        {
            marker_x[i] = 255;
            marker_y[i] = 0;
        }
    }
}

/**
 * @brief 在当前二值图上画一个黑色十字，便于屏幕观察目标中心
 * @param cx 十字中心 X 坐标
 * @param cy 十字中心 Y 坐标
 */
void draw_black_cross(uint8 cx, uint8 cy) {
    for(int i = -5; i <= 5; i++) {
        if(cy + i >= 0 && cy + i < CAMERA_H) image_buffer[cy + i][cx] = 0; 
        if(cx + i >= 0 && cx + i < CAMERA_W) image_buffer[cy][cx + i] = 0; 
    }
}

/**
 * @brief 删除面积过小的白色连通域，优先去掉零散噪点
 * @note 只修改 image_buffer，不负责选目标
 */
static void remove_small_blobs(void)
{
    const int8 dx[4] = { 1, -1, 0, 0 };
    const int8 dy[4] = { 0, 0, 1, -1 };
    uint16 x, y;

    memset(visited, 0, sizeof(visited));

    for (y = 0; y < CAMERA_H; y++)
    {
        for (x = 0; x < CAMERA_W; x++)
        {
            uint32 head, tail;

            if (visited[y][x] || image_buffer[y][x] == 0)
            {
                continue;
            }

            visited[y][x] = 1;
            queue_buf[0].x = x;
            queue_buf[0].y = y;
            head = 0;
            tail = 1;

            while (head < tail)
            {
                uint16 cx = queue_buf[head].x;
                uint16 cy = queue_buf[head].y;
                uint8 k;
                head++;

                for (k = 0; k < 4; k++)
                {
                    int16 nx = (int16)cx + dx[k];
                    int16 ny = (int16)cy + dy[k];

                    if (nx < 0 || nx >= (int16)CAMERA_W) continue;
                    if (ny < 0 || ny >= (int16)CAMERA_H) continue;
                    if (visited[ny][nx]) continue;
                    if (image_buffer[ny][nx] == 0) continue;

                    visited[ny][nx] = 1;
                    queue_buf[tail].x = (uint16)nx;
                    queue_buf[tail].y = (uint16)ny;
                    tail++;
                }
            }

            if (tail < MIN_IR_PIXEL_COUNT)
            {
                uint32 i;
                for (i = 0; i < tail; i++)
                {
                    image_buffer[queue_buf[i].y][queue_buf[i].x] = 0;
                }
            }
        }
    }
}

/**
 * @brief 在剩余候选连通域中选出当前最可信的目标中心
 * @param center_x 输出目标中心 X 坐标
 * @param center_y 输出目标中心 Y 坐标
 * @return 1 表示找到目标，0 表示没有合格目标
 * @note 同时使用宽高比、亮核比例和上一帧锁定位置做联合筛选
 */
static uint8 find_best_blob_center(uint8 *center_x, uint8 *center_y)
{
    const int8 dx[4] = { 1, -1, 0, 0 };
    const int8 dy[4] = { 0, 0, 1, -1 };
    uint16 x, y;
    uint32 best_count = 0;
    uint32 best_x_sum = 0;
    uint32 best_y_sum = 0;
    uint32 near_count = 0;
    uint32 near_x_sum = 0;
    uint32 near_y_sum = 0;
    uint32 near_dist2 = 0xFFFFFFFFU;

    memset(visited, 0, sizeof(visited));

    for (y = 0; y < CAMERA_H; y++)
    {
        for (x = 0; x < CAMERA_W; x++)
        {
            uint32 head, tail;
            uint32 cur_x_sum = 0;
            uint32 cur_y_sum = 0;
            uint32 core_count = 0;
            uint16 min_x;
            uint16 max_x;
            uint16 min_y;
            uint16 max_y;

            if (visited[y][x] || image_buffer[y][x] == 0)
            {
                continue;
            }

            visited[y][x] = 1;
            queue_buf[0].x = x;
            queue_buf[0].y = y;
            head = 0;
            tail = 1;
            min_x = x;
            max_x = x;
            min_y = y;
            max_y = y;

            while (head < tail)
            {
                uint16 cx = queue_buf[head].x;
                uint16 cy = queue_buf[head].y;
                uint8 k;
                head++;

                cur_x_sum += cx;
                cur_y_sum += cy;
                if (mt9v03x_image[cy][cx] >= IR_CORE_THRESHOLD) core_count++;
                if (cx < min_x) min_x = cx;
                if (cx > max_x) max_x = cx;
                if (cy < min_y) min_y = cy;
                if (cy > max_y) max_y = cy;

                for (k = 0; k < 4; k++)
                {
                    int16 nx = (int16)cx + dx[k];
                    int16 ny = (int16)cy + dy[k];

                    if (nx < 0 || nx >= (int16)CAMERA_W) continue;
                    if (ny < 0 || ny >= (int16)CAMERA_H) continue;
                    if (visited[ny][nx]) continue;
                    if (image_buffer[ny][nx] == 0) continue;

                    visited[ny][nx] = 1;
                    queue_buf[tail].x = (uint16)nx;
                    queue_buf[tail].y = (uint16)ny;
                    tail++;
                }
            }

            {
                uint16 width = (uint16)(max_x - min_x + 1);
                uint16 height = (uint16)(max_y - min_y + 1);
                uint32 ratio_x100;
                uint16 cur_center_x;
                uint16 cur_center_y;

                if (height == 0)
                {
                    continue;
                }

                ratio_x100 = ((uint32)width * 100U) / height;
                if (ratio_x100 < BEACON_RATIO_MIN_X100 || ratio_x100 > BEACON_RATIO_MAX_X100)
                {
                    continue;
                }

                if ((core_count * 100U) < (tail * BLOB_CORE_RATIO_MIN_X100))
                {
                    continue;
                }

                cur_center_x = (uint16)(cur_x_sum / tail);
                cur_center_y = (uint16)(cur_y_sum / tail);

                if (target_locked)
                {
                    int32 dx_lock = (int32)cur_center_x - (int32)target_lock_x;
                    int32 dy_lock = (int32)cur_center_y - (int32)target_lock_y;
                    uint32 dist2 = (uint32)(dx_lock * dx_lock + dy_lock * dy_lock);
                    uint32 radius2 = (uint32)LOCK_SEARCH_RADIUS * (uint32)LOCK_SEARCH_RADIUS;

                    if (dist2 <= radius2)
                    {
                        if (near_count == 0 || dist2 < near_dist2 || (dist2 == near_dist2 && tail > near_count))
                        {
                            near_count = tail;
                            near_x_sum = cur_x_sum;
                            near_y_sum = cur_y_sum;
                            near_dist2 = dist2;
                        }
                    }
                }
            }

            if (tail > best_count)
            {
                best_count = tail;
                best_x_sum = cur_x_sum;
                best_y_sum = cur_y_sum;
            }
        }
    }

    if (target_locked && near_count > 0 && (near_count * 100U) >= (best_count * LOCK_KEEP_RATIO_X100))
    {
        best_count = near_count;
        best_x_sum = near_x_sum;
        best_y_sum = near_y_sum;
    }

    if (best_count == 0)
    {
        target_locked = 0;
        return 0;
    }

    *center_x = (uint8)(best_x_sum / best_count);
    *center_y = (uint8)(best_y_sum / best_count);
    target_lock_x = *center_x;
    target_lock_y = *center_y;
    target_locked = 1;
    return 1;
}

/**
 * @brief 摄像头模块初始化
 */
void camera_init(void)
{
    // 状态指示灯
    gpio_init(CAMERA_STATUS_LED, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    // 初始化摄像头硬件，失败则闪烁 LED 重试
    while (mt9v03x_init())
    {
        gpio_toggle_level(CAMERA_STATUS_LED);
        system_delay_ms(500);
    }

    // 初始化 IPS200 屏幕，直接在本地显示摄像头图像
    ips200_init(IPS200_TYPE_SPI);
    ips200_clear();
    ips200_show_string(0, 0, "Camera Ready");

    printf("Camera Module Ready.\n");
}

/**
 * @brief 摄像头核心任务逻辑
 * @note 流程为：采集原图 -> 二值化 -> 去小噪点 -> 选目标 -> 画十字 -> 刷新 IPS200
 */
 void camera_process(void)
 {
     if (mt9v03x_finish_flag)
     {
         uint16 x, y;
        uint8 center_x, center_y;

         // 1. 拷贝一帧原始灰度图
         memcpy(image_buffer, mt9v03x_image, MT9V03X_IMAGE_SIZE);

        // 2. 全图固定阈值二值化
         for (y = 0; y < CAMERA_H; y++)
         {
             for (x = 0; x < CAMERA_W; x++)
             {
                if (image_buffer[y][x] >= IR_THRESHOLD)
                 {
                     image_buffer[y][x] = 255;
                 }
                 else
                 {
                     image_buffer[y][x] = 0;
                 }
             }
         }

        // 3. 删除面积过小的白色连通域，保留更像信标的亮斑
        remove_small_blobs();

        // 4. 选择剩余亮斑中面积最大的一个，计算中心点
        if (find_best_blob_center(&center_x, &center_y))
        {
            draw_black_cross(center_x, center_y);
            update_marker(center_x, center_y, 1);
        }
        else
        {
            update_marker(0, 0, 0);
        }

        // 5. 显示到 IPS200 屏幕
        ips200_displayimage03x(image_buffer[0], CAMERA_W, CAMERA_H);
        mt9v03x_finish_flag = 0;
     }
 }