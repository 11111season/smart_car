/*********************************************************************************************************************
* CYT4BB Opensourec Library
* Copyright (c) 2022 SEEKFREE
*
* 文件名称          App_Menu.c
* 功能描述          主菜单显示任务
*                   实现菜单显示控制不同任务
* 适用平台          CYT4BB
*
* 修改记录
* 日期              作者                               备注
* 2026-08-07        电子创新实验室-蓝尚旋               v1.0.0
* 2026-08-07        Claude                             v1.1.0 结构体数组化 (枚举+switch → 菜单项数组, ->/. 访问)
* 2026-08-07        Claude                             v1.2.0 一级菜单: 标题居中红字 + 整行白框高亮 + 最右*标记
* 2026-08-07        Claude                             v1.3.0 增量刷新(脏矩形): 空闲零绘制, 仅按键时重绘变化的两行
*                                                           白框改用 show_rgb565_image 一次区域写 (原 draw_line 逐点太慢)
*                                                           按键兼容短按/长按 (避免长按判空)
* 2026-08-07        Claude                             v1.4.0 页面切换滑动过渡: 新页整页从屏幕底缘向上滑入覆盖旧页
* 2026-08-07        Claude                             v1.5.0 撤销滑动过渡: 回到瞬时切页; 修复切二级整屏变白
* 2026-08-07        Claude                             v1.6.0 切页清屏改 ips200_full 显式填黑 + 选中行后恢复黑背景
* 2026-08-08        Claude                             v2.0.0 多级菜单显示框架 (纯显示, 无控制逻辑):
*                                                           · 每页统一"标题行 + 内容行", 内容行从屏幕第二行起
*                                                           · 惯导设置: 记录航点 / 清除历史数据; 数据已清除 1s 自动回二级
*                                                           · 记录航点: 按航点数循环"开始第X个→等待完成→下一航点", 完毕 1s 回主菜单
*                                                           · 无人机控制: 电机启动→请起飞→请发车→已发车, KEY_2/KEY_4 推进
*                                                           · 停止所有控制任务: 直接显示"所有任务已停止"
*                                                           · 1s 自动返回用独立节拍器 PIT_CH10 (10ms), 不与其他任务挤一个定时器
*                                                           · 已/再/次/比/赛/历/史/等/待/毕 十缺字已取模填入字库
* 2026-08-08        Claude                             v2.0.1 修复+视觉统一:
*                                                           · 修复进入自动返回页瞬间误判超时立即跳回 (进入时即记节拍起点)
*                                                           · 停止所有控制任务 1s 自动回主菜单
*                                                           · 所有提示/确认页内容行统一白底*高亮 (便于观看当前页)
* 2026-08-08        Claude                             v2.1.0 设置航点数量 (惯导设置二级中间项):
*                                                           · 二级新增"设置航点数量": 请设置记录航点数:num
*                                                           · KEY_1 ++ 封顶9, KEY_4 -- 保底1, KEY_2 确认
*                                                           · 未确认期间 num 闪烁 (200ms 亮灭交替), 确认后"航点数量设置完成" 1s 回二级
*                                                           · 记录流程改由已确认航点数 wp_set 驱动 (原 MENU_WP_COUNT 静态宏废弃)
*                                                           · 缺字 量 已由用户取模填入 menu_font.c 78
* 2026-08-08        Claude                             v2.1.1 修复: 惯导设置整页重绘循环 i<2 漏画第三行
*                                                           (清除历史数据进入即黑, 需滚动才显示) → 改 i<3
* 2026-08-08        Claude                             v2.2.0 发车区航点设置 (惯导设置二级第3项):
*                                                           · 二级新增"发车区航点设置" (设置航点数量/清除历史数据 之间)
*                                                           · 进入页: 是否启用发车区航点：是/否 (KEY_1 切换, 未确认闪烁, KEY_2 确认)
*                                                           · 选否 → "已禁用发车区航点" 1s 回二级
*                                                           · 选是 → "请输入偏移坐标" 页: 第三行设偏移x (KEY_1 +0.2 封顶+2, KEY_4 -0.2 保底-2)
*                                                                  确认后白框*跳到第四行设偏移y (同键位), x 定住不再闪烁
*                                                           · y 确认 → "发车区航点设置完毕" 1s 回二级
*                                                           · 偏移值带符号位, 存 int16 0.1m 单位 (前正后负=x, 左正右负=y)
*                                                           · 缺字 区/是/否/用/禁/输/偏/移/坐/标/米 已由用户取模填入 menu_font.c 79-89
* 2026-08-08        Claude                             v2.3.0 菜单控制逻辑接线 (重大更新, 配套 w25q64_storage + 惯导运行时航点上限):
*                                                           · 记录航点: 首次KEY_4取消闭环(车可自由推动摆车头), 之后KEY_4开始/结束记录
*                                                                      全部录完 → Inav_BuildMap + 写W25Q64地图区 → "所有航点记录完毕" 1s回二级
*                                                           · 设置航点数量: KEY_2确认 → 重算上限 + 写W25Q64设置区
*                                                           · 发车区航点设置: 确认是/否 → 写启用标志(影响总航点); y确认 → 写偏移
*                                                           · 清除历史数据: KEY_2 → FlashStore_ClearAll清两区 + 清内存 + 全局恢复默认
*                                                           · 无人机控制: KEY_2起桨/KEY_2起飞(发送指令为//占位, 遥控模拟期间注释)
*                                                                      KEY_4发车 → Inav_Launch(武装+3.5s)
*                                                           · 停止所有控制任务: Inav_StopAll → 关环+停桨+race_done完赛级锁定(断电刷新)
*                                                           · KEY_3始终退回主菜单, 记录页取消闭环放首次KEY_4 (不与菜单冲突)
* 2026-08-09        Claude                             v2.4.0 设置惯导速度限幅 (惯导设置二级第4项, 发车区航点设置/清除历史数据 之间):
*                                                           · 进入页: 第二行 x速度限幅:0.XXm/s, 第三行 y速度限制:0.XXm/s
*                                                           · KEY_1 +0.05m/s 封顶0.5, KEY_4 -0.05m/s 保底0, 未确认值闪烁
*                                                           · 两次KEY_2确认(先x后y) → 写W25Q64设置区 → "写入速度限幅成功" 1s回二级
*                                                           · 存储 flash_settings_t 扩展 8→10 字节 (尾部 spd_limit_x/y), 存0~10 (0.05m/s步进)
*                                                           · 清除历史数据 顺延为第5项, 一并清零速度限幅恢复默认0.30m/s
*                                                           · 缺字 速/度/限/幅/功 待用户取模填入 menu_font.c 90-94
* 2026-08-10        Claude                             v2.5.0 设置位置环速度限幅 (主菜单一级第3项, 无人机控制/停止所有控制任务 之间):
*                                                           · 一级菜单 3行→4行 (MAIN_ROWS[4], 重绘 i<4, 取模 %4 三处同步)
*                                                           · KEY_2进入 → x速度限幅:0.XXm/s (KEY_1 +0.05m/s 封顶0.5, KEY_4 -0.05m/s 保底0, 未确认闪烁)
*                                                           · 两次KEY_2确认(先x后y) → 写W25Q64设置区 → "写入位置环速度限幅成功" 1s回主菜单
*                                                           · 存储 flash_settings_t 扩展 10→12 字节 (尾部 pos_limit_x/y), 存0~10 (0.05m/s步进)
*                                                           · 接线: motor.c PositionControl_Update 每周期用 pos_limit_x/y 同步位置PID输出限幅 (替代硬编码 POS_V_MAX)
*                                                           · 清除历史数据 一并清零位置环限幅恢复默认0.30m/s
*                                                           · 缺字 位/环 已由用户取模填入 menu_font.c 95-96
* 2026-08-12        Claude                             v2.6.0 设置航点记录模式 (惯导设置二级第2项, 记录航点/设置航点数量 之间):
*                                                           · 二级新增"设置航点记录模式": 请选择记录模式：手动/坐标
*                                                           · 进入页: 手动默认闪烁 (KEY_1 切换 手动↔坐标, 未确认闪烁, KEY_2 确认 → 写W25Q64设置区 → 直接回二级)
*                                                           · rec_mode 存 flash_settings_t 原 _pad 字节 (旧数据0=手动, 布局不变仍12字节)
*                                                           · 手动模式: 记录航点流程与 v2.3.0 完全一致 (首次KEY_4取消闭环)
*                                                           · 坐标模式: 记录航点不取消闭环(保持), 直接进坐标输入
*                                                                      第二行"请输入第n个航点坐标"(n随wp_cur变 一/二/三), 第三行x坐标为:, 第四行y坐标为:
*                                                           · x/y 同发车区偏移逻辑: KEY_1 +0.2m KEY_4 -0.2m, 带符号位, 范围 ±9.9m (存int16 0.1m单位 ±99)
*                                                           · 每航点两次KEY_2确认(先x后y) → Inav_SetWaypoint 直写 bcn_abs[wp_cur], y左正右负存储翻转
*                                                           · 录完最后一个 → Inav_BuildMap + 写W25Q64航点地图区 → "所有航点记录完毕" 1s回二级
*                                                           · 缺字 模/式/选/择/手/为 已由用户取模填入 menu_font.c 98-103
* 2026-08-12        Claude                             v2.7.0 设置步长 (发车区偏移输入前插入步长设置页, 验证试点):
*                                                           · 发车区航点设置选"是"后, 进偏移xy前新增"请设置步长"页
*                                                           · 步长 KEY_1 +0.05m 封顶0.50, KEY_4 -0.05m 保底0.05, 无符号位, 未确认闪烁, KEY_2 确认 → 进偏移x
*                                                           · 偏移x/y 步进改由步长驱动 (原硬编码 0.2m); 步长存 flash_settings_t 新字段 launch_step (0.05m, 1~10, 默认4=0.20m)
*                                                           · 偏移值 0.1m → 0.01m 单位 (int16 ±200 = ±2.00m, 显示两位小数); W25Q64 magic 0x5A→0x5B, 老偏移作废需重设
*                                                           · 验证通过后逐步加到坐标模式等其它需要步长的地方 (未扩展, 见 TECH_NOTES 待解决)
*                                                           · 缺字 步/长 由用户取模填入 menu_font.c 104-105
* 2026-08-14        Claude                             v2.8.0 设置步长推广到坐标模式航点输入:
*                                                           · 坐标模式进"记录航点"前插入步长设置页 (复用"请设置步长"页, 新字段 coord_step)
*                                                           · coord_step 独立于 launch_step (0.05m, 1~10, 默认4=0.20m); W25Q64 magic 0x5B→0x5C 老设置作废
*                                                           · 坐标值 0.1m → 0.01m 单位 (int16 ±800 = ±8.00m, 显示两位小数); 步进 coord_step*5
*                                                           · Inav_SetWaypoint 改 0.01m 单位入参 (0.05m 步长可精确表示)
********************************************************************************************************************/

#include "App_menu.h"
#include "menu_font.h"
#include "inertial_nav.h"      // 菜单控制接口: Inav_* (记录/构建/发车/停止)
#include "w25q64_storage.h"    // W25Q64 掉电存储分区: flash_settings_t / FlashStore_*

extern volatile uint16_t menu_tick_10ms;   // 菜单节拍器 (10ms, cm7_0_isr.c pit0_ch10_isr 维护)

/*****************************************************************全局变量*****************************************************************/
int menu_index = 0;                         // 当前页高亮行 (从0开始)

/* 多级菜单状态 (每页 = 标题行 + 若干内容行) */
typedef enum{
    STATE_MAIN,                  //主菜单 (一级)
    STATE_Internal_Nav_Record,   //惯导设置 (二级): 记录航点 / 清除历史数据
    STATE_INR_CLEAR_DONE,        //数据已清除 (1s 后自动回二级)
    STATE_INR_SET_WP_COUNT,      //设置航点数量: 请设置记录航点数:num (KEY_1/KEY_4 调整, KEY_2 确认)
    STATE_INR_SET_WP_DONE,       //航点数量设置完成 (1s 后自动回二级)
    STATE_INR_WP_START,          //请开始记录第X个航点 (KEY_4 开始记录)
    STATE_INR_WP_REC,            //等待第X个航点记录完成 (KEY_4 结束记录)
    STATE_INR_WP_DONE,           //所有航点记录完毕 (1s 后自动回惯导二级)
    STATE_INR_REC_MODE,          //设置航点记录模式: 请选择记录模式：手动/坐标 (KEY_1 切换, KEY_2 确认, 未确认闪烁)
    STATE_INR_COORD_X,           //请输入第n个航点坐标: 第三行 x坐标为: (KEY_1 +0.2 封顶+9.9, KEY_4 -0.2 保底-9.9, KEY_2 确认, 闪烁)
    STATE_INR_COORD_Y,           //请输入第n个航点坐标: 第四行 y坐标为: (x 已确认, y 闪烁; KEY_2 确认 → 写第n个航点)
    STATE_INR_COORD_STEP,        //请设置步长 (坐标模式): 步长:0.XX米 (KEY_1 +0.05 封顶0.50, KEY_4 -0.05 保底0.05, 无符号位, KEY_2 确认 → 进坐标x, 闪烁)
    STATE_INR_LAUNCH_ENABLE,     //发车区航点设置: 是否启用发车区航点：是/否 (KEY_1 切换, KEY_2 确认, 未确认闪烁)
    STATE_INR_LAUNCH_STEP,       //请设置步长: 步长:0.XX米 (KEY_1 +0.05 封顶0.50, KEY_4 -0.05 保底0.05, 无符号位, KEY_2 确认, 闪烁)
    STATE_INR_LAUNCH_DISABLED,   //已禁用发车区航点 (1s 后自动回二级)
    STATE_INR_LAUNCH_OFFSET_X,   //请输入偏移坐标: 设置偏移x (第三行白框*, x 闪烁)
    STATE_INR_LAUNCH_OFFSET_Y,   //请输入偏移坐标: 设置偏移y (第四行白框*, y 闪烁, x 定住)
    STATE_INR_LAUNCH_DONE,       //发车区航点设置完毕 (1s 后自动回二级)
    STATE_INR_SPD_LIMIT_X,       //设置惯导速度限幅: 第二行 x速度限幅 (KEY_1 +0.05封顶0.5, KEY_4 -0.05保底0, KEY_2 确认, 闪烁)
    STATE_INR_SPD_LIMIT_Y,       //设置惯导速度限幅: 第三行 y速度限制 (x 已确认, y 闪烁)
    STATE_INR_SPD_LIMIT_DONE,    //写入速度限幅成功 (1s 后自动回二级)
    STATE_POS_LIMIT_X,           //设置位置环速度限幅: 第二行 x速度限幅 (键位同惯导限幅, 闪烁)
    STATE_POS_LIMIT_Y,           //设置位置环速度限幅: 第三行 y速度限制 (x 已确认, y 闪烁)
    STATE_POS_LIMIT_DONE,        //写入位置环速度限幅成功 (1s 后自动回主菜单)
    STATE_UAV_Control,           //无人机控制 (二级): 电机启动
    STATE_UAV_TAKEOFF,           //请起飞
    STATE_UAV_DEPART,            //请发车
    STATE_UAV_RACE,              //已发车 / 开始比赛
    STATE_Stop_All_Control,      //停止所有控制任务 (二级): 所有任务已停止
}MenuState;

MenuState menu_state = STATE_MAIN;          // 当前所在菜单状态

/* 惯导记录流程局部变量 */
static uint8_t  wp_cur          = 0;   // 当前记录航点 (0-based, 显示第X个)
uint8_t         wp_set          = 1;   // 已确认航点总数 (菜单"设置航点数量"改, 驱动记录流程显示次数; 不含发车区航点; 上电由W25Q64覆盖)
static uint8_t  rec_cancel_done = 0;   // 记录页第一次KEY_4是否已取消闭环 (1=已取消, 之后的KEY_4才正式记录)
static uint8_t  set_num         = 1;   // 设置页实时调整值 (进入页重置为1)
static uint8_t  set_blink_last  = 0xFF;// 上次闪烁相位 (用于闪烁变化检测)
static uint16_t auto_back_tick  = 0;   // 自动返回计时起点 (menu_tick_10ms)
static int16    coord_x         = 0;   // 坐标模式输入 x (单位0.01m, 范围 -800~+800 = ±8.00m, 前正后负)
static int16    coord_y         = 0;   // 坐标模式输入 y (单位0.01m, 范围 -800~+800 = ±8.00m, 左正右负)

/* 发车区航点设置配置 (非 static, 后续控制逻辑经 App_menu.h 读取) */
uint8_t launch_enable = 1;             // 发车区航点启用 (1=启用 0=禁用); 实际总航点数 = wp_set + launch_enable
uint8_t launch_step   = 4;             // 发车区偏移步长 (单位0.05m, 范围 1~10 → 0.05~0.50m, 默认4=0.20m; 上电由W25Q64覆盖)
uint8_t coord_step    = 4;             // 坐标模式航点输入步长 (单位0.05m, 范围 1~10 → 0.05~0.50m, 默认4=0.20m; 上电由W25Q64覆盖)
int16   launch_off_x  = 0;             // 发车区偏移x (单位0.01m, 范围 -200~+200, 前正后负, 对应小车 vx)
int16   launch_off_y  = 0;             // 发车区偏移y (单位0.01m, 范围 -200~+200, 左正右负, 对应小车 vy)

/* 惯导速度限幅 (非 static, 后续控制逻辑经 App_menu.h 读取) */
uint8_t spd_limit_x = 6;               // 惯导速度限幅x (单位0.05m/s, 范围 0~10 → 0~0.5m/s; 默认0.30m/s 与 POS_MAX 一致)
uint8_t spd_limit_y = 6;               // 惯导速度限幅y (同上, 默认0.30m/s)

/* 位置环速度限幅 (非 static, 后续控制逻辑经 App_menu.h 读取) */
uint8_t pos_limit_x = 6;               // 位置环速度限幅x (单位0.05m/s, 范围 0~10 → 0~0.5m/s; 默认0.30m/s 与 POS_V_MAX 一致)
uint8_t pos_limit_y = 6;               // 位置环速度限幅y (同上, 默认0.30m/s)

/* 航点记录模式 (非 static, 后续控制逻辑经 App_menu.h 读取) */
uint8_t rec_mode = 0;                  // 记录模式: 0=手动推车记录 1=坐标输入; 上电由W25Q64覆盖

/* 发车区/限幅/记录模式 设置页闪烁局部变量 */
static uint8_t  launch_blink_last = 0xFF;   // 发车区设置闪烁相位 (上次值)
static uint8_t  spd_blink_last    = 0xFF;   // 速度限幅设置闪烁相位 (上次值)
static uint8_t  pos_blink_last    = 0xFF;   // 位置环限幅设置闪烁相位 (上次值)
static uint8_t  rec_mode_blink_last = 0xFF; // 记录模式切换闪烁相位 (上次值)
static uint8_t  coord_blink_last   = 0xFF;  // 坐标输入闪烁相位 (上次值)

/* 当前菜单配置 → W25Q64 Region2 (设置区: 航点数量 + 发车区启用标志 + 偏移xy + 惯导/位置环速度限幅) */
static void menu_save_settings(void)
{
    flash_settings_t st;
    st.rec_mode      = rec_mode;          // 记录模式 (0=手动 1=坐标)
    st.magic         = FLASH_MAGIC;
    st.wp_set        = wp_set;
    st.launch_enable = launch_enable;
    st.launch_step   = launch_step;       // 发车区偏移步长 (0.05m, 1~10)
    st.coord_step    = coord_step;        // 坐标模式航点输入步长 (0.05m, 1~10)
    st.launch_off_x  = launch_off_x;
    st.launch_off_y  = launch_off_y;
    st.spd_limit_x   = spd_limit_x;
    st.spd_limit_y   = spd_limit_y;
    st.pos_limit_x   = pos_limit_x;
    st.pos_limit_y   = pos_limit_y;
    FlashStore_SaveSettings(&st);
    printf("FLASH: settings saved (wp=%d en=%d off=%d,%d step=%d cstep=%d spd=%d,%d pos=%d,%d rec=%d)\n",
           wp_set, launch_enable, launch_off_x, launch_off_y, launch_step, coord_step, spd_limit_x, spd_limit_y, pos_limit_x, pos_limit_y, rec_mode);
}

#define MENU_SET_MAX            9     // 航点数设置上限 (对应字模零~九; 0 = 不记录惯导, 仅发车区偏移点)
#define MENU_SET_NUM_X          152   // 设置页 num 显示 x (文字占8字128px + ':' 8px + 4px间距)
#define MENU_SET_BLINK_TICKS    20    // num 闪烁半周期: 200ms (20 × 10ms)
#define MENU_AUTO_BACK_TICKS    100   // 自动返回延迟: 1s (100 × 10ms)

/*****************************************************************一级排版常量*****************************************************************/
#define MENU_SCREEN_W    240     // IPS200 宽 (纵向)
#define MENU_TITLE_Y     4       // 标题行 y (第1行)
#define MENU_ROW_Y0      28      // 内容行第1行 y (第2行)
#define MENU_ROW_H       20      // 行高 (16px字 + 4px间距)
#define MENU_TEXT_X      12      // 行文字 x
#define MENU_STR_X       216     // * 号 x (最右, 8px边距)

/* 发车区航点设置排版: 偏移行 = 偏移(2字) + x/y(8px) + 坐标(2字) + ':'(8px) + 数值(5×8px) + 米(16px) */
#define MENU_LAUNCH_TOGGLE_X   (MENU_TEXT_X + 9*MENU_CHN_W + 8)   // 是/否 x: 是否启用发车区航点 9字 + ':' (156)
#define MENU_OFF_VAL_X         (MENU_TEXT_X + 4*MENU_CHN_W + 16)  // 偏移坐标值 x: 偏移32+x8+坐标32+:8 (92)
/* 设置步长排版: 步长(2字=32px) + ':'(8px); 数值 4字符 + 米(16px) */
#define MENU_STEP_COLON_X      (MENU_TEXT_X + 2*MENU_CHN_W)       // ':' x (44)
#define MENU_STEP_VAL_X        (MENU_STEP_COLON_X + 8)            // 数值 x (52)
#define MENU_STEP_MI_X         (MENU_STEP_VAL_X + 4*8)            // 米 x (84)

/* 设置航点记录模式排版: 手动/坐标 = 请选择记录模式 7字 + ':' (8px) 之后 */
#define MENU_REC_MODE_TOGGLE_X (MENU_TEXT_X + 7*MENU_CHN_W + 8)   // 手动/坐标 x (132)
/* 坐标输入排版: 坐标为 = x/y(8px) + 坐标为(3字=48px) + ':'(8px); 数值 x = 12+64=76, 5字符(±.XX) + 米 */
#define MENU_COORD_VAL_X       (MENU_TEXT_X + 64)                 // 坐标值 x (76, v2.8.0 起两位小数)

/* 惯导速度限幅排版: 速度限幅行 = x/y(8px) + 速度限幅/限制(4字=64px) + ':'(8px) + 数值(4×8px) + m/s(3×8px) */
#define MENU_SPD_LABEL_X       (MENU_TEXT_X + 8)                  // 速度限幅/限制 汉字起点: x/y 之后 (20)
#define MENU_SPD_COLON_X       (MENU_TEXT_X + 8 + 4*MENU_CHN_W)   // ':' x: 速度限幅 4字 后 (92)
#define MENU_SPD_VAL_X         (MENU_SPD_COLON_X + 8)             // 数值 x (100)
#define MENU_SPD_MS_X          (MENU_SPD_VAL_X + 4*8)             // m/s x: 数值 4字符 后 (132)

/*****************************************************************文字定义 (字库索引数组, 顺序任意)*****************************************************************/
/* 标题 */
static const menu_chn_idx_enum TXT_TITLE_MAIN[] = { MENU_CHN_MAI, MENU_CHN_LUN, MENU_CHN_XIAO, MENU_CHN_CHE, MENU_CHN_CAI, MENU_CHN_DAN };  // 麦轮小车菜单

/* 一级菜单 4 行 */
static const menu_chn_idx_enum TXT_GUANDAO_SHEZHI[] = { MENU_CHN_GUAN, MENU_CHN_DAO, MENU_CHN_SHE, MENU_CHN_ZHI3 };                 // 惯导设置
static const menu_chn_idx_enum TXT_WURENJI[]         = { MENU_CHN_WU2, MENU_CHN_REN, MENU_CHN_JI2, MENU_CHN_KONG, MENU_CHN_ZHI2 }; // 无人机控制
static const menu_chn_idx_enum TXT_TINGZHI_RENWU[]   = { MENU_CHN_TING, MENU_CHN_ZHI, MENU_CHN_SUO, MENU_CHN_YOU, MENU_CHN_KONG, MENU_CHN_ZHI2, MENU_CHN_REN2, MENU_CHN_WU3 };  // 停止所有控制任务

/* 惯导设置二级 + 记录流程 */
static const menu_chn_idx_enum TXT_JILU_HANGDIAN[]   = { MENU_CHN_JI, MENU_CHN_LU, MENU_CHN_HANG, MENU_CHN_DIAN };                             // 记录航点
static const menu_chn_idx_enum TXT_QINGCHU_LISHI[]   = { MENU_CHN_QING, MENU_CHN_CHU, MENU_CHN_LI, MENU_CHN_SHI2, MENU_CHN_SHU2, MENU_CHN_JU }; // 清除历史数据
static const menu_chn_idx_enum TXT_SHUJU_YI_QINGCHU[]= { MENU_CHN_SHU2, MENU_CHN_JU, MENU_CHN_YI2, MENU_CHN_QING, MENU_CHN_CHU };              // 数据已清除
static const menu_chn_idx_enum TXT_JIANSI_KAISHI[]   = { MENU_CHN_QING2, MENU_CHN_AN, MENU_CHN_XIA, MENU_CHN_JIAN, MENU_CHN_SI, MENU_CHN_KAI, MENU_CHN_SHI, MENU_CHN_JI, MENU_CHN_LU };  // 请按下按键四开始记录
static const menu_chn_idx_enum TXT_JIANSI_JIESHU[]   = { MENU_CHN_AN, MENU_CHN_XIA, MENU_CHN_JIAN, MENU_CHN_SI, MENU_CHN_JIE, MENU_CHN_SHU, MENU_CHN_JI, MENU_CHN_LU };                // 按下按键四结束记录
static const menu_chn_idx_enum TXT_SUOYOU_WANBI[]    = { MENU_CHN_SUO, MENU_CHN_YOU, MENU_CHN_HANG, MENU_CHN_DIAN, MENU_CHN_JI, MENU_CHN_LU, MENU_CHN_WAN, MENU_CHN_BI3 };              // 所有航点记录完毕
static const menu_chn_idx_enum TXT_SHEZHI_HANGDIANSHU[]        = { MENU_CHN_SHE, MENU_CHN_ZHI3, MENU_CHN_HANG, MENU_CHN_DIAN, MENU_CHN_SHU2, MENU_CHN_LIANG };               // 设置航点数量
static const menu_chn_idx_enum TXT_QING_SHEZHI_JILU_HANGDIANSHU[] = { MENU_CHN_QING2, MENU_CHN_SHE, MENU_CHN_ZHI3, MENU_CHN_JI, MENU_CHN_LU, MENU_CHN_HANG, MENU_CHN_DIAN, MENU_CHN_SHU2 };  // 请设置记录航点数
static const menu_chn_idx_enum TXT_HANGDIANSHU_SHEZHI_WANCHENG[]  = { MENU_CHN_HANG, MENU_CHN_DIAN, MENU_CHN_SHU2, MENU_CHN_LIANG, MENU_CHN_SHE, MENU_CHN_ZHI3, MENU_CHN_WAN, MENU_CHN_CHENG };  // 航点数量设置完成

/* 发车区航点设置 */
static const menu_chn_idx_enum TXT_FACHEQU_HANGDIAN_SHEZHI[]  = { MENU_CHN_FA, MENU_CHN_CHE, MENU_CHN_QU2, MENU_CHN_HANG, MENU_CHN_DIAN, MENU_CHN_SHE, MENU_CHN_ZHI3 };  // 发车区航点设置
static const menu_chn_idx_enum TXT_SHI_FOU_QIDONG_FACHEQU[]    = { MENU_CHN_SHI3, MENU_CHN_FOU, MENU_CHN_QI3, MENU_CHN_YONG, MENU_CHN_FA, MENU_CHN_CHE, MENU_CHN_QU2, MENU_CHN_HANG, MENU_CHN_DIAN };  // 是否启用发车区航点
static const menu_chn_idx_enum TXT_YI_JINYONG_FACHEQU[]        = { MENU_CHN_YI2, MENU_CHN_JIN, MENU_CHN_YONG, MENU_CHN_FA, MENU_CHN_CHE, MENU_CHN_QU2, MENU_CHN_HANG, MENU_CHN_DIAN };  // 已禁用发车区航点
static const menu_chn_idx_enum TXT_QING_SHU_RU_PIANYI_ZUOBIAO[]= { MENU_CHN_QING2, MENU_CHN_SHU3, MENU_CHN_RU, MENU_CHN_PIAN, MENU_CHN_YI3, MENU_CHN_ZUO, MENU_CHN_BIAO };  // 请输入偏移坐标
static const menu_chn_idx_enum TXT_FACHEQU_SHEZHI_WANBI[]      = { MENU_CHN_FA, MENU_CHN_CHE, MENU_CHN_QU2, MENU_CHN_HANG, MENU_CHN_DIAN, MENU_CHN_SHE, MENU_CHN_ZHI3, MENU_CHN_WAN, MENU_CHN_BI3 };  // 发车区航点设置完毕
static const menu_chn_idx_enum TXT_PIAN_YI[]                   = { MENU_CHN_PIAN, MENU_CHN_YI3 };   // 偏移
static const menu_chn_idx_enum TXT_ZUO_BIAO[]                  = { MENU_CHN_ZUO, MENU_CHN_BIAO };   // 坐标
static const menu_chn_idx_enum TXT_MI[]                        = { MENU_CHN_MI };                   // 米
static const menu_chn_idx_enum TXT_BU_CHANG[]                  = { MENU_CHN_BU, MENU_CHN_CHANG };   // 步长
static const menu_chn_idx_enum TXT_QING_SHEZHI_BUCHANG[]       = { MENU_CHN_QING2, MENU_CHN_SHE, MENU_CHN_ZHI3, MENU_CHN_BU, MENU_CHN_CHANG };  // 请设置步长
static const menu_chn_idx_enum TXT_LAUNCH_TOGGLE[]             = { MENU_CHN_SHI3, MENU_CHN_FOU };   // 是/否 (切换用, 与 launch_enable 对应)

/* 设置航点记录模式 */
static const menu_chn_idx_enum TXT_SHEZHI_JILU_MOSHI[]       = { MENU_CHN_SHE, MENU_CHN_ZHI3, MENU_CHN_JI, MENU_CHN_LU, MENU_CHN_HANG, MENU_CHN_DIAN, MENU_CHN_MO, MENU_CHN_SHI4 };  // 设置航点记录模式
static const menu_chn_idx_enum TXT_QING_XUANZE_JILU_MOSHI[]  = { MENU_CHN_QING2, MENU_CHN_XUAN, MENU_CHN_ZE, MENU_CHN_JI, MENU_CHN_LU, MENU_CHN_MO, MENU_CHN_SHI4 };  // 请选择记录模式
static const menu_chn_idx_enum TXT_REC_MODE_TOGGLE[]         = { MENU_CHN_SHOU, MENU_CHN_DONG, MENU_CHN_ZUO, MENU_CHN_BIAO };   // 手动/坐标 (切换用, 与 rec_mode 对应)
static const menu_chn_idx_enum TXT_WEI_ZUO_BIAO[]            = { MENU_CHN_WEI2, MENU_CHN_ZUO, MENU_CHN_BIAO };                  // 坐标为 (x/y 行后缀)

/* 惯导速度限幅 */
static const menu_chn_idx_enum TXT_SHEZHI_SUDU_XIANFU[]        = { MENU_CHN_SHE, MENU_CHN_ZHI3, MENU_CHN_GUAN, MENU_CHN_DAO, MENU_CHN_SU, MENU_CHN_DU2, MENU_CHN_XIAN, MENU_CHN_FU };  // 设置惯导速度限幅
static const menu_chn_idx_enum TXT_SUDU_XIANFU[]               = { MENU_CHN_SU, MENU_CHN_DU2, MENU_CHN_XIAN, MENU_CHN_FU };            // 速度限幅 (x 行用)
static const menu_chn_idx_enum TXT_SUDU_XIANZHI[]              = { MENU_CHN_SU, MENU_CHN_DU2, MENU_CHN_XIAN, MENU_CHN_ZHI2 };          // 速度限制 (y 行用)
static const menu_chn_idx_enum TXT_XIERU_SUDU_XIANFU_CHENGONG[]= { MENU_CHN_XIE, MENU_CHN_RU, MENU_CHN_SU, MENU_CHN_DU2, MENU_CHN_XIAN, MENU_CHN_FU, MENU_CHN_CHENG, MENU_CHN_GONG };  // 写入速度限幅成功

/* 位置环速度限幅 */
static const menu_chn_idx_enum TXT_SHEZHI_WEIZHIHUAN_SUDU_XIANFU[]        = { MENU_CHN_SHE, MENU_CHN_ZHI3, MENU_CHN_WEI, MENU_CHN_ZHI3, MENU_CHN_HUAN, MENU_CHN_SU, MENU_CHN_DU2, MENU_CHN_XIAN, MENU_CHN_FU };  // 设置位置环速度限幅
static const menu_chn_idx_enum TXT_XIERU_WEIZHIHUAN_SUDU_XIANFU_CHENGONG[]= { MENU_CHN_XIE, MENU_CHN_RU, MENU_CHN_WEI, MENU_CHN_ZHI3, MENU_CHN_HUAN, MENU_CHN_SU, MENU_CHN_DU2, MENU_CHN_XIAN, MENU_CHN_FU, MENU_CHN_CHENG, MENU_CHN_GONG };  // 写入位置环速度限幅成功

/* 无人机控制二级 + 起飞流程 */
static const menu_chn_idx_enum TXT_DIANJI_QIDONG[]   = { MENU_CHN_DIAN2, MENU_CHN_JI2, MENU_CHN_QI3, MENU_CHN_DONG };                          // 电机启动
static const menu_chn_idx_enum TXT_JIANER_QIFEI[]    = { MENU_CHN_QING2, MENU_CHN_AN, MENU_CHN_XIA, MENU_CHN_JIAN, MENU_CHN_ER, MENU_CHN_QI2, MENU_CHN_FEI };  // 请按下按键二起飞
static const menu_chn_idx_enum TXT_QING_QIFEI[]      = { MENU_CHN_QING2, MENU_CHN_QI2, MENU_CHN_FEI };                                         // 请起飞
static const menu_chn_idx_enum TXT_ZAICI_QIFEI[]     = { MENU_CHN_ZAI, MENU_CHN_CI, MENU_CHN_AN, MENU_CHN_XIA, MENU_CHN_JIAN, MENU_CHN_ER, MENU_CHN_QI2, MENU_CHN_FEI };  // 再次按下按键二起飞
static const menu_chn_idx_enum TXT_QING_FACHE[]      = { MENU_CHN_QING2, MENU_CHN_FA, MENU_CHN_CHE };                                         // 请发车
static const menu_chn_idx_enum TXT_JIANSI[]          = { MENU_CHN_QING2, MENU_CHN_AN, MENU_CHN_XIA, MENU_CHN_JIAN, MENU_CHN_SI };              // 请按下按键四
static const menu_chn_idx_enum TXT_YI_FACHE[]        = { MENU_CHN_YI2, MENU_CHN_FA, MENU_CHN_CHE };                                            // 已发车
static const menu_chn_idx_enum TXT_KAISHI_BISAI[]    = { MENU_CHN_KAI, MENU_CHN_SHI, MENU_CHN_BI2, MENU_CHN_SAI };                            // 开始比赛

/* 停止所有控制任务 */
static const menu_chn_idx_enum TXT_SUOYOU_RENWU_YITINGZHI[] = { MENU_CHN_SUO, MENU_CHN_YOU, MENU_CHN_REN2, MENU_CHN_WU3, MENU_CHN_YI2, MENU_CHN_TING, MENU_CHN_ZHI };  // 所有任务已停止

/* KEY_1 增量重绘用的行文字表 (下标=行号) */
static const menu_text_t MAIN_ROWS[4] = {
    { TXT_GUANDAO_SHEZHI,               4 },   // 惯导设置
    { TXT_WURENJI,                      5 },   // 无人机控制
    { TXT_SHEZHI_WEIZHIHUAN_SUDU_XIANFU, 9 },   // 设置位置环速度限幅
    { TXT_TINGZHI_RENWU,                8 },   // 停止所有控制任务
};
static const menu_text_t INR_ROWS[6] = {
    { TXT_JILU_HANGDIAN,            4 },    // 记录航点
    { TXT_SHEZHI_JILU_MOSHI,        8 },    // 设置航点记录模式
    { TXT_SHEZHI_HANGDIANSHU,       6 },    // 设置航点数量
    { TXT_FACHEQU_HANGDIAN_SHEZHI,  7 },    // 发车区航点设置
    { TXT_SHEZHI_SUDU_XIANFU,       8 },    // 设置惯导速度限幅
    { TXT_QINGCHU_LISHI,            6 },    // 清除历史数据
};

/*****************************************************************局部函数*****************************************************************/
static uint16 menu_fill_buf[MENU_SCREEN_W];   // 480B: 填充矩形用的一行solid缓存

/* 填充矩形: 一次区域设置 + 整块连续写 (比 draw_line 逐点快约 3 个数量级) */
static void menu_fill_rect(uint16 x1, uint16 y1, uint16 x2, uint16 y2, uint16 color)
{
    uint16 w = x2 - x1 + 1;
    uint16 h = y2 - y1 + 1;
    uint16 i;
    for (i = 0; i < w; i++) menu_fill_buf[i] = color;
    /* 每显示行都采样同一份 solid 缓存 → 整块同色填充; color_mode=0 走16bit写, 与文字渲染一致 */
    ips200_show_rgb565_image(x1, y1, menu_fill_buf, w, 1, w, h, 0);
}

/* 由字库索引数组构造文本 (数组必须是真数组, 不能用指针) */
static menu_text_t menu_make_text(const menu_chn_idx_enum *chars, uint8_t num)
{
    menu_text_t t;
    t.chars = chars;
    t.num   = num;
    return t;
}
#define MT(arr)  menu_make_text((arr), (uint8_t)(sizeof(arr)/sizeof((arr)[0])))

/* 绘制标题行 (居中红字, 黑底) */
static void menu_draw_title(const menu_chn_idx_enum *chars, uint8_t num)
{
    menu_text_t t = menu_make_text(chars, num);
    ips200_set_color(RGB565_RED, RGB565_BLACK);
    menu_show_text((MENU_SCREEN_W - num*MENU_CHN_W)/2, MENU_TITLE_Y, &t, RGB565_RED);
}

/* 绘制内容行 (line_idx=0 → 屏幕第2行): selected=1 白框红字*, 0 黑底红字 */
static void menu_draw_line(uint8_t line_idx, menu_text_t t, uint8_t selected)
{
    uint16 y = MENU_ROW_Y0 + line_idx*MENU_ROW_H;
    if (selected)
    {
        menu_fill_rect(0, y, MENU_SCREEN_W - 1, y + MENU_ROW_H - 1, RGB565_WHITE);
        ips200_set_color(RGB565_RED, RGB565_WHITE);
        menu_show_text(MENU_TEXT_X, y + 2, &t, RGB565_RED);
        ips200_show_char(MENU_STR_X, y + 2, '*');
        ips200_set_color(RGB565_RED, RGB565_BLACK);  // 画完立即恢复黑背景, 杜绝残留白色污染后续清屏
    }
    else
    {
        menu_fill_rect(0, y, MENU_SCREEN_W - 1, y + MENU_ROW_H - 1, RGB565_BLACK);
        ips200_set_color(RGB565_RED, RGB565_BLACK);
        menu_show_text(MENU_TEXT_X, y + 2, &t, RGB565_RED);
    }
}

/* 运行时拼装 "请开始记录第X个航点"(kind=0) / "等待第X个航点记录完成"(kind=1), X 由 wp_cur 决定 */
static menu_text_t menu_wp_line(uint8_t kind)
{
    static menu_text_t         t;
    static menu_chn_idx_enum   buf[12];
    static const menu_chn_idx_enum PRE_START[] = { MENU_CHN_QING2, MENU_CHN_KAI, MENU_CHN_SHI, MENU_CHN_JI, MENU_CHN_LU };  // 请开始记录
    static const menu_chn_idx_enum PRE_REC[]   = { MENU_CHN_DENG, MENU_CHN_DAI };  // 等待
    static const menu_chn_idx_enum NUM[]       = { MENU_CHN_YI, MENU_CHN_ER, MENU_CHN_SAN, MENU_CHN_SI, MENU_CHN_WU,
                                                   MENU_CHN_LIU, MENU_CHN_QI, MENU_CHN_BA, MENU_CHN_JIU };                  // 一~九
    static const menu_chn_idx_enum TAIL[]      = { MENU_CHN_GE, MENU_CHN_HANG, MENU_CHN_DIAN };                             // 个航点
    static const menu_chn_idx_enum DONE[]      = { MENU_CHN_JI, MENU_CHN_LU, MENU_CHN_WAN, MENU_CHN_CHENG };               // 记录完成
    const menu_chn_idx_enum *pre   = (kind == 0) ? PRE_START : PRE_REC;
    uint8_t pre_n = (kind == 0) ? 5 : 2;
    uint8_t n = 0, i;

    for (i = 0; i < pre_n; i++) buf[n++] = pre[i];
    buf[n++] = MENU_CHN_DI;                          // 第
    buf[n++] = (wp_cur < 9) ? NUM[wp_cur] : NUM[8];  // X (0→一, 超9用九兜底)
    for (i = 0; i < 3; i++) buf[n++] = TAIL[i];      // 个航点
    if (kind == 1) for (i = 0; i < 4; i++) buf[n++] = DONE[i];  // 记录完成
    t.chars = buf;
    t.num   = n;
    return t;
}

/* 设置航点数量页: 画 num (红字零~九, 落在白底上) */
static void menu_draw_set_num(void)
{
    static const menu_chn_idx_enum NUM[] = { MENU_CHN_LING, MENU_CHN_YI, MENU_CHN_ER, MENU_CHN_SAN, MENU_CHN_SI,
                                             MENU_CHN_WU, MENU_CHN_LIU, MENU_CHN_QI, MENU_CHN_BA, MENU_CHN_JIU };  // 零~九
    menu_text_t t;
    t.chars = &NUM[(set_num <= MENU_SET_MAX) ? set_num : MENU_SET_MAX];
    t.num   = 1;
    menu_show_text(MENU_SET_NUM_X, MENU_ROW_Y0 + 2, &t, RGB565_RED);
}

/* 设置航点数量页: 擦 num (白底上填白, 闪烁灭相位用) */
static void menu_erase_set_num(void)
{
    menu_fill_rect(MENU_SET_NUM_X, MENU_ROW_Y0, MENU_SET_NUM_X + MENU_CHN_W - 1, MENU_ROW_Y0 + MENU_ROW_H - 1, RGB565_WHITE);
}

/* 设置航点数量页: 画完整第二行 (白底* + 文字 + ':' + num) */
static void menu_draw_set_line(void)
{
    menu_draw_line(0, MT(TXT_QING_SHEZHI_JILU_HANGDIANSHU), 1);   // 请设置记录航点数 (白底*)
    ips200_set_color(RGB565_RED, RGB565_WHITE);                    // ':' 与 num 都落在白底上
    ips200_show_char(MENU_TEXT_X + 8*MENU_CHN_W, MENU_ROW_Y0 + 2, ':');
    menu_draw_set_num();
    ips200_set_color(RGB565_RED, RGB565_BLACK);                    // 画完恢复黑背景
}

/* 发车区偏移值 → 4 个 ASCII 字符: 符号 + 整数位 + 小数点 + 小数位 (如 +1.2 / -0.8 / +2.0) */
static void menu_show_offset_val(int16 v_tenths, uint16 x, uint16 y)
{
    uint16 absv = (v_tenths < 0) ? (uint16)(-v_tenths) : (uint16)v_tenths;
    ips200_show_char(x,      y, (v_tenths < 0) ? '-' : '+');
    ips200_show_char(x +  8, y, (char)('0' + absv / 10));
    ips200_show_char(x + 16, y, '.');
    ips200_show_char(x + 24, y, (char)('0' + absv % 10));
}

/* 发车区偏移值 (0.01m 单位) → 5 个 ASCII 字符: 符号 + 整数位 + 小数点 + 两位小数 (如 +2.00 / -0.15 / +0.05)
 * v2.7.0: 步长最小 0.05m → 偏移需两位小数分辨率 (原 0.1m 单位只显示一位) */
static void menu_show_offset_val_cm(int16 v_cent, uint16 x, uint16 y)
{
    uint16 absv = (v_cent < 0) ? (uint16)(-v_cent) : (uint16)v_cent;
    ips200_show_char(x,      y, (v_cent < 0) ? '-' : '+');
    ips200_show_char(x +  8, y, (char)('0' + absv / 100));        // 整数位 (0~2)
    ips200_show_char(x + 16, y, '.');
    ips200_show_char(x + 24, y, (char)('0' + (absv / 10) % 10));  // 十分位
    ips200_show_char(x + 32, y, (char)('0' + absv % 10));         // 百分位
}

/* 只重绘偏移行中的数值区 (按键调整 / 闪烁用, 不重绘整行) */
static void menu_draw_offset_val_only(uint8_t line_idx, uint8_t is_x, uint8_t selected, uint8_t visible)
{
    uint16 y = MENU_ROW_Y0 + line_idx*MENU_ROW_H;
    uint16 bg = selected ? RGB565_WHITE : RGB565_BLACK;
    if (visible)
    {
        ips200_set_color(RGB565_RED, bg);
        menu_show_offset_val_cm(is_x ? launch_off_x : launch_off_y, MENU_OFF_VAL_X, y + 2);
        if (selected) ips200_set_color(RGB565_RED, RGB565_BLACK);
    }
    else
    {
        menu_fill_rect(MENU_OFF_VAL_X, y, MENU_OFF_VAL_X + 5*8 - 1, y + MENU_ROW_H - 1, bg);  // 底色盖掉 = 灭相位
    }
}

/* 偏移坐标整行: 偏 移 x/y 坐 标 : ±.XX米 (0.01m 单位两位小数); selected=1 白框*, 0 黑底红字 */
static void menu_draw_offset_line(uint8_t line_idx, uint8_t is_x, uint8_t selected, uint8_t val_visible)
{
    uint16 y = MENU_ROW_Y0 + line_idx*MENU_ROW_H;
    uint16 bg = selected ? RGB565_WHITE : RGB565_BLACK;
    menu_text_t t_py = MT(TXT_PIAN_YI);    // 偏移
    menu_text_t t_zb = MT(TXT_ZUO_BIAO);   // 坐标
    menu_text_t t_mi = MT(TXT_MI);         // 米

    menu_fill_rect(0, y, MENU_SCREEN_W - 1, y + MENU_ROW_H - 1, bg);
    ips200_set_color(RGB565_RED, bg);
    menu_show_text(MENU_TEXT_X,                    y + 2, &t_py, RGB565_RED);               // 偏移
    ips200_show_char(MENU_TEXT_X + 2*MENU_CHN_W,   y + 2, is_x ? 'x' : 'y');                // x/y
    menu_show_text(MENU_TEXT_X + 2*MENU_CHN_W + 8, y + 2, &t_zb, RGB565_RED);               // 坐标
    ips200_show_char(MENU_TEXT_X + 4*MENU_CHN_W + 8, y + 2, ':');                           // :
    if (val_visible) menu_show_offset_val_cm(is_x ? launch_off_x : launch_off_y, MENU_OFF_VAL_X, y + 2);   // 数值
    else             menu_fill_rect(MENU_OFF_VAL_X, y, MENU_OFF_VAL_X + 5*8 - 1, y + MENU_ROW_H - 1, bg);
    menu_show_text(MENU_OFF_VAL_X + 5*8, y + 2, &t_mi, RGB565_RED);                         // 米
    if (selected) ips200_show_char(MENU_STR_X, y + 2, '*');
    if (selected) ips200_set_color(RGB565_RED, RGB565_BLACK);                               // 恢复黑背景
}

/* 速度限幅值 → 4 个 ASCII 字符: 整数位 + 小数点 + 两位小数 (如 0.30 / 0.05, 无符号位, 0~0.5)
 * v_steps 单位 0.05m/s, 0~10; 百分位 = 步数×5 (0/5/10/.../50) */
static void menu_show_spd_val(uint8_t v_steps, uint16 x, uint16 y)
{
    uint16 cent = (uint16)v_steps * 5;             // 0.01m/s 单位: 0~50
    ips200_show_char(x,      y, '0');              // 整数位恒为 0
    ips200_show_char(x +  8, y, '.');
    ips200_show_char(x + 16, y, (char)('0' + cent / 10));          // 十分位
    ips200_show_char(x + 24, y, (char)('0' + cent % 10));          // 百分位
}

/* 只重绘速度限幅行中的数值区 (按键调整 / 闪烁用, 不重绘整行); v = 值源指针 (惯导 spd_limit 或 位置环 pos_limit) */
static void menu_draw_spd_val_only(uint8_t line_idx, const uint8_t *v, uint8_t selected, uint8_t visible)
{
    uint16 y = MENU_ROW_Y0 + line_idx*MENU_ROW_H;
    uint16 bg = selected ? RGB565_WHITE : RGB565_BLACK;
    if (visible)
    {
        ips200_set_color(RGB565_RED, bg);
        menu_show_spd_val(*v, MENU_SPD_VAL_X, y + 2);
        if (selected) ips200_set_color(RGB565_RED, RGB565_BLACK);
    }
    else
    {
        menu_fill_rect(MENU_SPD_VAL_X, y, MENU_SPD_VAL_X + 4*8 - 1, y + MENU_ROW_H - 1, bg);  // 底色盖掉 = 灭相位
    }
}

/* 速度限幅整行: x/y 速度限幅/限制 : 0.XXm/s; v = 值源指针, selected=1 白框*, 0 黑底红字 */
static void menu_draw_spd_line(uint8_t line_idx, const uint8_t *v, uint8_t is_x, uint8_t selected, uint8_t val_visible)
{
    uint16 y = MENU_ROW_Y0 + line_idx*MENU_ROW_H;
    uint16 bg = selected ? RGB565_WHITE : RGB565_BLACK;
    menu_text_t t_lab = is_x ? MT(TXT_SUDU_XIANFU) : MT(TXT_SUDU_XIANZHI);   // 速度限幅 / 速度限制

    menu_fill_rect(0, y, MENU_SCREEN_W - 1, y + MENU_ROW_H - 1, bg);
    ips200_set_color(RGB565_RED, bg);
    ips200_show_char(MENU_TEXT_X,                  y + 2, is_x ? 'x' : 'y');   // x/y
    menu_show_text(MENU_SPD_LABEL_X,               y + 2, &t_lab, RGB565_RED); // 速度限幅/限制
    ips200_show_char(MENU_SPD_COLON_X,             y + 2, ':');                // :
    if (val_visible) menu_show_spd_val(*v, MENU_SPD_VAL_X, y + 2);             // 数值
    else             menu_fill_rect(MENU_SPD_VAL_X, y, MENU_SPD_VAL_X + 4*8 - 1, y + MENU_ROW_H - 1, bg);
    ips200_show_string(MENU_SPD_MS_X,              y + 2, "m/s");              // m/s
    if (selected) ips200_show_char(MENU_STR_X, y + 2, '*');
    if (selected) ips200_set_color(RGB565_RED, RGB565_BLACK);                  // 恢复黑背景
}

/* 只重绘步长行中的数值区 (按键调整 / 闪烁用, 不重绘整行); v = 值源指针 (发车区 launch_step / 坐标模式 coord_step), 0.05m 单位, 复用速度限幅显示 (0.XX) */
static void menu_draw_step_val_only(uint8_t line_idx, const uint8_t *v, uint8_t selected, uint8_t visible)
{
    uint16 y = MENU_ROW_Y0 + line_idx*MENU_ROW_H;
    uint16 bg = selected ? RGB565_WHITE : RGB565_BLACK;
    if (visible)
    {
        ips200_set_color(RGB565_RED, bg);
        menu_show_spd_val(*v, MENU_STEP_VAL_X, y + 2);
        if (selected) ips200_set_color(RGB565_RED, RGB565_BLACK);
    }
    else
    {
        menu_fill_rect(MENU_STEP_VAL_X, y, MENU_STEP_VAL_X + 4*8 - 1, y + MENU_ROW_H - 1, bg);  // 底色盖掉 = 灭相位
    }
}

/* 步长整行: 步 长 : 0.XX 米; v = 值源指针 (launch_step/coord_step), selected=1 白框*, 0 黑底红字 */
static void menu_draw_step_line(uint8_t line_idx, const uint8_t *v, uint8_t selected, uint8_t val_visible)
{
    uint16 y = MENU_ROW_Y0 + line_idx*MENU_ROW_H;
    uint16 bg = selected ? RGB565_WHITE : RGB565_BLACK;
    menu_text_t t_bc = MT(TXT_BU_CHANG);   // 步长
    menu_text_t t_mi = MT(TXT_MI);         // 米

    menu_fill_rect(0, y, MENU_SCREEN_W - 1, y + MENU_ROW_H - 1, bg);
    ips200_set_color(RGB565_RED, bg);
    menu_show_text(MENU_TEXT_X, y + 2, &t_bc, RGB565_RED);                    // 步长
    ips200_show_char(MENU_STEP_COLON_X, y + 2, ':');                          // :
    if (val_visible) menu_show_spd_val(*v, MENU_STEP_VAL_X, y + 2);           // 0.XX
    else             menu_fill_rect(MENU_STEP_VAL_X, y, MENU_STEP_VAL_X + 4*8 - 1, y + MENU_ROW_H - 1, bg);
    menu_show_text(MENU_STEP_MI_X, y + 2, &t_mi, RGB565_RED);                 // 米
    if (selected) ips200_show_char(MENU_STR_X, y + 2, '*');
    if (selected) ips200_set_color(RGB565_RED, RGB565_BLACK);                 // 恢复黑背景
}

/* 是否启用发车区航点： 后面的 是/否 (白底红字, 由 launch_enable 决定) */
static void menu_draw_launch_toggle(void)
{
    menu_text_t t;
    t.chars = &TXT_LAUNCH_TOGGLE[launch_enable ? 0 : 1];   // 1=是, 0=否
    t.num   = 1;
    ips200_set_color(RGB565_RED, RGB565_WHITE);
    menu_show_text(MENU_LAUNCH_TOGGLE_X, MENU_ROW_Y0 + 2, &t, RGB565_RED);
    ips200_set_color(RGB565_RED, RGB565_BLACK);
}

/* 擦除是/否 (闪烁灭相位用, 白底填白) */
static void menu_erase_launch_toggle(void)
{
    menu_fill_rect(MENU_LAUNCH_TOGGLE_X, MENU_ROW_Y0, MENU_LAUNCH_TOGGLE_X + MENU_CHN_W - 1,
                   MENU_ROW_Y0 + MENU_ROW_H - 1, RGB565_WHITE);
}

/* 发车区航点设置: 是否启用发车区航点：是/否 (整行白底*, 是/否 由调用方闪烁) */
static void menu_draw_launch_enable_line(void)
{
    menu_draw_line(0, MT(TXT_SHI_FOU_QIDONG_FACHEQU), 1);   // 是否启用发车区航点 (白底*)
    menu_draw_launch_toggle();                               // 是/否
}

/* 手动/坐标 切换值 (白底红字, 由 rec_mode 决定) */
static void menu_draw_rec_mode_toggle(void)
{
    menu_text_t t;
    t.chars = &TXT_REC_MODE_TOGGLE[rec_mode ? 2 : 0];   // 0=手动, 1=坐标
    t.num   = 2;
    ips200_set_color(RGB565_RED, RGB565_WHITE);
    menu_show_text(MENU_REC_MODE_TOGGLE_X, MENU_ROW_Y0 + 2, &t, RGB565_RED);
    ips200_set_color(RGB565_RED, RGB565_BLACK);
}

/* 擦除手动/坐标 (闪烁灭相位用, 白底填白) */
static void menu_erase_rec_mode_toggle(void)
{
    menu_fill_rect(MENU_REC_MODE_TOGGLE_X, MENU_ROW_Y0, MENU_REC_MODE_TOGGLE_X + 2*MENU_CHN_W - 1,
                   MENU_ROW_Y0 + MENU_ROW_H - 1, RGB565_WHITE);
}

/* 设置航点记录模式页: 请选择记录模式：手动/坐标 (整行白底*, 手动/坐标 由调用方闪烁) */
static void menu_draw_rec_mode_line(void)
{
    menu_draw_line(0, MT(TXT_QING_XUANZE_JILU_MOSHI), 1);   // 请选择记录模式 (白底*)
    menu_draw_rec_mode_toggle();                             // 手动/坐标
}

/* 运行时拼装 "请输入第n个航点坐标", n 由 wp_cur 决定 (0→一, 与 menu_wp_line 数字一致) */
static menu_text_t menu_coord_line(void)
{
    static menu_text_t         t;
    static menu_chn_idx_enum   buf[12];
    static const menu_chn_idx_enum PRE[]  = { MENU_CHN_QING2, MENU_CHN_SHU3, MENU_CHN_RU, MENU_CHN_DI };  // 请输入第
    static const menu_chn_idx_enum NUM[]  = { MENU_CHN_YI, MENU_CHN_ER, MENU_CHN_SAN, MENU_CHN_SI, MENU_CHN_WU,
                                              MENU_CHN_LIU, MENU_CHN_QI, MENU_CHN_BA, MENU_CHN_JIU };     // 一~九
    static const menu_chn_idx_enum TAIL[] = { MENU_CHN_GE, MENU_CHN_HANG, MENU_CHN_DIAN, MENU_CHN_ZUO, MENU_CHN_BIAO };  // 个航点坐标
    uint8_t n = 0, i;

    for (i = 0; i < 4; i++) buf[n++] = PRE[i];              // 请输入第
    buf[n++] = (wp_cur < 9) ? NUM[wp_cur] : NUM[8];         // n (0→一, 超9用九兜底)
    for (i = 0; i < 5; i++) buf[n++] = TAIL[i];             // 个航点坐标
    t.chars = buf;
    t.num   = n;
    return t;
}

/* 只重绘坐标行中的数值区 (按键调整 / 闪烁用, 不重绘整行); 0.01m 单位两位小数 (同发车区偏移) */
static void menu_draw_coord_val_only(uint8_t line_idx, uint8_t is_x, uint8_t selected, uint8_t visible)
{
    uint16 y = MENU_ROW_Y0 + line_idx*MENU_ROW_H;
    uint16 bg = selected ? RGB565_WHITE : RGB565_BLACK;
    if (visible)
    {
        ips200_set_color(RGB565_RED, bg);
        menu_show_offset_val_cm(is_x ? coord_x : coord_y, MENU_COORD_VAL_X, y + 2);
        if (selected) ips200_set_color(RGB565_RED, RGB565_BLACK);
    }
    else
    {
        menu_fill_rect(MENU_COORD_VAL_X, y, MENU_COORD_VAL_X + 5*8 - 1, y + MENU_ROW_H - 1, bg);  // 底色盖掉 = 灭相位
    }
}

/* 坐标输入整行: x/y 坐标为 : ±.XX米 (0.01m 单位两位小数); selected=1 白框*, 0 黑底红字 */
static void menu_draw_coord_line(uint8_t line_idx, uint8_t is_x, uint8_t selected, uint8_t val_visible)
{
    uint16 y = MENU_ROW_Y0 + line_idx*MENU_ROW_H;
    uint16 bg = selected ? RGB565_WHITE : RGB565_BLACK;
    menu_text_t t_wz = MT(TXT_WEI_ZUO_BIAO);   // 坐标为
    menu_text_t t_mi = MT(TXT_MI);             // 米

    menu_fill_rect(0, y, MENU_SCREEN_W - 1, y + MENU_ROW_H - 1, bg);
    ips200_set_color(RGB565_RED, bg);
    ips200_show_char(MENU_TEXT_X,                       y + 2, is_x ? 'x' : 'y');   // x/y
    menu_show_text(MENU_TEXT_X + 8,                     y + 2, &t_wz, RGB565_RED);  // 坐标为
    ips200_show_char(MENU_TEXT_X + 8 + 3*MENU_CHN_W,    y + 2, ':');                // :
    if (val_visible) menu_show_offset_val_cm(is_x ? coord_x : coord_y, MENU_COORD_VAL_X, y + 2);   // 数值
    else             menu_fill_rect(MENU_COORD_VAL_X, y, MENU_COORD_VAL_X + 5*8 - 1, y + MENU_ROW_H - 1, bg);
    menu_show_text(MENU_COORD_VAL_X + 5*8, y + 2, &t_mi, RGB565_RED);               // 米
    if (selected) ips200_show_char(MENU_STR_X, y + 2, '*');
    if (selected) ips200_set_color(RGB565_RED, RGB565_BLACK);                       // 恢复黑背景
}

/* 整页重绘 (页面切换时调用): 标题行 + 全部内容行 */
static void menu_draw_screen(MenuState st)
{
    uint8_t i;

    switch (st)
    {
    case STATE_MAIN:
        menu_draw_title(TXT_TITLE_MAIN, 6);
        for (i = 0; i < 4; i++) menu_draw_line(i, MAIN_ROWS[i], (i == (uint8_t)menu_index));
        break;

    case STATE_Internal_Nav_Record:
        menu_draw_title(TXT_GUANDAO_SHEZHI, 4);
        for (i = 0; i < 6; i++) menu_draw_line(i, INR_ROWS[i], (i == (uint8_t)menu_index));
        break;

    case STATE_INR_CLEAR_DONE:
        menu_draw_title(TXT_GUANDAO_SHEZHI, 4);
        menu_draw_line(0, MT(TXT_SHUJU_YI_QINGCHU), 1);   // 数据已清除
        break;

    case STATE_INR_SET_WP_COUNT:
        menu_draw_title(TXT_GUANDAO_SHEZHI, 4);
        menu_draw_set_line();                             // 请设置记录航点数:num (num 未确认期间闪烁)
        break;

    case STATE_INR_SET_WP_DONE:
        menu_draw_title(TXT_GUANDAO_SHEZHI, 4);
        menu_draw_line(0, MT(TXT_HANGDIANSHU_SHEZHI_WANCHENG), 1);  // 航点数量设置完成
        break;

    case STATE_INR_WP_START:
        menu_draw_title(TXT_GUANDAO_SHEZHI, 4);
        menu_draw_line(0, menu_wp_line(0), 1);            // 请开始记录第X个航点
        menu_draw_line(1, MT(TXT_JIANSI_KAISHI), 1);      // 请按下按键四开始记录
        break;

    case STATE_INR_WP_REC:
        menu_draw_title(TXT_GUANDAO_SHEZHI, 4);
        menu_draw_line(0, menu_wp_line(1), 1);            // 等待第X个航点记录完成
        menu_draw_line(1, MT(TXT_JIANSI_JIESHU), 1);      // 按下按键四结束记录
        break;

    case STATE_INR_WP_DONE:
        menu_draw_title(TXT_GUANDAO_SHEZHI, 4);
        menu_draw_line(0, MT(TXT_SUOYOU_WANBI), 1);       // 所有航点记录完毕
        break;

    case STATE_INR_REC_MODE:
        menu_draw_title(TXT_GUANDAO_SHEZHI, 4);
        menu_draw_rec_mode_line();                        // 请选择记录模式：手动/坐标 (整行白底*)
        break;

    case STATE_INR_COORD_X:
        menu_draw_title(TXT_GUANDAO_SHEZHI, 4);
        menu_draw_line(0, menu_coord_line(), 0);          // 请输入第n个航点坐标 (黑底红字)
        menu_draw_coord_line(1, 1, 1, 1);                 // x坐标为:±..米 (白框*, x 可见)
        menu_draw_coord_line(2, 0, 0, 1);                 // y坐标为:±..米 (黑底)
        break;

    case STATE_INR_COORD_Y:
        menu_draw_title(TXT_GUANDAO_SHEZHI, 4);
        menu_draw_line(0, menu_coord_line(), 0);          // 请输入第n个航点坐标 (黑底红字)
        menu_draw_coord_line(1, 1, 0, 1);                 // x坐标为:±..米 (已定, 黑底)
        menu_draw_coord_line(2, 0, 1, 1);                 // y坐标为:±..米 (白框*, y 可见)
        break;

    case STATE_INR_COORD_STEP:
        menu_draw_title(TXT_GUANDAO_SHEZHI, 4);
        menu_draw_line(0, MT(TXT_QING_SHEZHI_BUCHANG), 0);   // 请设置步长 (黑底红字)
        menu_draw_step_line(1, &coord_step, 1, 1);           // 步长:0.XX米 (白框*, 可见)
        break;

    case STATE_INR_LAUNCH_ENABLE:
        menu_draw_title(TXT_GUANDAO_SHEZHI, 4);
        menu_draw_launch_enable_line();                   // 是否启用发车区航点：是/否 (整行白底*)
        break;

    case STATE_INR_LAUNCH_DISABLED:
        menu_draw_title(TXT_GUANDAO_SHEZHI, 4);
        menu_draw_line(0, MT(TXT_YI_JINYONG_FACHEQU), 1); // 已禁用发车区航点
        break;

    case STATE_INR_LAUNCH_STEP:
        menu_draw_title(TXT_GUANDAO_SHEZHI, 4);
        menu_draw_line(0, MT(TXT_QING_SHEZHI_BUCHANG), 0);   // 请设置步长 (黑底红字)
        menu_draw_step_line(1, &launch_step, 1, 1);          // 步长:0.XX米 (白框*, 可见)
        break;

    case STATE_INR_LAUNCH_OFFSET_X:
        menu_draw_title(TXT_GUANDAO_SHEZHI, 4);
        menu_draw_line(0, MT(TXT_QING_SHU_RU_PIANYI_ZUOBIAO), 0);   // 请输入偏移坐标 (黑底红字)
        menu_draw_offset_line(1, 1, 1, 1);                // 偏移x坐标:±.XX米 (白框*, x 可见)
        menu_draw_offset_line(2, 0, 0, 1);                // 偏移y坐标:±.XX米 (黑底)
        break;

    case STATE_INR_LAUNCH_OFFSET_Y:
        menu_draw_title(TXT_GUANDAO_SHEZHI, 4);
        menu_draw_line(0, MT(TXT_QING_SHU_RU_PIANYI_ZUOBIAO), 0);   // 请输入偏移坐标 (黑底红字)
        menu_draw_offset_line(1, 1, 0, 1);                // 偏移x坐标:±.XX米 (已定, 黑底)
        menu_draw_offset_line(2, 0, 1, 1);                // 偏移y坐标:±.XX米 (白框*, y 可见)
        break;

    case STATE_INR_LAUNCH_DONE:
        menu_draw_title(TXT_GUANDAO_SHEZHI, 4);
        menu_draw_line(0, MT(TXT_FACHEQU_SHEZHI_WANBI), 1);         // 发车区航点设置完毕
        break;

    case STATE_INR_SPD_LIMIT_X:
        menu_draw_title(TXT_GUANDAO_SHEZHI, 4);
        menu_draw_spd_line(0, &spd_limit_x, 1, 1, 1);  // x速度限幅:0.XXm/s (白框*, x 可见)
        menu_draw_spd_line(1, &spd_limit_y, 0, 0, 1);  // y速度限制:0.XXm/s (黑底)
        break;

    case STATE_INR_SPD_LIMIT_Y:
        menu_draw_title(TXT_GUANDAO_SHEZHI, 4);
        menu_draw_spd_line(0, &spd_limit_x, 1, 0, 1);  // x速度限幅:0.XXm/s (已定, 黑底)
        menu_draw_spd_line(1, &spd_limit_y, 0, 1, 1);  // y速度限制:0.XXm/s (白框*, y 可见)
        break;

    case STATE_INR_SPD_LIMIT_DONE:
        menu_draw_title(TXT_GUANDAO_SHEZHI, 4);
        menu_draw_line(0, MT(TXT_XIERU_SUDU_XIANFU_CHENGONG), 1);   // 写入速度限幅成功
        break;

    case STATE_POS_LIMIT_X:
        menu_draw_title(TXT_SHEZHI_WEIZHIHUAN_SUDU_XIANFU, 9);
        menu_draw_spd_line(0, &pos_limit_x, 1, 1, 1);  // x速度限幅:0.XXm/s (白框*, x 可见)
        menu_draw_spd_line(1, &pos_limit_y, 0, 0, 1);  // y速度限制:0.XXm/s (黑底)
        break;

    case STATE_POS_LIMIT_Y:
        menu_draw_title(TXT_SHEZHI_WEIZHIHUAN_SUDU_XIANFU, 9);
        menu_draw_spd_line(0, &pos_limit_x, 1, 0, 1);  // x速度限幅:0.XXm/s (已定, 黑底)
        menu_draw_spd_line(1, &pos_limit_y, 0, 1, 1);  // y速度限制:0.XXm/s (白框*, y 可见)
        break;

    case STATE_POS_LIMIT_DONE:
        menu_draw_title(TXT_SHEZHI_WEIZHIHUAN_SUDU_XIANFU, 9);
        menu_draw_line(0, MT(TXT_XIERU_WEIZHIHUAN_SUDU_XIANFU_CHENGONG), 1);   // 写入位置环速度限幅成功
        break;

    case STATE_UAV_Control:
        menu_draw_title(TXT_WURENJI, 5);
        menu_draw_line(0, MT(TXT_DIANJI_QIDONG), 1);      // 电机启动
        menu_draw_line(1, MT(TXT_JIANER_QIFEI), 1);       // 提示: 请按下按键二起飞
        break;

    case STATE_UAV_TAKEOFF:
        menu_draw_title(TXT_WURENJI, 5);
        menu_draw_line(0, MT(TXT_QING_QIFEI), 1);         // 请起飞
        menu_draw_line(1, MT(TXT_ZAICI_QIFEI), 1);        // 再次按下按键二起飞
        break;

    case STATE_UAV_DEPART:
        menu_draw_title(TXT_WURENJI, 5);
        menu_draw_line(0, MT(TXT_QING_FACHE), 1);         // 请发车
        menu_draw_line(1, MT(TXT_JIANSI), 1);             // 请按下按键四
        break;

    case STATE_UAV_RACE:
        menu_draw_title(TXT_WURENJI, 5);
        menu_draw_line(0, MT(TXT_YI_FACHE), 1);           // 已发车
        menu_draw_line(1, MT(TXT_KAISHI_BISAI), 1);       // 开始比赛
        break;

    case STATE_Stop_All_Control:
        menu_draw_title(TXT_TINGZHI_RENWU, 8);
        menu_draw_line(0, MT(TXT_SUOYOU_RENWU_YITINGZHI), 1);  // 所有任务已停止
        break;

    default:
        break;
    }
}

void App_Menu_Init(void)
{
    ips200_init(IPS200_TYPE_SPI);
    ips200_set_font(IPS200_8X16_FONT);          // ASCII (* 号等) 用 8x16, 与 16px 汉字等高
    printf("MENU v2.7.0\r\n");                  // 版本标记: 串口确认烧录的是最新固件
    ips200_set_color(RGB565_RED, RGB565_BLACK);
    ips200_full(RGB565_BLACK);                  // 显式填黑, 不依赖全局背景色
    menu_draw_screen(menu_state);               // 开机绘制一次主界面
}

void App_Menu_Task(void)
{
    static MenuState prev_state = STATE_MAIN;   // 记录上一页, 页面切换时全页重绘
    key_state_enum k;
    uint8_t old;

    /* 0. 页面切换 → 清屏 + 整页重绘 (空闲时零 SPI 流量)
     * 注意: 清屏用 ips200_full 显式填黑, 不依赖全局背景色 */
    if (menu_state != prev_state)
    {
        ips200_set_color(RGB565_RED, RGB565_BLACK);
        ips200_full(RGB565_BLACK);
        menu_index = 0;                          // 进入新页默认选中第1行
        if (menu_state == STATE_INR_CLEAR_DONE || menu_state == STATE_INR_SET_WP_DONE || menu_state == STATE_INR_WP_DONE || menu_state == STATE_Stop_All_Control
            || menu_state == STATE_INR_LAUNCH_DISABLED || menu_state == STATE_INR_LAUNCH_DONE || menu_state == STATE_INR_SPD_LIMIT_DONE || menu_state == STATE_POS_LIMIT_DONE)
            auto_back_tick = menu_tick_10ms;     // 进入自动返回状态, 记录节拍起点
        menu_draw_screen(menu_state);
        prev_state = menu_state;
    }

    switch (menu_state)
    {
    /* 一级: 主菜单 (KEY_1 循环高亮, KEY_2 进入分支) */
    case STATE_MAIN:
        k = key_get_state(KEY_1);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            old = (uint8_t)menu_index;
            menu_index++;
            menu_index %= 4;
            key_clear_state(KEY_1);
            menu_draw_line(old, MAIN_ROWS[old], 0);                        // 旧行: 取消选中
            menu_draw_line((uint8_t)menu_index, MAIN_ROWS[menu_index], 1); // 新行: 选中
        }
        k = key_get_state(KEY_2);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_2);
            if      (menu_index == 0) menu_state = STATE_Internal_Nav_Record;             // 惯导设置
            else if (menu_index == 1) menu_state = STATE_UAV_Control;                     // 无人机控制
            else if (menu_index == 2) { pos_blink_last = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1); menu_state = STATE_POS_LIMIT_X; }  // 设置位置环速度限幅 → 限幅x (值初始可见)
            else { Inav_StopAll(); menu_state = STATE_Stop_All_Control; auto_back_tick = menu_tick_10ms; } // 停止所有控制任务 (关环+停桨+完赛级锁定, 提示1s后回主菜单)
        }
        break;

    /* 惯导设置二级 (KEY_1 循环高亮, KEY_2 进入对应流程) */
    case STATE_Internal_Nav_Record:
        k = key_get_state(KEY_1);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            old = (uint8_t)menu_index;
            menu_index++;
            menu_index %= 6;
            key_clear_state(KEY_1);
            menu_draw_line(old, INR_ROWS[old], 0);                        // 旧行: 取消选中
            menu_draw_line((uint8_t)menu_index, INR_ROWS[menu_index], 1); // 新行: 选中
        }
        k = key_get_state(KEY_2);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_2);
            if (menu_index == 0) {                                                                                       // 记录航点 → 按记录模式分流程
                if (wp_set == 0) {
                    /* v1.4.9: 0航点 = 不记录惯导, 发车区偏移点为唯一惯导点 → 直接构建导航航点 + 提示"所有航点记录完毕" */
                    Inav_ResetMap();
                    Inav_BuildMap();
                    menu_state = STATE_INR_WP_DONE;
                    auto_back_tick = menu_tick_10ms;
                } else if (rec_mode == 1) {
                    /* 坐标模式: 不取消闭环(保持), 先设步长 → 再进坐标输入; 每航点两次KEY_2确认(先x后y) */
                    wp_cur = 0; coord_x = 0; coord_y = 0; Inav_ResetMap();
                    coord_blink_last = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
                    menu_state = STATE_INR_COORD_STEP;
                } else {
                    /* 手动模式: 与 v2.3.0 完全一致 (首次KEY4取消闭环, 之后KEY4开始/结束记录; 重进先清内存航点防残留) */
                    wp_cur = 0; rec_cancel_done = 0; Inav_ResetMap(); menu_state = STATE_INR_WP_START;
                }
            }
            else if (menu_index == 1) { rec_mode_blink_last = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1); menu_state = STATE_INR_REC_MODE; }          // 设置航点记录模式 → 切换页 (手动/坐标初始可见)
            else if (menu_index == 2) { set_num = 1; set_blink_last = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1); menu_state = STATE_INR_SET_WP_COUNT; }  // 设置航点数量 → 调整页 (num初始可见)
            else if (menu_index == 3) { launch_blink_last = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1); menu_state = STATE_INR_LAUNCH_ENABLE; }          // 发车区航点设置 → 启用页 (是/否初始可见)
            else if (menu_index == 4) { spd_blink_last = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1); menu_state = STATE_INR_SPD_LIMIT_X; }              // 设置惯导速度限幅 → 限幅x (值初始可见)
            else {
                FlashStore_ClearAll();       // 清 W25Q64: 航点地图 + 设置(航点数量/发车区启用标志/偏移/步长/惯导+位置环速度限幅/记录模式)
                Inav_ResetMap();             // 清内存航点
                wp_set = 1; launch_enable = 0; launch_off_x = 0; launch_off_y = 0;  // 恢复默认 (发车区标志清零)
                launch_step = 4;                    // 恢复默认步长 0.20m
                coord_step  = 4;                    // 恢复默认步长 0.20m
                spd_limit_x = 6; spd_limit_y = 6;   // 恢复默认限幅 0.30m/s (与 POS_MAX 一致)
                pos_limit_x = 6; pos_limit_y = 6;   // 恢复默认限幅 0.30m/s (与 POS_V_MAX 一致)
                rec_mode = 0;                       // 恢复默认记录模式: 手动推车
                Inav_UpdateMax();            // 重算 bcn_max/wp_max (=1, 无发车区)
                printf("FLASH: all data cleared\n");
                menu_state = STATE_INR_CLEAR_DONE;      // 数据已清除 → 提示1s
                auto_back_tick = menu_tick_10ms;
            }
        }
        break;

    /* 设置航点数量: KEY_1 ++ (封顶9), KEY_4 -- (保底0), KEY_2 确认; num 未确认期间闪烁
     * v1.4.9: 允许0航点 = 不记录惯导, 仅用发车区偏移点当唯一惯导点 (PlanB, 赛场可能不允许推车打点) */
    case STATE_INR_SET_WP_COUNT:
        k = key_get_state(KEY_1);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_1);
            if (set_num < MENU_SET_MAX) set_num++;   // 封顶9, 到顶不再递增
            menu_draw_set_num();
        }
        k = key_get_state(KEY_4);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_4);
            if (set_num > 0) set_num--;              // 保底0, 到底不再递减
            menu_draw_set_num();
        }
        k = key_get_state(KEY_2);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_2);
            wp_set = set_num;                        // 正式确认航点数量
            Inav_UpdateMax();                        // 重算 bcn_max/wp_max
            menu_save_settings();                    // 写 W25Q64 Region2 (航点数量)
            menu_state = STATE_INR_SET_WP_DONE;
            auto_back_tick = menu_tick_10ms;         // 提示1s后回二级
        }
        /* 闪烁: 未确认期间 num 按 200ms 亮灭交替 */
        {
            uint8_t blink = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
            if (blink != set_blink_last)
            {
                set_blink_last = blink;
                if (blink) menu_draw_set_num();
                else       menu_erase_set_num();
            }
        }
        break;

    /* 惯导记录流程: 第一次KEY_4取消闭环(车可自由推动摆车头), 之后 KEY_4 开始/结束记录, 按已确认航点数 wp_set 循环 */
    case STATE_INR_WP_START:
        k = key_get_state(KEY_4);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_4);
            if (!rec_cancel_done) {
                Inav_DisableControl();   // 第一次按键4: 取消闭环 (PID全关+PWM清零), 车可自由推动
                rec_cancel_done = 1;
            } else {
                Inav_RecStart();         // 从第二次起: 正式开始记录当前航点
                menu_state = STATE_INR_WP_REC;
            }
        }
        break;

    case STATE_INR_WP_REC:
        k = key_get_state(KEY_4);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_4);
            Inav_RecEnd();               // 结束记录当前航点 (存 bcn_abs[bcn_idx++])
            if (wp_cur + 1 < wp_set) { wp_cur++; menu_state = STATE_INR_WP_START; }   // 还有下一个航点 (v1.4.9: 原 wp_cur<wp_set-1 在 wp_set=0 时下溢成255死循环, 改加法比较)
            else {
                Inav_BuildMap();                                   // 全部录完: 构建导航航点 (含发车区航点)
                FlashStore_SaveWaypoints(Inav_GetMap(), wp_set);   // 写 W25Q64 Region1 (航点地图)
                printf("FLASH: waypoints saved (%d)\n", wp_set);
                menu_state = STATE_INR_WP_DONE;                    // 所有航点记录完毕
                auto_back_tick = menu_tick_10ms;                   // 1s后回二级
            }
        }
        break;

    /* 设置航点记录模式: 请选择记录模式：手动/坐标 (KEY_1 切换, 未确认闪烁, KEY_2 确认) */
    case STATE_INR_REC_MODE:
        k = key_get_state(KEY_1);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_1);
            rec_mode ^= 1;                                            // 手动↔坐标 切换
            menu_draw_rec_mode_toggle();
            rec_mode_blink_last = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);  // 切换后从可见相位继续闪烁
        }
        k = key_get_state(KEY_2);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_2);
            menu_save_settings();        // 写 W25Q64 Region2 (记录模式)
            menu_state = STATE_Internal_Nav_Record;                     // 确认后直接回二级
        }
        {
            uint8_t blink = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
            if (blink != rec_mode_blink_last)
            {
                rec_mode_blink_last = blink;
                if (blink) menu_draw_rec_mode_toggle();
                else       menu_erase_rec_mode_toggle();
            }
        }
        break;

    /* 请输入第n个航点坐标: 设置 x (KEY_1 +步长 封顶+8.00, KEY_4 -步长 保底-8.00, KEY_2 确认, x 闪烁) */
    case STATE_INR_COORD_X:
        k = key_get_state(KEY_1);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_1);
            if (coord_x < 800) coord_x += (int16)coord_step * 5;   // 每次 +步长 (0.05m×5=0.01m单位), 封顶 +8.00
            menu_draw_coord_val_only(1, 1, 1, 1);
            coord_blink_last = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
        }
        k = key_get_state(KEY_4);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_4);
            if (coord_x > -800) coord_x -= (int16)coord_step * 5;  // 每次 -步长, 保底 -8.00
            menu_draw_coord_val_only(1, 1, 1, 1);
            coord_blink_last = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
        }
        k = key_get_state(KEY_2);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_2);
            coord_blink_last = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
            menu_state = STATE_INR_COORD_Y;                     // x 定住, 进入设置 y
        }
        {
            uint8_t blink = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
            if (blink != coord_blink_last)
            {
                coord_blink_last = blink;
                if (blink) menu_draw_coord_val_only(1, 1, 1, 1);
                else       menu_draw_coord_val_only(1, 1, 1, 0);
            }
        }
        break;

    /* 请输入第n个航点坐标: 设置 y (x 已定住不再闪烁, y 闪烁; KEY_2 确认 → 直写第n个航点) */
    case STATE_INR_COORD_Y:
        k = key_get_state(KEY_1);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_1);
            if (coord_y < 800) coord_y += (int16)coord_step * 5;   // 每次 +步长, 封顶 +8.00
            menu_draw_coord_val_only(2, 0, 1, 1);
            coord_blink_last = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
        }
        k = key_get_state(KEY_4);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_4);
            if (coord_y > -800) coord_y -= (int16)coord_step * 5;  // 每次 -步长, 保底 -8.00
            menu_draw_coord_val_only(2, 0, 1, 1);
            coord_blink_last = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
        }
        k = key_get_state(KEY_2);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_2);
            Inav_SetWaypoint(wp_cur, coord_x, coord_y);         // 直写 bcn_abs[wp_cur] (y左正右负存储翻转)
            if (wp_cur + 1 < wp_set) { wp_cur++; menu_state = STATE_INR_COORD_X; }   // 还有下一个航点 → 继续输入 (坐标沿用上一航点, 便于连续相近点)
            else {
                Inav_BuildMap();                                   // 全部录完: 构建导航航点 (含发车区航点)
                FlashStore_SaveWaypoints(Inav_GetMap(), wp_set);   // 写 W25Q64 Region1 (航点地图)
                printf("FLASH: waypoints saved (%d)\n", wp_set);
                menu_state = STATE_INR_WP_DONE;                    // 所有航点记录完毕
                auto_back_tick = menu_tick_10ms;                   // 1s后回二级
            }
        }
        {
            uint8_t blink = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
            if (blink != coord_blink_last)
            {
                coord_blink_last = blink;
                if (blink) menu_draw_coord_val_only(2, 0, 1, 1);
                else       menu_draw_coord_val_only(2, 0, 1, 0);
            }
        }
        break;

    /* 请设置步长 (坐标模式): 步长:0.XX米 (KEY_1 +0.05 封顶0.50, KEY_4 -0.05 保底0.05, 无符号位, KEY_2 确认 → 进坐标x, 闪烁) */
    case STATE_INR_COORD_STEP:
        k = key_get_state(KEY_1);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_1);
            if (coord_step < 10) coord_step++;                          // 每次 +0.05m, 封顶 0.50
            menu_draw_step_val_only(1, &coord_step, 1, 1);
            coord_blink_last = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
        }
        k = key_get_state(KEY_4);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_4);
            if (coord_step > 1) coord_step--;                           // 每次 -0.05m, 保底 0.05
            menu_draw_step_val_only(1, &coord_step, 1, 1);
            coord_blink_last = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
        }
        k = key_get_state(KEY_2);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_2);
            menu_save_settings();          // 写 W25Q64 Region2 (步长; 航点录完写地图区)
            coord_blink_last = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
            menu_state = STATE_INR_COORD_X;                             // 步长确认 → 输入坐标x
        }
        {
            uint8_t blink = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
            if (blink != coord_blink_last)
            {
                coord_blink_last = blink;
                if (blink) menu_draw_step_val_only(1, &coord_step, 1, 1);
                else       menu_draw_step_val_only(1, &coord_step, 1, 0);
            }
        }
        break;

    /* 发车区航点设置: 是否启用发车区航点：是/否 (KEY_1 切换, 未确认闪烁, KEY_2 确认) */
    case STATE_INR_LAUNCH_ENABLE:
        k = key_get_state(KEY_1);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_1);
            launch_enable ^= 1;                                            // 是↔否 切换
            menu_draw_launch_toggle();
            launch_blink_last = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);  // 切换后从可见相位继续闪烁
        }
        k = key_get_state(KEY_2);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_2);
            Inav_UpdateMax();            // launch_enable 变化 → 重算 wp_max
            menu_save_settings();        // 写 W25Q64 Region2 (发车区启用标志, 影响总航点计算)
            if (launch_enable) { launch_blink_last = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1); menu_state = STATE_INR_LAUNCH_STEP; }   // 启用 → 先设步长
            else { menu_state = STATE_INR_LAUNCH_DISABLED; auto_back_tick = menu_tick_10ms; }                // 禁用 → 提示1s
        }
        {
            uint8_t blink = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
            if (blink != launch_blink_last)
            {
                launch_blink_last = blink;
                if (blink) menu_draw_launch_toggle();
                else       menu_erase_launch_toggle();
            }
        }
        break;

    /* 请设置步长: 步长:0.XX米 (KEY_1 +0.05m 封顶0.50, KEY_4 -0.05m 保底0.05, 无符号位, KEY_2 确认 → 进偏移x, 闪烁) */
    case STATE_INR_LAUNCH_STEP:
        k = key_get_state(KEY_1);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_1);
            if (launch_step < 10) launch_step++;                          // 每次 +0.05m, 封顶 0.50
            menu_draw_step_val_only(1, &launch_step, 1, 1);
            launch_blink_last = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
        }
        k = key_get_state(KEY_4);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_4);
            if (launch_step > 1) launch_step--;                           // 每次 -0.05m, 保底 0.05
            menu_draw_step_val_only(1, &launch_step, 1, 1);
            launch_blink_last = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
        }
        k = key_get_state(KEY_2);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_2);
            menu_save_settings();          // 写 W25Q64 Region2 (步长; 偏移仍在下面设置)
            launch_blink_last = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
            menu_state = STATE_INR_LAUNCH_OFFSET_X;                       // 步长确认 → 输入偏移x
        }
        {
            uint8_t blink = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
            if (blink != launch_blink_last)
            {
                launch_blink_last = blink;
                if (blink) menu_draw_step_val_only(1, &launch_step, 1, 1);
                else       menu_draw_step_val_only(1, &launch_step, 1, 0);
            }
        }
        break;

    /* 请输入偏移坐标: 设置偏移x (KEY_1 +步长 封顶+2.00, KEY_4 -步长 保底-2.00, KEY_2 确认, x 闪烁) */
    case STATE_INR_LAUNCH_OFFSET_X:
        k = key_get_state(KEY_1);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_1);
            if (launch_off_x < 200) launch_off_x += (int16)launch_step * 5;    // 每次 +步长 (0.05m×5=0.01m单位), 封顶 +2.00
            menu_draw_offset_val_only(1, 1, 1, 1);
            launch_blink_last = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
        }
        k = key_get_state(KEY_4);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_4);
            if (launch_off_x > -200) launch_off_x -= (int16)launch_step * 5;   // 每次 -步长, 保底 -2.00
            menu_draw_offset_val_only(1, 1, 1, 1);
            launch_blink_last = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
        }
        k = key_get_state(KEY_2);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_2);
            launch_blink_last = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
            menu_state = STATE_INR_LAUNCH_OFFSET_Y;                       // x 定住, 进入设置 y
        }
        {
            uint8_t blink = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
            if (blink != launch_blink_last)
            {
                launch_blink_last = blink;
                if (blink) menu_draw_offset_val_only(1, 1, 1, 1);
                else       menu_draw_offset_val_only(1, 1, 1, 0);
            }
        }
        break;

    /* 请输入偏移坐标: 设置偏移y (x 已定住不再闪烁, y 闪烁; 键位同 x, 步长同 x) */
    case STATE_INR_LAUNCH_OFFSET_Y:
        k = key_get_state(KEY_1);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_1);
            if (launch_off_y < 200) launch_off_y += (int16)launch_step * 5;    // 每次 +步长, 封顶 +2.00
            menu_draw_offset_val_only(2, 0, 1, 1);
            launch_blink_last = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
        }
        k = key_get_state(KEY_4);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_4);
            if (launch_off_y > -200) launch_off_y -= (int16)launch_step * 5;   // 每次 -步长, 保底 -2.00
            menu_draw_offset_val_only(2, 0, 1, 1);
            launch_blink_last = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
        }
        k = key_get_state(KEY_2);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_2);
            Inav_UpdateMax();
            menu_save_settings();          // 写 W25Q64 Region2 (偏移xy)
            menu_state = STATE_INR_LAUNCH_DONE;                           // xy 都设好 → 完毕提示1s
            auto_back_tick = menu_tick_10ms;
        }
        {
            uint8_t blink = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
            if (blink != launch_blink_last)
            {
                launch_blink_last = blink;
                if (blink) menu_draw_offset_val_only(2, 0, 1, 1);
                else       menu_draw_offset_val_only(2, 0, 1, 0);
            }
        }
        break;

    /* 设置惯导速度限幅: 设置 x (KEY_1 +0.05m/s 封顶0.5, KEY_4 -0.05m/s 保底0, KEY_2 确认, x 闪烁) */
    case STATE_INR_SPD_LIMIT_X:
        k = key_get_state(KEY_1);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_1);
            if (spd_limit_x < 10) spd_limit_x++;                      // 每次 +0.05, 封顶 0.50
            menu_draw_spd_val_only(0, &spd_limit_x, 1, 1);
            spd_blink_last = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
        }
        k = key_get_state(KEY_4);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_4);
            if (spd_limit_x > 0) spd_limit_x--;                       // 每次 -0.05, 保底 0
            menu_draw_spd_val_only(0, &spd_limit_x, 1, 1);
            spd_blink_last = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
        }
        k = key_get_state(KEY_2);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_2);
            spd_blink_last = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
            menu_state = STATE_INR_SPD_LIMIT_Y;                       // x 定住, 进入设置 y
        }
        {
            uint8_t blink = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
            if (blink != spd_blink_last)
            {
                spd_blink_last = blink;
                if (blink) menu_draw_spd_val_only(0, &spd_limit_x, 1, 1);
                else       menu_draw_spd_val_only(0, &spd_limit_x, 1, 0);
            }
        }
        break;

    /* 设置惯导速度限幅: 设置 y (x 已定住不再闪烁, y 闪烁; 键位同 x) */
    case STATE_INR_SPD_LIMIT_Y:
        k = key_get_state(KEY_1);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_1);
            if (spd_limit_y < 10) spd_limit_y++;
            menu_draw_spd_val_only(1, &spd_limit_y, 1, 1);
            spd_blink_last = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
        }
        k = key_get_state(KEY_4);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_4);
            if (spd_limit_y > 0) spd_limit_y--;
            menu_draw_spd_val_only(1, &spd_limit_y, 1, 1);
            spd_blink_last = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
        }
        k = key_get_state(KEY_2);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_2);
            menu_save_settings();          // 写 W25Q64 Region2 (速度限幅 xy)
            menu_state = STATE_INR_SPD_LIMIT_DONE;                    // 限幅都设好 → 写入成功提示1s
            auto_back_tick = menu_tick_10ms;
        }
        {
            uint8_t blink = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
            if (blink != spd_blink_last)
            {
                spd_blink_last = blink;
                if (blink) menu_draw_spd_val_only(1, &spd_limit_y, 1, 1);
                else       menu_draw_spd_val_only(1, &spd_limit_y, 1, 0);
            }
        }
        break;

    /* 设置位置环速度限幅: 设置 x (KEY_1 +0.05m/s 封顶0.5, KEY_4 -0.05m/s 保底0, KEY_2 确认, x 闪烁) */
    case STATE_POS_LIMIT_X:
        k = key_get_state(KEY_1);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_1);
            if (pos_limit_x < 10) pos_limit_x++;                      // 每次 +0.05, 封顶 0.50
            menu_draw_spd_val_only(0, &pos_limit_x, 1, 1);
            pos_blink_last = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
        }
        k = key_get_state(KEY_4);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_4);
            if (pos_limit_x > 0) pos_limit_x--;                       // 每次 -0.05, 保底 0
            menu_draw_spd_val_only(0, &pos_limit_x, 1, 1);
            pos_blink_last = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
        }
        k = key_get_state(KEY_2);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_2);
            pos_blink_last = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
            menu_state = STATE_POS_LIMIT_Y;                           // x 定住, 进入设置 y
        }
        {
            uint8_t blink = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
            if (blink != pos_blink_last)
            {
                pos_blink_last = blink;
                if (blink) menu_draw_spd_val_only(0, &pos_limit_x, 1, 1);
                else       menu_draw_spd_val_only(0, &pos_limit_x, 1, 0);
            }
        }
        break;

    /* 设置位置环速度限幅: 设置 y (x 已定住不再闪烁, y 闪烁; 键位同 x) */
    case STATE_POS_LIMIT_Y:
        k = key_get_state(KEY_1);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_1);
            if (pos_limit_y < 10) pos_limit_y++;
            menu_draw_spd_val_only(1, &pos_limit_y, 1, 1);
            pos_blink_last = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
        }
        k = key_get_state(KEY_4);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_4);
            if (pos_limit_y > 0) pos_limit_y--;
            menu_draw_spd_val_only(1, &pos_limit_y, 1, 1);
            pos_blink_last = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
        }
        k = key_get_state(KEY_2);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_2);
            menu_save_settings();          // 写 W25Q64 Region2 (位置环速度限幅 xy)
            menu_state = STATE_POS_LIMIT_DONE;                    // 限幅都设好 → 写入成功提示1s
            auto_back_tick = menu_tick_10ms;
        }
        {
            uint8_t blink = ((menu_tick_10ms / MENU_SET_BLINK_TICKS) & 1);
            if (blink != pos_blink_last)
            {
                pos_blink_last = blink;
                if (blink) menu_draw_spd_val_only(1, &pos_limit_y, 1, 1);
                else       menu_draw_spd_val_only(1, &pos_limit_y, 1, 0);
            }
        }
        break;

    /* 无人机控制: 起桨 → 起飞 → 发车 → 比赛 (KEY_2/KEY_4 推进)
     * 2026-08-16: 恢复发送指令 (通信链路已修, CONTROL_SRC_DRONE=1 走 HC06 UART1) */
    case STATE_UAV_Control:
        k = key_get_state(KEY_2);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_2);
            HC06_SendDroneCmd(1);   // 起桨指令 → 'A' (cmd=1→'A'), 无人机起桨
            menu_state = STATE_UAV_TAKEOFF;    // 确认起桨 → 请起飞
        }
        break;

    case STATE_UAV_TAKEOFF:
        k = key_get_state(KEY_2);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_2);
            HC06_SendDroneCmd(2);   // 起飞/启动闭环指令 → 'B' (cmd=2→'B'), 无人机进入闭环
            menu_state = STATE_UAV_DEPART;     // 确认起飞 → 请发车
        }
        break;

    case STATE_UAV_DEPART:
        k = key_get_state(KEY_4);
        if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_4);
            if (Inav_Launch()) {
                HC06_SendDroneCmd(4);   // 通知无人机已发车 → 'D' (cmd=4→'D'), 此后小车持续发速度前馈
                menu_state = STATE_UAV_RACE;       // 武装成功 → 已发车/开始比赛
            }
            // 武装失败 (航点未录齐): 停留在本页, 串口已打印原因, 菜单无提示
        }
        break;

    default:
        /* 其余为纯显示/自动返回状态: STATE_Stop_All_Control / STATE_UAV_RACE /
         * STATE_INR_CLEAR_DONE / STATE_INR_SET_WP_DONE / STATE_INR_WP_DONE /
         * STATE_INR_LAUNCH_DISABLED / STATE_INR_LAUNCH_DONE, 无按键逻辑 */
        break;
    }

    /* 3. KEY_3: 从任意状态返回主界面 (主界面按 KEY_3 无效果) */
    k = key_get_state(KEY_3);
    if (k == KEY_SHORT_PRESS || k == KEY_LONG_PRESS)
    {
        key_clear_state(KEY_3);
        menu_state = STATE_MAIN;
    }

    /* 4. 自动返回: 提示页 1s 后自动跳回
     *    数据已清除/航点数设置完成/发车区已禁用/发车区设置完毕/所有航点记录完毕 → 惯导二级
     *    停止所有控制任务 → 主菜单 */
    if (menu_state == STATE_INR_CLEAR_DONE || menu_state == STATE_INR_SET_WP_DONE || menu_state == STATE_INR_WP_DONE || menu_state == STATE_Stop_All_Control
        || menu_state == STATE_INR_LAUNCH_DISABLED || menu_state == STATE_INR_LAUNCH_DONE || menu_state == STATE_INR_SPD_LIMIT_DONE || menu_state == STATE_POS_LIMIT_DONE)
    {
        if (menu_tick_10ms - auto_back_tick >= MENU_AUTO_BACK_TICKS)
        {
            if (menu_state == STATE_Stop_All_Control || menu_state == STATE_POS_LIMIT_DONE)
                menu_state = STATE_MAIN;                       // 停止所有控制 / 位置环限幅完成 → 主菜单
            else
                menu_state = STATE_Internal_Nav_Record;        // 含 STATE_INR_WP_DONE: 所有航点记录完毕也回二级
        }
    }
}
