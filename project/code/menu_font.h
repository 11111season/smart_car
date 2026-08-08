/*********************************************************************************************************************
* 文件名称          menu_font
* 功能描述          汉字点阵字库 (IPS200 菜单显示用)
*
* 取模软件: PCtoLCD2002
* 取模参数: 阴码 / 逐行式 / 顺向 / 16x16 / C51格式
* 每字字节: 32 (16 行 × 每行 2 字节), bit=1 画前景色
*
* 添加新字的步骤:
*   1. PCtoLCD2002 按上述参数取模, 得到 32 字节数据
*   2. menu_font.c 数组末尾追加一行 { 32字节 }
*   3. 本文件 MENU_CHN_* 索引枚举末尾追加一项 (放在 MENU_CHN_COUNT 之前)
*********************************************************************************************************************/

#ifndef __MENU_FONT_H_
#define __MENU_FONT_H_

#include "zf_common_headfile.h"

/*==================================================== 字库常量 ====================================================*/
#define MENU_CHN_W        16     // 字宽 (px)
#define MENU_CHN_H        16     // 字高 (px)
#define MENU_CHN_BYTES    32     // 单字字节数 = 16*16/8

/*==================================================== 汉字索引 ====================================================*/
// 顺序必须与 menu_font.c 数组完全一致
// 命名规则: 全拼小写; 同音字加数字后缀 (如 记=JI, 机=JI2)
typedef enum
{
    /* ---- 主菜单 ---- */
    MENU_CHN_MAI    = 0,    // 麦
    MENU_CHN_LUN    = 1,    // 轮
    MENU_CHN_XIAO   = 2,    // 小
    MENU_CHN_CHE    = 3,    // 车
    MENU_CHN_CAI    = 4,    // 菜
    MENU_CHN_DAN    = 5,    // 单

    /* ---- 航点记录 + 数字 ---- */
    MENU_CHN_DI     = 6,    // 第
    MENU_CHN_YI     = 7,    // 一
    MENU_CHN_GE     = 8,    // 个
    MENU_CHN_ER     = 9,    // 二
    MENU_CHN_SAN    = 10,   // 三
    MENU_CHN_SI     = 11,   // 四
    MENU_CHN_WU     = 12,   // 五
    MENU_CHN_LIU    = 13,   // 六
    MENU_CHN_QI     = 14,   // 七
    MENU_CHN_BA     = 15,   // 八
    MENU_CHN_JIU    = 16,   // 九
    MENU_CHN_JI     = 17,   // 记
    MENU_CHN_LU     = 18,   // 录
    MENU_CHN_HANG   = 19,   // 航
    MENU_CHN_DIAN   = 20,   // 点

    /* ---- 流程提示 ---- */
    MENU_CHN_KAI    = 21,   // 开
    MENU_CHN_SHI    = 22,   // 始
    MENU_CHN_JIE    = 23,   // 结
    MENU_CHN_SHU    = 24,   // 束

    /* ---- 惯导数据 ---- */
    MENU_CHN_GUAN   = 25,   // 惯
    MENU_CHN_DAO    = 26,   // 导
    MENU_CHN_QING   = 27,   // 清
    MENU_CHN_CHU    = 28,   // 除
    MENU_CHN_SHU2   = 29,   // 数
    MENU_CHN_JU     = 30,   // 据

    /* ---- 无人机 ---- */
    MENU_CHN_WU2    = 31,   // 无
    MENU_CHN_REN    = 32,   // 人
    MENU_CHN_JI2    = 33,   // 机
    MENU_CHN_QI2    = 34,   // 起
    MENU_CHN_FEI    = 35,   // 飞
    MENU_CHN_JIANG  = 36,   // 桨

    /* ---- 任务/控制 ---- */
    MENU_CHN_TING   = 37,   // 停
    MENU_CHN_ZHI    = 38,   // 止
    MENU_CHN_SUO    = 39,   // 所
    MENU_CHN_YOU    = 40,   // 有
    MENU_CHN_KONG   = 41,   // 控
    MENU_CHN_ZHI2   = 42,   // 制
    MENU_CHN_REN2   = 43,   // 任
    MENU_CHN_WU3    = 44,   // 务

    /* ---- 提示语 ---- */
    MENU_CHN_QING2  = 45,   // 请
    MENU_CHN_GUAN2  = 46,   // 关
    MENU_CHN_BI     = 47,   // 闭
    MENU_CHN_AN     = 48,   // 按
    MENU_CHN_XIA    = 49,   // 下
    MENU_CHN_JIAN   = 50,   // 键
    MENU_CHN_WAN    = 51,   // 完
    MENU_CHN_CHENG  = 52,   // 成
    MENU_CHN_FA     = 53,   // 发
    MENU_CHN_COMMA  = 54,   // ，(全角逗号)
    MENU_CHN_SHE    = 55,   // 设
    MENU_CHN_ZHI3   = 56,   // 置

    /* ---- 2026-08-08 补全二级菜单缺字 ---- */
    MENU_CHN_JIANG2 = 57,   // 降
    MENU_CHN_LUO    = 58,   // 落
    MENU_CHN_DIAN2  = 59,   // 电
    MENU_CHN_QI3    = 60,   // 启
    MENU_CHN_DONG   = 61,   // 动
    MENU_CHN_DU     = 62,   // 读
    MENU_CHN_QU     = 63,   // 取
    MENU_CHN_XIE    = 64,   // 写
    MENU_CHN_RU     = 65,   // 入
    MENU_CHN_SUO2   = 66,   // 索
    MENU_CHN_YIN    = 67,   // 引

    /* ---- 2026-08-08 多级菜单缺字 (字形已由用户取模填入 menu_font.c 68-77) ---- */
    MENU_CHN_YI2    = 68,   // 已 (一=YI 已占, 同音字后缀2)
    MENU_CHN_ZAI    = 69,   // 再
    MENU_CHN_CI     = 70,   // 次
    MENU_CHN_BI2    = 71,   // 比 (闭=BI 已占, 同音字后缀2)
    MENU_CHN_SAI    = 72,   // 赛
    MENU_CHN_LI     = 73,   // 历
    MENU_CHN_SHI2   = 74,   // 史 (始=SHI 已占, 同音字后缀2)
    MENU_CHN_DENG   = 75,   // 等
    MENU_CHN_DAI    = 76,   // 待
    MENU_CHN_BI3    = 77,   // 毕 (比=BI2 已占, 同音字后缀3)
    MENU_CHN_LIANG  = 78,   // 量 (设置航点数量/数量设置完成用; 字形已由用户取模填入 menu_font.c 78)

    /* ---- 2026-08-08 发车区航点设置缺字 (字形已由用户取模填入 menu_font.c 79-89) ---- */
    MENU_CHN_QU2    = 79,   // 区 (取=QU 已占, 同音字后缀2)
    MENU_CHN_SHI3   = 80,   // 是 (始=SHI 已占, 史=SHI2 已占, 同音字后缀3)
    MENU_CHN_FOU    = 81,   // 否
    MENU_CHN_YONG   = 82,   // 用
    MENU_CHN_JIN    = 83,   // 禁
    MENU_CHN_SHU3   = 84,   // 输 (束=SHU 已占, 数=SHU2 已占, 同音字后缀3)
    MENU_CHN_PIAN   = 85,   // 偏
    MENU_CHN_YI3    = 86,   // 移 (一=YI 已占, 已=YI2 已占, 同音字后缀3)
    MENU_CHN_ZUO    = 87,   // 坐
    MENU_CHN_BIAO   = 88,   // 标
    MENU_CHN_MI     = 89,   // 米

    MENU_CHN_COUNT,          // 字库总字数 (自动递增, 勿手动改)
} menu_chn_idx_enum;

/*==================================================== 字库数据 ====================================================*/
// 按索引连续存放: menu_font[0]=麦 ... menu_font[COUNT-1]=，(全角逗号)
// 数组连续, 因此相邻索引可一次显示多个字
extern const uint8 menu_font[MENU_CHN_COUNT][MENU_CHN_BYTES];

/*==================================================== 汉字文本结构 ====================================================*/
// 一段汉字文本 = 一串字库索引 (顺序任意, 不要求字库连续)
// 例: "无人机控制" = { 无(31), 人(32), 机(33), 控(41), 制(42) }
typedef struct
{
    const menu_chn_idx_enum *chars;   // 字库索引数组 (指向 menu_font 下标)
    uint8_t                  num;     // 字数
} menu_text_t;

/*==================================================== 接口函数 ====================================================*/
// 从 start_idx 起连续显示 num 个汉字 (总宽 = num*16 px)
void menu_show_chinese(uint16 x, uint16 y, menu_chn_idx_enum start_idx, uint8 num, uint16 color);

// 按字库索引数组显示一段汉字 (每个字 x 坐标 = x + i*16), 支持不连续词组
void menu_show_text(uint16 x, uint16 y, const menu_text_t *text, uint16 color);

#endif
