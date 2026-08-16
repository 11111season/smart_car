/*********************************************************************************************************************
* CYT4BB Opensourec Library 即 CYT4BB 开源库, 一个位于官方 SDK 接口上的单片机开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 该文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是自由软件
* 你可以根据自由软件基金会发布的 GPL(GNU General Public License, GNU通用公共许可证)
* 的 GPL 的第3版(或 GPL3.0), 或任何你选择的更高版本, 来重新分发或修改它
*
* 这个库的分发是希望它能有用, 但不提供任何保证
* 甚至没有任何适销性或适用于特定用途的保证
* 更详细的说明请参见 GPL
*
* 你应该已经收到了这个库的副本, 和一份 GPL 的副本
* 如果没有, 请访问 <https://www.gnu.org/licenses/>
*
* 版权注意:
* 这个库使用了 GPL3.0 开源协议, 所有产品化均为非开源版本
* 如需商用英译见 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件
* 版权见 libraries 文件夹内 各文件夹下的 LICENSE 文件
* 欢迎各位使用并反馈问题, 修改或发布时请保留逐飞科技的相关版权信息
*
* 文件名称          cm7_0_isr
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹中的 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 技术支持          https://seekfree.taobao.com/
*
* 修改记录
* 日期             作者               备注
* 2024-1-9      pudding            first version
* 2024-5-14     pudding            新增12个pit定时器中断 添加注释说明
* 2025-2-4      pudding            优化定时器中断逻辑, 防止长时间调用导致的错误问题, 优化在线采集初始化逻辑
* 2025-2-4      pudding            新增在线调试接口
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "Encoder.h"
#include "My_imu660ra.h"
#include "HC06_Driver.h"
#include "inertial_nav.h"   // 暂时注释
#include "QMC5883L.h"       // 磁力计椭圆校准采集
#include "key_task.h"       // MAG_CALIB_MODE / MAG_CALIB_MOTOR_TEST
#include "App_lora3a22.h"   // CONTROL_SRC_DRONE (ISR 里按控制源切 UART_1 接收处理, 2026-08-15)
volatile int count = 0;
volatile uint64_t time_us = 0;
volatile uint16_t menu_tick_10ms = 0;   // 菜单显示节拍器 (10ms, pit0_ch10_isr 维护)
volatile int Start_Pid_Flag = 0;
volatile int Stop_Pid_Flag = 0;
volatile uint8_t send_uart_cmd_flag = 0;
volatile uint8_t send_uart_cmd_value = 0;
volatile uint8_t send_vel_flag = 0;        // 主循环消费: 发送目标速度给无人机

// ===== 动态零偏学习参数 =====
// 无人机/位置环从不发转向指令 (PositionControl_Update 只写 vx/vy, 自主模式 angle_target 恒0),
// 真实转向只来自遥控右杆(ω大、angle_target大)或压信标瞬态(ω尖峰).
// 故自主模式下持续的小 ω_target = 零漂, 反减回 gyro_z_offset, 收敛从原~28s 提到~1s.
#define DRIFT_LEARN_K            0.01f   // 学习增益: 收敛τ≈1s (可调0.005~0.02)
#define DRIFT_LEARN_OMEGA_GATE   3.0f    // 指令角速度门限(°/s): <3°视为零漂, 真实转舵5~30°
#define DRIFT_LEARN_ANGLE_GATE   5.0f    // 目标角度门限(°): 仅|angle_target|<5°(自主模式)才学, 防吃掉手动锁角

// ===== 零漂学习总开关 =====
// 0 = 架构重构 (2026-08-09): 零漂学习自指回路已废弃 (悬空三跑证明参数不可调)
//     改为磁力计作为实时绝对基准, 地图/控制环/记录统一信任 fused_yaw
//     本开关禁用: comp累加 + 0.00003转移 + 动态学习器 (全部 ISR 侧零漂)
//     保留: IMU 静态学习 (My_imu660ra.c, 静止时校准零偏, 标准做法)
#define DRIFT_LEARN_ENABLE   0

// **************************** PIT中断函数 ****************************
void pit0_ch0_isr()
{
    pit_isr_flag_clear(PIT_CH0);
    count++;

    Encoder_Data_Get();
    My_Imu660ra_Update();
    MagYaw_Update();                                    // 磁力计融合 (2026-08-09 架构: 实时绝对基准, 常开)
#if MAG_CALIB_MODE && MAG_CALIB_MOTOR_TEST
    qmc5883l_motor_test_collect();                      // 电机磁场干扰测试 (KEY4触发, PWM缓变)
#elif MAG_CALIB_MODE
    qmc5883l_calibration_collect();                     // 椭圆校准采集 (KEY4控制, 记录期每10ms打印)
#endif
    InertialNav_PosUpdate();

    PositionControl_Update();

    // 优先级: 标志位1(信标) > 巡逻/去中心
    // 当无人机发现信标(flag=1)时, 立即打断任何导航, 转向信标
    if (mission_armed && drone_beacon_flag == 1) {
        go_center = 0;
        bcn_nav_on = 0;
        patrol_active = 0;    // 清除巡逻, 下次flag=2时重新初始化
    }

    if (bcn_nav_on) {
        target_vx = bcn_nav_vx;
        target_vy = bcn_nav_vy;
        angle_target = bcn_nav_angle;
    }

    // ===== 完赛逻辑 (最高优先级): race_done=1 → 锁住当前位置 =====
    static uint8_t race_locked = 0;

    if (race_done && !race_locked) {
        race_locked = 1;
        bcn_nav_on = 0;
        patrol_active = 0;
        printf("RACE: done, locked!\n");
    }

    if (race_locked) {
        target_vx = 0.0f; target_vy = 0.0f;
        bcn_nav_on = 0;
        patrol_active = 0;
    }

    if (angle_pid_yaw.Enable && !race_locked) {
        // ---- 航向反馈: 磁力计融合航向 (2026-08-09 架构: 陀螺仪信任磁力计) ----
        // fused_yaw = 陀螺积分 + 磁力计实时绝对修正 (恒定权重0.02, 无运动门控)
        // 替代零漂补偿 GetYawComp: 磁力计提供绝对基准, 控制环不再依赖零漂学习
#if DRIFT_LEARN_ENABLE
        // (保留旧零漂代码路径, 置0时不编译)
        yaw_drift_comp -= angle_pid_yaw.err_int_k_1 * 0.0005f;
#endif

        float yaw = fused_yaw;

        // 角度环绕: 把PID目标平移到当前yaw的±180内, 使误差走最短路径
        // (否则 target=180° yaw=-180° 时 PID 误算330°误差, 舍近求远)
        float target_eff = angle_target;
        while (target_eff - yaw >  180.0f) target_eff -= 360.0f;
        while (target_eff - yaw < -180.0f) target_eff += 360.0f;
        PID_SetTarget(&angle_pid_yaw, target_eff);
        float omega_target = PID_Calculate(&angle_pid_yaw, yaw);

        // 积分已部分转移至漂移补偿, 缓慢衰减避免双重计数
        angle_pid_yaw.err_int_k_1 *= 0.999f;

#if DRIFT_LEARN_ENABLE
        // PID积分 → 陀螺零偏: 极慢 (~50x慢于yaw_drift_comp), 传感器层面修正
        // 当积分持续偏向一侧, 说明陀螺存在零偏, 缓慢转移到gyro_z_offset (无门限兜底)
        gyro_z_offset -= angle_pid_yaw.err_int_k_1 * 0.00003f;

        // ---- 动态零偏学习 (门限): 小角度指令=零漂, 反减回陀螺零偏 ----
        // 稳态 ω_target ≈ (Z−B) 即残余零偏; 真实转向(右杆/压信标)ω大且angle_target大 → 门限关闭不学
        // 收敛从原~28s 提到~1s, 追得上温度/电压引起的零偏漂移 (原0.00003保留作门限关闭时的兜底)
        if (fabsf(angle_target) < DRIFT_LEARN_ANGLE_GATE && fabsf(omega_target) < DRIFT_LEARN_OMEGA_GATE) {
            gyro_z_offset -= DRIFT_LEARN_K * omega_target;
        }
#endif
        if (gyro_z_offset >  2.0f) gyro_z_offset =  2.0f;
        if (gyro_z_offset < -2.0f) gyro_z_offset = -2.0f;

        // 内环: 角速度闭环 (陀螺低通反馈)
        PID_SetTarget(&angle_pid_gyro, omega_target);
        static float gz_filt = 0.0f;
        gz_filt = 0.8f * gz_filt + 0.2f * imu660_gz;
        float omega_des = PID_Calculate(&angle_pid_gyro, gz_filt);

        float omega_rad = omega_des * (PI / 180.0f);
        Mecanum_Move(target_vx, target_vy, omega_rad);
    } else {
        Mecanum_Move(target_vx, target_vy, 0.0f);
    }

    //AntiSlipControl();
    Motor_PID_Control_All();

    // 每10ms置标志位, 主循环消费发送目标速度给无人机前馈
    if (mission_armed) send_vel_flag = 1;
}

void pit0_ch1_isr()                     // 定时器通道 1 的中断处理函数
{
    pit_isr_flag_clear(PIT_CH1);
    key_scanner();
//    if(key_get_state(KEY_1)== KEY_SHORT_PRESS)//按键短按中间按P21.5
//    {
//          send_uart_cmd_flag = 1;
//          send_uart_cmd_value = 1;
//          printf("send_uart_cmd_value:%d\n", send_uart_cmd_value);
//        if(imu660ra_ready)
//        {
//            Start_Pid_Flag = 1;
//            Stop_Pid_Flag = 0;  // 清停止标志
//        }
//    }
//    if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
//    {
//          send_uart_cmd_flag = 1;
//          send_uart_cmd_value = 2;
//          printf("send_uart_cmd_value:%d\n", send_uart_cmd_value);
//    }
//
//    if(key_get_state(KEY_3) == KEY_SHORT_PRESS)//按键短按停止
//    {
//        send_uart_cmd_flag = 1;
//        send_uart_cmd_value = 3;
//        printf("send_uart_cmd_value:%d\n", send_uart_cmd_value);
//        Start_Pid_Flag = 0;
//        Stop_Pid_Flag = 1;
//    }
}

void pit0_ch2_isr()                     // 定时器通道 2 的中断处理函数
{
    pit_isr_flag_clear(PIT_CH2);
    time_us++;

}

void pit0_ch10_isr()                    // 定时器通道 10 的中断处理函数 (菜单显示节拍器 10ms)
{
    pit_isr_flag_clear(PIT_CH10);
    menu_tick_10ms++;
}

void pit0_ch11_isr()                    // 定时器通道 11 的中断处理函数
{
    pit_isr_flag_clear(PIT_CH11);

}

void pit0_ch12_isr()                    // 定时器通道 12 的中断处理函数
{
    pit_isr_flag_clear(PIT_CH12);

}

void pit0_ch13_isr()                    // 定时器通道 13 的中断处理函数
{
    pit_isr_flag_clear(PIT_CH13);

}

void pit0_ch14_isr()                    // 定时器通道 14 的中断处理函数
{
    pit_isr_flag_clear(PIT_CH14);

}

void pit0_ch15_isr()                    // 定时器通道 15 的中断处理函数
{
    pit_isr_flag_clear(PIT_CH15);

}

void pit0_ch16_isr()                    // 定时器通道 16 的中断处理函数
{
    pit_isr_flag_clear(PIT_CH16);

}

void pit0_ch17_isr()                    // 定时器通道 17 的中断处理函数
{
    pit_isr_flag_clear(PIT_CH17);

}

void pit0_ch18_isr()                    // 定时器通道 18 的中断处理函数
{
    pit_isr_flag_clear(PIT_CH18);

}

void pit0_ch19_isr()                    // 定时器通道 19 的中断处理函数
{
    pit_isr_flag_clear(PIT_CH19);

}

void pit0_ch20_isr()                    // 定时器通道 20 的中断处理函数
{
    pit_isr_flag_clear(PIT_CH20);

}

void pit0_ch21_isr()                    // 定时器通道 21 的中断处理函数
{
    pit_isr_flag_clear(PIT_CH21);
    tsl1401_collect_pit_handler();
}
// **************************** PIT中断函数 ****************************


// **************************** 串口中断函数 ****************************
// 串口0默认为调试串口
void uart0_isr (void)
{
    if(uart_isr_mask(UART_0))            // 串口0接收中断
    {

#if DEBUG_UART_USE_INTERRUPT             // 使能调试串口接收中断
        debug_interrupr_handler();       // 处理 debug 串口接收的数据, 数据会被 debug 缓存后被获取
#endif                                   // 如果想修改 DEBUG_UART_INDEX 或其他位置请放到对应的串口中断里去

    }
    else                                 // 串口0发送中断
    {



    }
}

void uart1_isr (void)
{
    if(uart_isr_mask(UART_1))            // 串口1接收中断
    {
        // 2026-08-15 修复: 之前 HC06_UART_RX_Handler 被注释 → 视觉/无人机帧字节没人读, 收不到任何数据
        // 按控制源切换: 无人机/视觉走 HC06 (#x,y,flag$), LoRa遥控模拟走无线模块统一回调 (18字节结构化帧)
#if CONTROL_SRC_DRONE
        // 2026-08-15 修复4: 恢复单次调用 (两个 do-while 版本都有问题:
        //  a) uart_isr_mask() 恒返回1(查使能掩码而非数据) → 无条件死循环
        //  b) 按FIFO数量循环 → 循环体不搬新字节, 软件缓冲消费完但FIFO还有 → 死循环)
        // SCB UART RX_NOT_EMPTY 是电平中断: FIFO 非空持续请求, 每次中断搬+消费1字节即可, 无需取空
        HC06_UART_RX_Handler();      // 无人机/视觉: 读 UART_1 字节进 FIFO → HC06_Task 解析
#else
        wireless_module_uart_handler();  // LoRa遥控模拟: 无线模块统一回调函数 (当前挂 lora3a22_uart_callback)
#endif

    }
    else                                // 串口1发送中断
    {



    }
}

void uart2_isr (void)
{
    if(uart_isr_mask(UART_2))            // 串口2接收中断
    {

        gnss_uart_callback();            // GPS模块回调函数

    }
    else                                // 串口2发送中断
    {



    }
}

void uart3_isr (void)
{
    if(uart_isr_mask(UART_3))            // 串口3接收中断
    {



    }
    else                                // 串口3发送中断
    {



    }
}

void uart4_isr (void)
{
    if(uart_isr_mask(UART_4))            // 串口4接收中断
    {

        uart_receiver_handler();                                                                // 串口接收回调函数

    }
    else                                // 串口4发送中断
    {



    }
}

void uart5_isr (void)
{
    if(uart_isr_mask(UART_5))            // 串口5接收中断
    {



    }
    else                                // 串口5发送中断
    {



    }
}

void uart6_isr (void)
{
    if(uart_isr_mask(UART_6))            // 串口6接收中断
    {



    }
    else                                // 串口6发送中断
    {



    }
}
// **************************** 串口中断函数 ****************************

// **************************** 外部中断函数 ****************************
void gpio_0_exti_isr()                  // 外部 GPIO_0 中断处理函数
{



}

void gpio_1_exti_isr()                  // 外部 GPIO_1 中断处理函数
{
    if(exti_flag_get(P01_0))		// 示例P1_0端口外部中断判断
    {




    }
    if(exti_flag_get(P01_1))
    {


    }
}

void gpio_2_exti_isr()                  // 外部 GPIO_2 中断处理函数
{
    if(exti_flag_get(P02_0))
    {


    }
    if(exti_flag_get(P02_4))
    {


    }

}

void gpio_3_exti_isr()                  // 外部 GPIO_3 中断处理函数
{



}

void gpio_4_exti_isr()                  // 外部 GPIO_4 中断处理函数
{



}

void gpio_5_exti_isr()                  // 外部 GPIO_5 中断处理函数
{



}


void gpio_6_exti_isr()                  // 外部 GPIO_6 中断处理函数
{



}

void gpio_7_exti_isr()                  // 外部 GPIO_7 中断处理函数
{



}

void gpio_8_exti_isr()                  // 外部 GPIO_8 中断处理函数
{



}

void gpio_9_exti_isr()                  // 外部 GPIO_9 中断处理函数
{



}

void gpio_10_exti_isr()                  // 外部 GPIO_10 中断处理函数
{



}

void gpio_11_exti_isr()                  // 外部 GPIO_11 中断处理函数
{



}

void gpio_12_exti_isr()                  // 外部 GPIO_12 中断处理函数
{



}

void gpio_13_exti_isr()                  // 外部 GPIO_13 中断处理函数
{



}

void gpio_14_exti_isr()                  // 外部 GPIO_14 中断处理函数
{



}

void gpio_15_exti_isr()                  // 外部 GPIO_15 中断处理函数
{



}

void gpio_16_exti_isr()                  // 外部 GPIO_16 中断处理函数
{



}

void gpio_17_exti_isr()                  // 外部 GPIO_17 中断处理函数
{



}

void gpio_18_exti_isr()                  // 外部 GPIO_18 中断处理函数
{



}

void gpio_19_exti_isr()                  // 外部 GPIO_19 中断处理函数
{



}

void gpio_20_exti_isr()                  // 外部 GPIO_20 中断处理函数
{



}

void gpio_21_exti_isr()                  // 外部 GPIO_21 中断处理函数
{



}

void gpio_22_exti_isr()                  // 外部 GPIO_22 中断处理函数
{



}

void gpio_23_exti_isr()                  // 外部 GPIO_23 中断处理函数
{



}
// **************************** 外部中断函数 ****************************
