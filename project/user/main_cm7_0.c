/*********************************************************************************************************************
* CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main_cm7_0
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************

//#define PWM_CH1             (TCPWM_CH30_P10_2)                                 // PWM输出端口

  int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_init();                       // 调试串口信息初始化
    // 此处编写用户代码 例如外设初始化代码等
    
     ALL_Init();


     //try
         
//     pmw3901_init();//光流初始化



    while(true)
    {
        // 此处编写需要循环执行的代码
        
      111111111111111
//        ips200_show_string( 16*0,  16*4, "pitch=:");     ips200_show_float(16*6,  16*4, imu660rc_pitch,  3,5);
//        ips200_show_string( 16*0,  16*5, "roll=:" );     ips200_show_float(16*6,  16*5, imu660rc_roll,   3,5);
//        ips200_show_string( 16*0,  16*6, "yaw11111=:"  );     ips200_show_float(16*6,  16*6, imu660rc_yaw, 3,5);
//      
//            motor_set(1,600);
//            motor_set(2,650);
//            motor_set(3,650);
//            motor_set(4,650);

      
      
      
      
      
//-------------------printf串口打印------------      
//        printf("111\r\n");
//         // 调试：打印IMU原始数据
//        printf("gyro_x=%f, gyro_y=%f, gyro_z=%f\r\n", 
//               imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z);
//        printf("roll=%f, pitch=%f, yaw=%f\r\n",
//               eulerAngle.roll, eulerAngle.pitch, eulerAngle.yaw);
//        
//        // 调试：打印PID中间值
//        printf("Roll.out=%f, Pitch.out=%f, Yaw.out=%f\r\n",
//               PIDRoll.out, PIDPitch.out, PIDYaw.out);
//        printf("RateX.out=%f, RateY.out=%f, RateZ.out=%f\r\n",
//               PIDRateX.out, PIDRateY.out, PIDRateZ.out);
//        
//        // 调试：打印最终电机值
//        printf("m1=%d, m2=%d, m3=%d, m4=%d\r\n", m1, m2, m3, m4);
      
      
      
//--------------------vofa------------
        //欧拉角
        printf("%5f, %5f, %5f\r\n", imu660rc_roll, imu660rc_pitch,  imu660rc_yaw);
        // 调试：打印IMU原始数据
        printf("%5f, %5f, %5f\r\n", imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z);
        // 调试：打印PID中间值
        printf("%5f, %5f, %5f\r\n",PIDRoll.out, PIDPitch.out, PIDYaw.out);
        printf("%5f, %5f, %5f\r\n",PIDRateX.out, PIDRateY.out, PIDRateZ.out);
        
        
        
        
        
//        printf("\r\nimu660rb acc data:  x=%5f, y=%5f, z=%5f\r\n", imu660rc_roll, imu660rc_pitch,  imu660rc_yaw);
        
        
        
        
        
//--------------------IPS----------
        ips200_show_string( 16*0,  16*4, "roll=:");     ips200_show_float(16*6,  16*4, eulerAngle.roll,  3,5);
        ips200_show_string( 16*0,  16*5, "pitch=:" );     ips200_show_float(16*6,  16*5, eulerAngle.pitch,   3,5);
        ips200_show_string( 16*0,  16*6, "yaw11111=:"  );     ips200_show_float(16*6,  16*6, eulerAngle.yaw, 3,5);
        
        ips200_show_string( 16*0,  16*7, "m1=:");     ips200_show_float(16*6,  16*7, m1, 3,5);
        ips200_show_string( 16*0,  16*8, "m2=:");     ips200_show_float(16*6,  16*8, m2, 3,5);
        ips200_show_string( 16*0,  16*9, "m3=:");     ips200_show_float(16*6,  16*9, m3, 3,5);
        ips200_show_string( 16*0,  16*10, "m4=:");    ips200_show_float(16*6,  16*10, m4, 3,5);

        
        
        
        
        // 此处编写需要循环执行的代码
    }
}

// **************************** 代码区域 ****************************
