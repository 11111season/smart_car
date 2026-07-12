/*********************************************************************************************************************
* CYT4BB Opensourec Library
* Copyright (c) 2022 SEEKFREE
*
* 文件名称          main_cm7_0
* 模式说明          正常业务模式
*                   KEY_1: 缓启动  KEY_2: 发送PID指令  KEY_3: 停止
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "init.h"
#include "key_task.h"
#include "HC06_Driver.h"
#include "inertial_nav.h"
#include "test.h"

// **************************** 代码区域 ****************************

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    debug_init();
    Init_all();
    KeyTask_Init();

    while(true)
    {
        KeyTask_Handler();
        InertialNav_Update();
        HC06_Task();
//        printf("11111\r\n");

     //   printf("x=%.3f y=%.3f s=%.3f \r\n",target_vx, target_vy, ready_s);


    }
}
