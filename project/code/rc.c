#include "zf_common_headfile.h"

void RC_data_get(float dt)
{

    if (lora3a22_state_flag == 1 )
    {       
        if( lora3a22_finsh_flag == 1)
        {
            //左边摇杆左右值
            rc.yaw = -lora3a22_uart_transfer.joystick[0];//1961--0--（-2034）
            //左边摇杆上下值
            rc.thr = lora3a22_uart_transfer.joystick[1];//1982--0--（-2013）
            //右边摇杆左右值
            rc.roll = -lora3a22_uart_transfer.joystick[2];//2000--0--（-1994）
            //右边摇杆上下值
            rc.pitch = lora3a22_uart_transfer.joystick[3];//1946--0--（-2048）

            //左边摇杆按键
            rc.key1 = lora3a22_uart_transfer.key[2];
            //右边摇杆按键
            rc.key2 = lora3a22_uart_transfer.key[3];
            // 左边侧向按键
            rc.key3 = lora3a22_uart_transfer.key[0];
            // 右边侧向按键
            rc.key4 = lora3a22_uart_transfer.key[1];

            // 左边拨码开关_1
            rc.aux1 = lora3a22_uart_transfer.switch_key[1];
            // 左边拨码开关_2
            rc.aux2 = lora3a22_uart_transfer.switch_key[0];
            // 右边拨码开关_1
            rc.aux3 = lora3a22_uart_transfer.switch_key[3];
            // 右边拨码开关_2
            rc.aux4 = lora3a22_uart_transfer.switch_key[2];

              
            flag.hover_lock= 0;
            lora3a22_finsh_flag = 0;
        } 
    }
    else 
    {
        flight_state = STATE_LAND;    
    }
}

void RC_data_deal(float dt)
{
    //解锁
    static uint16_t unlock_cnt = 0;
    if( rc.thr < -1650 && rc.yaw < -1700 && rc.pitch < -1600 && rc.roll > 1700)
    {
        unlock_cnt++;

        if(unlock_cnt > 100)
        {
            rc.unlock_cmd = 1;
            unlock_cnt = 0;
        }
    }
    else
    {
        unlock_cnt = 0;
    }

    //上锁
    static uint16_t lock_cnt = 0;
    if( rc.thr < -1650 && rc.yaw > 1650 && rc.pitch < -1700 && rc.roll < -1700)
    {
        lock_cnt++;

        if(lock_cnt > 200)
        {
            rc.lock_cmd = 1;
            lock_cnt = 0;
        }
    }
    else
    {
        lock_cnt = 0;
    }
    

    
    
    // AUX1:
    static uint8 last_aux1;
    if(rc.aux1 == 1)
    {
        rc.land_cmd = 1;
    }
    else
    {
        rc.land_cmd = 0;
    }
    last_aux1 = rc.aux1;


    // AUX2:
    // 一键起飞
    static uint8 last_aux2;
    if( last_aux2 == 1)
    {
        rc.takeoff_cmd  = 1;
    }
    else
    {
        rc.takeoff_cmd = 0;
    }
    last_aux2 = rc.aux2;
    
    // AUX3
    static uint8 last_aux3;
    if(rc.aux3 == 1)
    {
        rc.task_cmd = 1;
    }
    else
    {
        rc.task_cmd = 0;
    }
    last_aux3 = rc.aux3;

    // AUX4:
    static uint8 last_aux4;
    if(rc.aux4 == 1)
    {
        rc.emergency_cmd = 1;
    }
    else
    {
        rc.emergency_cmd = 0;
    }
    last_aux4 = rc.aux4;

}

/*
保持秒数
调用周期
通道值

    rc.unlock_cmd = 0;
    rc.lock_cmd = 0;
上电清零

*/
//解锁
void FC_unlock(void)
{

}

//上锁
void FC_lock(void)
{
    
}

//










//定高
//定点
//程控
//一键返航
/*摇杆数据处理
  加死区防止中位抖动
*/

//失控准备上锁保护
/*
死区
限幅
中值校准
归一化

*/






























/*
                  printf (" rc.yaw = %d\r\n",rc.yaw);
                //左边摇杆上下值
                printf (" rc.thr  = %d\r\n",rc.thr);
                //右边摇杆左右值
                printf (" rc.roll =  %d\r\n",rc.roll);
                //右边摇杆上下值
                printf ("rc.pitch = %d\r\n",rc.pitch);

                //左边摇杆按键
                printf ("key1 = %d\r\n",rc.key1);
                //右边摇杆按键
                printf ("key2 = %d\r\n",rc.key2);
                // 左边侧向按键
                printf ("key3 = %d\r\n",rc.key3);
                // 右边侧向按键
                printf ("key4 = %d\r\n",rc.key4);

                // 左边拨码开关_1
                printf ("rc.aux1 = %d\r\n",rc.aux1);
                // 左边拨码开关_2
                printf ("rc.aux2 = %d\r\n",rc.aux2);
                // 右边拨码开关_1
                printf ("rc.aux3 = %d\r\n",rc.aux3);
                // 右边拨码开关_2
                printf ("rc.aux4 = %d\r\n",rc.aux4);*/
               