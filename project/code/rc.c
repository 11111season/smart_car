#include "zf_common_headfile.h"

void RC_data_get(float dt)
{

    if (lora3a22_state_flag == 1 )
    {       
        if( lora3a22_finsh_flag == 1)
        {
            //���ҡ������ֵ
            rc.yaw = -lora3a22_uart_transfer.joystick[0];//1961--0--��-2034��
            //���ҡ������ֵ
            rc.thr = lora3a22_uart_transfer.joystick[1];//1982--0--��-2013��
            //�ұ�ҡ������ֵ
            rc.roll = -lora3a22_uart_transfer.joystick[2];//2000--0--��-1994��
            //�ұ�ҡ������ֵ
            rc.pitch = lora3a22_uart_transfer.joystick[3];//1946--0--��-2048��

            //���ҡ�˰���
            rc.key1 = lora3a22_uart_transfer.key[2];
            //�ұ�ҡ�˰���
            rc.key2 = lora3a22_uart_transfer.key[3];
            // ��߲��򰴼�
            rc.key3 = lora3a22_uart_transfer.key[0];
            // �ұ߲��򰴼�
            rc.key4 = lora3a22_uart_transfer.key[1];

            // ��߲��뿪��_1
            rc.aux1 = lora3a22_uart_transfer.switch_key[1];
            // ��߲��뿪��_2
            rc.aux2 = lora3a22_uart_transfer.switch_key[0];
            // �ұ߲��뿪��_1
            rc.aux3 = lora3a22_uart_transfer.switch_key[3];
            // �ұ߲��뿪��_2
            rc.aux4 = lora3a22_uart_transfer.switch_key[2];

              
            flag.hover_lock= 0;
            lora3a22_finsh_flag = 0;
        } 
    }
    else 
    {
        flight_state = STATE_LOCK;    
    }
}

void RC_data_deal(float dt)
{
    //����
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

    //����
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
    // һ�����
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
��������
��������
ͨ��ֵ

    rc.unlock_cmd = 0;
    rc.lock_cmd = 0;
�ϵ�����

*/
//����
void FC_unlock(void)
{

}

//����
void FC_lock(void)
{
    
}

//










//����
//����
//�̿�
//һ������
/*ҡ�����ݴ���
  ��������ֹ��λ����
*/

//ʧ��׼����������
/*
����
�޷�
��ֵУ׼
��һ��

*/






























/*
                  printf (" rc.yaw = %d\r\n",rc.yaw);
                //���ҡ������ֵ
                printf (" rc.thr  = %d\r\n",rc.thr);
                //�ұ�ҡ������ֵ
                printf (" rc.roll =  %d\r\n",rc.roll);
                //�ұ�ҡ������ֵ
                printf ("rc.pitch = %d\r\n",rc.pitch);

                //���ҡ�˰���
                printf ("key1 = %d\r\n",rc.key1);
                //�ұ�ҡ�˰���
                printf ("key2 = %d\r\n",rc.key2);
                // ��߲��򰴼�
                printf ("key3 = %d\r\n",rc.key3);
                // �ұ߲��򰴼�
                printf ("key4 = %d\r\n",rc.key4);

                // ��߲��뿪��_1
                printf ("rc.aux1 = %d\r\n",rc.aux1);
                // ��߲��뿪��_2
                printf ("rc.aux2 = %d\r\n",rc.aux2);
                // �ұ߲��뿪��_1
                printf ("rc.aux3 = %d\r\n",rc.aux3);
                // �ұ߲��뿪��_2
                printf ("rc.aux4 = %d\r\n",rc.aux4);*/
               