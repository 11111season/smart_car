/*********************************************************************************************************************
* CYT4BB Opensourec Library ���� CYT4BB ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� CYT4BB ��Դ���һ����
*
* CYT4BB ��Դ�� ���������
* �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          zf_device_dl1b
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 9.40.1
* ����ƽ̨          CYT4BB
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2024-01-12       pudding           first version
********************************************************************************************************************/
/*********************************************************************************************************************
* ���߶��壺
*                   ------------------------------------
*                   ģ��ܽ�            ��Ƭ���ܽ�
*                   SCL                 �鿴 zf_device_dl1b.h �� DL1B_SCL_PIN  �궨��
*                   SDA                 �鿴 zf_device_dl1b.h �� DL1B_SDA_PIN  �궨��
*                   XS                  �鿴 zf_device_dl1b.h �� DL1B_XS_PIN  �궨��
*                   VCC                 5V ��Դ
*                   GND                 ��Դ��
*                   ------------------------------------
********************************************************************************************************************/

#include "zf_common_debug.h"

#include "zf_driver_delay.h"
#include "zf_driver_exti.h"
#include "zf_driver_soft_iic.h"

#include "zf_device_dl1b.h"
#include "zf_device_config.h"
#include "zf_device_type.h"

uint8 dl1b_init_flag = 0;
uint8 dl1b_finsh_flag = 0;
uint16 dl1b_distance_mm = 8192;

#if DL1B_USE_SOFT_IIC
static soft_iic_info_struct dl1b_iic_struct;

#define dl1b_transfer_8bit_array(tdata, tlen, rdata, rlen)      (soft_iic_transfer_8bit_array(&dl1b_iic_struct, (tdata), (tlen), (rdata), (rlen)))
#else
#error "�ݲ�֧��Ӳ��IICͨѶ"
#endif

//-------------------------------------------------------------------------------------------------------------------
// �������     �����Ժ���Ϊ��λ�ķ�Χ����
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     dl1b_get_distance();
// ��ע��Ϣ     �ڿ�ʼ������̲�����Ҳ���ô˺���
//-------------------------------------------------------------------------------------------------------------------
void dl1b_get_distance (void)
{
    if(dl1b_init_flag)
    {
        uint8 data_buffer[3] = {0};
        int16 dl1b_distance_temp = 0;

        data_buffer[0] = DL1B_GPIO__TIO_HV_STATUS >> 8;
        data_buffer[1] = DL1B_GPIO__TIO_HV_STATUS & 0xFF;
        dl1b_transfer_8bit_array(data_buffer, 2, &data_buffer[2], 1);

        if(data_buffer[2])
        {

            data_buffer[0] = DL1B_SYSTEM__INTERRUPT_CLEAR >> 8;
            data_buffer[1] = DL1B_SYSTEM__INTERRUPT_CLEAR & 0xFF;
            data_buffer[2] = 0x01;
            dl1b_transfer_8bit_array(data_buffer, 3, data_buffer, 0);// clear Interrupt

            data_buffer[0] = DL1B_RESULT__RANGE_STATUS >> 8;
            data_buffer[1] = DL1B_RESULT__RANGE_STATUS & 0xFF;
            dl1b_transfer_8bit_array(data_buffer, 2, &data_buffer[2], 1);

            if(0x89 == data_buffer[2])
            {
                data_buffer[0] = DL1B_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 >> 8;
                data_buffer[1] = DL1B_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 & 0xFF;
                dl1b_transfer_8bit_array(data_buffer, 2, data_buffer, 2);
                dl1b_distance_temp = data_buffer[0];
                dl1b_distance_temp = (dl1b_distance_temp << 8) | data_buffer[1];

                if(dl1b_distance_temp > 4000 || dl1b_distance_temp < 5) // 0~4mm 视为无效
                {
                    dl1b_distance_mm = 8192;
                    dl1b_finsh_flag = 0;
                }
                else
                {
                    dl1b_distance_mm = dl1b_distance_temp;
                    dl1b_finsh_flag = 1;
                }
            }
            else
            {
                dl1b_distance_mm = 8192;
                dl1b_finsh_flag = 0;
            }
        }
        else
        {
            dl1b_distance_mm = 8192;
            dl1b_finsh_flag = 0;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     DL1B INT �ж���Ӧ��������
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     dl1b_int_handler();
// ��ע��Ϣ     ��������Ҫ�� DL1B_INT_PIN ��Ӧ���ⲿ�жϴ��������е���
//-------------------------------------------------------------------------------------------------------------------
void dl1b_int_handler (void)
{
#if DL1B_INT_ENABLE
    dl1b_get_distance();
#endif
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʼ�� DL1B
// ����˵��     void
// ���ز���     uint8           1-��ʼ��ʧ�� 0-��ʼ���ɹ�
// ʹ��ʾ��     dl1b_init();
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
uint8 dl1b_init (void)
{
    uint8   return_state    = 0;
    uint8   data_buffer[2 + sizeof(dl1b_config_file)];
    uint16  time_out_count  = 0;

#if DL1B_USE_SOFT_IIC
    soft_iic_init(&dl1b_iic_struct, DL1B_DEV_ADDR, DL1B_SOFT_IIC_DELAY, DL1B_SCL_PIN, DL1B_SDA_PIN);
#else
    iic_init(DL1B_IIC, DL1B_DEV_ADDR, DL1B_IIC_SPEED, DL1B_SCL_PIN, DL1B_SDA_PIN);
#endif
    gpio_init(DL1B_XS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    do
    {
        system_delay_ms(50);
        gpio_low(DL1B_XS_PIN);
        system_delay_ms(10);
        gpio_high(DL1B_XS_PIN);
        system_delay_ms(50);

        data_buffer[0] = DL1B_FIRMWARE__SYSTEM_STATUS >> 8;
        data_buffer[1] = DL1B_FIRMWARE__SYSTEM_STATUS & 0xFF;
        dl1b_transfer_8bit_array(data_buffer, 2, &data_buffer[2], 1);
        return_state = (0x01 == (data_buffer[2] & 0x01)) ? (0) : (1);
        if(1 == return_state)
        {
            break;
        }

        data_buffer[0] = DL1B_I2C_SLAVE__DEVICE_ADDRESS >> 8;
        data_buffer[1] = DL1B_I2C_SLAVE__DEVICE_ADDRESS & 0xFF;
        memcpy(&data_buffer[2], (uint8 *)dl1b_config_file, sizeof(dl1b_config_file));
        dl1b_transfer_8bit_array(data_buffer, 2 + sizeof(dl1b_config_file), data_buffer, 0);

        while(1)
        {
            data_buffer[0] = DL1B_GPIO__TIO_HV_STATUS >> 8;
            data_buffer[1] = DL1B_GPIO__TIO_HV_STATUS & 0xFF;
            dl1b_transfer_8bit_array(data_buffer, 2, &data_buffer[2], 1);
            if(0x00 == (data_buffer[2] & 0x01))
            {
                time_out_count = 0;
                break;
            }
            if(DL1B_TIMEOUT_COUNT < time_out_count ++)
            {
                return_state = 1;
                break;
            }
            system_delay_ms(1);
        }

        dl1b_init_flag = 1;
    }while(0);

#if DL1B_INT_ENABLE
    exti_init(DL1B_INT_PIN, EXTI_TRIGGER_FALLING);
    dl1b_int_handler();
    dl1b_finsh_flag = 0;
#endif
    set_tof_type(TOF_DL1B, dl1b_int_handler);

    return return_state;
}
