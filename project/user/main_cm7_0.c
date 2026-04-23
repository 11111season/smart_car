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
* �ļ�����          main_cm7_0
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 9.40.1
* ����ƽ̨          CYT4BB
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
// ���µĹ��̻��߹����ƶ���λ�����ִ�����²���
// ��һ�� �ر��������д򿪵��ļ�
// �ڶ��� project->clean  �ȴ��·�����������

// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������

// **************************** �������� ****************************


int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// ʱ�����ü�ϵͳ��ʼ��<��ر���>
    debug_init();                       // ���Դ�����Ϣ��ʼ��
    // �˴���д�û����� ���������ʼ�������
    
     ALL_Init();

    
    
    // �˴���д�û����� ���������ʼ�������
    while(true)
    {
        // �˴���д��Ҫѭ��ִ�еĴ���
        
         PIDVelY.kp=buff_value;
//         printf("2e=%5f\r\n", PIDVelY.kp);  
          
         

      
//          pmw3901_get_motion();
//          printf("%5f\r\n", qmc5883l_heading);  
//          printf("%d\r\n", dl1b_distance_mm);  
            
      
//--------------------vofa------------
//        //imu
//        printf("%5f, %5f, %5f\r\n", imu660rc_roll, imu660rc_pitch,  imu660rc_yaw);
//        printf("%d, %d, %d\r\n", imu660rc_acc_x, imu660rc_acc_y,  imu660rc_acc_z);
//        printf("%5f, %5f, %5f\r\n", imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z);
//        printf("%5f, %5f, %5f\r\n",eulerAngle.roll, eulerAngle.roll, eulerAngle.roll);

          
//        // ���ԣ���ӡPID�м�ֵ
//        printf("%5f, %5f, %5f\r\n",PIDRoll.out, PIDPitch.out, PIDYaw.out);
//        printf("%5f, %5f, %5f\r\n",PIDRateX.out, PIDRateY.out, PIDRateZ.out);
          
         //ROLL��
//         printf("%5f, %5f\r\n", PIDRateX.target,imu_data.gyro_x);//�ڻ�
//        printf("%5f, %5f\r\n",PIDRoll.out,imu_data.gyro_x);//�ڻ�

//        printf("%5f, %5f\r\n",PIDRoll.target,imu660rc_roll);//�⻷


       
//          printf("%f, %f,%d,%d\r\n",-pmw3901_delta_x/0.02,-pmw3901_delta_y/0.02,imu660rc_gyro_x,imu660rc_gyro_y);//�⻷
//          printf("%d,%d\r\n",pmw3901_delta_x,pmw3901_delta_y);

//          printf("%f, %f,%f,%f,%d\n",of.vx,of.vy,of.vx1,of.vy1,dl1b_distance_mm);//�⻷
//           printf("%5f, %5f,%5f,%5f,%5f\r\n",imu_data.acc_x ,imu_data.acc_y,world_data.ax,world_data.ay,world_data.az);//�⻷
//           printf("%5f, %5f,%5f,%5f\r\n",imu660rc_quarternion[0] ,imu660rc_quarternion[1],imu660rc_quarternion[2],imu660rc_quarternion[3]);//�⻷

        
        
        
//        printf("%5f, %5f\r\n",alt.vz_deriv,alt.vz_deriv1);
//        printf("%5f, %5f, %5f\r\n",alt.vz_acc,alt.vz_deriv1,world_data.vz);

           
        
////--------------------IPS----------
//        ips200_show_string( 16*0,  16*4, "roll=:");     ips200_show_float(16*6,  16*4, eulerAngle.roll,  3,5);
//        ips200_show_string( 16*0,  16*5, "pitch=:" );     ips200_show_float(16*6,  16*5, eulerAngle.pitch,   3,5);
        ips200_show_string( 16*0,  16*6, "yaw=:"  );     ips200_show_float(16*6,  16*6, eulerAngle.yaw, 3,5);
        
//        ips200_show_string( 16*0,  16*4, "roll=:");     ips200_show_float(16*6,  16*4, imu660rc_roll,  3,5);
//        ips200_show_string( 16*0,  16*5, "pitch=:" );     ips200_show_float(16*6,  16*5, imu660rc_pitch,   3,5);
        ips200_show_string( 16*0,  16*6, "yaw=:"  );     ips200_show_float(16*6,  16*5, imu660rc_yaw, 3,5);
        
        
        ips200_show_string( 16*0,  16*7, "m1=:");     ips200_show_float(16*6,  16*7, m1, 3,5);
        ips200_show_string( 16*0,  16*8, "m2=:");     ips200_show_float(16*6,  16*8, m2, 3,5);
        ips200_show_string( 16*0,  16*9, "m3=:");     ips200_show_float(16*6,  16*9, m3, 3,5);
        ips200_show_string( 16*0,  16*10, "m4=:");    ips200_show_float(16*6,  16*10, m4, 3,5);
//        ips200_show_string( 16*0,  16*11, "test=:");    ips200_show_float(16*6,  16*11, buff_value, 3,5);

        
        
        system_delay_ms(20);
      
        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}

// **************************** �������� ****************************
