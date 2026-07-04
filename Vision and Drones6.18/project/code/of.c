#include "zf_common_headfile.h"
/*


����+IMU+�߶�


w=��x/f*��t
v=w*h

��ʵ�ٶ� �� �������� �� �߶�
v=��x*h/f*t

���� + IMU�����ٶ�/���ٶȣ��ں�
�������˲�
�򻥲��˲�


�ṩ��λ�Ʊ仯��
�ṩ���ۼ�λ�ơ�
int16 pmw3901_delta_x
int16 pmw3901_delta_y

int32 pmw3901_delta_x_i
int32 pmw3901_delta_y_i


  vx = �� * (�����ٶ�) + (1-��) * (IMU�����ٶ�);


������ʼ��λcounts�����ؼ��� / ����������

float vx = scale * pmw3901yong _delta_x * height;

pos_x += vx * dt;




*/

void OF_init(void)
{
    upflow302_receive_init();                                              
    //������ʼ��
    world_data.vx = 0;
    world_data.vy = 0;

    world_data.px = 0;
    world_data.py = 0;
    
    of.vx = 0;
    of.vy = 0;
    of.vx_pt1 = 0;
    of.vy_pt1 = 0;

    of.dx = 0;
    of.dy = 0;
    of.dx_i = 0;
    of.dy_i = 0;
    
    if(!flag.of_init)
{
    world_data.vx = of.vx_pt1;
    world_data.vy = of.vy_pt1;

    flag.of_init = 1;
    return;
}

}

//�Ȳ�K,�ٲ�scale
void OF_data_deal(float dt)
{
    //��ȡ�Ѿ��˲��߶�
    of.height = alt.height;
    
    //������
    of.dx = -upflow302_receive.upflow302_y;
    of.dy = upflow302_receive.upflow302_x;
//    of.dx_i = pmw3901_delta_x_i;
//    of.dy_i = pmw3901_delta_y_i;
    
    //ת���ɻ���
    imu_data.gyro_rad_y = imu_data.gyro_y * (PI / 180.0f);
    imu_data.gyro_rad_x = imu_data.gyro_x * (PI / 180.0f);

    of.optical_ang_x = of.dx / (dt * of.K); // ������֪����X����ٶȣ�����/�룩  
    of.optical_ang_y = of.dy / (dt * of.K);

    float wx = imu_data.gyro_x * (PI / 180.0f);
    float wz = imu_data.gyro_z * (PI / 180.0f);
//    of.optical_ang_x -= (wz * OF_SENSOR_Y) / of.height;
//    of.optical_ang_y -= (wz * OF_SENSOR_Y) / of.height;
 //  of.optical_ang_y -= (wx * OF_SENSOR_Y) / of.height; // ������֪����Y����ٶ� 

    //ȥ��������Ӱ�죬,g��rad/s������ת���ٶ�
    
    of.rotation_ex = of.optical_ang_x + imu_data.gyro_rad_y ;
    of.rotation_ey = of.optical_ang_y + imu_data.gyro_rad_x ;



    of.vx = of.rotation_ex * of.height ;
    of.vy = of.rotation_ey * of.height ;
    
    //�˲�
    of.vx_pt1 = PT1Filter_Apply(&filter_pwm3901_vx,of.vx);
    of.vy_pt1 = PT1Filter_Apply(&filter_pwm3901_vy,of.vy);
    
}




//----------------------------------�������ٶȼ��ں�------------------------------
//�����ǹ۲⣬����У׼���ٶȼ�Ư��
void velocity_mahony_fusion(float dt) 
{
    static float Kp, Ki,ex,ey;
    static float integralX = 0.0f, integralY = 0.0f;

    //��������Ӧ����
//��Ӧ����
    Kp = 0.8f;   // P����
    // 起飞阶段不用 I（高度还没到，水平速度不可靠）
    Ki = flag.takeoff_phase ? 0.0f : 0.2f;
    
    // ���ù۲�ֵ������x��y������ٶ��ϵĹ۲����
// ���ٶȻ��֣���й¶����ֱֹ��ƫ��Ư�ƣ�
    world_data.vx += world_data.ax * dt;


    world_data.vy += world_data.ay * dt;

                        
    ex = of.vx_pt1 - world_data.vx;
    ey = of.vy_pt1 - world_data.vy;  
   
    // ������У��
    if (Ki > 0.0f) 
    {
        integralX += Ki * ex * dt;
        integralY += Ki * ey * dt;
     }
            
    //�ں�
    world_data.vx += Kp * ex + integralX ; 
    world_data.vy += Kp * ey + integralY ;     
    
    //���ֳ�λ��
    world_data.px += world_data.vx * dt; 
    world_data.py += world_data.vy * dt;     

    // 速度环诊断：期望速度 vs 实际速度 vs PID输出 vs 实际角度

    }

