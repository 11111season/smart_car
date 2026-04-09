#include "zf_common_headfile.h"
/*


光流+IMU+高度


w=Δx/f*Δt
v=w*h

真实速度 ≈ 光流像素 × 高度
v=Δx*h/f*t

光流 + IMU（加速度/角速度）融合
卡尔曼滤波
或互补滤波


提供“位移变化”
提供“累计位移”
int16 pmw3901_delta_x
int16 pmw3901_delta_y

int32 pmw3901_delta_x_i
int32 pmw3901_delta_y_i


  vx = α * (光流速度) + (1-α) * (IMU积分速度);


光流初始单位counts（像素计数 / 光流计数）

float vx = scale * pmw3901yong _delta_x * height;

pos_x += vx * dt;




*/


//先测K,再测scale
void OF_data_deal(float dt)
{
    //获取已经滤波高度
    of.height = alt.height;
    
    //读光流
    of.dx = pmw3901_delta_x;
    of.dy = pmw3901_delta_y;
    of.dx_i = pmw3901_delta_x_i;
    of.dy_i = pmw3901_delta_y_i;
    
    //转化成弧度
    float gyro_rad_y = imu_data.gyro_y * (PI / 180.0f);
    float gyro_rad_x = imu_data.gyro_x * (PI / 180.0f);
    
    //去除陀螺仪影响，,g用rad/s，并且转换速度
    of.vx = ((-of.dx/dt*430) - gyro_rad_y)* of.height ;
    of.vy = ((-of.dy/dt*430) - gyro_rad_x)* of.height ;
    
    //滤波
    of.vx1 = PT1Filter_Apply(&filter_pwm3901_vx,of.vx);
    of.vy1 = PT1Filter_Apply(&filter_pwm3901_vy,of.vy);
    
}




//----------------------------------光流加速度计融合------------------------------
//光流是观测，用于校准加速度计漂移
void velocity_mahony_fusion(float dt) 
{
    static float Kp, Ki,ex,ey;
    static float integralX = 0.0f, integralY = 0.0f;

    //用于自适应增益
    Kp = 0.3f ;  //0.08f ，越大则越信任观测数据
    Ki = 0.027f;  //0.000001f
    
    // 利用观测值，计算x，y方向加速度上的观测误差
    world_data.vx += world_data.ax * dt;
    world_data.vy += world_data.ay * dt;
                        
    ex = of.vx1 - world_data.vx;
    ey = of.vy1 - world_data.vy;  

    // 积分项校正
    if (Ki > 0.0f) 
    {
        integralX += Ki * ex * dt;
        integralY += Ki * ey * dt;
     }
            
    //融合
    world_data.vx += Kp * ex + integralX ; 
    world_data.vy += Kp * ey + integralY ;     
    
    //积分出位置
    world_data.px += world_data.vx * dt; 
    world_data.py += world_data.vy * dt;     
 
}


////先测K,再测scale
//void OF_data_deal(float dt)
//{
//    //获取高度
//    of.height = dl1b_distance_mm/1000.0f;
//    
//    //读光流
//    
//    //转角速度
//    
//    //光流去旋转
//
//
//    //算水平方向速度
//
//    
//    //imu速度积分
//
//
//    //融合光流速度和陀螺仪积分速度
//
//      
//    //滤波
//  
//   //位置积分
//
//   //水平位置融合光流位置和陀螺仪积分位置  
//
//}
