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
    //获取高度滤波
    of.height = PT1Filter_Apply(&filter_height,dl1b_distance_mm)/1000.0f;
    
    //读光流
    of.dx = pmw3901_delta_x;
    of.dy = pmw3901_delta_y;
    of.dx_i = pmw3901_delta_x_i;
    of.dy_i = pmw3901_delta_y_i;
    
    
    //去除陀螺仪影响，,g用rad/s，并且转换速度
    of.vx = ((-of.dx/dt) - imu660rc_gyro_y)* of.height ;
    of.vy = ((-of.dy/dt) - imu660rc_gyro_x)* of.height ;
    
    //滤波
    of.vx1 = PT1Filter_Apply(&filter_pwm3901_vx,of.vx);
    of.vy1 = PT1Filter_Apply(&filter_pwm3901_vy,of.vy);
    
    //imu速度积分
    of.vx_imu += world_data.vx * dt;
    of.vy_imu += world_data.vy * dt;
//
//    //融合光流速度和陀螺仪积分速度
//    world_data.vx =
//    world_data.vy =
//      
//
//   //陀螺仪位置积分
//   of.px=
//   of.py=
//
//   //imu位置积分
//   of.px_imu =
//   of.py_imu =
//
//   //水平位置融合光流位置和陀螺仪积分位置  
//    world_data.px =
//    world_data.py =
    
}








////先测K,再测scale
//void OF_data_deal(float dt)
//{
//    //获取高度
//    of.height = dl1b_distance_mm/1000.0f;
//    
//    //读光流
//    of.dx = pmw3901_delta_x;
//    of.dy = pmw3901_delta_y;
//    of.dx_i = pmw3901_delta_x_i;
//    of.dy_i = pmw3901_delta_y_i;
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
//
//
//
//   //水平位置融合光流位置和陀螺仪积分位置  
//
//}
