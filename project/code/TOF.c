#include "zf_common_headfile.h"

/*
//----------融合速度，互补滤波，权重参数
//融合速度：加速度积分，高度差分
//融合高度：测量高度和速度积分

清零
alt.vz_acc = 0;
alt.height_acc = 初始高度（重要！！）
world_data.vz = 0;
world_data.pz = 初始高度
last_az = 0;

赋值否则起飞会跳变
alt.height_acc = alt.height;
world_data.pz = alt.height;

*/
void height_data_deal(const float dt)
{
    float az;
    float last_az = 0.0f;

    //获取高度数据
    alt.height= dl1b_distance_mm/1000.0f;
    //滤波
      
    //激光高度差分速度
    alt.vz_deriv = (alt.height - alt.last_height) / dt;
    
    // 差分速度简单低通滤波（防抖）
    alt.vz_deriv = 0.7f * alt.vz_last_deriv + 0.3f * alt.vz_deriv;
    alt.vz_last_deriv=alt.vz_deriv;
    
    //梯形积分得到速度
    az = world_data.az;
    alt.vz_acc += (az + last_az) * 0.5f * dt;                //纯IMU积分链”

    //速度融合
    world_data.vz =  0.98f *  alt.vz_acc + (1.0f -  0.98f) * alt.vz_deriv;     // IMU权重大VZ_ALPHA = 0.98f  
    
    // 防漂移（泄露）
    world_data.vz *=  0.98f ;                    // 防漂 VEL_DECAY= 0.98f   
    
    // 积分高度
    alt.height_acc += world_data.vz * dt;
    
    // 高度融合
    world_data.pz = 0.99f * alt.height_acc + (1.0f - 0.99f) * alt.height;     // 高度更信IMU积分   H_BETA   = 0.99f   
    
    //  更新历史值
    alt.last_height = alt.height;
    last_az = az;
    
}