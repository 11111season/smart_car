#include "zf_common_headfile.h"

/*
//----------融合速度，互补滤波，权重参数
//融合速度：加速度积分，高度差分
//高度：激光比较准，直接用

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

void height_data_deal(float dt)
{
    //获取高度数据滤波,并直接赋值给高度，激光比较准
    alt.height = PT1Filter_Apply(&filter_height,dl1b_distance_mm)/1000.0f; 
    world_data.pz = alt.height;
    
    //激光高度差分速度+低通滤波（未调）
    alt.vz_deriv = (alt.height - alt.last_height) / dt;
    alt.vz_deriv1 = PT1Filter_Apply(&filter_height_vz,alt.vz_deriv);
    alt.last_height = alt.height;
       
    //积分得到速度
    alt.vz_acc += world_data.az * dt;               

    //互补滤波,alpha未测
    //速度在漂imu大了，速度很抖激光大了，vz太抖imu权重加
    float alpha_height_vz = 0.98;
    world_data.vz =  alpha_height_vz *  alt.vz_acc + (1.0f -  alpha_height_vz) * alt.vz_deriv;     
    
    // 防漂移（泄露）
    world_data.vz *=  0.98f ;                    // 防漂 VEL_DECAY= 0.98f   
       
}

//void height_data_deal(const float dt)
//{
//    float az;
//    float last_az ;//= 0.0f;
//
//    //获取高度数据滤波,并直接赋值给高度，激光比较准
//    alt.height = PT1Filter_Apply(&filter_height,dl1b_distance_mm)/1000.0f; 
//    world_data.pz = alt.height;
//    
//    //激光高度差分速度
//    alt.vz_deriv = (alt.height - alt.last_height) / dt;
//    alt.last_height = alt.height;
//    
//    // 差分速度简单低通滤波（防抖）
//    alt.vz_deriv = 0.7f * alt.vz_last_deriv + 0.3f * alt.vz_deriv;
//    alt.vz_last_deriv=alt.vz_deriv;
//    
//    //梯形积分得到速度
//    az = world_data.az;
//    alt.vz_acc += (az + last_az) * 0.5f * dt;                //纯IMU积分链”
//
//    //速度融合
//    world_data.vz =  0.98f *  alt.vz_acc + (1.0f -  0.98f) * alt.vz_deriv;     // IMU权重大VZ_ALPHA = 0.98f  
//    
//    // 防漂移（泄露）
//    world_data.vz *=  0.98f ;                    // 防漂 VEL_DECAY= 0.98f   
//    
//    // 积分高度
//    alt.height_acc += world_data.vz * dt;
//    
//    // 高度融合
//    world_data.pz = 0.99f * alt.height_acc + (1.0f - 0.99f) * alt.height;     // 高度更信IMU积分   H_BETA   = 0.99f   
//    
//    //  更新历史值
//    
//    last_az = az;
//    
//}