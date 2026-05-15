#include "zf_common_headfile.h"
/*
1    2
4    3
右倾roll>0,23电机+
前倾pitch>0,34电机+
逆时针yaw>0,23反牙顺时针+




flight_control()
 ├── 状态机
 ├── takeoff_control()
 ├── hover_control()
 │     ├── position_control()
 │     ├── height_control()
 │     └── stabilization()
 └── land_control()

👉 姿态环：dt ≈ 0.002 ~ 0.005（200~500Hz）
👉 位置/高度环：dt ≈ 0.01 ~ 0.02（50~100Hz）
| 控制层                 | 频率        | dt          |
| ------------------- | --------- | ----------- |
| 角速度环（PIDVelX/Y/Z）   | 250~500Hz | 0.002~0.004 |
| 姿态环（Roll/Pitch/Yaw） | 100~200Hz | 0.005~0.01  |
| 高度环                 | 50~100Hz  | 0.01~0.02   |
| 位置环                 | 50Hz      | 0.02        |



切换状态要加等待


*/
//控制代码

uint8_t out_flag;

void flight_control(float dt)
{
    static flight_state_e last_flight_state = STATE_LOCK;
     // 状态变化检测
    if(flight_state != last_flight_state)
    {
        PID_Rest_Change (pPidObject,12);   // ⭐关键
        flag.hover_lock = 0;  // 重新锁点
        flag.take_off_yaw = 0;

        last_flight_state = flight_state;
    }

    
    
    if(rc.emergency_cmd)
    {
        motor_set_all(0,0,0,0);

        flight_state = STATE_LOCK;

        return;
    }

    switch(flight_state)
    {
        case STATE_LOCK:
            motor_set_all(0,0,0,0);
            if(rc.unlock_cmd)
            {
                flight_state = STATE_UNLOCK;
            }
            break;

        case STATE_UNLOCK:
            motor_set_all(500,500,500,500);//电机转动但是不起飞
            if(rc.lock_cmd)
            {
                flight_state = STATE_LOCK;
            }
            if(rc.takeoff_cmd)
            {
                flight_state = STATE_TAKEOFF;
            }
            break;
            
        case STATE_TAKEOFF:
            take_off(dt);
            break;
            
        case STATE_FLY:
//            flight_mode_control(dt);
            pos_hold_control(dt);
            if(rc.land_cmd)
            {
                flight_state = STATE_LAND;
            }

            break;
        case STATE_LAND:
            land(dt);
            break;    
           
       
//        case STATE_IDLE://手飞
//            PIDRoll.target = 0;
//            PIDPitch.target = 0;
//            //高度怎么处理？油门怎么给
//            stabilization(dt);
//            break;
//            
//        case STATE_TASK://路径规划
//            hover_control(dt);
//            break;
            
    }
}
//------------------- 自稳-------------------
void stabilization(float dt)
{  
    
//---------调试------------
  //  PID_Update(&PIDVelX, 0, imu_data.gyro_x, dt);   

        
//-------------------------pid------------
    //roll
    PID_Update_d_measure(&PIDRoll, PIDRoll.target, eulerAngle.roll, dt);           
    PID_Update(&PIDVelX, PIDRoll.out,imu_data.gyro_x, dt);        
 //   printf("%5f,%5f,%5f\r\n",PIDRoll.out,PT1Filter_Apply(&filter_imu_gyro_x,imu_data.gyro_x),PIDRoll.deriv_integ);
    //  printf("%5f,%5f,%5f,%5f\r\n",PIDVelX.out,imu_data.gyro_x,PIDVelX.deriv_integ,PIDVelX.integ);
   
    //pitch
//     PID_Update(&PIDVelY, 0, imu_data.gyro_y, dt);     
    
    PID_Update_d_measure(&PIDPitch, PIDPitch.target, eulerAngle.pitch, dt);         
    PID_Update(&PIDVelY, PIDPitch.out, imu_data.gyro_y, dt);          

    //yaw
//    PID_Update(&PIDVelZ, 0, imu_data.gyro_z, dt);   
 
    PID_Update_d_measure(&PIDYaw, PIDYaw.target, eulerAngle.yaw, dt);            
    PID_Update(&PIDVelZ, PIDYaw.out, imu_data.gyro_z, dt);         
//      printf("%5f\r\n",PIDVelZ.out);
    //Height
    PID_Update(&PIDHeight, PIDHeight.target, world_data.pz, dt);
    PID_Update(&PIDVelH, PIDHeight.out, world_data.vz, dt);
    
    //  电机混控输出 
    float THR;
    float hover_thr = 700;//待测量
    THR = hover_thr;
    //    THR = hover_thr + PIDVelH.out;
    
    //  THR为基础油门，PID输出为修正量
    m1 = (uint16_t)THR + (uint16_t)PIDVelX.out - (uint16_t)PIDVelY.out - (uint16_t)PIDVelZ.out; // 左前电机
    m2 = (uint16_t)THR - (uint16_t)PIDVelX.out - (uint16_t)PIDVelY.out + (uint16_t)PIDVelZ.out; // 右前电机
    m3 = (uint16_t)THR - (uint16_t)PIDVelX.out + (uint16_t)PIDVelY.out - (uint16_t)PIDVelZ.out; // 左后电机
    m4 = (uint16_t)THR + (uint16_t)PIDVelX.out + (uint16_t)PIDVelY.out + (uint16_t)PIDVelZ.out; // 右后电机
    
    //  电机输出限幅
    m1 = LIMIT(m1, MOTOR_MIN_DUTY, MOTOR_MAX_DUTY);
    m2 = LIMIT(m2, MOTOR_MIN_DUTY, MOTOR_MAX_DUTY);
    m3 = LIMIT(m3, MOTOR_MIN_DUTY, MOTOR_MAX_DUTY);
    m4 = LIMIT(m4, MOTOR_MIN_DUTY, MOTOR_MAX_DUTY);
    
    if(out_flag == 1)
    {
      motor_set(1, m1);
      motor_set(2, m2);
      motor_set(3, m3);
      motor_set(4, m4);    
    }
    else if(out_flag == 0)
    {
      m1 = 0;
      m2 = 0;
      m3 = 0;
      m4 = 0;
  
      motor_set(1, m1);
      motor_set(2, m2);
      motor_set(3, m3);
      motor_set(4, m4); 
    }
    
    //  电机输出

}

////高度
//float height_control(float dt)
//{
//    PID_Update(&PIDHeight, 1, world_data.pz, dt);
//    PID_Update(&PIDVelH, PIDHeight.out, world_data.vz, dt);
//    
//    return PIDVelH.out;
//    
//    
//    //base_thr = hover_thr + PIDVelH.out;
//}

//位置
void position_control(float dt)
{   
    // 位置外环
    PID_Update(&PIDPosX, PIDPosX.target, world_data.px, dt);
    PID_Update(&PIDPosY, PIDPosY.target, world_data.py, dt);
    
    // 位置内环
    PID_Update(&PIDPosX_Vel, PIDPosX.out, world_data.vx, dt);  // 输出给 Roll
    PID_Update(&PIDPosY_Vel, PIDPosY.out, world_data.vy, dt);  // 输出给 Pitch
    
    // 输出姿态目标
    PIDRoll.target  = PIDPosX_Vel.out;
    PIDPitch.target = PIDPosY_Vel.out;
}



//锁点代码
void hover_lock(void)
{
    if(!flag.hover_lock)
    {
        PIDPosX.target = world_data.px;
        PIDPosY.target = world_data.py;
        PIDHeight.target = world_data.pz;

        flag.hover_lock = 1;
    }
}

//悬停
void hover_control(float dt)
{
    // 锁点
    hover_lock();
    
    //位置环控制
    position_control(dt);

    //姿态控制
    stabilization(dt);
}

//一键降落OK
void land(float dt)
{
    //高度控制+限幅
    PIDHeight.target -= 0.3f * dt; 
    
    if(PIDHeight.target < 0.05f)
    PIDHeight.target = 0.05f; 
    
    //原地降落rollpitch都等于0
    PIDRoll.target = 0;
    PIDPitch.target = 0;
    
    //锁当前yaw角度
    if(!flag.take_off_yaw)
    {
        PIDYaw.target = eulerAngle.yaw;
        flag.take_off_yaw = 1;
    }
     
     //近地保护（参数未调）
    if(world_data.pz < 0.15f)
    {
        // 减弱控制（防弹）
        PIDVelH.out *= 0.5f;
    }
    
    //姿态控制
    stabilization(dt);
    
    //检测上锁条件：低高度+低速度（参数未确定）
    static uint16_t land_cnt = 0;
    if(world_data.pz < 0.1f&& fabs(world_data.vz) < 0.1f)
    {
        land_cnt++;
        
        if(land_cnt > 50)   // 50次 ≈ 0.5秒（100Hz）
        {
            flight_state = STATE_LOCK;
        }
    }
    else
    {
        land_cnt = 0;
    }

    
}

//起飞OK里面还有dt
void take_off(float dt)
{
    static uint8_t step = 0;
    static float takeoff_z = 1.0f;

    switch(step)
    {
        // 初始化锁角度
        case 0:
            // 锁yaw
            PIDYaw.target = eulerAngle.yaw;

            // 清高度目标（从当前开始）
            PIDHeight.target = world_data.pz;
            step = 1;
        break;

        case 1://起飞
          
            PIDHeight.target += 0.3f * dt; 

            // 姿态保持水平
            PIDRoll.target = 0;
            PIDPitch.target = 0;

            stabilization(dt);

            // 离地判断（有高度）
            if(world_data.pz > 0.15f)
            {
                step = 2;
            }
        break;

        case 2: //上升目标高度
        
            PIDHeight.target += 0.5f * dt;  //加大增幅 
            
            PIDRoll.target = 0;
            PIDPitch.target = 0;

            stabilization(dt);

            // 到达目标高度
            if(fabs(world_data.pz - takeoff_z) < 0.1f&& fabs(world_data.vz) < 0.2f)
            {
                step = 0;   // 重置
                flag.hover_lock = 0;
                step = 3;
                
            }
        break;
        
        case 3://悬停保持
          
            hover_control(dt);
            flight_state = STATE_FLY;

        break;
        
    }
}


//void flight_mode_control(float dt)
//{
//    switch(flight_mode)
//    {
//        case MODE_MANUAL:
//            {
//                manual_control(dt);
//            }
//        break;
//
//        case MODE_ALT_HOLD:
//            {
//                alt_hold_control(dt);
//            }
//        break;
//        
//        case MODE_POS_HOLD:
//            {
//                pos_hold_control(dt);
//            }
//        break;
//
//        case MODE_TASK:
//            {
////                task_control(dt);
//            }
//        break;
//    }
//}


void pos_hold_control(float dt)
{
    
    // 第一次进入锁点
    hover_lock();

    // 摇杆微调目标位置
    PIDPosX.target = rc.roll  * 0.0f;
    PIDPosY.target = rc.pitch * 0.0f;

    // 高度控制
    PIDHeight.target = rc.thr * 0.0f;

    // yaw控制
    PIDYaw.target = rc.yaw * 0.0f;

    // 位置控制
    position_control(dt);

    // 姿态控制
    stabilization(dt);
}


//void alt_hold_control(float dt)
//{
//    // 高度目标
//    PIDHeight.target += rc.thr * 0.0f;
//
//    // 姿态
//    PIDRoll.target  = rc.roll  * 0.0f;
//    PIDPitch.target = rc.pitch * 0.0f;
//
//    // yaw
//    PIDYaw.target += rc.yaw * 0.0f * dt;
//
//    stabilization(dt);
//}


//
//void manual_control(float dt)
//{
//    // 摇杆直接控制角度
//
//    PIDRoll.target  = rc.roll  * 0.0f;
//    PIDPitch.target = rc.pitch * 0.0f;
//
//    // yaw速度控制
//    PIDYaw.target += rc.yaw * 0.0f * dt;
//
////     油门直接控制
//    THR =  rc.thr * 300;
//
//    stabilization(dt);
//}




////悬停加死区
//if(fabs(PIDPosX.target - world_data.px) < 0.02f)
//    vx_target = 0;
//
//if(fabs(PIDPosY.target - world_data.py) < 0.02f)
//    vy_target = 0;
//
//
////光流失效保护
//if(of.height < 0.15f || of.height > 2.5f)
//{
//    PIDRoll.target = 0;
//    PIDPitch.target = 0;
//}
//
////限角度
//PIDRoll.target  = LIMIT(PIDRoll.target,  -10, 10);
//PIDPitch.target = LIMIT(PIDPitch.target, -10, 10);



////低高度禁止位置控制
//if(world_data.pz < 0.2f)
//{
//    roll_target = 0;
//    pitch_target = 0;
//    return;
//}




//打印转换
//const char* get_state_name(flight_state_e s) {
//    switch (s) {
//        case STATE_LOCK:     return "LOCK";
//        case STATE_IDLE:     return "IDLE";
//        case STATE_TAKEOFF:  return "TAKEOFF";
//        case STATE_EMERGENCY: return "EMERGENCY";
//        default:             return "UNKNOWN";
//    }
//}

// 这样打印就很直观了
//printf("当前飞控状态: %s", get_state_name(state));