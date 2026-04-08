#include "zf_common_headfile.h"
/*
1    2
4    3
右倾roll>0,23电机+
抬头pitch>0,12电机+
逆时针yaw>0,23反牙顺时针+
*/
//------------------- 自稳-------------------
void stabilization(float dt)
{
//    if(!flag.unlock) return; // 未解锁直接返回
  

    
//---------调试------------
    PID_Update(&PIDRateX, 0, imu_data.gyro_x, dt);   



  
        
//-------------------------pid------------
//    //roll
//    PID_Update(&PIDRoll, 0, eulerAngle.roll, dt);           
//    PID_Update(&PIDRateX, PIDRoll.out, imu_data.gyro_x, dt);        
//
//    //pitch
//    PID_Update(&PIDPitch, 0, eulerAngle.pitch, dt);         
//    PID_Update(&PIDRateY, PIDPitch.out,imu_data.gyro_y, dt);          
//
//    //yaw
//    PID_Update(&PIDYaw, 180, eulerAngle.pitch, dt);            
//    PID_Update(&PIDRateZ, PIDYaw.out,imu_data.gyro_z, dt);         
//    
//    //height
//    PID_Update(&PIDHeight, 1, sensor.height, dt);            
//    PID_Update(&PIDRateH, PIDHeight.out,world_data.vz, dt);
    
    
    
    
    //  电机混控输出 
    uint16_t THR = 580;
//    uint16_t hover_thr;//待测量
//    THR = hover_thr + (uint16_t)PIDRateH.out;
    
    //  THR为基础油门，PID输出为修正量
    m1 = (int16_t)THR + (int16_t)PIDRateX.out + (int16_t)PIDRateY.out - (int16_t)PIDRateZ.out; // 左前电机
    m2 = (int16_t)THR - (int16_t)PIDRateX.out + (int16_t)PIDRateY.out + (int16_t)PIDRateZ.out; // 右前电机
    m3 = (int16_t)THR - (int16_t)PIDRateX.out - (int16_t)PIDRateY.out - (int16_t)PIDRateZ.out; // 左后电机
    m4 = (int16_t)THR + (int16_t)PIDRateX.out - (int16_t)PIDRateY.out + (int16_t)PIDRateZ.out; // 右后电机
    
    //  电机输出限幅
    m1 = LIMIT(m1, MOTOR_MIN_DUTY, MOTOR_MAX_DUTY);
    m2 = LIMIT(m2, MOTOR_MIN_DUTY, MOTOR_MAX_DUTY);
    m3 = LIMIT(m3, MOTOR_MIN_DUTY, MOTOR_MAX_DUTY);
    m4 = LIMIT(m4, MOTOR_MIN_DUTY, MOTOR_MAX_DUTY);
    
    //  电机输出
    motor_set(1, m1);
    motor_set(2, m2);
    motor_set(3, m3);
    motor_set(4, m4);
}
























////------------------- 电机控制 -------------------
//void MotorControl(void)
//{	
//    static uint8_t status = WAITING_1;
//    
//    //  紧急锁定
//    if(flag.unlock == EMERGENT)
//        status = EXIT_255;
//        
//    switch(status)
//    {
//        //  等待状态
//        case WAITING_1:
//            m1 = m2 = m3 = m4 = 0;  // 电机停转
//            motor_set(1, 0);
//            motor_set(2, 0);
//            motor_set(3, 0);
//            motor_set(4, 0);
//            if(flag.unlock)
//            {
//                status = WAITING_2;
//            }
//            break;
//            
//        //  等待电机启动
//        case WAITING_2:
//            if(rc.thr > 1100)  // 等待油门高于启动阈值
//            {
//                PID_Rest(pPidObject, 6); // 复位所有PID
//                status = PROCESS_31;
//            }
//            break;
//            
//        //  正常电机控制
//        case PROCESS_31:
//        {
//            //  安全检测：角度异常且油门较高 → 强制锁定
//            if(eulerAngle.pitch < -50 || eulerAngle.pitch > 50 || 
//               eulerAngle.roll < -50  || eulerAngle.roll > 50)
//                if(rc.thr > 1200)
//                    flag.unlock = EMERGENT;
//            
//            //  PID姿态控制已在中断 stabilization() 中完成
//        }	
//        break;
//        
//        //  退出
//        case EXIT_255:
//            m1 = m2 = m3 = m4 = 0;
//            motor_set(1, 0);
//            motor_set(2, 0);
//            motor_set(3, 0);
//            motor_set(4, 0);
//            PID_Rest(pPidObject, 6);
//            status = WAITING_1;	
//            break;
//    }
//}
//
////------------------- 遥控器解锁逻辑 -------------------
//void remote_unlock(void)
//{
//    static uint8_t status = WAITING_1;
//    static uint16_t cnt = 0;
//    
//    //  油门遥杆左下角 + 偏航左 → 锁定
//    if(rc.thr < 1050 && rc.yaw < 1200)
//    {
//        status = EXIT_255;
//    }
//    
//    switch(status)
//    {
//        case WAITING_1: // 等待解锁 - 步骤1
//            if(rc.thr < 1150)           
//            {			 
//                status = WAITING_2;				 
//            }		
//            break;
//            
//        case WAITING_2: // 步骤2 - 推油门到高位
//            if(rc.thr > 1600)          
//            {		
//                static uint8_t cnt_high = 0;
//                cnt_high++;		
//                if(cnt_high > 5) // 最高油门需保持200ms以上
//                {	
//                    cnt_high = 0;
//                    status = WAITING_3;
//                }
//            }
//            else
//            {
//                static uint8_t cnt_high = 0;
//                cnt_high = 0;
//            }
//            break;
//            
//        case WAITING_3: // 步骤3 - 油门回到低位
//            if(rc.thr < 1100)          
//            {			 
//                status = WAITING_4;				 
//            }			
//            break;			
//            
//        case WAITING_4: // 解锁前准备
//            flag.unlock = 1;
//            status = PROCESS_31;
//            //  偏航归零
//            eulerAngle.yaw = 0;
//            PIDYaw.target = 0;
//            PIDYaw.measured = 0;
//            break;		
//            
//        case PROCESS_31: // 进入解锁状态
//            if(rc.thr < 1020)
//            {
//                if(cnt++ > 3000) // 油门遥杆处于最低9S自动上锁
//                {								
//                    status = EXIT_255;								
//                }
//            }
//            else if(!flag.unlock) // 其他条件锁定
//            {
//                status = EXIT_255;				
//            }
//            else					
//                cnt = 0;
//            break;
//            
//        case EXIT_255: // 进入锁定
//            cnt = 0;
//            flag.unlock = 0;
//            status = WAITING_1;
//            break;
//            
//        default:
//            status = EXIT_255;
//            break;
//    }
//}
