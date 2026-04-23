
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
