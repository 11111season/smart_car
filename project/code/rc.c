//#include "zf_common_headfile.h"
//
////解锁
//void remote_unlock(void)
//{
//	volatile static uint8_t status=WAITING_1;
//	static uint16_t cnt=0;
//
//	if(rc.thr<1050 &&rc.yaw<1200)                         //油门遥杆左下角锁定
//	{
//		status = EXIT_255;
//	}
//	
//	switch(status)
//	{
//		case WAITING_1://等待解锁
//			if(rc.thr<1150)           //解锁三步奏，油门最低->油门最高->油门最低 看到LED灯不闪了 即完成解锁
//			{			 
//					 status = WAITING_2;				 
//			}		
//			break;
//		case WAITING_2:
//			if(rc.thr>1600)          
//			{		
//						static uint8_t cnt = 0;
//					 	cnt++;		
//						if(cnt>5) //最高油门需保持200ms以上，防止遥控开机初始化未完成的错误数据
//						{	
//								cnt=0;
//								status = WAITING_3;
//						}
//			}			
//			break;
//		case WAITING_3:
//			if(rc.thr<1100)          
//			{			 
//					 status = WAITING_4;				 
//			}			
//			break;			
//		case WAITING_4:	//解锁前准备	               
//				flag.unlock = 1;
//				status = PROCESS_31;
//				LED.status = AlwaysOn;	
//			  imu_rest();		
//				 break;		
//		case PROCESS_31:	//进入解锁状态
//				if(rc.thr<1020)
//				{
//					if(cnt++ > 3000)                                     // 油门遥杆处于最低9S自动上锁
//					{								
//						status = EXIT_255;								
//					}
//				}
//				else if(!flag.unlock)                           //Other conditions lock 
//				{
//					status = EXIT_255;				
//				}
//				else					
//					cnt = 0;
//			break;
//		case EXIT_255: //进入锁定
//			LED.status = AllFlashLight;	                                 //exit
//			cnt = 0;
//			LED.FlashTime = 100; //100*3ms		
//			flag.unlock = 0;
//			status = WAITING_1;
//			break;
//		default:
//			status = EXIT_255;
//			break;
//	}
//}
//
//
//
//
//
//
//
