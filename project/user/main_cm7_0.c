#include "zf_common_headfile.h"

extern uint8_t out_flag;
int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	
    debug_init();                   
  
    
     ALL_Init();

    uint8_t key_val1;
    uint8_t key_val2;
    
    PIDRoll.target = 0;
    PIDPitch.target = 0;
    PIDYaw.target = 0;
    of.K = 530;
    out_flag =0;
    
    while(true)
    {
      if(buff_value!=0&&buff_value<600)
      {
         PIDVelZ.kp = buff_value;
      }
      
      key_scanner();
      key_val1 = key_get_state(KEY_1);
      key_val2 = key_get_state(KEY_2);
      
      if(key_val1 ==1 )
      {
        out_flag = 1;
        key_val1 = 0;
      } 
      else if(key_val2 == 1)
      {
         out_flag = 0;
         key_val2 = 0;    
      }

     
//--------------------vofa------------
        //  printf("%5f,%5f,%5f",world_data.px,world_data.vx,world_data.ax);
     //     printf("%5f,%5f,%5f",of.optical_ang_x,imu_data.gyro_rad_y,world_data.vx);
//          printf("%d",out_flag);
//          printf("\r\n"); 
        
////--------------------IPS----------
        ips200_show_string( 16*0,  16*5, "roll=:");     ips200_show_float(16*6,  16*5, eulerAngle.roll,  3,5);
        ips200_show_string( 16*0,  16*6, "pitch=:" );     ips200_show_float(16*6,  16*6, eulerAngle.pitch,   3,5);
        ips200_show_string( 16*0,  16*7, "yaw=:"  );     ips200_show_float(16*6,  16*7, eulerAngle.yaw, 3,5);
//        
//        ips200_show_string( 16*0,  16*4, "roll=:");     ips200_show_float(16*6,  16*4, imu660rc_roll,  3,5);
//        ips200_show_string( 16*0,  16*5, "pitch=:" );     ips200_show_float(16*6,  16*5, imu660rc_pitch,   3,5);
//        ips200_show_string( 16*0,  16*6, "yaw=:"  );     ips200_show_float(16*6,  16*6, imu660rc_yaw, 3,5);
        
        
        ips200_show_string( 16*0,  16*8, "m1=:");     ips200_show_float(16*6,  16*8, m1, 3,5);
        ips200_show_string( 16*0,  16*9, "m2=:");     ips200_show_float(16*6,  16*9, m2, 3,5);
        ips200_show_string( 16*0,  16*10, "m3=:");     ips200_show_float(16*6,  16*10, m3, 3,5);
        ips200_show_string( 16*0,  16*11, "m4=:");    ips200_show_float(16*6,  16*11, m4, 3,5);
        ips200_show_string( 16*0,  16*12, "test=:");    ips200_show_float(16*6,  16*12, buff_value, 3,5);
        ips200_show_string( 16*0,  16*13, "of.dx=:");    ips200_show_float(16*6,  16*13, of.dx, 3,5);
        ips200_show_string( 16*0,  16*14, "world.vx=:");    ips200_show_float(16*6,  16*14, world_data.vx, 3,5);
        ips200_show_string( 16*0,  16*15, "imu.gx=:");    ips200_show_float(16*6,  16*15, imu_data.gyro_x, 3,5);
        ips200_show_string( 16*0,  16*16, "of.height=:");    ips200_show_float(16*6,  16*16,of.height, 3,5);
        
        system_delay_ms(10);
       

         
    }
}

