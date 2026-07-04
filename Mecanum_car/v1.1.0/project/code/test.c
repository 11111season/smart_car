#include "test.h"

/*****************************************************************静态变量层*****************************************************************/
static uint8_t angle_test_started = 0;
// 添加静态变量记录启动状态
static uint8_t pid_started = 0;
static uint8_t square_started = 0;
static enum { FWD, RIGHT, BACK, LEFT } square_state = FWD;
static uint64_t square_start_time = 0;
static uint8_t figure8_started = 0;
static enum { STAGE0, STAGE1, STAGE2, STAGE3 } figure8_stage = STAGE0;
static uint64_t figure8_start_time = 0;

int change_flag = 0;

/*****************************************************************************
 * @name       : Angle_Test
 * @date       : 2026-01-26
 * @function   : 角度环测试函数
 * @parameters : vx：x目标速度（线速度单位m/s）,vy：y目标速度（线速度单位m/s）
 * @retvalue   : 无
 * @note       : 在主函数循环中调用
******************************************************************************/
void Angle_Test(float vx,float vy)
{
    if(Start_Pid_Flag == 1 && angle_test_started == 0) {
        //My_MPU6050_ResetYaw();
        My_Imu660ra_ResetYaw();
        Motor_Enable_PID(1);
        PID_Reset(&angle_pid_yaw);
        PID_Enable(&angle_pid_yaw, 1);
        target_vx = vx;
        target_vy = vy;
        angle_test_started = 1;
        Start_Pid_Flag = 0;
    }

    if(Stop_Pid_Flag == 1 && angle_test_started == 1) {
        PID_Enable(&angle_pid_yaw, 0);
        Motor_Enable_PID(0);
        Motor_SetSpeed(&motor_L1,0, MotorL1_Turn, MotorL1_Pwm);
        Motor_SetSpeed(&motor_L2,0, MotorL2_Turn, MotorL2_Pwm);
        Motor_SetSpeed(&motor_R1,0, MotorR1_Turn, MotorR1_Pwm);
        Motor_SetSpeed(&motor_R2,0, MotorR2_Turn, MotorR2_Pwm);
        angle_test_started = 0;
        Stop_Pid_Flag = 0;
        target_vx = 0.0f;
        target_vy = 0.0f;
    }
    // 当角度环运行时，打印调试信息（每10ms一次）
    if(angle_test_started) {
       // 打印当前角度、PID输出等
        printf("Yaw=%.1f, Pitch=%.1f, Roll=%.1f, Omega_des=%.1f\n",
               My_Imu660ra_GetYaw(),
               My_Imu660ra_GetPitch(),
               My_Imu660ra_GetRoll(),
               angle_pid_yaw.Output);
    }
}

/*****************************************************************************
 * @name       : Horizon_test
 * @date       : 2026-01-26
 * @function   : 角度环的平移测试函数
 * @parameters : direction：平移方向，1为向右平移，0为向左平移
 * @parameters : target_speed：目标速度，填入线速度单位m/s
 * @retvalue   : 无
 * @note       : 在主函数循环中调用
******************************************************************************/
void Horizon_test(int direction, float target_speed)
{
    if(Start_Pid_Flag == 1 && pid_started == 0) {
        Motor_Enable_PID(1);
        PID_Reset(&angle_pid_yaw);
        PID_Enable(&angle_pid_yaw, 1);
        target_vx = 0.0f;
        target_vy = (direction > 0) ? target_speed : -target_speed;
        pid_started = 1;
        Start_Pid_Flag = 0;
    }

    if(Stop_Pid_Flag == 1 && pid_started == 1) {
        PID_Enable(&angle_pid_yaw, 0);
        Motor_Enable_PID(0);
        Motor_SetSpeed(&motor_L1,0, MotorL1_Turn, MotorL1_Pwm);
        Motor_SetSpeed(&motor_L2,0, MotorL2_Turn, MotorL2_Pwm);
        Motor_SetSpeed(&motor_R1,0, MotorR1_Turn, MotorR1_Pwm);
        Motor_SetSpeed(&motor_R2,0, MotorR2_Turn, MotorR2_Pwm);
        pid_started = 0;
        Stop_Pid_Flag = 0;
        target_vx = 0.0f;
        target_vy = 0.0f;
    }
}

/*****************************************************************************
 * @name       : Square_Test
 * @date       : 2026-01-26
 * @function   : 正方形闭环测试函数
 * @parameters : speed_mps：目标速度，填入线速度单位m/s
 * @parameters : duration_ms：延迟的持续时间（每个动作的持续时间）
 * @retvalue   : 无
 * @note       : 在主函数循环中调用
******************************************************************************/
void Square_Test(float speed_mps, uint32_t duration_ms)
{
    if (Start_Pid_Flag == 1 && square_started == 0) {
        Motor_Enable_PID(1);
        PID_Reset(&angle_pid_yaw);
        PID_Enable(&angle_pid_yaw, 1);
        square_state = FWD;
        square_start_time = 0;
        target_vx = speed_mps;
        target_vy = 0.0f;
        square_started = 1;
        Start_Pid_Flag = 0;
    }

    if (Stop_Pid_Flag == 1 && square_started == 1) {
        PID_Enable(&angle_pid_yaw, 0);
        Motor_Enable_PID(0);
        Motor_SetSpeed(&motor_L1,0, MotorL1_Turn, MotorL1_Pwm);
        Motor_SetSpeed(&motor_L2,0, MotorL2_Turn, MotorL2_Pwm);
        Motor_SetSpeed(&motor_R1,0, MotorR1_Turn, MotorR1_Pwm);
        Motor_SetSpeed(&motor_R2,0, MotorR2_Turn, MotorR2_Pwm);
        square_started = 0;
        Stop_Pid_Flag = 0;
        target_vx = 0.0f;
        target_vy = 0.0f;
    }

    if (!square_started) return;
    if (!angle_pid_yaw.Enable) return;

    uint64_t now_ms = time_us / 1000;

    if (square_start_time == 0) {
        square_start_time = now_ms;
        return;
    }

    if (now_ms - square_start_time >= duration_ms) {
        switch (square_state) {
            case FWD:
                square_state = RIGHT;
                target_vx = 0.0f;
                target_vy = speed_mps;
                break;
            case RIGHT:
                square_state = BACK;
                target_vx = -speed_mps;
                target_vy = 0.0f;
                break;
            case BACK:
                square_state = LEFT;
                target_vx = 0.0f;
                target_vy = -speed_mps;
                break;
            case LEFT:
                square_state = FWD;
                target_vx = speed_mps;
                target_vy = 0.0f;
                break;
        }
        square_start_time = now_ms;
    }
}

/*****************************************************************************
 * @name       : Figure8_Test
 * @date       : 2026-01-26
 * @function   : 8字闭环测试函数
 * @parameters : speed_mps：目标速度，填入线速度单位m/s
 * @parameters : duration_ms：延迟的持续时间（每个动作的持续时间）
 * @retvalue   : 无
 * @note       : 在主函数循环中调用
******************************************************************************/
void Figure8_Test(float speed_mps, uint32_t duration_ms)
{
    if (Start_Pid_Flag == 1 && figure8_started == 0) {
        Motor_Enable_PID(1);
        motor_L1.pid.Enable = 1; motor_L2.pid.Enable = 1;
        motor_R1.pid.Enable = 1; motor_R2.pid.Enable = 1;
        PID_Reset(&angle_pid_yaw);
        PID_Enable(&angle_pid_yaw, 1);
        // 初始：右平移
        target_vx = 0.0f;
        target_vy = speed_mps;
        figure8_stage = STAGE0;
        figure8_start_time = 0;
        figure8_started = 1;
        Start_Pid_Flag = 0;
    }

    if (Stop_Pid_Flag == 1 && figure8_started == 1) {
        PID_Enable(&angle_pid_yaw, 0);
        Motor_Enable_PID(0);
        Motor_SetSpeed(&motor_L1,0, MotorL1_Turn, MotorL1_Pwm);
        Motor_SetSpeed(&motor_L2,0, MotorL2_Turn, MotorL2_Pwm);
        Motor_SetSpeed(&motor_R1,0, MotorR1_Turn, MotorR1_Pwm);
        Motor_SetSpeed(&motor_R2,0, MotorR2_Turn, MotorR2_Pwm);
        figure8_started = 0;
        Stop_Pid_Flag = 0;
        target_vx = 0.0f;
        target_vy = 0.0f;
    }

    if (!figure8_started) return;
    if (!angle_pid_yaw.Enable) return;

    uint64_t now_ms = time_us / 1000;

    if (figure8_start_time == 0) {
        figure8_start_time = now_ms;
        return;
    }

    if (now_ms - figure8_start_time >= duration_ms) {
        switch (figure8_stage) {
            case STAGE0: // 右平移 → 左下45°
                target_vx = -speed_mps;
                target_vy = -speed_mps;
                figure8_stage = STAGE1;
                break;
            case STAGE1: // 左下45° → 右平移
                target_vx = 0.0f;
                target_vy = speed_mps;
                figure8_stage = STAGE2;
                break;
            case STAGE2: // 右平移 → 左上45°
                target_vx = speed_mps;
                target_vy = -speed_mps;
                figure8_stage = STAGE3;
                break;
            case STAGE3: // 左上45° → 右平移（循环）
                target_vx = 0.0f;
                target_vy = speed_mps;
                figure8_stage = STAGE0;
                break;
        }
        figure8_start_time = now_ms;
    }
}

/*****************************************************************************
 * @name       : IMU660RA_Test
 * @date       : 2026-01-26
 * @function   : IMU660RA测试函数
 * @parameters : 无
 * @retvalue   : 无
 * @note       : 在主函数循环中调用
******************************************************************************/
void IMU660RA_Test(void)
{
    imu660ra_get_acc();
    imu660ra_get_gyro();
    // 每100ms打印一次角度
    static uint32_t print_cnt = 0;
    if (print_cnt++ % 10 == 0)
    {
        printf("ax=%d,ay=%d,az=%d\n",
                imu660ra_acc_x,imu660ra_acc_y,imu660ra_acc_z);
        printf("gx=%d,gy=%d,gz=%d\n",imu660ra_gyro_x,imu660ra_gyro_y,imu660ra_gyro_z);
    }
    system_delay_ms(10);
}

/*****************************************************************************
 * @name       : My_IMU660RA_Test
 * @date       : 2026-01-26
 * @function   : My_IMU660RA测试函数
 * @parameters : 无
 * @retvalue   : 无
 * @note       : 在主函数循环中调用
******************************************************************************/
void My_IMU660RA_Test(void)
{
    // 每100ms打印一次角度
    static uint32_t print_cnt = 0;
    if (print_cnt++ % 10 == 0)
    {
        printf("%.1f,%.1f,%.1f\n",
               My_Imu660ra_GetYaw(),
               My_Imu660ra_GetPitch(),
               My_Imu660ra_GetRoll());
    }
    system_delay_ms(10);
}
