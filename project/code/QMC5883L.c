/*********************************************************************************************************************
* CYT4BB Opensourec Library 基于逐飞库接口封装的QMC5883L磁力计驱动
* Copyright (c) 2026 YourName
*
* 文件名称   zf_driver_qmc5883l
* 版本信息   v1.0.0
* 开发环境   IAR 9.40.1
* 适用平台  CYT4BB
*
* 修改记录
* 日期        作者        备注
* 2026-2-17   YourName    first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "QMC5883L.h"
#include "motor.h"          // Motor_SetSpeed, motor_L1..R2
#include "My_imu660ra.h"    // My_Imu660ra_GetYaw
#include "inertial_nav.h"   // fused_yaw
#include "key_task.h"       // MAG_CALIB_MODE / MAG_CALIB_MOTOR_TEST

// ====================================================全局变量定义====================================================
// 定义全局变量
int16 qmc5883l_mag_x = 0;          // X轴磁场强度原始值
int16 qmc5883l_mag_y = 0;          // Y轴磁场强度原始值
int16 qmc5883l_mag_z = 0;          // Z轴磁场强度原始值
float qmc5883l_mag_x_gauss = 0.0f; // X轴磁场强度（高斯）
float qmc5883l_mag_y_gauss = 0.0f; // Y轴磁场强度（高斯）
float qmc5883l_mag_z_gauss = 0.0f; // Z轴磁场强度（高斯）
float qmc5883l_temperature = 0.0f; // 温度值（℃）
float qmc5883l_heading = 0.0f;     // 航向角（度，0-360）
uint32_t qmc5883l_mag_reject_cnt = 0;  // 错位读被拒次数 (调试用)

// ====================================================内部变量====================================================
static soft_iic_info_struct qmc5883l_iic_struct;  // 软件IIC对象
static float current_sensitivity = QMC5883L_SENSITIVITY_2G; // 当前灵敏度

// ====================================================内部函数====================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     QMC5883L 写寄存器
// 参数说明     reg         寄存器地址
// 参数说明     data        寄存器数据
// 返回参数     void
// 使用示例     qmc5883l_write_register(QMC5883L_REG_CONTROL_1, 0x0D);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void qmc5883l_write_register(uint8 reg, uint8 data)
{
    soft_iic_write_8bit_register(&qmc5883l_iic_struct, reg, data);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     QMC5883L 写数据
// 参数说明     reg         寄存器地址
// 参数说明     data        数据
// 参数说明     len         数据长度
// 返回参数     void
// 使用示例     qmc5883l_write_registers(QMC5883L_REG_CONTROL_1, config_data, 2);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void qmc5883l_write_registers(uint8 reg, const uint8 *data, uint32 len)
{
    soft_iic_write_8bit_registers(&qmc5883l_iic_struct, reg, data, len);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     QMC5883L 读寄存器
// 参数说明     reg         寄存器地址
// 返回参数     uint8       读取的数据
// 使用示例     uint8 chip_id = qmc5883l_read_register(QMC5883L_REG_CHIP_ID);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8 qmc5883l_read_register(uint8 reg)
{
    return soft_iic_read_8bit_register(&qmc5883l_iic_struct, reg);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     QMC5883L 读数据
// 参数说明     reg         寄存器地址
// 参数说明     data        数据缓冲区
// 参数说明     len         数据长度
// 返回参数     void
// 使用示例     uint8 buffer[6]; qmc5883l_read_registers(QMC5883L_REG_DATA_X_LSB, buffer, 6);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void qmc5883l_read_registers(uint8 reg, uint8 *data, uint32 len)
{
    soft_iic_read_8bit_registers(&qmc5883l_iic_struct, reg, data, len);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     QMC5883L 自检
// 参数说明     void
// 返回参数     uint8       1-自检失败 0-自检成功
// 使用示例     qmc5883l_self_check();
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8 qmc5883l_self_check(void)
{
    uint8 dat = 0, return_state = 0;
    uint16 timeout_count = 0;
    
    do
    {
        if(timeout_count++ > QMC5883L_TIMEOUT_COUNT)
        {
            return_state = 1;
            break;
        }
        
        dat = qmc5883l_read_register(QMC5883L_REG_CHIP_ID);
        system_delay_ms(1);
    }
    while(QMC5883L_CHIP_ID != dat);  // 读取设备ID是否等于0xFF，如果不是则认为没检测到设备
    
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检查数据就绪
// 参数说明     void
// 返回参数     uint8       1-数据就绪 0-数据未就绪
// 使用示例     if(qmc5883l_check_drdy()) { /* 可以读取数据 */ }
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8 qmc5883l_check_drdy(void)
{
    uint8 status = qmc5883l_read_register(QMC5883L_REG_STATUS);
    return (status & QMC5883L_STATUS_DRDY) ? 1 : 0;
}

// ====================================================API函数实现====================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取 QMC5883L 磁场数据
//-------------------------------------------------------------------------------------------------------------------
void qmc5883l_get_mag(void)
{
    uint8 dat[6];
    int16 mx, my, mz;
    // 上次有效值 (错位读校验用)
    static int16 prev_mx = 0, prev_my = 0, prev_mz = 0;
    static uint8 mag_valid_init = 0;   // 基线已建立(首读合理后置1)

    // 等待数据就绪
//    while(!qmc5883l_check_drdy())
//    {
//        system_delay_ms(1);
//    }

    // 软件I2C时序敏感: 关闭中断防止位时序被打断(否则读到错位字节→跳变)
    // 与IMU(My_imu660ra.c)同样的保护方式
    interrupt_global_disable();

    // 读取6个字节的磁场数据
    qmc5883l_read_registers(QMC5883L_REG_DATA_X_LSB, dat, 6);

    interrupt_global_enable(0);

    // 组合原始数据
    mx = (int16)(((uint16)dat[1] << 8) | dat[0]);
    my = (int16)(((uint16)dat[3] << 8) | dat[2]);
    mz = (int16)(((uint16)dat[5] << 8) | dat[4]);

#if MAG_CALIB_MODE && MAG_CALIB_MOTOR_TEST == 0
    // ---- 椭圆标定模式: 错位读窗口关闭, 原始数据直通 ----
    // 2026-08-09 架高后硬铁全变, 旧硬铁中心化幅度必越窗(冻结prev→标定数据污染)
    // 标定只需要原始mx/my, 不需要窗口保护; 硬铁中心由上位机椭圆拟合重新求出
#else
    // ---- 错位读校验: 中心化幅度校验 (唯一判据) ----
    // 去硬铁中心后磁场幅度应≈3450counts(2G实测, 磁力计绕Z转物理约束)
    // 错位读垃圾值(0/265/2558/±32512)去偏后幅度≥9000 → 必判垃圾
    // 真实持续偏移(如带载大电流)幅度仍落在窗口内 → 不会误拦
    // 首读(0,0)幅度24535 → 判垃圾不立基线 → 修复之前被污染卡死bug
    float cxm = (float)mx - QMC5883L_HARD_IRON_X;
    float cym = (float)my - QMC5883L_HARD_IRON_Y;
    float mag2 = cxm*cxm + cym*cym;
    if ((mag2 > QMC5883L_MAG_MIN_ABS2) && (mag2 < QMC5883L_MAG_MAX_ABS2)) {
        // 合理读数: 更新基线 (首读合理即立基线)
        prev_mx = mx;
        prev_my = my;
        prev_mz = mz;
        mag_valid_init = 1;
    } else if (mag_valid_init) {
        // 垃圾读数: 沿用上次有效值 (不更新基线)
        mx = prev_mx;
        my = prev_my;
        mz = prev_mz;
        qmc5883l_mag_reject_cnt++;
    }
    // 首读即垃圾(罕见): 不立基线, 输出本帧, 下一帧合理即可恢复
#endif

    // 写入全局
    qmc5883l_mag_x = mx;
    qmc5883l_mag_y = my;
    qmc5883l_mag_z = mz;

    // 转换为高斯单位
    qmc5883l_mag_x_gauss = qmc5883l_mag_transition(qmc5883l_mag_x);
    qmc5883l_mag_y_gauss = qmc5883l_mag_transition(qmc5883l_mag_y);
    qmc5883l_mag_z_gauss = qmc5883l_mag_transition(qmc5883l_mag_z);

//    qmc5883l_mag_x_gauss = PT1Filter_Apply(&filter_compass,qmc5883l_mag_x_gauss);
//    qmc5883l_mag_y_gauss = PT1Filter_Apply(&filter_compass,qmc5883l_mag_y_gauss);
//    qmc5883l_mag_z_gauss = PT1Filter_Apply(&filter_compass,qmc5883l_mag_z_gauss);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取 QMC5883L 温度数据
//-------------------------------------------------------------------------------------------------------------------
void qmc5883l_get_temp(void)
{
    uint8 temp_buffer[2];
    
    qmc5883l_read_registers(QMC5883L_REG_TEMP_LSB, temp_buffer, 2);
    
    int16 temp_raw = (int16)(((uint16)temp_buffer[1] << 8) | temp_buffer[0]);
    qmc5883l_temperature = temp_raw / QMC5883L_TEMP_COEFF;  // 转换为摄氏度
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取 QMC5883L 所有数据
//-------------------------------------------------------------------------------------------------------------------
void qmc5883l_get_all(void)
{
    qmc5883l_get_mag();      // 获取磁场数据
    qmc5883l_get_temp();     // 获取温度数据
    qmc5883l_heading = qmc5883l_calculate_heading();  // 计算航向角
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 QMC5883L 磁场数据转换为实际物理数据
//-------------------------------------------------------------------------------------------------------------------
float qmc5883l_mag_transition(int16 mag_value)
{
    return (float)mag_value / current_sensitivity;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算航向角
//-------------------------------------------------------------------------------------------------------------------
float qmc5883l_calculate_heading(void)
{
    float mx = (float)(qmc5883l_mag_x - QMC5883L_HARD_IRON_X);
    float my = (float)(qmc5883l_mag_y - QMC5883L_HARD_IRON_Y);

    // 软铁修正 (2026-08-09 架高标定): 逆旋转 -θ 把长轴转回X轴, 再缩放 y/r 成圆
    // θ=77.9° (长轴方向), r=a/b=1.091
    float mxr = mx*QMC5883L_SOFT_IRON_COS + my*QMC5883L_SOFT_IRON_SIN;
    float myr = -mx*QMC5883L_SOFT_IRON_SIN + my*QMC5883L_SOFT_IRON_COS;
    mx = mxr;
    my = myr / QMC5883L_SOFT_IRON_RATIO;

    float heading_rad = atan2f(my, mx);
    float heading_deg = heading_rad * 57.29578f;
    if (heading_deg < 0.0f) heading_deg += 360.0f;

    return heading_deg;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 QMC5883L
//-------------------------------------------------------------------------------------------------------------------
uint8 qmc5883l_init(void)
{
    uint8 return_state = 0;
    
    system_delay_ms(20);  // 等待设备上电成功
    
    // 初始化软件IIC
    soft_iic_init(&qmc5883l_iic_struct, QMC5883L_DEV_ADDR, 
                  QMC5883L_SOFT_IIC_DELAY, 
                  QMC5883L_SCL_PIN, 
                  QMC5883L_SDA_PIN);
    
    do
    {
        if(qmc5883l_self_check())  // QMC5883L 自检
        {
            // 如果程序在输出了断言信息并且提示出错位置在这里
            // 那么就是 QMC5883L 自检出错并超时退出了
            // 检查一下接线有没有问题，如果没问题可能就是坏了
            zf_log(0, "qmc5883l self check error.");
            return_state = 1;
            break;
        }
        
        // 写入SET/RESET周期寄存器（建议值0x01）
        qmc5883l_write_register(QMC5883L_REG_SET_RESET, 0x01);
        
        // 写入控制寄存器1，使用默认配置
        qmc5883l_write_register(QMC5883L_REG_CONTROL_1, QMC5883L_DEFAULT_CONFIG);
        
        // 设置控制寄存器2，禁用中断
        qmc5883l_write_register(QMC5883L_REG_CONTROL_2, 0x01);
        
        // 根据选择的量程设置灵敏度
        // 注意: RNG_2G=0x00, 用 &RNG_2G 判断永远为假, 必须查 8G 位
        if(QMC5883L_DEFAULT_CONFIG & QMC5883L_RNG_8G)
        {
            current_sensitivity = QMC5883L_SENSITIVITY_8G;
        }
        else
        {
            current_sensitivity = QMC5883L_SENSITIVITY_2G;
        }
    }
    while(0);

    return return_state;
}

/*****************************************************************************
 * @name       : qmc5883l_calibration_key_handler
 * @date       : 2026-08-06
 * @function   : 磁力计椭圆校准采集: KEY4按下开始/结束
 *               开始记录后, 需调用 qmc5883l_calibration_collect() 每10ms采集一次
 *               采集期间慢速匀速自转≥2圈, 结束后用上位机对全量数据做椭圆拟合
 * @parameters : 无
 * @retvalue   : 无
 * @note       : 本函数不负责椭圆拟合, 只负责原始数据采集与发送(debug串口)
******************************************************************************/
static uint8_t qmc_cal_record = 0;   // 1=记录中

void qmc5883l_calibration_key_handler(void)
{
    if (key_get_state(KEY_4) != KEY_SHORT_PRESS) return;

    qmc_cal_record = !qmc_cal_record;
    key_clear_state(KEY_4);

    if (qmc_cal_record) {
        printf("MAGCAL: start, rotate slowly >=2 turns\n");
    } else {
        printf("MAGCAL: end\n");
    }
}

// 在 ISR 10ms 里调用: 记录期间每帧打印 mag_x,mag_y 原始值
// 格式: x,y (每行一条, 上位机直接读)
void qmc5883l_calibration_collect(void)
{
    if (!qmc_cal_record) return;
    printf("%d,%d\n", qmc5883l_mag_x, qmc5883l_mag_y);
}

uint8_t qmc5883l_calibration_is_recording(void)
{
    return qmc_cal_record;
}

/*****************************************************************************
 * @name       : 电机磁场干扰测试 (PWM → 磁力计偏移)
 * @date       : 2026-08-06
 * @function   : KEY4触发, 单电机 PWM 从0缓慢增加到指定值, 记录磁力计偏移
 *               用于评估电机电流产生的磁场干扰, 决定是否需要动态补偿
 * @note       : 测试时四轮架起(悬空), 电机逐个测
 *               KEY4: 开始/停止  (与椭圆校准KEY4冲突, 编译时二选一)
 *               ISR只做缓变(轻量), 打印放主循环(task函数)避免卡中断
******************************************************************************/
// 测试的电机: 0=L1左前 1=L2左后 2=R1右前 3=R2右后
#define MOTOR_TEST_INDEX   0
// 扫频上限: 0→MOTOR_TEST_MAX_DUTY 逐+1 (PWM值, PWM_DUTY_MAX=1000 → 百分比=值/10)
// 300 = 30%占空比. 每30ms +1 → 全程9秒
// 关键: 累加只在 KEY4 按下后(motor_test_run=1)才开始, 上电不累加, 避免直接冲到300
#define MOTOR_TEST_MAX_DUTY   300

static uint8_t motor_test_run = 0;
static uint8_t motor_test_tick_div = 0;  // 30ms分频: 每3个10ms节拍 duty +1
static int motor_test_duty = 0;          // 当前扫频PWM (仅在run后从0累加)
static motor_t *motor_test_ptr = NULL;
static gpio_pin_enum motor_test_turn;
static pwm_channel_enum motor_test_pwm;
static volatile uint8_t motor_test_tick = 0;   // ISR置1, 主循环消费

void qmc5883l_motor_test_key_handler(void)
{
    if (key_get_state(KEY_4) != KEY_SHORT_PRESS) return;
    key_clear_state(KEY_4);

    if (!motor_test_run) {
        // 开始: 选电机, 禁用PID闭环(避免ISR中Motor_PID_Control_All覆盖开环PWM)
        Motor_Enable_PID(0);        // 关闭环, 让开环PWM生效
        PID_Enable(&angle_pid_yaw, 0);
        PID_Enable(&angle_pid_gyro, 0);
        motor_test_duty = 0;        // 从0开始扫频 (KEY4后才累加, 上电不累加)
        motor_test_tick_div = 0;    // 30ms分频清零
        switch (MOTOR_TEST_INDEX) {
            case 0: motor_test_ptr = &motor_L1; motor_test_turn = MotorL1_Turn; motor_test_pwm = MotorL1_Pwm; break;
            case 1: motor_test_ptr = &motor_L2; motor_test_turn = MotorL2_Turn; motor_test_pwm = MotorL2_Pwm; break;
            case 2: motor_test_ptr = &motor_R1; motor_test_turn = MotorR1_Turn; motor_test_pwm = MotorR1_Pwm; break;
            default: motor_test_ptr = &motor_R2; motor_test_turn = MotorR2_Turn; motor_test_pwm = MotorR2_Pwm; break;
        }
        Motor_SetSpeed(motor_test_ptr, 0, motor_test_turn, motor_test_pwm);
        motor_test_run = 1;
        printf("MAGT: motor%d start, sweep 0->%d (PID off)\n", MOTOR_TEST_INDEX, MOTOR_TEST_MAX_DUTY);
    } else {
        // 停止: 关电机 + 恢复PID
        Motor_SetSpeed(motor_test_ptr, 0, motor_test_turn, motor_test_pwm);
        motor_test_run = 0;
        Motor_Enable_PID(1);        // 恢复闭环
        PID_Reset(&angle_pid_yaw);   PID_Enable(&angle_pid_yaw, 1);
        PID_Reset(&angle_pid_gyro);  PID_Enable(&angle_pid_gyro, 1);
        printf("MAGT: stop (PID restored)\n");
    }
}

// ISR 10ms: 只置标志, 分档执行全放主循环
void qmc5883l_motor_test_collect(void)
{
    if (motor_test_run) motor_test_tick = 1;
}

// 主循环调用: 每10ms节拍, 扫频 +1 执行
// 关键: 只在 motor_test_run=1(KEY4按下)后累加, 上电不累加
void qmc5883l_motor_test_task(void)
{
    if (!motor_test_tick) return;
    motor_test_tick = 0;
    if (!motor_test_run) return;

    // 记录: pwm, mag_x, mag_y, 融合yaw, 陀螺yaw, 四轮编码器速度(ω)
    // ω用于电流估计 I≈a·PWM−b·ω 拟合 (无载高ω→电流小, 堵转ω≈0→电流大, 磁场=Σk·I)
    // 4个spd列与 mag_motor_test.py 的9列格式对齐 (spd_l1,l2,r1,r2)
    printf("%d,%d,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n",
           motor_test_duty, qmc5883l_mag_x, qmc5883l_mag_y,
           fused_yaw, My_Imu660ra_GetYaw(),
           motor_L1.encoder_speed, motor_L2.encoder_speed,
           motor_R1.encoder_speed, motor_R2.encoder_speed);

    // 扫频: 每3个10ms(30ms) +1 → 0到300, 全程9秒, 到300保持等KEY4 (期间可堵轮测带载)
    if (motor_test_duty < MOTOR_TEST_MAX_DUTY) {
        if (++motor_test_tick_div < 3) return;   // 30ms分频: 前2拍只打印不累加
        motor_test_tick_div = 0;
        motor_test_duty++;
        Motor_SetSpeed(motor_test_ptr, motor_test_duty, motor_test_turn, motor_test_pwm);
        if (motor_test_duty == MOTOR_TEST_MAX_DUTY) {
            printf("MAGT: reached %d (30%%), holding. Block wheel to test load.\n", MOTOR_TEST_MAX_DUTY);
        }
    }
}