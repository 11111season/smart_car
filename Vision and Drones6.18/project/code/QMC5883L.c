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
    
    // 等待数据就绪
//    while(!qmc5883l_check_drdy())
//    {
//        system_delay_ms(1);
//    }
    
    // 读取6个字节的磁场数据
    qmc5883l_read_registers(QMC5883L_REG_DATA_X_LSB, dat, 6);
    
    // 组合原始数据
    qmc5883l_mag_x = (int16)(((uint16)dat[1] << 8) | dat[0]);
    qmc5883l_mag_y = (int16)(((uint16)dat[3] << 8) | dat[2]);
    qmc5883l_mag_z = (int16)(((uint16)dat[5] << 8) | dat[4]);
    
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
    float heading_rad = 0.0f;
    
    // 计算反正切得到角度（弧度）
    // 注意：X轴对应东，Y轴对应北
    heading_rad = atan2f(qmc5883l_mag_y_gauss, qmc5883l_mag_x_gauss);
    
    // 转换为度
    float heading_deg = heading_rad * 180.0f / 3.14159265358979323846f;
    
    // 将角度调整到0-360度范围
    if(heading_deg < 0)
    {
        heading_deg += 360.0f;
    }
    
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
        if(QMC5883L_DEFAULT_CONFIG & QMC5883L_RNG_2G)
        {
            current_sensitivity = QMC5883L_SENSITIVITY_2G;
        }
        else
        {
            current_sensitivity = QMC5883L_SENSITIVITY_8G;
        }
    }
    while(0);
    
    return return_state;
}