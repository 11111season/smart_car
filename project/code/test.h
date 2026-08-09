#ifndef CODE_TEST_H_
#define CODE_TEST_H_

#include "zf_common_headfile.h"
#include "Motor.h"
#include "My_imu660ra.h"
#include "QMC5883L.h"
#include "W25Q64.h"

/*----------------------- 外部变量声明 -----------------------*/
extern volatile int count;
extern int change_flag;
extern volatile uint64_t time;
extern volatile int Start_Pid_Flag;
extern volatile int Stop_Pid_Flag;
extern volatile uint64_t time_us;
/*----------------------- 函数接口声明 -----------------------*/
void Angle_Test(float vx,float vy);
void Square_Test(float speed_mps, uint32_t duration_ms);
void Figure8_Test(float speed_mps, uint32_t duration_ms);
void IMU660RA_Test(void);
void My_IMU660RA_Test(void);
void Mag_Test(void);     // 磁力计 vs 陀螺仪yaw对比打印
void Mag_Yaw_Verify(void);   // 连续打印磁力计/融合/陀螺三路yaw

// 闭环正方形测试
void Square_ClosedLoop_Init(void);   // 一次调用, 初始化状态机和PID
void Square_ClosedLoop_Update(void); // 在10ms ISR中调用, 状态机分步执行

// 闭环八字测试
void Figure8_ClosedLoop_Init(void);
void Figure8_ClosedLoop_Update(void);

// 闭环圆形测试
void Circle_ClosedLoop_Init(float radius_m, int dir);   // dir: 1=逆时针, -1=顺时针
void Circle_ClosedLoop_Update(void);

// W25Q64 掉电存储测试通路: 读ID + 扇区擦除 + 写读回比对, 结果打印到 UART_0
void W25Q64_Test(void);

// W25Q64 掉电持久化测试: 写"Hello W25Q64!" → 掉电 → 改宏重烧 → 读取验证掉电不丢数据
// 模式切换在 test.c 内 W25Q64_PERSIST_WRITE 宏 (1=写, 0=读)
void W25Q64_Persist_Test(void);

// 零漂测量实验: 悬空跑惯导, CSV 打印原始陀螺漂移 (需 cm7_0_isr.c DRIFT_LEARN_ENABLE=0)
// 主循环连续调用, 内部门控于 mission_armed + 100ms 限频
void Drift_Measure_Test(void);

// 零漂测量实验: KEY_1 绕过菜单直接发车 (从 W25Q64 重读地图 + Inav_Launch)
// 主循环连续调用, 发车后 mission_armed=1 自动忽略后续按键
void Test_QuickLaunch(void);

#endif /* CODE_TEST_H_ */