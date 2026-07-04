# 智能无人机飞控项目 Code Wiki

## 目录
1. [项目概述](#项目概述)
2. [项目架构](#项目架构)
3. [主要模块说明](#主要模块说明)
4. [核心函数说明](#核心函数说明)
5. [依赖关系](#依赖关系)
6. [运行方式](#运行方式)
7. [配置说明](#配置说明)

---

## 项目概述

### 项目简介
这是一个基于英飞凌(Infineon) Traveo II MCU的四旋翼无人机飞控系统，采用X型四旋翼布局。项目使用智飞科技的开发框架，实现了完整的无人机姿态控制、高度控制、位置保持和自动起降功能。

### 核心功能
- **姿态稳定控制** - 串级PID控制（角度外环+角速度内环）
- **高度控制** - 基于TOF激光测距的定高飞行
- **位置保持** - 基于光流传感器的定点悬停
- **自动起降** - 一键起飞和自动降落
- **磁力计融合** - 起飞后自动启用磁力计融合修正航向
- **Anti-Gravity** - 油门前馈补偿，防止急加速掉头

### 硬件平台
- **主控芯片**: 英飞凌Traveo II (CYT4BB系列)
- **姿态传感器**: IMU660RC (陀螺仪+加速度计)
- **磁力计**: QMC5883L
- **高度传感器**: TOF激光测距模块
- **位置传感器**: PMW3901光流传感器
- **显示**: IPS200屏幕
- **电机驱动**: 小型驱动模块（UART通信）

---

## 项目架构

### 目录结构
```
smart_car-main/
├── libraries/              # 库文件目录
│   ├── sdk/               # 英飞凌SDK
│   ├── zf_driver/         # 智飞驱动库
│   ├── zf_device/         # 智飞设备库
│   ├── zf_common/         # 智飞通用库
│   └── zf_components/     # 智飞组件库
├── project/               # 项目代码目录
│   ├── code/             # 核心控制代码
│   ├── user/             # 用户代码
│   └── iar/              # IAR工程文件
└── Code_Wiki.md          # 本文档
```

### 软件架构
```
┌─────────────────────────────────────────┐
│         应用层 (User Application)        │
│  main_cm7_0.c / main_cm7_1.c             │
└─────────────────────────────────────────┘
                  ↓
┌─────────────────────────────────────────┐
│         控制层 (Control Layer)           │
│  flight_control / stabilization          │
│  take_off / land / pos_hold_control     │
└─────────────────────────────────────────┘
                  ↓
┌─────────────────────────────────────────┐
│         算法层 (Algorithm Layer)         │
│  PID控制 / 滤波器 / 姿态融合             │
│  Mahony滤波 / 光流融合                   │
└─────────────────────────────────────────┘
                  ↓
┌─────────────────────────────────────────┐
│         驱动层 (Driver Layer)            │
│  IMU / TOF / 光流 / 磁力计 / 电机        │
└─────────────────────────────────────────┘
                  ↓
┌─────────────────────────────────────────┐
│         硬件抽象层 (HAL)                 │
│  智飞驱动库 / 英飞凌SDK                  │
└─────────────────────────────────────────┘
```

### 控制环路架构

| 控制环路       | 频率       | 控制器         | 说明                      |
|----------------|------------|----------------|---------------------------|
| 角速度内环     | 200~500Hz  | PIDVelX/Y/Z    | Roll/Pitch/Yaw角速度控制  |
| 姿态外环       | 100~200Hz  | PIDRoll/Pitch/Yaw | Roll/Pitch/Yaw角度控制 |
| 高度外环       | 50~100Hz   | PIDHeight      | 高度→目标垂直速度         |
| 高度内环       | 50~100Hz   | PIDVelH        | 垂直速度→油门修正         |
| 位置外环       | 50Hz       | PIDPosX/Y      | 位置→目标水平速度         |
| 位置内环       | 50Hz       | PIDPosX_Vel/Y_Vel | 水平速度→目标角度     |

---

## 主要模块说明

### 1. 主程序模块 ([main_cm7_0.c](file:///d:/UsersASUSDesktop/smart_car-main/smart_car-main/project/user/main_cm7_0.c))

**功能**: 系统入口，初始化、主循环

**主要流程**:
1. 时钟初始化 (250MHz)
2. 调试初始化
3. 全局初始化 (`ALL_Init()`)
4. 设置PID目标值
5. 主循环：
   - 按键检测处理
   - 电机控制
   - VOFA+调试输出
   - IPS屏幕显示

---

### 2. 初始化模块 ([INIT.c](file:///d:/UsersASUSDesktop/smart_car-main/smart_car-main/project/code/INIT.c), [INIT.h](file:///d:/UsersASUSDesktop/smart_car-main/smart_car-main/project/code/INIT.h))

**功能**: 系统各模块初始化

**主要初始化内容**:
- 按键初始化
- IPS屏幕初始化
- IMU660RC初始化 (240Hz四元数输出)
- 电机驱动初始化 (UART通信)
- PID参数初始化
- 磁力计QMC5883L初始化
- TOF激光测距初始化
- 串口初始化
- 光流传感器初始化
- 滤波器初始化
- PIT定时器初始化

**关键定时器配置**:
- PIT_CH0: 5ms (姿态控制)
- PIT_CH1: 5ms 
- PIT_CH2: 20ms
- PIT_CH10: 10ms
- PIT_CH11: 25ms
- PIT_CH12: 10ms (遥控器)

**数据结构**:

#### PID结构体
```c
typedef struct {
    float target;          // 目标值
    float error;           // 误差
    float last_error;      // 上次误差
    float integ;           // 积分项
    float deriv;           // 微分项
    float last_deriv;      // 上次微分项
    float measured;        // 测量值
    float last_measured;   // 上次测量值
    float kp;              // 比例系数
    float ki;              // 积分系数
    float kd;              // 微分系数
    float out;             // 输出
    float Integ_LimitHigh; // 积分上限
    float Integ_LimitLow;  // 积分下限
    float Out_LimitHigh;   // 输出上限
    float Out_LimitLow;    // 输出下限
    PT1Filter_t deriv_filter; // 微分滤波器
} _PID_param_st;
```

#### IMU数据结构体
```c
typedef volatile struct {
    float gyro_x;       // 陀螺仪X轴 (°/s)
    float gyro_y;       // 陀螺仪Y轴 (°/s)
    float gyro_z;       // 陀螺仪Z轴 (°/s)
    float acc_x;        // 加速度计X轴
    float acc_y;        // 加速度计Y轴
    float acc_z;        // 加速度计Z轴
    float q0, q1, q2, q3; // 四元数
} _imu_param_st;
```

#### 欧拉角结构体
```c
typedef volatile struct {
    float pitch;    // 俯仰角 (°)
    float roll;     // 横滚角 (°)
    float yaw;      // 偏航角 (°)
} _euler_param_st;
```

#### 世界坐标系数据
```c
typedef struct {
    float vx;    // 世界坐标系X轴速度 (m/s)
    float vy;    // 世界坐标系Y轴速度 (m/s)
    float vz;    // 世界坐标系Z轴速度 (m/s)
    float ax;    // 世界坐标系X轴加速度
    float ay;    // 世界坐标系Y轴加速度
    float az;    // 世界坐标系Z轴加速度
    float px;    // 世界坐标系X轴位置 (m)
    float py;    // 世界坐标系Y轴位置 (m)
    float pz;    // 世界坐标系Z轴位置 (m)
} _world_param_st;
```

#### 飞行状态枚举
```c
typedef enum {
    STATE_LOCK,      // 上锁状态
    STATE_UNLOCK,    // 解锁状态
    STATE_IDLE,      // 空闲状态
    STATE_TAKEOFF,   // 起飞状态
    STATE_HOVER,     // 悬停状态
    STATE_TASK,      // 任务状态
    STATE_FLY,       // 飞行状态
    STATE_LAND,      // 降落状态
    STATE_EMERGENCY  // 紧急状态
} flight_state_e;
```

---

### 3. 控制模块 ([control.c](file:///d:/UsersASUSDesktop/smart_car-main/smart_car-main/project/code/control.c), [control.h](file:///d:/UsersASUSDesktop/smart_car-main/smart_car-main/project/code/control.h))

**功能**: 飞行控制核心，包含飞行状态机和姿态控制

#### 电机布局 (X型四旋翼，俯视图)
```
    1     2
        X
    4     3
```
- 电机1: 右前 (CCW, 逆时针)
- 电机2: 左前 (CW,  顺时针)
- 电机3: 左后 (CCW, 逆时针)
- 电机4: 右后 (CW,  顺时针)

#### 电机混控公式
```c
m1 = THR + PIDVelX.out - PIDVelY.out - PIDVelZ.out;  // 右前
m2 = THR - PIDVelX.out - PIDVelY.out + PIDVelZ.out;  // 左前
m3 = THR - PIDVelX.out + PIDVelY.out - PIDVelZ.out;  // 左后
m4 = THR + PIDVelX.out + PIDVelY.out + PIDVelZ.out;  // 右后
```

#### 姿态修正方向
- **右倾 roll > 0**: 左侧电机(2,3)加速，右侧电机(1,4)减速
- **前倾 pitch > 0**: 后方电机(3,4)加速，前方电机(1,2)减速
- **逆时针 yaw > 0**: CCW电机(1,3)加速，CW电机(2,4)减速

#### 核心函数

##### `flight_control(float dt)`
- **功能**: 飞行状态机顶层调度
- **调用频率**: 200Hz (由PIT定时中断触发)
- **状态流转**:
  ```
  LOCK → (解锁/按键1) → TAKEOFF → FLY → (降落) → LAND → (落地) → LOCK
  ```

##### `stabilization(float dt)`
- **功能**: 姿态稳定控制（串级PID+电机混控）
- **流程**:
  1. Anti-Gravity油门前馈更新
  2. 高度外环+内环控制
  3. 位置外环+内环控制
  4. Roll轴角度环+角速度环
  5. Pitch轴角度环+角速度环
  6. Yaw轴角度环+角速度环
  7. 电机混控输出

##### `take_off(float dt)`
- **功能**: 自动起飞控制
- **步骤**:
  1. 初始化：锁当前偏航，禁用磁力计融合
  2. 离地阶段：缓慢上升至0.15m，保持水平
  3. 上升阶段：加速上升至目标高度，检测磁力计融合条件
  4. 完成：转入定点悬停模式

##### `land(float dt)`
- **功能**: 自动降落控制
- **策略**:
  1. 高度目标以0.3m/s速度下降
  2. Roll/Pitch保持水平
  3. Yaw锁定当前方向
  4. 近地保护(<15cm)：减弱控制防止弹跳
  5. 落地检测：低高度+低速度持续0.5秒自动上锁

##### `check_mag_switch(float dt)`
- **功能**: 检测是否启用磁力计融合
- **启用条件**:
  1. 高度 > 0.5m (TAKEOFF_ALT_SWITCH)
  2. 三轴角速度 < 30°/s (姿态稳定)
  3. 稳定时间 > 2秒 (TAKEOFF_STABLE_TIME)

---

### 4. PID控制模块 ([PID.c](file:///d:/UsersASUSDesktop/smart_car-main/smart_car-main/project/code/PID.c), [PID.h](file:///d:/UsersASUSDesktop/smart_car-main/smart_car-main/project/code/PID.h))

**功能**: PID控制器实现

#### PID参数初始化

##### 内环角速度PID
| 控制器 | Kp | Ki | Kd | 积分限幅 | 输出限幅 |
|--------|----|----|----|----------|----------|
| PIDVelX (Roll) | 10.0 | 4.0 | 1.0 | ±800 | ±2000 |
| PIDVelY (Pitch) | 10.0 | 4.0 | 1.0 | ±800 | ±2000 |
| PIDVelZ (Yaw) | 9.0 | 1.0 | 1.0 | ±800 | ±1500 |

##### 外环角度PID
| 控制器 | Kp | Ki | Kd | 积分限幅 | 输出限幅 |
|--------|----|----|----|----------|----------|
| PIDRoll | 5.5 | 0 | 0 | 0 | ±200 |
| PIDPitch | 5.5 | 0 | 0 | 0 | ±200 |
| PIDYaw | 4.0 | 0 | 0 | 0 | ±150 |

##### 高度环PID
| 控制器 | Kp | Ki | Kd | 积分限幅 | 输出限幅 |
|--------|----|----|----|----------|----------|
| PIDHeight (外环) | 1.0 | 0.5 | 0.5 | ±100 | ±0.3 |
| PIDVelH (内环) | 1600 | 20 | 0 | ±300 | ±800 |

##### 位置环PID
| 控制器 | Kp | Ki | Kd | 积分限幅 | 输出限幅 |
|--------|----|----|----|----------|----------|
| PIDPosX/Y (外环) | 1.0 | 0 | 0 | ±50 | ±200 |
| PIDPosX_Vel/Y_Vel (内环) | 2.0 | 0.1 | 0 | ±100 | ±10 |

#### 核心函数

##### `PID_Update(_PID_param_st *pid, float target, float measured, float dt)`
- **用途**: 误差微分PID（适用于内环角速度控制）
- **特点**: 
  - 误差微分
  - Anti-Gravity积分项增益
  - PT1滤波器平滑微分项

##### `PID_Update_d_measure(_PID_param_st *pid, float target, float measured, float dt)`
- **用途**: 测量值微分PID（适用于外环角度控制）
- **特点**:
  - 测量值微分（微分先行，对噪声不敏感）
  - PT1滤波器平滑微分项

##### `PID_Update_Yaw(_PID_param_st *pid, float target, float measured, float dt)`
- **用途**: Yaw专用PID（处理角度环绕问题）
- **特点**: 自动计算最短路径角度误差（±180°范围）

##### `anti_gravity_update(float throttle, float dt)`
- **功能**: Anti-Gravity油门前馈
- **原理**: 
  1. 油门变化率检测
  2. 油门越大，增益越小（inv = 1 - throttle）
  3. 油门增加时额外增益
  4. PT1滤波平滑
- **效果**: 急加速时临时放大内环I项，防止掉头

---

### 5. 滤波器模块 ([filter.c](file:///d:/UsersASUSDesktop/smart_car-main/smart_car-main/project/code/filter.c), [filter.h](file:///d:/UsersASUSDesktop/smart_car-main/smart_car-main/project/code/filter.h))

**功能**: 各类数字滤波器实现

#### PT1一阶低通滤波器
```c
typedef struct {
    float tau;          // 时间常数
    float last_output;  // 上次输出
    float dt;           // 采样周期
} PT1Filter_t;
```

**核心公式**:
```
output = (input * dt + last_output * tau) / (tau + dt)
```

**初始化方法**:
- `PT1Filter_Init()`: 直接设置时间常数tau
- `PT1Filter_InitWithFreq()`: 通过截止频率fc初始化（推荐）

#### PT2二阶低通滤波器
```c
typedef struct {
    float cutoffHz;     // 截止频率
    float sampleRateHz; // 采样频率
    float b0, b1, b2;   // 前向系数
    float a1, a2;       // 反馈系数
    float x1, x2;       // 输入历史
    float y1, y2;       // 输出历史
} pt2Filter_t;
```

#### 卡尔曼滤波器
```c
typedef struct {
    float q;  // 过程噪声协方差
    float r;  // 测量噪声协方差
    float x;  // 状态估计值
    float p;  // 估计误差协方差
    float k;  // 卡尔曼增益
} KalmanFilter;
```

#### 全局滤波器实例
| 滤波器 | 截止频率 | 用途 |
|--------|----------|------|
| filter_height | 100Hz | 高度数据滤波 |
| filter_height_vz | 2Hz | 垂直速度滤波 |
| filter_pwm3901_vx | 10Hz | 光流X速度滤波 |
| filter_pwm3901_vy | 10Hz | 光流Y速度滤波 |

---

### 6. 姿态传感器模块 ([IMU.h](file:///d:/UsersASUSDesktop/smart_car-main/smart_car-main/project/code/IMU.h))

**功能**: IMU数据读取和姿态融合

####