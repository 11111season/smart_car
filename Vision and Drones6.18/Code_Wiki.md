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
这是一个基于英飞凌(Infineon) Traveo II MCU的双核四旋翼无人机飞控系统，采用X型四旋翼布局。项目使用智飞科技的开发框架，实现了完整的无人机姿态控制、高度控制、位置保持、自动起降和**视觉识别追踪**功能。

### 核心功能
- **姿态稳定控制** - 串级PID控制（角度外环+角速度内环）
- **高度控制** - 基于TOF激光测距的定高飞行
- **位置保持** - 基于光流传感器+IMU融合的定点悬停
- **视觉识别** - 红外摄像头检测V形车标和信标
- **目标追踪** - 识别小车位置和航向，引导无人机追踪
- **自动起降** - 一键起飞和自动降落
- **磁力计融合** - 起飞后自动启用磁力计融合修正航向
- **Anti-Gravity** - 油门前馈补偿，防止急加速掉头

### 硬件平台
- **主控芯片**: 英飞凌Traveo II (CYT4BB系列，双核ARM Cortex-M7)
  - CM7_0: 控制核 (姿态控制、传感器读取)
  - CM7_1: 视觉核 (图像处理、目标识别)
- **姿态传感器**: IMU660RC (陀螺仪+加速度计)
- **磁力计**: QMC5883L
- **高度传感器**: TOF激光测距模块
- **位置传感器**: UPFLOW302 光流传感器
- **视觉传感器**: MT9V03x 红外摄像头 (通过SmartIO+DMA采集)
- **显示**: IPS200屏幕
- **电机驱动**: 小型驱动模块（UART通信）

---

## 项目架构

### 目录结构
```
smart_car-Vision-and-Drones/
├── libraries/              # 库文件目录
│   ├── sdk/               # 英飞凌SDK
│   ├── zf_driver/         # 智飞驱动库
│   ├── zf_device/         # 智飞设备库
│   ├── zf_common/         # 智飞通用库
│   └── zf_components/     # 智飞组件库
├── project/               # 项目代码目录
│   ├── code/             # 核心控制代码
│   │   ├── camera.c/h    # 视觉识别处理
│   │   ├── control.c/h   # 飞行控制
│   │   ├── of.c/h        # 光流数据处理
│   │   ├── PID.c/h       # PID控制器
│   │   ├── filter.c/h    # 数字滤波器
│   │   ├── IMU.c/h       # 姿态传感器
│   │   ├── motor.c/h     # 电机驱动
│   │   ├── TOF.c/h       # 激光测距
│   │   ├── QMC5883L.c/h  # 磁力计
│   │   ├── rc.c/h        # 遥控器
│   │   ├── battery.c/h   # 电池管理
│   │   ├── INIT.c/h      # 系统初始化
│   │   └── config.h      # 全局配置
│   ├── user/             # 用户代码 (main入口)
│   ├── check/            # 测试代码副本
│   └── iar/              # IAR工程文件
├── tools/                 # 工具脚本
│   ├── mag_cal.py        # 磁力计校准
│   └── mag_eval.py       # 磁力计评估
└── Code_Wiki.md          # 本文档
```

### 软件架构
```
┌─────────────────────────────────────────┐
│         应用层 (User Application)        │
│  main_cm7_0.c (控制核)                   │
│  main_cm7_1.c (视觉核)                   │
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
│  视觉识别 / Blob分析 / 软判决            │
└─────────────────────────────────────────┘
                  ↓
┌─────────────────────────────────────────┐
│         驱动层 (Driver Layer)            │
│  IMU / TOF / 光流 / 磁力计 / 电机       │
│  摄像头 / IPS屏幕                        │
└─────────────────────────────────────────┘
                  ↓
┌─────────────────────────────────────────┐
│         硬件抽象层 (HAL)                 │
│  智飞驱动库 / 英飞凌SDK                  │
└─────────────────────────────────────────┘
```

### 双核架构说明
本项目使用Traveo II的双核架构，两个ARM Cortex-M7核心分工明确：

| 核心 | 运行文件 | 主要职责 |
|------|----------|----------|
| **CM7_0** (控制核) | main_cm7_0.c | 姿态控制、传感器读取、电机输出、IPS显示 |
| **CM7_1** (视觉核) | main_cm7_1.c | 图像采集、视觉识别、Blob分析、目标追踪 |

**双核通信**：通过共享内存 `vision_share_t`（地址 `0x28001000`）交换数据。
- CM7_1 写入识别结果并刷新 D-Cache
- CM7_0 读取视觉数据进行位置控制

### 控制环路架构

| 控制环路       | 频率       | 控制器         | 说明                      |
|----------------|------------|----------------|---------------------------|
| 角速度内环     | 200~500Hz  | PIDVelX/Y/Z    | Roll/Pitch/Yaw角速度控制  |
| 姿态外环       | 100~200Hz  | PIDRoll/Pitch/Yaw | Roll/Pitch/Yaw角度控制 |
| 高度外环       | 50~100Hz   | PIDHeight      | 高度→目标垂直速度         |
| 高度内环       | 50~100Hz   | PIDVelH        | 垂直速度→油门修正         |
| 位置外环       | 50Hz       | PIDPosX/Y      | 位置→目标水平速度         |
| 位置内环       | 50Hz       | PIDPosX_Vel/Y_Vel | 水平速度→目标角度     |
| 视觉识别       | 帧同步      | 软判决评分      | 每帧图像处理+目标识别     |

---

## 主要模块说明

### 1. 主程序模块 ([main_cm7_0.c](file:///d:/UsersASUSDesktop/project/smart_car-Vision-and-Drones/smart_car-Vision-and-Drones/Vision%20and%20Drones6.18/project/user/main_cm7_0.c), [main_cm7_1.c](file:///d:/UsersASUSDesktop/project/smart_car-Vision-and-Drones/smart_car-Vision-and-Drones/Vision%20and%20Drones6.18/project/user/main_cm7_1.c))

#### CM7_0 (控制核) - main_cm7_0.c
**功能**: 系统主入口，负责控制、传感器、电机调度

**主要流程**:
1. 时钟初始化 (250MHz)
2. 调试初始化
3. 全局初始化 (`ALL_Init()`)
4. 设置PID目标值
5. 主循环：
   - 按键检测处理
   - 电机控制
   - VOFA+调试输出
   - IPS屏幕显示（读取共享内存中的视觉数据）

#### CM7_1 (视觉核) - main_cm7_1.c
**功能**: 视觉处理入口，负责图像采集和目标识别

**主要流程**:
1. 时钟初始化
2. 摄像头初始化 (`camera_init()`)
3. 主循环：
   - 图像处理 (`camera_process()`)
   - 识别结果写入共享内存
   - 刷新 D-Cache 通知控制核

---

### 2. 初始化模块 ([INIT.c](file:///d:/UsersASUSDesktop/project/smart_car-Vision-and-Drones/smart_car-Vision-and-Drones/Vision%20and%20Drones6.18/project/code/INIT.c), [INIT.h](file:///d:/UsersASUSDesktop/project/smart_car-Vision-and-Drones/smart_car-Vision-and-Drones/Vision%20and%20Drones6.18/project/code/INIT.h))

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

#### 视觉共享内存 ([camera.h](file:///d:/UsersASUSDesktop/project/smart_car-Vision-and-Drones/smart_car-Vision-and-Drones/Vision%20and%20Drones6.18/project/code/camera.h))
```c
typedef struct {
    uint32 frame_id;       // 帧号计数器

    int16 err_x;           // 信标相对小车的X误差 (前后)
    int16 err_y;           // 信标相对小车的Y误差 (左右)
    uint16 target_found;   // 同时识别到信标和小车

    int16 beacon_err_x;    // 信标相对画面中心的X误差
    int16 beacon_err_y;    // 信标相对画面中心的Y误差
    uint16 beacon_found;   // 信标检测标志

    int16 car_x;           // 小车X坐标
    int16 car_y;           // 小车Y坐标
    uint16 car_found;      // 小车检测标志
    uint16 reserved;

    float heading_angle;   // 小车航向角 (弧度)

    // IPS显示数据段 (CM7_0写入, CM7_1读取并驱动屏幕)
    float  disp_roll, disp_pitch, disp_yaw;
    uint16 disp_m1, disp_m2, disp_m3, disp_m4;
    float  disp_mag_x, disp_of_dx, disp_world_vx;
    float  disp_imu_gx, disp_of_height, disp_target;
    float  disp_volt;
    uint32 disp_dirty;     // 非0表示有新数据显示
} vision_share_t;
```

---

### 3. 控制模块 ([control.c](file:///d:/UsersASUSDesktop/project/smart_car-Vision-and-Drones/smart_car-Vision-and-Drones/Vision%20and%20Drones6.18/project/code/control.c), [control.h](file:///d:/UsersASUSDesktop/project/smart_car-Vision-and-Drones/smart_car-Vision-and-Drones/Vision%20and%20Drones6.18/project/code/control.h))

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

#### 视觉位置控制
当 `POSITION_TARGET_ZERO = 0` 且 `POSITION_HOLD = 1` 时启用：
- **测量值**: 屏幕中心（无人机自身位置）
- **目标值**: 视觉识别的小车坐标 (car_x, car_y)
- 串级控制：位置外环 → 速度内环 → Roll/Pitch目标角度

```c
// stabilization() 中的视觉位置控制段
PIDPosX.target = g_vision_share.car_x;  // 小车X坐标 → 位置目标
PIDPosY.target = g_vision_share.car_y;  // 小车Y坐标 → 位置目标
// 位置外环：误差 → 期望速度
PID_Update(&PIDPosX, 目标, 屏幕中心X, dt);
PID_Update(&PIDPosY, 目标, 屏幕中心Y, dt);
// 速度内环：期望速度 → 期望角度
PIDRoll.target  = PIDPosY_Vel.out;  // Y误差 → Roll
PIDPitch.target = PIDPosX_Vel.out;  // X误差 → Pitch
```

#### 核心函数

##### `flight_control(float dt)`
- **功能**: 飞行状态机顶层调度
- **调用频率**: 200Hz (由PIT定时中断触发)
- **状态流转**:
  ```
  LOCK → (解锁) → UNLOCK → (起飞) → TAKEOFF → FLY → (降落) → LAND → (落地) → LOCK
  ```

##### `stabilization(float dt)`
- **功能**: 姿态稳定控制（串级PID+电机混控）
- **流程**:
  1. Anti-Gravity油门前馈更新
  2. 高度外环+内环控制
  3. 位置外环+内环控制（含视觉追踪）
  4. Roll轴角度环+角速度环
  5. Pitch轴角度环+角速度环
  6. Yaw轴角度环+角速度环
  7. 电机混控输出

##### `take_off(float dt)`
- **功能**: 自动起飞控制
- **步骤**:
  1. 锁当前偏航，锁当前位置，标记起飞阶段
  2. 速度环纯P控制（清零积分，避免起飞累积）
  3. 线性升至目标高度 1.2m
  4. 到达目标高度后转入定点悬停

---

### 4. 视觉识别模块 ([camera.c](file:///d:/UsersASUSDesktop/project/smart_car-Vision-and-Drones/smart_car-Vision-and-Drones/Vision%20and%20Drones6.18/project/code/camera.c), [camera.h](file:///d:/UsersASUSDesktop/project/smart_car-Vision-and-Drones/smart_car-Vision-and-Drones/Vision%20and%20Drones6.18/project/code/camera.h))

**功能**: 红外摄像头图像采集、Blob分析、V形车标和信标识别

#### 工作流程 (每帧)
```
DMA读取灰度图 → 二值化 → BFS连通域提取(extract_blobs)
→ 原始分数计算(compute_raw_scores) → 跨帧跟踪(cross_frame_track)
→ 分数平滑与判决(smooth_and_decide) → V形去重 → 姿态解算
→ 共享内存刷新 → IPS显示/串口调试
```

#### 图像预处理
- 摄像头: MT9V03x 红外摄像头
- 分辨率: `MT9V03X_W × MT9V03X_H`（由DMA直接捕获到固定地址 `0x28026024`）
- 二值化阈值: `IR_THRESHOLD = 140`

#### 连通域提取 (extract_blobs)
- 算法: 4邻域BFS（广度优先搜索）
- 输入: 二值化图像
- 输出: Blob结构体数组（最多20个）
- 每个Blob属性:
  - **质心** (cx, cy): 所有像素坐标均值
  - **外接矩形** (width, height): 轴对齐边界框
  - **面积** (area): 像素总数
  - **核心高亮数** (core_count): 原始灰度 > 200 的像素数
  - **方向矩** (owidth/oheight): 通过二阶矩计算主轴方向，投影得到旋转不变矩形

#### V形车标检测 (compute_car_marker_feature)

**几何原理**: 小车顶部的V形标记（箭头），尖端指向车头

```
         A (角点/顶点)
        / \
       /   \
      /     \
     B-------C (底边两端)
        |
       D (底边中点 = 车中心)
```

**检测步骤**:
1. **预过滤**: 面积 [20, 900]，长宽比 ≥ 1.35（排除圆形）
2. **找候选点**: 8个方向的极值点 (x/y/x+y/x+反向y 的min/max)
3. **找底边 BC**: 候选点中距离最远的一对
4. **找顶点 A**: 到直线BC垂直距离最远的像素
5. **几何验证**:
   - 底边长 ≥ 8像素
   - 两臂长 ≥ 6像素
   - 顶点垂距 ≥ 4像素
   - 顶点夹角 ∈ [50°, 120°]

**验证通过后存储的特征**:
```c
marker_vertex_x/y       // 角点A (指向车头)
marker_base1_x/y        // 底边端点B
marker_base2_x/y        // 底边端点C
marker_base_mid_x/y     // 底边中点D (车中心)
marker_angle_deg        // 顶点夹角 (度)
marker_height           // 顶点到底边垂距
marker_base_len         // 底边长度 B-C
marker_arm1_len         // 臂长 A-B
marker_arm2_len         // 臂长 A-C
marker_heading          // 车头方向 D→A (弧度)
```

#### 软判决评分系统

每个Blob计算两种分数，通过分数高低决定分类：

**车标分 (满分100)**:
| 维度 | 满分 | 评分依据 |
|------|------|----------|
| 面积分 | 30 | 面积 [80,300] 满分，两侧线性衰减 |
| 角度分 | 30 | ∠A 越接近80°越高 |
| 对称性分 | 20 | 两臂长度比 ≥70% 满分 |
| 高度分 | 20 | 顶点垂距 ≥12 像素满分 |

**信标分 (满分100)**:
| 维度 | 满分 | 评分依据 |
|------|------|----------|
| 面积分 | 50 | 面积适中(10~150)，峰值在50附近 |
| 圆度分 | 50 | 长宽比越接近1:1越高 |
| 长条惩罚 | - | 长宽比>1.3时线性衰减 |
| 亮度惩罚 | - | 核心亮度<40时衰减 |

#### 跨帧跟踪 (cross_frame_track)
- 当前帧Blob与上一帧按**质心距离**匹配（阈值25像素）
- 匹配上的继承历史track_id和滤波分数
- 未匹配的新Blob分配唯一track_id
- 分数一阶滞后滤波：`filt = 0.65×历史 + 0.35×当前`

#### 类型判决 (smooth_and_decide)
- 首帧: 最低置信度15，领先差距8
- 匹配帧: 最低置信度30，领先差距12
- V形车标额外校验：分数 ≥ 55，角度 [50°, 120°]
- 继承历史类型（与当前最高分类一致时）

#### 姿态解算 (find_car_pose_by_marker)
- **车中心** = 底边中点D → `(car_x, car_y)`
- **车头方向** = D→A 向量角度 → `heading_angle`
- 航向角一阶低通滤波平滑（α=0.7）
- 小车坐标丢失后以 `CAR_RETAIN_DECAY=0.85` 每帧衰减

#### 目标追踪误差计算
当同时检测到信标和小车时：
```
raw_dx = beacon_cx - car_x  (像素X差)
raw_dy = beacon_cy - car_y  (像素Y差)

// 旋转到机体坐标系
err_x = raw_dx·cos(heading) + raw_dy·sin(heading)  → 前后误差
err_y = -raw_dx·sin(heading) + raw_dy·cos(heading)  → 左右误差
```
- 丢失目标后误差以 `ERR_DECAY_FACTOR=0.5` 每帧衰减
- 多信标时通过 track_id 锁定追踪，避免来回切换

#### 摄像头硬件初始化 (camera_init)
```c
gpio_init(P18_0~P18_7, GPI, ...);  // 8位数据引脚
gpio_init(P06_5, P06_6, ...);       // PCLK和VSYNC
mt9v03x_init();                      // 摄像头初始化(含重试)
ips200_init();                       // IPS屏幕初始化
```

---

### 5. 光流模块 ([of.c](file:///d:/UsersASUSDesktop/project/smart_car-Vision-and-Drones/smart_car-Vision-and-Drones/Vision%20and%20Drones6.18/project/code/of.c), [of.h](file:///d:/UsersASUSDesktop/project/smart_car-Vision-and-Drones/smart_car-Vision-and-Drones/Vision%20and%20Drones6.18/project/code/of.h))

**功能**: UPFLOW302 光流传感器数据处理 + 光流-IMU速度融合

#### 光流测速原理
光流传感器检测地面图像运动，输出像素位移 (Δx, Δy)，结合高度和焦距参数推算水平速度：

```
角速度 ω = Δx / (dt × K)         // 像素位移 → 角速度
线速度 v = ω × h                  // 角速度 × 高度 → 线速度
最终: v = Δx × h / (dt × K)
```

其中 K 为传感器角度-像素换算系数，需实际标定。

#### 陀螺仪补偿
无人机旋转时，光流会测到额外的"虚假"运动，需用陀螺仪数据抵消：
```
ω_true = ω_optical + ω_gyro
```

#### 核心函数

##### `OF_init()`
- 启动UPFLOW302传感器
- 清零所有速度/位置/位移状态变量
- 首次初始化时将滤波后光流速度赋给世界坐标系

##### `OF_data_deal(float dt)`
- **输入**: dt (时间间隔)
- **处理流程**:
  1. 获取当前高度 (来自TOF激光测距)
  2. 读取UPFLOW302原始像素位移 (dx, dy)
  3. 像素位移 ÷ (dt × K) → 角速度
  4. 陀螺仪补偿 → 去除旋转分量
  5. 角速度 × 高度 → 水平线速度
  6. PT1一阶低通滤波平滑输出

##### `velocity_mahony_fusion(float dt)`
- **算法**: 类Mahony互补滤波

借鉴Mahony姿态融合思路，将光流速度作为"观测值"，加速度积分速度作为"预测值"：
```
预测: v_pred += a_imu × dt        (加速度积分)
误差: e = v_of - v_pred           (光流观测 - 积分预测)
校正: v_fusion = v_pred + Kp×e + Ki×∫e·dt
```

**参数**:
- Kp = 0.8: 光流观测置信度
- Ki = 0.2: 积分消除加速度漂移（起飞阶段Ki=0）

**输出**:
- `world_data.vx/vy`: 融合后速度
- `world_data.px/py`: 位置（速度积分）

#### 滤波器配置
```c
#define OF_VX_FILTER_FREQ   30.0f   // 光流X速度滤波 (Hz)
#define OF_VY_FILTER_FREQ   30.0f   // 光流Y速度滤波 (Hz)
```

---

### 6. PID控制模块 ([PID.c](file:///d:/UsersASUSDesktop/project/smart_car-Vision-and-Drones/smart_car-Vision-and-Drones/Vision%20and%20Drones6.18/project/code/PID.c), [PID.h](file:///d:/UsersASUSDesktop/project/smart_car-Vision-and-Drones/smart_car-Vision-and-Drones/Vision%20and%20Drones6.18/project/code/PID.h))

**功能**: PID控制器实现

#### PID参数初始化

##### 内环角速度PID
| 控制器 | Kp | Ki | Kd | 积分限幅 | 输出限幅 |
|--------|----|----|----|----------|----------|
| PIDVelX (Roll) | 6.5 | 2.6 | 0.6 | ±800 | ±2000 |
| PIDVelY (Pitch) | 6.5 | 2.6 | 0.6 | ±800 | ±2000 |
| PIDVelZ (Yaw) | 10.0 | 3.0 | 0.0 | ±800 | ±1500 |

##### 外环角度PID
| 控制器 | Kp | Ki | Kd | 积分限幅 | 输出限幅 |
|--------|----|----|----|----------|----------|
| PIDRoll | 3.0 | 0 | 0 | 0 | ±200 |
| PIDPitch | 3.0 | 0 | 0 | 0 | ±200 |
| PIDYaw | 4.0 | 0 | 0 | 0 | ±150 |

##### 高度环PID
| 控制器 | Kp | Ki | Kd | 积分限幅 | 输出限幅 |
|--------|----|----|----|----------|----------|
| PIDHeight (外环) | 1.0 | 0 | 0 | ±150 | ±0.3 |
| PIDVelH (内环) | 450.0 | 20.0 | 0 | ±100 | ±600 |

##### 位置环PID
| 控制器 | Kp | Ki | Kd | 积分限幅 | 输出限幅 |
|--------|----|----|----|----------|----------|
| PIDPosX/Y (外环) | 0.02 | 0 | 0 | 0 | ±0.8 |
| PIDPosX_Vel/Y_Vel (内环) | 8.0 | 0.5 | 0 | ±100 | ±30 |

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

### 7. 滤波器模块 ([filter.c](file:///d:/UsersASUSDesktop/project/smart_car-Vision-and-Drones/smart_car-Vision-and-Drones/Vision%20and%20Drones6.18/project/code/filter.c), [filter.h](file:///d:/UsersASUSDesktop/project/smart_car-Vision-and-Drones/smart_car-Vision-and-Drones/Vision%20and%20Drones6.18/project/code/filter.h))

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
| filter_height | 50Hz | 高度数据滤波 |
| filter_height_vz | 5Hz | 垂直速度滤波 |
| filter_pwm3901_vx | 30Hz | 光流X速度滤波 |
| filter_pwm3901_vy | 30Hz | 光流Y速度滤波 |

---

### 8. 姿态传感器模块 ([IMU.h](file:///d:/UsersASUSDesktop/project/smart_car-Vision-and-Drones/smart_car-Vision-and-Drones/Vision%20and%20Drones6.18/project/code/IMU.h))

**功能**: IMU数据读取和姿态融合

#### Mahony姿态融合
- 使用经典的Mahony互补滤波算法
- 融合陀螺仪和加速度计数据
- 自适应Kp: 水平加速时降低修正权重

##### 自适应Kp策略
```c
#define MAHONY_KP_STARTUP      7.0f    // 开机前5秒Kp上限 (快速收敛)
#define MAHONY_KP_NORMAL       0.4f    // 5秒后Kp上限 (稳态收敛)
#define MAHONY_KP_MIN          0.05f   // 剧烈机动时Kp
#define MAHONY_ACC_DEV_THRESH  0.15f   // 加速度偏离阈值(g)
#define MAHONY_STARTUP_TIME    6.0f    // 开机加速收敛时长
```

核心思路: `acc_dev = |加速度模长 - 1g|` → acc_dev越大说明机动越剧烈 → Kp降得越低，更多依赖陀螺仪积分。

---

### 9. 配置模块 ([config.h](file:///d:/UsersASUSDesktop/project/smart_car-Vision-and-Drones/smart_car-Vision-and-Drones/Vision%20and%20Drones6.18/project/code/config.h))

**功能**: 全局配置参数

#### 功能模块开关
| 宏 | 默认 | 说明 |
|----|------|------|
| ATTITUDE | 1 | 姿态控制 |
| HEIGHT | 1 | 高度控制 |
| POSITION_HOLD | 1 | 位置保持 |
| POSITION_TARGET_ZERO | 1 | 1=定点悬停, 0=视觉追踪小车 |
| THROTTLE_ATTENUATION | 1 | 油门衰减(电池电压补偿) |
| BASE_SPEED | 1 | 基础速度模式 |

#### 视觉追踪参数
| 宏 | 值 | 说明 |
|----|----|------|
| CAR_RETAIN_DECAY | 0.85 | 小车坐标丢失后衰减系数 |
| PIXEL_TO_VEL_SCALE | 0.02 | 像素误差→速度转换系数 |
| BEACON_HOLD_FRAMES_MAX | 100 | 信标丢失后保持帧数 |
| BEACON_FADE_DECAY | 0.92 | 信标超时衰减系数 |

---

## 依赖关系

```
main_cm7_0.c
  ├── INIT.c/h           → 系统初始化
  ├── control.c/h         → 飞行控制
  │   ├── PID.c/h        → PID控制
  │   ├── filter.c/h      → 数字滤波
  │   ├── motor.c/h       → 电机驱动
  │   ├── IMU.c/h         → 姿态数据
  │   ├── of.c/h          → 光流数据
  │   ├── TOF.c/h         → 高度数据
  │   ├── QMC5883L.c/h    → 磁力计
  │   ├── rc.c/h          → 遥控器
  │   ├── battery.c/h     → 电池管理
  │   ├── camera.h        → 视觉共享内存
  │   └── config.h        → 全局配置
  └── zf_driver/          → 智飞驱动库

main_cm7_1.c
  └── camera.c/h          → 视觉识别
      ├── zf_driver/      → 驱动
      ├── zf_device/      → 设备库(MT9V03x, IPS200等)
      └── HC06_Driver.c/h → 蓝牙调试
```

---

## 运行方式

### 开发环境
- **IDE**: IAR Embedded Workbench for ARM
- **编译器**: IAR ARM Compiler
- **调试器**: J-Link / I-jet

### 编译与烧录
1. 打开 IAR 工程文件 (`project/iar/` 目录下)
2. 选择目标核心 (CM7_0 或 CM7_1)
3. 编译 (F7)
4. 烧录 (Ctrl+D)

### 调试输出
- 串口: 115200 baud
- VOFA+协议: 用于实时波形显示
- IPS屏幕: 显示姿态/高度/视觉识别结果

---

## 配置说明

### 飞行参数调整
所有飞行参数集中在 `config.h` 中调整：

1. **PID参数**: 各环路的Kp/Ki/Kd值
2. **滤波器参数**: 截止频率调整
3. **功能开关**: 启用/禁用特定功能
4. **视觉参数**: 检测阈值、衰减系数
5. **起飞参数**: 目标高度、上升速度

### 视觉参数调整
在 `camera.h` 中调整视觉识别参数：

1. **二值化阈值** `IR_THRESHOLD`: 根据光照环境调整
2. **V形检测参数**: 面积/长度/角度阈值
3. **分数权重**: 面积分/角度分/对称性分权重
4. **跟踪参数**: 匹配距离、平滑系数
