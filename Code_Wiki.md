# 智能无人机飞控项目 Code Wiki

## 目录
1. [项目概述](#项目概述)
2. [项目架构](#项目架构)
3. [主要模块说明](#主要模块说明)
4. [核心函数说明](#核心函数说明)
5. [依赖关系](#依赖关系)
6. [运行方式](#运行方式)
7. [配置说明](#配置说明)
8. [版本历史](#版本历史)

---

## 项目概述

### 项目简介
这是一个基于英飞凌(Infineon) Traveo II MCU的双核四旋翼无人机飞控系统，采用X型四旋翼布局。项目使用智飞科技的开发框架，实现了完整的无人机姿态控制、高度控制、位置保持、自动起降和**视觉识别追踪**功能。上位机工具链支持实时数据可视化与摄像头画面显示。

### 核心功能
- **姿态稳定控制** - 串级PID控制（角度外环+角速度内环）
- **高度控制** - 基于TOF激光测距的定高飞行
- **位置保持** - 基于光流传感器+IMU融合的定点悬停
- **视觉识别** - 红外摄像头检测V形车标、信标和车体碎片
- **目标追踪** - 识别小车位置和航向，引导无人机追踪
- **碎片追车** - V形丢失时用方向长宽比>150的碎片平均中心作为替代
- **动态前馈** - 像素速度前馈随飞机速度自适应衰减（0.8m/s归零）
- **自动起降** - 一键起飞和自动降落
- **上位机可视化** - Python实时监控，支持串口数据+摄像头双路显示
- **IPS屏幕显示** - 实时显示二值化图像和飞行数据

### 硬件平台
- **主控芯片**: 英飞凌Traveo II (CYT4BB系列，双核ARM Cortex-M7)
  - CM7_0: 控制核 (姿态控制、传感器读取)
  - CM7_1: 视觉核 (图像处理、目标识别)
- **姿态传感器**: IMU660RC (陀螺仪+加速度计)
- **磁力计**: QMC5883L
- **高度传感器**: TOF激光测距模块
- **位置传感器**: UPFLOW302 光流传感器
- **视觉传感器**: MT9V03x 红外摄像头 (188×120，通过SmartIO+DMA采集)
- **显示**: IPS200屏幕
- **电机驱动**: 小型驱动模块（UART通信）
- **蓝牙**: HC06串口透传（接收小车指令、发送误差数据）

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
│   │   ├── HC06_Driver.c/h # 蓝牙驱动
│   │   ├── INIT.c/h      # 系统初始化
│   │   └── config.h      # 全局配置
│   ├── user/             # 用户代码 (main入口)
│   │   ├── main_cm7_0.c  # CM7_0: 控制核入口
│   │   ├── main_cm7_1.c  # CM7_1: 视觉核入口
│   │   └── cm7_0_isr.c   # CM7_0 PIT中断服务
│   ├── check/            # 测试代码副本
│   └── iar/              # IAR工程文件
├── tools/                 # 工具脚本
│   ├── realtime_monitor.py # 上位机实时监控
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
│  摄像头 / IPS屏幕 / HC06蓝牙             │
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
| **CM7_0** (控制核) | main_cm7_0.c | 姿态控制、传感器读取、电机输出、HC06通信 |
| **CM7_1** (视觉核) | main_cm7_1.c | 图像采集、视觉识别、Blob分析、IPS显示 |

**双核通信**：通过共享内存 `vision_share_t`（地址 `0x28001000`）交换数据。
- CM7_1 写入识别结果并刷新 D-Cache
- CM7_0 读取视觉数据进行位置控制
- 双核均调用 `SCB_DisableDCache()` 禁用D-Cache，避免缓存一致性问题

### 控制环路架构

| 控制环路       | 频率       | 控制器         | 说明                      |
|----------------|------------|----------------|---------------------------|
| 角速度内环     | 200Hz      | PIDVelX/Y/Z    | Roll/Pitch/Yaw角速度控制  |
| 姿态外环       | 200Hz      | PIDRoll/Pitch/Yaw | Roll/Pitch/Yaw角度控制 |
| 高度外环       | 200Hz      | PIDHeight      | 高度→目标垂直速度         |
| 高度内环       | 200Hz      | PIDVelH        | 垂直速度→油门修正         |
| 位置/速度环    | 40Hz       | PIDPosX/Y + Vel | 位置→速度→角度 (分频5) |
| 视觉识别       | 50Hz       | 软判决评分      | 每帧图像处理+目标识别     (MT9V03X_FPS_DEF=50) |
| HC06蓝牙通信   | 100Hz      | cm7_0_isr.c    | 发送误差、接收指令         |

---

## 主要模块说明

### 1. 主程序模块

#### CM7_0 (控制核) - main_cm7_0.c
**功能**: 系统主入口，负责控制、传感器、HC06通信调度

**主要流程**:
1. 时钟初始化 (250MHz)
2. INTT初始化（NVIC优先级分组）
3. `ALL_Init()` 全局初始化
4. 设置PID目标值
5. 主循环：
   - 按键检测处理
   - 电机控制输出
   - `HC06_Task()` 蓝牙指令收发
   - 飞行状态机调度 (`flight_control`)

#### CM7_1 (视觉核) - main_cm7_1.c
**功能**: 视觉处理入口，负责图像采集和IPS显示

**主要流程**:
1. 时钟初始化
2. `camera_init()` 摄像头初始化
3. 主循环：
   - `camera_process()` 图像处理
   - IPS屏幕显示（二值化图像 + 飞行数据覆盖）
   - 识别结果写入共享内存，刷新 D-Cache

---

### 2. 视觉识别模块 (camera.c / camera.h)

**功能**: 红外摄像头图像采集、Blob分析、V形车标检测、信标识别、碎片追车

#### 工作流程 (每帧 25Hz)
```
DMA读取灰度图 → 二值化 → BFS连通域提取(extract_blobs)
→ 原始分数计算(soft_decision_scores) → 跨帧跟踪(cross_frame_track)
→ 分数平滑与判决(smooth_and_decide) → V形轮廓内信标清除
→ 碎片追车(found_car=0且碎片存在时) → 姿态解算 → 共享内存刷新
```

#### 图像预处理
- 摄像头: MT9V03x 红外摄像头
- 分辨率: 188 × 120
- 二值化阈值: `IR_THRESHOLD = 140`
- 核心高亮阈值: `IR_CORE_THRESHOLD = 200`

#### 连通域提取 (extract_blobs)
- 算法: 4邻域BFS（广度优先搜索）
- 输入: 二值化图像
- 输出: Blob结构体数组（最多20个）
- 每个Blob属性包含质心、外接矩形、面积、核心高亮数、方向矩等

#### V形车标检测 (compute_car_marker_feature)

**几何原理**: 小车顶部的V形标记（箭头），尖端指向车头

```
         A (角点/顶点)
        / \
       /   \
      /     \
     B-------C (底边两端)
        |
       G (三角形重心 = (A+B+C)/3 = 车中心)
```

**检测步骤**:
1. **预过滤**: 面积 [20, 900]，长宽比 ≥ 1.35
2. **极值点**: 8个方向的最远点
3. **找底边 BC**: 候选点中距离最远的一对
4. **找顶点 A**: 到直线BC垂直距离最远的像素
5. **几何验证**: 底边≥8px, 臂长≥6px, 垂距≥5px, 夹角[12°, 90°]

#### 软判决评分系统

每个Blob计算三种分数：

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

**车头分** 和 **车尾分**: 用于V形以外的辅助分类（目前由V形检测主导）

#### 跨帧跟踪 (cross_frame_track)
- 当前帧Blob与上一帧按**质心距离**匹配（阈值25像素）
- 匹配上的继承历史track_id和滤波分数
- 分数一阶滞后滤波：`filt = 0.65×历史 + 0.35×当前`

#### 碎片追车逻辑
当V形车标未识别到 (`found_car == 0`) 时：

1. **V形检测到**:
   - `car_found = 1, car_fresh = 1`
   - 更新保留坐标到当前检测值

2. **V形丢失 → 碎片追车**:
   - 遍历所有blob，筛选**方向长宽比 > 150**的碎片
   - 排除已被标记为 `BLOB_CAR_MARKER` 的blob
   - 面积 ≥ `CAR_MARK_MIN_AREA(20)`
   - 计算符合条件的碎片中心平均值 → 作为小车坐标
   - `car_found = 1, car_fresh = 1`

3. **无碎片 → 衰减归中**:
   - 每帧按 `CAR_RETAIN_DECAY(0.90)` 衰减到图像中心 (94, 60)
   - 公式: `retain = (retain - center) * 0.90 + center`
   - 衰减到中心附近 ±0.5px 时，`car_found = 0, car_fresh = 0`

#### V形轮廓内信标清除
检测到V形车标时，以车标blob的外接矩形 +1px 为区域，区域内所有 `BLOB_BEACON` 类型强制设为 `BLOB_UNKNOWN`，防止V形内部误识别伪信标。

#### 姿态解算 (find_car_pose_by_marker)
- **车中心** = 三角形重心 (A+B+C)/3 → `(car_x, car_y)`
- **车头方向** = D→A 向量角度 → `heading_angle`
- 航向角一阶低通滤波平滑（α=0.7）
- 信标-小车误差转换到机体坐标系传给CM7_0

---

### 3. 共享内存结构 (vision_share_t)

```c
typedef struct {
    uint32 frame_id;            // 帧序号

    int16 err_x, err_y;         // 信标相对小车的机体坐标系误差 (前后/左右)
    uint16 target_found;        // 同时识别到信标和小车

    int16 beacon_err_x, beacon_err_y; // 信标相对画面中心的误差
    uint16 beacon_found;        // 信标存在标志

    int16 car_x, car_y;         // 小车中心坐标 (画面坐标系)
    uint16 car_found;           // 小车存在标志
    uint16 car_fresh;           // 1=真实检测/碎片, 0=衰减保留值 (前馈用)

    float heading_angle;        // 滤波后小车航向角 (弧度)

    // IPS 显示数据段 (CM7_0 写入, CM7_1 读取)
    float  disp_roll, disp_pitch, disp_yaw;
    uint16 disp_m1, disp_m2, disp_m3, disp_m4;
    float  disp_mag_x, disp_of_dx, disp_world_vx;
    float  disp_world_vy;       // 世界坐标系 Y 轴速度
    float  disp_imu_gx, disp_of_height, disp_target;
    float  disp_volt;
    float  vel_tgt_x, vel_tgt_y; // 速度环目标 (上位机显示)
    float  ff_vel_x, ff_vel_y;    // 像素前馈量 (上位机显示)
    uint32 disp_dirty;

    // 视觉调试数据
    uint16 debug_car_score;
    int16  debug_car_angle;
    uint8  debug_car_method;
    uint8  debug_car_track_id;
} vision_share_t;
```

---

### 4. 控制模块 (control.c / control.h)

**功能**: 飞行控制核心，包含飞行状态机(FLY/TAKEOFF/LAND)和串级PID控制

#### 电机布局 (X型四旋翼，俯视图)
```
    1     2
        X
    4     3
```
- 电机1: 右前 (CCW, 逆时针)
- 电机2: 左前 (CW, 顺时针)
- 电机3: 左后 (CCW, 逆时针)
- 电机4: 右后 (CW, 顺时针)

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

---

### 飞行状态机 `flight_control()`（200Hz PIT中断）

```
STATE_LOCK → STATE_UNLOCK（#1$）
          → STATE_TAKEOFF（#2$）→ STATE_FLY（到达1.3m自动切换）
                                   → STATE_LAND（再次收到#2$或10s超时）
                                   → STATE_LOCK（降落完成）
```

#### STATE_LOCK — 停浆复位
- 全部电机停转，`out_flag = 0`
- 清零所有标志：`yaw_angle_enabled`、`pos_ctrl_enabled`、`takeoff_phase`、`hover_lock`、`height_vel_only`
- 恢复 PID 参数为宏默认值（角速度 Kp 恢复 VELX_KP/VELY_KP/VELZ_KP，速度环 Kp 恢复 POS_VELX_KP/POS_VELY_KP）
- 复位 10s 自动降落计时器 `g_no_beacon_timer`
- `takeoff_base_thr = HOVER_THRUST`

#### STATE_UNLOCK — 电机测试
- `out_flag = 1`，主循环输出固定占空比

#### STATE_TAKEOFF — 起飞爬升

| 子步骤 | 操作 |
|--------|------|
| Step 0 | 锁偏航目标 `PIDYaw.target = eulerAngle.yaw`、高度目标归零、锁当前位置、速度环 Ki 清零 |
| Step 1 | 1.5s 目标高度线性爬升至 1.3m；`stabilization()` 自动做 1.5s 油门斜坡 3500→HOVER_THRUST |
| `pz ≥ 1.0m` | 开启偏航角度环 `yaw_angle_enabled=1`，重新锁 `PIDYaw.target` |
| `target_height ≥ 1.3m` | 恢复速度环 Ki (`POS_VEL_KI`)、锁 hover_point、**自动切换 STATE_FLY** |

关键设计：
- 起飞全程 `pos_ctrl_enabled=0`，速度环 V=0 定点（不跑位置环）
- `takeoff_phase=1` → 光流融合 Ki 清零 + 油门斜坡生效
- Yaw 仅角速度环 `target=0`，防止磁力计干扰导致自旋

#### STATE_FLY — 正常飞行
- `pos_ctrl_enabled=1` → 视觉追车（位置外环+速度内环串级）
- 维持目标高度 `TAKEOFF_TARGET_HEIGHT = 1.3m`
- 若未使能偏航角度环则自动开启（兜底）
- **10s 自动降落计时器**：进入 FLY 开始累加，满 10s → 发送 `#0,0,3# 智能无人机飞控项目 Code Wiki

## 目录
1. [项目概述](#项目概述)
2. [项目架构](#项目架构)
3. [主要模块说明](#主要模块说明)
4. [核心函数说明](#核心函数说明)
5. [依赖关系](#依赖关系)
6. [运行方式](#运行方式)
7. [配置说明](#配置说明)
8. [版本历史](#版本历史)

---

## 项目概述

### 项目简介
这是一个基于英飞凌(Infineon) Traveo II MCU的双核四旋翼无人机飞控系统，采用X型四旋翼布局。项目使用智飞科技的开发框架，实现了完整的无人机姿态控制、高度控制、位置保持、自动起降和**视觉识别追踪**功能。上位机工具链支持实时数据可视化与摄像头画面显示。

### 核心功能
- **姿态稳定控制** - 串级PID控制（角度外环+角速度内环）
- **高度控制** - 基于TOF激光测距的定高飞行
- **位置保持** - 基于光流传感器+IMU融合的定点悬停
- **视觉识别** - 红外摄像头检测V形车标、信标和车体碎片
- **目标追踪** - 识别小车位置和航向，引导无人机追踪
- **碎片追车** - V形丢失时用方向长宽比>150的碎片平均中心作为替代
- **动态前馈** - 像素速度前馈随飞机速度自适应衰减（0.8m/s归零）
- **自动起降** - 一键起飞和自动降落
- **上位机可视化** - Python实时监控，支持串口数据+摄像头双路显示
- **IPS屏幕显示** - 实时显示二值化图像和飞行数据

### 硬件平台
- **主控芯片**: 英飞凌Traveo II (CYT4BB系列，双核ARM Cortex-M7)
  - CM7_0: 控制核 (姿态控制、传感器读取)
  - CM7_1: 视觉核 (图像处理、目标识别)
- **姿态传感器**: IMU660RC (陀螺仪+加速度计)
- **磁力计**: QMC5883L
- **高度传感器**: TOF激光测距模块
- **位置传感器**: UPFLOW302 光流传感器
- **视觉传感器**: MT9V03x 红外摄像头 (188×120，通过SmartIO+DMA采集)
- **显示**: IPS200屏幕
- **电机驱动**: 小型驱动模块（UART通信）
- **蓝牙**: HC06串口透传（接收小车指令、发送误差数据）

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
│   │   ├── HC06_Driver.c/h # 蓝牙驱动
│   │   ├── INIT.c/h      # 系统初始化
│   │   └── config.h      # 全局配置
│   ├── user/             # 用户代码 (main入口)
│   │   ├── main_cm7_0.c  # CM7_0: 控制核入口
│   │   ├── main_cm7_1.c  # CM7_1: 视觉核入口
│   │   └── cm7_0_isr.c   # CM7_0 PIT中断服务
│   ├── check/            # 测试代码副本
│   └── iar/              # IAR工程文件
├── tools/                 # 工具脚本
│   ├── realtime_monitor.py # 上位机实时监控
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
│  摄像头 / IPS屏幕 / HC06蓝牙             │
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
| **CM7_0** (控制核) | main_cm7_0.c | 姿态控制、传感器读取、电机输出、HC06通信 |
| **CM7_1** (视觉核) | main_cm7_1.c | 图像采集、视觉识别、Blob分析、IPS显示 |

**双核通信**：通过共享内存 `vision_share_t`（地址 `0x28001000`）交换数据。
- CM7_1 写入识别结果并刷新 D-Cache
- CM7_0 读取视觉数据进行位置控制
- 双核均调用 `SCB_DisableDCache()` 禁用D-Cache，避免缓存一致性问题

### 控制环路架构

| 控制环路       | 频率       | 控制器         | 说明                      |
|----------------|------------|----------------|---------------------------|
| 角速度内环     | 200Hz      | PIDVelX/Y/Z    | Roll/Pitch/Yaw角速度控制  |
| 姿态外环       | 200Hz      | PIDRoll/Pitch/Yaw | Roll/Pitch/Yaw角度控制 |
| 高度外环       | 200Hz      | PIDHeight      | 高度→目标垂直速度         |
| 高度内环       | 200Hz      | PIDVelH        | 垂直速度→油门修正         |
| 位置/速度环    | 40Hz       | PIDPosX/Y + Vel | 位置→速度→角度 (分频5) |
| 视觉识别       | 50Hz       | 软判决评分      | 每帧图像处理+目标识别     (MT9V03X_FPS_DEF=50) |
| HC06蓝牙通信   | 100Hz      | cm7_0_isr.c    | 发送误差、接收指令         |

---

## 主要模块说明

### 1. 主程序模块

#### CM7_0 (控制核) - main_cm7_0.c
**功能**: 系统主入口，负责控制、传感器、HC06通信调度

**主要流程**:
1. 时钟初始化 (250MHz)
2. INTT初始化（NVIC优先级分组）
3. `ALL_Init()` 全局初始化
4. 设置PID目标值
5. 主循环：
   - 按键检测处理
   - 电机控制输出
   - `HC06_Task()` 蓝牙指令收发
   - 飞行状态机调度 (`flight_control`)

#### CM7_1 (视觉核) - main_cm7_1.c
**功能**: 视觉处理入口，负责图像采集和IPS显示

**主要流程**:
1. 时钟初始化
2. `camera_init()` 摄像头初始化
3. 主循环：
   - `camera_process()` 图像处理
   - IPS屏幕显示（二值化图像 + 飞行数据覆盖）
   - 识别结果写入共享内存，刷新 D-Cache

---

### 2. 视觉识别模块 (camera.c / camera.h)

**功能**: 红外摄像头图像采集、Blob分析、V形车标检测、信标识别、碎片追车

#### 工作流程 (每帧 25Hz)
```
DMA读取灰度图 → 二值化 → BFS连通域提取(extract_blobs)
→ 原始分数计算(soft_decision_scores) → 跨帧跟踪(cross_frame_track)
→ 分数平滑与判决(smooth_and_decide) → V形轮廓内信标清除
→ 碎片追车(found_car=0且碎片存在时) → 姿态解算 → 共享内存刷新
```

#### 图像预处理
- 摄像头: MT9V03x 红外摄像头
- 分辨率: 188 × 120
- 二值化阈值: `IR_THRESHOLD = 140`
- 核心高亮阈值: `IR_CORE_THRESHOLD = 200`

#### 连通域提取 (extract_blobs)
- 算法: 4邻域BFS（广度优先搜索）
- 输入: 二值化图像
- 输出: Blob结构体数组（最多20个）
- 每个Blob属性包含质心、外接矩形、面积、核心高亮数、方向矩等

#### V形车标检测 (compute_car_marker_feature)

**几何原理**: 小车顶部的V形标记（箭头），尖端指向车头

```
         A (角点/顶点)
        / \
       /   \
      /     \
     B-------C (底边两端)
        |
       G (三角形重心 = (A+B+C)/3 = 车中心)
```

**检测步骤**:
1. **预过滤**: 面积 [20, 900]，长宽比 ≥ 1.35
2. **极值点**: 8个方向的最远点
3. **找底边 BC**: 候选点中距离最远的一对
4. **找顶点 A**: 到直线BC垂直距离最远的像素
5. **几何验证**: 底边≥8px, 臂长≥6px, 垂距≥5px, 夹角[12°, 90°]

#### 软判决评分系统

每个Blob计算三种分数：

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

**车头分** 和 **车尾分**: 用于V形以外的辅助分类（目前由V形检测主导）

#### 跨帧跟踪 (cross_frame_track)
- 当前帧Blob与上一帧按**质心距离**匹配（阈值25像素）
- 匹配上的继承历史track_id和滤波分数
- 分数一阶滞后滤波：`filt = 0.65×历史 + 0.35×当前`

#### 碎片追车逻辑
当V形车标未识别到 (`found_car == 0`) 时：

1. **V形检测到**:
   - `car_found = 1, car_fresh = 1`
   - 更新保留坐标到当前检测值

2. **V形丢失 → 碎片追车**:
   - 遍历所有blob，筛选**方向长宽比 > 150**的碎片
   - 排除已被标记为 `BLOB_CAR_MARKER` 的blob
   - 面积 ≥ `CAR_MARK_MIN_AREA(20)`
   - 计算符合条件的碎片中心平均值 → 作为小车坐标
   - `car_found = 1, car_fresh = 1`

3. **无碎片 → 衰减归中**:
   - 每帧按 `CAR_RETAIN_DECAY(0.90)` 衰减到图像中心 (94, 60)
   - 公式: `retain = (retain - center) * 0.90 + center`
   - 衰减到中心附近 ±0.5px 时，`car_found = 0, car_fresh = 0`

#### V形轮廓内信标清除
检测到V形车标时，以车标blob的外接矩形 +1px 为区域，区域内所有 `BLOB_BEACON` 类型强制设为 `BLOB_UNKNOWN`，防止V形内部误识别伪信标。

#### 姿态解算 (find_car_pose_by_marker)
- **车中心** = 三角形重心 (A+B+C)/3 → `(car_x, car_y)`
- **车头方向** = D→A 向量角度 → `heading_angle`
- 航向角一阶低通滤波平滑（α=0.7）
- 信标-小车误差转换到机体坐标系传给CM7_0

---

### 3. 共享内存结构 (vision_share_t)

```c
typedef struct {
    uint32 frame_id;            // 帧序号

    int16 err_x, err_y;         // 信标相对小车的机体坐标系误差 (前后/左右)
    uint16 target_found;        // 同时识别到信标和小车

    int16 beacon_err_x, beacon_err_y; // 信标相对画面中心的误差
    uint16 beacon_found;        // 信标存在标志

    int16 car_x, car_y;         // 小车中心坐标 (画面坐标系)
    uint16 car_found;           // 小车存在标志
    uint16 car_fresh;           // 1=真实检测/碎片, 0=衰减保留值 (前馈用)

    float heading_angle;        // 滤波后小车航向角 (弧度)

    // IPS 显示数据段 (CM7_0 写入, CM7_1 读取)
    float  disp_roll, disp_pitch, disp_yaw;
    uint16 disp_m1, disp_m2, disp_m3, disp_m4;
    float  disp_mag_x, disp_of_dx, disp_world_vx;
    float  disp_world_vy;       // 世界坐标系 Y 轴速度
    float  disp_imu_gx, disp_of_height, disp_target;
    float  disp_volt;
    float  vel_tgt_x, vel_tgt_y; // 速度环目标 (上位机显示)
    float  ff_vel_x, ff_vel_y;    // 像素前馈量 (上位机显示)
    uint32 disp_dirty;

    // 视觉调试数据
    uint16 debug_car_score;
    int16  debug_car_angle;
    uint8  debug_car_method;
    uint8  debug_car_track_id;
} vision_share_t;
```

---

 给小车 + 自动切 `STATE_LAND`

#### STATE_LAND — 降落 `land()`

| 阶段 | 条件 | 控制方式 |
|------|------|----------|
| 初始化 Step 0 | 进入降落 | `yaw_angle_enabled=0`(偏航仅角速度环)、`height_vel_only=1`(高度速度环直控)、水平速度环 V=0、Ki 清零、`PIDVelH.target=-0.2`、保存原始角速度/速度环 Kp |
| 阶段1 Step 1 | `pz > 0.3m` | 高度速度环直控 `-0.2m/s` 下降，水平速度环 V=0 定点 |
| 阶段2 Step 2 | `pz ≤ 0.3m` | 角速度 Kp+0.7、水平速度环 Kp+1；**冻结当前总油门** `takeoff_base_thr += PIDVelH.out`，**2s 线性衰减至 0**；高度环/速度环积分清零防反向修正 |
| 接地检测 | 任意 | `\|vz\|<0.1m/s` + `of.height<0.2m` 持续 100ms (20次) → 回 `STATE_LOCK` |

---

### `stabilization()` 控制流程（200Hz）

```
高度控制
  ├─ height_vel_only=0 → 高度外环 PIDHeight + 速度内环 PIDVelH 串级（正常飞行/起飞）
  └─ height_vel_only=1 → 跳过高度外环，直控速度内环 PIDVelH（降落阶段1）

油门 = takeoff_base_thr + PIDVelH.out
  └─ 电池电压补偿（可选）: THR = takeoff_base_thr * Battery_GetThrottleComp() + PIDVelH.out

起飞油门斜坡（takeoff_phase=1 时）
  3500 → HOVER_THRUST(4450)，1.5s 线性上升

位置/速度环 → 角度目标（40Hz 分频，stabilization 内统一管理）
  ├─ POSITION_HOLD=0 → 目标角度=0（纯自稳）
  ├─ POSITION_HOLD=1 → 视觉追车（car_found + pos_ctrl_enabled 门控）
  │   ├─ 有信标: 位置外环(POS_KP=0.03) → 速度内环(POS_VEL_KP=6.0, KI=0.05) → 角度目标
  │   ├─ 无信标: 速度环 V=0 保持定点
  │   └─ 角速度前馈 ANG_RATE_FF_GAIN=0.021 平铺到200Hz
  └─ POSITION_HOLD=2 → 速度环 V=0 纯定点

姿态控制（完整串级 PID）
  ├─ Roll/Pitch: 角度环(测量微分) + 角速度环(误差微分)
  │   Roll:  PIDRoll(KP=3.0) → PIDVelX(KP=5.9, KI=2.3, KD=0.5)
  │   Pitch: PIDPitch(KP=3.0) → PIDVelY(KP=5.9, KI=2.3, KD=0.5)
  ├─ Yaw:
  │   ├─ yaw_angle_enabled=0 → 仅角速度环 target=0（起飞/降落，防磁力计自旋）
  │   └─ yaw_angle_enabled=1 → 角度环PIDYaw(KP=4.0) + 角速度环PIDVelZ(KP=9.0, KI=3.0)
  └─ 起飞阶段(takeoff_phase=1)禁用角速度前馈

电机混控（X型，int16_t 有符号域计算防溢出）
  m1 = THR + Roll - Pitch - Yaw
  m2 = THR - Roll - Pitch + Yaw
  m3 = THR - Roll + Pitch - Yaw
  m4 = THR + Roll + Pitch + Yaw
  输出限幅: MOTOR_MIN_DUTY(2800) ~ MOTOR_MAX_DUTY(7500)
```

#### 视觉追车控制（串级PID）
```
图像像素误差 → 位置外环(POS_KP=0.03, KI=0.001) → 速度目标(m/s)
像素速度前馈(动态衰减) → 加到速度目标
速度内环(POS_VEL_KP=6.0, KI=0.05) → 角度目标(±30°限幅)
角度环(Roll/Pitch KP=3.0) + 角速度前馈(0.021) → 角速度目标
角速度环(KP=5.9, KI=2.3, KD=0.5) → 电机混控
```

#### 像素速度前馈
```
vel_x = (car_x - prev_car_x) * 40.0f;           // 像素/帧 → 像素/秒 (@40Hz)
speed = sqrtf(vx² + vy²);                         // 飞机当前速度
ff_scale = FF_SCALE_MAX * (1.0 - speed / 0.8);   // 动态比例 (0.8m/s时归零)
ff_vel_x = vel_x * ff_scale;                      // 前馈量
PIDPosX_Vel.target = PIDPosX.out + ff_vel_x;     // 加到速度环目标
```
保护机制：
- **跳变滤除**：相邻帧 `|dx| > 20px` 时清零前馈
- **衰减值屏蔽**：`car_fresh == 0` 时清零前馈
- **速度衰减**：飞机速度 > 0.8m/s 时线性衰减到 0

#### 丢帧保护 (cm7_0_isr.c)
```
frame_id 被视觉核更新 → 正常: filt_err_x/y 使用当前值
frame_id 连续 > 100帧(25Hz≈4s) 未更新 → 每帧 × FRAME_DROP_DECAY(0.94)
→ 3秒内误差衰减到 ≈ 1%
```

#### 10s 自动降落逻辑
- 变量 `g_no_beacon_timer` 在 STATE_FLY 中每周期累加 `dt`
- 满 10s → 设置 `g_car_flag3_pending=1`（主循环发送 `#0,0,3# 智能无人机飞控项目 Code Wiki

## 目录
1. [项目概述](#项目概述)
2. [项目架构](#项目架构)
3. [主要模块说明](#主要模块说明)
4. [核心函数说明](#核心函数说明)
5. [依赖关系](#依赖关系)
6. [运行方式](#运行方式)
7. [配置说明](#配置说明)
8. [版本历史](#版本历史)

---

## 项目概述

### 项目简介
这是一个基于英飞凌(Infineon) Traveo II MCU的双核四旋翼无人机飞控系统，采用X型四旋翼布局。项目使用智飞科技的开发框架，实现了完整的无人机姿态控制、高度控制、位置保持、自动起降和**视觉识别追踪**功能。上位机工具链支持实时数据可视化与摄像头画面显示。

### 核心功能
- **姿态稳定控制** - 串级PID控制（角度外环+角速度内环）
- **高度控制** - 基于TOF激光测距的定高飞行
- **位置保持** - 基于光流传感器+IMU融合的定点悬停
- **视觉识别** - 红外摄像头检测V形车标、信标和车体碎片
- **目标追踪** - 识别小车位置和航向，引导无人机追踪
- **碎片追车** - V形丢失时用方向长宽比>150的碎片平均中心作为替代
- **动态前馈** - 像素速度前馈随飞机速度自适应衰减（0.8m/s归零）
- **自动起降** - 一键起飞和自动降落
- **上位机可视化** - Python实时监控，支持串口数据+摄像头双路显示
- **IPS屏幕显示** - 实时显示二值化图像和飞行数据

### 硬件平台
- **主控芯片**: 英飞凌Traveo II (CYT4BB系列，双核ARM Cortex-M7)
  - CM7_0: 控制核 (姿态控制、传感器读取)
  - CM7_1: 视觉核 (图像处理、目标识别)
- **姿态传感器**: IMU660RC (陀螺仪+加速度计)
- **磁力计**: QMC5883L
- **高度传感器**: TOF激光测距模块
- **位置传感器**: UPFLOW302 光流传感器
- **视觉传感器**: MT9V03x 红外摄像头 (188×120，通过SmartIO+DMA采集)
- **显示**: IPS200屏幕
- **电机驱动**: 小型驱动模块（UART通信）
- **蓝牙**: HC06串口透传（接收小车指令、发送误差数据）

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
│   │   ├── HC06_Driver.c/h # 蓝牙驱动
│   │   ├── INIT.c/h      # 系统初始化
│   │   └── config.h      # 全局配置
│   ├── user/             # 用户代码 (main入口)
│   │   ├── main_cm7_0.c  # CM7_0: 控制核入口
│   │   ├── main_cm7_1.c  # CM7_1: 视觉核入口
│   │   └── cm7_0_isr.c   # CM7_0 PIT中断服务
│   ├── check/            # 测试代码副本
│   └── iar/              # IAR工程文件
├── tools/                 # 工具脚本
│   ├── realtime_monitor.py # 上位机实时监控
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
│  摄像头 / IPS屏幕 / HC06蓝牙             │
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
| **CM7_0** (控制核) | main_cm7_0.c | 姿态控制、传感器读取、电机输出、HC06通信 |
| **CM7_1** (视觉核) | main_cm7_1.c | 图像采集、视觉识别、Blob分析、IPS显示 |

**双核通信**：通过共享内存 `vision_share_t`（地址 `0x28001000`）交换数据。
- CM7_1 写入识别结果并刷新 D-Cache
- CM7_0 读取视觉数据进行位置控制
- 双核均调用 `SCB_DisableDCache()` 禁用D-Cache，避免缓存一致性问题

### 控制环路架构

| 控制环路       | 频率       | 控制器         | 说明                      |
|----------------|------------|----------------|---------------------------|
| 角速度内环     | 200Hz      | PIDVelX/Y/Z    | Roll/Pitch/Yaw角速度控制  |
| 姿态外环       | 200Hz      | PIDRoll/Pitch/Yaw | Roll/Pitch/Yaw角度控制 |
| 高度外环       | 200Hz      | PIDHeight      | 高度→目标垂直速度         |
| 高度内环       | 200Hz      | PIDVelH        | 垂直速度→油门修正         |
| 位置/速度环    | 40Hz       | PIDPosX/Y + Vel | 位置→速度→角度 (分频5) |
| 视觉识别       | 50Hz       | 软判决评分      | 每帧图像处理+目标识别     (MT9V03X_FPS_DEF=50) |
| HC06蓝牙通信   | 100Hz      | cm7_0_isr.c    | 发送误差、接收指令         |

---

## 主要模块说明

### 1. 主程序模块

#### CM7_0 (控制核) - main_cm7_0.c
**功能**: 系统主入口，负责控制、传感器、HC06通信调度

**主要流程**:
1. 时钟初始化 (250MHz)
2. INTT初始化（NVIC优先级分组）
3. `ALL_Init()` 全局初始化
4. 设置PID目标值
5. 主循环：
   - 按键检测处理
   - 电机控制输出
   - `HC06_Task()` 蓝牙指令收发
   - 飞行状态机调度 (`flight_control`)

#### CM7_1 (视觉核) - main_cm7_1.c
**功能**: 视觉处理入口，负责图像采集和IPS显示

**主要流程**:
1. 时钟初始化
2. `camera_init()` 摄像头初始化
3. 主循环：
   - `camera_process()` 图像处理
   - IPS屏幕显示（二值化图像 + 飞行数据覆盖）
   - 识别结果写入共享内存，刷新 D-Cache

---

### 2. 视觉识别模块 (camera.c / camera.h)

**功能**: 红外摄像头图像采集、Blob分析、V形车标检测、信标识别、碎片追车

#### 工作流程 (每帧 25Hz)
```
DMA读取灰度图 → 二值化 → BFS连通域提取(extract_blobs)
→ 原始分数计算(soft_decision_scores) → 跨帧跟踪(cross_frame_track)
→ 分数平滑与判决(smooth_and_decide) → V形轮廓内信标清除
→ 碎片追车(found_car=0且碎片存在时) → 姿态解算 → 共享内存刷新
```

#### 图像预处理
- 摄像头: MT9V03x 红外摄像头
- 分辨率: 188 × 120
- 二值化阈值: `IR_THRESHOLD = 140`
- 核心高亮阈值: `IR_CORE_THRESHOLD = 200`

#### 连通域提取 (extract_blobs)
- 算法: 4邻域BFS（广度优先搜索）
- 输入: 二值化图像
- 输出: Blob结构体数组（最多20个）
- 每个Blob属性包含质心、外接矩形、面积、核心高亮数、方向矩等

#### V形车标检测 (compute_car_marker_feature)

**几何原理**: 小车顶部的V形标记（箭头），尖端指向车头

```
         A (角点/顶点)
        / \
       /   \
      /     \
     B-------C (底边两端)
        |
       G (三角形重心 = (A+B+C)/3 = 车中心)
```

**检测步骤**:
1. **预过滤**: 面积 [20, 900]，长宽比 ≥ 1.35
2. **极值点**: 8个方向的最远点
3. **找底边 BC**: 候选点中距离最远的一对
4. **找顶点 A**: 到直线BC垂直距离最远的像素
5. **几何验证**: 底边≥8px, 臂长≥6px, 垂距≥5px, 夹角[12°, 90°]

#### 软判决评分系统

每个Blob计算三种分数：

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

**车头分** 和 **车尾分**: 用于V形以外的辅助分类（目前由V形检测主导）

#### 跨帧跟踪 (cross_frame_track)
- 当前帧Blob与上一帧按**质心距离**匹配（阈值25像素）
- 匹配上的继承历史track_id和滤波分数
- 分数一阶滞后滤波：`filt = 0.65×历史 + 0.35×当前`

#### 碎片追车逻辑
当V形车标未识别到 (`found_car == 0`) 时：

1. **V形检测到**:
   - `car_found = 1, car_fresh = 1`
   - 更新保留坐标到当前检测值

2. **V形丢失 → 碎片追车**:
   - 遍历所有blob，筛选**方向长宽比 > 150**的碎片
   - 排除已被标记为 `BLOB_CAR_MARKER` 的blob
   - 面积 ≥ `CAR_MARK_MIN_AREA(20)`
   - 计算符合条件的碎片中心平均值 → 作为小车坐标
   - `car_found = 1, car_fresh = 1`

3. **无碎片 → 衰减归中**:
   - 每帧按 `CAR_RETAIN_DECAY(0.90)` 衰减到图像中心 (94, 60)
   - 公式: `retain = (retain - center) * 0.90 + center`
   - 衰减到中心附近 ±0.5px 时，`car_found = 0, car_fresh = 0`

#### V形轮廓内信标清除
检测到V形车标时，以车标blob的外接矩形 +1px 为区域，区域内所有 `BLOB_BEACON` 类型强制设为 `BLOB_UNKNOWN`，防止V形内部误识别伪信标。

#### 姿态解算 (find_car_pose_by_marker)
- **车中心** = 三角形重心 (A+B+C)/3 → `(car_x, car_y)`
- **车头方向** = D→A 向量角度 → `heading_angle`
- 航向角一阶低通滤波平滑（α=0.7）
- 信标-小车误差转换到机体坐标系传给CM7_0

---

### 3. 共享内存结构 (vision_share_t)

```c
typedef struct {
    uint32 frame_id;            // 帧序号

    int16 err_x, err_y;         // 信标相对小车的机体坐标系误差 (前后/左右)
    uint16 target_found;        // 同时识别到信标和小车

    int16 beacon_err_x, beacon_err_y; // 信标相对画面中心的误差
    uint16 beacon_found;        // 信标存在标志

    int16 car_x, car_y;         // 小车中心坐标 (画面坐标系)
    uint16 car_found;           // 小车存在标志
    uint16 car_fresh;           // 1=真实检测/碎片, 0=衰减保留值 (前馈用)

    float heading_angle;        // 滤波后小车航向角 (弧度)

    // IPS 显示数据段 (CM7_0 写入, CM7_1 读取)
    float  disp_roll, disp_pitch, disp_yaw;
    uint16 disp_m1, disp_m2, disp_m3, disp_m4;
    float  disp_mag_x, disp_of_dx, disp_world_vx;
    float  disp_world_vy;       // 世界坐标系 Y 轴速度
    float  disp_imu_gx, disp_of_height, disp_target;
    float  disp_volt;
    float  vel_tgt_x, vel_tgt_y; // 速度环目标 (上位机显示)
    float  ff_vel_x, ff_vel_y;    // 像素前馈量 (上位机显示)
    uint32 disp_dirty;

    // 视觉调试数据
    uint16 debug_car_score;
    int16  debug_car_angle;
    uint8  debug_car_method;
    uint8  debug_car_track_id;
} vision_share_t;
```

---

 给小车）→ 切 STATE_LAND
- 在 STATE_LOCK 中复位计时器

#### 偏航控制策略
| 阶段 | 控制模式 | 目标值 |
|------|----------|--------|
| 起飞(0~1m) | 仅角速度环 | `PIDVelZ.target=0`（不自旋）|
| 爬升(≥1m) | 角度环+角速度环 | `PIDYaw.target = 当前eulerAngle.yaw`（锁当前航向）|
| 飞行 | 角度环+角速度环 | `PIDYaw.target = 锁定值`（保持航向）|
| 降落 | 仅角速度环 | `PIDVelZ.target=0`（不自旋）|

---

### 5. 光流模块 (of.c / of.h)

**功能**: UPFLOW302 光流传感器数据处理 + 光流-IMU速度融合

#### 光流测速原理
```
角速度 ω = Δx / (dt × K)                     // 像素位移 → 角速度
线速度 v = ω × h                              // 角速度 × 高度 → 线速度
```

#### 陀螺仪补偿
无人机旋转时，光流会测到额外的"虚假"运动，需用陀螺仪数据抵消：
```
ω_true = ω_optical + ω_gyro
```

#### velocity_mahony_fusion
采用类Mahony互补滤波融合光流和加速度计：
```
预测: v_pred += a_imu × dt        (加速度积分)
误差: e = v_of - v_pred           (光流观测 - 积分预测)
校正: v_fusion = v_pred + Kp×e + Ki×∫e·dt
```
- 起飞阶段(Ki=0)避免起飞时积分累积

---

### 6. PID控制模块 (PID.c / PID.h)

#### PID参数总表

##### 内环角速度PID
| 控制器 | Kp | Ki | Kd | 积分限幅 | 输出限幅 |
|--------|----|----|----|----------|----------|
| PIDVelX (Roll) | 5.9 | 2.3 | 0.5 | ±800 | ±2000 |
| PIDVelY (Pitch) | 5.9 | 2.3 | 0.5 | ±800 | ±2000 |
| PIDVelZ (Yaw) | 9.0 | 3.0 | 0.0 | ±800 | ±1500 |

##### 外环角度PID
| 控制器 | Kp | Ki | Kd | 积分限幅 | 输出限幅 |
|--------|----|----|----|----------|----------|
| PIDRoll | 3.0 | 0 | 0 | 0 | ±200 |
| PIDPitch | 3.0 | 0 | 0 | 0 | ±200 |
| PIDYaw | 4.0 | 0 | 0 | 0 | ±150 |

##### 高度环PID
| 控制器 | Kp | Ki | Kd | 积分限幅 | 输出限幅 |
|--------|----|----|----|----------|----------|
| PIDHeight (外环) | 1.0 | 0 | 0 | ±150 | ±0.35 |
| PIDVelH (内环) | 750.0 | 40.0 | 0 | ±100 | ±600 |

##### 位置/速度环PID (视觉追车)
| 控制器 | Kp | Ki | Kd | 积分限幅 | 输出限幅 |
|--------|----|----|----|----------|----------|
| PIDPosX/Y (位置) | 0.03 | 0.001 | 0 | 积分钳位 | ±1.5 m/s |
| PIDPosX_Vel/Y_Vel (速度) | 6.0 | 0.05 | 0 | ±100 | ±30° |

---

### 7. 调试与可视化

#### 上位机实时监控 (tools/realtime_monitor.py)
- **功能**：串口数据可视化 + 实时摄像头画面
- **数据包格式**：
  - **H包** (19字段)：帧ID、car_x/y、信标坐标、高度、航向、vx/vy、vel_tgt、ff_vel 等
  - **I包** (10字段)：简版数据（高度、航向、roll/pitch、vel_tgt）
- **画面布局**：左侧1份机体坐标图，右侧2.5份摄像头画面（1:2.5比例）
- **信息卡片**：显示实时vx/vy、速度环目标、前馈量

#### IPS屏幕显示 (CM7_1)
- 模式0 (正常)：显示二值化图像 + 飞行数据覆盖
  - 姿态角 (roll/pitch/yaw)
  - 电机值 (m1~m4)
  - 世界速度 (vx/vy)
  - 速度环目标 (tvx/tvy)
  - 磁力计值、IMU陀螺仪数据
- 模式1 (调试)：纯数据文本

#### HC06蓝牙通信 (CM7_0)
- `#1$` 起桨、`#2$` 起飞、`#3$` 停止
- `#err_x,err_y,flag$` 发送视觉误差给地面小车
- 100Hz ISR中发送，flag=1(正常)/2(丢帧)

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

### 上位机
```powershell
cd D:\UsersASUSDesktop\Vision and Drones
python tools\realtime_monitor.py COM25 115200 --camera
```
- 依赖：`pip install pyserial matplotlib opencv-python`
- 串口数据 + USB摄像头双路实时显示

### 调试输出格式
```
H,frame_id,car_x,car_y,b1x,b1y,b2x,b2y,b_ext,angle,height,heading,vx,roll,pitch,vtgt_x,vtgt_y,vy,ff_x,ff_y$
I,frame_id,height,heading,0,roll,pitch,vtgt_x,vtgt_y,vy$
B,frame_id,x,y,found$
```

---

## 配置说明

### 关键参数 (config.h)

| 宏 | 默认值 | 说明 |
|----|--------|------|
| `TAKEOFF_TARGET_HEIGHT` | 1.3m | 起飞目标高度 |
| `HOVER_THRUST` | 4450 | 悬停油门 |
| `CAR_RETAIN_DECAY` | 0.90 | 小车坐标丢失后每帧衰减系数 |
| `FRAME_DROP_DECAY` | 0.94 | 视觉帧丢失后误差衰减系数 |
| `FF_SCALE_MAX` | 0.006 | 像素前馈最大增益 |
| `FF_SPEED_CEIL` | 0.8 m/s | 前馈衰减截止速度 |
| `PIXELS_PER_DEG` | 1.3 | 像素/度 (姿态补偿) |

### 视觉参数 (camera.h)

| 宏 | 默认值 | 说明 |
|----|--------|------|
| `IR_THRESHOLD` | 140 | 二值化阈值 |
| `CAR_MARK_MIN_AREA` | 20 | V形候选最小面积 |
| `CAR_MARK_MAX_AREA` | 900 | V形候选最大面积 |
| `CAR_MARK_MIN_SCORE` | 55 | V形分类最低分数 |
| `ERR_DECAY_FACTOR` | 0.85 | 误差衰减系数 |
| `HEADING_FILTER_ALPHA` | 0.7 | 航向滤波系数 |

---

## 版本历史

| 版本 | Tag | 主要变更 |
|------|-----|---------|
| v5.4.0 | `v5.4.0` | 首次到达1m锁偏航角，磁力计手动抑制 |
| v5.3.0 | `v5.3.0` | 碎片追车+前馈动态衰减+坐标衰减改向 |
| v5.1.0 | `v5.1.0` | 更新小车速度计算前馈 |
| v5.0.0 | `v5.0.0` | 完赛代码，只待调参 |
| v4.0.0 | `v4.0.0` | 更新视觉逻辑，完整识别 |
| v3.1.0 | `v3.1.0` | BFS+Hough双验证+端点交换修复 |
| v3.0.0 | `v3.0.0` | V形检测参数调优 |
| v2.0.0 | `v2.0.0` | HC06双向通信+VL53L5CX修复 |
| v1.2.0 | `v1.2.0` | Hough方向修正+视觉位置控制 |
