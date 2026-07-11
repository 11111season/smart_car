# VL53L5CX ToF 传感器驱动技术文档

## 1. 概述

本文档基于 STMicroelectronics X-CUBE-TOF1 软件包（版本 3.4.0），详细说明 **VL53L5CX** 多区 Time-of-Flight（ToF）测距传感器在 STM32F4 平台上的驱动原理、软件架构、接口说明及使用流程。

VL53L5CX 是一款先进的多区 ToF 传感器，可提供 4x4（16区）或 8x8（64区）分辨率的同时测距，最远测距距离可达 4 米。

---

## 2. 硬件接口

### 2.1 引脚定义

以下引脚定义位于 `Target/custom_tof_conf.h` 和 `Target/app_tof_pin_conf.h`：

| 信号名称 | 端口 | 引脚 | 功能描述 |
|---------|------|------|---------|
| PWR_EN | GPIOA | Pin 6 | 传感器电源使能，高电平供电 |
| I2C_RST | GPIOA | Pin 7 | I2C 总线复位 |
| LPn | GPIOB | Pin 0 | 低功耗模式控制（高电平=正常模式） |
| INT | GPIOA | Pin 4 | 中断输出（EXTI），传感器就绪时触发 |
| I2C1 SCL | I2C1 | — | I2C 时钟线 |
| I2C1 SDA | I2C1 | — | I2C 数据线 |

### 2.2 硬件接线示意图

```
STM32F4                          VL53L5CX
┌──────┐                        ┌────────┐
│ PA6  ├────────────────────────┤ PWR_EN │
│ PA7  ├────────────────────────┤ I2C_RST│
│ PB0  ├────────────────────────┤ LPn    │
│ PA4  ├────────────────────────┤ INT    │
│ I2C1 ├────────────────────────┤ SDA/SCL│
└──────┘                        └────────┘
```

### 2.3 I2C 通信

传感器通过 I2C1 总线与主控通信。地址定义：

```c
#define RANGING_SENSOR_VL53L5CX_ADDRESS  (VL53L5CX_DEVICE_ADDRESS)
```

底层 I2C 读写函数通过宏映射：

```c
#define CUSTOM_VL53L5CX_I2C_INIT      BSP_I2C1_Init
#define CUSTOM_VL53L5CX_I2C_DEINIT    BSP_I2C1_DeInit
#define CUSTOM_VL53L5CX_I2C_WRITEREG  BSP_I2C1_WriteReg16
#define CUSTOM_VL53L5CX_I2C_READREG   BSP_I2C1_ReadReg16
```

---

## 3. 软件架构

### 3.1 文件结构

```
TOF/
├── App/
│   ├── app_tof.c              # 应用层主逻辑
│   └── app_tof.h              # 应用层头文件
├── Target/
│   ├── custom_tof_conf.h      # TOF 硬件配置（引脚、I2C 宏定义）
│   ├── app_tof_pin_conf.c     # 中断回调函数
│   ├── app_tof_pin_conf.h     # 中断引脚定义
│   ├── custom_ranging_sensor.c  # BSP 层驱动接口实现
│   ├── custom_ranging_sensor.h  # BSP 层驱动接口声明与数据结构
│   └── ranging_sensor.h       # 通用测距传感器驱动结构体
```

### 3.2 分层架构

```
┌─────────────────────────────────────┐
│         应用层 (app_tof.c)          │
│  MX_TOF_Init() / MX_TOF_Process()  │
├─────────────────────────────────────┤
│       BSP 层 (custom_ranging_*)     │
│   CUSTOM_RANGING_SENSOR_xxx()       │
├─────────────────────────────────────┤
│       组件驱动 (vl53l5cx 库)        │
│   VL53L5CX_Init / Start / ...       │
├─────────────────────────────────────┤
│      HAL 层 (STM32F4 HAL)          │
│   I2C / GPIO / EXTI / UART          │
└─────────────────────────────────────┘
```

---

## 4. 关键数据结构

### 4.1 传感器能力

```c
typedef struct {
  uint32_t NumberOfZones;          // 测距区数量
  uint32_t MaxNumberOfTargetsPerZone; // 每区最大目标数
  uint32_t CustomROI;              // 是否支持自定义 ROI
  uint32_t ThresholdDetection;     // 是否支持阈值检测
} RANGING_SENSOR_Capabilities_t;
```

### 4.2 测距配置

```c
typedef struct {
  uint32_t RangingProfile;   // 测距模式（4x4/8x8，连续/自主）
  uint32_t TimingBudget;     // 积分时间（ms，范围 5~100ms）
  uint32_t Frequency;        // 测距频率（Hz）
  uint32_t EnableAmbient;    // 使能环境光输出（1=启用）
  uint32_t EnableSignal;     // 使能信号质量输出（1=启用）
} RANGING_SENSOR_ProfileConfig_t;
```

### 4.3 测距结果

```c
typedef struct {
  uint32_t NumberOfTargets;                                    // 检测到的目标数
  uint32_t Distance[RANGING_SENSOR_NB_TARGET_PER_ZONE];       // 距离值（毫米）
  uint32_t Status[RANGING_SENSOR_NB_TARGET_PER_ZONE];         // 状态（0=有效）
  float_t Ambient[RANGING_SENSOR_NB_TARGET_PER_ZONE];         // 环境光（kcps/spad）
  float_t Signal[RANGING_SENSOR_NB_TARGET_PER_ZONE];          // 信号（kcps/spad）
} RANGING_SENSOR_ZoneResult_t;

typedef struct {
  uint32_t NumberOfZones;
  RANGING_SENSOR_ZoneResult_t ZoneResult[RANGING_SENSOR_MAX_NB_ZONES];
} RANGING_SENSOR_Result_t;
```

### 4.4 测距模式常量

| 宏定义 | 值含义 |
|-------|--------|
| `RS_PROFILE_4x4_CONTINUOUS` | 4x4 分辨率，连续测距 |
| `RS_PROFILE_4x4_AUTONOMOUS` | 4x4 分辨率，自主模式（低功耗） |
| `RS_PROFILE_8x8_CONTINUOUS` | 8x8 分辨率，连续测距 |
| `RS_PROFILE_8x8_AUTONOMOUS` | 8x8 分辨率，自主模式（低功耗） |
| `RS_MODE_BLOCKING_CONTINUOUS` | 阻塞式连续模式 |
| `RS_MODE_BLOCKING_ONESHOT` | 阻塞式单次模式 |
| `RS_MODE_ASYNC_CONTINUOUS` | 异步连续模式（中断驱动） |
| `RS_MODE_ASYNC_ONESHOT` | 异步单次模式（中断驱动） |

---

## 5. API 接口说明

### 5.1 初始化与反初始化

| 函数 | 描述 |
|------|------|
| `CUSTOM_RANGING_SENSOR_Init(Instance)` | 初始化传感器（复位→探测→初始化） |
| `CUSTOM_RANGING_SENSOR_DeInit(Instance)` | 反初始化传感器 |

**初始化内部流程**：
1. `reset_device()` — 硬件复位：
   - PWR_EN 拉低 2ms → 拉高 2ms（电源复位）
   - LPn 拉低 2ms → 拉高 2ms（低功耗模式复位）
2. `VL53L5CX_Probe()` — 传感器探测：
   - 注册 I2C 读写函数
   - 读取设备 ID（校验 `VL53L5CX_ID`）
   - 调用传感器初始化
   - 获取能力信息

### 5.2 设备管理

| 函数 | 描述 |
|------|------|
| `CUSTOM_RANGING_SENSOR_ReadID(Instance, pId)` | 读取设备 ID |
| `CUSTOM_RANGING_SENSOR_GetCapabilities(Instance, pCap)` | 获取传感器能力 |
| `CUSTOM_RANGING_SENSOR_SetAddress(Instance, Address)` | 设置 I2C 地址 |
| `CUSTOM_RANGING_SENSOR_GetAddress(Instance, pAddress)` | 获取 I2C 地址 |
| `CUSTOM_RANGING_SENSOR_SetPowerMode(Instance, PowerMode)` | 设置电源模式 |
| `CUSTOM_RANGING_SENSOR_GetPowerMode(Instance, pPowerMode)` | 获取电源模式 |

### 5.3 测距控制

| 函数 | 描述 |
|------|------|
| `CUSTOM_RANGING_SENSOR_ConfigProfile(Instance, pConfig)` | 配置测距参数 |
| `CUSTOM_RANGING_SENSOR_Start(Instance, Mode)` | 启动测距 |
| `CUSTOM_RANGING_SENSOR_Stop(Instance)` | 停止测距 |
| `CUSTOM_RANGING_SENSOR_GetDistance(Instance, pResult)` | 获取测距结果 |
| `CUSTOM_RANGING_SENSOR_ConfigROI(Instance, pConfig)` | 配置感兴趣区域（可选） |
| `CUSTOM_RANGING_SENSOR_ConfigIT(Instance, pConfig)` | 配置中断阈值（可选） |

---

## 6. 典型使用流程

### 6.1 基本流程

```
系统启动
    │
    ├─ 初始化串口 ──→ BSP_COM_Init(COM1)
    │
    ├─ 初始化 TOF ──→ MX_TOF_Init()
    │                     │
    │                     └─ CUSTOM_RANGING_SENSOR_Init()
    │                           ├─ reset_device()    // 硬件复位
    │                           └─ VL53L5CX_Probe()  // 探测传感器
    │
    └─ 主循环 ─────→ MX_TOF_Process()
                          │
                          ├─ 读取设备 ID
                          ├─ 获取能力信息
                          ├─ 配置测距参数 (4x4, 30ms, 5Hz)
                          ├─ 启动测距
                          └─ 循环：
                                ├─ 获取距离数据
                                ├─ 打印结果
                                ├─ 处理用户命令
                                └─ 延时等待
```

### 6.2 代码示例

```c
// 主函数
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();

    MX_TOF_Init();      // 初始化 TOF 传感器

    while (1)
    {
        MX_TOF_Process();  // 持续测距
    }
}
```

### 6.3 配置参数建议

默认推荐参数（见 `app_tof.c`）：

```c
#define TIMING_BUDGET     (30U)    // 积分时间 30ms，范围 5~100ms
#define RANGING_FREQUENCY (5U)     // 测距频率 5Hz
#define POLLING_PERIOD    (200U)   // 轮询周期 200ms
```

**参数选择指南**：
- **TimingBudget（积分时间）**：越大精度越高，但帧率降低。短距离（<1m）可用 10~30ms，长距离（>3m）建议 50~100ms。
- **Frequency（频率）**：需与 TimingBudget 匹配，确保 `1000/Frequency > TimingBudget`。
- **分辨率**：4x4（16区）帧率更高，8x8（64区）空间分辨率更高但帧率较低。

---

## 7. 中断驱动模式

在异步模式下使用中断，需注意以下机制：

### 7.1 中断引脚

INT 引脚（PA4）配置为 EXTI 外部中断，当传感器有新数据就绪时触发。

### 7.2 中断回调

```c
// app_tof_pin_conf.c
volatile uint8_t ToF_EventDetected = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == TOF_INT_EXTI_PIN)  // GPIO_PIN_4
  {
    ToF_EventDetected = 1;
  }
}
```

### 7.3 异步模式启动

启动时使用非阻塞模式：

```c
CUSTOM_RANGING_SENSOR_Start(CUSTOM_VL53L5CX, RS_MODE_ASYNC_CONTINUOUS);
```

主循环中检查 `ToF_EventDetected` 标志位，在中断触发后调用 `GetDistance` 获取数据。

---

## 8. 用户交互命令

通过串口输入以下字符控制传感器行为：

| 按键 | 功能 | 实现函数 |
|------|------|---------|
| `r` | 切换分辨率 4x4 ↔ 8x8 | `toggle_resolution()` |
| `s` | 切换信号/环境光输出 | `toggle_signal_and_ambient()` |
| `c` | 清屏 | `clear_screen()` |

---

## 9. 常见问题排查

### 9.1 初始化失败

| 错误原因 | 检查项 |
|---------|--------|
| I2C 通信异常 | 硬件连接是否正确？上拉电阻是否安装？ |
| 设备 ID 不匹配 | VL53L5CX 是否供电正常？I2C 地址是否正确？ |
| 复位时序错误 | PWR_EN/LPn 控制引脚是否正确配置？ |

### 9.2 测距异常

| 现象 | 可能原因 |
|------|---------|
| 所有区返回 X/X | 传感器未检测到目标，或目标超出范围 |
| 距离值跳变异常 | 环境光干扰、目标表面反射率低 |
| 测距频率不符合预期 | TimingBudget 与 Frequency 参数不匹配 |
| 部分区数据无效 | 目标在 ROI 边缘、遮挡或不均匀反射面 |

### 9.3 调试建议

1. 通过串口观察初始化阶段的打印信息
2. 使用逻辑分析仪抓取 I2C 总线波形，确认通信是否正常
3. 先使用 4x4 连续模式验证基础功能，再切换到 8x8 模式
4. 注意 VL53L5CX 的盖板玻璃会增加固定的偏移量

---

## 10. 参考资源

- STMicroelectronics VL53L5CX 数据手册
- X-CUBE-TOF1 软件包文档
- STM32F4 HAL 库参考手册
