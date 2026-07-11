# VL53L5CX TOF 传感器驱动库生成计划

## 摘要

为 VL53L5CX 多区 ToF 传感器编写驱动库，仿照逐飞（SEEKFREE）`zf_device_dl1b` 的库风格，生成 `zf_device_vl53l5cx.c` 和 `zf_device_vl53l5cx.h` 两个文件，放入 `project/code/` 文件夹。

---

## 当前状态分析

### 技术文档依据
- 文档 `libraries/doc/TOF驱动技术文档.md` 描述了 VL53L5CX 在 STM32F4 平台上的驱动架构：
  - 使用 I2C (16位寄存器地址) 通信
  - 4x4 (16区) 或 8x8 (64区) 分辨率
  - 测距距离 0~4m
  - 需要 PWR_EN、I2C_RST、LPn 三个控制引脚 + INT 中断引脚
  - 分层架构：应用层 → BSP层 → 组件驱动层 → HAL层，引脚采用原tof使用的引脚

### 现有代码结构
- `libraries/zf_device/zf_device_dl1b.c/h` — 逐飞风格的 DL1B TOF 驱动器（目标风格参考）
- `project/code/TOF.c/h` — 已有的应用层高度融合代码（使用 dl1b_distance_mm）先不进行融合，先检查原始数据是否正常再做判断
- `libraries/zf_driver/zf_driver_soft_iic.h` — 软件 I2C 驱动 API（支持16位寄存器读写）
- `libraries/zf_device/zf_device_type.h` — 设备类型枚举（含 TOF_DL1A / TOF_DL1B）
- `libraries/zf_device/zf_device_config.h` — 配置文件声明（含 dl1b_config_file 等）

### 逐飞库风格特征
1. 文件头版权声明（GPL3.0，逐飞科技）
2. 接线定义注释块（模块管脚 ↔ 单片机管脚）
3. `#include "zf_common_typedef.h"` 使用逐飞类型
4. 宏定义区域分类清晰：`=====硬件IIC=====` / `=====内部地址=====` / `=====全局变量=====` / `=====基础函数=====`
5. 函数注释格式：`函数简介` / `参数说明` / `返回参数` / `使用示例` / `备注信息`
6. 全局变量用 `extern` 导出，函数返回 `uint8`（0=成功，1=失败）

---

## 拟变更的文件

### 新建文件 1: `project/code/zf_device_vl53l5cx.h`

**目的**：VL53L5CX 驱动头文件，定义引脚、寄存器地址、API声明。

**内容**：
1. **逐飞风格文件头**（版权声明、接线定义）
2. **引脚宏定义**：
   - `VL53L5CX_SCL_PIN` — I2C SCL 引脚（默认 P19_0，与 DL1B 一致）
   - `VL53L5CX_SDA_PIN` — I2C SDA 引脚（默认 P19_1）
   - `VL53L5CX_PWR_EN_PIN` — 电源使能引脚（默认 P13_0，用户可改）
   - `VL53L5CX_I2C_RST_PIN` — I2C 复位引脚（默认 P13_1）
   - `VL53L5CX_LPn_PIN` — 低功耗模式引脚（默认 P13_2）
   - `VL53L5CX_INT_PIN` — 中断引脚（默认 P13_3）
3. **I2C 配置宏**：
   - `VL53L5CX_USE_SOFT_IIC` — 默认使用软件 I2C
   - `VL53L5CX_SOFT_IIC_DELAY` — I2C 速率延时
   - `VL53L5CX_DEV_ADDR` — I2C 设备地址 (`0x52 >> 1`)
   - `VL53L5CX_INT_ENABLE` — 中断使能（默认关闭）
4. **寄存器地址宏定义**（VL53L5CX 关键寄存器）：
   - `VL53L5CX_SOFT_RESET` (0x0000)
   - `VL53L5CX_DEVICE_ID` (0x010F)
   - `VL53L5CX_DEVICE_STATUS` (0x010B)
   - `VL53L5CX_INTERRUPT_CLEAR` (0x0017)
   - `VL53L5CX_GPIO_CTRL` (0x001F)
   - `VL53L5CX_RANGE_START` (0x0030)
   - `VL53L5CX_RANGE_STATUS` (0x0031)
   - `VL53L5CX_RESULT_BASE` (0x0080)
   - 测距配置相关寄存器
5. **数据结构**：
   - `vl53l5cx_result_t` — 测距结果结构体（含各区的距离、状态、信号质量）
6. **全局变量声明**：
   - `vl53l5cx_init_flag`
   - `vl53l5cx_finsh_flag`
   - `vl53l5cx_distance_mm`（兼容现有 TOF.c 的首区距离）
   - `vl53l5cx_zone_result`（全16/64区结果）
7. **API 函数声明**（逐飞风格注释）：
   - `uint8 vl53l5cx_init(void)` — 初始化传感器
   - `void vl53l5cx_start_ranging(void)` — 启动测距
   - `void vl53l5cx_stop_ranging(void)` — 停止测距
   - `void vl53l5cx_get_distance(void)` — 获取测距数据
   - `void vl53l5cx_int_handler(void)` — 中断处理函数
   - `uint8 vl53l5cx_set_resolution(uint8 mode)` — 设置分辨率 (4x4/8x8)

---

### 新建文件 2: `project/code/zf_device_vl53l5cx.c`

**目的**：VL53L5CX 驱动实现文件，基于软 I2C 实现通信。

**内容**：
1. **逐飞风格文件头**
2. **头文件引用**：
   - `zf_common_debug.h`
   - `zf_driver_delay.h`
   - `zf_driver_exti.h`
   - `zf_driver_soft_iic.h`
   - `zf_device_config.h`
   - `zf_device_type.h`
   - `zf_device_vl53l5cx.h`
3. **全局变量定义**：
   - `vl53l5cx_init_flag`, `vl53l5cx_finsh_flag`
   - `vl53l5cx_distance_mm`
   - `vl53l5cx_zone_result` 结构体数组
   - `vl53l5cx_iic_struct` (soft_iic_info_struct)
4. **I2C 传输宏**：
   - `vl53l5cx_write_16bit_register(reg, data)` — 写16位寄存器
   - `vl53l5cx_read_16bit_register(reg, data)` — 读16位寄存器
   - `vl53l5cx_transfer_8bit_array(tdata, tlen, rdata, rlen)` — 通用传输
5. **内部辅助函数**（非导出）：
   - `vl53l5cx_reset_device()` — 硬件复位时序（PWR_EN → I2C_RST → LPn）
   - `vl53l5cx_probe()` — 探测传感器（读设备ID）
   - `vl53l5cx_software_reset()` — 软复位
   - `vl53l5cx_wait_for_boot()` — 等待传感器启动完成
6. **导出函数实现**：
   - **`vl53l5cx_init()`**：
     1. 初始化软 I2C
     2. GPIO 初始化（PWR_EN, I2C_RST, LPn, INT）
     3. 硬件复位时序
     4. 探测传感器（校验 ID）
     5. 加载初始化配置
     6. 注册到 `set_tof_type(TOF_VL53L5CX, ...)`
   - **`vl53l5cx_start_ranging()`**：
     - 配置测距模式 → 写入启动命令
   - **`vl53l5cx_stop_ranging()`**：
     - 写入停止命令 → 等待停止完成
   - **`vl53l5cx_get_distance()`**：
     - 检查中断/状态 → 读取结果寄存器 → 解析各区数据 → 更新全局变量
   - **`vl53l5cx_int_handler()`**：
     - 中断触发时调用 `vl53l5cx_get_distance()`
   - **`vl53l5cx_set_resolution()`**：
     - 设置 4x4 或 8x8 分辨率

7. **VL53L5CX 初始化配置数组**（配置固件参数）

---

### 变更文件 3: `project/code/TOF.c`

**目的（可选项）**：更新 TOF.c 使其支持 VL53L5CX 和 DL1B 两种传感器的切换使用。

**内容**（最小化改动）：
- 添加条件编译或运行时检测来选择使用 `dl1b_*` 还是 `vl53l5cx_*` 函数
- 后续可根据需要修改

---

### 变更文件 4: `libraries/zf_device/zf_device_type.h`

**目的**：在 `tof_type_enum` 中添加 `TOF_VL53L5CX` 枚举值。

**内容**：
```c
typedef enum
{
    NO_TOF     = 0,
    TOF_DL1A,
    TOF_DL1B,
    TOF_VL53L5CX,   // 新增
}tof_type_enum;
```

---

## 假设与决策

| 假设/决策 | 说明 |
|-----------|------|
| 使用软件 I2C | DL1B 默认使用软 I2C，VL53L5CX 同样适用，频率可调 |
| 默认引脚分配 | SCL=P19_0, SDA=P19_1（与 DL1B 共用总线，同一条 I2C） |
| 控制引脚默认 P13_0~P13_3 | 供用户通过宏自行修改 |
| VL53L5CX 设备地址 | `0x52 >> 1 = 0x29`（7位地址），与 ST 标准一致 |
| 初始化以探测+固件配置完成 | 参考 ST 库的初始化流程，简化为核心步骤 |
| 默认使用 4x4 连续模式 | 帧率高，适合飞控高度测量场景 |
| 不添加 8x8 模式的完整支持 | 仅通过 `set_resolution` 提供基础切换功能 |
| 全量 64 区结果的缓冲区 | 保留最大空间，由用户按需读取 |

---

## 验证步骤

1. **编译检查** — 确保 `zf_device_vl53l5cx.c/h` 在 IAR 工程中编译无误
2. **头文件完整性** — 检查所有宏定义、结构体、函数声明是否齐全
3. **风格一致性** — 与 `zf_device_dl1b.c/h` 逐行对比注释风格、命名规范、代码布局
4. **TOF.c 集成验证** — 确认 TOF.c 中 `dl1b_distance_mm` → `vl53l5cx_distance_mm` 的数据流正确
