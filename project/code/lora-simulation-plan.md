# 遥控器模拟无人机通信 — 技术修改大纲

## 一、背景与目标

无人机视觉尚未调试完成，当前用 **LoRa 遥控器 (lora3a22)** 模拟无人机，向小车发送"像素误差"，走通「遥控器 → 小车位置环 → 移动」的完整闭环，验证小车侧逻辑。

无人机侧真实链路（原计划）：`无人机视觉像素误差 → 485 → 小车位置环`
模拟链路（本次）：`LoRa遥控器摇杆 → 像素误差模拟 → 小车位置环`

---

## 二、串口规划（复用 UART_1）

| 串口 | 引脚 | 说明 |
|------|------|------|
| UART_1 | P04_1/P04_0 | **LoRa 复用**（485 暂时不用） |

**关键**：LoRa 默认就用 UART_1（`lora3a22.h` 里 `LORA3A22_UART_INDEX = UART_1`），所以**驱动宏完全不用改**。

物理上把 LoRa 模块接到 UART_1 的 TX/RX 引脚，模拟无人机发数据；485 那边暂时断开/不初始化。

---

## 三、文件清单

| 文件 | 操作 | 说明 |
|------|------|------|
| `libraries/zf_device/zf_device_lora3a22.h` | **不改** | 默认就是 UART_1，正好复用 |
| `project/code/App_lora3a22.c` | **新建** | 业务逻辑：摇杆→像素误差→写 pos_error |
| `project/code/App_lora3a22.h` | **新建** | 接口声明 |
| `project/code/init.c` | 修改 | 调 lora3a22_init() + App_Lora_Init()，注释 HC06 |
| `project/code/HC06_Driver.c` | 修改（暂缓） | 若发送阻塞则注释 SendVelocity |
| `project/user/cm7_0_isr.c` | 修改 | 挂 UART_1 的 LoRa 回调 |
| `project/iar/project_config/*.ewp` | 修改 | 添加 App_lora3a22.c |

---

## 四、实现步骤

### 步骤 1：确认 LoRa 驱动可用 UART_1
- `lora3a22_init()` 已用 UART_1（无需改宏）
- 初始化语句：`uart_init(UART_1, 115200, UART1_TX_P04_1, UART1_RX_P04_0)` 已在驱动内

### 步骤 2：新建 App_lora3a22.h / .c
接口：
```c
void App_Lora_Init(void);        // 调 lora3a22_init
void App_Lora_Task(void);        // 主循环轮询
```
- `lora3a22_uart_callback` 在中断里自动填充 `lora3a22_uart_transfer` 结构体
- App 层读 `lora3a22_uart_transfer.joystick[0..3]`

### 步骤 3：遥控器信道映射与控制设计

#### 3.1 用到的信道（仅 3 个摇杆轴，共 joystick[0..2]）

| 信道 | 物理操作 | 模拟内容 |
|------|---------|---------|
| `joystick[0]` 左杆 X | 左右推 | 小车左右平移像素误差 → `pos_error_y` |
| `joystick[1]` 左杆 Y | 前后推 | 小车前进/后退像素误差 → `pos_error_x` |
| `joystick[2]` 右杆 X | 左右推 | 小车自转（目标角度增量） |

> 按键、开关信道本次不用（S4/S5/S6 保留，可作备用触发）

#### 3.2 左摇杆 → 平移（模拟无人机像素误差）

```c
// App_Lora_Task 内, 每帧执行:
pos_error_x = joystick[1] * K_FWD   * SIGN_FWD;   // 左杆前后 → 前进/后退误差
pos_error_y = joystick[0] * K_LAT   * SIGN_LAT;   // 左杆左右 → 左右平移误差
```

- `joystick[1]` 前推为正向（默认），`joystick[0]` 右推为正向（默认）
- `K_FWD / K_LAT` 缩放系数（把摇杆 int16 缩到合适像素/误差量级）
- `SIGN_FWD / SIGN_LAT` **符号标定**：位置环内部是 `err = -GetPositionErrorX/Y()`（视觉翻转负号），App 写入时**不额外改符号**，与无人机语义保持一致；方向若反，实测后翻转 SIGN 常量即可
- 写入后 `PositionControl_Update()` 经 `GetPositionErrorX/Y()` 自动读取

#### 3.3 右摇杆 → 自转（增量式目标角度，推荐方案 B）

**推荐：不用模式切换，右摇杆做角度增量**（比方案 A 的 S4 切换更简洁）：

```c
// 右摇杆: 摇杆推着就累加目标角度, 回中自动锁住当前角度
angle_target += joystick[2] * K_YAW * dt_sec;   // dt_sec 为周期秒(0.01)

// 角度归一化到 ±180, 防止长期自转浮点溢出
if (angle_target >  180.0f) angle_target -= 360.0f;
if (angle_target < -180.0f) angle_target += 360.0f;
```

| 右摇杆操作 | 效果 |
|-----------|------|
| 往右打 | `angle_target` 增大 → 顺时针自转 |
| 往左打 | `angle_target` 减小 → 逆时针自转 |
| 回中 | `angle_target` 保持 → 锁住当前朝向 |

**为什么选增量式（方案B）而非绝对式/按键切换（方案A）：**
- 绝对式 `angle_target = joystick[2]*K`：摇杆回中会强制目标回 0°，车会自己转回去，不符合"自转后锁角度"的预期
- 按键切换需维护模式状态机，操作多一步；增量式天然"推着转、松手锁"
- 与左摇杆平移并行，无需模式切换

> ⚠️ `K_YAW` 与自转方向符号待实测标定（`yaw` 正方向对应顺/逆时针由 IMU 安装决定）

#### 3.4 发送侧（暂缓）
- 小车→无人机 `#vx,vy$` 先注释
- 确认 `uart_write_byte` 是否阻塞；若阻塞主循环则注释

### 步骤 4：init.c 接入
```c
// 注释掉: HC06_Init(1000000);
lora3a22_init();
App_Lora_Init();
```

### 步骤 5：中断调整
- UART_1 isr 里挂 `lora3a22_uart_callback`（`set_wireless_type` 已自动挂）
- HC06 的 `HC06_UART_RX_Handler` 可保留或注释（485 暂不用）

### 步骤 6：工程文件添加
- 两个 .ewp 加 App_lora3a22.c

---

## 五、关键风险点

1. **复用 UART_1**：LoRa 和 485 同串口，物理上只能接一个，485 要断开
2. **编码**：lora3a22.h 是 GBK，不改它
3. **数据源**：App 写的 `pos_error_x/y` 必须被 `PositionControl_Update()` 读到
4. **摇杆死区**：不回中会有残留误差，需死区处理（|joystick| < 阈值则视为 0）
5. **符号标定**：`SIGN_FWD/SIGN_LAT/K_YAW` 方向依赖硬件安装，需实测翻转
6. **角度增量溢出**：`angle_target` 长期自转需 ±180° 归一化
7. **发送阻塞**：确认 `uart_write_byte` 阻塞情况，必要时注释发送

---

## 六、验证方法

1. 上电后串口看到 LoRa 数据（printf 打印 joystick 原始值）
2. 推左摇杆，DBG 行 `px/py` 跟随变化（确认左右/前后方向正确）
3. 推右摇杆，小车自转，回中锁住当前朝向（确认顺/逆时针方向正确）
4. 按 KEY4 发车 → 位置环启动 → 左摇杆控制平移，右摇杆控制自转
5. 摇杆回中 → 小车停
