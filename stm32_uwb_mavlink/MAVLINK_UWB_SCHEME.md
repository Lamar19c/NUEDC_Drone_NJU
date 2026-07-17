# STM32 UWB → MAVLink → ArduPilot 室内定位方案

> 精度 厘米级 | 端到端量化损失 0 | 替代 GPS 口 NMEA 方案

## 目录

1. [STM32 端：UWB 解算](#1-stm32-端uwb-解算)
2. [STM32 → 飞控通信：MAVLink v1](#2-stm32--飞控通信mavlink-v1)
3. [飞控端：ArduPilot 参数配置](#3-飞控端ardupilot-参数配置)
4. [部署清单](#4-部署清单)
5. [排错指南](#5-排错指南)

---

## 1. STM32 端：UWB 解算

### 1.1 信号链

```
$DIST ──→ USART2 RX ISR ──→ UWB_Parser ──→ EKF ──→ MAVLink 封包 ──→ USART3 TX
```

### 1.2 硬件

| STM32 Blue Pill | 连接 |
|----------------|------|
| PA3 (USART2 RX) | ← JZM01 UWB 模块 TX (115200) |
| PA9 (USART1 TX) | → USB-TTL 串口调试 (115200) |
| PB10 (USART3 TX) | → **飞控 TELEM1 RX** (57600) |
| GND | 与飞控共地 |

### 1.3 EKF 参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 状态 | `[x, y, z, vx, vy, vz]` | 6 维常速模型 |
| `q_acc_xy` | **自适应** (<0.3m/s→0.05, <1m/s→0.3, >1m/s→1.5) | 慢速平滑, 快速跟手 |
| `r_range` | 0.04 | 单次测距方差 (σ≈0.2m) |
| `gate_sq` | 36 | 6σ innovation 门限 |
| `init_pos_var` | 1.0 | 冷启动 HDOP≈1.4 |
| 固定高度 | 1.6m (set_fixed_z) | Z 几何不可观测, 硬固定 |

### 1.4 UWB → NED 坐标映射

```
UWB  X (东)  →  NED  y (东)        UWB  Y (北)  →  NED  x (北)
UWB  Z (上)  →  NED  -z (下)       UWB  vx      →  NED  vy
UWB  vy      →  NED  vx            航向 = atan2f(vx, vy)
```

---

## 2. STM32 → 飞控通信：MAVLink v1

### 2.1 物理层

| 项目 | 值 |
|------|-----|
| 物理线 | STM32 PB10 (USART3 TX) → 飞控 TELEM1 RX |
| 波特率 | **57600** (可调, 需与 `SERIAL1_BAUD` 一致) |
| 电平 | 3.3V TTL |
| 奇偶校验 | 无 |
| 停止位 | 1 |

### 2.2 消息格式

每条 MAVLink v1 帧包含 3 条消息：

#### 2.2a VISION_POSITION_ESTIMATE (msg_id=102, 5Hz)

| 偏移 | 字段 | 类型 | 来源 | 说明 |
|------|------|------|------|------|
| 0 | usec | uint64 | `HAL_GetTick()*1000` | 时间戳 (μs) |
| 8 | x | float32 | `UWB gy` | NED 北向位置 (m) |
| 12 | y | float32 | `UWB gx` | NED 东向位置 (m) |
| 16 | z | float32 | `-gz` | NED 下向位置 (m); 固定 -1.6 |
| 20 | roll | float32 | `0` | 横滚未知 |
| 24 | pitch | float32 | `0` | 俯仰未知 |
| 28 | yaw | float32 | `atan2f(vx,vy)` | 偏航角 (rad, 0=北 90=东) |

帧总长: **40 字节** (6 头 + 32 载荷 + 2 CRC)

#### 2.2b HEARTBEAT (msg_id=0, 1Hz)

| 字段 | 值 | 说明 |
|------|-----|------|
| type | 18 | MAV_TYPE_ONBOARD_CONTROLLER |
| autopilot | 8 | MAV_AUTOPILOT_INVALID |
| base_mode | 1 | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
| system_status | 4 | MAV_STATE_ACTIVE |

帧总长: **17 字节**

#### 2.2c 串口实际字节流 (示例)

```
00:  FE 20 01 01 BF 66  ← 头: 魔数|len=32|seq=1|sysid=1|compid=191|msgid=102
06:  XX XX XX XX XX XX XX XX  ← usec (uint64)
0E:  YY YY YY YY             ← x (float32) = N (m)
12:  YY YY YY YY             ← y (float32) = E (m)  
16:  YY YY YY YY             ← z (float32) = D (m)
1A:  00 00 00 00             ← roll = 0.0
1E:  00 00 00 00             ← pitch = 0.0
22:  YY YY YY YY             ← yaw (float32, rad)
26:  XX XX                   ← CRC16
```

### 2.3 时序

```
每 200ms 循环:
  ├─ 5Hz:  VISION_POSITION_ESTIMATE (每 200ms 发一帧, 5 帧/秒)
  └─ 1Hz:  HEARTBEAT (每 1000ms 发一帧)
```

### 2.4 代码位置

```
stm32_uwb_mavlink/Core/
├── Inc/mavlink.h    ← MAVLink v1 协议头, 消息定义
└── Src/mavlink.c    ← CRC16, 封包, HEARTBEAT + VPE 生成, USART 发送
```

---

## 3. 飞控端：ArduPilot 参数配置

### 3.1 端口配置

| 参数 | 值 | 说明 |
|------|-----|------|
| `SERIAL1_PROTOCOL` | **2** | MAVLink 协议 |
| `SERIAL1_BAUD` | **57** | 57600 bps |
| `SERIAL1_OPTIONS` | 0 | 默认 |

### 3.2 EKF 源切换

**关键**：告诉飞控 EK3 使用 ExternalNav 作为水平位置和航向源。

| 参数 | 值 | 含义 |
|------|-----|------|
| `EK3_SRC1_POSXY` | **6** (ExternalNav) | 水平位置从 MAVLink VPE 来 |
| `EK3_SRC1_VELXY` | 0 (None) | 速度不从 VPE 来 (无协方差不可靠) |
| `EK3_SRC1_POSZ` | 1 (Baro) | 高度继续用气压计 |
| `EK3_SRC1_YAW` | **6** (ExternalNav) | 航向从 MAVLink VPE 来 |

### 3.3 视觉里程计参数

| 参数 | 建议值 | 说明 |
|------|--------|------|
| `VISO_TYPE` | **1** | 启用 MAVLink 视觉里程计 |
| `VISO_POS_X` | 0.3 | Y 轴位置噪声 σ (m), 调小→更信 VPE |
| `VISO_POS_Y` | 0.3 | X 轴位置噪声 σ (m) |
| `VISO_POS_Z` | 0.5 | Z 轴位置噪声 σ (m) (本方案 Z 固定, 设大) |
| `VISO_ORIENT` | 0.2 | 姿态噪声 σ (rad) |
| `VISO_DELAY_MS` | 20 | 视觉估计延迟 (ms) |
| `VISO_VEL_M_NSE` | 0.5 | 速度噪声 σ (m/s) |

### 3.4 可选：辅助参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `GPS_TYPE` | **0** | 禁用 GPS (室内没有) |
| `ARMING_CHECK` | 可选减 1 | 跳过 GPS 解锁检查 |
| `EK2_ENABLE` | 0 | 仅用 EK3 |

### 3.5 验证

MAVLink Inspector (`mavlink inspector` 命令或 Mission Planner MAVLink Inspector):

```
VISION_POSITION_ESTIMATE:
  x: ~2.5      (锚点质心)
  y: ~2.5      
  z: ~1.6      
  yaw: ~0.0    (静止时)

EKF3:
  XKF3.PX ~2.5 ← 飞控 EKF 已融合 VPE 位置
  XKF3.PY ~2.5
```

串口调试输出 (STM32 USART1 @ 115200):

```
[0] x=2.51 y=2.47 z=1.58  vx=0.00 vy=0.00 vz=0.00  fix=1 anc=4 hdop=0.85
  MAVLink VPE: x=2.47 y=2.51 z=-1.58 hdg=0.0
```

---

## 4. 部署清单

### 4.1 硬件

- [ ] STM32 PB10 → 飞控 TELEM1 RX 连线
- [ ] 飞控 TELEM1 口空闲可用 (不与数传冲突)
- [ ] STM32 和飞控共地 (GND 线)

### 4.2 单片机

- [ ] `stm32_uwb_mavlink` 在 CubeIDE 中打开 `.ioc`
- [ ] Clean Build, 确认 0 Error 0 Warning
- [ ] 烧录 firmware
- [ ] 串口助手确认 `MAVLink VPE` 启动信息输出

### 4.3 飞控

- [ ] `SERIAL1_PROTOCOL = 2`
- [ ] `SERIAL1_BAUD = 57`
- [ ] `EK3_SRC1_POSXY = 6`
- [ ] `EK3_SRC1_YAW = 6`
- [ ] `GPS_TYPE = 0`
- [ ] 重启飞控
- [ ] Mission Planner MAVLink Inspector 确认 VPE 消息到达

---

## 5. 排错指南

| 症状 | 可能原因 | 检查 |
|------|---------|------|
| 无 VPE 消息 | UART 没连接/参数不对 | `SERIAL1_BAUD=57`, 飞控 TELEM1 RX 线连通 |
| VPE 收到但 EKF 不融合 | 缺 HEARTBEAT | STM32 1Hz H 输出正常吗 |
| EKF 融合后位置抖动 | `VISO_POS_X/Y` 设太低 | 先设 0.5 → 逐步降低 |
| EKF 位置偏离太多 | UWB→NED 映射反了 | 检查 VPE x=gy (北) y=gx (东) |
| 串口输出卡住 | DMA/中断冲突 | 当前用阻塞 TX, 检查 HAL_UART_STATE_BUSY_TX |
| Debug 不输出 | printf 超时 | `_write` 用 100ms 超时, 检查 huart1 初始化 |
