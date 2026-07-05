# 无人机地面站系统

ArduPilot 地面控制站（GCS），支持室内 UWB 伪 GPS 定位与室外 GPS 定位双模式，HTML 浏览器端 + Python 桥接后端。

## 项目结构

```
main/
├── gcs_bridge.py              # MAVLink ↔ WebSocket 桥接后端
├── gcs.html                   # 浏览器端地面站前端
├── drone_mission_rectangle.py # MAVLink 矩形航线任务示例脚本
├── UWB/
│   ├── uwb.py                 # Python UWB 室内定位（零配置，PC/Pi 运行）
│   ├── uwb_config.json        # 可选覆盖配置（所有字段可省略）
│   ├── uwb_arduino/           # Arduino C++ 移植版（Nano 33 BLE / ESP32）
│   └── README.md              # UWB 模块详细文档
├── stm32_uwb/                 # STM32F103C8T6 纯 C 移植（CubeMX + HAL）
│   ├── Core/Src/main.c        # HAL 主循环
│   ├── Core/Inc/uwb_config.h  # 配置参数
│   ├── Core/Inc/uwb_solver.h  # 解析 + 双中值滤波 + 三边求解
│   ├── Core/Inc/uwb_nmea.h    # NMEA 生成器
│   └── stm32_uwb.ioc          # CubeMX 项目
├── docs/
│   ├── hardware-deployment.md # 树莓派部署方案
│   └── superpowers/           # 设计规格 + 实现计划
├── DEVLOG.md                  # 开发日志
├── README.md                  # 本文件
└── CLAUDE.md                  # Claude Code 项目指引
```

## 环境要求

- Python ≥ 3.8
- 操作系统：Windows 10+ / Ubuntu 20.04+ / Raspberry Pi OS (Bookworm)
- Linux 需配置串口权限（`sudo usermod -aG dialout $USER`）

## 快速开始

### 1. 安装依赖

```bash
pip install pymavlink websockets pyserial numpy
```

> **注意**：`pymavlink` 仅 GCS 桥接和任务脚本需要。UWB 定位模块只需 `pyserial` + `numpy`，不依赖 pymavlink。

### 2. 启动桥接

```bash
python gcs_bridge.py
# 或指定端口
python gcs_bridge.py --ws-port 9000
```

### 3. 打开地面站

浏览器打开 `gcs.html`，选择连接方式（串口/WiFi UDP/SITL），点击连接后可使用全部功能。

---

## 室内 UWB 伪 GPS 定位

### 系统架构

```
UWB 锚点 S1~S4 ──[$DIST]──→ uwb.py ──NMEA $GPGGA/$GPRMC──→ 飞控 GPS 端口
                              │
                              ├── 终端打印 (坐标 + GPS)
                              └── UDP JSON ──→ gcs_bridge.py ──→ gcs.html
```

### 接线（仅需 2 根杜邦线）

```
机载计算机 TX  ──→  飞控 GPS 口 RX
机载计算机 GND ──→  飞控 GND
```

> 飞控 GPS 口的 5V、SCK、SDA 不需要连接。5V 是供电输出（两端都是），互接会烧板。

### 使用步骤

```bash
# 1. 首次部署：标定锚点（交互式菜单，三种方式可选）
python UWB/uwb.py --calibrate

# 2. 直接设置锚点坐标（已知场地尺寸）
python UWB/uwb.py --set-anchors "0,0,1.5;2,0,1.5;0,2,1.5;2,2,1.5"

# 3. 日常使用：GPS 模拟输出到飞控
python UWB/uwb.py --gps-emu-serial /dev/ttyS6 --gps-emu-baud 38400

# Windows 上指定 COM 口
python UWB/uwb.py --gps-emu-serial COM4 --gps-emu-baud 57600
```

标定结果直接写入 `UWB/uwb_config.json`，同时缓存到 `~/.uwb_calib.json` 作为后备。

### 飞控参数配置

| 参数 | 值 | 说明 |
|------|-----|------|
| `SERIALx_PROTOCOL` | 5 | GPS 协议（x 为 GPS 口对应的串口号） |
| `SERIALx_BAUD` | 38 (38400) 或 57 (57600) | 必须与 `--gps-emu-baud` 一致 |
| `GPS_TYPE` | 1 (AUTO) 或 5 (NMEA) | 飞控按 NMEA 协议解析 |
| `ekf_origin_lat/lon/alt` | 按实际场地填写 | 在 `uwb_config.json` 中配置 |

### ⚠️ 常见问题：COMPASS not healthy

拔掉外置 GPS 模块后，其内置 I2C 罗盘（SCL/SDA）随之断开，飞控报 `COMPASS not healthy`。

**推荐解决**：校准飞控内置罗盘，然后禁用外置罗盘：
- `COMPASS_TYPEMASK` — 仅勾选实际存在的 compass
- `COMPASS_USE2=0`、`COMPASS_USE3=0`
- 重新做一次罗盘校准（Mission Planner → 初始设置 → 强制校准 → 罗盘）

---

## 连接方式

| 方式 | 说明 | 参数 |
|---|---|---|
| 串口 | 数传直连飞控 | 端口 + 波特率 |
| WiFi UDP | 网络数传 | IP + 端口 |
| SITL | 仿真测试 | 自动 `udp:127.0.0.1:14550` |

## 室内 / 室外双模式

| | 室内 | 室外 |
|---|---|---|
| 定位来源 | UWB 伪 GPS (NMEA 串口注入) | 真实 GPS |
| 坐标系 | NED (Canvas 网格画布) | GPS (Leaflet 瓦片地图) |
| 航点显示 | X/Y/Z (米) | 纬度/经度/高度 |
| 锚点标记 | UWB S1-S4 锚点 | 无 |

## 航点功能

- **混合输入**：地图点击 + 表格编辑双向联动
- **逐航点参数**：高度、悬停时间、朝向独立设置
- **两种执行方式**：
  - 上传飞控：MISSION_ITEM_INT 写入，飞控自主执行
  - 逐步执行：GCS 逐个下发 SET_POSITION_TARGET_LOCAL_NED

## 遥测状态栏

高度 · 速度 · 航向 · 飞行模式 · 电池 · GPS 卫星 · 当前位置 · 目标航点

## SITL 仿真测试

```bash
# 终端 1：启动 SITL（多输出端口）
sim_vehicle.py -v ArduCopter --console --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551

# 终端 2：桥接
python gcs_bridge.py

# 浏览器打开 gcs.html → 选 SITL → 连接
```

`drone_mission_rectangle.py` 可同时连 14551 端口，与 GCS 互不干扰。

## 部署到树莓派

参见[硬件部署方案](docs/hardware-deployment.md)。

---

## STM32F103C8T6 嵌入式部署（取消机载计算机）

将 UWB 定位解算从 Python 下沉到 ¥12 单片机，直接输出 NMEA 伪 GPS 到飞控。

### 硬件（6 根线）

```
JZM01 基座 TX → STM32 PA3  (USART2 RX)    ← $DIST 测距数据
JZM01 基座 GND → STM32 GND
STM32 PB10     → 飞控 GPS RX              ← NMEA 伪 GPS 输出
飞控 GND        → STM32 GND
USB-TTL        ← STM32 PA9  (USART1 TX)   ← 调试 printf (可选)
飞控 BEC 5V    → STM32 5V                  ← 供电
```

### 配置（仅首次）

编辑 `stm32_uwb/Core/Inc/uwb_config.h`：

```c
// 锚点坐标 — 卷尺量场地四角
static const float ANCHOR_POSITIONS[4][3] = {
    {0.0f, 0.0f, 1.8f},   // S1
    {6.0f, 0.0f, 1.8f},   // S2
    {0.0f, 8.0f, 1.8f},   // S3
    {6.0f, 8.0f, 1.8f},   // S4
};

// GPS 参考原点 — Google 地图查场地中心
#define GPS_ORIGIN_LAT   321148408
#define GPS_ORIGIN_LON   1189590664
#define GPS_ORIGIN_ALT   1200    // 厘米
```

改完 → 编译 → ST-Link 烧录 → 上电即用（无需标定、无需串口助手、零交互）。

### 飞控参数

| 参数 | 值 |
|------|-----|
| `SERIALx_PROTOCOL` | 5 |
| `SERIALx_BAUD` | 57 (57600) |
| `GPS_TYPE` | 1 (AUTO) |

### 与 Python 版对比

| | Python (uwb.py) | STM32 (stm32_uwb/) |
|---|---|---|
| 运行平台 | Pi / 旭日X3 / PC | STM32F103C8T6 |
| 成本 | ~¥200+ | ~¥12 |
| 重量 | ~50g+ | ~5g |
| 滤波 | 6 状态 EKF | 双中值滤波 |
| 标定 | 交互式 3 模式菜单 | 硬编码坐标 |
| 输出 | 终端 + UDP + NMEA | printf 调试 + NMEA |
| 语言 | Python | 纯 C |
| 框架 | pyserial + numpy | STM32CubeIDE + HAL |

## 技术栈

- **前端**：HTML/CSS/JS、Canvas API、Leaflet
- **后端**：Python、asyncio、websockets、pymavlink
- **定位**：UWB 三边测量 + EKF 滤波，NMEA 伪 GPS 串口注入
- **地图源**：高德 / 必应 / 谷歌 / 腾讯 / OSM
