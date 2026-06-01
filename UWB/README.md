# UWB 室内 3D 定位模块

基于四锚点 UWB 测距的室内实时定位系统，通过三边定位 + 扩展卡尔曼滤波计算标签 3D 坐标，并将位置以标准 NMEA 语句通过串口注入飞控 GPS 端口，实现室内自主飞行。

## 系统架构

```
┌─────────────┐   $DIST,M1,S1,3.21   ┌──────────────┐
│ UWB 锚点 S1~S4 │ ─────────────────→ │  SerialReader │
│  (固定位置)     │   $DIST,M1,S2,4.10   │  (串口协议解析) │
└─────────────┘                      └───────┬───────┘
                                             │ [d1,d2,d3,d4]
                                      ┌──────▼───────┐
                                      │   UWBSolver  │
                                      │  球面交汇+LSTQ │ → (x, y, z)
                                      │  + EKF 平滑   │ → (vx,vy,vz)
                                      └──────┬───────┘
                                             │
                              ┌──────────────┼──────────────┐
                              ▼              ▼              ▼
                         终端打印       UDP JSON (GCS)   串口 NMEA (飞控)
                                                        $GPGGA + $GPRMC
```

## 硬件需求

| 组件 | 数量 | 说明 |
|------|------|------|
| UWB 基站 (Anchor) | 4 | 固定位置，供电 |
| UWB 标签 (Tag, M1) | 1 | 安装于无人机上 |
| 机载计算机 | 1 | 旭日 X3 / Raspberry Pi / 笔记本，连接标签串口 |
| 串口线 | 2 | 标签→计算机 (UWB 数据)，计算机→飞控 GPS 口 (NMEA) |

## 快速开始

### 1. 安装依赖

```bash
pip install pyserial numpy
```

> UWB 定位模块**不需要 pymavlink**，仅需 `pyserial` + `numpy`。

### 2. 首次标定锚点

```bash
cd UWB/
python uwb.py --calibrate
```

交互式菜单提供三种标定方式：

| 方式 | 适用场景 |
|------|----------|
| **步长标定** | 场地无可参考坐标点，测量标签移动距离 |
| **已知点位标定** | 场地有网格线或标记点，知道标签在各点的实际坐标 |
| **直接输入** | 已通过其他方式测得锚点位置 |

标定结果自动保存到 `uwb_config.json`（主）和 `~/.uwb_calib.json`（备）。

也可以命令行非交互式标定：

```bash
# 已知点位标定：3 个位置的标签坐标
python uwb.py --calibrate --cal-points "0,0,0;1.5,0,0;0,2,0"

# 直接设置锚点坐标
python uwb.py --set-anchors "0,0,1.5;2,0,1.5;0,2,1.5;2,2,1.5"
```

### 3. 日常使用

```bash
# 最简启动（自动发现 UWB 串口，加载标定结果）
python uwb.py

# 指定 UWB 串口
python uwb.py --port COM4 --baud 19200

# 启用 GPS 模拟输出（串口直连飞控 GPS 端口）
python uwb.py --gps-emu-serial /dev/ttyS6

# 全部开启
python uwb.py --gps-emu-serial /dev/ttyS6 --no-udp
```

## GPS 模拟（NMEA 输出）

### 工作原理

UWB 解算的局部坐标 `(x, y, z)` → `local_to_gps()` 转为 WGS84 经纬度 → `_deg_to_nmea()` 格式化为 NMEA 语句 → `serial.write()` 输出到飞控 GPS 端口。

飞控以 `SERIALx_PROTOCOL=5` (GPS 协议) 接收，与真实 GPS 模块**完全无区别**。

### NMEA 语句

每次定位更新发送两条标准 NMEA 语句（限速 5 Hz）：

```
$GPGGA,143052.00,3206.8904,N,11857.5440,E,1,08,1.0,1.2,M,0.0,M,,*5F
$GPRMC,143052.00,A,3206.8904,N,11857.5440,E,1.94,45.0,310526,,,A*6C
```

| 字段 | 说明 |
|------|------|
| `$GPGGA` | 定位数据 — 时间、经纬度、高度、fix 质量 |
| `$GPRMC` | 最小推荐数据 — 位置、地速 (knots)、航向 (真北角) |

### 飞控连接

```
机载计算机 TX ──→ 飞控 GPS 口 RX
机载计算机 GND ──→ 飞控 GND
```

飞控无需任何参数修改。M9N 模块默认 38400 baud，UWB NMEA 默认同样为 38400 baud。

### 室内/室外切换

```
室外: 插真实 GPS 模块 → GPS 端口
室内: 插 UWB 计算机串口 → GPS 端口
```

拔插即切换，同一端口，无需改参。

## 配置参考 (`uwb_config.json`)

所有字段均为可选，未填写时使用自动检测值或默认值。优先级：**CLI 参数 > uwb_config.json > ~/.uwb_calib.json > 自动检测**。

```json
{
  "_comment": "所有字段均为可选",
  "serial_port": "COM4",
  "baud_rate": 19200,
  "anchors": {
    "S1": [0.0, 0.0, 1.5],
    "S2": [2.0, 0.0, 1.5],
    "S3": [0.0, 2.0, 1.5],
    "S4": [2.0, 2.0, 1.5]
  },
  "output": {
    "terminal": true,
    "udp_broadcast": {
      "enabled": true,
      "host": "127.0.0.1",
      "port": 14550
    },
    "gps_emulation": {
      "enabled": false,
      "serial_port": "/dev/ttyS6",
      "serial_baud": 38400
    }
  },
  "ekf_origin_lat": 321148408,
  "ekf_origin_lon": 1189590664,
  "ekf_origin_alt": 1200
}
```

### 配置字段说明

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `serial_port` | str | 自动发现 | UWB 标签串口设备 |
| `baud_rate` | int | 19200 | UWB 标签串口波特率 |
| `anchors` | dict | 标定获取 | `{"S1":[x,y,z], ...}` 锚点坐标 (m) |
| `output.terminal` | bool | true | 终端打印定位结果 |
| `output.udp_broadcast.enabled` | bool | false | UDP JSON 广播（给 GCS 桥接） |
| `output.udp_broadcast.host` | str | 127.0.0.1 | UDP 目标地址 |
| `output.udp_broadcast.port` | int | 14550 | UDP 目标端口 |
| `output.gps_emulation.enabled` | bool | false | NMEA GPS 模拟输出 |
| `output.gps_emulation.serial_port` | str | /dev/ttyS6 | 飞控 GPS 串口设备 |
| `output.gps_emulation.serial_baud` | int | 38400 | 飞控 GPS 串口波特率 |
| `ekf_origin_lat` | int | 321148408 | EKF 原点纬度 (E7 格式) |
| `ekf_origin_lon` | int | 1189590664 | EKF 原点经度 (E7 格式) |
| `ekf_origin_alt` | int | 1200 | EKF 原点高度 (mm) |

## CLI 参数

| 参数 | 说明 | 示例 |
|------|------|------|
| `--config` | 配置文件路径 | `--config uwb_config.json` |
| `--port` | UWB 串口（覆盖自动检测） | `--port COM4` |
| `--baud` | UWB 波特率（覆盖自动检测） | `--baud 19200` |
| `--default-height` | 3 anchor 模式的固定高度 (m) | `--default-height 1.0` |
| `--calibrate` | 交互式锚点标定 | `--calibrate` |
| `--cal-points` | 非交互式已知点位标定 | `--cal-points "0,0,0;1.5,0,0;0,2,0"` |
| `--set-anchors` | 命令行直接设置锚点 | `--set-anchors "0,0,1.5;2,0,1.5;0,2,1.5;2,2,1.5"` |
| `--no-udp` | 关闭 UDP 广播 | `--no-udp` |
| `--no-gps-emu` | 关闭 GPS 模拟输出 | `--no-gps-emu` |
| `--gps-emu-serial` | 飞控 GPS 串口设备 | `--gps-emu-serial /dev/ttyS6` |
| `--gps-emu-baud` | 飞控 GPS 串口波特率 | `--gps-emu-baud 38400` |

## 锚点部署

```
      S3 (0, 2, z)          S4 (2, 2, z)
          ●                      ●
          │                      │
          │       飞行区域        │
          │                      │
          ●                      ●
      S1 (0, 0, z)          S2 (2, 0, z)
```

- 锚点固定在飞行区域四角，高度一致（称 z 为锚点距地面高度）
- 标签 (M1) 安装于无人机，天线朝下无遮挡
- 锚点坐标系原点通常取 S1 下方地面点，X 轴向东、Y 轴向北
- 锚点 S4 为可选 — 3 锚点退化为固定高度 2D 定位；4 锚点实现完整 3D 定位

## 定位原理

### 三边定位

已知 4 个锚点坐标 `(xᵢ, yᵢ, zᵢ)` 和标签到各锚点的距离 `dᵢ`，最小二乘求解交点：

```
‖p - pᵢ‖ = dᵢ    (i = 1..4)
```

线性化为 `A·p = b`，`lstsq(A, b)` 求解。

### EKF 平滑

6 状态扩展卡尔曼滤波 `[x, y, z, vx, vy, vz]`，过程模型为匀速运动，观测为三边定位结果。3 锚点模式退化为 2D EKF（z 固定为 `default_height`）。

## 嵌入调用

```python
from uwb import AnchorConfig, UWBSolver, SerialReader, OutputRouter, local_to_gps
from uwb import auto_discover_uwb, calibrate_anchors_interactive

# 加载配置
cfg = AnchorConfig.load("uwb_config.json")

# 读取 UWB 数据
reader = SerialReader(cfg.serial_port, cfg.baud_rate)
reader.open()
distances = reader.read_distances()  # → [d1, d2, d3, d4] or None

# 解算位置
solver = UWBSolver(cfg.anchor_positions())
result = solver.solve(distances)  # → (x, y, z, vx, vy, vz) or None

# 输出到各路
router = OutputRouter(cfg)
router.output_position(x, y, z, vx, vy, vz)
```

## 故障排查

| 问题 | 原因 | 解决 |
|------|------|------|
| 未检测到 UWB 设备 | 串口占用 / 接线松动 | 检查串口是否被其他程序占用，接线是否牢固 |
| 标定后定位偏差大 | 锚点坐标不准 | 重新标定，推荐使用已知点位标定法 |
| GPS 模拟飞控不识别 | 波特率不匹配 | 确认 `--gps-emu-baud` 与飞控 `SERIALx_BAUD` 一致 |
| EKF 原点错误 | 默认原点距实际太远 | 修改 `uwb_config.json` 中 `ekf_origin_lat/lon/alt` |
| NMEA 串口写入失败 | 串口设备路径错误 | 检查 `--gps-emu-serial` 对应的设备是否存在 |
