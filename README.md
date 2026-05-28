# 无人机地面站系统

ArduPilot 地面控制站（GCS），支持室内 UWB 定位与室外 GPS 定位双模式，HTML 浏览器端 + Python 桥接后端。

## 项目结构

```
main/
├── gcs_bridge.py          # MAVLink ↔ WebSocket 桥接后端
├── gcs.html               # 浏览器端地面站前端
├── drone_mission.py        # MAVLink 单次任务示例脚本
├── UWB/
│   ├── uwb.py              # UWB 室内定位（零配置，自动发现+一键标定）
│   └── uwb_config.json     # 可选覆盖配置（所有字段可省略）
├── DEVLOG.md               # 开发日志
└── CLAUDE.md               # Claude Code 项目指引
```

## 环境要求

- Python ≥ 3.8
- 操作系统：Windows 10+ / Ubuntu 20.04+ / Raspberry Pi OS (Bookworm)
- Linux 需配置串口权限（见下方）

## 快速开始

### 1. 安装依赖

```bash
pip install pymavlink websockets pyserial numpy
```

### 2. 启动桥接

```bash
python gcs_bridge.py
```

### 3. 打开地面站

浏览器打开 `gcs.html`，连接飞控后可使用全部功能。

### 4. 室内定位（可选）

```bash
# 首次部署：标定锚点（交互式菜单，三种方式可选）
python UWB/uwb.py --calibrate

# 直接设置锚点坐标（已知场地尺寸）
python UWB/uwb.py --set-anchors "0,0,1.5;2,0,1.5;0,2,1.5;2,2,1.5"

# 日常使用：自动发现串口 + 加载标定结果
python UWB/uwb.py

# 指定飞控 MAVLink 地址
python UWB/uwb.py --mavlink-host 192.168.1.100:14550
```

标定结果直接写入 `UWB/uwb_config.json`，同时缓存到 `~/.uwb_calib.json` 作为后备。

## 连接方式

| 方式 | 说明 | 参数 |
|---|---|---|
| 串口 | 数传直连飞控 | 端口 + 波特率 |
| WiFi UDP | 网络数传 | IP + 端口 |
| SITL | 仿真测试 | 自动 `udp:127.0.0.1:14550` |

## 室内 / 室外双模式

| | 室内 | 室外 |
|---|---|---|
| 定位来源 | UWB 伪 GPS (GPS_INPUT 注入) | 真实 GPS |
| 坐标系 | NED (Canvas 网格画布) | GPS (Leaflet 瓦片地图) |
| 航点显示 | X/Y/Z (米) | 纬度/经度/高度 |
| 锚点标记 | UWB S1-S4 锚点 | 无 |

## 航点功能

- **混合输入**：地图点击 + 表格编辑双向联动
- **逐航点参数**：高度、悬停时间、朝向独立设置
- **两种执行方式**：
  - 上传飞控：MISSION_ITEM 写入，飞控自主执行
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

`drone_mission.py` 可同时连 14551 端口，与 GCS 互不干扰。

## 部署到树莓派

参见[硬件部署方案](docs/hardware-deployment.md)。

## 技术栈

- **前端**：HTML/CSS/JS、Canvas API、Leaflet
- **后端**：Python、asyncio、websockets、pymavlink
- **定位**：UWB 三边测量 + EKF 滤波
- **地图源**：高德 / 必应 / 谷歌 / 腾讯 / OSM
