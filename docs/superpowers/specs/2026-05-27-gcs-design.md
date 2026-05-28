# GCS 地面站设计文档

## 概述

基于 ArduPilot 的 HTML 地面控制站（GCS）。前端为浏览器中的单页面应用，后端为 Python WebSocket 桥接进程，负责 MAVLink 协议编解码与飞控通信。

## 架构

```
HTML 前端 (浏览器)  ←──WebSocket JSON──→  Python 桥接 (本地进程)
                                            ├── MAVLink → 飞控 (串口/UDP)
                                            └── UWB 位置监听 (UDP, 室内模式)
```

- **前端**：纯 HTML/CSS/JS，三栏布局，WebSocket 客户端
- **后端**：`gcs_bridge.py`，pymavlink 连接管理 + WebSocket server + MAVLink ↔ JSON 双向转换

## 三栏布局

| 左栏 (280px) | 中区 (flex) | 右栏 (260px) |
|---|---|---|
| 连接配置 | 室内: NED Canvas 画布 / 室外: Leaflet 瓦片地图 | 航点可编辑表格 |
| 室内/室外模式切换 | 无人机实时位置 + 航点预览线 | 每行: X/Y/Z/悬停时间/朝向 |
| 飞行模式 + Arm/Disarm | 底部遥测状态栏 (8项) | 上传飞控 / 逐步执行按钮 |
| 任务控制 (起飞/降落/RTL) | | 任务进度指示 |

## 连接方式

左栏顶部三种方式切换：

- **串口**：端口下拉 (scan) + 波特率下拉 (9600–921600)
- **WiFi UDP**：IP + Port 输入
- **SITL**：固定 `udp:127.0.0.1:14550`

室内/室外切换按钮控制中区地图呈现方式和坐标解释。

## 航点系统

- 右侧表格，逐航点：X(m) / Y(m) / Z(m) / 悬停时间(s) / 朝向(°) / 删除
- 地图点击 → 坐标追加到表格 (双向联动)
- 每条航点的悬停指到达该点后的等待

### 执行模式

1. **上传飞控**：MISSION_ITEM 批量写入 → MISSION_START。飞控自主导航，可暂停/继续
2. **逐步执行**：GCS 逐个发送 SET_POSITION_TARGET_LOCAL_NED，到达容差后等待悬停时间，再发下一航点

## 室内 vs 室外

| | 室内 | 室外 |
|---|---|---|
| 地图 | Canvas NED 网格 (原点=home) | Leaflet 瓦片地图 |
| 坐标格式 | NED (x, y, z) 米 | GPS 经纬度 |
| 位置来源 | MAVLink GLOBAL_POSITION_INT (UWB 伪GPS) | MAVLink GLOBAL_POSITION_INT (真实GPS) |
| 背景 | Anchor 锚点静态标记 | 无 |

## 遥测状态栏 (8项)

高度 | 速度 | 航向 | 飞行模式+解锁 | 电池 | GPS(室外) | 当前位置坐标 | 目标航点+剩余距离

## WebSocket 消息协议

### 前端 → 后端

| 消息 | payload |
|---|---|
| `connect` | `{type: "serial"/"udp"/"sitl", port, baud, host, ...}` |
| `disconnect` | `{}` |
| `set_mode` | `{mode: "GUIDED"}` |
| `arm` / `disarm` | `{}` |
| `takeoff` | `{alt: 3.0}` |
| `land` | `{}` |
| `goto` | `{north, east, down}` |
| `upload_mission` | `{waypoints: [{x,y,z,delay,yaw}, ...]}` |
| `mission_start` / `mission_pause` | `{}` |
| `set_param` | `{indoor_mode: true/false}` |

### 后端 → 前端

| 消息 | payload |
|---|---|
| `connected` / `disconnected` | `{system, component}` |
| `telemetry` | `{alt, vx, vy, vz, heading, mode, armed, battery, satellites, hdop, lat, lon, ned_x, ned_y, ned_z}` |
| `mission_progress` | `{seq, total}` |
| `status` | `{msg, level}` |

## 依赖

- 前端：Leaflet (CDN), 无框架
- 后端：`pymavlink`, `websockets` (Python)

## 文件清单

```
main/
├── gcs_bridge.py          # Python WebSocket 桥接后端
├── gcs.html               # 前端 (单文件)
├── UWB/                   # 现有 UWB 模块 (不动)
├── drone_mission.py       # 现有任务脚本 (不动)
└── docs/superpowers/specs/2026-05-27-gcs-design.md  # 本文件
```
