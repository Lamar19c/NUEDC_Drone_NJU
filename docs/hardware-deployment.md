# 硬件部署方案

## 系统拓扑

```
┌─────────────────────────────────────────────────────────────┐
│                        机载系统                              │
│  ┌─────────┐    ┌──────────┐    ┌───────────────────┐      │
│  │ 摄像头   │───→│ 机载 Pi   │←──→│ 飞控 (ArduPilot)  │      │
│  │ (USB)   │    │ YOLO推理  │    │ MAVLink / 串口    │      │
│  └─────────┘    └─────┬─────┘    └─────────┬─────────┘      │
│                       │                     │                │
│                       │    5V BEC 供电       │                │
│                       └──────────┬──────────┘                │
│                                  │ 串口 (TX/RX)               │
│                          数传 (天空端)                        │
└──────────────────────────────┬──────────────────────────────┘
                               │
                    433/915MHz 无线链路
                               │
┌──────────────────────────────┴──────────────────────────────┐
│                        地面系统                              │
│                     数传 (地面端)                             │
│                         │ USB / UART                         │
│                  ┌──────┴───────┐                            │
│                  │   地面 Pi     │                            │
│                  │ GCS Bridge     │                           │
│                  │ Web 界面 (屏)  │                           │
│                  └──────┬────────┘                            │
│                         │ HDMI                                │
│                   7寸触摸显示屏                               │
│                                                              │
│                  电池组 (12V / 5V)                            │
└──────────────────────────────────────────────────────────────┘
```

## 硬件清单

### 机载部分

| 部件 | 型号建议 | 用途 | 重量参考 |
|---|---|---|---|
| 机载计算机 | Raspberry Pi 4B 2GB / Sunrise X3 | YOLO 推理 + MAVLink 收发 | ~46g |
| 摄像头 | USB 广角摄像头 1080p | 视觉输入 | ~30g |
| 飞控 | Pixhawk 2.4.8 / Cube Orange | 飞行控制 | ~40g |
| 数传模块 | 3DR Radio V2 (433/915MHz) | MAVLink 无线透传 | ~15g |
| GPS 模块 | M8N GPS + 罗盘 | 室外定位 | ~25g |
| UWB 标签 | DWM1000 模块 | 室内测距（可选） | ~5g |
| BEC 电源 | 5V 3A mini BEC | 给 Pi 供电 | ~10g |
| 电池 | 4S LiPo 3000-5000mAh | 全系统供电 | — |

**机载总重**：约 200-300g（不含电池、无人机自身）。

### 地面部分

| 部件 | 型号建议 | 用途 |
|---|---|---|
| 地面计算机 | Raspberry Pi 4B 2GB | 运行 GCS Bridge |
| 显示屏 | 7寸 HDMI 触摸屏 (1024×600) | GCS 界面显示 |
| 数传模块 | 3DR Radio V2 | MAVLink 收发 |
| 电源 | 12V 锂电池组 / 充电宝 | 给 Pi + 屏供电 |
| 手持支架 | 3D 打印外壳 | 整合 Pi + 屏 |

## 接线

### 机载 Pi → 飞控

```
Pi GPIO14 (TXD)  ────→  飞控 TELEM2 RX
Pi GPIO15 (RXD)  ────→  飞控 TELEM2 TX
Pi GND            ────→  飞控 GND
```

波特率：921600（与飞控 TELEM2 默认一致）。

### 机载 Pi → 数传（天空端）

```
Pi USB 口 ────→ 数传模块 (USB-TTL)
```

数传为透明桥接，飞控 MAVLink 数据直接到地面。

### 地面数传 → 地面 Pi

```
数传 USB ────→ 地面 Pi USB 口
```

### 地面 Pi → 屏幕

```
Pi HDMI ────→ 7寸屏 HDMI
Pi USB   ────→ 7寸屏 USB (触摸 + 供电，可选)
```

## 软件部署

### 1. 烧录系统

两个 Pi 均刷 Raspberry Pi OS Lite (64-bit, Bookworm)：

```bash
# 用 Raspberry Pi Imager 或 dd
# 烧录后创建 SSH 文件 + wpa_supplicant.conf 联网
```

### 2. 安装依赖

```bash
sudo apt update && sudo apt install -y python3-pip python3-opencv
pip3 install pymavlink websockets pyserial numpy
```

### 3. 部署代码

```bash
# 两个 Pi 都克隆同一份代码
git clone <repo> ~/gcs
cd ~/gcs/main
```

### 4. 机载 Pi 开机启动

```bash
# /etc/systemd/system/onboard.service
[Unit]
Description=Onboard Controller
After=network.target

[Service]
Type=simple
ExecStart=/usr/bin/python3 /home/pi/gcs/main/onboard_bridge.py
Restart=always
User=pi
WorkingDirectory=/home/pi/gcs/main

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable onboard.service
```

### 5. 地面 Pi 开机启动

```bash
# /etc/systemd/system/gcs.service
[Unit]
Description=GCS Bridge + Web UI
After=network.target

[Service]
Type=simple
ExecStart=/usr/bin/python3 /home/pi/gcs/main/gcs_bridge.py
Restart=always
User=pi
WorkingDirectory=/home/pi/gcs/main

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable gcs.service

# 开机自动启动 Chromium 全屏打开 GCS
mkdir -p ~/.config/autostart
cat > ~/.config/autostart/gcs.desktop << 'EOF'
[Desktop Entry]
Type=Application
Name=GCS
Exec=chromium-browser --kiosk --noerrdialogs --disable-infobars http://localhost:8765/gcs.html
EOF
```

## UWB 室内定位部署

### 锚点部署

```
房间俯视图:

 S1 (0,0)          S2 (L,0)
     ┌──────────────────┐
     │                  │
     │    飞行区域      │
     │                  │
     │     ▣ 无人机     │
     │                  │
     └──────────────────┘
 S3 (0,W)          S4 (L,W)

L = 房间长度, W = 房间宽度 (米)
```

- 锚点安装在房间四角，高度 2-3 米
- 测量相对 NED 坐标系的实际位置
- 写入 `UWB/uwb_config.json`：

```json
{
  "anchors": {
    "S1": [0.0, 0.0, 2.5],
    "S2": [5.0, 0.0, 2.5],
    "S3": [0.0, 4.0, 2.5],
    "S4": [5.0, 4.0, 2.5]
  }
}
```

### 标签

无人机下方安装 1 个 DWM1000 标签模块，通过串口连接机载 Pi 的第二个 UART（GPIO 0/1，`/dev/ttyAMA1`）。

## SITL 仿真

在没有硬件的情况下：

```bash
# PC 上启动两组 SITL 输出
sim_vehicle.py -v ArduCopter --console --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551

# 桥接连 14550
python gcs_bridge.py

# drone_mission.py 连 14551
python drone_mission.py
```

## 比赛日检查清单

- [ ] 机载 Pi：YOLO 模型存在且可运行
- [ ] 机载 Pi：`onboard.service` 开机自启确认
- [ ] 飞控：GPS 3D fix（室外）或 UWB 定位正常（室内）
- [ ] 数传：地面站遥测数据刷新正常
- [ ] 地面 Pi：`gcs.service` + 浏览器自启确认
- [ ] 触摸屏：可正常操作 GCS 界面
- [ ] 电池：机载 / 地面各充满电
- [ ] 备用：一套 microSD 镜像 + 电源线
