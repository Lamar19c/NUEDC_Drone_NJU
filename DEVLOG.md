# 开发日志

## 2026-05-27

### 项目初始化
- 创建开发日志文件
- 分析现有代码库：UWB 室内定位系统 + MAVLink 无人机任务脚本
- 创建 CLAUDE.md 项目文档

### GCS 地面站 — 需求分析与设计
- 确定整体布局：三栏仪表盘（A方案）
  - 左栏：连接配置 + 飞行控制
  - 中区：地图/画布 + 遥测状态栏
  - 右栏：航点表格 + 任务控制
- 航点输入：混合模式（地图点击 + 可编辑表格双向联动）
- 连接方式：串口（可选端口+波特率）/ WiFi UDP / SITL 仿真
- 室内/室外双模式：
  - 室内：NED Canvas 画布，位置来自 UWB 伪GPS（MAVLink读取）
  - 室外：Leaflet 瓦片地图，真实 GPS
- 航点参数：逐航点设置（高度/悬停时间/朝向）
- 执行方式：上传飞控 + 逐步执行 均支持
- 遥测：8项实时数据（高度/速度/航向/模式/电池/GPS/坐标/目标）
- 架构：HTML 前端 + Python WebSocket 桥接后端
- 设计文档写入 `docs/superpowers/specs/2026-05-27-gcs-design.md`

### GCS 地面站 — 实现

#### gcs_bridge.py — Python WebSocket ↔ MAVLink 桥接后端
- `connect_mavlink(params)` — 连接飞控，支持 serial/udp/sitl 三种方式
- `disconnect_mavlink()` — 断开并清理连接
- `_mavlink_reader()` — 后台线程：循环读取 MAVLink 消息 (HEARTBEAT, GLOBAL_POSITION_INT, LOCAL_POSITION_NED, SYS_STATUS, GPS_RAW_INT, MISSION_CURRENT)，更新 telemetry_data 字典
- `do_set_mode(mode_name)` — 切换飞行模式 (GUIDED/AUTO/LOITER/RTL/LAND 等)
- `do_arm()` / `do_disarm()` — 解锁/上锁 (MAV_CMD_COMPONENT_ARM_DISARM)
- `do_takeoff(alt_m)` — 起飞到指定高度 (MAV_CMD_NAV_TAKEOFF)
- `do_land()` — 切换到 LAND 模式降落
- `do_goto(north, east, down)` — NED 坐标系位置移动 (SET_POSITION_TARGET_LOCAL_NED)
- `do_mission_upload(waypoints)` — 将航点列表上传到飞控 (MISSION_COUNT + MISSION_ITEM)
- `do_mission_start()` — 开始执行任务 (MAV_CMD_MISSION_START)
- `do_mission_pause()` — 暂停任务 (MAV_CMD_DO_PAUSE_CONTINUE)
- `_ned_to_global(north, east, down)` — NED 坐标转 WGS84 经纬度（基于 home 位置）
- `ws_handler(websocket)` — WebSocket 客户端处理：接收 JSON 指令，分派到对应命令函数，返回响应
- `telemetry_broadcast()` — 异步循环，每 250ms 向所有已连接客户端广播遥测快照
- `run_server(ws_port)` — 启动 WebSocket 服务器 + 遥测广播
- 全局状态：mav 连接对象、telemetry_data 字典（thread-safe via Lock）、ws_clients 集合

#### gcs.html — 浏览器端地面站前端
- **布局**：三栏 Flexbox 布局（左 280px 连接/控制 + 中区自适应地图 + 右 280px 航点/任务）
- **左侧面板 — 连接配置**：
  - 三种连接方式 radio 切换：串口（端口+波特率下拉）、WiFi UDP（IP+端口输入）、SITL
  - 连接/断开按钮 + 状态指示灯
- **左侧面板 — 定位模式**：
  - 室内 (UWB) / 室外 (GPS) radio 切换
- **左侧面板 — 飞行控制**：
  - 飞行模式下拉 (GUIDED/AUTO/LOITER/RTL/LAND/STABILIZE) + 切换按钮
  - 解锁/上锁 按钮
  - 起飞（可设高度）/降落/RTL 按钮
- **中间面板 — 地图区**：
  - 室内模式：Canvas 绘制 NED 网格坐标系，原点(Home)居中，N↑/E→ 轴标注，1m 网格线，5m 刻度
  - UWB Anchor 位置标记 (S1-S4)
  - 航点连线 + 编号标记
  - 无人机实时位置（红色三角形，朝向旋转）
  - 点击画布添加航点，滚轮缩放 (10-300 px/m)
  - 室外模式：Leaflet 瓦片地图 (OSM)，点击地图添加航点，circleMarker + polyline 显示航点
- **中间面板 — 遥测状态栏**：
  - 8 项实时数据：高度、速度、航向、飞行模式+解锁指示、电池电压/百分比、GPS卫星+HDOP、当前位置(NED或GPS)、目标航点
- **右侧面板 — 航点列表**：
  - 可编辑表格：序号/X(m)/Y(m)/Z(m)/悬停(s)/朝向(°)/删除
  - 表格修改 → NED画布/Leaflet地图实时联动更新
  - 添加/清空按钮
- **右侧面板 — 任务控制**：
  - 上传飞控：MISSION_ITEM 批量写入
  - 开始任务：MISSION_START
  - 逐步执行：逐个 SET_POSITION_TARGET_LOCAL_NED，悬停时间后自动下一航点
  - 暂停按钮 + 状态消息显示
- **WebSocket 客户端**：
  - 页面加载时自动连接 `ws://hostname:8765`
  - 页面关闭前发送 disconnect
  - JSON 命令格式：`{cmd: "xxx", ...params}`
- **演示数据**：3 个预置航点 [(0,0,-3), (4,0,-3), (4,4,-3)]

### Bug 修复 — SITL 连接无反应

- **websockets 版本兼容**：`websockets.asyncio.server.serve` 仅 >= 13.0 支持，加入 try/except 兼容旧版 `websockets.serve`
- **wait_heartbeat 超时**：SITL 未运行时原代码无限阻塞 WS handler，改为 `timeout=10` + try/except
- **connect_mavlink 返回值改为 dict**：`{"ok": True, ...}` 或 `{"error": "..."}` 格式，ws_handler 据此返回 connected 或 error status
- **前端连接状态两段显示**："桥接未启动" → "桥接就绪 - 点击连接飞控" → "飞控在线"
- **新增 ping 命令**：前端可检测桥接存活及 MAVLink 连接状态

### Bug 修复 — 室外地图无法显示

- OSM 瓦片在国内被墙，默认改为**高德地图**
- 新增地图源选择器：高德地图 / 高德卫星 / 腾讯地图 / OSM（国外）
- 切换室外模式时自动 invalidateSize 并重新加载瓦片

### 改进 — 室外航点坐标系 + 点击偏移修复

- **航点数据源改为 GPS**：每行用 `dataset.lat/lon/alt` 存储真实经纬度，NED 从中派生
- **室外模式**：航点表格显示纬度/经度/高度，输入精度到小数点后 6 位
- **室内模式**：航点表格显示 X/Y/Z(m)，编辑后转换回 GPS 存储
- **模式切换**：室内/室外切换时自动重建表格表头和数据格式
- **偏移修复**：室外点击地图直接取 `e.latlng.lat/lng` 存储，不再用地图中心算 NED 偏移；渲染也用存储的 lat/lon 直接定位，双向一致。`REF_LAT/REF_LON` 从 MAVLink 遥测的 `home_lat/home_lon` 自动同步
- 新增地图源：必应卫星、谷歌卫星、谷歌混合

### 新功能 — 飞行轨迹显示

- 自动跟踪无人机实时位置，在地图上绘制紫色航迹线
- 室内模式：Canvas 紫色 polyline 叠加在 NED 网格上
- 室外模式：Leaflet 紫色 `L.polyline` 叠加在瓦片地图上
- 每 2 分钟自动清除过期轨迹点（`TRAIL_TTL = 120s`）
- 每 10 秒异步清理过期点，模式切换时自动刷新

## 2026-05-28

### Bug 修复 — file:// 协议下 WebSocket 连接失败

- `location.hostname` 在 `file://` 协议下返回空字符串，导致 WS URL 变成 `ws://:8765`
- 修复：`WS_HOST = location.hostname || "localhost"` 兜底

### Bug 修复 — 航点上传 INVALID_PARAM5_X

- `_ned_to_global()` 返回 E7 格式 int（如 321148408），但 `mission_item_send` 参数 x/y（param5/param6）期望 float 度数（32.1148）
- 飞控收到超范围经纬度值 → `INVALID_PARAM5_X` → 所有航点被拒绝 → `OPERATION_CANCELLED`
- 修复：
  - 改为 `_ned_to_global_float()` 返回 float 度数
  - 优先使用前端传来的 `lat`/`lon`/`alt` 字段（GPS 坐标系存储）
  - 上传前先 `mission_clear_all_send` + 300ms 等待，避免旧任务残留

### Bug 修复 — drone_mission.py 无法同时连接 SITL

- SITL UDP 单播特性：MAVLink 数据只发到最后注册的客户端地址
- 两个程序（gcs_bridge + drone_mission）抢同一端口 → 先连的收不到心跳
- 解决：`--out udp:127.0.0.1:14551` 增加 SITL 输出端口，各程序连不同端口

### 改进 — UWB 锚点动态加载

- 锚点位置从硬编码改为读取 `UWB/uwb_config.json`
- 桥接新增 `get_config` 命令，前端 WebSocket 连接后自动请求锚点数据
- `uwb_config.json` 中修改锚点坐标后，刷新页面即可同步到 NED 画布显示

### 重构 — UWB 模块整合

- 将 5 个 UWB 子模块合并为单一 `UWB/uwb.py` 文件：
  - `uwb_config.py` → `AnchorConfig` 类
  - `uwb_solver.py` → `UWBEKF` + `UWBSolver` 类
  - `uwb_serial.py` → `SerialReader` 类
  - `uwb_output.py` → `OutputRouter` 类 + `local_to_gps()` 函数
  - `uwb_localizer.py` → `main()` 入口
- 原 5 个文件保留不删，`uwb.py` 为单文件独立运行的整合版
- 运行方式不变：`python UWB/uwb.py [--config uwb_config.json]`
- 可外部导入：`from uwb import AnchorConfig, UWBSolver, ...`
- 默认配置文件路径改为相对于 `uwb.py` 所在目录查找 `uwb_config.json`
- 原 5 个文件已删除，UWB 目录精简为 `uwb.py` + `uwb_config.json`
- 创建 `README.md` 项目说明文档
- 创建 `docs/hardware-deployment.md` 完整硬件部署方案（双 Pi 架构、接线、UWB 锚点部署、systemd 自启、比赛检查清单）

### UWB 零配置设计

- 分析当前配置痛点：E7 格式 GPS 不可读、JSON 嵌套深、两处独立解析、reload() 脆弱
- 确定四项全自动策略：
  - **串口** → 枚举端口自动发现 UWB 设备
  - **锚点坐标** → `--calibrate` 三点标定法（M1 依次放原点、X+1m、Y+1m），勾股定理反算每个锚点的 (x,y,z)
  - **输出通道** → UDP ping GCS / MAVLink heartbeat 自动检测，按需启用
  - **EKF origin** → 从飞控 HOME_POSITION 自动读取
- 锚点高度不等时采用三点法：每锚点三个未知数，三次测距解算，无需卷尺
- 保留 `uwb_config.json` 作为可选覆盖，所有字段可省略，空 `{}` 即全自动
- 优先级：命令行 > uwb_config.json > 缓存 (~/.uwb_calib.json) > 自动检测
- 设计文档写入 `docs/superpowers/specs/2026-05-28-uwb-zero-config-design.md`

### UWB 零配置 — 实现

- **自动串口发现** (`auto_discover_uwb`)：枚举所有 COM 口，依次尝试 19200/115200/57600 baud，收到 `$DIST` 即确认设备
- **三点标定** (`--calibrate`)：引导用户将 M1 依次放 (0,0,0) → (Δx,0,0) → (0,Δy,0)，每点 5 次读数取均值，勾股定理反算每个锚点的 (x,y,z)
- **标定缓存**：结果存入 `~/.uwb_calib.json`，二次启动自动加载
- **AnchorConfig.load() 重构**：
  - 串口/波特率未填时自动调用 `auto_discover_uwb()`
  - 锚点未填时自动从 `~/.uwb_calib.json` 加载
  - 所有字段改为可选，移除了 `reload()` 中脆弱的字段名列表
  - 配置文件不存在也不报错（全部自动检测）
- **EKF Origin 自动读取** (`OutputRouter.try_read_ekf_origin`)：MAVLink 连接后等待 `HOME_POSITION` 消息获取 GPS 原点，无需手填 E7 数字
- **输出通道 CLI 覆盖**：`--no-udp`、`--no-mavlink`、`--mavlink-host host:port`
- **gcs_bridge.py**：`load_uwb_config()` 优先读 `~/.uwb_calib.json`，其次 `uwb_config.json`
- **uwb_config.json**：添加 `_comment` 说明所有字段可选，保留原值作为示例

### GCS 自动跟随无人机

- 中间地图区右上角新增「跟随 ON/OFF」切换按钮，默认开启
- 室内模式：自动将 NED 画布偏移对准无人机当前位置（`nedOffsetX = -droneNED.y`, `nedOffsetY = -droneNED.x`）
- 室外模式：每次位置更新调用 `leafletMap.panTo()` 平滑跟随
- 用户手动拖拽 Leaflet 地图时自动关闭跟随，点击按钮可重新开启

### Bug 修复 — 室内跟随模式下坐标轴与锚点不显示

- Home 标记渲染公式错误：`ctx.arc(cx + nedOffsetX * nedScale, ...)` 多加了偏移项
- 原 offset 恒为 0 时未暴露，启用 follow 后 offset 非零 → home 标记被推到错误 NED 坐标
- 修复：改为 `ctx.arc(cx, cy, ...)`，NED (0,0) 到 canvas 的正确映射即为 (cx, cy)
- 坐标轴、锚点、航点、轨迹等元素公式均正确，无需修改

### Bug 修复 — 航点上传 MISSION_ITEM → MISSION_ITEM_INT

- 飞控收到 `MISSION_ITEM` (#39) 后提示应发送 `MISSION_ITEM_INT` (#73)
- 修复：`mission_item_send()` → `mission_item_int_send()`，lat/lon 改为 `int(lat * 1e7)` E7 格式
- frame 同步改为 `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT` (10)

### Bug 修复 — Windows GBK 编码

- `open()` 在中文 Windows 默认用 GBK 解码，遇到 UTF-8 字符（中文注释等）报错
- 修复：`uwb.py` 和 `gcs_bridge.py` 中所有 `open()` 调用显式指定 `encoding="utf-8"`

## 2026-05-29

### UWB 标定逻辑重构 — 三种标定方式

- **痛点**：原有三点标定法要求标签精确放在 (0,0)、(1m,0)、(0,1m) 三个位置，比赛场地难以精确测量 1m 距离
- **重构为交互式菜单**：`python uwb.py --calibrate` 进入三选一标定：
  1. **步长标定** — 移动标签并输入实际测量距离（不再强制 1m）
  2. **已知点位标定** — 将标签放在场地已知坐标点（网格交点、标记点等），输入坐标
  3. **直接输入锚点坐标** — 跳过测量，直接手填（适用于已知场地锚点大致位置）
- **新增 CLI 参数**：
  - `--cal-points "x,y,z;x,y,z;..."` — 非交互式已知点位标定
  - `--set-anchors "x,y,z;x,y,z;..."` — 命令行直接设置锚点
- **核心求解器提取**：`_solve_anchors_from_measurements()` 通用最小二乘反算，已知标签位置 + 测距 → 锚点坐标，步长标定和点位标定共用
- **辅助函数提取**：`_read_distances_avg()` 公共测距均值、`_print_anchors()` 公共打印
- **前向引用修复**：添加 `from __future__ import annotations`，避免 `_read_distances_avg(reader: SerialReader)` 在类定义前的 NameError

### UWB 自动串口发现增强

- **波特率扩展**：从 3 个 (19200/115200/57600) 增至 7 个 (921600/115200/57600/19200/38400/9600/230400)，优先尝试高速率
- **详细输出**：显示检测到的所有串口、每次尝试的端口+波特率、成功/失败原因
- **超时延长**：读取窗口 1.0s → 1.5s
- **错误可见**：不再静默吞异常，打印具体错误信息便于诊断

### UWB 标定结果保存修复

- **修复前**：标定结果仅存入 `~/.uwb_calib.json`，但 `AnchorConfig._resolve_anchors()` 优先读 `uwb_config.json`，旧锚点值会覆盖标定结果
- **修复后**：新增 `save_anchors_to_config()`，标定结果直接写入 `uwb_config.json` 的 `anchors` 字段，同时保留缓存文件作为后备

## 2026-05-31

### UWB fake-GPS 输出：NMEA GPS 模拟替换 MAVLink GPS_INPUT

- **协议分析**：收到反馈指出 fake-GPS 数据的接口与真实 GPS 信号传入飞控的接口相同 — 走的是 GPS 串口 (`SERIALx_PROTOCOL=5`)，而非 MAVLink TELEM 口 (`SERIALx_PROTOCOL=1`)。GPS 口期望 NMEA ASCII 文本或 u-blox 二进制，无法解析 MAVLink 帧，存在协议层不匹配
- **解决方案 B（采用）**：用标准 NMEA 语句 (`$GPGGA` + `$GPRMC`) 替代 MAVLink `GPS_INPUT` 消息，通过 raw pyserial 串口直接发送
- **新增辅助函数**：
  - `_deg_to_nmea(lat_e7, lon_e7)` — E7 经纬度 → NMEA `ddmm.mmmm` 度分格式 + 方向指示 (N/S/E/W)
  - `_nmea_checksum(sentence)` — NMEA XOR 异或校验和
- **OutputRouter 重写**：
  - `_init_gps_emulation()` — 使用 `serial.Serial()` 直连，无需 pymavlink
  - `_send_nmea_sentences()` — 生成并发送 `$GPGGA` (定位) + `$GPRMC` (速度/航向)，限速 5Hz
  - 移除 `try_read_ekf_origin()` — NMEA 是单向输出（机载计算机→飞控），不能读取 HOME_POSITION；EKF 原点需手动在配置文件中设定
  - `self.mavlink_conn` → `self._gps_serial` (raw `serial.Serial`)
  - 速度转换：m/s → knots (×1.94384)；航向：`atan2(vx, vy)` → 真北角度
- **AnchorConfig 重命名**：`mavlink_enabled/serial_port/serial_baud` → `gps_emu_enabled/serial_port/serial_baud`
- **配置文件格式**：`output.mavlink` → `output.gps_emulation`，默认波特率从 921600 改为 57600（匹配典型 GPS 模块）
- **CLI 参数重命名**：`--mavlink-serial` → `--gps-emu-serial`，`--mavlink-baud` → `--gps-emu-baud`，`--no-mavlink` → `--no-gps-emu`
- **移除功能**：`auto_discover_mavlink_serial()` 函数和 `MAVLINK_BAUD_RATES` 常量 — GPS 端口是输入型，无法通过监听 HEARTBEAT 自动发现
- **依赖简化**：UWB 定位模块不再依赖 pymavlink（仅 pyserial + numpy 即可，GPS 模拟通过 raw serial 输出）

## 2026-06-01

### UWB 伪 GPS — 地面站成功接收 GPS 数据

- **验证通过**：UWB NMEA 伪 GPS 信号经串口送达飞控 GPS 端口，地面站 (Mission Planner) 已成功显示 GPS 定位数据
- **接线确认**：机载 TX → 飞控 RX，机载 GND → 飞控 GND（仅需 2 根杜邦线）
- **飞控参数确认**：
  - `SERIALx_PROTOCOL=5` (GPS 协议)
  - `SERIALx_BAUD` 与 `uwb.py --gps-emu-baud` 一致 (默认 38400)
  - `GPS_TYPE=1` (AUTO) 或 `5` (NMEA)

### 终端 GPS 坐标显示

- `OutputRouter.output_position()` 终端输出增加 GPS 坐标：每行尾部追加 `GPS: lat,lon,alt_m`
- 转换调用 `local_to_gps()` 与 NMEA 发送同源，方便调试时肉眼验证

### NMEA 输出诊断增强

- **串口初始化失败 → 醒目错误框**：之前仅一行小字 `NMEA GPS 串口初始化失败`，易被坐标刷屏淹没。现改为多行框体，明确指出端口、错误原因、排查建议
- **首条 NMEA 打印到终端**：第一条成功发送的 `$GPGGA` + `$GPRMC` 完整打印，可肉眼校验语句格式和校验和
- **定期心跳**：每 ~5s (每 25 条 @ 5Hz) 打印 `📡 NMEA 已发送 N 组 → COMx (lat,lon)`
- **写入失败告警**：串口写入连续失败 3 次后打印警告，不再静默吞异常
- **新增状态追踪字段**：`_nmea_sent_count`、`_nmea_fail_count`、`_nmea_fail_warned`、`_gps_init_failed`

### ⚠ 待解决：COMPASS not healthy

- **现象**：地面站报 `COMPASS not healthy`，飞控检测不到罗盘
- **根因**：UWB 伪 GPS 模式下拔掉了外置 GPS 模块（如 M9N），该模块内置的 I2C 罗盘（SCL/SDA 引脚）随之断开。飞控预期 GPS 口上有外置罗盘，检测不到即报 unhealthy
- **解决方案（三选一，推荐方案 A）**：
  - **方案 A**：校准飞控内置罗盘 + 禁用外置罗盘
    - `COMPASS_TYPEMASK` — 仅勾选实际存在的 compass（通常是内部 I2C），取消不存在的外部罗盘
    - `COMPASS_USE=1`、`COMPASS_USE2=0`、`COMPASS_USE3=0`
    - 重新做一次罗盘校准
  - **方案 B**：保留 GPS 模块的 5V/GND/SCL/SDA 接线（仅断开 TX/RX 改接机载计算机），让罗盘保持在线上
  - **方案 C**：临时跳过（仅地面测试用，不实飞）— `ARMING_CHECK` 去掉 compass 检查位

## 2026-06-02

### Bug 修复 — EKF 卡尔曼滤波算法审查与修复

对 `UWB/uwb.py` 中 `UWBEKF` + `UWBSolver` + `SerialReader` 进行了系统性审查，发现并修复 4 个问题：

#### 🔴 Bug 1（严重）：距离-锚点 ID 失配 — `SerialReader.read_distances()`

- **根因**：`read_distances()` 在某个锚点未响应时（如 S3 丢失），返回跳过缺失 ID 的压缩列表 `[d1, d2, d4]`（3 元素），但 `UWBSolver.solve()` 按 `anchors[:3]` 取 S1/S2/S3 的坐标，导致 S4 的距离错误配对到 S3 的位置
- **修复**：返回列表改为 `[self._distances.get(i) for i in range(1, 5)]`，始终 4 元素，缺失锚点填 `None`；`solve()` 中按 `valid_indices` 精确匹配距离与锚点坐标
- **验证**：数值测试证实旧 bug 在 S3 缺失时产生 0.75m 定位误差

#### 🟡 Bug 2（中等）：`predict()` 中 dt 固定为 0.1

- **根因**：`UWBEKF.predict(dt=0.1)` 始终使用默认值，不追踪实际时间间隔。若 UWB 数据率非 10Hz，速度预测会有系统性偏差
- **修复**：
  - `predict()` 移除默认值，`dt` 改为必传参数
  - `UWBSolver` 新增 `_last_t` 属性，`solve()` 中计算实际 `dt = now - _last_t`，钳制到 [0.02, 2.0] 防止暂停后异常跳变

#### 🟡 Issue 3（次要）：过程噪声 Q 矩阵过于简化

- **根因**：`Q = I * process_noise` 用统一标量对角阵，忽略了位置/速度噪声的不同物理量纲
- **修复**：采用连续白噪声加速度模型 (CWNA) 推导的标准形式：
  ```
  Q_block = [[dt³/3, dt²/2],
             [dt²/2, dt   ]] * q    (每轴)
  ```
  新增 `accel_noise` 参数（加速度噪声 PSD，单位 m²/s³，默认 0.5），Q 矩阵在每次 `predict()` 中根据实际 dt 动态计算
- **API 变更**：`UWBSolver(process_noise, measurement_noise, ...)` → `UWBSolver(measurement_noise, accel_noise, ...)`

#### 🟢 Issue 4（轻微）：EKF 无初始化门控

- **根因**：`_initialized` 标志只写不读，滤波器从零初态 `x=[0,0,0,0,0,0]` 启动，首帧 innovation 很大导致状态突跳
- **修复**：`update()` 和 `update_2d()` 中增加初始化分支 — 首帧直接设位置（跳过 KF 更新），后续帧才走标准 Kalman 更新

#### 协方差更新稳定性提升

- `P = (I-KH) @ P` 简化形式 → `P = (I-KH) @ P @ (I-KH)ᵀ + K @ R @ Kᵀ` Joseph 形式，保证 P 始终对称正定

## 2026-06-03

### UWB EKF 测量噪声方差调优

- **变更**：EKF 测量噪声方差 `measurement_noise` 从 `0.1` 降低为 `0.01`（m²）
- **影响**：
  - 测量标准差 σ 从 ~0.32 m 降至 ~0.10 m
  - 滤波器对原始 UWB 测距值的信任度提升 **10 倍**，更紧密跟踪测量值
  - 噪声抑制减弱、响应速度提升 — 适用于高精度 UWB 锚点部署场景
- **修改位置**（`UWB/uwb.py`）：
  - `UWBEKF.__init__()` 默认参数：`measurement_noise: float = 0.01`
  - `UWBSolver.__init__()` 默认参数：`measurement_noise: float = 0.01`
  - EKF 初始协方差 `self.P`：`np.eye(6) * 0.01`（与原噪声水平保持一致）
  - 文档字符串：`σ_meas ≈ 0.32 m` → `σ_meas ≈ 0.1 m`

## 2026-06-06

### UWB 系统重新设计 — Anchor↔Anchor 自标定方案

- **需求分析**：当前 JZM01 UWB 方案需要人工标定锚点位置（三点法/已知点位法），无法自动部署
- **关键洞察**：如果 Anchor 之间可以互相测距（A2A TWR），可以通过 MDS（多维缩放）从距离矩阵自动求解锚点坐标，实现上电即自动标定
- **Download 目录发现**：下载的 `UWB_TDOA_STATION_V1_1`（TREK Station）代码已完整实现了：
  - Anchor↔Anchor TWR (`ANCTOANCTWR=1`)
  - TDMA superframe/slot 调度
  - UWB 帧内嵌 ToF 自动转发至 Gateway (S1)
  - 4 锚点角色管理 (A0 Gateway / A1 / A2 / A3 Listener)
- **TREK Tag 分析**：`UWB_TDOA_TAG_V1_1` 是基于 FreeRTOS 的 Blink 发射器，协议与 TREK Station 不兼容（帧控制字 0xC5 vs 0x41，功能码不同），仅作参考

### 硬件方案确定

- **放弃 ESP32-C3 方案**：TREK 代码基于 STM32F1 HAL，选 ESP32 需要重写整个 HAL 层
- **放弃额外无线模块**（HC-42 BLE、NRF24L01）：经代码审查发现 TREK 协议通过 UWB Response/Final 帧自动转发锚间 ToF 数据，Gateway 收齐全部距离，无需外部通信链路
- **最终方案**：STM32F103 + DWM1000 模组，Station 和 Tag 使用同一 TREK 代码库

### PCB 设计

| | Station V1 (基站) | Tag V1 (机载) |
|---|---|---|
| MCU | STM32F103RCT6 (256KB) | STM32F103C8T6 (64KB) |
| UWB | DWM1000 模组 | DWM1000 模组 |
| 供电 | 18650 + TP4056 + USB-C | 5V BEC → AMS1117 |
| USB-UART | CP2102N | 排针 (接 Pi/旭日) |
| LED | RGB LED (状态指示) | 单色 LED |
| DIP 开关 | 2位 (S1~S4 身份选择) | 无 (固定 TAG) |
| 尺寸 | 55×55mm 4层 | 30×40mm 2层 |
| BOM 成本 | ~¥77 | ~¥53 |

- **设计文档**：`main/uwb_system/pcb/station_v1.md` 和 `tag_v1.md`

### 固件架构 — TREK 代码 90% 复用

- **Station 固件**：Download 目录 TREK Station 代码的 `instance.c`、`instance_common.c`、`deca_device.c`、`port.c` 完全不动
- **仅需修改 `main.c`**：删除 LCD/W5500，新增 3 个模块调用
- **新增模块**（`main/uwb_system/modules/`）：

| 模块 | 功能 |
|------|------|
| `dist_aggregator.c/h` | 从 TREK `TOF_REPORT_A2A` 回调读取距离 → 构建 4×4 矩阵 |
| `auto_calib.c/h` | Jacobi 特征分解 → MDS 求解锚点 3D 坐标 → UART 输出 `$CALIB` |
| `led_status.c/h` | RGB LED 部署状态: 白闪→蓝闪→蓝亮→绿亮 |

- **自动部署流程**：上电 → TDMA 锚间测距(~3s) → 收齐 6 条距离 → MDS 求解(~1s) → 绿亮就绪
- **UART 输出协议**：原有 `mc`/`mr`/`ma` (TREK) + 新增 `$CALIB`/`$READY` (自标定)

### 弃用文件清理

- 删除 `__pycache__/` (残留缓存)
- 删除 `JZM01/anchor_twr_auto_v2.c` (JZM01 方案已放弃)
- 删除 `docs/hardware/port_esp32.c` (ESP32 方案已放弃)
- 删除 `docs/hardware/anchor_mesh.c/h` (HC-42 BLE 方案已放弃)
- 删除 `docs/hardware/jzm01_compat.c` (JZM01 兼容层不再需要)

## 2026-07-06

### STM32F103C8T6 UWB 定位模块 — 纯 C 移植

- **动机**：当前 Python UWB 方案依赖机载计算机（Pi/旭日X3），成本高、重量大。将定位解算下沉到 ¥12 单片机，取消机载计算机依赖。
- **设计讨论**：对比 STM32duino / PlatformIO / CubeMX+HAL 三种框架，选 CubeMX+HAL 裸机；对比 C++ class / C++桥接 / 纯C，选纯 C（CubeMX 生态一致，无桥接开销）；对比 EKF / 中值滤波，选双中值滤波（M3 内核无 FPU，整数运算更合适）。
- **锚点策略**：硬编码坐标，无需现场标定，上电即用。

### `uwb_config.h` — 纯 C 配置头文件

- Arduino `uwb_arduino/uwb_config.h` → C 版，删 `#include <Arduino.h>`
- 添加 `POS_WINDOW_SIZE` (从 Arduino class 提取为全局 define)
- 添加 `RAD_TO_DEG` / `DEG_TO_RAD` (Arduino 宏替代)
- `GPS_ORIGIN_ALT` 注释修正：1200 cm = 12.0 m，非 120.0 m

### `uwb_solver.h` — C struct 改写 (424 行)

Arduino C++ class → C struct + 独立函数，算法逻辑不变：

| C++ (Arduino 版) | C (STM32 版) |
|---|---|
| `class UWB_Parser { ... };` | `struct UWB_Parser { ... };` + `uwb_parser_feed(&p, c)` |
| `class DistanceFilter { ... };` | `struct DistanceFilter { ... };` + `dist_filter_apply(&df, raw, n)` |
| `class PositionFilter { ... };` | `struct PositionFilter { ... };` + `pos_filter_apply(&pf, &x, &y, &z)` |
| `class UWBSolver { ... };` | `struct UWBSolver { ... };` + `uwb_solver_solve(&s, ...)` |
| 构造函数 | `*_init()` 函数 |
| `private:` 方法 | `static` 函数 |
| `new` / `delete` | 栈分配全局变量 |

包含 4 个模块：
1. `median_float()` — 冒泡排序中位数 (n ≤ 8)，DistanceFilter / PositionFilter 共用
2. `UWB_Parser` — `$DIST,M1,S<id>,<m>` 协议解析，逐字符 feed，8 锚点缓存
3. `DistanceFilter` — 8 帧滑动窗口中位数 + 跳变限幅 (0.8m)
4. `PositionFilter` — 解算后 x,y,z 8 帧中值平滑
5. `UWBSolver` — 加权最小二乘三边测量 (solve3x3 Cramer → solve_2d/solve_3d)，含速度有限差分

### `uwb_nmea.h` — NMEA 生成器 C 改写 (160 行)

- `class NMEA_Generator` → `struct NMEA_Generator` + 独立函数
- **HAL-free 设计**：时间戳 `now_sec` 由调用者传入（main.c 用 `HAL_GetTick()/1000`），头文件不依赖 HAL
- `local_to_gps()` — UWB 局部坐标 → GPS E7（flat-earth projection, cos(lat) 修正）
- `deg_to_nmea_lat/lon()` — 十进制度 → NMEA `ddmm.mmmm` 格式
- `nmea_gen_generate()` — 7 步管线生成 `$GPGGA` + `$GPRMC`
- `date_str[8]` → `date_str[16]` — 修复 `-Wformat-truncation` 警告

### `main.c` — HAL 主循环 (CubeMX USER CODE 填充)

**串口分配**：

| USART | 引脚 | 连接 | 波特率 | 用途 |
|-------|------|------|--------|------|
| USART1 | PA9 TX | USB-TTL → PC | 115200 | printf 调试 |
| USART2 | PA3 RX | JZM01 基座 TX | 19200 | `$DIST` 接收 (IT) |
| USART3 | PB10 TX | 飞控 GPS RX | 57600 | NMEA 输出 (DMA) |

**关键实现**：
- `_write()` 重定向 printf → USART1
- USART2 中断逐字符接收 → `HAL_UART_RxCpltCallback` → `uwb_parser_feed()`
- USART3 DMA 发送 NMEA，带 50ms 超时保护
- `rx2_len` 加 `volatile`（ISR 写入，主循环读取）
- 所有 printf 使用 `%d` 整数格式（无 `%f`，省 ~12KB flash）

### `stm32_uwb.ioc` — CubeMX 项目配置

- MCU: STM32F103C8Tx, 72MHz HSE+PLL
- USART1 115200 / USART2 19200 / USART3 57600
- USART2_RX → DMA1 Channel6 循环模式，USART2 NVIC 中断使能
- USART3_TX → DMA1 Channel2 普通模式
- 生成完整 HAL 项目骨架（CMSIS, HAL Driver, 启动文件, 链接脚本）

### 物理接线 (6 根)

```
JZM01 TX  → PA3    |  STM32F103C8T6
JZM01 GND → GND    |  Blue Pill (~¥12)
飞控 RX   ← PB10   |  5V ← 飞控 BEC
飞控 GND  → GND    |
USB-TTL   ← PA9    |  (调试用，非必须)
USB-TTL GND → GND  |
```

### 设计文档

- `docs/superpowers/specs/2026-07-06-stm32-uwb-design.md` — 完整设计规格
- `docs/superpowers/plans/2026-07-06-stm32-uwb-plan.md` — 实现计划 (6 tasks)
- `stm32_uwb/` — 4 源文件 (~860 行纯 C) + CubeMX 项目

### 代码组织

- 创建 `main/uwb_system/` 目录，整合所有 UWB 系统相关代码：
  - `station/` — Station 固件核心代码（从 Download 复制）
  - `tag/` — Tag 参考代码（从 Download 复制）
  - `modules/` — 新增功能模块
  - `pcb/` — PCB 设计文档

## 2026-07-08

### NMEA 格式对齐 u-blox NEO-M9N (Python + STM32)

- **问题**：NMEA 伪 GPS 与真实 u-blox 模块存在格式差异，导致飞控频繁报错。
- **根因**：
  1. 缺失 `$GPVTG` 语句 — ArduPilot NMEA 驱动必需
  2. `$GPRMC` 磁偏角字段为 `,,`（空）vs u-blox `0.0,E,A`（显式值 + mode indicator）
  3. HDOP=1.0 / 卫星数=8 硬编码，值过于"完美"
- **修复**（三端同步：Python `uwb.py`、`gps_fixed_emu.py`、STM32 `uwb_nmea.c`）：
  - 新增 `$GPVTG` 语句（course/speed/knots/kmh）
  - `$GPRMC` 格式：`{date},,,A` → `{date},0.0,E,A`
  - HDOP / 卫星数可配置：新增 `nmea_hdop`(1.5) / `nmea_satellites`(10) 参数
  - Python CLI 新增 `--nmea-hdop` / `--nmea-sats`
- **推荐飞控配置**：`GPS_TYPE=5` (NMEA) — 不用 AUTO=1 避免 u-blox 探测冲突

### STM32 算法修复 — 加权最小二乘权重平方 bug

- **位置**：`uwb_solver.c` 中 `solve_2d` / `solve_3d`
- **问题**：加权最小二乘的权重 `w = 1/(d+0.1)` 直接乘到系数上，正规方程权重被平方 (`W→W²`)，远距离锚点被过度压低。
- **修复**：`w = sqrtf(1.0f/(d+0.1f))` — 正规方程权重 = w
- **Python 同步修复**：`uwb.py` 中 `np.sqrt(1.0/(d+0.1))`

### STM32 距离管理重构 — 超时失效替代主动清空

- **问题**：每次解算后 `uwb_parser_clear_distances()` 清空所有距离，4 锚点系统中第 4 个锚点数据经常来不及到达就被丢弃，系统长期退化为 3 锚点 2D 模式。
- **修复**：
  - `UWB_Parser` 新增 `last_update_ms[8]` 逐锚点时间戳
  - 新增 `DIST_TIMEOUT_MS`(150ms) 超时自动失效
  - `main.c` 移除 `clear_distances` 调用，改用 `get_valid_distances(now_ms)`
  - UWB 锚点数据不再丢失，4 锚点始终参与 3D 解算

### STM32 解算增强

- **最近锚点参考**：`solve_3d` 选距离最小的锚点做差分参考基点（提升精度）
- **残差校验**：新增 `compute_mean_residual()`，超 `RESIDUAL_MAX_3D`(2.0m) / `RESIDUAL_MAX_2D`(1.5m) 丢弃
- **NaN/Inf 防护**：解算输出 `(x==x && x*0==0)` 可移植校验，异常值不污染滤波窗口
- **限幅统一**：`local_to_gps` 硬编码 ±20m → 统一用 `POS_CLAMP_MIN/MAX`(±10m)
- **移除 `mode_2d`** 冗余字段

### STM32 内核优化 — 移除 %f 浮点 printf

- **问题**：`uwb_nmea.h` 中 `snprintf` 使用 `%f`，STM32 newlib-nano 默认不包含浮点 printf 支持
- **修复**：新增 `fmt_1dp()` / `fmt_2dp()` 整数格式化函数，所有 NMEA 生成改用 `%d` + `%s`
- **配套修复**：`deg_to_nmea_lat/lon` 分钟进位边界处理（`mm_int>=60 → d++`）、`%04d`→`%04u` 消除 truncation warning

### STM32 头文件 → .h + .c 拆分

- **`uwb_solver.h`**→ struct + declarations only; **`uwb_solver.c`**← all implementations (new)
- **`uwb_nmea.h`**→ struct + declarations only; **`uwb_nmea.c`**← all implementations (new)
- Debug 构建系统同步更新 (`subdir.mk`, `objects.list`)

### STM32 非阻塞 DMA NMEA 发送

- **旧逻辑**：DMA 发送后 `while(HAL_UART_GetState != READY)` 死等 50ms × 3 句 = 阻塞主循环
- **新逻辑**：
  - `nmea_start_dma_send()` 启动 GGPA DMA 后立即返回
  - `HAL_UART_TxCpltCallback` ISR 链式触发 RMC → VTG → IDLE
  - DMA 发送中不覆盖缓冲区 (`nmea_tx_state == NMEA_IDLE` 守卫)
  - 主循环无阻塞，DMA 在后台完成

### STM32 性能与稳定性优化

- **非阻塞主循环**：`HAL_Delay(20)` → tick-based 时间片调度 (`last_loop_time`)
- **UART 异常恢复**：`HAL_UART_ErrorCallback` 自动清除/重启（防接收永久中断）
- **速度独立滤波**：`VelocityFilter` 一阶低通，速度由原始位置差分（跳过 8 窗口中值），滞后减半
- **预计算常量**：`g_m_per_deg_lon` 首次计算 `cos(lat)`，后续复用
- **运行时统计**：每 5s 打印 parse/solve/NMEA 成功率