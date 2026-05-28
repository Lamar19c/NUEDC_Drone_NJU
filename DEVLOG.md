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