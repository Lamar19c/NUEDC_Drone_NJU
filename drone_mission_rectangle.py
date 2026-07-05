"""
ArduPilot 无人机 MAVLink 控制教学示例
=========================================
通过串口 / USB 给 ArduCopter 飞控发送 MAVLink 指令，让无人机完成：
解锁 → 起飞 → 悬停 → 前飞 → 悬停 → 飞一个矩形 → 降落

作者:  李希才  (南京大学 电子科学与工程学院)
依赖:  pip install pymavlink
适用:  ArduPilot Copter 4.x + Python 3.8+

⚠️ 安全提示
- 第一次跑务必先在 SITL 仿真里跑通；
- 室外飞行需要良好的 GPS 信号；
- 室内飞行需要光流 / VICON / Optitrack 等定位系统；
- 桨叶请在确认逻辑无误后再装。
"""

import time
import math
from pymavlink import mavutil


# =========================================================
# 1. 连接飞控
# =========================================================
def connect_vehicle(connection_string: str, baud: int = None):
    """
    建立与飞控的 MAVLink 连接，并等待第一个心跳包。

    常见连接字符串：
      USB 直连 (Linux):       '/dev/ttyACM0'           波特率通常 115200
      数传电台 (Linux):       '/dev/ttyUSB0'           波特率通常 57600
      USB 直连 (Windows):     'COM3'                   波特率通常 115200
      WiFi 数传 (UDP):        'udp:192.168.4.1:14550'  端口号必须带在 URL 中
      SITL 仿真:              'udp:127.0.0.1:14550'
    """
    is_network = connection_string.startswith(('udp:', 'tcp:', 'udpin:', 'tcpin:',
                                                'udpout:', 'tcpout:', 'udpbcast:'))

    if is_network:
        print(f"[连接] 正在连接 {connection_string} ...")
        master = mavutil.mavlink_connection(connection_string)
    else:
        if baud is None:
            baud = 115200
        print(f"[连接] 正在连接 {connection_string} @ {baud} baud ...")
        master = mavutil.mavlink_connection(connection_string, baud=baud)

    print("[连接] 等待心跳...", end=" ", flush=True)
    try:
        master.wait_heartbeat(timeout=10)
    except Exception:
        print("\n[连接] ✗ 超时 (10s) — 未收到飞控心跳")
        if is_network:
            print(f"  请检查: 是否已连接飞控 WiFi? IP {connection_string} 正确?")
            print(f"          飞控 SERIALx_PROTOCOL 是否设为 1 (MAVLink)?")
        else:
            print(f"  请检查: 串口 {connection_string} 是否存在? 波特率 {baud} 是否匹配?")
            print(f"          飞控是否已上电?")
        raise ConnectionError(f"无法连接到 {connection_string}")

    print("✓")
    print(f"[连接] 收到心跳: system={master.target_system}, "
          f"component={master.target_component}")
    return master


# =========================================================
# 2. 切换飞行模式
# =========================================================
def set_mode(master, mode_name: str):
    """
    切换 ArduCopter 飞行模式。
    我们做自动任务一般用 GUIDED 模式 —— 这种模式下飞控接受
    地面站 / 脚本发来的目标位置和速度指令。
    """
    if mode_name not in master.mode_mapping():
        raise ValueError(f"未知模式: {mode_name}")

    mode_id = master.mode_mapping()[mode_name]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )

    # 等待飞控确认模式已切换
    print(f"[模式] 请求切换到 {mode_name} ...")
    while True:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if hb is None:
            print("[模式] 等待心跳超时，重试...")
            continue
        if hb.custom_mode == mode_id:
            print(f"[模式] 已切换到 {mode_name}")
            break


# =========================================================
# 3. 解锁 / 上锁
# =========================================================
def arm(master):
    """通过 MAV_CMD_COMPONENT_ARM_DISARM 解锁电机。"""
    print("[解锁] 发送解锁指令...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,        # confirmation
        1,        # param1: 1 = arm, 0 = disarm
        0, 0, 0, 0, 0, 0
    )
    master.motors_armed_wait()
    print("[解锁] 电机已解锁 ✔")


def disarm(master):
    """上锁。"""
    print("[上锁] 发送上锁指令...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    master.motors_disarmed_wait()
    print("[上锁] 电机已上锁 ✔")


# =========================================================
# 4. 起飞 / 降落
# =========================================================
def takeoff(master, target_alt_m: float):
    """
    GUIDED 模式下起飞到指定相对高度 (单位: 米)。
    """
    print(f"[起飞] 目标高度 {target_alt_m} m")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,      # 前 4 个参数 Copter 不使用
        0, 0,            # 经纬度填 0 代表使用当前位置
        target_alt_m
    )

    # 等待无人机爬升到目标高度的 95%
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_alt = msg.relative_alt / 1000.0   # mm → m
        print(f"  当前高度: {current_alt:.2f} m")
        if current_alt >= target_alt_m * 0.95:
            print("[起飞] 到达目标高度 ✔")
            break
        time.sleep(0.5)


def land(master):
    """切换到 LAND 模式自动降落。"""
    print("[降落] 切换到 LAND 模式")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0, 0, 0, 0
    )

    # 等待真正落地 (高度 < 0.2 m 并且电机自动上锁)
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        alt = msg.relative_alt / 1000.0
        print(f"  下降中, 当前高度: {alt:.2f} m")
        if alt < 0.2:
            print("[降落] 已着陆 ✔")
            break
        time.sleep(0.5)


# =========================================================
# 5. 位置移动 (本地 NED 坐标系)
# =========================================================
# NED = North-East-Down  也就是 X 朝北, Y 朝东, Z 朝下
# 这是相对于 EKF 原点 (一般等于起飞点) 的局部坐标系。
# 飞 3m 高: Down = -3  (Z 轴朝下, 所以负值代表上方)
# =========================================================
def goto_local_ned(master, north: float, east: float, down: float):
    """
    给飞控发送一个 NED 坐标系下的目标位置。
    飞控会自动规划路径并以默认速度飞过去。
    """
    # type_mask: 第 0~2 位 = 位置使能, 其它位禁用
    # 0b0000_1111_1111_1000 → 只看位置，速度/加速度/yaw 忽略
    type_mask = 0b0000111111111000

    master.mav.set_position_target_local_ned_send(
        0,                                              # time_boot_ms
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        north, east, down,                              # 位置
        0, 0, 0,                                        # 速度 (忽略)
        0, 0, 0,                                        # 加速度 (忽略)
        0, 0                                            # yaw, yaw_rate (忽略)
    )


def wait_until_reached(master, target_north, target_east, target_down,
                       tolerance=0.5, timeout=30):
    """
    一直等到无人机抵达目标点附近 (默认 0.5 m 容差) 或者超时。
    用 LOCAL_POSITION_NED 消息读取无人机当前的 NED 位置。
    """
    start = time.time()
    while time.time() - start < timeout:
        msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
        if msg is None:
            continue
        dx = msg.x - target_north
        dy = msg.y - target_east
        dz = msg.z - target_down
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        print(f"  位置 N={msg.x:5.2f} E={msg.y:5.2f} D={msg.z:5.2f}  "
              f"距目标 {dist:.2f} m")
        if dist < tolerance:
            print("  → 到达目标点 ✔")
            return True
        time.sleep(0.5)
    print("  ⚠ 超时, 但任务继续")
    return False


def hover(seconds: float):
    """原地悬停一段时间 (GUIDED 模式下不发新指令就会自动保持位置)。"""
    print(f"[悬停] {seconds} 秒")
    time.sleep(seconds)


# =========================================================
# 6. 主任务流程
# =========================================================
def run_mission(connection_string: str, baud: int = None):
    # ----- 飞行参数 (单位: 米) -----
    ALTITUDE       = 3.0       # 起飞高度
    FORWARD_DIST   = 5.0       # 第一段往前飞的距离
    RECT_SIZE      = 5.0       # 矩形边长
    HOVER_TIME     = 3.0       # 每段悬停时间

    # ===== 连接并准备 =====
    master = connect_vehicle(connection_string, baud)
    set_mode(master, 'GUIDED')
    arm(master)

    # ===== 起飞 → 悬停 =====
    takeoff(master, ALTITUDE)
    hover(HOVER_TIME)

    # 用 -ALTITUDE 是因为 NED 中 Down 朝下，飞机在上方坐标是负的
    z = -ALTITUDE

    # ===== 往前 (北) 飞 5 m → 悬停 =====
    print("\n=== 阶段 1: 向北飞 5 m ===")
    goto_local_ned(master, FORWARD_DIST, 0, z)
    wait_until_reached(master, FORWARD_DIST, 0, z)
    hover(HOVER_TIME)

    # ===== 飞一个矩形 (从当前点出发) =====
    # 当前位置: (FORWARD_DIST, 0)
    # 矩形 4 个顶点 (依次飞过去):
    #   1) 当前点 (FORWARD_DIST, 0)            ← 已经在这里
    #   2) (FORWARD_DIST,           RECT_SIZE)  向东
    #   3) (FORWARD_DIST - RECT_SIZE, RECT_SIZE) 向南
    #   4) (FORWARD_DIST - RECT_SIZE, 0)        向西
    #   5) (FORWARD_DIST, 0)                    向北 → 回到起点
    print("\n=== 阶段 2: 飞矩形 ===")
    rectangle = [
        (FORWARD_DIST,             RECT_SIZE, z, "向东"),
        (FORWARD_DIST - RECT_SIZE, RECT_SIZE, z, "向南"),
        (FORWARD_DIST - RECT_SIZE, 0,         z, "向西"),
        (FORWARD_DIST,             0,         z, "向北 (回到矩形起点)"),
    ]
    for i, (n, e, d, label) in enumerate(rectangle, 1):
        print(f"\n--- 矩形第 {i}/4 边: {label} → ({n}, {e}, {d}) ---")
        goto_local_ned(master, n, e, d)
        wait_until_reached(master, n, e, d)
        hover(HOVER_TIME)

    # ===== 返航 + 降落 =====
    print("\n=== 阶段 3: 回起飞点上方并降落 ===")
    goto_local_ned(master, 0, 0, z)
    wait_until_reached(master, 0, 0, z)
    hover(HOVER_TIME)

    land(master)
    # 一般 ArduCopter 落地后会自动上锁，这里再发一次确保安全
    try:
        disarm(master)
    except Exception:
        pass

    print("\n🎉 任务完成！")


# =========================================================
# 入口
# =========================================================
if __name__ == '__main__':
    # ===== 选择连接方式 (取消注释你要用的那一行) =====
    # WiFi 数传 (UDP) — 端口号必须带在 URL 中
    CONN = 'udp:192.168.4.2:14550'
    # 数传电台 (串口)
    # CONN = '/dev/ttyUSB0'; BAUD = 57600
    # USB 直连飞控
    # CONN = '/dev/ttyACM0'; BAUD = 115200
    # SITL 仿真
    # CONN = 'udp:127.0.0.1:14550'
    # Windows
    # CONN = 'COM9'; BAUD = 115200

    run_mission(CONN)
