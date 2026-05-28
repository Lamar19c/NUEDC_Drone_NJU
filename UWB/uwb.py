#!/usr/bin/env python3
"""
UWB 室内 3D 定位模块 — 零配置版

首次使用:
    python uwb.py --calibrate          # 三点标定锚点坐标
    python uwb.py                      # 自动发现串口，加载标定结果

日常:
    python uwb.py [--port COMx] [--baud 19200] [--default-height 1.0]
    python uwb.py --no-udp --mavlink-host 192.168.1.100:14550

嵌入调用:
    from uwb import AnchorConfig, UWBSolver, SerialReader, OutputRouter, local_to_gps
"""

import argparse
import json
import math
import os
import socket
import sys
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np
import serial
import serial.tools.list_ports


# ============================================================================
# 1. Configuration
# ============================================================================


@dataclass
class AnchorConfig:
    serial_port: str
    baud_rate: int
    anchors: dict
    output_terminal: bool
    udp_enabled: bool
    udp_host: str
    udp_port: int
    mavlink_enabled: bool
    mavlink_host: str
    mavlink_port: int
    ekf_origin_lat: int
    ekf_origin_lon: int
    ekf_origin_alt: int

    @classmethod
    def _auto_discover_or_default(cls, data: dict) -> tuple:
        """Resolve serial port and baud rate: auto-discover if not in config."""
        port = data.get("serial_port")
        baud = data.get("baud_rate")

        if port and baud:
            return (port, baud)

        print("  自动搜索 UWB 设备...")
        result = auto_discover_uwb()
        if result:
            print(f"  发现 UWB 设备: {result[0]} @ {result[1]} baud")
            return result
        else:
            print("  未找到 UWB 设备，使用默认 COM3 @ 19200")
            return (data.get("serial_port", "COM3"),
                    data.get("baud_rate", 19200))

    @classmethod
    def _resolve_anchors(cls, data: dict) -> dict:
        """Load anchors from: config > calibration cache."""
        anchors = data.get("anchors")
        if anchors:
            return anchors

        cache = load_calibration_cache()
        if cache and cache.get("anchors"):
            print(f"  从 {CALIB_CACHE} 加载锚点坐标")
            return cache["anchors"]

        print("  警告: 未配置锚点坐标，请运行 --calibrate 标定")
        return {}

    @classmethod
    def _resolve_ekf_origin(cls, data: dict) -> tuple:
        """Resolve EKF origin from config. Returns (lat, lon, alt) or defaults."""
        lat = data.get("ekf_origin_lat", 321148408)
        lon = data.get("ekf_origin_lon", 1189590664)
        alt = data.get("ekf_origin_alt", 1200)
        return (lat, lon, alt)

    @classmethod
    def load(cls, path: str) -> "AnchorConfig":
        data = {}
        if os.path.exists(path):
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)

        port, baud = cls._auto_discover_or_default(data)
        anchors = cls._resolve_anchors(data)

        output = data.get("output", {})
        udp = output.get("udp_broadcast", {})
        mav = output.get("mavlink", {})

        lat, lon, alt = cls._resolve_ekf_origin(data)

        return cls(
            serial_port=port,
            baud_rate=baud,
            anchors=anchors,
            output_terminal=output.get("terminal", True),
            udp_enabled=udp.get("enabled", False),
            udp_host=udp.get("host", "127.0.0.1"),
            udp_port=udp.get("port", 14550),
            mavlink_enabled=mav.get("enabled", False),
            mavlink_host=mav.get("host", "192.168.1.100"),
            mavlink_port=mav.get("port", 14550),
            ekf_origin_lat=lat,
            ekf_origin_lon=lon,
            ekf_origin_alt=alt,
        )

    def anchor_positions(self) -> np.ndarray:
        positions = []
        for key in sorted(self.anchors.keys()):
            pos = self.anchors[key]
            positions.append([float(pos[0]), float(pos[1]), float(pos[2])])
        return np.array(positions)


# ---- calibration cache ----

CALIB_CACHE = os.path.join(os.path.expanduser("~"), ".uwb_calib.json")
BAUD_RATES = [19200, 115200, 57600]


def load_calibration_cache() -> Optional[dict]:
    if os.path.exists(CALIB_CACHE):
        with open(CALIB_CACHE, "r", encoding="utf-8") as f:
            return json.load(f)
    return None


def save_calibration_cache(anchors: dict) -> None:
    with open(CALIB_CACHE, "w", encoding="utf-8") as f:
        json.dump({"anchors": anchors}, f, indent=2)
    print(f"  锚点坐标已缓存到 {CALIB_CACHE}")


def auto_discover_uwb() -> Optional[tuple]:
    """枚举串口寻找 UWB 设备，返回 (port, baud_rate) 或 None"""
    ports = [p.device for p in serial.tools.list_ports.comports()]
    if not ports:
        return None

    for port in ports:
        for baud in BAUD_RATES:
            try:
                ser = serial.Serial(port, baud, timeout=0.5)
                ser.write(b"\r\n")
                time.sleep(0.3)
                buf = b""
                t0 = time.time()
                while time.time() - t0 < 1.0:
                    chunk = ser.read(256)
                    if chunk:
                        buf += chunk
                    if b"$DIST" in buf:
                        ser.close()
                        return (port, baud)
                ser.close()
            except Exception:
                pass
    return None


def calibrate_anchors(port: str, baud: int, step: float = 1.0) -> Optional[dict]:
    """三点标定法：引导用户将 M1 放在三个位置，反算锚点坐标"""
    reader = SerialReader(port, baud)
    try:
        reader.open()
    except Exception as e:
        print(f"  错误: 无法打开串口 {port}: {e}")
        return None

    def read_distances_avg(n_samples: int = 5) -> Optional[list]:
        """读取多组距离取均值"""
        all_dists = []
        for _ in range(n_samples):
            d = reader.read_distances(timeout=2.0)
            if d:
                all_dists.append(d)
            time.sleep(0.1)
        if not all_dists:
            return None
        arr = np.array(all_dists)
        return arr.mean(axis=0).tolist()

    positions = [(0, 0, "原点"), (step, 0, f"X+{step}m"), (0, step, f"Y+{step}m")]
    measurements = []

    for i, (px, py, label) in enumerate(positions):
        print(f"\n  [{i+1}/3] 将 M1 放在 ({px}, {py}, 0) = {label}，按 Enter 确认...", end="", flush=True)
        input()
        print("    读取中...", end=" ", flush=True)
        d = read_distances_avg()
        if d is None:
            print("✗ 未收到数据，标定失败")
            reader.close()
            return None
        measurements.append(d)
        print(f"{'  '.join(f'S{j+1}={v:.2f}m' for j, v in enumerate(d))} ✓")

    reader.close()

    n_anchors = len(measurements[0])
    d0 = np.array(measurements[0])
    dx = np.array(measurements[1])
    dy = np.array(measurements[2])

    anchors = {}
    for i in range(n_anchors):
        x = (d0[i]**2 - dx[i]**2 + step**2) / (2 * step)
        y = (d0[i]**2 - dy[i]**2 + step**2) / (2 * step)
        z_sq = d0[i]**2 - x**2 - y**2
        z = math.sqrt(max(z_sq, 0.0))
        anchors[f"S{i+1}"] = [round(x, 4), round(y, 4), round(z, 4)]

    print(f"\n  ✓ 标定完成，锚点坐标：")
    for k, v in anchors.items():
        print(f"    {k}: ({v[0]:.2f}, {v[1]:.2f}, {v[2]:.2f})")

    return anchors


# ============================================================================
# 2. EKF + Solver
# ============================================================================


class UWBEKF:
    """扩展卡尔曼滤波，状态 [x, y, z, vx, vy, vz]"""

    def __init__(self, process_noise: float = 0.01, measurement_noise: float = 0.1):
        self.x = np.zeros((6, 1))
        self.P = np.eye(6) * 0.1
        self.Q = np.eye(6) * process_noise
        self.R = np.eye(3) * measurement_noise
        self._initialized = False

    def predict(self, dt: float = 0.1) -> None:
        F = np.eye(6)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z_meas: np.ndarray) -> None:
        H = np.array([[1, 0, 0, 0, 0, 0],
                       [0, 1, 0, 0, 0, 0],
                       [0, 0, 1, 0, 0, 0]])
        y = z_meas.reshape(3, 1) - H @ self.x
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P
        self._initialized = True

    def update_2d(self, z_meas: np.ndarray, fixed_z: float) -> None:
        """2D 更新：仅更新 x, y，z 固定为 fixed_z，vz 清零"""
        H = np.array([[1, 0, 0, 0, 0, 0],
                       [0, 1, 0, 0, 0, 0]])
        y_vec = z_meas.reshape(2, 1) - H @ self.x
        R_2d = self.R[:2, :2]
        S = H @ self.P @ H.T + R_2d
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y_vec
        self.P = (np.eye(6) - K @ H) @ self.P
        self.x[2, 0] = fixed_z
        self.x[5, 0] = 0.0
        self._initialized = True

    @property
    def position(self) -> tuple:
        return (self.x[0, 0], self.x[1, 0], self.x[2, 0])

    @property
    def velocity(self) -> tuple:
        return (self.x[3, 0], self.x[4, 0], self.x[5, 0])


class UWBSolver:
    """四球面交汇求解器（3 anchor 时退化为固定高度的 2D 求解）"""

    def __init__(self, anchor_positions: np.ndarray,
                 process_noise: float = 0.01,
                 measurement_noise: float = 0.1,
                 default_height: float = 1.0):
        self.anchors = anchor_positions
        self.default_height = default_height
        self.ekf = UWBEKF(process_noise, measurement_noise)
        self._mode_2d = False

    def solve(self, distances: list) -> Optional[tuple]:
        if len(distances) < 3:
            return None

        n = len(distances)
        anchors = self.anchors[:n]
        d = np.array(distances, dtype=float)

        if n == 3:
            return self._solve_2d(anchors, d)
        else:
            return self._solve_3d(anchors, d)

    def _solve_3d(self, anchors: np.ndarray, d: np.ndarray) -> Optional[tuple]:
        self._mode_2d = False
        p0 = anchors[0]
        d0 = d[0]
        p0_norm_sq = np.dot(p0, p0)

        A = np.zeros((len(anchors) - 1, 3))
        b = np.zeros(len(anchors) - 1)

        for i in range(1, len(anchors)):
            pi = anchors[i]
            A[i - 1] = 2.0 * (pi - p0)
            b[i - 1] = (d0 * d0 - d[i] * d[i]
                        - p0_norm_sq + np.dot(pi, pi))

        result, _, _, _ = np.linalg.lstsq(A, b, rcond=None)

        z_meas = np.array(result)
        self.ekf.predict()
        self.ekf.update(z_meas)

        x, y, z = self.ekf.position
        vx, vy, vz = self.ekf.velocity
        return (x, y, z, vx, vy, vz)

    def _solve_2d(self, anchors: np.ndarray, d: np.ndarray) -> Optional[tuple]:
        self._mode_2d = True
        fixed_z = self.default_height
        p0 = anchors[0]
        d0 = d[0]
        p0_norm_sq = np.dot(p0, p0)

        A = np.zeros((2, 2))
        b = np.zeros(2)

        for i in range(1, 3):
            pi = anchors[i]
            rhs = (d0 * d0 - d[i] * d[i]
                   - p0_norm_sq + np.dot(pi, pi)
                   - 2.0 * (pi[2] - p0[2]) * fixed_z)
            A[i - 1] = 2.0 * (pi[:2] - p0[:2])
            b[i - 1] = rhs

        result, _, _, _ = np.linalg.lstsq(A, b, rcond=None)

        z_meas = np.array([result[0], result[1]])
        self.ekf.predict()
        self.ekf.update_2d(z_meas, fixed_z)

        x, y, z = self.ekf.position
        vx, vy, vz = self.ekf.velocity
        return (x, y, z, vx, vy, vz)


# ============================================================================
# 3. Serial Reader
# ============================================================================


class SerialReader:
    """UWB 串口数据读取器"""

    def __init__(self, port: str, baud_rate: int = 19200):
        self.port = port
        self.baud_rate = baud_rate
        self.ser: Optional[serial.Serial] = None
        self._distances: dict[int, float] = {}
        self._buffer = ""

    def open(self) -> None:
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baud_rate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1,
        )

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()

    @staticmethod
    def parse_line(line: str) -> Optional[tuple]:
        line = line.strip()
        if not line.startswith("$DIST,"):
            return None

        parts = line.split(",")
        if len(parts) != 4:
            return None

        master = parts[1]
        slave = parts[2]

        if master != "M1" or not slave.startswith("S"):
            return None

        try:
            slave_id = int(slave[1:])
            distance = float(parts[3])
        except ValueError:
            return None

        return (slave_id, distance)

    def _feed_data(self, chunk: str) -> list:
        self._buffer += chunk
        results = []
        while "\n" in self._buffer:
            line, self._buffer = self._buffer.split("\n", 1)
            parsed = self.parse_line(line)
            if parsed:
                self._distances[parsed[0]] = parsed[1]
                results.append(parsed)
        return results

    def read_distances(self, timeout: float = 2.0) -> Optional[list]:
        if not self.ser or not self.ser.is_open:
            return None

        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                chunk = self.ser.read(256).decode("utf-8", errors="ignore")
                if chunk:
                    self._feed_data(chunk)
                if len(self._distances) >= 3:
                    result = []
                    for i in range(1, 5):
                        if i in self._distances:
                            result.append(self._distances[i])
                    self._distances.clear()
                    return result
            except Exception:
                pass
        return None

    @property
    def is_open(self) -> bool:
        return self.ser is not None and self.ser.is_open


# ============================================================================
# 4. Output Router
# ============================================================================


def local_to_gps(x: float, y: float, z: float,
                 origin_lat: int, origin_lon: int, origin_alt: int) -> tuple:
    lat_rad = math.radians(origin_lat / 1e7)
    meters_per_deg_lat = 111320.0
    meters_per_deg_lon = 111320.0 * math.cos(lat_rad)

    lat = origin_lat + int(y * 1e7 / meters_per_deg_lat)
    lon = origin_lon + int(x * 1e7 / meters_per_deg_lon)
    alt = origin_alt + int(z * 1000)
    return (lat, lon, alt)


class OutputRouter:
    """三路输出: 终端 / UDP广播(给GCS) / WiFi MAVLink(给飞控)"""

    def __init__(self, config: AnchorConfig):
        self.config = config
        self.udp_socket: Optional[socket.socket] = None
        self.mavlink_conn = None

        if config.udp_enabled:
            self._init_udp()

        if config.mavlink_enabled:
            self._init_mavlink()

    def _init_udp(self) -> None:
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_target = (self.config.udp_host, self.config.udp_port)
        print(f"UDP 输出: {self.config.udp_host}:{self.config.udp_port}")

    def _init_mavlink(self) -> None:
        try:
            from pymavlink import mavutil
            self.mavlink_conn = mavutil.mavlink_connection(
                f"udp:{self.config.mavlink_host}:{self.config.mavlink_port}",
                source_system=1,
                source_component=1,
                input=False,
            )
            print(f"MAVLink 输出: {self.config.mavlink_host}:{self.config.mavlink_port}")
        except Exception as e:
            print(f"MAVLink 初始化失败: {e}")
            self.mavlink_conn = None

    def try_read_ekf_origin(self, timeout: float = 5.0) -> bool:
        """尝试从飞控读取 HOME_POSITION 作为 EKF 原点，成功返回 True"""
        if not self.config.mavlink_enabled or self.mavlink_conn is None:
            return False

        try:
            from pymavlink import mavutil
            conn = mavutil.mavlink_connection(
                f"udp:{self.config.mavlink_host}:{self.config.mavlink_port}",
                source_system=1,
                source_component=1,
                input=True,
            )
            print(f"  等待飞控 HOME_POSITION ({timeout}s)...", end=" ", flush=True)
            t0 = time.time()
            while time.time() - t0 < timeout:
                msg = conn.recv_match(type="HOME_POSITION", blocking=False)
                if msg:
                    lat = getattr(msg, "latitude", None)
                    lon = getattr(msg, "longitude", None)
                    alt = getattr(msg, "altitude", None)
                    if lat and lon:
                        self.config.ekf_origin_lat = int(lat)
                        self.config.ekf_origin_lon = int(lon)
                        self.config.ekf_origin_alt = int(alt * 1000) if alt else self.config.ekf_origin_alt
                        print(f"✓ lat={lat}, lon={lon}, alt={alt}")
                        conn.close()
                        return True
                time.sleep(0.1)
            print("超时，使用默认值")
            conn.close()
        except Exception as e:
            print(f"失败: {e}")
        return False

    def output_position(self, x: float, y: float, z: float,
                        vx: float, vy: float, vz: float,
                        timestamp: Optional[float] = None) -> None:
        if timestamp is None:
            timestamp = time.time()

        ts = time.strftime("%H:%M:%S", time.localtime(timestamp))

        if self.config.output_terminal:
            print(f"[{ts}] x={x:+.2f}, y={y:+.2f}, z={z:+.2f}  "
                  f"vx={vx:+.2f}, vy={vy:+.2f}, vz={vz:+.2f}")

        if self.udp_socket:
            payload = json.dumps({
                "x": round(x, 4),
                "y": round(y, 4),
                "z": round(z, 4),
                "vx": round(vx, 4),
                "vy": round(vy, 4),
                "vz": round(vz, 4),
                "ts": round(timestamp, 3),
            }).encode("utf-8")
            try:
                self.udp_socket.sendto(payload, self.udp_target)
            except OSError:
                pass

        if self.config.mavlink_enabled:
            self._send_mavlink_gps_input(x, y, z, vx, vy, vz, timestamp)

    def _send_mavlink_gps_input(self, x: float, y: float, z: float,
                                 vx: float, vy: float, vz: float,
                                 timestamp: float) -> None:
        if not self.mavlink_conn:
            return

        lat, lon, alt = local_to_gps(
            x, y, z,
            self.config.ekf_origin_lat,
            self.config.ekf_origin_lon,
            self.config.ekf_origin_alt,
        )

        alt_m = alt / 1000.0
        vn = vy
        ve = vx
        vd = vz

        self.mavlink_conn.mav.gps_input_send(
            0,              # time_usec
            0,              # gps_id
            0,              # ignore_flags
            0,              # time_week_ms
            0,              # time_week
            3,              # fix_type = 3 (3D fix)
            lat,            # lat (E7)
            lon,            # lon (E7)
            alt_m,          # alt (meters)
            1.0,            # hdop
            1.0,            # vdop
            vn,             # vn (m/s)
            ve,             # ve (m/s)
            vd,             # vd (m/s)
            1.0,            # speed_accuracy
            1.0,            # horiz_accuracy
            1.0,            # vert_accuracy
            0,              # satellites_visible
        )

    def close(self) -> None:
        if self.udp_socket:
            self.udp_socket.close()
            self.udp_socket = None
        if self.mavlink_conn:
            try:
                self.mavlink_conn.close()
            except Exception:
                pass


# ============================================================================
# 5. Main
# ============================================================================


def main():
    default_config = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "uwb_config.json")

    parser = argparse.ArgumentParser(description="UWB 室内 3D 定位模块")
    parser.add_argument("--config", default=default_config,
                        help="配置文件路径 (默认: uwb_config.json，所有字段可选)")
    parser.add_argument("--port", default=None,
                        help="串口端口 (覆盖自动检测)")
    parser.add_argument("--baud", type=int, default=None,
                        help="串口波特率 (覆盖自动检测)")
    parser.add_argument("--default-height", type=float, default=1.0,
                        help="3 anchor 模式下的默认高度值 (单位: 米，默认: 1.0)")
    parser.add_argument("--calibrate", action="store_true",
                        help="三点标定法标定锚点坐标")
    parser.add_argument("--step", type=float, default=1.0,
                        help="标定时 M1 移动步长 (默认: 1.0m)")
    parser.add_argument("--no-udp", action="store_true",
                        help="强制关闭 UDP 输出")
    parser.add_argument("--no-mavlink", action="store_true",
                        help="强制关闭 MAVLink 输出")
    parser.add_argument("--mavlink-host", default=None,
                        help="飞控 MAVLink 地址 (格式: host:port)")
    args = parser.parse_args()

    # ---- calibrate mode: guide user, save cache, exit ----
    if args.calibrate:
        port = args.port
        baud = args.baud or 19200
        if not port:
            print("  搜索 UWB 设备...")
            result = auto_discover_uwb()
            if result:
                port, baud = result
                print(f"  发现: {port} @ {baud} baud")
            else:
                print("  未找到 UWB 设备，请用 --port 指定")
                sys.exit(1)

        anchors = calibrate_anchors(port, baud, args.step)
        if anchors:
            save_calibration_cache(anchors)
        else:
            sys.exit(1)
        return

    # ---- normal mode ----
    cfg = AnchorConfig.load(args.config)

    # CLI overrides
    if args.port:
        cfg.serial_port = args.port
    if args.baud:
        cfg.baud_rate = args.baud
    if args.no_udp:
        cfg.udp_enabled = False
    if args.no_mavlink:
        cfg.mavlink_enabled = False
    if args.mavlink_host:
        parts = args.mavlink_host.rsplit(":", 1)
        cfg.mavlink_enabled = True
        cfg.mavlink_host = parts[0]
        if len(parts) == 2:
            cfg.mavlink_port = int(parts[1])

    positions = cfg.anchor_positions()
    n_anchors = positions.shape[0]
    if n_anchors < 2:
        print("错误: 至少需要 2 个 anchor 才能定位")
        print("  请运行: python uwb.py --calibrate 进行标定")
        sys.exit(1)
    if n_anchors < 3:
        print(f"警告: 仅 {n_anchors} 个 anchor，降级为 2D 定位")

    print(f"\n加载 {n_anchors} 个 anchor:")
    for i in range(n_anchors):
        print(f"  S{i+1}: ({positions[i,0]:.2f}, {positions[i,1]:.2f}, {positions[i,2]:.2f})")

    solver = UWBSolver(positions, default_height=args.default_height)
    router = OutputRouter(cfg)

    if cfg.mavlink_enabled:
        router.try_read_ekf_origin()

    print(f"\n串口: {cfg.serial_port} @ {cfg.baud_rate} baud")
    print(f"输出: terminal={'ON' if cfg.output_terminal else 'OFF'}"
          f"  udp={'ON' if cfg.udp_enabled else 'OFF'}"
          f"  mavlink={'ON' if cfg.mavlink_enabled else 'OFF'}")
    if n_anchors == 3:
        print(f"3 anchor 模式: 高度固定为 z={args.default_height:.2f}m，仅解算 x, y")
    print("等待 UWB 数据...\n")

    reader = SerialReader(cfg.serial_port, cfg.baud_rate)
    try:
        reader.open()
    except Exception as e:
        print(f"错误: 无法打开串口 {cfg.serial_port}: {e}")
        print("请检查:")
        print("  1. 串口是否被其他程序占用")
        print("  2. 端口名称是否正确 (Windows: COMx, Linux: /dev/ttyUSBx)")
        sys.exit(1)

    loop_count = 0
    try:
        while True:
            distances = reader.read_distances(timeout=2.0)
            if distances is None:
                continue

            result = solver.solve(distances)
            if result is None:
                continue

            x, y, z, vx, vy, vz = result
            ts = time.time()
            loop_count += 1

            router.output_position(x, y, z, vx, vy, vz, ts)

    except KeyboardInterrupt:
        print(f"\n\n已处理 {loop_count} 组数据")
    finally:
        reader.close()
        router.close()
        print("UWB 定位模块已停止")


if __name__ == "__main__":
    main()
