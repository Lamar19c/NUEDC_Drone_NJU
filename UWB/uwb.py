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

from __future__ import annotations

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
            default_port = "COM3" if sys.platform == "win32" else "/dev/ttyUSB0"
        print(f"  未找到 UWB 设备，使用默认 {default_port} @ 19200")
        return (data.get("serial_port", default_port),
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
BAUD_RATES = [921600, 115200, 57600, 19200, 38400, 9600, 230400]


def load_calibration_cache() -> Optional[dict]:
    if os.path.exists(CALIB_CACHE):
        with open(CALIB_CACHE, "r", encoding="utf-8") as f:
            return json.load(f)
    return None


def save_anchors_to_config(config_path: str, anchors: dict) -> None:
    """将锚点坐标写入 uwb_config.json 的 anchors 字段"""
    data = {}
    if os.path.exists(config_path):
        with open(config_path, "r", encoding="utf-8") as f:
            data = json.load(f)
    data["anchors"] = anchors
    with open(config_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    print(f"  锚点坐标已保存到 {config_path}")


def save_calibration_cache(anchors: dict) -> None:
    """后备：同时存一份到用户目录缓存"""
    with open(CALIB_CACHE, "w", encoding="utf-8") as f:
        json.dump({"anchors": anchors}, f, indent=2)
    print(f"  锚点坐标已缓存到 {CALIB_CACHE}")


def auto_discover_uwb(verbose: bool = True) -> Optional[tuple]:
    """枚举串口寻找 UWB 设备，返回 (port, baud_rate) 或 None"""
    ports = [p.device for p in serial.tools.list_ports.comports()]
    if not ports:
        if verbose:
            print("    未检测到任何串口设备")
        return None

    if verbose:
        print(f"    发现 {len(ports)} 个串口: {', '.join(ports)}")

    for port in ports:
        for baud in BAUD_RATES:
            if verbose:
                print(f"    尝试 {port} @ {baud} ...", end=" ", flush=True)
            try:
                ser = serial.Serial(port, baud, timeout=0.5)
                ser.write(b"\r\n")
                time.sleep(0.3)
                buf = b""
                t0 = time.time()
                while time.time() - t0 < 1.5:
                    chunk = ser.read(256)
                    if chunk:
                        buf += chunk
                    if b"$DIST" in buf:
                        ser.close()
                        if verbose:
                            print("✓ 找到 UWB 设备")
                        return (port, baud)
                ser.close()
                if verbose:
                    print("✗ 无 $DIST 响应")
            except Exception as e:
                if verbose:
                    print(f"✗ {e}")
    if verbose:
        print("    未找到 UWB 设备")
    return None


def _read_distances_avg(reader: SerialReader, n_samples: int = 5,
                        timeout: float = 2.0) -> Optional[list]:
    """读取多组距离取均值"""
    all_dists = []
    for _ in range(n_samples):
        d = reader.read_distances(timeout=timeout)
        if d:
            all_dists.append(d)
        time.sleep(0.1)
    if not all_dists:
        return None
    arr = np.array(all_dists)
    return arr.mean(axis=0).tolist()


def _print_anchors(anchors: dict) -> None:
    """打印锚点坐标"""
    for k, v in anchors.items():
        print(f"    {k}: ({v[0]:.2f}, {v[1]:.2f}, {v[2]:.2f})")


def _solve_anchors_from_measurements(measurements: list,
                                      positions: list) -> dict:
    """从已知标签位置和距离测量反算锚点坐标。

    measurements: [[d1, d2, ...], ...]  每个位置的测距列表
    positions:    [(x, y, z), ...]      对应的已知标签坐标
    返回: {"S1": [x,y,z], "S2": [x,y,z], ...}
    """
    n_anchors = len(measurements[0])
    n_points = len(positions)
    p0 = np.array(positions[0])
    p0_norm_sq = np.dot(p0, p0)
    d0_arr = np.array(measurements[0])

    anchors = {}
    for ai in range(n_anchors):
        d0 = d0_arr[ai]
        A = np.zeros((n_points - 1, 3))
        b = np.zeros(n_points - 1)
        for j in range(1, n_points):
            pj = np.array(positions[j])
            dj = measurements[j][ai]
            A[j - 1] = 2.0 * (pj - p0)
            b[j - 1] = d0**2 - dj**2 - p0_norm_sq + np.dot(pj, pj)
        result, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        anchors[f"S{ai + 1}"] = [round(float(result[0]), 4),
                                  round(float(result[1]), 4),
                                  round(float(result[2]), 4)]
    return anchors


def calibrate_anchors(port: str, baud: int, step: float = 1.0) -> Optional[dict]:
    """三点标定法：引导用户将 M1 放在三个位置，反算锚点坐标

    step 仅作为默认值，交互式下会提示用户输入实际测量距离。
    """
    reader = SerialReader(port, baud)
    try:
        reader.open()
    except Exception as e:
        print(f"  错误: 无法打开串口 {port}: {e}")
        return None

    # 提示输入步长
    step_str = input(f"\n  请输入标签移动步长(米) [默认={step}]: ").strip()
    if step_str:
        try:
            step = float(step_str)
        except ValueError:
            print(f"  输入无效，使用默认步长 {step}m")
    print(f"  使用步长: {step}m")

    positions = [(0, 0, 0, "原点"),
                 (step, 0, 0, f"X+{step}m"),
                 (0, step, 0, f"Y+{step}m")]
    measurements = []

    for i, (px, py, pz, label) in enumerate(positions):
        print(f"\n  [{i+1}/3] 将 M1 放在 ({px}, {py}, {pz}) = {label}，按 Enter 确认...", end="", flush=True)
        input()
        print("    读取中...", end=" ", flush=True)
        d = _read_distances_avg(reader)
        if d is None:
            print("✗ 未收到数据，标定失败")
            reader.close()
            return None
        measurements.append(d)
        print(f"{'  '.join(f'S{j+1}={v:.2f}m' for j, v in enumerate(d))} ✓")

    reader.close()

    # 仅保留坐标部分传给求解器
    pos_xyz = [(px, py, pz) for px, py, pz, _ in positions]
    anchors = _solve_anchors_from_measurements(measurements, pos_xyz)

    print(f"\n  ✓ 标定完成，锚点坐标：")
    _print_anchors(anchors)
    return anchors


def calibrate_anchors_with_points(port: str, baud: int,
                                    points: Optional[list] = None) -> Optional[dict]:
    """已知点位标定法：将标签放在已知坐标点，反算锚点坐标。

    points: [(x, y, z, label), ...] — 如果为 None 则交互式输入
    至少需要 3 个不共线点位。
    """
    reader = SerialReader(port, baud)
    try:
        reader.open()
    except Exception as e:
        print(f"  错误: 无法打开串口 {port}: {e}")
        return None

    if points is None:
        # 交互式输入点位
        points = []
        print("\n  已知点位标定 — 将标签依次放在已知坐标点")
        print("  输入格式: x,y,z （如: 0,0,0  或 1.5,0,0.2）")
        print("  至少需要 3 个点，空行结束输入\n")
        while True:
            label = input(f"  点位{len(points) + 1} 坐标 (x,y,z): ").strip()
            if not label:
                if len(points) >= 3:
                    break
                print(f"    至少需要 3 个点，当前仅 {len(points)} 个")
                continue
            try:
                parts = [float(v.strip()) for v in label.split(",")]
                if len(parts) != 3:
                    print("    格式错误，请输入 x,y,z")
                    continue
            except ValueError:
                print("    格式错误，请输入数字")
                continue
            points.append((parts[0], parts[1], parts[2],
                           f"({parts[0]},{parts[1]},{parts[2]})"))
    else:
        # 确保每个元素是 4 元组
        points = [(p[0], p[1], p[2], p[3] if len(p) > 3 else f"({p[0]},{p[1]},{p[2]})")
                  for p in points]

    if len(points) < 3:
        print(f"  错误: 至少需要 3 个点位，当前仅 {len(points)} 个")
        reader.close()
        return None

    measurements = []
    for i, (px, py, pz, label) in enumerate(points):
        print(f"\n  [{i + 1}/{len(points)}] 将 M1 放在 {label}，按 Enter 确认...", end="", flush=True)
        input()
        print("    读取中...", end=" ", flush=True)
        d = _read_distances_avg(reader)
        if d is None:
            print("✗ 未收到数据，标定失败")
            reader.close()
            return None
        measurements.append(d)
        print(f"{'  '.join(f'S{j + 1}={v:.2f}m' for j, v in enumerate(d))} ✓")

    reader.close()

    pos_xyz = [(px, py, pz) for px, py, pz, _ in points]
    anchors = _solve_anchors_from_measurements(measurements, pos_xyz)

    print(f"\n  ✓ 标定完成，锚点坐标：")
    _print_anchors(anchors)
    return anchors


def set_anchors_directly(num_anchors: Optional[int] = None) -> dict:
    """直接输入锚点坐标（跳过测量）。

    num_anchors: 锚点数量，如果为 None 则交互式询问
    """
    if num_anchors is None:
        n_str = input("  锚点数量 [默认=4]: ").strip()
        try:
            num_anchors = int(n_str) if n_str else 4
        except ValueError:
            num_anchors = 4

    print(f"\n  直接输入 {num_anchors} 个锚点坐标")
    print("  输入格式: x,y,z （如: 0,0,1.5）\n")

    anchors = {}
    for i in range(num_anchors):
        while True:
            val = input(f"  S{i + 1} 坐标 (x,y,z): ").strip()
            try:
                parts = [float(v.strip()) for v in val.split(",")]
                if len(parts) != 3:
                    print("    格式错误，请输入 x,y,z")
                    continue
                anchors[f"S{i + 1}"] = parts
                break
            except ValueError:
                print("    格式错误，请输入数字")

    print(f"\n  ✓ 锚点坐标已设置：")
    _print_anchors(anchors)
    return anchors


def calibrate_anchors_interactive(port: str, baud: int) -> Optional[dict]:
    """交互式标定入口 — 用户选择标定方式"""
    print("\n" + "=" * 50)
    print("  UWB 锚点标定")
    print("=" * 50)
    print("\n  选择标定方式:")
    print("    1. 步长标定 — 移动标签并测量实际距离")
    print("    2. 已知点位标定 — 将标签放在场地已知坐标点")
    print("    3. 直接输入锚点坐标（跳过测量）")

    while True:
        choice = input("\n  请输入 [1/2/3]: ").strip()
        if choice == "1":
            return calibrate_anchors(port, baud)
        elif choice == "2":
            return calibrate_anchors_with_points(port, baud)
        elif choice == "3":
            return set_anchors_directly()
        else:
            print("  无效选择，请输入 1、2 或 3")


def _parse_points_arg(arg: str) -> list:
    """解析 --cal-points / --set-anchors 参数。

    格式: "x1,y1,z1;x2,y2,z2;..."
    返回: [(x1,y1,z1), (x2,y2,z2), ...]
    """
    points = []
    for item in arg.split(";"):
        item = item.strip()
        if not item:
            continue
        parts = [float(v.strip()) for v in item.split(",")]
        if len(parts) != 3:
            raise ValueError(f"点位格式错误: '{item}'，需要 x,y,z")
        points.append(tuple(parts))
    return points


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
    parser.add_argument("--cal-points", default=None,
                        help="已知点位标定 (格式: \"x1,y1,z1;x2,y2,z2;...\"，至少3个点)")
    parser.add_argument("--set-anchors", default=None,
                        help="直接设置锚点坐标 (格式: \"x1,y1,z1;x2,y2,z2;...\")")
    parser.add_argument("--no-udp", action="store_true",
                        help="强制关闭 UDP 输出")
    parser.add_argument("--no-mavlink", action="store_true",
                        help="强制关闭 MAVLink 输出")
    parser.add_argument("--mavlink-host", default=None,
                        help="飞控 MAVLink 地址 (格式: host:port)")
    args = parser.parse_args()

    # ---- calibrate / set-anchors mode: configure anchors, save cache, exit ----
    if args.calibrate or args.set_anchors:
        # --set-anchors: 直接设置锚点，无需串口
        if args.set_anchors:
            try:
                pts = _parse_points_arg(args.set_anchors)
            except ValueError as e:
                print(f"  参数错误: {e}")
                sys.exit(1)
            anchors = {}
            for i, (x, y, z) in enumerate(pts):
                anchors[f"S{i + 1}"] = [x, y, z]
            print(f"\n  ✓ 锚点坐标已设置：")
            _print_anchors(anchors)
            save_anchors_to_config(args.config, anchors)
            save_calibration_cache(anchors)
            return

        # --calibrate: 需要串口
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

        # --cal-points: 非交互式已知点位标定
        if args.cal_points:
            try:
                pts = _parse_points_arg(args.cal_points)
            except ValueError as e:
                print(f"  参数错误: {e}")
                sys.exit(1)
            anchors = calibrate_anchors_with_points(port, baud, pts)
        else:
            # 交互式标定
            anchors = calibrate_anchors_interactive(port, baud)

        if anchors:
            save_anchors_to_config(args.config, anchors)
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
