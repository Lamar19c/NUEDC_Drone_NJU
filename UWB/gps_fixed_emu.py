#!/usr/bin/env python3
"""
固定GPS信号模拟器 — 持续发送固定经纬度NMEA语句到飞控GPS端口

用途: 验证飞控能否识别GPS信号（不依赖UWB，输出恒定位置）
      飞控端 SERIALx_PROTOCOL=5 (GPS) 即可接收

用法:
    python gps_fixed_emu.py COM4
    python gps_fixed_emu.py /dev/ttyS6 --baud 57600 --rate 5
    python gps_fixed_emu.py COM4 --lat 32.1148408 --lon 118.9590664 --alt 12.0
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import serial


# ═══════════════════════════════════════════════════════════════════════════════
# NMEA 工具函数 (与 uwb.py 完全一致)
# ═══════════════════════════════════════════════════════════════════════════════


def _deg_to_nmea(lat_deg: float, lon_deg: float) -> tuple:
    """十进制度 → NMEA ddmm.mmmm 格式

    返回 (lat_str, lat_dir, lon_str, lon_dir)
    例: (32.1148408, 118.9590664) → ("3206.8904", "N", "11857.5440", "E")
    """
    lat_abs = abs(lat_deg)
    lat_d = int(lat_abs)
    lat_m = (lat_abs - lat_d) * 60.0
    lat_str = f"{lat_d:02d}{lat_m:07.4f}"
    lat_dir = "N" if lat_deg >= 0 else "S"

    lon_abs = abs(lon_deg)
    lon_d = int(lon_abs)
    lon_m = (lon_abs - lon_d) * 60.0
    lon_str = f"{lon_d:03d}{lon_m:07.4f}"
    lon_dir = "E" if lon_deg >= 0 else "W"

    return (lat_str, lat_dir, lon_str, lon_dir)


def _nmea_checksum(sentence: str) -> str:
    """NMEA 异或校验和 (XOR of all chars between $ and *)"""
    cksum = 0
    for c in sentence[1:]:
        cksum ^= ord(c)
    return f"{cksum:02X}"


def build_nmea_sentences(
    lat_deg: float,
    lon_deg: float,
    alt_m: float,
    speed_kn: float = 0.0,
    course_deg: float = 0.0,
    hdop: float = 1.5,
    satellites: int = 10,
    timestamp: float | None = None,
) -> tuple:
    """生成一组 $GPGGA + $GPRMC + $GPVTG 语句 (与 u-blox NEO-M9N NMEA 4.10 对齐)

    参数:
        lat_deg, lon_deg: 十进制度经纬度
        alt_m: 海拔高度(米)
        speed_kn: 地面速率(节)
        course_deg: 地面航向(度, 真北)
        hdop: 水平精度因子 (1.0~2.0, 默认 1.5)
        satellites: 可见卫星数 (默认 10)
        timestamp: Unix时间戳, None=当前时间

    返回: (ggpa_str, rmc_str, vtg_str) — 带 \\r\\n 的完整NMEA语句
    """
    if timestamp is None:
        timestamp = time.time()

    dt = time.localtime(timestamp)
    time_str = time.strftime("%H%M%S", dt)
    date_str = time.strftime("%d%m%y", dt)

    lat_str, lat_dir, lon_str, lon_dir = _deg_to_nmea(lat_deg, lon_deg)

    speed_kmh = speed_kn * 1.852   # knots → km/h

    # $GPGGA — GPS Fix Data
    ggpa_body = (
        f"$GPGGA,{time_str}.00,{lat_str},{lat_dir},{lon_str},{lon_dir},"
        f"1,{satellites:02d},{hdop:.1f},{alt_m:.1f},M,0.0,M,,"
    )
    ggpa = f"{ggpa_body}*{_nmea_checksum(ggpa_body)}\r\n"

    # $GPRMC — Recommended Minimum Navigation Information
    rmc_body = (
        f"$GPRMC,{time_str}.00,A,{lat_str},{lat_dir},{lon_str},{lon_dir},"
        f"{speed_kn:.2f},{course_deg:.1f},{date_str},0.0,E,A"
    )
    rmc = f"{rmc_body}*{_nmea_checksum(rmc_body)}\r\n"

    # $GPVTG — Course Over Ground and Ground Speed
    vtg_body = (
        f"$GPVTG,{course_deg:.1f},T,0.0,M,"
        f"{speed_kn:.2f},N,{speed_kmh:.2f},K,A"
    )
    vtg = f"{vtg_body}*{_nmea_checksum(vtg_body)}\r\n"

    return (ggpa, rmc, vtg)


# ═══════════════════════════════════════════════════════════════════════════════
# 主程序
# ═══════════════════════════════════════════════════════════════════════════════


def main():
    parser = argparse.ArgumentParser(
        description="固定GPS信号模拟器 — 持续发送NMEA语句到飞控GPS端口",
    )
    parser.add_argument(
        "port",
        help="飞控GPS串口 (Windows: COM4, Linux: /dev/ttyS6)",
    )
    parser.add_argument(
        "--baud", type=int, default=57600,
        help="串口波特率 (默认: 57600, 匹配u-blox NEO-M9N)",
    )
    parser.add_argument(
        "--rate", type=float, default=5.0,
        help="NMEA发送频率Hz (默认: 5)",
    )
    parser.add_argument(
        "--lat", type=float, default=32.1148408,
        help="固定纬度(十进制度, 默认: 32.1148408 = 南京大学仙林)",
    )
    parser.add_argument(
        "--lon", type=float, default=118.9590664,
        help="固定经度(十进制度, 默认: 118.9590664)",
    )
    parser.add_argument(
        "--alt", type=float, default=12.0,
        help="固定海拔高度(米, 默认: 12.0)",
    )
    parser.add_argument(
        "--speed", type=float, default=0.0,
        help="模拟地面速率(节, 默认: 0 = 静止)",
    )
    parser.add_argument(
        "--verbose", action="store_true",
        help="打印每条发送的NMEA语句（诊断用）",
    )
    parser.add_argument(
        "--hdop", type=float, default=1.5,
        help="HDOP 水平精度因子 (默认: 1.5, u-blox 典型范围 1.0~2.0)",
    )
    parser.add_argument(
        "--sats", type=int, default=10,
        help="可见卫星数 (默认: 10, u-blox 典型范围 8~16)",
    )
    args = parser.parse_args()

    # ── 1. 打开串口 ──
    print(f"\n  打开串口: {args.port} @ {args.baud} baud ...", end=" ", flush=True)
    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0,
            write_timeout=0.5,
        )
    except Exception as e:
        print(f"✗ 失败")
        print(f"\n  错误: {e}")
        print(f"  请检查:")
        print(f"    1. 串口 {args.port} 是否存在")
        print(f"    2. 是否被其他程序占用")
        print(f"    3. 是否有读写权限")
        sys.exit(1)
    print("✓")

    # ── 2. 打印配置 ──
    print(f"\n  ═══════════════════════════════════════")
    print(f"  固定GPS信号模拟器")
    print(f"  ═══════════════════════════════════════")
    print(f"  串口:     {args.port} @ {args.baud} baud")
    print(f"  频率:     {args.rate} Hz (间隔 {1/args.rate:.3f}s)")
    print(f"  纬度:     {args.lat:.7f}°")
    print(f"  经度:     {args.lon:.7f}°")
    print(f"  海拔:     {args.alt:.1f} m")
    print(f"  速率:     {args.speed:.1f} kn")
    print(f"  ═══════════════════════════════════════")

    # ── 3. 预生成 NMEA 样例并打印 ──
    ggpa_sample, rmc_sample, vtg_sample = build_nmea_sentences(
        args.lat, args.lon, args.alt, args.speed,
        hdop=args.hdop, satellites=args.sats,
    )
    print(f"\n  NMEA 样例 (每次发送时时间戳更新):")
    print(f"    {ggpa_sample.strip()}")
    print(f"    {rmc_sample.strip()}")
    print(f"    {vtg_sample.strip()}")

    # ── 4. 主循环 ──
    interval = 1.0 / args.rate
    count = 0
    print(f"\n  开始发送 (Ctrl+C 停止)...\n")

    try:
        while True:
            now = time.time()
            ggpa, rmc, vtg = build_nmea_sentences(
                args.lat, args.lon, args.alt, args.speed,
                hdop=args.hdop, satellites=args.sats, timestamp=now,
            )

            try:
                n1 = ser.write(ggpa.encode("ascii"))
                n2 = ser.write(rmc.encode("ascii"))
                n3 = ser.write(vtg.encode("ascii"))
                ser.flush()  # 强制刷新，确保数据立刻送出
            except (OSError, serial.SerialException) as e:
                print(f"  ⚠ 串口写入失败 (第{count+1}组): {e}")
                break

            count += 1

            # 首条：打印实际发送内容 + 字节数
            if count == 1:
                print(f"  ✓ 首条已发送 ({n1}+{n2}+{n3}={n1+n2+n3} bytes) → {args.port}")
                print(f"    {ggpa.strip()}")
                print(f"    {rmc.strip()}")
                print(f"    {vtg.strip()}")
                print(f"    飞控应能识别GPS定位: {args.lat:.7f}, {args.lon:.7f}")

            # --verbose: 每条都打印
            if args.verbose:
                print(f"  [{count}] {ggpa.strip()}")
                print(f"  [{count}] {rmc.strip()}")
                print(f"  [{count}] {vtg.strip()}")

            # 定期心跳
            if count % max(1, int(args.rate * 5)) == 0:  # 每5秒
                ts = time.strftime("%H:%M:%S")
                print(f"  [{ts}] 已发送 {count} 组 NMEA → {args.port}")

            # 维持精确频率
            elapsed = time.time() - now
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        pass

    finally:
        ser.close()
        print(f"\n  已停止, 共发送 {count} 组 NMEA 语句")


if __name__ == "__main__":
    main()
