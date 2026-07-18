#!/usr/bin/env python3
"""
UWB 全链路诊断：定位"快大慢小"出在哪一层

解析固件 DBG 行：
  DBG t=.. d=d1,d2,d3,d4 age=a1,a2,a3,a4 ekf=x,y v=vx,vy e7=lat_e7,lon_e7

产出三张对比：
  (a) EKF 原始解轨迹（未量化）  按速度上色
  (b) 4 位 NMEA 量化后轨迹（飞控实际收到）
  (c) 两者逐点差 vs 速度  —— 量化误差随不随速度变
并打印时间偏斜统计（锚点数据年龄差）。

判读：
  若 (a) 的圈随速度变大小 -> 问题在 EKF 层(时间补偿/q)
  若 (a) 干净、(b) 才出现快大慢小 -> 问题是 4 位量化，与 EKF 无关

用法:
  python plot_uwb_diagnostic.py 调试日志.txt
  可选 --origin LAT_E7,LON_E7 (默认 321148408,1189590664)
"""
import re
import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

DBG = re.compile(
    r"DBG\s+t=(\d+)\s+d=([\-\d]+),([\-\d]+),([\-\d]+),([\-\d]+)\s+"
    r"age=(\d+),(\d+),(\d+),(\d+)\s+"
    r"ekf=([\-\d]+),([\-\d]+)\s+v=([\-\d]+),([\-\d]+)\s+"
    r"e7=([\-\d]+),([\-\d]+)")


def parse(path):
    rows = []
    for line in open(path, encoding="utf-8", errors="ignore"):
        m = DBG.search(line)
        if m:
            g = m.groups()
            rows.append([int(v) for v in g])
    return np.array(rows, dtype=np.int64)


def quantize_4dp(e7, width, m_per_deg, origin_deg):
    """复现固件 4 位小数分量化：e7 度 -> ddmm.mmmm -> 解码回本地米。"""
    a = np.abs(e7).astype(np.float64)
    deg_int = a // 10000000
    deg_frac_e7 = a % 10000000
    min_e7 = deg_frac_e7 * 60
    min_int = min_e7 // 10000000
    min_frac_e7 = min_e7 % 10000000
    min_frac4 = (min_frac_e7 + 500) // 1000        # 0..10000
    carry = min_frac4 >= 10000
    min_frac4 = np.where(carry, min_frac4 - 10000, min_frac4)
    min_int = min_int + carry
    q_deg = deg_int + (min_int + min_frac4 / 1e4) / 60.0
    q_deg = np.sign(e7) * q_deg
    return (q_deg - origin_deg) * m_per_deg      # 本地米


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("-o", "--out", default="uwb_diagnostic.png")
    ap.add_argument("--origin", default="321148408,1189590664",
                    help="原点 LAT_E7,LON_E7")
    args = ap.parse_args()
    olat_e7, olon_e7 = [int(v) for v in args.origin.split(",")]
    olat, olon = olat_e7 / 1e7, olon_e7 / 1e7
    m_lat = 111319.9
    m_lon = 111319.9 * np.cos(np.radians(olat))

    R = parse(args.log)
    if len(R) == 0:
        raise SystemExit("没解析到 DBG 行，确认固件在打印 DBG t=.. 那种行")

    t = R[:, 0].astype(float) / 1000.0
    d = R[:, 1:5].astype(float) / 100.0            # 原始距离 m
    age = R[:, 5:9].astype(float)                   # ms
    ekf_x = R[:, 9].astype(float) / 100.0           # EKF 原始 m
    ekf_y = R[:, 10].astype(float) / 100.0
    vx = R[:, 11].astype(float) / 100.0
    vy = R[:, 12].astype(float) / 100.0
    lat_e7 = R[:, 13]                                # 编码 y
    lon_e7 = R[:, 14]                                # 编码 x
    spd = np.hypot(vx, vy)

    # 量化后本地坐标（lat->y, lon->x）
    q_y = quantize_4dp(lat_e7, 2, m_lat, olat)
    q_x = quantize_4dp(lon_e7, 3, m_lon, olon)
    q_err = np.hypot(q_x - ekf_x, q_y - ekf_y)      # 量化误差 m

    # 时间偏斜
    skew = age.max(axis=1) - age.min(axis=1)        # ms
    print(f"帧数 {len(R)}  速度[{spd.min():.2f},{spd.max():.2f}]m/s")
    print(f"时间偏斜(锚点年龄差) 中位{np.median(skew):.0f}ms 最大{skew.max():.0f}ms")
    print(f"量化误差 均值{q_err.mean()*100:.1f}cm 最大{q_err.max()*100:.1f}cm")

    fig, ax = plt.subplots(1, 3, figsize=(19, 6.3))

    def color_traj(a, X, Y, c, title):
        pts = np.array([X, Y]).T.reshape(-1, 1, 2)
        segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
        lc = LineCollection(segs, cmap="plasma", linewidth=1.4)
        lc.set_array(c[:-1])
        a.add_collection(lc)
        a.set_aspect("equal")
        a.set_title(title, fontsize=11)
        a.set_xlabel("X/East (m)"); a.set_ylabel("Y/North (m)")
        a.autoscale()
        return lc

    lc0 = color_traj(ax[0], ekf_x, ekf_y, spd,
                     "(a) EKF raw solution (unquantized)\ncolor = speed")
    fig.colorbar(lc0, ax=ax[0], fraction=0.046, label="speed m/s")

    lc1 = color_traj(ax[1], q_x, q_y, spd,
                     "(b) After 4-dp NMEA quantization\n(what FC receives)")
    fig.colorbar(lc1, ax=ax[1], fraction=0.046, label="speed m/s")

    ax[2].scatter(spd, q_err * 100, s=8, c="tab:red", alpha=0.5)
    ax[2].set_title("(c) Quantization error vs speed\nflat = grid effect not speed-driven",
                    fontsize=11)
    ax[2].set_xlabel("speed (m/s)"); ax[2].set_ylabel("|quantized - EKF| (cm)")

    plt.tight_layout()
    plt.savefig(args.out, dpi=130, bbox_inches="tight")
    print(f"saved {args.out}")


if __name__ == "__main__":
    main()
