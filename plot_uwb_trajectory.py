#!/usr/bin/env python3
"""
UWB 轨迹可视化

从调试串口日志里解析 [N] x=.. y=.. 行，画两张图：
  左  按时间上色的连续轨迹（起点绿、终点红）
  右  所有点半透明叠加，看定点停留簇

用法:
  python plot_uwb_trajectory.py 你的日志.txt
  python plot_uwb_trajectory.py 你的日志.txt -o out.png
可选:
  --anchors "0,0 5,0 0,5 5,5"   自定义锚点 XY(米)，空格分隔
  --no-ref                       不画十字/对角/外框参考虚线
"""
import re
import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection


def parse_log(path):
    """解析日志中的 x= y= 坐标，按出现顺序返回。"""
    txt = open(path, encoding="utf-8", errors="ignore").read()
    pat = re.compile(r"\[?\d+\]\s*x=([\-\d.]+)\s*y=([\-\d.]+)")
    xs, ys = [], []
    for line in txt.splitlines():
        m = pat.search(line)
        if m:
            xs.append(float(m.group(1)))
            ys.append(float(m.group(2)))
    return np.array(xs), np.array(ys)


def draw_anchors(ax, anc, labels):
    ax.scatter(anc[:, 0], anc[:, 1], marker="s", s=150, c="k", zorder=6)
    for (x, y), l in zip(anc, labels):
        ax.annotate(l, (x, y), textcoords="offset points",
                    xytext=(7, 7), fontsize=11, fontweight="bold")


def draw_reference(ax, anc):
    """外围矩形 + 十字 + 副对角线参考虚线（按锚点包围盒画）。"""
    x0, y0 = anc[:, 0].min(), anc[:, 1].min()
    x1, y1 = anc[:, 0].max(), anc[:, 1].max()
    xm, ym = (x0 + x1) / 2, (y0 + y1) / 2
    ax.plot([x0, x1, x1, x0, x0], [y0, y0, y1, y1, y0], ls=":", c="0.75", lw=1)
    ax.plot([x0, x1], [y0, y1], ls=":", c="0.8", lw=1)
    ax.plot([xm, xm], [y0, y1], ls=":", c="0.8", lw=1)
    ax.plot([x0, x1], [ym, ym], ls=":", c="0.8", lw=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", help="调试串口日志文件")
    ap.add_argument("-o", "--out", default="uwb_trajectory.png", help="输出图片路径")
    ap.add_argument("--anchors", default="0,0 5,0 0,5 5,5",
                    help='锚点XY(米)，如 "0,0 5,0 0,5 5,5"')
    ap.add_argument("--no-ref", action="store_true", help="不画参考虚线")
    args = ap.parse_args()

    anc = np.array([[float(v) for v in p.split(",")]
                    for p in args.anchors.split()])
    labels = [f"S{i+1}" for i in range(len(anc))]

    X, Y = parse_log(args.log)
    n = len(X)
    if n == 0:
        raise SystemExit("没解析到坐标，检查日志格式是否为 [N] x=.. y=..")

    pad = 0.7
    xlo, xhi = anc[:, 0].min() - pad, anc[:, 0].max() + pad
    ylo, yhi = anc[:, 1].min() - pad, anc[:, 1].max() + pad

    fig, ax = plt.subplots(1, 2, figsize=(15, 7.2))

    # 左：按时间上色的连续轨迹
    a = ax[0]
    pts = np.array([X, Y]).T.reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    lc = LineCollection(segs, cmap="viridis", linewidth=1.4)
    lc.set_array(np.arange(n))
    a.add_collection(lc)
    draw_anchors(a, anc, labels)
    if not args.no_ref:
        draw_reference(a, anc)
    a.scatter(X[0], Y[0], c="lime", s=90, zorder=7, edgecolor="k", label="start")
    a.scatter(X[-1], Y[-1], c="red", s=90, zorder=7, edgecolor="k", label="end")
    cb = fig.colorbar(lc, ax=a, fraction=0.046, pad=0.02)
    cb.set_label("sample index (time)")
    a.set_title(f"Walked trajectory over time ({n} points)", fontsize=11)
    a.legend(loc="upper right", fontsize=9)

    # 右：所有点半透明叠加，看停留/测点簇
    a = ax[1]
    draw_anchors(a, anc, labels)
    if not args.no_ref:
        draw_reference(a, anc)
    a.scatter(X, Y, s=10, c="tab:blue", alpha=0.25)
    a.set_title("All points overlaid (dark = dwell spots)", fontsize=11)

    for a in ax:
        a.set_xlabel("X / East-ish (m)")
        a.set_ylabel("Y / North-ish (m)")
        a.set_aspect("equal")
        a.set_xlim(xlo, xhi)
        a.set_ylim(ylo, yhi)

    plt.tight_layout()
    plt.savefig(args.out, dpi=130, bbox_inches="tight")
    print(f"saved {args.out}  points={n}  "
          f"X[{X.min():.2f},{X.max():.2f}] Y[{Y.min():.2f},{Y.max():.2f}]")


if __name__ == "__main__":
    main()
