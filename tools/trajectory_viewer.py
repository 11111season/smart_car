#!/usr/bin/env python3
"""
V形端点轨迹可视化工具
======================
读取 hough_logger.py 录制的 CSV 文件，绘制 vertex/base1/base2/mid 轨迹图，
直观判断跟踪框是否"卡住"以及单臂退化是否发生。

用法:
    python trajectory_viewer.py <csv文件>              # 基本用法
    python trajectory_viewer.py <csv文件> --stuck-only # 只显示有卡住区域
    python trajectory_viewer.py <csv文件> --save <图.png>  # 保存到文件
"""

import sys
import csv
import math
import argparse
from collections import defaultdict

try:
    import matplotlib
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
except ImportError:
    print("请先安装 matplotlib: pip install matplotlib")
    sys.exit(1)

# ── 解析参数 ──
parser = argparse.ArgumentParser(description="V形端点轨迹可视化")
parser.add_argument("csv_file", help="hough_logger.py 录制的 CSV 文件路径")
parser.add_argument("--stuck-only", action="store_true", help="只放大显示卡住区域")
parser.add_argument("--save", help="保存图片到文件 (如 trajectory.png)")
parser.add_argument("--camera-w", type=int, default=188, help="图像宽度 (默认 188)")
parser.add_argument("--camera-h", type=int, default=120, help="图像高度 (默认 120)")
args = parser.parse_args()

# ── 读取 CSV ──
rows = []
with open(args.csv_file, newline="", encoding="utf-8") as f:
    reader = csv.DictReader(f)
    for r in reader:
        rows.append(r)

if not rows:
    print("错误: CSV 文件为空")
    sys.exit(1)

print(f"读取了 {len(rows)} 帧数据")

# ── 分类 ──
v_found_rows = [r for r in rows if r.get("v_found") == "1"]
print(f"V形检出: {len(v_found_rows)}/{len(rows)} ({len(v_found_rows)/len(rows)*100:.1f}%)")

# ── 检测退化帧 (base1==base2) ──
degenerate = []
for r in v_found_rows:
    b1x, b1y = int(r["bx"]), int(r["by"])
    b2x, b2y = int(r["cx_e"]), int(r["cy_e"])
    if b1x == b2x and b1y == b2y:
        degenerate.append(r)

if degenerate:
    print(f"单臂退化帧: {len(degenerate)}/{len(v_found_rows)}")
    # 统计退化连续段
    degen_ids = {int(r["frame_id"]) for r in degenerate}
    sorted_ids = sorted(degen_ids)
    runs = []
    start = sorted_ids[0]
    prev = sorted_ids[0]
    for fid in sorted_ids[1:]:
        if fid != prev + 1:
            runs.append((start, prev))
            start = fid
        prev = fid
    runs.append((start, prev))
    for s, e in runs:
        cnt = e - s + 1
        r = degenerate[[i for i, d in enumerate(degenerate) if int(d["frame_id"]) == s][0]]
        print(f"  退化段: frame {s}~{e} ({cnt}帧) mid=({r['mid_x']},{r['mid_y']})")
else:
    print("无单臂退化帧 ✓")

# ── 检测中点完全不变的帧 (卡住) ──
stuck_frames = []
for i in range(1, len(v_found_rows)):
    rx0, ry0 = int(v_found_rows[i-1]["mid_x"]), int(v_found_rows[i-1]["mid_y"])
    rx1, ry1 = int(v_found_rows[i]["mid_x"]), int(v_found_rows[i]["mid_y"])
    if rx0 == rx1 and ry0 == ry1:
        stuck_frames.append(int(v_found_rows[i]["frame_id"]))

if stuck_frames:
    print(f"\n中点不变帧 (卡住): {len(stuck_frames)}/{len(v_found_rows)}")
    # 找连续卡住段
    stuck_sorted = sorted(set(stuck_frames))
    runs = []
    start = stuck_sorted[0]
    prev = stuck_sorted[0]
    for fid in stuck_sorted[1:]:
        if fid != prev + 1:
            runs.append((start, prev))
            start = fid
        prev = fid
    runs.append((start, prev))
    for s, e in runs:
        cnt = e - s + 1
        if cnt >= 2:
            print(f"  卡住段: frame {s}~{e} ({cnt}帧)")
else:
    print("\n无卡住帧 ✓")

# ── 提取轨迹数据 ──
frames = []
vx_list, vy_list = [], []
b1x_list, b1y_list = [], []
b2x_list, b2y_list = [], []
mx_list, my_list = [], []
rx_list, ry_list = [], []
area_list = []

for r in v_found_rows:
    frames.append(int(r["frame_id"]))
    vx_list.append(int(r.get("cx", 0)))       # blob cx 作为顶点参考
    vy_list.append(int(r.get("cy", 0)))
    b1x_list.append(int(r["bx"]))
    b1y_list.append(int(r["by"]))
    b2x_list.append(int(r["cx_e"]))
    b2y_list.append(int(r["cy_e"]))
    mx_list.append(int(r["mid_x"]))
    my_list.append(int(r["mid_y"]))
    rx_list.append(int(r.get("raw_mid_x", 0)))
    ry_list.append(int(r.get("raw_mid_y", 0)))
    area_list.append(int(r.get("area", 0)))

# ── 绘制 ──
fig, axes = plt.subplots(2, 2, figsize=(16, 9), gridspec_kw={
    "width_ratios": [2, 1], "height_ratios": [1, 1]})
fig.suptitle("V-shape Endpoint Trajectory Analysis", fontsize=14, fontweight="bold")

ax_traj = axes[0, 0]  # 左上: 2D轨迹
ax_x = axes[1, 0]     # 左下: X坐标时序
ax_y_ts = axes[0, 1]  # 右上: Y坐标时序
ax_info = axes[1, 1]  # 右下: 统计信息

# ── 2D 轨迹 ──
ax_traj.set_title("Camera-plane trajectory (x, y)")
ax_traj.set_xlim(0, args.camera_w)
ax_traj.set_ylim(args.camera_h, 0)  # 图像Y轴向下
ax_traj.set_xlabel("x (px)")
ax_traj.set_ylabel("y (px)")
ax_traj.grid(True, alpha=0.3)
ax_traj.set_aspect("equal")
ax_traj.invert_yaxis()  # 图像坐标系: y向下

# 画图像边界
ax_traj.add_patch(Rectangle((0, 0), args.camera_w, args.camera_h,
                             fill=False, edgecolor="gray", linestyle="--", alpha=0.5))

# 画轨迹线 + 点
alpha_val = min(1.0, 200.0 / max(len(frames), 1))

# base1 轨迹 (蓝色)
ax_traj.plot(b1x_list, b1y_list, "b-", alpha=alpha_val*0.5, linewidth=0.8, label="base1")
ax_traj.scatter(b1x_list, b1y_list, c="blue", s=8, alpha=alpha_val, marker="o")

# base2 轨迹 (橙色)
ax_traj.plot(b2x_list, b2y_list, "orange", alpha=alpha_val*0.5, linewidth=0.8, label="base2")
ax_traj.scatter(b2x_list, b2y_list, c="orange", s=8, alpha=alpha_val, marker="s")

# mid 轨迹 (红色, 粗线)
ax_traj.plot(mx_list, my_list, "r-", linewidth=1.5, alpha=0.8, label="mid (平滑)")
ax_traj.scatter(mx_list, my_list, c="red", s=15, alpha=0.6, marker="D", zorder=3)

# raw_mid 轨迹 (绿色虚线)
ax_traj.plot(rx_list, ry_list, "g--", alpha=alpha_val*0.5, linewidth=0.8, label="mid (原始)")

# 标记退化帧
for r in degenerate:
    fidx = v_found_rows.index(r)
    ax_traj.scatter(mx_list[fidx], my_list[fidx], c="magenta",
                    s=80, marker="x", zorder=5, alpha=0.8)

ax_traj.legend(fontsize=8, loc="upper right")

# ── X 时序 ──
ax_x.set_title("X coordinate over frame")
ax_x.set_xlabel("frame")
ax_x.set_ylabel("x (px)")
ax_x.grid(True, alpha=0.3)

ax_x.plot(frames, b1x_list, "b-", alpha=0.6, linewidth=0.8, label="base1_x")
ax_x.plot(frames, b2x_list, "orange", alpha=0.6, linewidth=0.8, label="base2_x")
ax_x.plot(frames, mx_list, "r-", linewidth=1.5, label="mid_x (平滑)")
ax_x.plot(frames, rx_list, "g--", alpha=0.6, linewidth=0.8, label="raw_mid_x")

# 标记退化帧
for r in degenerate:
    fidx = v_found_rows.index(r)
    ax_x.axvline(x=frames[fidx], color="magenta", alpha=0.3, linewidth=0.5)

ax_x.legend(fontsize=8)

# ── Y 时序 ──
ax_y_ts.set_title("Y coordinate over frame")
ax_y_ts.set_xlabel("frame")
ax_y_ts.set_ylabel("y (px)")
ax_y_ts.grid(True, alpha=0.3)

ax_y_ts.plot(frames, b1y_list, "b-", alpha=0.6, linewidth=0.8, label="base1_y")
ax_y_ts.plot(frames, b2y_list, "orange", alpha=0.6, linewidth=0.8, label="base2_y")
ax_y_ts.plot(frames, my_list, "r-", linewidth=1.5, label="mid_y (平滑)")
ax_y_ts.plot(frames, ry_list, "g--", alpha=0.6, linewidth=0.8, label="raw_mid_y")

for r in degenerate:
    fidx = v_found_rows.index(r)
    ax_y_ts.axvline(x=frames[fidx], color="magenta", alpha=0.3, linewidth=0.5)

ax_y_ts.legend(fontsize=8)

# ── 统计信息 ──
ax_info.set_title("Statistics Summary")
ax_info.axis("off")

# 计算中点帧间漂移
mid_drifts = []
for i in range(1, len(v_found_rows)):
    dm = math.sqrt((mx_list[i] - mx_list[i-1])**2 + (my_list[i] - my_list[i-1])**2)
    mid_drifts.append(dm)

stats_lines = [
    f"Total frames: {len(rows)}",
    f"V-shape detected: {len(v_found_rows)} ({len(v_found_rows)/len(rows)*100:.1f}%)",
    f"Single-arm degenerate: {len(degenerate)}",
]
if mid_drifts:
    avg_drift = sum(mid_drifts) / len(mid_drifts)
    max_drift = max(mid_drifts)
    stuck_count = sum(1 for d in mid_drifts if d < 2)
    stats_lines.extend([
        f"",
        f"Mid-point frame drift:",
        f"  mean={avg_drift:.1f}px  max={max_drift:.0f}px",
        f"  unchanged(<2px): {stuck_count}/{len(mid_drifts)}",
    ])
    if stuck_frames:
        stats_lines.append(f"  stuck segments: {len(runs)}")

# 原始vs平滑偏离
if any(rx_list) and any(ry_list):
    raw_deltas = []
    for i in range(len(v_found_rows)):
        if rx_list[i] == 0 and ry_list[i] == 0:
            continue
        d = math.sqrt((mx_list[i] - rx_list[i])**2 + (my_list[i] - ry_list[i])**2)
        raw_deltas.append(d)
    if raw_deltas:
        stats_lines.extend([
            f"",
            f"Smooth-raw deviation:",
            f"  mean={sum(raw_deltas)/len(raw_deltas):.1f}px",
            f"  max={max(raw_deltas):.0f}px",
            f"  >5px: {sum(1 for d in raw_deltas if d>5)}/{len(raw_deltas)}",
        ])

# 退化帧占比
if len(degenerate) > 0:
    stats_lines.append(f"")
    stats_lines.append(f"Degenerate rate: {len(degenerate)/len(v_found_rows)*100:.1f}%")

ax_info.text(0.05, 0.95, "\n".join(stats_lines),
             transform=ax_info.transAxes, fontsize=10,
             verticalalignment="top", fontfamily="monospace")

plt.tight_layout()

if args.save:
    plt.savefig(args.save, dpi=150, bbox_inches="tight")
    print(f"\n图片已保存: {args.save}")
else:
    plt.show()
