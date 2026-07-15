#!/usr/bin/env python3
"""
实时视觉识别上位机 (双视图版)
============================
左: 虚拟坐标图 (V形+信标)  右: 摄像头画面 (叠加识别标记)

依赖:
    pip install pyserial matplotlib numpy opencv-python

用法:
    python realtime_monitor.py COM25 115200 --camera
    python realtime_monitor.py COM25 115200 --camera 1
"""

import sys, threading, time, argparse, math
from collections import deque

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

try:
    import serial
except ImportError:
    print("请先安装 pyserial: pip install pyserial")
    sys.exit(1)

try:
    import cv2
    import numpy as np
    HAVE_CV2 = True
except ImportError:
    HAVE_CV2 = False

parser = argparse.ArgumentParser(description="实时视觉识别上位机 (双视图版)")
parser.add_argument("port", help="串口号 (如 COM25)")
parser.add_argument("baud", type=int, nargs="?", default=115200, help="波特率 (默认 115200)")
parser.add_argument("--camera", nargs="?", const=0, type=int, default=None,
                    help="启用摄像头 (可选索引, 默认 0)")
args = parser.parse_args()

# ── 数据 ──
MAX_HISTORY = 60
car_data   = deque(maxlen=MAX_HISTORY)
beacon_xy  = None
beacon_hx  = deque(maxlen=MAX_HISTORY)
beacon_hy  = deque(maxlen=MAX_HISTORY)
timestamps = deque(maxlen=MAX_HISTORY)
frame_cnt = 0
last_h_time = 0.0
last_b_time = 0.0
TIMEOUT = 0.5
lock = threading.Lock()
running = True

car_latest = None
beacon_latest = None
camera_frame = None
latest_heading = 0.0  # 最新航向角 (独立存储, 无车也画坐标轴)
CAM_W, CAM_H = 188, 120

# ── 串口 ──
def serial_reader():
    global frame_cnt, car_latest, beacon_latest
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except Exception as e:
        print(f"无法打开串口 {args.port}: {e}")
        return
    print(f"已连接 {args.port} @ {args.baud}")
    buf = b""
    while running:
        try:
            data = ser.read(1024)
            if not data: continue
            buf += data
            while b"\r\n" in buf or b"\n" in buf:
                if b"\r\n" in buf: line, buf = buf.split(b"\r\n", 1)
                else:              line, buf = buf.split(b"\n", 1)
                text = line.decode("utf-8", errors="replace").strip()
                if text: parse_line(text)
        except serial.SerialException: break
        except Exception: continue
    ser.close()

def parse_line(text: str):
    global frame_cnt, beacon_xy, last_h_time, last_b_time, car_latest, beacon_latest, latest_heading
    parts = text.split(",")
    if not parts: return
    if text.startswith("H,") and len(parts) >= 16:
        try:
            d = {"frame_id":int(parts[1]),"b1x":int(parts[2]),"b1y":int(parts[3]),
                 "b2x":int(parts[4]),"b2y":int(parts[5]),"vx":int(parts[6]),"vy":int(parts[7]),
                 "angle":int(parts[8]),"height":float(parts[9]),"heading":float(parts[10]),
                 "speed":float(parts[11]),"roll":float(parts[12]),"pitch":float(parts[13]),
                 "vel_tgt_x":float(parts[14]),"vel_tgt_y":float(parts[15]),
                 "world_vy":float(parts[16]) if len(parts) >= 17 else 0,
                 "ff_vel_x":float(parts[17]) if len(parts) >= 19 else 0,
                 "ff_vel_y":float(parts[18]) if len(parts) >= 19 else 0}
            with lock:
                car_data.append(d); car_latest = d
                timestamps.append(time.time()); last_h_time = time.time(); frame_cnt += 1
        except: pass
    elif text.startswith("B,") and len(parts) >= 4:
        try:
            with lock:
                beacon_xy = (int(parts[1]), int(parts[2]), int(parts[3]))
                beacon_latest = beacon_xy
                beacon_hx.append(int(parts[1])); beacon_hy.append(int(parts[2]))
                last_b_time = time.time()
        except: pass
    elif text.startswith("I,") and len(parts) >= 9:
        try:
            d = {"frame_id":int(parts[1]),"b1x":0,"b1y":0,"b2x":0,"b2y":0,"vx":0,"vy":0,
                 "angle":0,"height":float(parts[2]),"heading":float(parts[3]),
                 "speed":float(parts[4]),"roll":float(parts[5]),"pitch":float(parts[6]),
                 "vel_tgt_x":float(parts[7]),"vel_tgt_y":float(parts[8]),
                 "world_vy":float(parts[9]) if len(parts) >= 10 else 0,
                 "ff_vel_x":0.0,"ff_vel_y":0.0}
            with lock:
                car_data.append(d); car_latest = d
                timestamps.append(time.time()); last_h_time = time.time(); frame_cnt += 1
        except: pass

# ── 摄像头 ──
def camera_thread_func():
    global camera_frame
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"无法打开摄像头 (索引 {args.camera})")
        return
    print(f"摄像头已打开 (索引 {args.camera})")
    while running:
        ret, frame = cap.read()
        if not ret: time.sleep(0.05); continue
        with lock: camera_frame = frame.copy()
    cap.release()

# ── 获取摄像头分辨率 ──
cam_res = (CAM_W, CAM_H)
if HAVE_CV2 and args.camera is not None:
    tmp = cv2.VideoCapture(args.camera)
    if tmp.isOpened():
        ret, f0 = tmp.read()
        if ret: cam_res = (f0.shape[1], f0.shape[0])
        tmp.release()
cw, ch = cam_res

# ── 创建双视图 ──
fig, (ax_left, ax_right) = plt.subplots(
    1, 2, figsize=(12, 5),
    gridspec_kw={"width_ratios": [1, 2.5]}
)
fig.set_dpi(100)
fig.suptitle(f"Real-time Vision Monitor - {args.port} @ {args.baud}")

# 左图: 虚拟坐标
ax_left.set_xlim(0, CAM_W)
ax_left.set_ylim(CAM_H, 0)
ax_left.set_xlabel("x (px)"); ax_left.set_ylabel("y (px)")
ax_left.grid(True, alpha=0.15); ax_left.set_aspect("equal")
ax_left.set_title("Virtual Coordinate View")

# 右图: 摄像头
ax_right.set_xlim(0, cw); ax_right.set_ylim(ch, 0)
ax_right.set_aspect("equal")
ax_right.set_title("Camera View")
ax_right.set_xticks([]); ax_right.set_yticks([])  # 省去坐标轴绘制
cam_img = ax_right.imshow(np.zeros((ch, cw, 3), dtype=np.uint8),
                           extent=[0, cw, ch, 0], alpha=1.0, zorder=0)

def update_plot():
    with lock:
        cars = list(car_data); beac = beacon_xy
        ts = list(timestamps); n_frames = frame_cnt
        h_age = time.time() - last_h_time; b_age = time.time() - last_b_time
        frame = camera_frame

    # ── 左图: 虚拟坐标 ──
    ax_left.cla()
    ax_left.set_xlim(0, CAM_W); ax_left.set_ylim(CAM_H, 0)
    ax_left.set_xlabel("x (px)"); ax_left.set_ylabel("y (px)")
    ax_left.grid(True, alpha=0.15); ax_left.set_aspect("equal")
    ax_left.set_title("Virtual Coordinate View")

    # 画面中心十字 (淡灰色)
    ax_left.axhline(y=CAM_H/2, color='gray', alpha=0.3, linewidth=0.5, linestyle='--')
    ax_left.axvline(x=CAM_W/2, color='gray', alpha=0.3, linewidth=0.5, linestyle='--')

    # 机体坐标轴 (红色, 随航向旋转)
    heading_rad = math.radians(latest_heading)
    cx, cy = CAM_W/2, CAM_H/2
    arrow_len = 40
    # X 轴 (机头方向)
    dx = -math.cos(heading_rad) * arrow_len
    dy = math.sin(heading_rad) * arrow_len
    ax_left.annotate('', xy=(cx+dx, cy+dy), xytext=(cx, cy),
                     arrowprops=dict(arrowstyle='->', color='red', lw=2.5),
                     zorder=15)
    ax_left.text(cx+dx*1.2, cy+dy*1.2, 'X', color='red', fontsize=11,
                 fontweight='bold', ha='center', va='center', zorder=15)
    # Y 轴 (机体右向)
    dx = math.sin(heading_rad) * arrow_len
    dy = -math.cos(heading_rad) * arrow_len
    ax_left.annotate('', xy=(cx+dx, cy+dy), xytext=(cx, cy),
                     arrowprops=dict(arrowstyle='->', color='red', lw=2.5),
                     zorder=15)
    ax_left.text(cx+dx*1.2, cy+dy*1.2, 'Y', color='red', fontsize=11,
                 fontweight='bold', ha='center', va='center', zorder=15)

    v_visible = bool(cars) and h_age < TIMEOUT and cars[-1]["angle"] > 0
    if v_visible:
        latest = cars[-1]
        # 完整 V 形三角形 (三个顶点)
        pts = [[latest["b1x"],latest["b1y"]],[latest["b2x"],latest["b2y"]],[latest["vx"],latest["vy"]]]
        ax_left.add_patch(Polygon(pts, closed=True, fill=True, alpha=0.15, color="green", edgecolor="green", linewidth=1.5))
        ax_left.scatter([latest["vx"]],[latest["vy"]],c="red",s=70,marker="^",zorder=5,label="vertex")
        ax_left.scatter([latest["b1x"]],[latest["b1y"]],c="blue",s=50,marker="s",zorder=5,label="base1")
        ax_left.scatter([latest["b2x"]],[latest["b2y"]],c="orange",s=50,marker="s",zorder=5,label="base2")
        # 质心轨迹
        cx = (latest["b1x"]+latest["b2x"]+latest["vx"])//3
        cy = (latest["b1y"]+latest["b2y"]+latest["vy"])//3
        ax_left.scatter([cx],[cy],c="red",s=40,marker="o",zorder=6)
        if len(cars) > 1:
            hx = []; hy = []
            for m in cars[-40:]:
                hx.append((m["b1x"]+m["b2x"]+m["vx"])//3)
                hy.append((m["b1y"]+m["b2y"]+m["vy"])//3)
            ax_left.plot(hx, hy, "r-", alpha=0.4, linewidth=1.0)

    b_visible = (beac is not None) and b_age < TIMEOUT
    if b_visible:
        cx, cy, score = beac
        ax_left.scatter([cx],[cy],c="cyan",s=100,marker="o",edgecolors="blue",linewidth=2,zorder=4,label="beacon")
        ax_left.axvline(x=cx,color="cyan",alpha=0.2,linewidth=0.5)
        ax_left.axhline(y=cy,color="cyan",alpha=0.2,linewidth=0.5)
        if len(beacon_hx) > 1:
            ax_left.plot(list(beacon_hx),list(beacon_hy),"c-",alpha=0.3,linewidth=0.8)

    fps = 0
    if len(ts) >= 5:
        dt = ts[-1]-ts[0]
        if dt > 0.3: fps = (len(ts)-1)/dt
    lines = [f"Frame: {n_frames}  |  FPS: {fps:.1f}"]
    if v_visible:
        lines.append(f"V: angle={latest['angle']} deg  pos=({cx},{cy})")
    # 非视觉数据常显示 (使用最近的有效值)
    if cars:
        d = cars[-1]
        lines.append(f"   height={d['height']:.2f}m  heading={d['heading']:.0f}deg")
        vy = d.get('world_vy', 0)
        lines.append(f"   vx: {d['speed']:.2f}  vy: {vy:.2f} m/s  roll={d['roll']:.1f}deg  pitch={d['pitch']:.1f}deg")
        if 'vel_tgt_x' in d:
            lines.append(f"   Vtgt: {d['vel_tgt_x']:.2f},{d['vel_tgt_y']:.2f} m/s")
        if 'ff_vel_x' in d:
            lines.append(f"   FF: {d['ff_vel_x']:.3f},{d['ff_vel_y']:.3f} m/s")
    if b_visible:
        lines.append(f"Beacon: ({beac[0]},{beac[1]})  score={beac[2]}")
    ax_left.text(CAM_W-5,5,"\n".join(lines),fontsize=10,color="white",
                 ha="right",va="top",family="monospace",
                 bbox=dict(facecolor="black",alpha=0.6,boxstyle="round,pad=0.3"))

    hh, ll = ax_left.get_legend_handles_labels()
    seen = set(); unique = []
    for h, l in zip(hh, ll):
        if l not in seen: seen.add(l); unique.append((h,l))
    if unique:
        ax_left.legend([u[0] for u in unique],[u[1] for u in unique],fontsize=8,loc="lower right")

    # ── 右图: 摄像头 (用 set_data 更新, 避免 imshow 重建) ──
    for artist in ax_right.patches + ax_right.lines + ax_right.collections:
        artist.remove()

    if frame is not None:
        cam_img.set_data(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    else:
        cam_img.set_data(np.zeros((ch, cw, 3), dtype=np.uint8))

    # 在摄像头画面上叠加识别标记 (缩放到摄像头分辨率)
    sx, sy = cw / CAM_W, ch / CAM_H
    if v_visible:
        cx = int((latest["b1x"]+latest["b2x"]+latest["vx"])//3 * sx)
        cy = int((latest["b1y"]+latest["b2y"]+latest["vy"])//3 * sy)
        ax_right.scatter([cx],[cy],c="green",s=60,marker="o",zorder=6)
        # 画三角形
        tri_x = [int(latest["b1x"]*sx), int(latest["b2x"]*sx), int(latest["vx"]*sx)]
        tri_y = [int(latest["b1y"]*sy), int(latest["b2y"]*sy), int(latest["vy"]*sy)]
        ax_right.add_patch(Polygon(list(zip(tri_x, tri_y)), closed=True, fill=True, alpha=0.12, color="green", edgecolor="green", linewidth=1.0))

    if b_visible:
        cx, cy, score = beac
        ax_right.scatter([int(cx*sx)],[int(cy*sy)],c="cyan",s=80,marker="o",edgecolors="blue",linewidth=2,zorder=4)
        ax_right.axvline(x=int(cx*sx),color="cyan",alpha=0.2,linewidth=0.5)
        ax_right.axhline(y=int(cy*sy),color="cyan",alpha=0.2,linewidth=0.5)

    # 摄像头画面中心十字
    ax_right.axhline(y=ch/2, color='white', alpha=0.4, linewidth=0.5, linestyle='--')
    ax_right.axvline(x=cw/2, color='white', alpha=0.4, linewidth=0.5, linestyle='--')

    fig.tight_layout()
    fig.canvas.draw_idle()


# ── 启动 ──
reader_thread = threading.Thread(target=serial_reader, daemon=True)
reader_thread.start()

if HAVE_CV2 and args.camera is not None:
    cam_thread = threading.Thread(target=camera_thread_func, daemon=True)
    cam_thread.start()

timer = fig.canvas.new_timer(interval=50)
timer.add_callback(update_plot)
timer.start()

print("按窗口关闭按钮退出...")
plt.show()
running = False
print("已退出")
