#!/usr/bin/env python3
"""
实时视觉识别上位机 (精简版)
============================
通过串口读取 V 形车标和信标数据，实时绘制在相机视图上。

依赖:
    pip install pyserial matplotlib numpy

用法:
    python realtime_monitor.py COM25 115200
"""

import sys, threading, time, argparse
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

# ── 参数 ──
parser = argparse.ArgumentParser(description="实时视觉识别上位机 (精简版)")
parser.add_argument("port", help="串口号 (如 COM25)")
parser.add_argument("baud", type=int, nargs="?", default=115200, help="波特率 (默认 115200)")
args = parser.parse_args()

# ── 数据 ──
MAX_HISTORY = 60
car_data   = deque(maxlen=MAX_HISTORY)  # 每帧 V 形 {frame_id,area,angle,arm1,arm2,base_len,
                                        #   b1x,b1y,b2x,b2y,mx,my,vx,vy}
beacon_xy  = None                       # 最新信标 (cx, cy, score)
beacon_hx  = deque(maxlen=MAX_HISTORY)  # 信标轨迹 x
beacon_hy  = deque(maxlen=MAX_HISTORY)  # 信标轨迹 y
timestamps = deque(maxlen=MAX_HISTORY)
frame_cnt  = 0
last_h_time = 0.0   # 最近一次 V 形数据到达时间
last_b_time = 0.0   # 最近一次信标数据到达时间
TIMEOUT = 0.5       # 超过此秒数无数据视为丢失
lock = threading.Lock()
running = True

# ── 串口读取 ──
def serial_reader():
    global frame_cnt
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
            if not data:
                continue
            buf += data
            while b"\r\n" in buf or b"\n" in buf:
                if b"\r\n" in buf:
                    line, buf = buf.split(b"\r\n", 1)
                else:
                    line, buf = buf.split(b"\n", 1)
                text = line.decode("utf-8", errors="replace").strip()
                if not text:
                    continue
                parse_line(text)
        except serial.SerialException:
            break
        except Exception:
            continue
    ser.close()
    print("串口已断开")

def parse_line(text: str):
    global frame_cnt, beacon_xy, last_h_time, last_b_time
    parts = text.split(",")
    if not parts:
        return

    if text.startswith("H,") and len(parts) >= 15:
        # H,frame_id,area,angle,arm1,arm2,base_len,b1x,b1y,b2x,b2y,mx,my,vx,vy
        try:
            d = {
                "frame_id": int(parts[1]),
                "area":     int(parts[2]),
                "angle":    int(parts[3]),  # > 0 = V-shape found
                "arm1":     int(parts[4]),
                "arm2":     int(parts[5]),
                "base_len": int(parts[6]),
                "b1x":      int(parts[7]),
                "b1y":      int(parts[8]),
                "b2x":      int(parts[9]),
                "b2y":      int(parts[10]),
                "mx":       int(parts[11]),
                "my":       int(parts[12]),
                "vx":       int(parts[13]),
                "vy":       int(parts[14]),
            }
            with lock:
                car_data.append(d)
                timestamps.append(time.time())
                last_h_time = time.time()
                frame_cnt += 1
        except (ValueError, IndexError):
            pass

    elif text.startswith("B,") and len(parts) >= 5:
        # B,frame_id,cx,cy,score
        try:
            with lock:
                beacon_xy = (int(parts[2]), int(parts[3]), int(parts[4]))
                beacon_hx.append(int(parts[2]))
                beacon_hy.append(int(parts[3]))
                last_b_time = time.time()
        except (ValueError, IndexError):
            pass


# ── 绘图 ──
fig, ax = plt.subplots(figsize=(8, 7))
fig.suptitle(f"Real-time Vision Monitor - {args.port} @ {args.baud}")

CAM_W, CAM_H = 320, 240
ax.set_xlim(0, CAM_W)
ax.set_ylim(CAM_H, 0)
ax.set_xlabel("x (px)")
ax.set_ylabel("y (px)")
ax.grid(True, alpha=0.15)
ax.set_aspect("equal")

def update_plot():
    with lock:
        cars = list(car_data)
        beac = beacon_xy
        ts   = list(timestamps)
        n_frames = frame_cnt
        h_age = time.time() - last_h_time
        b_age = time.time() - last_b_time

    ax.cla()
    ax.set_xlim(0, CAM_W)
    ax.set_ylim(CAM_H, 0)
    ax.set_xlabel("x (px)")
    ax.set_ylabel("y (px)")
    ax.grid(True, alpha=0.15)
    ax.set_aspect("equal")

    # ── V 形 (0.5 秒无数据则隐藏) ──
    v_visible = bool(cars) and h_age < TIMEOUT
    if v_visible:
        latest = cars[-1]
        if latest["angle"] > 0:
            pts = [[latest["b1x"], latest["b1y"]],
                   [latest["b2x"], latest["b2y"]],
                   [latest["vx"],  latest["vy"]]]
            tri = Polygon(pts, closed=True, fill=True, alpha=0.15, color="green",
                          edgecolor="green", linewidth=1.5)
            ax.add_patch(tri)

            ax.scatter([latest["vx"]], [latest["vy"]], c="red",  s=70, marker="^", zorder=5, label="vertex")
            ax.scatter([latest["b1x"]],[latest["b1y"]],c="blue", s=50, marker="s", zorder=5, label="base1")
            ax.scatter([latest["b2x"]],[latest["b2y"]],c="orange",s=50,marker="s", zorder=5, label="base2")
            ax.scatter([latest["mx"]], [latest["my"]], c="red", s=40, marker="o", zorder=6)

            if len(cars) > 1:
                hx = [m["mx"] for m in cars[-40:]]
                hy = [m["my"] for m in cars[-40:]]
                ax.plot(hx, hy, "r-", alpha=0.4, linewidth=1.0)

    # ── 信标 (0.5 秒无数据则隐藏) ──
    b_visible = (beac is not None) and b_age < TIMEOUT
    if b_visible:
        cx, cy, score = beac
        ax.scatter([cx], [cy], c="cyan", s=100, marker="o",
                   edgecolors="blue", linewidth=2, zorder=4, label="beacon")
        ax.axvline(x=cx, color="cyan", alpha=0.2, linewidth=0.5)
        ax.axhline(y=cy, color="cyan", alpha=0.2, linewidth=0.5)
        if len(beacon_hx) > 1:
            ax.plot(list(beacon_hx), list(beacon_hy), "c-", alpha=0.3, linewidth=0.8)

    # ── 信息文本 ──
    fps = 0
    if len(ts) >= 5:
        dt = ts[-1] - ts[0]
        if dt > 0.3:
            fps = (len(ts)-1) / dt

    lines = [f"Frame: {n_frames}  |  FPS: {fps:.1f}"]
    if v_visible:
        lines.append(f"V: angle={latest['angle']} deg  area={latest['area']}")
        lines.append(f"   arm1={latest['arm1']}  arm2={latest['arm2']}  base={latest['base_len']}")
        lines.append(f"   vertex=({latest['vx']},{latest['vy']})")
    if b_visible:
        lines.append(f"Beacon: ({beac[0]},{beac[1]})  score={beac[2]}")

    # 移动 info_text 到新 cla() 后的位置
    ax.text(CAM_W - 5, 5, "\n".join(lines), fontsize=10, color="white",
            ha="right", va="top", family="monospace",
            bbox=dict(facecolor="black", alpha=0.6, boxstyle="round,pad=0.3"))

    ax.legend(fontsize=8, loc="lower right")
    fig.canvas.draw_idle()


# ── 启动 ──
reader_thread = threading.Thread(target=serial_reader, daemon=True)
reader_thread.start()

timer = fig.canvas.new_timer(interval=100)
timer.add_callback(update_plot)
timer.start()

print("按窗口关闭按钮退出...")
plt.show()
running = False
print("已退出")
