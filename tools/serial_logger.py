#!/usr/bin/env python3
"""
串口数据录制工具
=================
将串口原始数据保存到 CSV 文件，带时间戳。

用法:
    python serial_logger.py COM6 115200
    python serial_logger.py COM6 115200 output.csv   # 指定文件名
"""

import sys, time, signal
from datetime import datetime

try:
    import serial
except ImportError:
    print("请先安装 pyserial: pip install pyserial")
    sys.exit(1)

port = sys.argv[1]
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
out_file = sys.argv[3] if len(sys.argv) > 3 else f"serial_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

running = True

def handler(sig, frame):
    global running
    running = False

signal.signal(signal.SIGINT, handler)

try:
    ser = serial.Serial(port, baud, timeout=0.1)
except Exception as e:
    print(f"无法打开 {port}: {e}")
    sys.exit(1)

print(f"录制中... 保存到 {out_file}", flush=True)
print("按 Ctrl+C 停止", flush=True)

with open(out_file, "w", encoding="utf-8") as f:
    f.write("time,data\n")
    buf = b""
    while running:
        try:
            data = ser.read(1024)
            if data:
                buf += data
                while b"\n" in buf:
                    if b"\r\n" in buf:
                        line, buf = buf.split(b"\r\n", 1)
                    else:
                        line, buf = buf.split(b"\n", 1)
                    text = line.decode("utf-8", errors="replace").strip()
                    if text:
                        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                        f.write(f"{ts},{text}\n")
                        f.flush()
                        print(text, flush=True)
        except serial.SerialException:
            break

ser.close()
print(f"\n已保存 {out_file}")
