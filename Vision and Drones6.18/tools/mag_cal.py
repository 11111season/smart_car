"""
磁力计零偏标定工具

使用方法：
  1. 飞控通过串口打印 mag:x,y,z 格式数据（已配置）
  2. 拿起板子绕 X/Y/Z 三个轴各旋转 360°，覆盖所有方向
  3. 运行本脚本，选择串口号和波特率
  4. 采集完成后按 Ctrl+C 停止
  5. 脚本自动计算零偏并输出 C 代码

数据格式: mag:X,Y,Z 或 X,Y,Z  (换行分隔)
"""

import serial
import serial.tools.list_ports
import sys
import time

def list_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("错误: 未检测到串口设备")
        sys.exit(1)
    print("可用串口:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device} - {p.description}")
    try:
        idx = int(input("请选择串口编号: "))
        return ports[idx].device
    except (ValueError, IndexError):
        print("输入无效")
        sys.exit(1)

def main():
    # 选择串口
    port = list_ports()
    baud = int(input("请输入波特率(默认 115200): ") or "115200")

    # 打开串口
    ser = serial.Serial(port, baud, timeout=1)
    print(f"\n已连接 {port} @ {baud} baud")
    print("请旋转板子覆盖所有方向，完成后按 Ctrl+C 停止采集...\n")
    print("采集进度: ", end="", flush=True)

    mag_x, mag_y, mag_z = [], [], []
    total = 0
    start = time.time()

    try:
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            # 解析 "mag:x,y,z" 格式 或 "x,y,z" 格式
            data_str = line
            if line.startswith("mag:"):
                data_str = line[4:]

            parts = data_str.split(",")
            if len(parts) == 3:
                try:
                    x = int(parts[0].strip())
                    y = int(parts[1].strip())
                    z = int(parts[2].strip())
                except ValueError:
                    continue

                mag_x.append(x)
                mag_y.append(y)
                mag_z.append(z)
                total += 1
                if total % 100 == 0:
                    print("#", end="", flush=True)

    except KeyboardInterrupt:
        print("\n\n采集结束")

    ser.close()

    if total < 100:
        print(f"采集数据太少({total}组)，请多旋转一会")
        return

    # 统计
    x_min, x_max = min(mag_x), max(mag_x)
    y_min, y_max = min(mag_y), max(mag_y)
    z_min, z_max = min(mag_z), max(mag_z)

    off_x = (x_min + x_max) / 2
    off_y = (y_min + y_max) / 2
    off_z = (z_min + z_max) / 2

    elapsed = time.time() - start
    print(f"采集时长: {elapsed:.1f}s")
    print(f"有效数据: {total} 组")
    print()

    # 输出统计结果
    print("=" * 50)
    print("各轴统计:")
    print(f"  X: min={x_min:6d}, max={x_max:6d}, range={x_max-x_min:6d}")
    print(f"  Y: min={y_min:6d}, max={y_max:6d}, range={y_max-y_min:6d}")
    print(f"  Z: min={z_min:6d}, max={z_max:6d}, range={z_max-z_min:6d}")
    print()

    # 数据质量检查
    ranges_ok = all([
        x_max - x_min > 50,
        y_max - y_min > 50,
        z_max - z_min > 50
    ])
    if not ranges_ok:
        print("⚠ 警告: 某个轴数据范围过小(<50)，可能旋转不充分")
        print()

    print("零偏结果 (可直接填入 INIT.c):")
    print(f"  offset[0] = {off_x:.1f}f;  // X轴零偏")
    print(f"  offset[1] = {off_y:.1f}f;  // Y轴零偏")
    print(f"  offset[2] = {off_z:.1f}f;  // Z轴零偏")
    print()
    print("对应的 C 代码:")
    print(f'''
MagCalibration_t mag_cal = {{
    .offset = {{{off_x:.1f}f, {off_y:.1f}f, {off_z:.1f}f}},
    .scale  = {{
        {{1.0f, 0.0f, 0.0f}},
        {{0.0f, 1.0f, 0.0f}},
        {{0.0f, 0.0f, 1.0f}}
    }}
}};
''')
    print("=" * 50)


if __name__ == "__main__":
    main()
