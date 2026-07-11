"""
磁力计校准效果评估工具

使用方法：
  1. 飞控已打印 mag_raw:x,y,z | cal:x,y,z 格式数据
  2. 拿起板子旋转，覆盖各方向
  3. 运行本脚本，选择串口号和波特率
  4. 采集足够数据后按 Ctrl+C 停止
  5. 脚本自动评估校准效果

数据格式: mag_raw:fx,fy,fz | cal:fx,fy,fz
"""

import serial
import serial.tools.list_ports
import sys
import time
import math
from collections import deque

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

def parse_line(line):
    """解析 mag_raw:x,y,z | cal:x,y,z 格式"""
    if "mag_raw:" not in line or "cal:" not in line:
        return None

    try:
        parts = line.split("|")
        raw_part = parts[0].strip()  # "mag_raw:x,y,z"
        cal_part = parts[1].strip()  # "cal:x,y,z"

        raw_vals = raw_part.replace("mag_raw:", "").split(",")
        cal_vals = cal_part.replace("cal:", "").split(",")

        raw = [float(v.strip()) for v in raw_vals]
        cal = [float(v.strip()) for v in cal_vals]

        if len(raw) == 3 and len(cal) == 3:
            return raw, cal
    except (ValueError, IndexError):
        pass
    return None

def evaluate(raw_data, cal_data):
    """评估校准效果"""
    n = len(raw_data)
    if n < 10:
        print("数据太少，无法评估")
        return

    # ---- 原始数据统计 ----
    raw_x = [d[0] for d in raw_data]
    raw_y = [d[1] for d in raw_data]
    raw_z = [d[2] for d in raw_data]

    rx_min, rx_max = min(raw_x), max(raw_x)
    ry_min, ry_max = min(raw_y), max(raw_y)
    rz_min, rz_max = min(raw_z), max(raw_z)

    rx_range = rx_max - rx_min
    ry_range = ry_max - ry_min
    rz_range = rz_max - rz_min

    rx_avg = (rx_min + rx_max) / 2
    ry_avg = (ry_min + ry_max) / 2
    rz_avg = (rz_min + rz_max) / 2

    # ---- 校准后数据统计 ----
    cal_x = [d[0] for d in cal_data]
    cal_y = [d[1] for d in cal_data]
    cal_z = [d[2] for d in cal_data]

    cx_min, cx_max = min(cal_x), max(cal_x)
    cy_min, cy_max = min(cal_y), max(cal_y)
    cz_min, cz_max = min(cal_z), max(cal_z)

    cx_range = cx_max - cx_min
    cy_range = cy_max - cy_min
    cz_range = cz_max - cz_min

    cx_avg = (cx_min + cx_max) / 2
    cy_avg = (cy_min + cy_max) / 2
    cz_avg = (cz_min + cz_max) / 2

    # ---- 模长分析 ----
    norms = [math.sqrt(d[0]**2 + d[1]**2 + d[2]**2) for d in cal_data]
    norm_mean = sum(norms) / len(norms)
    norm_std = math.sqrt(sum((n - norm_mean)**2 for n in norms) / len(norms))
    norm_min = min(norms)
    norm_max = max(norms)
    norm_dev = max(abs(norm_min - 1.0), abs(norm_max - 1.0))

    # ---- 球形度（各轴范围一致性） ----
    ranges = [cx_range, cy_range, cz_range]
    range_mean = sum(ranges) / 3
    range_dev = max(abs(r / range_mean - 1.0) for r in ranges) if range_mean > 0 else 0

    # ============ 输出报告 ============
    print("\n" + "=" * 56)
    print("  磁力计校准评估报告")
    print("=" * 56)

    print(f"\n  采集数据: {n} 组")

    print(f"\n  ┌─ 原始数据（高斯） ──────────────────────────┐")
    print(f"  │   X: {rx_min:8.3f} ~ {rx_max:8.3f}  range={rx_range:6.2f}  offset={rx_avg:+.3f} │")
    print(f"  │   Y: {ry_min:8.3f} ~ {ry_max:8.3f}  range={ry_range:6.2f}  offset={ry_avg:+.3f} │")
    print(f"  │   Z: {rz_min:8.3f} ~ {rz_max:8.3f}  range={rz_range:6.2f}  offset={rz_avg:+.3f} │")
    print(f"  └──────────────────────────────────────────────┘")

    print(f"\n  ┌─ 校准后数据（高斯） ────────────────────────┐")
    print(f"  │   X: {cx_min:8.3f} ~ {cx_max:8.3f}  range={cx_range:6.2f}  offset={cx_avg:+.3f} │")
    print(f"  │   Y: {cy_min:8.3f} ~ {cy_max:8.3f}  range={cy_range:6.2f}  offset={cy_avg:+.3f} │")
    print(f"  │   Z: {cz_min:8.3f} ~ {cz_max:8.3f}  range={cz_range:6.2f}  offset={cz_avg:+.3f} │")
    print(f"  └──────────────────────────────────────────────┘")

    print(f"\n  ┌─ 模长分析 ──────────────────────────────────┐")
    print(f"  │  均值: {norm_mean:.3f}  标准差: {norm_std:.4f}            │")
    print(f"  │  最小: {norm_min:.3f}  最大: {norm_max:.3f}            │")
    print(f"  │  偏离 1.0: {norm_dev:.3f}                          │")
    print(f"  └──────────────────────────────────────────────┘")

    # ---- 评估结论 ----
    print(f"\n  ┌─ 评估结论 ──────────────────────────────────┐")

    grades = []

    # 1. 偏移量是否接近零（理想应为 0）
    offset_mag = math.sqrt(cx_avg**2 + cy_avg**2 + cz_avg**2)
    if offset_mag < 0.05:
        grades.append(("✅ 偏移校正", "优秀", f"残余偏移 {offset_mag:.4f}"))
    elif offset_mag < 0.15:
        grades.append(("⚠ 偏移校正", "良好", f"残余偏移 {offset_mag:.4f}"))
    else:
        grades.append(("❌ 偏移校正", "需改进", f"残余偏移 {offset_mag:.4f}"))

    # 2. 模长一致性（理想模长 = 1）
    if norm_std < 0.05 and norm_dev < 0.1:
        grades.append(("✅ 模长一致", "优秀", f"标准差 {norm_std:.4f}, 最大偏离 {norm_dev:.3f}"))
    elif norm_std < 0.10 and norm_dev < 0.2:
        grades.append(("⚠ 模长一致", "良好", f"标准差 {norm_std:.4f}, 最大偏离 {norm_dev:.3f}"))
    else:
        grades.append(("❌ 模长一致", "需改进", f"标准差 {norm_std:.4f}, 最大偏离 {norm_dev:.3f}"))

    # 3. 各轴覆盖均匀性（range越接近越好）
    if range_dev < 0.15:
        grades.append(("✅ 球形度", "优秀", f"轴间差异 {range_dev*100:.1f}%"))
    elif range_dev < 0.30:
        grades.append(("⚠ 球形度", "良好", f"轴间差异 {range_dev*100:.1f}%"))
    else:
        grades.append(("❌ 球形度", "需改进", f"轴间差异 {range_dev*100:.1f}%"))

    # 4. 原始数据范围是否充分
    raw_ranges = [rx_range, ry_range, rz_range]
    min_range = min(raw_ranges)
    if min_range > 1.0:
        grades.append(("✅ 覆盖范围", "优秀", f"最小轴范围 {min_range:.2f}"))
    elif min_range > 0.5:
        grades.append(("⚠ 覆盖范围", "良好", f"最小轴范围 {min_range:.2f}"))
    else:
        grades.append(("❌ 覆盖范围", "需改进", f"最小轴范围 {min_range:.2f}"))

    for icon, label, detail in grades:
        print(f"  │ {icon} {label:<10s}  {detail:<28s} │")

    print(f"  └──────────────────────────────────────────────┘")
    print()


def recalibrate(raw_data, cal_data):
    """
    根据采集数据对现有校准做再校准（二阶校准），
    输出合并后的新校准参数可直接替换 INIT.c 的值。

    注意: raw_data 和 cal_data 都是高斯单位。
    """
    n = len(raw_data)
    if n < 100:
        print("数据太少 (< 100 组)，无法可靠再校准，请多旋转一会")
        return

    # ---- 从原始数据计算硬磁偏移（高斯单位） ----
    raw_x = [d[0] for d in raw_data]
    raw_y = [d[1] for d in raw_data]
    raw_z = [d[2] for d in raw_data]

    # 原始数据直接就是高斯值，offset = (min+max)/2
    off_x = (min(raw_x) + max(raw_x)) / 2
    off_y = (min(raw_y) + max(raw_y)) / 2
    off_z = (min(raw_z) + max(raw_z)) / 2

    # ---- 从校准后数据计算软磁缩放 ----
    cal_x = [d[0] for d in cal_data]
    cal_y = [d[1] for d in cal_data]
    cal_z = [d[2] for d in cal_data]

    cx_min, cx_max = min(cal_x), max(cal_x)
    cy_min, cy_max = min(cal_y), max(cal_y)
    cz_min, cz_max = min(cal_z), max(cal_z)

    rx = cx_max - cx_min
    ry = cy_max - cy_min
    rz = cz_max - cz_min

    # 残余偏移（校准后理想应为 0）
    res_off_x = (cx_min + cx_max) / 2
    res_off_y = (cy_min + cy_max) / 2
    res_off_z = (cz_min + cz_max) / 2

    # 软磁缩放：让各轴范围一致（椭球→球体）
    max_range = max(rx, ry, rz)
    if max_range < 1e-6:
        print("数据范围过小，无法计算缩放")
        return

    scale_xx = max_range / rx if rx > 1e-6 else 1.0
    scale_yy = max_range / ry if ry > 1e-6 else 1.0
    scale_zz = max_range / rz if rz > 1e-6 else 1.0

    # 输出报告
    print("=" * 56)
    print("  再校准结果（高斯单位，可直接填入 INIT.c）")
    print("=" * 56)

    print(f"\n  ┌─ 硬磁偏移（高斯） ──────────────────────────┐")
    print(f"  │   X offset = {off_x:+.4f}f                          │")
    print(f"  │   Y offset = {off_y:+.4f}f                          │")
    print(f"  │   Z offset = {off_z:+.4f}f                          │")
    print(f"  └──────────────────────────────────────────────┘")

    print(f"\n  ┌─ 软磁缩放矩阵 ─────────────────────────────┐")
    print(f"  │   scale[0][0] = {scale_xx:.4f}f  (X 缩放)          │")
    print(f"  │   scale[1][1] = {scale_yy:.4f}f  (Y 缩放)          │")
    print(f"  │   scale[2][2] = {scale_zz:.4f}f  (Z 缩放)          │")
    print(f"  │   非对角元素保持 0.0f                             │")
    print(f"  └──────────────────────────────────────────────┘")

    print(f"\n  ┌─ 残余分析 ──────────────────────────────────┐")
    res_mag = math.sqrt(res_off_x**2 + res_off_y**2 + res_off_z**2)
    print(f"  │  残余偏移: ({res_off_x:+.4f}, {res_off_y:+.4f}, {res_off_z:+.4f}) │")
    print(f"  │  残余偏移模长: {res_mag:.4f}  {'✅ 良好' if res_mag < 0.05 else '⚠️ 偏大'}                │")
    print(f"  │  各轴范围: X={rx:.3f}  Y={ry:.3f}  Z={rz:.3f}       │")
    note = "  校准后各轴范围越接近, 椭球越接近球体"
    print(f"  │  {note:<46s}│")
    print(f"  └──────────────────────────────────────────────┘")

    print(f"\n  可直接替换 INIT.c 的 C 代码:\n")
    print(f"  // 磁力计校准参数")
    print(f"  float offset[3] = {{")
    print(f"      {off_x:.4f}f, {off_y:.4f}f, {off_z:.4f}f")
    print(f"  }};")
    print(f"")
    print(f"  float scale[3][3] = {{")
    print(f"      {{{scale_xx:.4f}f, 0.0f, 0.0f}},")
    print(f"      {{0.0f, {scale_yy:.4f}f, 0.0f}},")
    print(f"      {{0.0f, 0.0f, {scale_zz:.4f}f}}")
    print(f"  }};")
    print(f"")
    print(f"  MagCalibration_t mag_cal = {{")
    print(f"      .offset = {{{off_x:.4f}f, {off_y:.4f}f, {off_z:.4f}f}},")
    print(f"      .scale  = {{")
    print(f"          {{{scale_xx:.4f}f, 0.0f, 0.0f}},")
    print(f"          {{0.0f, {scale_yy:.4f}f, 0.0f}},")
    print(f"          {{0.0f, 0.0f, {scale_zz:.4f}f}}")
    print(f"      }}")
    print(f"  }};")
    print(f"\n  {'=' * 48}")
    print()


def main():
    port = list_ports()
    baud = int(input("请输入波特率(默认 115200): ") or "115200")

    ser = serial.Serial(port, baud, timeout=1)
    print(f"\n已连接 {port} @ {baud} baud")
    print("旋转板子采集数据，完成后按 Ctrl+C 停止...\n")

    raw_data = []
    cal_data = []
    total = 0
    start = time.time()

    try:
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            result = parse_line(line)
            if result:
                raw_vals, cal_vals = result
                raw_data.append(raw_vals)
                cal_data.append(cal_vals)
                total += 1
                if total % 100 == 0:
                    print(f"\r  已采集 {total} 组...", end="", flush=True)

    except KeyboardInterrupt:
        print(f"\r  已采集 {total} 组，停止采集")

    ser.close()

    elapsed = time.time() - start
    print(f"  采集时长: {elapsed:.1f}s\n")

    if total < 10:
        print("数据太少 (< 10 组)，请多旋转一会再试")
        return

    evaluate(raw_data, cal_data)
    recalibrate(raw_data, cal_data)


if __name__ == "__main__":
    main()
