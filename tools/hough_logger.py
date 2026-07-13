#!/usr/bin/env python3
"""
霍夫变换特征录制与统计工具
=============================
通过串口读取无人机视觉调试数据（DEBUG_HOUGH=1 时打印的 H, 行），
录制期间实时显示统计，Ctrl+C 停止后输出完整摘要。

用法:
    python hough_logger.py COM3              # 默认 115200 波特率
    python hough_logger.py COM3 921600       # 指定波特率
    python hough_logger.py COM3 115200 data.csv  # 指定输出文件

输出格式 (每行):
    H,frame_id,area,cx,cy,w,h,hough_cnt,thresh,angles,v_found,angle,height,arm1,arm2,base,line0_rho,line0_theta,line0_votes,...

依赖: pip install pyserial
"""

import sys
import csv
import time
import signal
import os
from datetime import datetime
from collections import Counter, defaultdict

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("请先安装 pyserial: pip install pyserial")
    sys.exit(1)


class HoughLogger:
    def __init__(self, port: str, baudrate: int = 115200, output_file: str = None):
        self.port = port
        self.baudrate = baudrate
        self.output_file = output_file or f"hough_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.ser = None
        self.running = True

        # 统计数据
        self.total_frames = 0
        self.total_blobs = 0
        self.v_found_count = 0
        self.hough_cnt_dist = Counter()       # hough_cnt 分布
        self.v_angles = []                    # 找到 V 形时的角度列表
        self.v_heights = []                   # 找到 V 形时的高度列表
        self.v_arm1s = []                     # 臂长1
        self.v_arm2s = []                     # 臂长2
        self.v_bases = []                     # 底边长
        self.theta_counter = Counter()        # θ 角度出现次数
        self.vote_sum = 0                     # 总票数
        self.vote_count = 0                   # 直线数
        self.fail_no_lines = 0                # hough_cnt < 2 的 blob 数
        self.fail_no_pair = 0                 # hough_cnt >= 2 但配对失败的 blob 数
        self.start_time = None

        # 实时递减计数
        self.last_print_time = 0
        self.recent_v = 0
        self.recent_total = 0

    def open(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.5)
            print(f"[串口] 已连接 {self.port} @ {self.baudrate} bps")
            time.sleep(0.5)
            self.ser.reset_input_buffer()
        except serial.SerialException as e:
            print(f"[错误] 无法打开串口 {self.port}: {e}")
            print("可用串口:")
            for p in serial.tools.list_ports.comports():
                print(f"  {p.device} - {p.description}")
            sys.exit(1)

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"\n[串口] 已关闭 {self.port}")

    def parse_line(self, line: str):
        """解析 H,frame_id,area,cx,cy,... 行"""
        parts = line.strip().split(',')
        if len(parts) < 16 or parts[0] != 'H':
            return None

        try:
            data = {
                'frame_id': int(parts[1]),
                'area': int(parts[2]),
                'cx': int(parts[3]),
                'cy': int(parts[4]),
                'w': int(parts[5]),
                'h': int(parts[6]),
                'hough_cnt': int(parts[7]),
                'thresh': int(parts[8]),
                'angles': int(parts[9]),
                'v_found': int(parts[10]),
                'angle': int(parts[11]),
                'height': int(parts[12]),
                'arm1': int(parts[13]),
                'arm2': int(parts[14]),
                'base': int(parts[15]),
                'lines': []
            }
            # 解析直线数据: 每3个字段一组 (rho, theta, votes)
            i = 16
            while i + 2 < len(parts):
                data['lines'].append({
                    'rho': int(parts[i]),
                    'theta': int(parts[i + 1]),
                    'votes': int(parts[i + 2])
                })
                i += 3
            return data
        except (ValueError, IndexError):
            return None

    def update_stats(self, data: dict):
        """更新统计数据"""
        self.total_blobs += 1
        self.total_frames = max(self.total_frames, data['frame_id'])

        self.hough_cnt_dist[data['hough_cnt']] += 1

        if data['v_found']:
            self.v_found_count += 1
            self.v_angles.append(data['angle'])
            self.v_heights.append(data['height'])
            self.v_arm1s.append(data['arm1'])
            self.v_arm2s.append(data['arm2'])
            self.v_bases.append(data['base'])
            self.recent_v += 1
        else:
            if data['hough_cnt'] < 2:
                self.fail_no_lines += 1
            else:
                self.fail_no_pair += 1

        self.recent_total += 1

        for line in data['lines']:
            self.theta_counter[line['theta']] += 1
            self.vote_sum += line['votes']
            self.vote_count += 1

    def print_realtime(self):
        """每 2 秒打印实时统计"""
        now = time.time()
        if now - self.last_print_time >= 2.0 and self.recent_total > 0:
            rate = self.recent_v / self.recent_total * 100
            elapsed = now - self.start_time if self.start_time else 0
            print(f"  [实时] {elapsed:5.0f}s | 总blob={self.total_blobs:5d} | "
                  f"V检出率={rate:5.1f}% ({self.recent_v}/{self.recent_total})")
            self.last_print_time = now
            self.recent_v = 0
            self.recent_total = 0

    def print_summary(self):
        """打印统计摘要"""
        elapsed = time.time() - self.start_time if self.start_time else 0
        print("\n" + "=" * 60)
        print("                    霍夫变换特征统计摘要")
        print("=" * 60)
        print(f"  录制时长        : {elapsed:.1f} 秒")
        print(f"  总帧数          : {self.total_frames}")
        print(f"  总 blob 数      : {self.total_blobs}")
        print()

        # V 形检测率
        if self.total_blobs > 0:
            rate = self.v_found_count / self.total_blobs * 100
            print(f"  V 形检测率      : {rate:.1f}% ({self.v_found_count}/{self.total_blobs})")
        print()

        # 霍夫直线数分布
        print("  ── 霍夫直线数分布 ──")
        for cnt in sorted(self.hough_cnt_dist.keys()):
            n = self.hough_cnt_dist[cnt]
            label = f"5+" if cnt >= 5 else str(cnt)
            pct = n / self.total_blobs * 100 if self.total_blobs > 0 else 0
            bar = '█' * int(pct / 2)
            print(f"    hough_cnt={label:>3s}: {n:5d} ({pct:5.1f}%) {bar}")
        print()

        # 找到 V 形时的几何统计
        if self.v_found_count > 0:
            print("  ── 找到 V 形时的几何特征 ──")
            print(f"    数量            : {self.v_found_count}")
            print(f"    角度 (deg)      : 均值={self._avg(self.v_angles):.1f}  "
                  f"范围=[{min(self.v_angles)},{max(self.v_angles)}]")
            print(f"    高度 (px)       : 均值={self._avg(self.v_heights):.1f}  "
                  f"范围=[{min(self.v_heights)},{max(self.v_heights)}]")
            print(f"    臂长1 (px)      : 均值={self._avg(self.v_arm1s):.1f}  "
                  f"范围=[{min(self.v_arm1s)},{max(self.v_arm1s)}]")
            print(f"    臂长2 (px)      : 均值={self._avg(self.v_arm2s):.1f}  "
                  f"范围=[{min(self.v_arm2s)},{max(self.v_arm2s)}]")
            print(f"    底边 (px)       : 均值={self._avg(self.v_bases):.1f}  "
                  f"范围=[{min(self.v_bases)},{max(self.v_bases)}]")
            print()

            # 角度分布直方图 (10 度一档)
            print("  ── 角度分布直方图 (10° 一档) ──")
            angle_bins = defaultdict(int)
            for a in self.v_angles:
                bin_key = (a // 10) * 10
                angle_bins[bin_key] += 1
            for bin_key in sorted(angle_bins.keys()):
                n = angle_bins[bin_key]
                pct = n / self.v_found_count * 100
                bar = '█' * int(pct / 2)
                print(f"    {bin_key:3d}-{bin_key + 9:3d}°: {n:5d} ({pct:5.1f}%) {bar}")
            print()

        # 未找到 V 形的原因分布
        fail_total = self.fail_no_lines + self.fail_no_pair
        if fail_total > 0:
            print("  ── 未找到 V 形的原因 ──")
            print(f"    hough_cnt < 2  (直线不足) : {self.fail_no_lines:5d} "
                  f"({self.fail_no_lines / fail_total * 100:.1f}%)")
            print(f"    hough_cnt >= 2 (配对失败) : {self.fail_no_pair:5d} "
                  f"({self.fail_no_pair / fail_total * 100:.1f}%)")
            print()

        # 投票统计
        if self.vote_count > 0:
            avg_vote = self.vote_sum / self.vote_count
            print(f"  ── 投票统计 ──")
            print(f"    总直线数        : {self.vote_count}")
            print(f"    平均 votes/线   : {avg_vote:.1f}")
            print()

        # Top 10 θ 角度
        if self.theta_counter:
            print("  ── 最常见 θ 角度 Top 10 ──")
            for theta, count in self.theta_counter.most_common(10):
                pct = count / self.vote_count * 100 if self.vote_count > 0 else 0
                print(f"    θ={theta:4d}° : {count:5d} 次 ({pct:5.1f}%)")
            print()

        print("=" * 60)
        print(f"  原始数据已保存至: {self.output_file}")
        print("=" * 60)

    @staticmethod
    def _avg(lst):
        return sum(lst) / len(lst) if lst else 0

    def run(self):
        self.open()
        self.start_time = time.time()
        self.last_print_time = self.start_time

        print(f"[录制] 开始记录霍夫数据到 {self.output_file}")
        print(f"[提示] 按 Ctrl+C 停止录制并查看统计摘要\n")

        # 注册信号处理 (Ctrl+C)
        signal.signal(signal.SIGINT, lambda s, f: setattr(self, 'running', False))

        with open(self.output_file, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow([
                'frame_id', 'area', 'cx', 'cy', 'w', 'h',
                'hough_cnt', 'thresh', 'angles', 'v_found',
                'angle', 'height', 'arm1', 'arm2', 'base',
                'lines_data'
            ])

            while self.running:
                try:
                    line = self.ser.readline()
                    if not line:
                        self.print_realtime()
                        continue

                    text = line.decode('utf-8', errors='replace').strip()
                    if not text.startswith('H,'):
                        continue

                    data = self.parse_line(text)
                    if data is None:
                        continue

                    # 写入 CSV
                    lines_str = ';'.join(
                        f"{l['rho']},{l['theta']},{l['votes']}"
                        for l in data['lines']
                    )
                    writer.writerow([
                        data['frame_id'], data['area'], data['cx'], data['cy'],
                        data['w'], data['h'], data['hough_cnt'], data['thresh'],
                        data['angles'], data['v_found'], data['angle'],
                        data['height'], data['arm1'], data['arm2'], data['base'],
                        lines_str
                    ])
                    f.flush()

                    self.update_stats(data)
                    self.print_realtime()

                except serial.SerialException as e:
                    print(f"\n[错误] 串口异常: {e}")
                    break
                except Exception as e:
                    print(f"\n[错误] {e}")
                    continue

        self.close()
        self.print_summary()


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        print("用法: python hough_logger.py <端口> [波特率] [输出文件]")
        print("示例: python hough_logger.py COM3")
        print("      python hough_logger.py COM3 921600")
        print("      python hough_logger.py COM3 115200 my_log.csv")
        sys.exit(1)

    port = sys.argv[1]
    baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    output_file = sys.argv[3] if len(sys.argv) > 3 else None

    logger = HoughLogger(port, baudrate, output_file)
    logger.run()


if __name__ == '__main__':
    main()