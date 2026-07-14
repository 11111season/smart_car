#!/usr/bin/env python3
"""
霍夫变换特征录制与统计工具
=============================
通过串口读取无人机视觉调试数据（DEBUG_HOUGH=1 时打印的 H, 行），
录制期间实时显示统计，Ctrl+C 停止后输出完整摘要。

新增功能:
  - 端点 (B, C) 坐标录制与分析
  - 帧间端点漂移量统计
  - 帧率控制适配 (DEBUG_HOUGH_DIV)

用法:
    python hough_logger.py COM3              # 默认 115200 波特率
    python hough_logger.py COM3 921600       # 指定波特率
    python hough_logger.py COM3 115200 data.csv  # 指定输出文件

输出格式 (每行):
    H,frame_id,area,cx,cy,w,h,hough_cnt,thresh,angles,v_found,hough_v,
    angle,height,arm1,arm2,base,
    bx,by,cx,cy,mid_x,mid_y,          ← 新增端点坐标
    line0_rho,line0_theta,line0_votes,...

依赖: pip install pyserial
"""

import sys
import csv
import time
import signal
import os
import math
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
        self.hough_v_count = 0           # 霍夫验证通过数
        self.hough_cnt_dist = Counter()       # hough_cnt 分布
        self.v_angles = []                    # V 形角度
        self.v_heights = []                   # V 形高度
        self.v_arm1s = []                     # 臂长1
        self.v_arm2s = []                     # 臂长2
        self.v_bases = []                     # 底边长
        self.theta_counter = Counter()        # θ 角度出现次数
        self.vote_sum = 0                     # 总票数
        self.vote_count = 0                   # 直线数
        self.fail_no_lines = 0                # hough_cnt < 2
        self.fail_no_pair = 0                 # hough_cnt >= 2 但配对失败

        # ---- 端点分析 ----
        self.endpoint_bx = []     # B 点 x 坐标
        self.endpoint_by = []     # B 点 y 坐标
        self.endpoint_cx = []     # C 点 x 坐标
        self.endpoint_cy = []     # C 点 y 坐标
        self.endpoint_mx = []     # 底边中点 x
        self.endpoint_my = []     # 底边中点 y
        self.prev_bx = None       # 上一帧 Bx (用于帧间漂移)
        self.prev_by = None
        self.prev_cx = None
        self.prev_cy = None
        self.b_drift_list = []    # B 点帧间位移列表
        self.c_drift_list = []    # C 点帧间位移列表
        self.endpoint_frame_ids = []   # 记录端点时的 frame_id

        # ---- 信标数据 ----
        self.beacon_list = []     # 信标数据: {frame_id, area, cx, cy, w, h, score, filt_score, core, height}

        # ---- HDBG 配对拒绝原因统计 ----
        self.hdbg_frames = []
        self.hdbg_sum = {k: 0 for k in [
            'total', 'angle_diff', 'parallel', 'oob', 'nbhd', 'bbox',
            'base_w', 'angle', 'height', 'arm', 'edge', 'pass']}

        # ---- DEBUG_SCORES 数据 ----
        self.scores_list = []       # 所有分数行
        self.type_switches = []     # 类型切换事件
        self.prev_types = {}        # 上一帧 track_id → type

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
        if hasattr(self, 'beacon_csv_file') and self.beacon_csv_file:
            try:
                self.beacon_csv_file.close()
            except:
                pass

    def parse_line(self, line: str):
        """
        解析 H, 行
        格式: H,frame_id,area,cx,cy,w,h,hough_cnt,thresh,angles,
             v_found,angle,height,arm1,arm2,base,
             bx,by,cx_e,cy_e,mid_x,mid_y,raw_mid_x,raw_mid_y,convex_ratio,
             rho0,theta0,votes0,rho1,theta1,...
        """
        parts = line.strip().split(',')
        if len(parts) < 30 or parts[0] != 'H':
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
                'hough_v': int(parts[11]),
                'angle': int(parts[12]),
                'height': int(parts[13]),
                'arm1': int(parts[14]),
                'arm2': int(parts[15]),
                'base': int(parts[16]),
                'bx': int(parts[17]),
                'by': int(parts[18]),
                'cx_e': int(parts[19]),   # C 端点 (与 blob cx 区分)
                'cy_e': int(parts[20]),
                'mid_x': int(parts[21]),
                'mid_y': int(parts[22]),
                'raw_mid_x': int(parts[23]),  # EMA 平滑前底边中点
                'raw_mid_y': int(parts[24]),
                'convex_ratio': int(parts[25]),  # 凸包面积比 (%)
                'dist_std_ratio': int(parts[26]), # 轮廓距离标准差比 (%)
                'conv_mean_def': int(parts[27]),  # 平均凸度缺陷 (0-100)
                'conv_max_def': int(parts[28]),  # 最大凸度缺陷 (0-100)
                'conv_gt2_pct': int(parts[29]),  # 缺陷>2px的点占比(%)
                'lines': []
            }
            # 解析直线数据: 每3个字段一组 (rho, theta, votes)
            i = 30
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

    def parse_hdbg_line(self, line: str):
        """
        解析 HDBG, 行 (霍夫配对拒绝原因统计)
        格式: HDBG,frame_id,total,angle_diff,parallel,oob,nbhd,bbox,base_w,angle,height,arm,edge,pass
        """
        parts = line.strip().split(',')
        if len(parts) < 14 or parts[0] != 'HDBG':
            return None
        try:
            return {
                'frame_id': int(parts[1]),
                'total': int(parts[2]),
                'angle_diff': int(parts[3]),
                'parallel': int(parts[4]),
                'oob': int(parts[5]),
                'nbhd': int(parts[6]),
                'bbox': int(parts[7]),
                'base_w': int(parts[8]),
                'angle': int(parts[9]),
                'height': int(parts[10]),
                'arm': int(parts[11]),
                'edge': int(parts[12]),
                'pass': int(parts[13]),
            }
        except (ValueError, IndexError):
            return None

    def parse_beacon_line(self, line: str):
        """
        解析 B, 行 (信标数据)
        格式: B,frame_id,area,cx,cy,width,height,raw_score,filt_score,core,oheight,conv_mean_def,conv_max_def,conv_gt2_pct
        """
        parts = line.strip().split(',')
        if len(parts) < 16 or parts[0] != 'B':
            return None
        try:
            return {
                'frame_id': int(parts[1]),
                'area': int(parts[2]),
                'cx': int(parts[3]),
                'cy': int(parts[4]),
                'w': int(parts[5]),
                'h': int(parts[6]),
                'raw_score': int(parts[7]),
                'filt_score': int(parts[8]),
                'core': int(parts[9]),
                'oheight': int(parts[10]),
                'owidth': int(parts[11]),
                'conv_mean_def': int(parts[12]),
                'conv_max_def': int(parts[13]),
                'conv_gt2_pct': int(parts[14]),
                'convex_ratio': int(parts[15]),
            }
        except (ValueError, IndexError):
            return None

    def parse_scores_line(self, line: str):
        """
        解析 DEBUG_SCORES 行
        格式: frame_id,blob_num, track_id0,area0,core_ratio0,aspect0,raw_beacon0,raw_marker0,filt_beacon0,filt_marker0,type0, ...
        type: 0=UNKNOWN, 1=BEACON, 2=CAR_MARKER
        """
        parts = line.strip().split(',')
        if len(parts) < 4 or not parts[0].isdigit():
            return None
        try:
            fid = int(parts[0])
            bn = int(parts[1])
            blobs = []
            idx = 2
            for _ in range(min(bn, 5)):
                if idx + 8 >= len(parts):
                    break
                blobs.append({
                    'track_id': int(parts[idx]),
                    'area': int(parts[idx+1]),
                    'core_ratio': int(parts[idx+2]),
                    'aspect': int(parts[idx+3]),
                    'raw_beacon': int(parts[idx+4]),
                    'raw_marker': int(parts[idx+5]),
                    'filt_beacon': int(parts[idx+6]),
                    'filt_marker': int(parts[idx+7]),
                    'type': int(parts[idx+8]),
                })
                idx += 9
            return {'frame_id': fid, 'blob_num': bn, 'blobs': blobs}
        except (ValueError, IndexError):
            return None

    def update_stats(self, data: dict):
        """更新统计数据"""
        self.total_blobs += 1
        self.total_frames = max(self.total_frames, data['frame_id'])

        self.hough_cnt_dist[data['hough_cnt']] += 1

        if data['v_found']:
            self.v_found_count += 1
            if data.get('hough_v', 0):
                self.hough_v_count += 1
            self.v_angles.append(data['angle'])
            self.v_heights.append(data['height'])
            self.v_arm1s.append(data['arm1'])
            self.v_arm2s.append(data['arm2'])
            self.v_bases.append(data['base'])

            # ---- 端点坐标记录 ----
            self.endpoint_bx.append(data['bx'])
            self.endpoint_by.append(data['by'])
            self.endpoint_cx.append(data['cx_e'])
            self.endpoint_cy.append(data['cy_e'])
            self.endpoint_mx.append(data['mid_x'])
            self.endpoint_my.append(data['mid_y'])
            self.endpoint_frame_ids.append(data['frame_id'])

            # ---- 原始坐标记录 (诊断) ----
            if not hasattr(self, 'raw_mid_x_list'):
                self.raw_mid_x_list = []
                self.raw_mid_y_list = []
            if data.get('raw_mid_x', 0) != 0 or data.get('raw_mid_y', 0) != 0:
                self.raw_mid_x_list.append(data['raw_mid_x'])
                self.raw_mid_y_list.append(data['raw_mid_y'])

            # ---- 凸包比收集 ----
            if not hasattr(self, 'convex_ratio_list'):
                self.convex_ratio_list = []
            self.convex_ratio_list.append(data.get('convex_ratio', 100))

            # ---- 距离标准差比收集 ----
            if not hasattr(self, 'dist_std_ratio_list'):
                self.dist_std_ratio_list = []
            self.dist_std_ratio_list.append(data.get('dist_std_ratio', 0))

            # ---- 帧间漂移计算 ----
            if self.prev_bx is not None:
                b_drift = math.sqrt((data['bx'] - self.prev_bx)**2 +
                                    (data['by'] - self.prev_by)**2)
                c_drift = math.sqrt((data['cx_e'] - self.prev_cx)**2 +
                                    (data['cy_e'] - self.prev_cy)**2)
                self.b_drift_list.append(b_drift)
                self.c_drift_list.append(c_drift)
            self.prev_bx = data['bx']
            self.prev_by = data['by']
            self.prev_cx = data['cx_e']
            self.prev_cy = data['cy_e']

            self.recent_v += 1
        else:
            # 未找到 V 形时重置帧间追踪
            self.prev_bx = None
            self.prev_by = None
            self.prev_cx = None
            self.prev_cy = None

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
        """打印统计摘要（含端点分析）"""
        elapsed = time.time() - self.start_time if self.start_time else 0
        print("\n" + "=" * 60)
        print("                    霍夫变换特征统计摘要")
        # ---- DEBUG_SCORES 分类统计 ----
        if self.scores_list:
            print("  ── DEBUG_SCORES 分类统计 ──")
            print(f"    有效帧数        : {len(self.scores_list)}")
            # 统计各类型的 blob 出现次数
            type_counts = {0: 0, 1: 0, 2: 0}
            tnames = {0: 'UNKNOWN', 1: 'BEACON', 2: 'CAR_MARKER'}
            for s in self.scores_list:
                for b in s['blobs']:
                    type_counts[b['type']] = type_counts.get(b['type'], 0) + 1
            total_blob_occur = sum(type_counts.values())
            if total_blob_occur > 0:
                for t in [1, 2, 0]:
                    n = type_counts.get(t, 0)
                    pct = n / total_blob_occur * 100
                    print(f"    {tnames[t]:12s}: {n:5d} ({pct:5.1f}%)")
            # 类型切换事件
            if self.type_switches:
                print(f"\n    ⚠️ 类型切换事件 ({len(self.type_switches)} 次):")
                for fid, tid, old, new in self.type_switches[-10:]:
                    print(f"      frame={fid} T{tid}: {old}→{new}")
            print()

        # ---- 凸包比统计 ----
        if hasattr(self, 'convex_ratio_list') and len(self.convex_ratio_list) > 0:
            crs = self.convex_ratio_list
            avg_cr = sum(crs) / len(crs)
            min_cr = min(crs)
            max_cr = max(crs)
            concave = sum(1 for c in crs if c < 85)
            print("  ── 凸包面积比统计 ──")
            print(f"    平均: {avg_cr:.1f}%  最小: {min_cr}%  最大: {max_cr}%")
            print(f"    凹形(凸包比<85): {concave}/{len(crs)} ({concave*100/len(crs):.0f}%)")
            print()

        # ---- 距离标准差比统计 ----
        if hasattr(self, 'dist_std_ratio_list') and len(self.dist_std_ratio_list) > 0:
            srs = self.dist_std_ratio_list
            avg_sr = sum(srs) / len(srs)
            min_sr = min(srs)
            max_sr = max(srs)
            slender = sum(1 for s in srs if s > 35)
            print("  ── 距离标准差比统计 (V形>35) ──")
            print(f"    平均: {avg_sr:.1f}%  最小: {min_sr}%  最大: {max_sr}%")
            print(f"    V形特征(>35): {slender}/{len(srs)} ({slender*100/len(srs):.0f}%)")
            print()

        print("=" * 60)
        print(f"  录制时长        : {elapsed:.1f} 秒")
        print(f"  总帧数          : {self.total_frames}")
        print(f"  总 blob 数      : {self.total_blobs}")
        print()

        # V 形检测率
        if self.total_blobs > 0:
            rate = self.v_found_count / self.total_blobs * 100
            print(f"  V 形检出率      : {rate:.1f}% ({self.v_found_count}/{self.total_blobs})")
            hough_rate = self.hough_v_count / self.total_blobs * 100
            print(f"  霍夫验证通过率   : {hough_rate:.1f}% ({self.hough_v_count}/{self.total_blobs})")
            print(f"  (v_found=1但hough_v=0 → 可能是圆形信标)")

        # 信标统计
        if self.beacon_list:
            bcx = [b['cx'] for b in self.beacon_list]
            bcy = [b['cy'] for b in self.beacon_list]
            barea = [b['area'] for b in self.beacon_list]
            bscore = [b['filt_score'] for b in self.beacon_list]
            print()
            print(f"  ── 信标数据 ({len(self.beacon_list)} 帧) ──")
            print(f"    位置 X         : 均值={self._avg(bcx):.0f}  范围=[{min(bcx)},{max(bcx)}]")
            print(f"    位置 Y         : 均值={self._avg(bcy):.0f}  范围=[{min(bcy)},{max(bcy)}]")
            print(f"    面积           : 均值={self._avg(barea):.0f}  范围=[{min(barea)},{max(barea)}]")
            print(f"    滤波分数       : 均值={self._avg(bscore):.1f}  范围=[{min(bscore)},{max(bscore)}]")
            print()

        # 帧率估算 (基于总帧数)
        if elapsed > 0:
            fps = self.total_frames / elapsed
            print(f"  平均帧率        : {fps:.1f} fps")
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

            # ---- 端点坐标分析 ----
            print("  ── 端点坐标分析 (V 形成功时) ──")
            print(f"    B 点 (base1)    : x 均值={self._avg(self.endpoint_bx):.0f}  "
                  f"范围=[{min(self.endpoint_bx)},{max(self.endpoint_bx)}]")
            print(f"                    : y 均值={self._avg(self.endpoint_by):.0f}  "
                  f"范围=[{min(self.endpoint_by)},{max(self.endpoint_by)}]")
            print(f"    C 点 (base2)    : x 均值={self._avg(self.endpoint_cx):.0f}  "
                  f"范围=[{min(self.endpoint_cx)},{max(self.endpoint_cx)}]")
            print(f"                    : y 均值={self._avg(self.endpoint_cy):.0f}  "
                  f"范围=[{min(self.endpoint_cy)},{max(self.endpoint_cy)}]")
            print(f"    底边中点 (mid)  : x 均值={self._avg(self.endpoint_mx):.0f}  "
                  f"范围=[{min(self.endpoint_mx)},{max(self.endpoint_mx)}]")
            print(f"                    : y 均值={self._avg(self.endpoint_my):.0f}  "
                  f"范围=[{min(self.endpoint_my)},{max(self.endpoint_my)}]")

            # ---- 原始 vs 平滑坐标对比 (诊断"卡住"问题) ----
            if hasattr(self, 'raw_mid_x_list') and len(self.raw_mid_x_list) > 0:
                print()
                print("  ── 原始 vs 平滑坐标诊断 ──")
                delta_list = [math.sqrt((sx-rx)**2 + (sy-ry)**2)
                              for rx, ry, sx, sy in zip(self.raw_mid_x_list, self.raw_mid_y_list,
                                                        self.endpoint_mx, self.endpoint_my)]
                print(f"    平滑-原始偏离 : 均值={self._avg(delta_list):.1f}  "
                      f"最大={max(delta_list):.1f} px")
                stuck = sum(1 for d in delta_list if d > 5)
                print(f"    偏离>5px      : {stuck}/{len(delta_list)}")
                # 找偏离大的帧
                for i, d in enumerate(delta_list):
                    if d > 10:
                        print(f"    frame={self.endpoint_frame_ids[i]}  "
                              f"raw=({self.raw_mid_x_list[i]},{self.raw_mid_y_list[i]})  "
                              f"smoothed=({self.endpoint_mx[i]},{self.endpoint_my[i]})  "
                              f"delta={d:.0f}px")

            # 端点相对于图像中心的偏移 (判断端点是否偏到角落)
            print()
            print("  ── 端点是否偏到角落 ──")
            img_cx = 188 / 2  # CAMERA_W/2
            img_cy = 120 / 2  # CAMERA_H/2
            b_off = [math.sqrt((bx - img_cx)**2 + (by - img_cy)**2)
                     for bx, by in zip(self.endpoint_bx, self.endpoint_by)]
            c_off = [math.sqrt((cx - img_cx)**2 + (cy - img_cy)**2)
                     for cx, cy in zip(self.endpoint_cx, self.endpoint_cy)]
            print(f"    B 距画面中心    : 均值={self._avg(b_off):.0f}  "
                  f"最大={max(b_off):.0f} px")
            print(f"    C 距画面中心    : 均值={self._avg(c_off):.0f}  "
                  f"最大={max(c_off):.0f} px")
            print()

            # ---- 帧间端点漂移分析 ----
            if len(self.b_drift_list) > 0:
                print("  ── 帧间端点漂移 ──")
                avg_b = self._avg(self.b_drift_list)
                max_b = max(self.b_drift_list)
                avg_c = self._avg(self.c_drift_list)
                max_c = max(self.c_drift_list)

                # 计算漂移波动 (标准差)
                std_b = math.sqrt(sum((d - avg_b)**2 for d in self.b_drift_list) / len(self.b_drift_list)) if len(self.b_drift_list) > 1 else 0
                std_c = math.sqrt(sum((d - avg_c)**2 for d in self.c_drift_list) / len(self.c_drift_list)) if len(self.c_drift_list) > 1 else 0

                print(f"    B 点漂移 (px)   : 均值={avg_b:.2f}  σ={std_b:.2f}  "
                      f"最大={max_b:.1f}  (n={len(self.b_drift_list)})")
                print(f"    C 点漂移 (px)   : 均值={avg_c:.2f}  σ={std_c:.2f}  "
                      f"最大={max_c:.1f}  (n={len(self.c_drift_list)})")

                # 漂移分级 (统计漂移 > 5px 的占比)
                bad_b = sum(1 for d in self.b_drift_list if d > 5)
                bad_c = sum(1 for d in self.c_drift_list if d > 5)
                if len(self.b_drift_list) > 0:
                    print(f"    B 漂移>5px占比  : {bad_b / len(self.b_drift_list) * 100:.1f}%")
                if len(self.c_drift_list) > 0:
                    print(f"    C 漂移>5px占比  : {bad_c / len(self.c_drift_list) * 100:.1f}%")
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

        # ---- HDBG 配对拒绝原因汇总 ----
        if self.hdbg_frames:
            print("  ── 霍夫配对拒绝原因汇总 (HDBG) ──")
            n = len(self.hdbg_frames)
            for k in ['total', 'angle_diff', 'parallel', 'oob', 'nbhd', 'bbox',
                      'base_w', 'angle', 'height', 'arm', 'edge', 'pass']:
                v = self.hdbg_sum.get(k, 0)
                avg = v / n if n > 0 else 0
                t = self.hdbg_sum.get('total', 1)
                pct = v / t * 100 if t > 0 else 0
                names = {
                    'angle_diff': 'θ差异不满足V形范围',
                    'parallel': '平行线(行列式≈0)',
                    'oob': '交点超出图像边界',
                    'nbhd': '3×3邻域白像素不足',
                    'bbox': '端点超出blob矩形',
                    'base_w': '底边宽度不合理',
                    'angle': '顶点角度不满足',
                    'height': '高度不足',
                    'arm': '臂长<4(交叉配对)',
                    'edge': '靠近图像边界',
                    'pass': '通过(可选入评分)',
                    'total': '总配对尝试'
                }
                label = names.get(k, k)
                pass_mark = '  ← 通过!' if k == 'pass' else ''
                print(f"    {label:20s}: 均值={avg:5.1f}  占比={pct:5.1f}%{pass_mark}")
            print()

        print("=" * 60)
        print(f"  原始数据已保存至: {self.output_file}")
        print("=" * 60)

    def _rel_to_center(self, x_vals, center_vals, _=None):
        """计算 x 序列相对质心的偏移"""
        if not x_vals or not center_vals:
            return [0]
        n = min(len(x_vals), len(center_vals))
        return [x_vals[i] - center_vals[i] for i in range(n)]

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

        # 信标 CSV 文件 (记录 B 行凸度数据)
        beacon_csv_file = self.output_file.replace('.csv', '_beacon.csv')
        self.beacon_csv_writer = None

        with open(self.output_file, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow([
                'frame_id', 'area', 'cx', 'cy', 'w', 'h',
                'hough_cnt', 'thresh', 'angles', 'v_found', 'hough_v',
                'angle', 'height', 'arm1', 'arm2', 'base',
                'bx', 'by', 'cx_e', 'cy_e', 'mid_x', 'mid_y',
                'raw_mid_x', 'raw_mid_y', 'convex_ratio', 'dist_std_ratio',
                'conv_mean_def', 'conv_max_def', 'conv_gt2_pct',
                'lines_data'
            ])

            # 信标 CSV
            bf = open(beacon_csv_file, 'w', newline='', encoding='utf-8')
            self.beacon_csv_writer = csv.writer(bf)
            self.beacon_csv_file = bf  # 保存用于 flush
            self.beacon_csv_writer.writerow([
                'frame_id', 'area', 'cx', 'cy', 'w', 'h',
                'raw_score', 'filt_score', 'core', 'oheight', 'owidth',
                'conv_mean_def', 'conv_max_def', 'conv_gt2_pct',
                'convex_ratio',
            ])
            bf.flush()

            while self.running:
                try:
                    line = self.ser.readline()
                    if not line:
                        self.print_realtime()
                        continue

                    text = line.decode('utf-8', errors='replace').strip()

                    # HDBG 行: 配对拒绝原因统计 (实时打印)
                    if text.startswith('HDBG,'):
                        hdbg = self.parse_hdbg_line(text)
                        if hdbg:
                            self.hdbg_frames.append(hdbg)
                            for k in self.hdbg_sum:
                                self.hdbg_sum[k] += hdbg[k]
                            if self.hdbg_frames:
                                last = self.hdbg_frames[-1]
                                if last['total'] > 0:
                                    items = [(k, last[k]) for k in [
                                        'angle_diff','nbhd','bbox','base_w','angle',
                                        'height','arm','edge','pass']]
                                    items = [(k, v) for k, v in items if v > 0]
                                    if items:
                                        detail = ' '.join(f"{k}={v}" for k, v in items)
                                        print(f"  [HDBG] frame={last['frame_id']} total={last['total']} | {detail}")
                        continue

                    # STUCK 诊断行: 实时打印 (在 H 行过滤之前)
                    if text.startswith('STUCK,'):
                        parts = text.split(',')
                        if len(parts) >= 13:
                            print(f"  [STUCK] frame={parts[1]} mid=({parts[2]},{parts[3]}) "
                                  f"vert=({parts[4]},{parts[5]}) cen=({parts[6]},{parts[7]}) "
                                  f"B=({parts[8]},{parts[9]}) C=({parts[10]},{parts[11]}) "
                                  f"area={parts[12]}")
                        continue

                    # DEBUG_SCORES 行: 打印每个 blob 的分数和分类
                    if text[0].isdigit() and not text.startswith('H,'):
                        sdata = self.parse_scores_line(text)
                        if sdata:
                            self.scores_list.append(sdata)
                            tnames = {0: '?', 1: 'B', 2: 'C'}
                            # 检测类型切换
                            blobs_str = []
                            for b in sdata['blobs']:
                                tid = b['track_id']
                                tname = tnames.get(b['type'], '?')
                                # 检查与前帧相比类型是否切换
                                if tid in self.prev_types and self.prev_types[tid] != b['type']:
                                    old = tnames.get(self.prev_types[tid], '?')
                                    new = tnames.get(b['type'], '?')
                                    self.type_switches.append((sdata['frame_id'], tid, old, new))
                                blobs_str.append(f"T{tid}:B={b['filt_beacon']} C={b['filt_marker']}→{tname}")
                            self.prev_types = {b['track_id']: b['type'] for b in sdata['blobs']}
                            print(f"  [分数#{sdata['frame_id']}] " + " | ".join(blobs_str))
                            # 实时打印类型切换
                            if self.type_switches and self.type_switches[-1][0] == sdata['frame_id']:
                                _, tid, old_t, new_t = self.type_switches[-1]
                                print(f"  ⚠️ 类型切换: T{tid}: {old_t}→{new_t}")
                        continue

                    if text.startswith('B,'):
                        bcon = self.parse_beacon_line(text)
                        if bcon:
                            self.beacon_list.append(bcon)
                            # 写入信标 CSV
                            if self.beacon_csv_writer:
                                self.beacon_csv_writer.writerow([
                                    bcon['frame_id'], bcon['area'],
                                    bcon['cx'], bcon['cy'],
                                    bcon['w'], bcon['h'],
                                    bcon['raw_score'], bcon['filt_score'],
                                    bcon['core'], bcon['oheight'],
                                    bcon['owidth'],
                                    bcon['conv_mean_def'],
                                    bcon['conv_max_def'],
                                    bcon['conv_gt2_pct'],
                                    bcon['convex_ratio'],
                                ])
                                self.beacon_csv_file.flush()
                            # 单行实时显示简洁的信标信息 + 凸度
                            print(f"  [信标] frame={bcon['frame_id']} "
                                  f"pos=({bcon['cx']},{bcon['cy']}) area={bcon['area']} "
                                  f"score={bcon['filt_score']} "
                                  f"凸比={bcon['convex_ratio']}% "
                                  f"core={bcon['core']} "
                                  f"方向宽高比={bcon['owidth']}/{bcon['oheight']}={bcon['owidth']*100//max(bcon['oheight'],1)}%")
                        continue

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
                        data['angles'], data['v_found'], data['hough_v'],
                        data['angle'],
                        data['height'], data['arm1'], data['arm2'], data['base'],
                        data['bx'], data['by'], data['cx_e'], data['cy_e'],
                        data['mid_x'], data['mid_y'],
                        data['raw_mid_x'], data['raw_mid_y'],
                        data['convex_ratio'],
                        data.get('dist_std_ratio', 0),
                        data['conv_mean_def'],
                        data['conv_max_def'],
                        data['conv_gt2_pct'],
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
