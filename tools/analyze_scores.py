#!/usr/bin/env python3
"""
DEBUG_SCORES 日志分析工具
=======================
分析串口录制的特征值 CSV 文件，输出完整统计报告。

用法:
    python analyze_scores.py serial_log_20260714_234912.csv
"""

import sys
from collections import defaultdict, Counter
from datetime import datetime

fname = sys.argv[1] if len(sys.argv) > 1 else "serial_log.csv"
lines = open(fname, "r", encoding="utf-8").readlines()

# 解析
frames = []        # [(frame_id, blob_num, [(track_id, ...)])]
type_log = []      # [(frame_id, track_id, old_type, new_type)]
unknowns = set()   # track_ids that were ever UNKNOWN
beacons = set()    # track_ids that were ever BEACON
markers = set()    # track_ids that were ever CAR_MARKER
prev_types = {}    # track_id -> last known type

beacon_scores = []  # (filt_beacon, filt_marker) for BEACON
marker_scores = []  # (filt_beacon, filt_marker) for CAR_MARKER

start_time = None
end_time = None
blob_count_hist = Counter()
frame_ids_seen = set()

for line in lines:
    line = line.strip()
    if not line or line.startswith("time,"):
        continue
    # Skip non-data lines
    if line.startswith("BEACON:") or line.startswith("[") or line.startswith("Welcome"):
        continue
    # Parse: time,frame_id,blob_num, [track_id,area,core_ratio,aspect,raw_b,raw_m,filt_b,filt_m,type] × N
    parts = line.split(",")
    if len(parts) < 3:
        continue
    try:
        ts = parts[0]
        frame_id = int(parts[1])
        blob_num = int(parts[2])
    except ValueError:
        continue

    if start_time is None:
        start_time = ts
    end_time = ts
    frame_ids_seen.add(frame_id)
    blob_count_hist[blob_num] += 1

    # Parse blobs
    blobs = []
    idx = 3
    for _ in range(blob_num):
        if idx + 9 > len(parts):
            break
        try:
            b = {
                "track_id": int(parts[idx]),
                "area": int(parts[idx+1]),
                "core_ratio": int(parts[idx+2]),
                "aspect": int(parts[idx+3]),
                "raw_beacon": int(parts[idx+4]),
                "raw_marker": int(parts[idx+5]),
                "filt_beacon": int(parts[idx+6]),
                "filt_marker": int(parts[idx+7]),
                "type": int(parts[idx+8]),
            }
            blobs.append(b)
            idx += 9
        except (ValueError, IndexError):
            break
    frames.append((frame_id, blob_num, blobs))

    # Track types per track_id
    for b in blobs:
        tid = b["track_id"]
        typ = b["type"]
        if typ == 0:
            unknowns.add(tid)
        elif typ == 1:
            beacons.add(tid)
        elif typ == 2:
            markers.add(tid)

        if tid in prev_types and prev_types[tid] != typ and prev_types[tid] > 0 and typ > 0:
            type_log.append((frame_id, tid, prev_types[tid], typ))
        prev_types[tid] = typ

# ── 报告 ──
print("=" * 60)
print("  DEBUG_SCORES 分析报告")
print("=" * 60)

# 基本信息
print(f"\n📊 基本统计")
print(f"  录制时间:    {start_time} ~ {end_time}")
print(f"  总帧数:      {len(frames)}")
print(f"  唯一帧号:    {len(frame_ids_seen)}")
print(f"  唯一 track_id: {len(set(prev_types.keys()))}")
print(f"  含 CAR_MARKER: {len(markers)} track(s)")
print(f"  含 BEACON:     {len(beacons)} track(s)")
print(f"  含 UNKNOWN:    {len(unknowns)} track(s)")

print(f"\n  Blob 数量分布:")
for k in sorted(blob_count_hist):
    pct = blob_count_hist[k] / len(frames) * 100
    bar = "█" * int(pct / 2)
    print(f"    {k} blob(s): {blob_count_hist[k]:>4} 帧 ({pct:5.1f}%) {bar}")

# 类型切换
print(f"\n🔄 类型切换")
if type_log:
    tnames = {0: "UNKNOWN", 1: "BEACON", 2: "CAR_MARKER"}
    for fid, tid, old, new in type_log:
        on = tnames.get(old, f"TYPE_{old}")
        nn = tnames.get(new, f"TYPE_{new}")
        print(f"  frame {fid:>5d}: T{tid}  {on:>10s} → {nn:<10s}")
else:
    print("  (无类型切换)")

# 分数分布
print(f"\n📈 滤波分数分布 (filt_beacon / filt_marker)")
beacon_scores = [(b["filt_beacon"], b["filt_marker"], b)
                 for _, _, blobs in frames for b in blobs if b["type"] == 1]
marker_scores = [(b["filt_beacon"], b["filt_marker"], b)
                 for _, _, blobs in frames for b in blobs if b["type"] == 2]

if beacon_scores:
    bs = [s[0] for s in beacon_scores]
    ms = [s[1] for s in beacon_scores]
    print(f"  ── BEACON ({len(beacon_scores)} samples) ──")
    print(f"    filt_beacon:  min={min(bs):>3d}  max={max(bs):>3d}  avg={sum(bs)/len(bs):.1f}")
    print(f"    filt_marker:  min={min(ms):>3d}  max={max(ms):>3d}  avg={sum(ms)/len(ms):.1f}")

if marker_scores:
    bs = [s[0] for s in marker_scores]
    ms = [s[1] for s in marker_scores]
    print(f"  ── CAR_MARKER ({len(marker_scores)} samples) ──")
    print(f"    filt_beacon:  min={min(bs):>3d}  max={max(bs):>3d}  avg={sum(bs)/len(bs):.1f}")
    print(f"    filt_marker:  min={min(ms):>3d}  max={max(ms):>3d}  avg={sum(ms)/len(ms):.1f}")

# 每个 track 的最后类型
print(f"\n🏷️  每个 track 最终类型")
tnames = {0: "UNKNOWN", 1: "BEACON", 2: "CAR_MARKER"}
for tid in sorted(prev_types.keys()):
    typ = prev_types[tid]
    tn = tnames.get(typ, f"TYPE_{typ}")
    n_frames = sum(1 for _, _, blobs in frames for b in blobs if b["track_id"] == tid)
    print(f"  T{tid:>2d}: {tn:>10s}  (出现 {n_frames:>4d} 帧)")

# 丢失段
print(f"\n📉 丢失段 (blob_num=0)")
lost_starts = []
in_lost = False
lost_start = None
for fid, bn, _ in frames:
    if bn == 0 and not in_lost:
        lost_start = fid
        in_lost = True
    elif bn > 0 and in_lost:
        if lost_start is not None:
            lost_dur = fid - lost_start
            lost_starts.append((lost_start, lost_dur))
        in_lost = False
if in_lost and lost_start is not None:
    lost_starts.append((lost_start, len(frames) - lost_start))

if lost_starts:
    total_lost = sum(d for _, d in lost_starts)
    print(f"  共 {len(lost_starts)} 次丢失, 总计 {total_lost} 帧")
    for s, d in lost_starts:
        print(f"    frame {s:>5d} 持续 {d:>3d} 帧")
else:
    print("  (无丢失)")

print("\n" + "=" * 60)
