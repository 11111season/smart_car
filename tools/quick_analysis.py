#!/usr/bin/env python3
"""Quick analysis of CSV log"""
import csv
import sys

fname = sys.argv[1] if len(sys.argv) > 1 else "logs/debug_log.csv"
with open(fname, 'r') as f:
    r = csv.DictReader(f)
    rows = list(r)

if not rows:
    print("No data")
    sys.exit(0)

# Small area frames
print('=== 小面积帧 (area < 200) ===')
for row in rows:
    a = int(row['area'])
    if a < 200:
        print(f"  area={a:3d} angle={row['angle']:>2s} std={row['dist_std_ratio']:>2s} convex={row['convex_ratio']:>2s} w={row['w']:>2s} h={row['h']:>2s} hv={row['hough_v']} bx={row['bx']:>2s} by={row['by']:>2s} cx={row['cx_e']:>2s} cy={row['cy_e']:>2s}")

# Stuck mid points
stuck = 0
for i in range(1, len(rows)):
    px, py = int(rows[i-1]['mid_x']), int(rows[i-1]['mid_y'])
    cx, cy = int(rows[i]['mid_x']), int(rows[i]['mid_y'])
    if px == cx and py == cy:
        stuck += 1

# Endpoint consistency (base1-base2 distance variation)
dists = []
for row in rows:
    bx, by = int(row['bx']), int(row['by'])
    cx, cy = int(row['cx_e']), int(row['cy_e'])
    d = ((bx-cx)**2 + (by-cy)**2)**0.5
    dists.append(d)

print(f'\n=== 统计 ===')
print(f'总帧数: {len(rows)}')
print(f'中点不变帧: {stuck}/{len(rows)} ({stuck*100/len(rows):.0f}%)')
print(f'面积范围: {min(int(r["area"]) for r in rows)}-{max(int(r["area"]) for r in rows)}')
print(f'底边长度: 平均={sum(dists)/len(dists):.1f} 范围={min(dists):.0f}-{max(dists):.0f}')

# Check endpoint wobble (frame-to-frame change)
bx_changes = []
by_changes = []
for i in range(1, len(rows)):
    dbx = abs(int(rows[i]['bx']) - int(rows[i-1]['bx']))
    dby = abs(int(rows[i]['by']) - int(rows[i-1]['by']))
    bx_changes.append(dbx)
    by_changes.append(dby)
print(f'B端点帧间变化: X平均={sum(bx_changes)/len(bx_changes):.1f}px Y平均={sum(by_changes)/len(by_changes):.1f}px')

# dist_std_ratio stats
srs = [int(r['dist_std_ratio']) for r in rows]
print(f'dist_std_ratio: 平均={sum(srs)/len(srs):.1f} 范围={min(srs)}-{max(srs)}')
