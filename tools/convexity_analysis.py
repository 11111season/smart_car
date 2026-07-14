import csv, sys

fn = sys.argv[1] if len(sys.argv) > 1 else 'hough_log_20260714_204614_beacon.csv'

with open(fn) as f:
    reader = csv.DictReader(f)
    blobs = {
        'T1(小面积-信标)': {'cr': [], 'avg': [], 'mx': [], 'gt2': []},
        'T2(大面积-V形)': {'cr': [], 'avg': [], 'mx': [], 'gt2': []},
    }

    for row in reader:
        area = float(row['area'])
        idx = 'T1(小面积-信标)' if area < 185 else 'T2(大面积-V形)'
        blobs[idx]['cr'].append(float(row['convex_ratio']))
        blobs[idx]['avg'].append(float(row['conv_mean_def']))
        blobs[idx]['mx'].append(float(row['conv_max_def']))
        blobs[idx]['gt2'].append(float(row['conv_gt2_pct']))

print('=' * 70)
for name, d in blobs.items():
    if not d['cr']:
        continue
    n = len(d['cr'])
    print(f'  {name} ({n} 帧):')
    print(f'    凸包比:     {min(d["cr"]):3.0f} ~ {max(d["cr"]):3.0f}  均值={sum(d["cr"])/n:5.1f}')
    print(f'    凸度avg:   {min(d["avg"]):3.0f} ~ {max(d["avg"]):3.0f}  均值={sum(d["avg"])/n:5.1f}')
    print(f'    凸度max:   {min(d["mx"]):3.0f} ~ {max(d["mx"]):3.0f}  均值={sum(d["mx"])/n:5.1f}')
    print(f'    >2%:        {min(d["gt2"]):3.0f} ~ {max(d["gt2"]):3.0f}  均值={sum(d["gt2"])/n:5.1f}')
    print()

# 区分度分析
if blobs['T1(小面积-信标)']['cr'] and blobs['T2(大面积-V形)']['cr']:
    t1_cr = sum(blobs['T1(小面积-信标)']['cr']) / len(blobs['T1(小面积-信标)']['cr'])
    t2_cr = sum(blobs['T2(大面积-V形)']['cr']) / len(blobs['T2(大面积-V形)']['cr'])
    t1_gt2 = sum(blobs['T1(小面积-信标)']['gt2']) / len(blobs['T1(小面积-信标)']['gt2'])
    t2_gt2 = sum(blobs['T2(大面积-V形)']['gt2']) / len(blobs['T2(大面积-V形)']['gt2'])

    print('--- 区分度分析 ---')
    print(f'  凸包比差:    T2={t2_cr:.1f}  T1={t1_cr:.1f}  差={t1_cr-t2_cr:.1f}  ★明显')
    print(f'  >2%%差:      T2={t2_gt2:.1f}  T1={t1_gt2:.1f}  差={t2_gt2-t1_gt2:.1f}   {"★明显" if abs(t2_gt2-t1_gt2)>8 else "不明显"}')
