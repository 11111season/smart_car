import csv, statistics, sys

fn = sys.argv[1] if len(sys.argv) > 1 else 'hough_log_20260714_205702_beacon.csv'

with open(fn) as f:
    rows = list(csv.DictReader(f))

# 凸包比分离: 信标≈100%, 车标<90%
beac = [r for r in rows if float(r['convex_ratio']) > 95]
car  = [r for r in rows if float(r['convex_ratio']) < 90]

print('=' * 65)
print(f'总帧数: {len(rows)}')
print(f'信标(凸比>95): {len(beac)} 帧')
print(f'车标(凸比<90): {len(car)} 帧')
print()

for label, group in [('信标', beac), ('车标', car)]:
    if not group:
        print(f'  {label}: 无数据')
        print()
        continue
    n = len(group)
    cr   = [float(r['convex_ratio']) for r in group]
    area = [int(r['area']) for r in group]
    wdir = [int(r['owidth'])*100//max(int(r['oheight']),1) for r in group]
    ca   = [int(r['core'])*100//int(r['area']) for r in group]
    cavg = [float(r['conv_mean_def']) for r in group]
    cmax = [float(r['conv_max_def']) for r in group]
    gt2  = [float(r['conv_gt2_pct']) for r in group]

    print(f'  {label} ({n}帧):')
    print(f'    面积:       {min(area)}~{max(area)}  均值={statistics.mean(area):.0f}')
    print(f'    凸包比:     {min(cr):.0f}~{max(cr):.0f}  均值={statistics.mean(cr):.1f}')
    print(f'    方向宽高比: {min(wdir)}~{max(wdir)}  均值={statistics.mean(wdir):.0f}%')
    print(f'    core占比:   {min(ca)}~{max(ca)}  均值={statistics.mean(ca):.0f}%')
    print(f'    凸度avg:    {min(cavg):.1f}~{max(cavg):.1f}  均值={statistics.mean(cavg):.1f}')
    print(f'    凸度max:    {min(cmax):.0f}~{max(cmax):.0f}  均值={statistics.mean(cmax):.1f}')
    print(f'    >2%:        {min(gt2):.0f}~{max(gt2):.0f}  均值={statistics.mean(gt2):.1f}')
    print()

# 区分度分析
if beac and car:
    print('--- 区分度分析 ---')
    for name, getter in [
        ('凸包比',     lambda g: statistics.mean([float(r['convex_ratio'])   for r in g])),
        ('方向宽高比', lambda g: statistics.mean([int(r['owidth'])*100//max(int(r['oheight']),1) for r in g])),
        ('core占比',  lambda g: statistics.mean([int(r['core'])*100//int(r['area']) for r in g])),
        ('凸度avg',  lambda g: statistics.mean([float(r['conv_mean_def']) for r in g])),
        ('凸度max',  lambda g: statistics.mean([float(r['conv_max_def']) for r in g])),
        ('>2%',      lambda g: statistics.mean([float(r['conv_gt2_pct'])  for r in g])),
    ]:
        v1 = getter(beac)
        v2 = getter(car)
        diff = abs(v2 - v1)
        mark = '★明显' if diff / max(v1, v2) > 0.2 else '一般'
        print(f'  {name}: 信标={v1:.1f}  车标={v2:.1f}  差={diff:.1f}  {mark}')
