import csv, math, sys

f = sys.argv[1]
with open(f) as fh:
    rows = list(csv.DictReader(fh))

found = [r for r in rows if r['v_found'] == '1']
print(f'总帧数: {len(found)}')

swap = 0
for i in range(1, len(found)):
    bx = int(found[i]['bx']); by = int(found[i]['by'])
    cx = int(found[i]['cx_e']); cy = int(found[i]['cy_e'])
    pbx = int(found[i-1]['bx']); pby = int(found[i-1]['by'])
    pcx = int(found[i-1]['cx_e']); pcy = int(found[i-1]['cy_e'])

    d_same = (bx-pbx)**2 + (by-pby)**2 + (cx-pcx)**2 + (cy-pcy)**2
    d_cross = (bx-pcx)**2 + (by-pcy)**2 + (cx-pbx)**2 + (cy-pby)**2

    if d_same > d_cross:
        swap += 1
        if swap <= 10:
            print(f'  frame={found[i]["frame_id"]} swap! B=({bx},{by}) C=({cx},{cy}) prev_B=({pbx},{pby}) prev_C=({pcx},{pcy})')

total = len(found) - 1
print(f'\nB/C交换次数: {swap}/{total} = {swap/total*100:.1f}%')
