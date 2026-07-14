import csv, math, sys

f = sys.argv[1]
with open(f) as fh:
    rows = list(csv.DictReader(fh))
print(f'总帧数: {len(rows)}')

v_n = sum(1 for r in rows if r['v_found']=='1')
hv_n = sum(1 for r in rows if r.get('hough_v','0')=='1')
print(f'V形检出率: {v_n}/{len(rows)} = {v_n/len(rows)*100:.1f}%')
print(f'霍夫验证率: {hv_n}/{len(rows)} = {hv_n/len(rows)*100:.1f}%')
if v_n > 0 and hv_n < v_n:
    print(f'  (!) {v_n-hv_n} 帧 v_found=1 但 hough_v=0')

found = [r for r in rows if r['v_found']=='1']
if found:
    arm1s = [int(r['arm1']) for r in found]
    arm2s = [int(r['arm2']) for r in found]
    print(f'\n臂长1: 均值={sum(arm1s)/len(arm1s):.0f} 范围=[{min(arm1s)},{max(arm1s)}]')
    print(f'臂长2: 均值={sum(arm2s)/len(arm2s):.0f} 范围=[{min(arm2s)},{max(arm2s)}]')

    drifts_b, drifts_c = [], []
    for i in range(1, len(found)):
        db = math.sqrt((int(found[i]['bx'])-int(found[i-1]['bx']))**2 + 
                       (int(found[i]['by'])-int(found[i-1]['by']))**2)
        dc = math.sqrt((int(found[i]['cx_e'])-int(found[i-1]['cx_e']))**2 + 
                       (int(found[i]['cy_e'])-int(found[i-1]['cy_e']))**2)
        drifts_b.append(db)
        drifts_c.append(dc)
    if drifts_b:
        print(f'\nB帧间漂移: 均值={sum(drifts_b)/len(drifts_b):.1f} 最大={max(drifts_b):.1f} >5px: {sum(1 for d in drifts_b if d>5)}/{len(drifts_b)}')
        print(f'C帧间漂移: 均值={sum(drifts_c)/len(drifts_c):.1f} 最大={max(drifts_c):.1f} >5px: {sum(1 for d in drifts_c if d>5)}/{len(drifts_c)}')
    
    print('\n大跳变帧(>10px):')
    for i in range(1, len(found)):
        db = math.sqrt((int(found[i]['bx'])-int(found[i-1]['bx']))**2 + 
                       (int(found[i]['by'])-int(found[i-1]['by']))**2)
        dc = math.sqrt((int(found[i]['cx_e'])-int(found[i-1]['cx_e']))**2 + 
                       (int(found[i]['cy_e'])-int(found[i-1]['cy_e']))**2)
        if db > 10 or dc > 10:
            print(f'  frame={found[i]["frame_id"]} B漂={db:.1f} B=({found[i]["bx"]},{found[i]["by"]}) C漂={dc:.1f} C=({found[i]["cx_e"]},{found[i]["cy_e"]}) arm1={found[i]["arm1"]} arm2={found[i]["arm2"]}')

    # 检查端点是否靠近图像边界
    bx_list = [int(r['bx']) for r in found]
    by_list = [int(r['by']) for r in found]
    cx_list = [int(r['cx_e']) for r in found]
    cy_list = [int(r['cy_e']) for r in found]
    print(f'\nBx范围=[{min(bx_list)},{max(bx_list)}] By范围=[{min(by_list)},{max(by_list)}]')
    print(f'Cx范围=[{min(cx_list)},{max(cx_list)}] Cy范围=[{min(cy_list)},{max(cy_list)}]')
    
    # B和C交换检测: arm异常短 + 另侧臂异常长
    flip = 0
    for r in found:
        a1, a2 = int(r['arm1']), int(r['arm2'])
        if (a1 < 5 and a2 > 15) or (a2 < 5 and a1 > 15):
            flip += 1
    if flip > 0:
        print(f'\n疑似B/C端点交换: {flip}/{len(found)} = {flip/len(found)*100:.1f}%')
