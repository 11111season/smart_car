import difflib
import os

check = r'D:\UsersASUSDesktop\new\smart_car-main\smart_car-main\smart_car-main\project\check'
user = r'D:\UsersASUSDesktop\new\smart_car-main\smart_car-main\smart_car-main\project\user'
code_dir = r'D:\UsersASUSDesktop\new\smart_car-main\smart_car-main\smart_car-main\project\code'

pairs = [
    (os.path.join(check, 'main_cm7_0.c'), os.path.join(user, 'main_cm7_0.c')),
    (os.path.join(check, 'main_cm7_1.c'), os.path.join(user, 'main_cm7_1.c')),
    (os.path.join(check, 'cm7_0_isr.c'),  os.path.join(user, 'cm7_0_isr.c')),
    (os.path.join(check, 'cm7_1_isr.c'),  os.path.join(user, 'cm7_1_isr.c')),
    (os.path.join(check, 'camera.c'),     os.path.join(code_dir, 'camera.c')),
    (os.path.join(check, 'camera.h'),     os.path.join(code_dir, 'camera.h')),
]

for chk_file, cur_file in pairs:
    print(f'\n{"="*60}')
    print(f'COMPARING: {os.path.basename(chk_file)}')
    print(f'  check: {chk_file}')
    print(f'  cur:   {cur_file}')
    
    if not os.path.exists(cur_file):
        print(f'  *** CUR FILE NOT FOUND ***')
        continue
    
    f1 = open(chk_file, encoding='utf-8').readlines()
    f2 = open(cur_file, encoding='utf-8').readlines()
    
    if f1 == f2:
        print('  ==> IDENTICAL')
    else:
        print(f'  ==> DIFFERENT (check={len(f1)} lines, cur={len(f2)} lines)')
        diff = difflib.unified_diff(f1, f2, fromfile='check', tofile='current', n=3)
        for line in diff:
            print('  ' + line.rstrip())
