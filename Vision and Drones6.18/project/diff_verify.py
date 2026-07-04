import os

check_dir = r'D:\UsersASUSDesktop\new\smart_car-main\smart_car-main\smart_car-main\project\check'
code_dir = r'D:\UsersASUSDesktop\new\smart_car-main\smart_car-main\smart_car-main\project\code'
user_dir = r'D:\UsersASUSDesktop\new\smart_car-main\smart_car-main\smart_car-main\project\user'

files = [
    ('HC06_Driver.c', code_dir),
    ('HC06_Driver.h', code_dir),
    ('main_cm7_0.c', user_dir),
    ('INIT.c', code_dir),
    ('config.h', code_dir),
    ('camera.c', code_dir),
    ('camera.h', code_dir),
    ('control.c', code_dir),
    ('main_cm7_1.c', user_dir),
    ('cm7_0_isr.c', user_dir),
    ('cm7_1_isr.c', user_dir),
    ('of.c', code_dir),
    ('PID.c', code_dir),
    ('filter.c', code_dir),
    ('small_driver_uart_control.c', code_dir),
    ('myuart.c', code_dir),
    ('rc.c', code_dir),
]

for fname, fdir in files:
    chk_path = os.path.join(check_dir, fname)
    cur_path = os.path.join(fdir, fname)
    if not os.path.exists(cur_path):
        print(f'{fname}: CUR FILE NOT FOUND')
        continue
    with open(chk_path, 'rb') as f1, open(cur_path, 'rb') as f2:
        chk_data = f1.read()
        cur_data = f2.read()
        if chk_data == cur_data:
            print(f'{fname}: IDENTICAL')
        else:
            first_diff = -1
            for i in range(min(len(chk_data), len(cur_data))):
                if chk_data[i] != cur_data[i]:
                    first_diff = i
                    break
            print(f'{fname}: DIFFERENT (chk={len(chk_data)}, cur={len(cur_data)}) first_diff_byte={first_diff}')
