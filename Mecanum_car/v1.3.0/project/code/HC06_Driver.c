#include "HC06_Driver.h"

/*****************************************************************������*****************************************************************/
// �Զ��� FIFO ������
#define RX_FIFO_SIZE 64
//��̬��������
static uint8_t rx_buffer[RX_FIFO_SIZE];
// �������������
static char cmd_line[32];
static uint8_t cmd_index = 0;
static volatile char last_cmd = 0;   // ������Чָ��
static fifo_struct rx_fifo;          // ���ԭʼ�ֽڵ� FIFO
// ������ǰ�ٶ�
static float speed_fwd = 0.0f;
static float speed_bwd = 0.0f;
static float speed_left = 0.0f;
static float speed_right = 0.0f;
// ������������ʱ��
static uint64_t time_fwd = 0;
static uint64_t time_bwd = 0;
static uint64_t time_left = 0;
static uint64_t time_right = 0;
static void Update_TargetFromButtons(void);  // ����ԭ��

// �ٶȲ���������
#define SPEED_STEP 0.02f      // ÿ������0.01 m/s
#define SPEED_MAX 0.15f        // ����ٶ� 0.2
#define SPEED_U 0.7                   //˥��ϵ������ֱ���ϵ��ٶȷ������ƽ��
#define CMD_TIMEOUT_US 300000 // ��ʱʱ��100ms
#define HC06_SCB   (SCB2)
// ģʽ�л�
static uint8_t mode = 0;      // 0:ƽ��ģʽ, 1:��תģʽ
static float rot_speed = 0.5f; // ��ת�ٶ� 0.5(rad/s)
static float manual_omega = 0.0f; // �ֶ���ת���ٶ�
volatile uint32_t uart_rx_irq_cnt = 0;

// ������λ�����������˻���
static float pos_error_x = 0.0f;
static float pos_error_y = 0.0f;

// ֡����״̬��
enum FrameState {
    FRAME_WAIT_START,   // �ȴ� '#'
    FRAME_RECV_DATA,    // ���ڽ�������
};
static enum FrameState frame_state = FRAME_WAIT_START;
static char frame_buffer[32];   // �洢֡��ԭʼ�ַ���
static uint8_t frame_idx = 0;

/*****************************************************************������*****************************************************************/

/*****************************************************************************
 * @name       : HC06_UART_RX_Handler
 * @date       : 2026-02-01
 * @function   : ���ڽ����жϴ�������
 * @parameters : ��
 * @retvalue   : ������������/10ms
 * @note       : �ᱻ isr.c ����
******************************************************************************/
// 
void HC06_UART_RX_Handler(void)
{
    uart_rx_irq_cnt++;   // ÿ����һ���жϣ�������1
    uint8_t data;
    // �Ӵ���Ӳ����ȡһ���ֽڣ���������ѯ����Ϊ�Ѿ������жϣ�
    if (uart_query_byte(UART_1, &data)) {
        // д�� FIFO
        fifo_write_buffer(&rx_fifo, &data, 1);
    }
}

/*****************************************************************************
 * @name       : HC06_Init
 * @date       : 2026-02-01
 * @function   : ������ʼ������
 * @parameters : baudrate�������ʣ�һ������ʹ��9600
 * @retvalue   : ��
 * @note       : ��init_all�������
******************************************************************************/
// ��ʼ������
void HC06_Init(uint32 baudrate)
{
    // 1. ��ʼ�� FIFO
    fifo_init(&rx_fifo, FIFO_DATA_8BIT, rx_buffer, RX_FIFO_SIZE);

    // 2. ��ʼ������ 2�����Ź̶�Ϊ P10_5 TX, P10_6 RX��
    uart_init(UART_1, baudrate, UART1_TX_P04_1, UART1_RX_P04_0);

    // 3. ����
    uart_rx_interrupt(UART_1, 1);
    
    //4.���������ж�
    //uart_tx_interrupt(UART_1,1);

    // 5. ���״̬
    cmd_index = 0;
    last_cmd = 0;
}

void ParseFrameData(const char *data, uint16_t len)
{
    // ������������������ʽ��x,y
    float x, y;
    if (sscanf(data, "%f,%f", &x, &y) == 2) {
        pos_error_x = x;
        pos_error_y = y;
        // ��ѡ�����յ�������ӡ��������
        // printf("Pos err: x=%.3f, y=%.3f\n", x, y);
    } else {
        // ����ʧ�ܣ�����
        // printf("Parse frame error: %s\n", data);
    }
}

float GetPositionErrorX(void) { return pos_error_x; }
float GetPositionErrorY(void) { return pos_error_y; }

/*****************************************************************************
 * @name       : HC06_Task
 * @date       : 2026-02-01
 * @function   : �������ݺ�Ĵ�����������������ָ�����
 * @parameters : ��
 * @retvalue   : ��
 * @note       : ��������ѭ���е���
******************************************************************************/
//void HC06_Task(void)
//{
//    uint8_t data;
//    while (fifo_read_element(&rx_fifo, &data, FIFO_READ_AND_CLEAN) == FIFO_SUCCESS) {
//        // ���ԣ�������ԣ�
//        //uart_write_byte(UART_1, data);
//
//        if (data == '!') {
//            // �յ�������������֮ǰ�ۻ�������
//            if (cmd_index > 0) {
//                cmd_line[cmd_index] = '\0';
//                // Ŀǰ����ֻʹ�õ��ַ�ָ�����ȡ��һ���ַ�
//                last_cmd = cmd_line[0];
//                // ��ջ�������׼����һ��ָ��
//                cmd_index = 0;
//            }
//        } else {
//            // ��ͨ�ַ��������л���������ֹ�����
//            if (cmd_index < sizeof(cmd_line) - 1) {
//                cmd_line[cmd_index++] = data;
//            }
//        }
//    }
//    // ���ݰ���״̬����Ŀ���ٶ�
//    Update_TargetFromButtons();
//    static uint64_t last_print_time = 0;
//    if (angle_pid_yaw.Enable) {
//        if (time_us - last_print_time > 100000) { // ÿ100ms��ӡһ��
//            //float yaw = My_MPU6050_GetYaw();
//            float yaw = My_Imu660ra_GetYaw();
//            // ע�⣺angle_pid_yaw.Output ���ϴ�PID��������ֵ����/s��
//            //printf("yaw=%.2f, omega_des=%.2f\n", yaw, angle_pid_yaw.Output);
//            last_print_time = time_us;
//        }
//    }
//}
void HC06_Task(void)
{
    uint8_t data;
    while (fifo_read_element(&rx_fifo, &data, FIFO_READ_AND_CLEAN) == FIFO_SUCCESS) {
        // ========== ֡Э�������# ... $�� ==========
        if (frame_state == FRAME_WAIT_START && data == '#') {
            frame_state = FRAME_RECV_DATA;
            frame_idx = 0;
            continue;
        }
        if (frame_state == FRAME_RECV_DATA) {
            if (data == '$') {
                // һ֡����������
                if (frame_idx > 0) {
                    frame_buffer[frame_idx] = '\0';
                    ParseFrameData(frame_buffer, frame_idx);
                }
                frame_state = FRAME_WAIT_START;
                continue;
            } else {
                // �洢���ݣ���ֹ���
                if (frame_idx < sizeof(frame_buffer)-1) {
                    frame_buffer[frame_idx++] = data;
                }
                continue;
            }
        }

        // 蓝牙单字符: 跳过!终止符, 其余传Handler
        if (data == '!') continue;
        Bluetooth_Command_Handler((char)data);
        continue;
//        // ========== ԭ�е��ַ�ָ��������� '!' ������ ==========
//        if (data == '!') {
//            if (cmd_index > 0) {
//                cmd_line[cmd_index] = '\0';
//                last_cmd = cmd_line[0];
//                cmd_index = 0;
//            }
//        } else {
//            if (cmd_index < sizeof(cmd_line)-1) {
//                cmd_line[cmd_index++] = data;
//            }
//        }
    }
    
    Update_TargetFromButtons();
    if (!go_center) { if (fabsf(target_vx) > 0.001f || fabsf(target_vy) > 0.001f) { bcn_nav_vx = target_vx; bcn_nav_vy = target_vy; bcn_nav_angle = 0.0f; bcn_nav_on = 1; } else { bcn_nav_on = 0; } }
    // Update_TargetFromButtons();
//    static uint64_t last_print_time = 0;
//    if (angle_pid_yaw.Enable) {
//        if (time_us - last_print_time > 100000) { // ÿ100ms��ӡһ��
//            //float yaw = My_MPU6050_GetYaw();
//            float yaw = My_Imu660ra_GetYaw();
//            // ע�⣺angle_pid_yaw.Output ���ϴ�PID��������ֵ����/s��
//            //printf("yaw=%.2f, omega_des=%.2f\n", yaw, angle_pid_yaw.Output);
//            last_print_time = time_us;
//        }
//    }
}

void HC06_SendDroneCmd(uint8_t cmd)
{
    uart_write_byte(UART_1, '#');
    uart_write_byte(UART_1, '0' + cmd);
    uart_write_byte(UART_1, '$');
    // �ȴ�������ɣ����ݲ����ʣ�3�ֽ�Լ�� 3ms��
}
//
//// ���������͵��ֽ�
//static void hc06_tx_byte(uint8 data)
//{
//    volatile stc_SCB_t *scb = HC06_SCB;
//    uint32 timeout = 5000;
//    while (Cy_SCB_GetNumInTxFifo(scb) >= 16)  // �ȴ� FIFO �п�λ
//    {
//        if (--timeout == 0) return;
//    }
//    Cy_SCB_WriteTxFifo(scb, data);
//}
//
//// �����ַ���
//static void hc06_tx_string(const char *str)
//{
//    while (*str)
//    {
//        hc06_tx_byte(*str++);
//    }
//}
//
//// �����еķ�����������޸�Ϊʹ�� hc06_tx_string��
//void HC06_SendDroneCmd(uint8_t cmd)
//{
//    // ��ʽ��#1$  #2$  #3$
//    char buf[4] = {'#', '0' + cmd, '$', '\0'};   // �����ַ���
//    hc06_tx_string(buf);
//}

/*****************************************************************************
 * @name       : HC06_GetCmd
 * @date       : 2026-02-01
 * @function   : ��ȡ����ָ���ȡ���Զ����㣩
 * @parameters : ��
 * @retvalue   : ��
 * @note       : ��������ѭ����ʹ��
******************************************************************************/
char HC06_GetCmd(void)
{
    char cmd = last_cmd;
    last_cmd = 0;
    return cmd;
}

/*****************************************************************************
 * @name       : Process_Bluetooth_Command
 * @date       : 2026-02-01
 * @function   : ��������ָ��������ݽ�����������ָ��ٿ�С��
 * @parameters : cmd�������������ָ���ַ�
 * @retvalue   : ��
 * @note       : ��������ѭ����ʹ��
******************************************************************************/
void Bluetooth_Command_Handler(char cmd)
{
    uint64_t now = time_us;

    if (cmd == 'F') { // ǰ��
        speed_fwd += SPEED_STEP;
        if (speed_fwd > SPEED_U * SPEED_MAX) speed_fwd = SPEED_U * SPEED_MAX;
        time_fwd = now;
    }
    else if (cmd == 'B') { // ����
        speed_bwd += SPEED_STEP;
        if (speed_bwd > SPEED_MAX) speed_bwd = SPEED_MAX;
        time_bwd = now;
    }
    else if (cmd == 'L') { // ���ƣ�����ת��
        speed_left += SPEED_STEP;
        if (speed_left > SPEED_U * SPEED_MAX) speed_left = SPEED_U * SPEED_MAX;
        time_left = now;
    }
    else if (cmd == 'R') { // ���ƣ�����ת��
        speed_right += SPEED_STEP;
        if (speed_right > SPEED_MAX) speed_right = SPEED_MAX;
        time_right = now;
    }
    else if (cmd == 'M') { speed_fwd = speed_bwd = speed_left = speed_right = 0.0f; go_center = 1; printf("GO CENTER!\n"); }
//    else if (cmd == 'M') {   // 原模式切换已废弃, 改为上面go_center
//        mode = !mode;
//        if (mode == 0) {
//            // �л�ƽ��ģʽʱ������ǰyaw��Ϊ����0��
////            angle_target = My_MPU6050_GetYaw();
//            angle_target = My_Imu660ra_GetYaw();
//        }
//        // �л�ģʽʱ��������ٶȣ���ѡ��
//        speed_fwd = speed_bwd = speed_left = speed_right = 0.0f;
    }
//    else if (cmd == 'X') {
//        balance_mode = !balance_mode;
//        target_vx = 0; target_vy = 0;
//        manual_omega = 0;
//        if (balance_mode) {
//            pitch_offset = My_Imu660ra_GetPitch();
//            roll_offset = My_Imu660ra_GetRoll();
////            pitch_offset = My_MPU6050_GetPitch();
////            roll_offset = My_MPU6050_GetRoll();
//            // �� printf ��ӡƫ������ȷ���Ƿ����
//            //printf("pitch_offset=%.2f, roll_offset=%.2f\n", pitch_offset, roll_offset);
//            //uart_write_string(UART_2, "Balance mode ON\r\n");
//        } else {
//            //uart_write_string(UART_2, "Balance mode OFF\r\n");
//        }
//    }

/*****************************************************************************
 * @name       : Get_ManualOmega
 * @date       : 2026-02-01
 * @function   : ��ȡ����������Ŀ��ƽǶ�
 * @parameters : ��
 * @retvalue   : ��
 * @note       : ���жϺ�����Ŀ��Ƕȿ��Ƽ��ϸýǶ�
******************************************************************************/
float Get_ManualOmega(void)
{
    return manual_omega;
}
/*****************************************************************�ڲ����ú�����*****************************************************************/

/*****************************************************************************
 * @name       : Check_Speed_Timeout
 * @date       : 2026-02-01
 * @function   : ��ʱ��⺯������⵽��ʱ�Ļ������ٶȱ���Ϊ0
 * @parameters : ��
 * @retvalue   : ��
 * @note       : �ڱ��ļ��ڲ�����
******************************************************************************/
static void Check_Speed_Timeout(void)
{
    uint64_t now = time_us;
    if (now - time_fwd > CMD_TIMEOUT_US) speed_fwd = 0.0f;
    if (now - time_bwd > CMD_TIMEOUT_US) speed_bwd = 0.0f;
    if (now - time_left > CMD_TIMEOUT_US) speed_left = 0.0f;
    if (now - time_right > CMD_TIMEOUT_US) speed_right = 0.0f;
}

/*****************************************************************************
 * @name       : Update_TargetFromButtons
 * @date       : 2026-02-01
 * @function   : ����ָ��ӳ����ƺ�������ָ��ת��ΪĿ���ٶ�/�Ƕ�
 * @parameters : ��
 * @retvalue   : ��
 * @note       : �ڱ��ļ��ڲ�����
******************************************************************************/
static void Update_TargetFromButtons(void)
{
    Check_Speed_Timeout();

    if (mode == 0) { // ƽ��ģʽ
        // ����ǰ��ϳ��ٶ� vx
        if (speed_fwd > speed_bwd) {
            target_vx = speed_fwd - speed_bwd;
        } else {
            target_vx = - (speed_bwd - speed_fwd);
        }
        // �������Һϳ��ٶ� vy
        if (speed_right > speed_left) {
            target_vy = speed_right - speed_left;
        } else {
            target_vy = - (speed_left - speed_right);
        }
        manual_omega = 0.0f; // ƽ��ģʽ���ֶ���ת
    }
    else { // ��תģʽ
        target_vx = 0.0f;
        target_vy = 0.0f;

        // ��̬���½ǶȻ�Ŀ��Ϊ��ǰyaw������ǶȻ�����
        angle_target = My_Imu660ra_GetYaw();
        //angle_target = My_MPU6050_GetYaw();   // ����
        // ���������ٶȲ������ת���ٶ�
        float diff = speed_right - speed_left;
        manual_omega = diff * rot_speed / SPEED_MAX; // ӳ�䵽��ת�ٶ�
        // �޷�
        if (manual_omega > rot_speed) manual_omega = rot_speed;
        if (manual_omega < -rot_speed) manual_omega = -rot_speed;
    }
}
//
//void Process_Bluetooth_Command(char cmd)
//{
//    switch (cmd) {
//        case 'e': // ���ÿ���
//            if (mpu6050_ready) {
//                Motor_Enable_PID(1);
//                PID_Reset(&angle_pid_yaw);
//                PID_Enable(&angle_pid_yaw, 1);
//                My_MPU6050_ResetYaw();
//                target_vx = 0.0f; target_vy = 0.0f;
//                printf("Angle control enabled\n");
//            } else {
//                // ��ʾ������δ����
//                printf("MPU6050 not ready\n");
//            }
//            break;
//        case 'v':
//            Motor_Enable_PID(1);
//            // �����ýǶȻ�
//            PID_Enable(&angle_pid_yaw, 0);
//            target_vx = 0.2f; target_vy = 0.0f;
//            printf("speed mode: vx=0.2\n");
//            break;
//        case 'd': // ���ÿ���
//            Motor_Enable_PID(0);
//            PID_Enable(&angle_pid_yaw, 0);
//            target_vx = 0.0f; target_vy = 0.0f;
//            // ǿ��ͣ������ο�������ֹͣ��ʽ��
//            Motor_SetSpeed(&motor_L1, 0, MotorL1_Turn, MotorL1_Pwm);
//            Motor_SetSpeed(&motor_L2, 0, MotorL2_Turn, MotorL2_Pwm);
//            Motor_SetSpeed(&motor_R1, 0, MotorR1_Turn, MotorR1_Pwm);
//            Motor_SetSpeed(&motor_R2, 0, MotorR2_Turn, MotorR2_Pwm);
//            printf("PID disabled, motors stopped\n");
//            break;
//        case 'f': // ǰ��
//            target_vx = 0.2f; target_vy = 0.0f;
//            printf("target: vx=0.2, vy=0\n");
//            break;
//        case 'b': // ����
//            target_vx = -0.2f; target_vy = 0.0f; break;
//        case 'l': // ����
//            target_vx = 0.0f; target_vy = -0.2f; break;
//        case 'r': // ����
//            target_vx = 0.0f; target_vy = 0.2f; break;
//        case 's': // ֹͣ
//            target_vx = 0.0f; target_vy = 0.0f;
//            Motor_SetSpeed(&motor_L1, 0, MotorL1_Turn, MotorL1_Pwm);
//            Motor_SetSpeed(&motor_L2, 0, MotorL2_Turn, MotorL2_Pwm);
//            Motor_SetSpeed(&motor_R1, 0, MotorR1_Turn, MotorR1_Pwm);
//            Motor_SetSpeed(&motor_R2, 0, MotorR2_Turn, MotorR2_Pwm);
//            printf("target: (0,0)\n");
//            break;
//        default:
//            break;
//    }
//}
