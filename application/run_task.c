
#include "run_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "CAN_receive.h"
#include "chassis_task.h"
#include "INS_task.h"
#include "gray_sensor.h"
#include "ir_sensor.h"
#include "voice_module.h"
#include "bsp_servo_pwm.h"
#include "actions.h"
#include "pid.h"

/* �Ӿ�ɨ�� (actions.c, �� include actions.h ���� turn ��ͻ) */
#define get_yaw    INS_angle_go[0]*57.29578f
#define get_pitch  INS_angle_go[1]*57.29578f
#define get_roll   INS_angle_go[2]*57.29578f
#define sensor data_storage
#define STOP  stop(); while(1) osDelay(1);

#define get_rount_cnt chassis_round_cnt()

/* �Ҷ�ȫ����ֵ (R4/R5 ��������) */
#define R4_BLACK_CNT 14
#define R5_BLACK_CNT 14

/* ---- �� 407 ������������ (�� sensor_update()/�Ӿ���ѯ ���) ---- */
uint8_t data_storage[27] = {1};
uint8_t hui[16] = {0};
uint16_t HUI_data = 0;
uint8_t pt1 = 0, pt2 = 0, pt3 = 0;
uint8_t hld = 0x30;
uint8_t zsp = 0x31;
uint8_t xiansuo1 = 0, xiansuo2 = 0x32; /* ��� 0x32: xiansuo ��֧�ߵ�һ�� */
;

/* ---- ȫ�� (run_task.h extern) ---- */
const fp32* INS_angle_go;
fp32 go_yaw, go_pitch, go_yaw_inia;
int16_t v_r, v_l, V, v_delt;
int16_t set_speed_rpm[4] = {0, 0, 0, 0};
fp32 vx_run_set = 0.0f, angle_run_set = 0.0f;
fp32 angle_sum = 0;
fp32 speed_sum = 0;
fp32 biass = 0;
int last_r = 0;
int intia_rount_cnt = 0;

/* ---- ������ˢ��: �Ҷ� + ���� -> ���������� ---- */
extern void sensor_update(void)
{
    gray_sensor_poll();
    const gray_sensor_t* gs = get_gray_sensor_point();
    uint16_t bm = gs->bitmask;
    HUI_data = (uint16_t)(~bm);
    for (int i = 0; i < 16; i++)
        hui[i] = ((bm >> i) & 1) ? 0 : 1;

    ir_sensor_poll();
    const ir_sensor_t* ir = get_ir_sensor_point();
    data_storage[12] = ir->front;
    data_storage[13] = ir->left;
    data_storage[14] = ir->right;
    data_storage[15] = (gs->line_count >= R4_BLACK_CNT) ? 0 : 1;
    data_storage[16] = (gs->line_count >= R5_BLACK_CNT) ? 0 : 1;
}

/* ---- PID ѭ�� (�ο� master line_track) ---- */
#define TRACK_PID_KP 3.0f
#define TRACK_PID_KI 0.001f
#define TRACK_PID_KD 0.4f
#define TRACK_PID_MAX_WZ   4.0f
#define TRACK_PID_MAX_IOUT 0.3f
static pid_type_def track_pid;

void trackxian(fp32 vx)
{
    sensor_update();
    const gray_sensor_t* gs = get_gray_sensor_point();
    if (gs->line_count >= 4)
    {
        chassis_set_velocity(vx, 0.0f);
        return;
    }
    PID_calc(&track_pid, -gs->position, 0.0f);
    chassis_set_velocity(vx, track_pid.out);
}

fp32 rex_abs(fp32 a)
{
    if (a < 0)
        a = -a;
    return a;
}


void stop(void);

void set_current(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

void turn_run(int speed, fp32 angle);
void turn2(int speed, fp32 angle);
void turnleft(void);
void turnright(void);
int lukou_detect(int det);
void xpt(void);
void go12(void);
void go24(void);
void go34(void);
void go4men1(void);
void go4men2(void);
void go4men3(void);
void go4men4(void);
void gomen1_2(void);
void gomen2_3(void);
void gomen3_4(void);
void gomen3_men4_5(void);
void gomen15(void);
void gomen25(void);
void gomen35(void);
void gomen45(void);
void go57(void);
void go78(void);
void go8zhi(void);
void gozhimen1(void);
void gozhimen2(void);
void gozhimen3(void);
void gozhimen4(void);
void gomen11(void);
void gomen21(void);
void gomen31(void);
void gomen41(void);
void go3men1(void);
void gozhimen33(void);
void go43(void);
void go36(void);
void go35(void);
void go45(void);
void go46(void);
void go315(void);
void go325(void);
void go335(void);
void go345(void);
void go3men15(void);
void go3men25(void);
void go3men35(void);
void go3men45(void);
void go3men16(void);
void go3men26(void);
void go3men36(void);
void go3men46(void);
void go25(void);
void go26(void);
void gozhimen43(void);
void gomen16(void);
void gomen26(void);
void gomen36(void);
void gomen46(void);
void go57(void);
void go76(void);
void go75(void);
void go7men14(void);
void go7men13(void);
void go7men12(void);
void go7men24(void);
void go7men23(void);
void go7men22(void);
void go7men34(void);
void go7men33(void);
void go7men32(void);
void go7men44(void);
void go7men43(void);
void go7men42(void);
void go58(void);
void go86(void);
void go85(void);
void go8men14(void);
void go8men13(void);
void go8men12(void);
void go8men24(void);
void go8men23(void);
void go8men22(void);
void go8men34(void);
void go8men33(void);
void go8men32(void);
void go8men44(void);
void go8men43(void);
void go8men42(void);
void go67(void);
void go68(void);
void go6men11(void);
void go6men21(void);
void go6men31(void);
void go6men41(void);
void go5men11(void);
void go5men21(void);
void go5men31(void);
void go5men41(void);
void go21(void);
void go31(void);
void go41(void);
void go7men11(void);
void go7men21(void);
void go7men31(void);
void go7men41(void);
void go23(void);
void go4men35(void);
void go4men1(void);
void downplat(void);
void gohill(void);
void gobri(void);
void gobri1(void);
void goqqb(void);
void set0rount(void);
void gomen1jia(void);
void go517(void);
void go78(void);
void go6men1jia(void);
void xingomen1jia(void);
void xingomen2jia(void);
void xingo4men4jia(void);
void xingo4men3jia(void);
void go4men4jia(void);
void go4men3jia(void);
void go3men2jia(void);
void xingo3men2jia(void);
void goplat(void);
void gohome(void);
void gomen2jia(void);
void go6men2jia(void);
void go4men45(void);
void go6men3jia(void);
void go6men4jia(void);
void go8zhi(void);
void gozhimen1jia(void);
void gozhimen2jia(void);
void gozhimen3jia(void);
void gozhimen4jia(void);
void go4zhi3(void);
void gomen3jia(void);
void wave_hand(void);
void yuyin(int a);
void trackxian1(void);
void trackxian2(void);
void trackxian3(void);
void trackxian5(void);
void trackxian8(void);
void trackxian12(void);
void trackxian15(void);
void go6men43(void);
void go3jia(void);
void go6men33(void);
void xingomen3jia(void);
void gozhimen13(void);
void gozhimen24(void);

void setspeed2(int speed1, int speed2)
{
    set_current(speed1, speed2, speed1, speed2);
}
void setspeed78(int speed1, int speed2)
{
    set_current(speed1, speed2, speed1-1000, speed2-1000);
}

void turnleft()
{
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1000, 8000, -1000, 8000);
            osDelay(1);
        }
    }
}

void turnright()
{
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(8000, -1000, 8000, -1000);
            osDelay(1);
        }
    }
}

void up_hand(void);

void gohome()
{
    while (get_roll < 15)
        trackxian5();

    while (get_roll > 2)
        trackxian5();


    setspeed2(3000, 3000);
    osDelay(300);
    stop();
    osDelay(200);

    turn_run(5000, 165);

    yuyin(9);
    stop();
    osDelay(200);
}

int lukou_detect(int det) //��⵽����det�������жϵ���·�ڣ���ʱ10ms���ж���ȻΪ·���� ����1 ���򷵻�0
{
    int num = 0;
    int lukou_num = 0;
    for (int i = 0; i < 2; i++) //ѭ�����·��2��
    {
        for (int j = 0; j < 16; j++)
        {
            if (hui[j] == 0) //�жϸ��ֽ��к��С�0���ĸ���
                num++;
        }
        if (num >= (det)) //���ڵ���5����ѹ����Ϊ�ǵ���·��
        {
            lukou_num++;
            if (lukou_num == 1) osDelay(10); //��һ�μ�⵽��ʱ10ms����������
        }
        num = 0;
    }
    if (lukou_num >= 2) return 1; //���2�ζ���·�ں����Ϊ����������·�ڣ���ֹ����

    return 0;
}

void trackxian1()
{
    sensor_update();

    switch (HUI_data) //1111111
    {
    //65432109 87654321
    case 0xFE7F: setspeed2(1000, 1000);
        break; //11111110 01111111
    case 0xFC7F: setspeed2(1000, 500);
        break; //11111100 01111111
    case 0xFE3F: setspeed2(500, 1000);
        break; //11111110 00111111
    case 0xFEFF: setspeed2(1000, 500);
        break; //11111110 11111111
    case 0xFCFF: setspeed2(1000, 500);
        break; //11111100 11111111
    case 0xFF7F: setspeed2(500, 1000);
        break; //11111111 01111111
    case 0xFF3F: setspeed2(500, 1000);
        break; //11111111 00111111
    case 0xF8FF: setspeed2(1000, 500);
        break; //11111000 11111111
    case 0xFF1F: setspeed2(500, 1000);
        break; //11111111 00011111

    case 0xFDFF: setspeed2(1000, 0);
        break; //11111101 11111111
    case 0xF9FF: setspeed2(1000, 0);
        break; //11111001 11111111
    case 0xFFBF: setspeed2(0, 1000);
        break; //11111111 10111111
    case 0xFF9F: setspeed2(0, 1000);
        break; //11111111 10011111
    case 0xF1FF: setspeed2(1000, 0);
        break; //11110001 11111111
    case 0xFF8F: setspeed2(0, 1000);
        break; //11111111 10001111

    case 0xFBFF: setspeed2(1000, 0);
        break; //11111011 11111111
    case 0xF3FF: setspeed2(1000, 0);
        break; //11110011 11111111
    case 0xFFDF: setspeed2(0, 1000);
        break; //11111111 11011111
    case 0xFFCF: setspeed2(0, 1000);
        break; //11111111 11001111
    case 0xE3FF: setspeed2(1000, 0);
        break; //11100011 11111111
    case 0xFFC7: setspeed2(0, 1000);
        break; //11111111 11000111

    case 0xF7FF: setspeed2(1000, 0);
        break; //11110111 11111111
    case 0xE7FF: setspeed2(1000, 0);
        break; //11100111 11111111
    case 0xFFEF: setspeed2(0, 1000);
        break; //11111111 11101111
    case 0xFFE7: setspeed2(0, 1000);
        break; //11111111 11100111
    case 0xC7FF: setspeed2(1000, 0);
        break; //11000111 11111111
    case 0xFFE3: setspeed2(0, 1000);
        break; //11111111 11100011

    case 0xEFFF: setspeed2(1000, 0);
        break; //11101111 11111111
    case 0xCFFF: setspeed2(1000, 0);
        break; //11001111 11111111
    case 0xFFF7: setspeed2(0, 1000);
        break; //11111111 11110111
    case 0xFFF3: setspeed2(0, 1000);
        break; //11111111 11110011
    case 0x8FFF: setspeed2(1000, 0);
        break; //10001111 11111111
    case 0xFFF1: setspeed2(0, 1000);
        break; //11111111 11110001

    case 0xFFFB: setspeed2(0, 1000);
        break; //11111111 11111011
    case 0xFFF9: setspeed2(0, 1000);
        break; //11111111 11111001
    case 0xDFFF: setspeed2(1000, 0);
        break; //11011111 11111111
    case 0x9FFF: setspeed2(1000, 0);
        break; //10011111 11111111
    case 0xFFF8: setspeed2(0, 1000);
        break; //11111111 11111000
    case 0x1FFF: setspeed2(1000, 0);
        break; //00011111 11111111

    case 0xFFFD: setspeed2(0, 1000);
        break; //11111111 11111101
    case 0xFFFC: setspeed2(0, 1000);
        break; //11111111 11111100
    case 0xBFFF: setspeed2(1000, 0);
        break; //10111111 11111111
    case 0x3FFF: setspeed2(1000, 0);
        break; //00111111 11111111
    case 0xFFFE: setspeed2(0, 1000);
        break; //11111111 11111110
    case 0x7FFF: setspeed2(1000, 0);
        break; //01111111 11111111
    default: setspeed2(1000, 1000);
        break;
    }
}

///0=??????   1?????  2=????  3=??? 4=???  5=????  6=???  7=???  8=???????  9=???


void trackxian3()
{
    sensor_update();

    switch (HUI_data) //1111111
    {
    //65432109 87654321
    case 0xFE7F: setspeed2(3000, 3000);
        break; //11111110 01111111
    case 0xFC7F: setspeed2(3000, 2500);
        break; //11111100 01111111
    case 0xFE3F: setspeed2(2500, 3000);
        break; //11111110 00111111
    case 0xFEFF: setspeed2(3000, 2500);
        break; //11111110 11111111
    case 0xFCFF: setspeed2(3000, 2500);
        break; //11111100 11111111
    case 0xFF7F: setspeed2(2500, 3000);
        break; //11111111 01111111
    case 0xFF3F: setspeed2(2500, 3000);
        break; //11111111 00111111
    case 0xF8FF: setspeed2(3000, 2500);
        break; //11111000 11111111
    case 0xFF1F: setspeed2(2500, 3000);
        break; //11111111 00011111

    case 0xFDFF: setspeed2(3000, 2000);
        break; //11111101 11111111
    case 0xF9FF: setspeed2(3000, 2000);
        break; //11111001 11111111
    case 0xFFBF: setspeed2(2000, 3000);
        break; //11111111 10111111
    case 0xFF9F: setspeed2(2000, 3000);
        break; //11111111 10011111
    case 0xF1FF: setspeed2(3000, 2000);
        break; //11110001 11111111
    case 0xFF8F: setspeed2(2000, 3000);
        break; //11111111 10001111

    case 0xFBFF: setspeed2(3000, 1500);
        break; //11111011 11111111
    case 0xF3FF: setspeed2(3000, 1500);
        break; //11110011 11111111
    case 0xFFDF: setspeed2(1500, 3000);
        break; //11111111 11011111
    case 0xFFCF: setspeed2(1500, 3000);
        break; //11111111 11001111
    case 0xE3FF: setspeed2(3000, 1500);
        break; //11100011 11111111
    case 0xFFC7: setspeed2(1500, 3000);
        break; //11111111 11000111

    case 0xF7FF: setspeed2(3000, 1000);
        break; //11110111 11111111
    case 0xE7FF: setspeed2(3000, 1000);
        break; //11100111 11111111
    case 0xFFEF: setspeed2(1000, 3000);
        break; //11111111 11101111
    case 0xFFE7: setspeed2(1000, 3000);
        break; //11111111 11100111
    case 0xC7FF: setspeed2(3000, 1000);
        break; //11000111 11111111
    case 0xFFE3: setspeed2(1000, 3000);
        break; //11111111 11100011

    case 0xEFFF: setspeed2(3000, 500);
        break; //11101111 11111111
    case 0xCFFF: setspeed2(3000, 500);
        break; //11001111 11111111
    case 0xFFF7: setspeed2(500, 3000);
        break; //11111111 11110111
    case 0xFFF3: setspeed2(500, 3000);
        break; //11111111 11110011
    case 0x8FFF: setspeed2(3000, 500);
        break; //10001111 11111111
    case 0xFFF1: setspeed2(500, 3000);
        break; //11111111 11110001

    case 0xFFFB: setspeed2(0, 3000);
        break; //11111111 11111011
    case 0xFFF9: setspeed2(0, 3000);
        break; //11111111 11111001
    case 0xDFFF: setspeed2(3000, 0);
        break; //11011111 11111111
    case 0x9FFF: setspeed2(3000, 0);
        break; //10011111 11111111
    case 0xFFF8: setspeed2(0, 3000);
        break; //11111111 11111000
    case 0x1FFF: setspeed2(3000, 0);
        break; //00011111 11111111

    case 0xFFFD: setspeed2(0, 3000);
        break; //11111111 11111101
    case 0xFFFC: setspeed2(0, 3000);
        break; //11111111 11111100
    case 0xBFFF: setspeed2(3000, 0);
        break; //10111111 11111111
    case 0x3FFF: setspeed2(3000, 0);
        break; //00111111 11111111
    case 0xFFFE: setspeed2(0, 3000);
        break; //11111111 11111110
    case 0x7FFF: setspeed2(3000, 0);
        break; //01111111 11111111
    default: setspeed2(3000, 3000);
        break;
    }
}


void trackxian2()
{
    sensor_update();

    switch (HUI_data) //1111111
    {
    //65432109 87654321
    case 0xFE7F: setspeed2(2000, 2000);
        break; //11111110 01111111
    case 0xFC7F: setspeed2(2000, 1500);
        break; //11111100 01111111
    case 0xFE3F: setspeed2(1500, 2000);
        break; //11111110 00111111
    case 0xFEFF: setspeed2(2000, 1500);
        break; //11111110 11111111
    case 0xFCFF: setspeed2(2000, 1500);
        break; //11111100 11111111
    case 0xFF7F: setspeed2(1500, 2000);
        break; //11111111 01111111
    case 0xFF3F: setspeed2(1500, 2000);
        break; //11111111 00111111
    case 0xF8FF: setspeed2(2000, 1500);
        break; //11111000 11111111
    case 0xFF1F: setspeed2(1500, 2000);
        break; //11111111 00011111

    case 0xFDFF: setspeed2(2000, 1000);
        break; //11111101 11111111
    case 0xF9FF: setspeed2(2000, 1000);
        break; //11111001 11111111
    case 0xFFBF: setspeed2(1000, 2000);
        break; //11111111 10111111
    case 0xFF9F: setspeed2(1000, 2000);
        break; //11111111 10011111
    case 0xF1FF: setspeed2(2000, 1000);
        break; //11110001 11111111
    case 0xFF8F: setspeed2(1000, 2000);
        break; //11111111 10001111

    case 0xFBFF: setspeed2(2000, 500);
        break; //11111011 11111111
    case 0xF3FF: setspeed2(2000, 500);
        break; //11110011 11111111
    case 0xFFDF: setspeed2(500, 2000);
        break; //11111111 11011111
    case 0xFFCF: setspeed2(500, 2000);
        break; //11111111 11001111
    case 0xE3FF: setspeed2(2000, 500);
        break; //11100011 11111111
    case 0xFFC7: setspeed2(500, 2000);
        break; //11111111 11000111

    case 0xF7FF: setspeed2(2000, 500);
        break; //11110111 11111111
    case 0xE7FF: setspeed2(2000, 500);
        break; //11100111 11111111
    case 0xFFEF: setspeed2(500, 2000);
        break; //11111111 11101111
    case 0xFFE7: setspeed2(500, 2000);
        break; //11111111 11100111
    case 0xC7FF: setspeed2(2000, 500);
        break; //11000111 11111111
    case 0xFFE3: setspeed2(500, 2000);
        break; //11111111 11100011

    case 0xEFFF: setspeed2(2000, 500);
        break; //11101111 11111111
    case 0xCFFF: setspeed2(2000, 500);
        break; //11001111 11111111
    case 0xFFF7: setspeed2(500, 2000);
        break; //11111111 11110111
    case 0xFFF3: setspeed2(500, 2000);
        break; //11111111 11110011
    case 0x8FFF: setspeed2(2000, 500);
        break; //10001111 11111111
    case 0xFFF1: setspeed2(500, 2000);
        break; //11111111 11110001

    case 0xFFFB: setspeed2(0, 2000);
        break; //11111111 11111011
    case 0xFFF9: setspeed2(0, 2000);
        break; //11111111 11111001
    case 0xDFFF: setspeed2(2000, 0);
        break; //11011111 11111111
    case 0x9FFF: setspeed2(2000, 0);
        break; //10011111 11111111
    case 0xFFF8: setspeed2(0, 2000);
        break; //11111111 11111000
    case 0x1FFF: setspeed2(2000, 0);
        break; //00011111 11111111

    case 0xFFFD: setspeed2(0, 2000);
        break; //11111111 11111101
    case 0xFFFC: setspeed2(0, 2000);
        break; //11111111 11111100
    case 0xBFFF: setspeed2(2000, 0);
        break; //10111111 11111111
    case 0x3FFF: setspeed2(2000, 0);
        break; //00111111 11111111
    case 0xFFFE: setspeed2(0, 2000);
        break; //11111111 11111110
    case 0x7FFF: setspeed2(2000, 0);
        break; //01111111 11111111
    default: setspeed2(2000, 2000);
        break;
    }
}

///0=??????   1?????  2=????  3=??? 4=???  5=????  6=???  7=???  8=???????  9=???
void trackxian5()
{
    sensor_update();

    switch (HUI_data) //1111111
    {
    //65432109 87654321
    case 0xFE7F: setspeed2(5000, 5000);
        break; //11111110 01111111		//���м�λ��
    case 0xFC7F: setspeed2(5000, 4000);
        break; //11111100 01111111
    case 0xFE3F: setspeed2(4000, 5000);
        break; //11111110 00111111
    case 0xFEFF: setspeed2(5000, 3500);
        break; //11111110 11111111
    case 0xFCFF: setspeed2(5000, 3500);
        break; //11111100 11111111
    case 0xFF7F: setspeed2(3500, 5000);
        break; //11111111 01111111
    case 0xFF3F: setspeed2(3500, 5000);
        break; //11111111 00111111
    case 0xF8FF: setspeed2(5000, 3500);
        break; //11111000 11111111
    case 0xFF1F: setspeed2(3500, 5000);
        break; //11111111 00011111

    case 0xFDFF: setspeed2(5000, 3000);
        break; //11111101 11111111
    case 0xF9FF: setspeed2(5000, 3000);
        break; //11111001 11111111
    case 0xFFBF: setspeed2(3000, 5000);
        break; //11111111 10111111
    case 0xFF9F: setspeed2(3000, 5000);
        break; //11111111 10011111
    case 0xF1FF: setspeed2(5000, 3000);
        break; //11110001 11111111
    case 0xFF8F: setspeed2(3000, 5000);
        break; //11111111 10001111

    case 0xFBFF: setspeed2(5000, 3000);
        break; //11111011 11111111
    case 0xF3FF: setspeed2(5000, 3000);
        break; //11110011 11111111
    case 0xFFDF: setspeed2(3000, 5000);
        break; //11111111 11011111
    case 0xFFCF: setspeed2(3000, 5000);
        break; //11111111 11001111
    case 0xE3FF: setspeed2(5000, 3000);
        break; //11100011 11111111
    case 0xFFC7: setspeed2(3000, 5000);
        break; //11111111 11000111

    case 0xF7FF: setspeed2(5000, 2000);
        break; //11110111 11111111
    case 0xE7FF: setspeed2(5000, 2000);
        break; //11100111 11111111
    case 0xFFEF: setspeed2(2000, 5000);
        break; //11111111 11101111
    case 0xFFE7: setspeed2(2000, 5000);
        break; //11111111 11100111
    case 0xC7FF: setspeed2(5000, 2000);
        break; //11000111 11111111
    case 0xFFE3: setspeed2(2000, 5000);
        break; //11111111 11100011

    case 0xEFFF: setspeed2(5000, 2000);
        break; //11101111 11111111
    case 0xCFFF: setspeed2(5000, 2000);
        break; //11001111 11111111
    case 0xFFF7: setspeed2(2000, 5000);
        break; //11111111 11110111
    case 0xFFF3: setspeed2(2000, 5000);
        break; //11111111 11110011
    case 0x8FFF: setspeed2(5000, 2000);
        break; //10001111 11111111
    case 0xFFF1: setspeed2(2000, 5000);
        break; //11111111 11110001

    case 0xFFFB: setspeed2(1000, 5000);
        break; //11111111 11111011
    case 0xFFF9: setspeed2(1000, 5000);
        break; //11111111 11111001
    case 0xDFFF: setspeed2(5000, 1000);
        break; //11011111 11111111
    case 0x9FFF: setspeed2(5000, 1000);
        break; //10011111 11111111
    case 0xFFF8: setspeed2(1000, 5000);
        break; //11111111 11111000
    case 0x1FFF: setspeed2(5000, 1000);
        break; //00011111 11111111

    case 0xFFFD: setspeed2(0, 5000);
        break; //11111111 11111101
    case 0xFFFC: setspeed2(0, 5000);
        break; //11111111 11111100
    case 0xBFFF: setspeed2(5000, 0);
        break; //10111111 11111111
    case 0x3FFF: setspeed2(5000, 0);
        break; //00111111 11111111
    case 0xFFFE: setspeed2(0, 5000);
        break; //11111111 11111110
    case 0x7FFF: setspeed2(5000, 0);
        break; //01111111 11111111

    default: setspeed2(5000, 5000);
        break;
    }
}
void trackxian78()
{
    sensor_update();

    switch (HUI_data) //1111111
    {
    //65432109 87654321
    case 0xFE7F: setspeed78(5000, 5000);
        break; //11111110 01111111		//���м�λ��
    case 0xFC7F: setspeed78(5000, 4000);
        break; //11111100 01111111
    case 0xFE3F: setspeed78(4000, 5000);
        break; //11111110 00111111
    case 0xFEFF: setspeed78(5000, 3500);
        break; //11111110 11111111
    case 0xFCFF: setspeed78(5000, 3500);
        break; //11111100 11111111
    case 0xFF7F: setspeed78(3500, 5000);
        break; //11111111 01111111
    case 0xFF3F: setspeed78(3500, 5000);
        break; //11111111 00111111
    case 0xF8FF: setspeed78(5000, 3500);
        break; //11111000 11111111
    case 0xFF1F: setspeed78(3500, 5000);
        break; //11111111 00011111

    case 0xFDFF: setspeed78(5000, 3000);
        break; //11111101 11111111
    case 0xF9FF: setspeed78(5000, 3000);
        break; //11111001 11111111
    case 0xFFBF: setspeed78(3000, 5000);
        break; //11111111 10111111
    case 0xFF9F: setspeed78(3000, 5000);
        break; //11111111 10011111
    case 0xF1FF: setspeed78(5000, 3000);
        break; //11110001 11111111
    case 0xFF8F: setspeed78(3000, 5000);
        break; //11111111 10001111

    case 0xFBFF: setspeed78(5000, 3000);
        break; //11111011 11111111
    case 0xF3FF: setspeed78(5000, 3000);
        break; //11110011 11111111
    case 0xFFDF: setspeed78(3000, 5000);
        break; //11111111 11011111
    case 0xFFCF: setspeed78(3000, 5000);
        break; //11111111 11001111
    case 0xE3FF: setspeed78(5000, 3000);
        break; //11100011 11111111
    case 0xFFC7: setspeed78(3000, 5000);
        break; //11111111 11000111

    case 0xF7FF: setspeed78(5000, 2000);
        break; //11110111 11111111
    case 0xE7FF: setspeed78(5000, 2000);
        break; //11100111 11111111
    case 0xFFEF: setspeed78(2000, 5000);
        break; //11111111 11101111
    case 0xFFE7: setspeed78(2000, 5000);
        break; //11111111 11100111
    case 0xC7FF: setspeed78(5000, 2000);
        break; //11000111 11111111
    case 0xFFE3: setspeed78(2000, 5000);
        break; //11111111 11100011

    case 0xEFFF: setspeed78(5000, 2000);
        break; //11101111 11111111
    case 0xCFFF: setspeed78(5000, 2000);
        break; //11001111 11111111
    case 0xFFF7: setspeed78(2000, 5000);
        break; //11111111 11110111
    case 0xFFF3: setspeed78(2000, 5000);
        break; //11111111 11110011
    case 0x8FFF: setspeed78(5000, 2000);
        break; //10001111 11111111
    case 0xFFF1: setspeed78(2000, 5000);
        break; //11111111 11110001

    case 0xFFFB: setspeed78(1000, 5000);
        break; //11111111 11111011
    case 0xFFF9: setspeed78(1000, 5000);
        break; //11111111 11111001
    case 0xDFFF: setspeed78(5000, 1000);
        break; //11011111 11111111
    case 0x9FFF: setspeed78(5000, 1000);
        break; //10011111 11111111
    case 0xFFF8: setspeed78(1000, 5000);
        break; //11111111 11111000
    case 0x1FFF: setspeed78(5000, 1000);
        break; //00011111 11111111

    case 0xFFFD: setspeed78(0, 5000);
        break; //11111111 11111101
    case 0xFFFC: setspeed78(0, 5000);
        break; //11111111 11111100
    case 0xBFFF: setspeed78(5000, 0);
        break; //10111111 11111111
    case 0x3FFF: setspeed78(5000, 0);
        break; //00111111 11111111
    case 0xFFFE: setspeed78(0, 5000);
        break; //11111111 11111110
    case 0x7FFF: setspeed78(5000, 0);
        break; //01111111 11111111

    default: setspeed78(5000, 5000);
        break;
    }
}
void trackxian8()
{
    sensor_update();

    switch (HUI_data) //1111111
    {
    //65432109 87654321
    case 0xFE7F: setspeed2(8000, 8000);
        break; //11111110 01111111		//���м�λ��
    case 0xFC7F: setspeed2(8000, 7000);
        break; //11111100 01111111
    case 0xFE3F: setspeed2(7000, 8000);
        break; //11111110 00111111
    case 0xFEFF: setspeed2(8000, 7000);
        break; //11111110 11111111
    case 0xFCFF: setspeed2(8000, 6500);
        break; //11111100 11111111
    case 0xFF7F: setspeed2(6500, 8000);
        break; //11111111 01111111
    case 0xFF3F: setspeed2(6500, 8000);
        break; //11111111 00111111
    case 0xF8FF: setspeed2(8000, 6500);
        break; //11111000 11111111
    case 0xFF1F: setspeed2(6500, 8000);
        break; //11111111 00011111

    case 0xFDFF: setspeed2(8000, 6000);
        break; //11111101 11111111
    case 0xF9FF: setspeed2(8000, 6000);
        break; //11111001 11111111
    case 0xFFBF: setspeed2(6000, 8000);
        break; //11111111 10111111
    case 0xFF9F: setspeed2(6000, 8000);
        break; //11111111 10011111
    case 0xF1FF: setspeed2(8000, 6000);
        break; //11110001 11111111
    case 0xFF8F: setspeed2(6000, 8000);
        break; //11111111 10001111

    case 0xFBFF: setspeed2(8000, 5000);
        break; //11111011 11111111
    case 0xF3FF: setspeed2(8000, 5000);
        break; //11110011 11111111
    case 0xFFDF: setspeed2(5000, 8000);
        break; //11111111 11011111
    case 0xFFCF: setspeed2(5000, 8000);
        break; //11111111 11001111
    case 0xE3FF: setspeed2(8000, 5000);
        break; //11100011 11111111
    case 0xFFC7: setspeed2(5000, 8000);
        break; //11111111 11000111

    case 0xF7FF: setspeed2(8000, 4000);
        break; //11110111 11111111
    case 0xE7FF: setspeed2(8000, 4000);
        break; //11100111 11111111
    case 0xFFEF: setspeed2(4000, 8000);
        break; //11111111 11101111
    case 0xFFE7: setspeed2(4000, 8000);
        break; //11111111 11100111
    case 0xC7FF: setspeed2(8000, 4000);
        break; //11000111 11111111
    case 0xFFE3: setspeed2(4000, 8000);
        break; //11111111 11100011

    case 0xEFFF: setspeed2(8000, 3000);
        break; //11101111 11111111
    case 0xCFFF: setspeed2(8000, 3000);
        break; //11001111 11111111
    case 0xFFF7: setspeed2(3000, 8000);
        break; //11111111 11110111
    case 0xFFF3: setspeed2(3000, 8000);
        break; //11111111 11110011
    case 0x8FFF: setspeed2(8000, 3000);
        break; //10001111 11111111
    case 0xFFF1: setspeed2(3000, 8000);
        break; //11111111 11110001

    case 0xFFFB: setspeed2(8000, 1000);
        break; //11111111 11111011
    case 0xFFF9: setspeed2(1000, 8000);
        break; //11111111 11111001
    case 0xDFFF: setspeed2(8000, 1000);
        break; //11011111 11111111
    case 0x9FFF: setspeed2(8000, 1000);
        break; //10011111 11111111
    case 0xFFF8: setspeed2(1000, 8000);
        break; //11111111 11111000
    case 0x1FFF: setspeed2(8000, 4000);
        break; //00011111 11111111

    case 0xFFFD: setspeed2(1000, 8000);
        break; //11111111 11111101
    case 0xFFFC: setspeed2(1000, 8000);
        break; //11111111 11111100
    case 0xBFFF: setspeed2(8000, 1000);
        break; //10111111 11111111
    case 0x3FFF: setspeed2(8000, 1000);
        break; //00111111 11111111
    case 0xFFFE: setspeed2(1000, 8000);
        break; //11111111 11111110
    case 0x7FFF: setspeed2(8000, 1000);
        break; //01111111 11111111
    default: setspeed2(8000, 8000);
        break;
    }
}


void trackxian10()
{
    switch (HUI_data) //1111111
    {
    //65432109 87654321
    case 0xFE7F: setspeed2(10000, 10000);
        break; //11111110 01111111		//���м�λ��
    case 0xFC7F: setspeed2(10000, 9500);
        break; //11111100 01111111
    case 0xFE3F: setspeed2(9500, 10000);
        break; //11111110 00111111
    case 0xFEFF: setspeed2(10000, 9000);
        break; //11111110 11111111
    case 0xFCFF: setspeed2(10000, 9000);
        break; //11111100 11111111
    case 0xFF7F: setspeed2(9000, 10000);
        break; //11111111 01111111
    case 0xFF3F: setspeed2(9000, 10000);
        break; //11111111 00111111
    case 0xF8FF: setspeed2(10000, 9000);
        break; //11111000 11111111
    case 0xFF1F: setspeed2(9000, 10000);
        break; //11111111 00011111

    case 0xFDFF: setspeed2(10000, 8000);
        break; //11111101 11111111
    case 0xF9FF: setspeed2(10000, 8000);
        break; //11111001 11111111
    case 0xFFBF: setspeed2(8000, 10000);
        break; //11111111 10111111
    case 0xFF9F: setspeed2(8000, 10000);
        break; //11111111 10011111
    case 0xF1FF: setspeed2(10000, 8000);
        break; //11110001 11111111
    case 0xFF8F: setspeed2(8000, 10000);
        break; //11111111 10001111

    case 0xFBFF: setspeed2(10000, 7500);
        break; //11111011 11111111
    case 0xF3FF: setspeed2(10000, 7500);
        break; //11110011 11111111
    case 0xFFDF: setspeed2(7500, 10000);
        break; //11111111 11011111
    case 0xFFCF: setspeed2(7500, 10000);
        break; //11111111 11001111
    case 0xE3FF: setspeed2(10000, 7500);
        break; //11100011 11111111
    case 0xFFC7: setspeed2(7500, 10000);
        break; //11111111 11000111

    case 0xF7FF: setspeed2(10000, 6500);
        break; //11110111 11111111
    case 0xE7FF: setspeed2(10000, 6500);
        break; //11100111 11111111
    case 0xFFEF: setspeed2(6500, 10000);
        break; //11111111 11101111
    case 0xFFE7: setspeed2(6500, 10000);
        break; //11111111 11100111
    case 0xC7FF: setspeed2(10000, 6500);
        break; //11000111 11111111
    case 0xFFE3: setspeed2(6500, 10000);
        break; //11111111 11100011

    case 0xEFFF: setspeed2(10000, 6000);
        break; //11101111 11111111
    case 0xCFFF: setspeed2(10000, 6000);
        break; //11001111 11111111
    case 0xFFF7: setspeed2(6000, 10000);
        break; //11111111 11110111
    case 0xFFF3: setspeed2(6000, 10000);
        break; //11111111 11110011
    case 0x8FFF: setspeed2(10000, 6000);
        break; //10001111 11111111
    case 0xFFF1: setspeed2(6000, 10000);
        break; //11111111 11110001

    case 0xFFFB: setspeed2(5000, 10000);
        break; //11111111 11111011
    case 0xFFF9: setspeed2(5000, 10000);
        break; //11111111 11111001
    case 0xDFFF: setspeed2(10000, 5000);
        break; //11011111 11111111
    case 0x9FFF: setspeed2(10000, 5000);
        break; //10011111 11111111
    case 0xFFF8: setspeed2(5000, 10000);
        break; //11111111 11111000
    case 0x1FFF: setspeed2(10000, 5000);
        break; //00011111 11111111

    case 0xFFFD: setspeed2(7000, 10000);
        break; //11111111 11111101
    case 0xFFFC: setspeed2(7000, 10000);
        break; //11111111 11111100
    case 0xBFFF: setspeed2(10000, 7000);
        break; //10111111 11111111
    case 0x3FFF: setspeed2(10000, 7000);
        break; //00111111 11111111
    case 0xFFFE: setspeed2(5000, 10000);
        break; //11111111 11111110
    case 0x7FFF: setspeed2(10000, 5000);
        break; //01111111 11111111
    default: setspeed2(10000, 10000);
        break;
    }
}


void trackxian12()
{
    sensor_update();

    switch (HUI_data) //1111111
    {
    //65432109 87654321
    case 0xFE7F: setspeed2(12000, 12000);
        break; //11111110 01111111		//���м�λ��
    case 0xFC7F: setspeed2(12000, 12000);
        break; //11111100 01111111
    case 0xFE3F: setspeed2(12000, 12000);
        break; //11111110 00111111
    case 0xFEFF: setspeed2(12000, 11500);
        break; //11111110 11111111
    case 0xFCFF: setspeed2(12000, 11500);
        break; //11111100 11111111
    case 0xFF7F: setspeed2(11500, 12000);
        break; //11111111 01111111
    case 0xFF3F: setspeed2(11500, 12000);
        break; //11111111 00111111
    case 0xF8FF: setspeed2(12000, 11500);
        break; //11111000 11111111
    case 0xFF1F: setspeed2(11500, 12000);
        break; //11111111 00011111

    case 0xFDFF: setspeed2(12000, 11000);
        break; //11111101 11111111
    case 0xF9FF: setspeed2(12000, 11000);
        break; //11111001 11111111
    case 0xFFBF: setspeed2(11000, 12000);
        break; //11111111 10111111
    case 0xFF9F: setspeed2(11000, 12000);
        break; //11111111 10011111
    case 0xF1FF: setspeed2(12000, 11000);
        break; //11110001 11111111
    case 0xFF8F: setspeed2(11000, 12000);
        break; //11111111 10001111

    case 0xFBFF: setspeed2(12000, 10500);
        break; //11111011 11111111
    case 0xF3FF: setspeed2(12000, 10500);
        break; //11110011 11111111
    case 0xFFDF: setspeed2(10500, 12000);
        break; //11111111 11011111
    case 0xFFCF: setspeed2(10500, 12000);
        break; //11111111 11001111
    case 0xE3FF: setspeed2(12000, 10500);
        break; //11100011 11111111
    case 0xFFC7: setspeed2(10500, 12000);
        break; //11111111 11000111

    case 0xF7FF: setspeed2(12000, 10000);
        break; //11110111 11111111
    case 0xE7FF: setspeed2(12000, 10000);
        break; //11100111 11111111
    case 0xFFEF: setspeed2(10000, 12000);
        break; //11111111 11101111
    case 0xFFE7: setspeed2(10000, 12000);
        break; //11111111 11100111
    case 0xC7FF: setspeed2(12000, 10000);
        break; //11000111 11111111
    case 0xFFE3: setspeed2(10000, 12000);
        break; //11111111 11100011

    case 0xEFFF: setspeed2(12000, 9000);
        break; //11101111 11111111
    case 0xCFFF: setspeed2(12000, 9000);
        break; //11001111 11111111
    case 0xFFF7: setspeed2(9000, 12000);
        break; //11111111 11110111
    case 0xFFF3: setspeed2(9000, 12000);
        break; //11111111 11110011
    case 0x8FFF: setspeed2(12000, 9000);
        break; //10001111 11111111
    case 0xFFF1: setspeed2(9000, 12000);
        break; //11111111 11110001

    case 0xFFFB: setspeed2(8000, 12000);
        break; //11111111 11111011
    case 0xFFF9: setspeed2(8000, 12000);
        break; //11111111 11111001
    case 0xDFFF: setspeed2(12000, 8000);
        break; //11011111 11111111
    case 0x9FFF: setspeed2(12000, 8000);
        break; //10011111 11111111
    case 0xFFF8: setspeed2(8000, 12000);
        break; //11111111 11111000
    case 0x1FFF: setspeed2(12000, 8000);
        break; //00011111 11111111

    case 0xFFFD: setspeed2(7000, 12000);
        break; //11111111 11111101
    case 0xFFFC: setspeed2(7000, 12000);
        break; //11111111 11111100
    case 0xBFFF: setspeed2(12000, 7000);
        break; //10111111 11111111
    case 0x3FFF: setspeed2(12000, 7000);
        break; //00111111 11111111
    case 0xFFFE: setspeed2(5000, 12000);
        break; //11111111 11111110
    case 0x7FFF: setspeed2(12000, 5000);
        break; //01111111 11111111
    default: setspeed2(12000, 12000);
        break;
    }
}


void trackxian15()
{
    sensor_update();

    switch (HUI_data) //1111111
    {
    //65432109 87654321
    case 0xFE7F: setspeed2(15000, 15000);
        break; //11111110 01111111		//���м�λ��
    case 0xFC7F: setspeed2(15000, 15000);
        break; //11111100 01111111
    case 0xFE3F: setspeed2(15000, 15000);
        break; //11111110 00111111
    case 0xFEFF: setspeed2(15000, 14500);
        break; //11111110 11111111
    case 0xFCFF: setspeed2(15000, 14500);
        break; //11111100 11111111
    case 0xFF7F: setspeed2(14500, 15000);
        break; //11111111 01111111
    case 0xFF3F: setspeed2(14500, 15000);
        break; //11111111 00111111
    case 0xF8FF: setspeed2(15000, 14500);
        break; //11111000 11111111
    case 0xFF1F: setspeed2(14500, 15000);
        break; //11111111 00011111

    case 0xFDFF: setspeed2(15000, 14000);
        break; //11111101 11111111
    case 0xF9FF: setspeed2(15000, 14000);
        break; //11111001 11111111
    case 0xFFBF: setspeed2(14000, 15000);
        break; //11111111 10111111
    case 0xFF9F: setspeed2(14000, 15000);
        break; //11111111 10011111
    case 0xF1FF: setspeed2(15000, 14000);
        break; //11110001 11111111
    case 0xFF8F: setspeed2(14000, 15000);
        break; //11111111 10001111

    case 0xFBFF: setspeed2(15000, 13500);
        break; //11111011 11111111
    case 0xF3FF: setspeed2(15000, 13500);
        break; //11110011 11111111
    case 0xFFDF: setspeed2(13500, 15000);
        break; //11111111 11011111
    case 0xFFCF: setspeed2(13500, 15000);
        break; //11111111 11001111
    case 0xE3FF: setspeed2(15000, 13500);
        break; //11100011 11111111
    case 0xFFC7: setspeed2(13500, 15000);
        break; //11111111 11000111

    case 0xF7FF: setspeed2(15000, 12500);
        break; //11110111 11111111
    case 0xE7FF: setspeed2(15000, 12500);
        break; //11100111 11111111
    case 0xFFEF: setspeed2(12500, 15000);
        break; //11111111 11101111
    case 0xFFE7: setspeed2(12500, 15000);
        break; //11111111 11100111
    case 0xC7FF: setspeed2(15000, 12500);
        break; //11000111 11111111
    case 0xFFE3: setspeed2(12500, 15000);
        break; //11111111 11100011

    case 0xEFFF: setspeed2(15000, 11500);
        break; //11101111 11111111
    case 0xCFFF: setspeed2(15000, 11500);
        break; //11001111 11111111
    case 0xFFF7: setspeed2(11500, 15000);
        break; //11111111 11110111
    case 0xFFF3: setspeed2(11500, 15000);
        break; //11111111 11110011
    case 0x8FFF: setspeed2(15000, 11500);
        break; //10001111 11111111
    case 0xFFF1: setspeed2(11500, 15000);
        break; //11111111 11110001

    case 0xFFFB: setspeed2(10000, 15000);
        break; //11111111 11111011
    case 0xFFF9: setspeed2(10000, 15000);
        break; //11111111 11111001
    case 0xDFFF: setspeed2(15000, 10000);
        break; //11011111 11111111
    case 0x9FFF: setspeed2(15000, 10000);
        break; //10011111 11111111
    case 0xFFF8: setspeed2(10000, 15000);
        break; //11111111 11111000
    case 0x1FFF: setspeed2(15000, 10000);
        break; //00011111 11111111

    case 0xFFFD: setspeed2(8000, 15000);
        break; //11111111 11111101
    case 0xFFFC: setspeed2(8000, 15000);
        break; //11111111 11111100
    case 0xBFFF: setspeed2(15000, 8000);
        break; //10111111 11111111
    case 0x3FFF: setspeed2(15000, 8000);
        break; //00111111 11111111
    case 0xFFFE: setspeed2(7000, 15000);
        break; //11111111 11111110
    case 0x7FFF: setspeed2(15000, 7000);
        break; //01111111 11111111
    default: setspeed2(15000, 15000);
        break;
    }
}

void gohill()
{
    intia_rount_cnt = get_rount_cnt;
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian3();
}

void goplat()
{
    while (data_storage[12] != 0)
        trackxian5();
    setspeed2(3000, 3000);
    osDelay(100);
    setspeed2(-2000, -2000);
    osDelay(200);
    stop();
    wave_hand();
    osDelay(100);


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 57.2974;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 57.2974;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(167) < rex_abs(angle_sum))
                break;
            set_current(-6000, 6000, -6000, 6000);
            osDelay(1);
        }
        stop();
    }

    stop();
    osDelay(50);

}


void run_task(void const* argument)
{
    INS_angle_go = get_INS_angle_point();
    v_r = v_l = 0;
    const fp32 pid_params[3] = {TRACK_PID_KP, TRACK_PID_KI, TRACK_PID_KD};
    PID_init(&track_pid, PID_POSITION, pid_params, TRACK_PID_MAX_WZ, TRACK_PID_MAX_IOUT);
    osDelay(3000);


    //	while(1)
    //	{ //?????????
    //	  set_spd1 = rc_ctrl_run->rc.ch[1] *16000/660;
    //		set_spd2 = rc_ctrl_run->rc.ch[0] *8000/660;
    //	  motor_pid[0].target = set_spd1+set_spd2;
    //		motor_pid[1].target = -set_spd1+set_spd2;
    //		motor_pid[2].target = set_spd1+set_spd2;
    //		motor_pid[3].target = -set_spd1+set_spd2;
    //	  set_current(set_spd1+set_spd2,set_spd1-set_spd2,set_spd1+set_spd2,set_spd1-set_spd2);
    //	}

    /* ####### start ########*/
    for (;;)
    {
        /* ������: ǰ���� (data_storage[12]==0 ����) �� ����->�ͷ� ���� */


        //
        //
        //
        // STOP
        //
        // go12();
        // go23();
        // go3men15();
        // go57();
        // go78();
        // go8zhi();
        // gozhimen13();
        // STOP


        sensor_update();
        while (data_storage[12] != 0)
        {
            sensor_update();
            osDelay(50);
        }
        while (data_storage[12] == 0)
        {
            sensor_update();
            osDelay(50);
        }
        yuyin(1);
        osDelay(50);
        up_hand();
        go12();
        go24();
        go43();
        go3men1();
        if (tl_scan()==1)
        {
            gomen15();
            go57();
            go78();
            go8zhi();
            gozhimen1();
            sensor_update();
            while (data_storage[12] != 0)
            {
                sensor_update();
                osDelay(50);
            }
            while (data_storage[12] == 0)
            {
                sensor_update();
                osDelay(50);
            }
            yuyin(1);
            osDelay(50);
            up_hand();
            go12();
            go23();
            go3men15();
            go57();
            go78();
            go8zhi();
            gozhimen13();
        }
        else
        {
            gomen1_2();
            if (tl_scan()==1)
            {
                gomen25();
                go57();
                go78();
                go8zhi();
                gozhimen2();
                sensor_update();
                while (data_storage[12] != 0)
                {
                    sensor_update();
                    osDelay(50);
                }
                while (data_storage[12] == 0)
                {
                    sensor_update();
                    osDelay(50);
                }
                yuyin(1);
                osDelay(50);
                wave_hand();
                go12();
                go23();
                go3men25();
                go57();
                go78();
                go8zhi();
                gozhimen24();
            }
            else
            {
                gomen2_3();
                if (tl_scan()==1)
                {
                    gomen35();
                    go57();
                    go78();
                    go8zhi();
                    gozhimen3();
                    sensor_update();
                    while (data_storage[12] != 0)
                    {
                        sensor_update();
                        osDelay(50);
                    }
                    while (data_storage[12] == 0)
                    {
                        sensor_update();
                        osDelay(50);
                    }
                    yuyin(1);
                    osDelay(50);
                    up_hand();
                    go12();
                    go24();
                    go4men35();
                    go57();
                    go78();
                    go8zhi();
                    gozhimen33();
                }
                else
                {
                    gomen3_men4_5();
                    go57();
                    go78();
                    go8zhi();
                    gozhimen4();
                    sensor_update();
                    while (data_storage[12] != 0)
                    {
                        sensor_update();
                        osDelay(50);
                    }
                    while (data_storage[12] == 0)
                    {
                        sensor_update();
                        osDelay(50);
                    }
                    yuyin(1);
                    osDelay(500);
                    up_hand();
                    go12();
                    go24();
                    go4men45();
                    go57();
                    go78();
                    go8zhi();
                    gozhimen43();
                }
            }
        }


    }
}

void turn_run(int speed, fp32 angle)
{
    fp32 last_angle = 0;
    angle_sum = 0;
    last_angle = INS_angle_go[0] * 57.2974;
    uint32_t t0 = osKernelSysTick();


    for (int i = 0;; i++)
    {
        go_yaw = INS_angle_go[0] * 57.2974;
        go_yaw_inia = last_angle - go_yaw;
        if (rex_abs(go_yaw_inia) < 45)
            angle_sum += go_yaw_inia;
        last_angle = go_yaw;
        if (rex_abs(angle) < rex_abs(angle_sum))
            break;
        if (osKernelSysTick() - t0 > 10000)
            break;
        if (angle > 0)
        {
            v_r = -speed;
            v_l = +speed;
        }
        else
        {
            v_r = +speed;
            v_l = -speed;
        }
        set_current(v_r, v_l, v_r, v_l);
        osDelay(1);
    }
    stop();
}

void turn2(int speed, fp32 angle)
{
    fp32 last_angle = 0;
    angle_sum = 0;
    last_angle = INS_angle_go[0] * 180 / 3.1415;


    for (int i = 0;; i++)
    {
        go_yaw = INS_angle_go[0] * 180 / 3.1415;
        go_yaw_inia = last_angle - go_yaw;
        if (rex_abs(go_yaw_inia) < 20)
            angle_sum += go_yaw_inia;
        last_angle = go_yaw;
        if (rex_abs(angle) < rex_abs(angle_sum))
            break;
        if (angle > 0)
        {
            v_r = -speed;
            v_l = +speed;
        }
        else
        {
            v_r = +speed;
            v_l = -speed;
        }
        set_current(v_r, v_l, v_r, v_l);
        osDelay(1);
    }
    stop();
}


void stop(void)
{
    set_current(0, 0, 0, 0);
}


void set0rount()
{
    intia_rount_cnt = get_rount_cnt;
}


void downplat()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian3();
}


void gobri()
{
    if (data_storage[14] != 0 && data_storage[13] == 0)
    {
        set_current(3000, 2000, 3000, 2000);
        osDelay(130);
    }
    else if (data_storage[14] == 0 && data_storage[13] != 0)
    {
        set_current(2000, 3000, 2000, 3000);
        osDelay(130);
    }
    else
    {
        set_current(3700, 3700, 3700, 3700);

        osDelay(130);
    }
}


void gobri1()
{
    if (data_storage[14] == 0 && data_storage[13] != 0)
    {
        set_current(5000, 2000, 5000, 2000);
        osDelay(10);
    }
    if (data_storage[14] != 0 && data_storage[13] == 0)
    {
        set_current(2000, 5000, 2000, 5000);
        osDelay(10);
    }
    else
    {
        set_current(5000, 5000, 5000, 5000);

        osDelay(10);
    }
}

void goqqb()
{
    if (data_storage[14] == 0 && data_storage[13] != 0)
    {
        set_current(3000, 1000, 3000, 1000);
        osDelay(30);
    }
    else if (data_storage[14] != 0 && data_storage[13] == 0)
    {
        set_current(1000, 3000, 1000, 3000);
        osDelay(30);
    }
    else
    {
        set_current(3000, 3000, 3000, 3000);

        osDelay(30);
    }
}

void set_current(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    set_speed_rpm[0] = motor1;
    set_speed_rpm[1] = motor2;
    set_speed_rpm[2] = motor3;
    set_speed_rpm[3] = motor4; ////*0.8173
    chassis_set_motor_rpm(set_speed_rpm);
}

void wave_hand(void)
{
    osDelay(200);
    servo_pwm_set(1500, 1);
    osDelay(200);

    servo_pwm_set(1500, 3);
    osDelay(200);
    servo_pwm_set(500, 3);
    osDelay(200);

    servo_pwm_set(2500, 1);
    osDelay(200);
}

void wave_handc78(void)
{
    osDelay(200);
    servo_pwm_set(1500, 1);
    osDelay(200);

    servo_pwm_set(1500, 3);
    osDelay(200);
    servo_pwm_set(500, 3);
    osDelay(200);

    servo_pwm_set(2500, 1);
    osDelay(200);
}

void up_hand(void)
{
    osDelay(200);
    servo_pwm_set(1500, 1);
    servo_pwm_set(1500, 3);
    osDelay(200);
    servo_pwm_set(500, 3);
    servo_pwm_set(2500, 1);
    osDelay(200);
}


void yuyin(int a)
{
    voice_module_play((voice_track_t)a);
}


void go12()
{
    downplat();

    while (get_roll < 15)
        trackxian(1.1f);
    // intia_rount_cnt=get_rount_cnt;
    // while(get_rount_cnt-intia_rount_cnt<35)
    // 	trackxian(0.5f);
    while (get_roll > -10)
        gobri();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian(1.0f);

    while (get_roll > 10)
        trackxian(1.0f);

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(100);

    setspeed2(-2000, -2000);
    osDelay(200);
    stop();
    wave_handc78();
    osDelay(100);
    turn_run(5000, 170);
    stop();
    osDelay(150);
    yuyin(2);
    osDelay(200);

}

void go24()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[13] != 0 && hui[14] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 57.2974;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 57.2974;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(7000, 500, 7000, 500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    intia_rount_cnt = get_rount_cnt;
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();


    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(100);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(-1500, 7000, -1500, 7000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 390)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 420)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 450)
        trackxian5();
    while (get_roll < 15)
        trackxian5();
    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(300);
    stop();
    wave_handc78();
    turn_run(5000, 170);
    stop();
    osDelay(100);
    yuyin(4);
    osDelay(50);
}
void go23()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[13] != 0 && hui[14] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 57.2974;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 57.2974;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(7000, 500, 7000,500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(130) < rex_abs(angle_sum))
                break;
            set_current(7000, -2000, 7000, -2000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 400)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 410)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 430)
        trackxian5();
    while (get_roll < 15)
        trackxian5();
    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(300);
    stop();
    wave_handc78();
    turn_run(5000, 170);
    stop();
    osDelay(50);
    yuyin(3);
}

void go34()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    intia_rount_cnt = get_rount_cnt;
    while (hui[15] != 0 && hui[14] != 0)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian10();
    while (hui[15] != 0 && hui[14] != 0)
        trackxian12();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian8();
    // while(get_rount_cnt-intia_rount_cnt<50)
    // 	trackxian8();
    //
    // while(get_rount_cnt-intia_rount_cnt<150)
    // 	trackxian12();
    // while(get_rount_cnt-intia_rount_cnt<290)
    // 	trackxian8();
    // while(hui[0] != 0 && hui[1] != 0)
    // 	trackxian8();

    while (get_roll < 15)
        trackxian5();
    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);
    // set0rount();
    // while(get_rount_cnt-intia_rount_cnt<180)
    trackxian5();
    setspeed2(-2000, -2000);
    osDelay(200);
    stop();
    osDelay(100);
    turn_run(5000, 165);
    stop();
    osDelay(100);
    yuyin(2);
    osDelay(100);
    wave_hand();
}

void go4men1()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 280)
        trackxian8();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, 500, 6500, 500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian5();
    while (hui[5] != 0 && hui[6] != 0 && hui[7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0)
        stop();
    osDelay(100);
}

void gomen1_2()
{
    turn_run(5000, 170);
    stop();
    osDelay(100);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();

    while (hui[14] != 0 && hui[13] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(7000, -2500, 7000, -2500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    while (!(lukou_detect(4)))
        trackxian5();
    osDelay(100);
    stop();
    osDelay(700);
}

void gomen2_3()
{
    turn_run(5000, 170);
    stop();
    osDelay(100);
    set0rount();
    // while (get_rount_cnt - intia_rount_cnt < 30)
    //     trackxian5();

    while (hui[13] != 0 && hui[14] != 0)
        trackxian5();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(135) < rex_abs(angle_sum))
                break;
            set_current(7000, -2500, 7000, -2500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 230)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 270)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(135) < rex_abs(angle_sum))
                break;
            set_current(7000, -2500, 7000, -2500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    while (!(lukou_detect(4)))
        trackxian5();
    osDelay(100);
    stop();
    osDelay(700);
}

void gomen3_4()
{
    turn_run(5000, 170);
    stop();
    osDelay(100);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(50);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(130) < rex_abs(angle_sum))
                break;
            set_current(6000, -1500, 6000, -1500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();
    // while (hui[5] != 0 && hui[6] != 0 && hui[7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0)
    //     stop();
    // osDelay(100);
}

void gomen3_men4_5()
{
    turn_run(5000, 170);
    stop();
    osDelay(100);
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(130) < rex_abs(angle_sum))
                break;
            set_current(6000, -2000, 6000, -2000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-2000, 7000, -2000, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 210)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 280)
        trackxian5();

    while (get_roll < 15)
        trackxian5();


    goplat();
    yuyin(5);
}

void gomen15()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    turnright();

    stop();
    osDelay(50);
    for (int i = 0; i < 80; i++)
    {
        trackxian5();
        osDelay(1);
    }
    stop();
    osDelay(50);
    yuyin(10);
    osDelay(50);
    setspeed2(-3000, -3000);
    osDelay(900);
    stop();
    // jindian[1] = digit_read();

    turn_run(5000, 85);
    while (!lukou_detect(5))
        trackxian5();

    turnleft();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();

    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian12();

    // while (get_rount_cnt - intia_rount_cnt < 230)
    //     trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 270)
        trackxian8();

    while (!lukou_detect(5))
        trackxian5();

    turnleft();

    while (!lukou_detect(5))
        trackxian5();

    turnleft();

    stop();
    osDelay(50);

    	// while(data_storage[12] != 0)
    	// 		trackxian5();
    for (int i = 0; i < 80; i++)
    {
        trackxian5();
        osDelay(1);
    }
    stop();
    osDelay(50);
    yuyin(12);
    osDelay(50);
    setspeed2(-3000, -3000);
    osDelay(800);
    stop();
    //jindian[2] = digit_read();

    turn_run(5000, -85);
    while (!lukou_detect(5))
        trackxian5();

    turnright();

    set0rount();
    while(get_rount_cnt-intia_rount_cnt<40)
        trackxian8();
    while(get_rount_cnt-intia_rount_cnt<60)
        trackxian12();
    while(get_rount_cnt-intia_rount_cnt<200)
        trackxian15();
    while(get_rount_cnt-intia_rount_cnt<230)
         trackxian12();
    // while(get_rount_cnt-intia_rount_cnt<270)
    //     trackxian8();
    while (get_roll < 15)
        trackxian8();

    goplat();
    yuyin(5);
}

void gomen25()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 230)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 300)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(100);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(-1500, 7500, -1500, 7500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 210)
        trackxian12();
    // while(get_rount_cnt-intia_rount_cnt<200)
    // 	trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 280)
        trackxian5();
    while (get_roll < 15)
        trackxian5();


    goplat();
    yuyin(5);
}

void gomen35()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian5();
    // while (hui[0] != 0 && hui[1] != 0)
    //     trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(100);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (hui[15] != 0 && hui[14] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }
    stop();
    osDelay(50);
    //	while(data_storage[12] != 0)
    //			trackxian5();
    for (int i = 0; i < 70; i++)
    {
        trackxian5();
        osDelay(1);
    }
    yuyin(10);
    stop();
    osDelay(100);
    setspeed2(-3000, -3000);
    osDelay(800);
    turn_run(3000, 85);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 240)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 270)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 290)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-2000, 6500, -2000, 6500);
            osDelay(1);
        }
    }
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-2000, 6500, -2000, 6500);
            osDelay(1);
        }
    }
    stop();
    osDelay(100);

    //	while(data_storage[12] != 0)
    //			trackxian5();
    for (int i = 0; i < 80; i++)
    {
        trackxian5();
        osDelay(1);
    }

    yuyin(12);
    stop();
    osDelay(100);
    setspeed2(-3000, -3000);
    osDelay(700);
    turn_run(3000, -85);
    while (!lukou_detect(5))
        trackxian5();

    turnright();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian12();
    // while(get_rount_cnt-intia_rount_cnt<200)
    // 	trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 240)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 280)
        trackxian5();
    while (get_roll < 15)
        trackxian5();


    goplat();
    yuyin(5);
}
void go4men35()
{
    downplat();
    while (get_roll < -5)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 220)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 280)
        trackxian5();
    // while (hui[0] != 0 && hui[1] != 0)
    //     trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 270)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 300)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 320)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }
    stop();
    osDelay(50);
    //	while(data_storage[12] != 0)
    //			trackxian5();
    for (int i = 0; i < 70; i++)
    {
        trackxian5();
        osDelay(1);
    }
    yuyin(10);
    stop();
    osDelay(100);
    setspeed2(-3000, -3000);
    osDelay(520);
    turn_run(3000, 85);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 240)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 270)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 290)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-2000, 6500, -2000, 6500);
            osDelay(1);
        }
    }
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-2000, 6500, -2000, 6500);
            osDelay(1);
        }
    }
    stop();
    osDelay(100);

    //	while(data_storage[12] != 0)
    //			trackxian5();
    for (int i = 0; i < 80; i++)
    {
        trackxian5();
        osDelay(1);
    }

    yuyin(10);
    stop();
    osDelay(100);
    setspeed2(-3000, -3100);
    osDelay(700);
    turn_run(3000, -85);
    while (!lukou_detect(5))
        trackxian5();

    turnright();

    set0rount();
    while(get_rount_cnt-intia_rount_cnt<20)
        trackxian5();
    while(get_rount_cnt-intia_rount_cnt<50)
        trackxian8();
    while(get_rount_cnt-intia_rount_cnt<100)
        trackxian10();
    while(get_rount_cnt-intia_rount_cnt<270)
        trackxian12();
    while(get_rount_cnt-intia_rount_cnt<280)
        trackxian8();
    while (get_roll < 15)
        trackxian5();


    goplat();
    yuyin(5);
}
void gomen45()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian5();


    while (hui[0] != 0 && hui[1] != 0)
        trackxian8();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 6500, -500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian8();
    while (get_roll < 15)
        trackxian5();
    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);
    // set0rount();
    // while(get_rount_cnt-intia_rount_cnt<180)
    trackxian5();
    setspeed2(-2000, -2000);
    osDelay(200);
    stop();
    osDelay(100);
    turn_run(5000, 165);
    stop();
    osDelay(100);
    yuyin(2);
    osDelay(100);
    wave_hand();
}

void go57()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian5();
    while (hui[1] != 0 && hui[2] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(50) < rex_abs(angle_sum))
                break;
            set_current(500, 7000, 500, 7000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 130)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(200);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(-3000, 5500, -3000, 5500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian8();
    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();
    while (get_roll < 15)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian5();
    while (data_storage[12] != 0)
        trackxian3();
    setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(300);
    stop();
    wave_handc78();
    osDelay(50);

    turn_run(5000, 169);
    stop();
    osDelay(50);
    yuyin(7);
    osDelay(50);

}

void go78()
{
    //servo_pwm_set(700, 0);
    //servo_pwm_set(2300, 1);
    //servo_pwm_set(1300, 2);
    while (get_roll > -5)
        set_current(3000, 3000, 3000, 3000);
    while (hui[0] != 0 && hui[1] != 0 && hui[2] != 0 && hui[3] != 0 && hui[4] != 0 && hui[5] != 0 && hui[6] != 0 && hui[
            7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 && hui
        [14]
        != 0 && hui[15] != 0)
        set_current(3000, 3000, 3000, 3000);
    osDelay(100);
    while (hui[0] != 0 && hui[1] != 0 && hui[2] != 0 && hui[3] != 0 && hui[4] != 0 && hui[5] != 0 && hui[6] != 0 && hui[
            7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 && hui
        [14]
        != 0 && hui[15] != 0)
        set_current(3000, 3000, 3000, 3000);

    while (get_roll < -5)
        trackxian78();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian3();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian5();

    while (get_roll < 15)
        trackxian8();
    gohill();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 90)
        trackxian8();
    // while(get_rount_cnt-intia_rount_cnt<100)
    // 	trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian8();
    while (get_roll < 15)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 210)
        trackxian5();
    while (data_storage[12] != 0)
        trackxian3();

    //servo_pwm_set(1750, 2);

    setspeed2(3000, 3000);
    osDelay(150);
    setspeed2(-2000, -2000);
    osDelay(200);
    stop();
    wave_handc78();
    osDelay(50);
    //servo_pwm_set(2300, 0);
    // osDelay(150);
    //servo_pwm_set(700, 1);

    turn_run(5000, -168);
    stop();
    osDelay(100);
    yuyin(8);
    osDelay(50);
}

void go8zhi()
{
    //servo_pwm_set(1300, 2);
    //servo_pwm_set(1500, 1);
    //servo_pwm_set(1500, 0);
    while (get_roll > -5)
        set_current(3000, 3000, 3000, 3000);
    while (hui[0] != 0 && hui[1] != 0 && hui[2] != 0 && hui[3] != 0 && hui[4] != 0 && hui[5] != 0 && hui[6] != 0 && hui[
            7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 && hui
        [14]
        != 0 && hui[15] != 0)
        set_current(2000, 2000, 2000, 2000);
    osDelay(100);
    while (hui[0] != 0 && hui[1] != 0 && hui[2] != 0 && hui[3] != 0 && hui[4] != 0 && hui[5] != 0 && hui[6] != 0 && hui[
            7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 && hui
        [14]
        != 0 && hui[15] != 0)
        set_current(2000, 2000, 2000, 2000);


    intia_rount_cnt = get_rount_cnt;
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();

    while (get_roll < -15)
        trackxian78();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian78();

    while (get_roll < -15)
        trackxian78();

    while (hui[1] != 0 && hui[2] != 0)
        trackxian5();
    osDelay(200);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;

            set_current(-4000, 7000, -4000, 7000);
            osDelay(1);
        }
        stop();
    }

    // while(hui[7] !=0 && hui[8] !=0  && hui[9] !=0 && hui[10] !=0 && hui[11] !=0 && hui[12] !=0 && hui[13] !=0 && hui[14] !=0 && hui[15] !=0)
    // 	set_current(-2000,5000,-2000,5000);

    set0rount();
    // while (get_rount_cnt - intia_rount_cnt < 30)
    //     trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(50);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(110) < rex_abs(angle_sum))
                break;

            set_current(-2000, 8000, -2000, 8000);
            osDelay(1);
        }
        stop();
    }

    // while(hui[7] !=0 && hui[8] !=0  && hui[9] !=0 && hui[10] !=0 && hui[11] !=0 && hui[12] !=0 && hui[13] !=0 && hui[14] !=0 && hui[15] !=0)
    // set_current(-1000,8000,-1000,8000);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian8();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 110)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();

    while (hui[14] != 0 && hui[13] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;

            set_current(7000, -1000, 7000, -1000);
            osDelay(1);
        }
        stop();
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();

    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;

            set_current(7000, -1000, 7000, -1000);
            osDelay(1);
        }
        stop();
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();

    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;

            set_current(7000, -1000, 7000, -1000);
            osDelay(1);
        }
        stop();
    }

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);
    stop();
    osDelay(50);
    yuyin(11);
    osDelay(500);
    // set0rount();
    // while(get_rount_cnt-intia_rount_cnt<180)
    //trackxian5();
    setspeed2(-3000, -3000);
    osDelay(800);
    stop();
    turn_run(5000, 85);
    stop();
    osDelay(100);
}

void gozhimen1()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    // while(get_rount_cnt-intia_rount_cnt<100)
    // 	trackxian8();
    // while(get_rount_cnt-intia_rount_cnt<150)
    // 	trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;

            set_current(7000, -1000, 7000, -1000);
            osDelay(1);
        }
        stop();
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 290)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 320)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;

            set_current(-1000, 7000, -1000, 7000);
            osDelay(1);
        }
        stop();
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 190)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 230)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;

            set_current(7000, -1000, 7000, -1000);
            osDelay(1);
        }
        stop();
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(100);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;

        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(1000, 8000, 1000, 8000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian3();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(150);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(8000, -1000, 8000, -1000);
            osDelay(1);
        }
    }
    while (get_roll < 15)
        trackxian5();

    while (get_roll > 5)
        trackxian5();

    setspeed2(3000, 3000);
    osDelay(250);
    stop();
    wave_handc78();
    osDelay(100);

    turn_run(5000, 170);

    yuyin(9);
    stop();
    osDelay(200);
}
void gozhimen13()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian8();
    // while(get_rount_cnt-intia_rount_cnt<100)
    // 	trackxian8();
    // while(get_rount_cnt-intia_rount_cnt<150)
    // 	trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;

            set_current(7000, -1000, 7000, -1000);
            osDelay(1);
        }
        stop();
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 290)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 320)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;

            set_current(-1000, 7000, -1000, 7000);
            osDelay(1);
        }
        stop();
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 220)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;

            set_current(6000, -1000, 6000, -1000);
            osDelay(1);
        }
        stop();
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 560)
        trackxian15();
    while (get_rount_cnt - intia_rount_cnt < 590)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 620)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 650)
        trackxian5();
    while (get_roll < 15)
        trackxian5();
    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(120);

    setspeed2(-2000, -2000);
    osDelay(200);
    stop();
    wave_handc78();
    osDelay(50);
    turn_run(5000, 168);
    stop();
    osDelay(50);
    yuyin(4);
    downplat();
    while (get_roll < -5)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();

    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 330)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 360)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 400)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;

        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(6000, -2000, 6000, -2000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 90)
        trackxian3();
    while (get_rount_cnt - intia_rount_cnt < 110)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(150);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(6000, -1000, 6000, -1000);
            osDelay(1);
        }
    }
    while (get_roll < 15)
        trackxian8();

    while (get_roll > 15)
        trackxian5();

    setspeed2(3000, 3000);
    osDelay(300);
    stop();
    wave_handc78();
    osDelay(50);

    turn_run(5000, 170);

    yuyin(9);
    stop();
    osDelay(200);

}
void gozhimen2()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while(get_rount_cnt-intia_rount_cnt<50)
    	trackxian8();
    // while(get_rount_cnt-intia_rount_cnt<150)
    // 	trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;

            set_current(7000, -1000, 7000, -1000);
            osDelay(1);
        }
        stop();
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 290)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 320)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;

            set_current(7000, -1000, 7000, -1000);
            osDelay(1);
        }
        stop();
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (hui[15] != 0 && hui[14] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(7000, -1000, 7000, -1000);
            osDelay(1);
        }
    }
    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);
    stop();
    yuyin(10);
    stop();
    osDelay(50);
    setspeed2(-3000, -3000);
    osDelay(800);
    turn_run(5000, 85);


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 6500, -500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 210)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 280)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 6500, -500, 6500);
            osDelay(1);
        }
    }
    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(100);
    stop();
    yuyin(12);
    stop();
    osDelay(100);
    setspeed2(-3000, -3000);
    osDelay(700);
    turn_run(5000, -85);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    // while(get_rount_cnt-intia_rount_cnt<100)
    // 	trackxian8();
    // while(get_rount_cnt-intia_rount_cnt<150)
    // 	trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(100);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 290)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 320)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 350)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(130) < rex_abs(angle_sum))
                break;
            set_current(7500, -2500, 7500, -2500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(100);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(-1000, 7000, -1000, 7000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian3();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(150);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(8000, -1000, 8000, -1000);
            osDelay(1);
        }
    }
    while (get_roll < 15)
        trackxian5();

    while (get_roll > 5)
        trackxian5();

    setspeed2(3000, 3000);
    osDelay(250);
    stop();
    wave_handc78();
    osDelay(100);

    turn_run(5000, 170);

    yuyin(9);
    stop();
    osDelay(200);
    wave_handc78();
}
void gozhimen24()

{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while(get_rount_cnt-intia_rount_cnt<80)
    	trackxian8();
    // while(get_rount_cnt-intia_rount_cnt<150)
    // 	trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;

            set_current(7000, -1000, 7000, -1000);
            osDelay(1);
        }
        stop();
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 290)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 320)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;

            set_current(7000, -1000, 7000, -1000);
            osDelay(1);
        }
        stop();
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (hui[15] != 0 && hui[14] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(7000, -1000, 7000, -1000);
            osDelay(1);
        }
    }
    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);
    stop();
    yuyin(10);
    stop();
    osDelay(100);
    setspeed2(-3000, -3000);
    osDelay(400);
    turn_run(5000, 85);


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 6500, -500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 210)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 280)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-500, 6500, -500, 6500);
            osDelay(1);
        }
    }
    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(100);
    stop();
    yuyin(10);
    stop();
    osDelay(100);
    setspeed2(-3000, -3100);
    osDelay(700);
    turn_run(5000, -85);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    // while(get_rount_cnt-intia_rount_cnt<100)
    // 	trackxian8();
    // while(get_rount_cnt-intia_rount_cnt<150)
    // 	trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(100);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 290)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 320)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 350)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(130) < rex_abs(angle_sum))
                break;
            set_current(7500, -2500, 7500, -2500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 450)
        trackxian15();
    while (get_rount_cnt - intia_rount_cnt < 480)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 520)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 550)
        trackxian5();
    while (get_roll < 15)
        trackxian5();
    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(120);

    setspeed2(-2000, -2000);
    osDelay(200);
    stop();
    wave_handc78();
    osDelay(500);
    turn_run(5000, 170);
    stop();
    osDelay(100);
    yuyin(4);
    osDelay(200);
    downplat();
    while (get_roll < -5)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();

    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 320)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 360)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 400)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(135) < rex_abs(angle_sum))
                break;
            set_current(7000, -2000, 7000, -2000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(50);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(8000, 1000, 8000, 1000);
            osDelay(1);
        }
    }
    // STOP
    // set0rount();
    // while (get_rount_cnt - intia_rount_cnt < 50)
    //     trackxian5();

    while (get_roll < 15)
        trackxian5();

    while (get_roll > 10)
        trackxian5();

    setspeed2(3000, 3000);
    osDelay(250);
    stop();
    wave_handc78();
    osDelay(200);

    turn_run(5000, 170);

    yuyin(9);
    stop();
    osDelay(200);
    wave_handc78();
}
void gozhimen3()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();
    // while(get_rount_cnt-intia_rount_cnt<100)
    // 	trackxian8();
    // while(get_rount_cnt-intia_rount_cnt<150)
    // 	trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;

            set_current(7000, -1000, 7000, -1000);
            osDelay(1);
        }
        stop();
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 290)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 320)
        trackxian5();
    while (hui[1] != 0 && hui[0] != 0)
        trackxian5();
    osDelay(20);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;

            set_current(-1000, 7000, -1000, 7000);
            osDelay(1);
        }
        stop();
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 330)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 370)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 400)
        trackxian5();
    while (hui[1] != 0 && hui[0] != 0)
        trackxian8();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(130) < rex_abs(angle_sum))
                break;
            set_current(-2000, 6000, -2000, 6000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian5();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian8();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(130) < rex_abs(angle_sum))
                break;
            set_current(6000, -2000, 6000, -2000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian3();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(100);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(8000, 2000, 8000, 2000);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian8();

    while (get_roll > 5)
        trackxian5();


    setspeed2(3000, 3000);
    osDelay(300);
    stop();
    wave_handc78();
    osDelay(200);

    turn_run(5000, 170);

    yuyin(9);
    stop();
    osDelay(200);
}
void gozhimen33()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();
    // while(get_rount_cnt-intia_rount_cnt<100)
    // 	trackxian8();
    // while(get_rount_cnt-intia_rount_cnt<150)
    // 	trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;

            set_current(7000, -1000, 7000, -1000);
            osDelay(1);
        }
        stop();
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 220)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 300)
        trackxian5();
    while (hui[1] != 0 && hui[0] != 0)
        trackxian5();
    osDelay(20);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;

            set_current(-1000, 7000, -1000, 7000);
            osDelay(1);
        }
        stop();
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 330)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 370)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 400)
        trackxian5();
    while (hui[1] != 0 && hui[0] != 0)
        trackxian8();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(130) < rex_abs(angle_sum))
                break;
            set_current(-2000,6000, -2000, 6000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 400)
        trackxian15();
    while (get_rount_cnt - intia_rount_cnt < 440)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 470)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 500)
        trackxian5();
    while (get_roll < 15)
        trackxian5();
    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(120);

    setspeed2(-2000, -2000);
    osDelay(200);
    stop();
    wave_handc78();
    osDelay(500);
    turn_run(5000, 170);
    stop();
    osDelay(100);
    yuyin(3);
    osDelay(200);
    downplat();
    while (get_roll < -5)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();

    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 320)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 360)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 400)
        trackxian5();

    while (hui[1] != 0 && hui[0] != 0)
        trackxian8();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(1000, 6000, 1000, 6000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian3();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(100);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(8000, 2000, 8000, 2000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();


    // while (get_roll < 15)
    //     trackxian8();

    while (get_roll > 5)
        trackxian5();


    setspeed2(3000, 3000);
    osDelay(300);
    stop();
    wave_handc78();
    osDelay(200);

    turn_run(5000, 170);

    yuyin(9);
    stop();
    osDelay(200);

}
void gozhimen4()
{
	set0rount();
	while(get_rount_cnt-intia_rount_cnt<30)
		trackxian5();
	while(get_rount_cnt-intia_rount_cnt<80)
		trackxian8();
	// while(get_rount_cnt-intia_rount_cnt<150)
	// 	trackxian5();
	while(hui[14] !=0 && hui[15] !=0)
		trackxian8();

	{    fp32 last_angle=0;
		angle_sum =0;
		last_angle=INS_angle_go[0]*180/3.1415;


		for(int i =0;;i++){
			go_yaw =INS_angle_go[0]*180/3.1415;
			go_yaw_inia = last_angle-go_yaw;
			if(rex_abs(go_yaw_inia)<20 )
				angle_sum+=go_yaw_inia;
			last_angle = go_yaw;
			if(rex_abs(80)<rex_abs(angle_sum))
				break;

			set_current(5500,-1500,5500,-1500);
			osDelay(1);
		}
		stop();

	}
	set0rount();
	while(get_rount_cnt-intia_rount_cnt<30)
		trackxian8();
	while(get_rount_cnt-intia_rount_cnt<250)
		trackxian12();
	// while(get_rount_cnt-intia_rount_cnt<150)
	// 	trackxian12();
	while(get_rount_cnt-intia_rount_cnt<300)
		trackxian8();
	while(hui[14] !=0 && hui[15] !=0)
		trackxian5();

	{    fp32 last_angle=0;
		angle_sum =0;
		last_angle=INS_angle_go[0]*180/3.1415;


		for(int i =0;;i++){
			go_yaw =INS_angle_go[0]*180/3.1415;
			go_yaw_inia = last_angle-go_yaw;
			if(rex_abs(go_yaw_inia)<20 )
				angle_sum+=go_yaw_inia;
			last_angle = go_yaw;
			if(rex_abs(80)<rex_abs(angle_sum))
				break;

			set_current(7000,-1000,7000,-1000);
			osDelay(1);
		}
		stop();

	}

	set0rount();
	while(get_rount_cnt-intia_rount_cnt<50)
		trackxian5();
	while(hui[15] != 0 && hui[14] != 0)
		trackxian5();
	{fp32 last_angle=0;
		angle_sum =0;
		last_angle=INS_angle_go[0]*180/3.1415;


		for(int i =0;;i++){
			go_yaw =INS_angle_go[0]*180/3.1415;
			go_yaw_inia = last_angle-go_yaw;
			if(rex_abs(go_yaw_inia)<20)
				angle_sum+=go_yaw_inia;
			last_angle = go_yaw;
			if(rex_abs(80)<rex_abs(angle_sum))
				break;
			set_current(6500,-500,6500,-500);
			osDelay(1);
		}
	}
	while(data_storage[12] != 0)
		setspeed2(3000,3000);
	osDelay(150);
	// set0rount();
	// while(get_rount_cnt-intia_rount_cnt<180)
	// trackxian5();
    yuyin(10);
    osDelay(50);
	setspeed2(-3000,-3000);
	osDelay(800);
	stop();


	turn_run(5000,80);
	stop();
	osDelay(100);


	set0rount();
	while(get_rount_cnt-intia_rount_cnt<50)
		trackxian5();
	while(hui[0] != 0 && hui[1] != 0)
		trackxian8();
	{fp32 last_angle=0;
		angle_sum =0;
		last_angle=INS_angle_go[0]*180/3.1415;


		for(int i =0;;i++){
			go_yaw =INS_angle_go[0]*180/3.1415;
			go_yaw_inia = last_angle-go_yaw;
			if(rex_abs(go_yaw_inia)<20)
				angle_sum+=go_yaw_inia;
			last_angle = go_yaw;
			if(rex_abs(80)<rex_abs(angle_sum))
				break;
			set_current(-500,6500,-500,6500);
			osDelay(1);
		}
	}
	set0rount();
	while(get_rount_cnt-intia_rount_cnt<50)
		trackxian8();
	while(get_rount_cnt-intia_rount_cnt<100)
		trackxian10();
	while(get_rount_cnt-intia_rount_cnt<240)
		trackxian12();
	while(get_rount_cnt-intia_rount_cnt<250)
		trackxian8();

	while(hui[0] != 0 && hui[1] != 0)
		trackxian5();
	{fp32 last_angle=0;
		angle_sum =0;
		last_angle=INS_angle_go[0]*180/3.1415;


		for(int i =0;;i++){
			go_yaw =INS_angle_go[0]*180/3.1415;
			go_yaw_inia = last_angle-go_yaw;
			if(rex_abs(go_yaw_inia)<20)
				angle_sum+=go_yaw_inia;
			last_angle = go_yaw;
			if(rex_abs(80)<rex_abs(angle_sum))
				break;
			set_current(-500,6500,-500,6500);
			osDelay(1);
		}
	}
	while(get_rount_cnt-intia_rount_cnt<45)
		trackxian5();
	while(hui[0] != 0 && hui[1] != 0)
		trackxian5();
	{fp32 last_angle=0;
		angle_sum =0;
		last_angle=INS_angle_go[0]*180/3.1415;


		for(int i =0;;i++){
			go_yaw =INS_angle_go[0]*180/3.1415;
			go_yaw_inia = last_angle-go_yaw;
			if(rex_abs(go_yaw_inia)<20)
				angle_sum+=go_yaw_inia;
			last_angle = go_yaw;
			if(rex_abs(80)<rex_abs(angle_sum))
				break;
			set_current(-500,6500,-500,6500);
			osDelay(1);
		}
	}
	while(data_storage[12] != 0)
		setspeed2(3000,3000);
	osDelay(150);
	// set0rount();
	// while(get_rount_cnt-intia_rount_cnt<180)
	trackxian5();
	setspeed2(-3000,-3000);
	osDelay(700);
	stop();
	osDelay(100);
	turn_run(5000,-80);
	stop();
	osDelay(100);
	yuyin(9);
	osDelay(100);
	wave_hand();


	set0rount();
	while(get_rount_cnt-intia_rount_cnt<30)
		trackxian5();
	while(get_rount_cnt-intia_rount_cnt<60)
		trackxian8();
	while(get_rount_cnt-intia_rount_cnt<250)
		trackxian12();
	while(get_rount_cnt-intia_rount_cnt<270)
		trackxian8();

	while(hui[1] != 0 && hui[0] != 0)
		trackxian8();
	osDelay(50);
	{fp32 last_angle=0;
		angle_sum =0;
		last_angle=INS_angle_go[0]*180/3.1415;


		for(int i =0;;i++){
			go_yaw =INS_angle_go[0]*180/3.1415;
			go_yaw_inia = last_angle-go_yaw;
			if(rex_abs(go_yaw_inia)<20)
				angle_sum+=go_yaw_inia;
			last_angle = go_yaw;
			if(rex_abs(80)<rex_abs(angle_sum))
				break;
			set_current(-1000,6000,-1000,6000);
			osDelay(1);
		}
	}

	set0rount();
	while(get_rount_cnt-intia_rount_cnt<90)
		trackxian8();
	while(get_rount_cnt-intia_rount_cnt<110)
		trackxian5();

	while(hui[14] != 0 && hui[15] != 0)
		trackxian5();
	osDelay(100);


	{fp32 last_angle=0;
		angle_sum =0;
		last_angle=INS_angle_go[0]*180/3.1415;


		for(int i =0;;i++){
			go_yaw =INS_angle_go[0]*180/3.1415;
			go_yaw_inia = last_angle-go_yaw;
			if(rex_abs(go_yaw_inia)<20)
				angle_sum+=go_yaw_inia;
			last_angle = go_yaw;
			if(rex_abs(130)<rex_abs(angle_sum))
				break;
			set_current(5000,-2000,5000,-2000);
			osDelay(1);
		}
	}

	set0rount();
	while(get_rount_cnt-intia_rount_cnt<50)
		trackxian3();
	while(get_rount_cnt-intia_rount_cnt<160)
		trackxian5();
	while(get_rount_cnt-intia_rount_cnt<180)
		trackxian8();

	while(hui[0] != 0 && hui[1] != 0)
		trackxian8();
	osDelay(100);
	{fp32 last_angle=0;
		angle_sum =0;
		last_angle=INS_angle_go[0]*180/3.1415;


		for(int i =0;;i++){
			go_yaw =INS_angle_go[0]*180/3.1415;
			go_yaw_inia = last_angle-go_yaw;
			if(rex_abs(go_yaw_inia)<20)
				angle_sum+=go_yaw_inia;
			last_angle = go_yaw;
			if(rex_abs(35)<rex_abs(angle_sum))
				break;
			set_current(7000,-1000,7000,-1000);
			osDelay(1);
		}
	}

	while(get_roll<15)
		trackxian8();

	while(get_roll>5)
		trackxian5();


	setspeed2(3000,3000);
	osDelay(300);
	stop();
	osDelay(200);

	turn_run(5000,170);

	yuyin(9);
	stop();
	osDelay(200);
}
void gozhimen43()
{
	set0rount();
	while(get_rount_cnt-intia_rount_cnt<30)
		trackxian5();
	while(get_rount_cnt-intia_rount_cnt<80)
		trackxian8();
	// while(get_rount_cnt-intia_rount_cnt<150)
	// 	trackxian5();
	while(hui[14] !=0 && hui[15] !=0)
		trackxian8();

	{    fp32 last_angle=0;
		angle_sum =0;
		last_angle=INS_angle_go[0]*180/3.1415;


		for(int i =0;;i++){
			go_yaw =INS_angle_go[0]*180/3.1415;
			go_yaw_inia = last_angle-go_yaw;
			if(rex_abs(go_yaw_inia)<20 )
				angle_sum+=go_yaw_inia;
			last_angle = go_yaw;
			if(rex_abs(80)<rex_abs(angle_sum))
				break;

			set_current(5500,-1500,5500,-1500);
			osDelay(1);
		}
		stop();

	}
	set0rount();
	while(get_rount_cnt-intia_rount_cnt<30)
		trackxian8();
	while(get_rount_cnt-intia_rount_cnt<250)
		trackxian12();
	// while(get_rount_cnt-intia_rount_cnt<150)
	// 	trackxian12();
	while(get_rount_cnt-intia_rount_cnt<300)
		trackxian8();
	while(hui[14] !=0 && hui[15] !=0)
		trackxian5();

	{    fp32 last_angle=0;
		angle_sum =0;
		last_angle=INS_angle_go[0]*180/3.1415;


		for(int i =0;;i++){
			go_yaw =INS_angle_go[0]*180/3.1415;
			go_yaw_inia = last_angle-go_yaw;
			if(rex_abs(go_yaw_inia)<20 )
				angle_sum+=go_yaw_inia;
			last_angle = go_yaw;
			if(rex_abs(80)<rex_abs(angle_sum))
				break;

			set_current(7000,-1000,7000,-1000);
			osDelay(1);
		}
		stop();

	}

	set0rount();
	while(get_rount_cnt-intia_rount_cnt<50)
		trackxian5();
	while(hui[15] != 0 && hui[14] != 0)
		trackxian5();
	{fp32 last_angle=0;
		angle_sum =0;
		last_angle=INS_angle_go[0]*180/3.1415;


		for(int i =0;;i++){
			go_yaw =INS_angle_go[0]*180/3.1415;
			go_yaw_inia = last_angle-go_yaw;
			if(rex_abs(go_yaw_inia)<20)
				angle_sum+=go_yaw_inia;
			last_angle = go_yaw;
			if(rex_abs(80)<rex_abs(angle_sum))
				break;
			set_current(6500,-500,6500,-500);
			osDelay(1);
		}
	}
	while(data_storage[12] != 0)
		setspeed2(3000,3000);
	osDelay(150);
	// set0rount();
	// while(get_rount_cnt-intia_rount_cnt<180)
	trackxian5();
	setspeed2(-3000,-3000);
	osDelay(500);
	stop();
	osDelay(100);
	turn_run(5000,80);
	stop();
	osDelay(100);
	yuyin(10);
	osDelay(100);
	wave_hand();

	set0rount();
	while(get_rount_cnt-intia_rount_cnt<50)
		trackxian5();
	while(hui[0] != 0 && hui[1] != 0)
		trackxian8();
	{fp32 last_angle=0;
		angle_sum =0;
		last_angle=INS_angle_go[0]*180/3.1415;


		for(int i =0;;i++){
			go_yaw =INS_angle_go[0]*180/3.1415;
			go_yaw_inia = last_angle-go_yaw;
			if(rex_abs(go_yaw_inia)<20)
				angle_sum+=go_yaw_inia;
			last_angle = go_yaw;
			if(rex_abs(80)<rex_abs(angle_sum))
				break;
			set_current(-500,6500,-500,6500);
			osDelay(1);
		}
	}
	set0rount();
	while(get_rount_cnt-intia_rount_cnt<50)
		trackxian8();
	while(get_rount_cnt-intia_rount_cnt<100)
		trackxian10();
	while(get_rount_cnt-intia_rount_cnt<240)
		trackxian12();
	while(get_rount_cnt-intia_rount_cnt<250)
		trackxian8();

	while(hui[0] != 0 && hui[1] != 0)
		trackxian5();
	{fp32 last_angle=0;
		angle_sum =0;
		last_angle=INS_angle_go[0]*180/3.1415;


		for(int i =0;;i++){
			go_yaw =INS_angle_go[0]*180/3.1415;
			go_yaw_inia = last_angle-go_yaw;
			if(rex_abs(go_yaw_inia)<20)
				angle_sum+=go_yaw_inia;
			last_angle = go_yaw;
			if(rex_abs(80)<rex_abs(angle_sum))
				break;
			set_current(-500,6500,-500,6500);
			osDelay(1);
		}
	}
	while(get_rount_cnt-intia_rount_cnt<45)
		trackxian5();
	while(hui[0] != 0 && hui[1] != 0)
		trackxian5();
	{fp32 last_angle=0;
		angle_sum =0;
		last_angle=INS_angle_go[0]*180/3.1415;


		for(int i =0;;i++){
			go_yaw =INS_angle_go[0]*180/3.1415;
			go_yaw_inia = last_angle-go_yaw;
			if(rex_abs(go_yaw_inia)<20)
				angle_sum+=go_yaw_inia;
			last_angle = go_yaw;
			if(rex_abs(80)<rex_abs(angle_sum))
				break;
			set_current(-500,6500,-500,6500);
			osDelay(1);
		}
	}
	while(data_storage[12] != 0)
		setspeed2(3000,3000);
	osDelay(150);
	// set0rount();
	// while(get_rount_cnt-intia_rount_cnt<180)
	trackxian5();
	setspeed2(-3000,-3000);
	osDelay(700);
	stop();
	osDelay(100);
	turn_run(5000,-80);
	stop();
	osDelay(100);
	yuyin(10);
	osDelay(100);
	wave_hand();


	set0rount();
	while(get_rount_cnt-intia_rount_cnt<30)
		trackxian5();
	while(get_rount_cnt-intia_rount_cnt<60)
		trackxian8();
	while(get_rount_cnt-intia_rount_cnt<250)
		trackxian12();
	while(get_rount_cnt-intia_rount_cnt<270)
		trackxian8();

	while(hui[1] != 0 && hui[0] != 0)
		trackxian8();
	osDelay(50);
	{fp32 last_angle=0;
		angle_sum =0;
		last_angle=INS_angle_go[0]*180/3.1415;


		for(int i =0;;i++){
			go_yaw =INS_angle_go[0]*180/3.1415;
			go_yaw_inia = last_angle-go_yaw;
			if(rex_abs(go_yaw_inia)<20)
				angle_sum+=go_yaw_inia;
			last_angle = go_yaw;
			if(rex_abs(80)<rex_abs(angle_sum))
				break;
			set_current(-1000,6000,-1000,6000);
			osDelay(1);
		}
	}

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 460)
        trackxian15();
    while (get_rount_cnt - intia_rount_cnt < 490)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 520)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 550)
        trackxian5();
    while (get_roll < 15)
        trackxian5();
    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(120);

    setspeed2(-2000, -2000);
    osDelay(200);
    stop();
    osDelay(500);
    turn_run(5000, 170);
    stop();
    osDelay(100);
    yuyin(3);
    osDelay(200);
    wave_handc78();
    downplat();
    while (get_roll < -5)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();

    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 320)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 360)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 400)
        trackxian5();

    while (hui[1] != 0 && hui[0] != 0)
        trackxian8();


    {
	    fp32 last_angle = 0;
	    angle_sum = 0;
	    last_angle = INS_angle_go[0] * 180 / 3.1415;


	    for (int i = 0;; i++)
	    {
	        go_yaw = INS_angle_go[0] * 180 / 3.1415;
	        go_yaw_inia = last_angle - go_yaw;
	        if (rex_abs(go_yaw_inia) < 20)
	            angle_sum += go_yaw_inia;
	        last_angle = go_yaw;
	        if (rex_abs(40) < rex_abs(angle_sum))
	            break;
	        set_current(1000, 6000, 1000, 6000);
	        osDelay(1);
	    }
    }


	set0rount();
	while(get_rount_cnt-intia_rount_cnt<50)
		trackxian5();
	while(get_rount_cnt-intia_rount_cnt<100)
		trackxian3();
	while(get_rount_cnt-intia_rount_cnt<120)
		trackxian5();

	while(hui[0] != 0 && hui[1] != 0)
		trackxian8();
	osDelay(100);
	{fp32 last_angle=0;
		angle_sum =0;
		last_angle=INS_angle_go[0]*180/3.1415;


		for(int i =0;;i++){
			go_yaw =INS_angle_go[0]*180/3.1415;
			go_yaw_inia = last_angle-go_yaw;
			if(rex_abs(go_yaw_inia)<20)
				angle_sum+=go_yaw_inia;
			last_angle = go_yaw;
			if(rex_abs(30)<rex_abs(angle_sum))
				break;
			set_current(7000,-1000,7000,-1000);
			osDelay(1);
		}
	}

	while(get_roll<15)
		trackxian8();

	while(get_roll>5)
		trackxian5();


	setspeed2(3000,3000);
	osDelay(300);
	stop();
	osDelay(200);

	turn_run(5000,170);

	yuyin(10);
	stop();
	osDelay(200);
    wave_handc78();
}
void go36()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 280)
        trackxian8();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (!(lukou_detect(5)))
        trackxian5();
    osDelay(50);
    stop();
    osDelay(500);
    if (tl_scan() == 0)
    {
        gomen16();
    }
    else
    {
        turn_run(5000, 165);
        stop();
        osDelay(150);
        set0rount();
        while (get_rount_cnt - intia_rount_cnt < 30)
            trackxian5();
        while (hui[14] != 0 && hui[15] != 0)
            trackxian5();

        {
            fp32 last_angle = 0;
            angle_sum = 0;
            last_angle = INS_angle_go[0] * 180 / 3.1415;


            for (int i = 0;; i++)
            {
                go_yaw = INS_angle_go[0] * 180 / 3.1415;
                go_yaw_inia = last_angle - go_yaw;
                if (rex_abs(go_yaw_inia) < 20)
                    angle_sum += go_yaw_inia;
                last_angle = go_yaw;
                if (rex_abs(125) < rex_abs(angle_sum))
                    break;
                set_current(6500, -1500, 6500, -1500);
                osDelay(1);
            }
        }
        set0rount();
        while (get_rount_cnt - intia_rount_cnt < 30)
            trackxian5();
        while (!(lukou_detect(5)))
            trackxian5();

        osDelay(50);
        stop();
        osDelay(500);
        if (tl_scan() == 0)
        {
            gomen26();
        }
        else
        {
            turn_run(5000, 165);
            stop();
            osDelay(150);
            set0rount();
            while (get_rount_cnt - intia_rount_cnt < 30)
                trackxian5();
            while (!(lukou_detect(4)))
                trackxian5();

            {
                fp32 last_angle = 0;
                angle_sum = 0;
                last_angle = INS_angle_go[0] * 180 / 3.1415;


                for (int i = 0;; i++)
                {
                    go_yaw = INS_angle_go[0] * 180 / 3.1415;
                    go_yaw_inia = last_angle - go_yaw;
                    if (rex_abs(go_yaw_inia) < 20)
                        angle_sum += go_yaw_inia;
                    last_angle = go_yaw;
                    if (rex_abs(140) < rex_abs(angle_sum))
                        break;
                    set_current(6500, -1500, 6500, -1500);
                    osDelay(1);
                }
            }

            set0rount();
            while (get_rount_cnt - intia_rount_cnt < 50)
                trackxian5();
            set0rount();
            while (get_rount_cnt - intia_rount_cnt < 140)
                trackxian8();

            while (!(lukou_detect(5)))
                trackxian5();

            {
                fp32 last_angle = 0;
                angle_sum = 0;
                last_angle = INS_angle_go[0] * 180 / 3.1415;


                for (int i = 0;; i++)
                {
                    go_yaw = INS_angle_go[0] * 180 / 3.1415;
                    go_yaw_inia = last_angle - go_yaw;
                    if (rex_abs(go_yaw_inia) < 20)
                        angle_sum += go_yaw_inia;
                    last_angle = go_yaw;
                    if (rex_abs(135) < rex_abs(angle_sum))
                        break;
                    set_current(6500, -1500, 6500, -1500);
                    osDelay(1);
                }
            }
            set0rount();
            while (get_rount_cnt - intia_rount_cnt < 30)
                trackxian5();
            while (!(lukou_detect(5)))
                trackxian5();

            osDelay(50);
            stop();
            osDelay(500);
            if (tl_scan() == 0)
            {
                gomen36();
            }
            else
            {
                gomen46();
            }
        }
    }
}

void go35()
{
    downplat();
    while (get_roll < -5)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();

    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 210)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 240)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    turnright();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 220)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 290)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    turnright();

    stop();
    osDelay(50);
    //	while(data_storage[12] != 0)
    //			trackxian5();
    for (int i = 0; i < 50; i++)
    {
        trackxian5();
        osDelay(1);
    }
    yuyin(10);
    stop();
    osDelay(100);
    setspeed2(-3000, -3000);
    osDelay(800);
    turn_run(3000, 85);
    while (!lukou_detect(5))
        trackxian5();

    turnleft();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(1.0f);

    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian(1.6f);

    while (get_rount_cnt - intia_rount_cnt < 220)
        trackxian(2.0f);

    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian(1.6f);

    while (get_rount_cnt - intia_rount_cnt < 280)
        trackxian(1.0f);

    while (!lukou_detect(5))
        trackxian5();

    turnleft();

    while (!lukou_detect(5))
        trackxian5();

    turnleft();

    stop();
    osDelay(100);

    //	while(data_storage[12] != 0)
    //			trackxian5();
    for (int i = 0; i < 50; i++)
    {
        trackxian5();
        osDelay(1);
    }

    yuyin(10);
    stop();
    osDelay(100);
    setspeed2(-3000, -3000);
    osDelay(800);
    turn_run(3000, -86);
    while (!lukou_detect(5))
        trackxian5();

    turnright();

    set0rount();
    while(get_rount_cnt-intia_rount_cnt<20)
        trackxian5();
    while(get_rount_cnt-intia_rount_cnt<50)
        trackxian8();
    while(get_rount_cnt-intia_rount_cnt<100)
        trackxian10();
    while(get_rount_cnt-intia_rount_cnt<270)
        trackxian12();
    while(get_rount_cnt-intia_rount_cnt<280)
        trackxian8();
    while (get_roll < 15)
        trackxian5();


    goplat();
    yuyin(5);

}

void go46()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 400)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (!(lukou_detect(5)))
        trackxian5();

    osDelay(50);
    stop();
    osDelay(500);
    if (tl_scan() == 0)
    {
        gomen16();
    }
    else
    {
        turn_run(5000, 165);
        stop();
        osDelay(150);
        set0rount();
        while (get_rount_cnt - intia_rount_cnt < 30)
            trackxian5();
        while (hui[14] != 0 && hui[15] != 0)
            trackxian5();

        {
            fp32 last_angle = 0;
            angle_sum = 0;
            last_angle = INS_angle_go[0] * 180 / 3.1415;


            for (int i = 0;; i++)
            {
                go_yaw = INS_angle_go[0] * 180 / 3.1415;
                go_yaw_inia = last_angle - go_yaw;
                if (rex_abs(go_yaw_inia) < 20)
                    angle_sum += go_yaw_inia;
                last_angle = go_yaw;
                if (rex_abs(125) < rex_abs(angle_sum))
                    break;
                set_current(6500, -1500, 6500, -1500);
                osDelay(1);
            }
        }
        set0rount();
        while (get_rount_cnt - intia_rount_cnt < 30)
            trackxian5();
        while (!(lukou_detect(5)))
            trackxian5();

        osDelay(50);
        stop();
        osDelay(500);
        if (tl_scan() == 0)
        {
            gomen26();
        }
        else
        {
            turn_run(5000, 165);
            stop();
            osDelay(150);
            set0rount();
            while (get_rount_cnt - intia_rount_cnt < 30)
                trackxian5();
            while (hui[14] != 0 && hui[15] != 0)
                trackxian5();

            {
                fp32 last_angle = 0;
                angle_sum = 0;
                last_angle = INS_angle_go[0] * 180 / 3.1415;


                for (int i = 0;; i++)
                {
                    go_yaw = INS_angle_go[0] * 180 / 3.1415;
                    go_yaw_inia = last_angle - go_yaw;
                    if (rex_abs(go_yaw_inia) < 20)
                        angle_sum += go_yaw_inia;
                    last_angle = go_yaw;
                    if (rex_abs(140) < rex_abs(angle_sum))
                        break;
                    set_current(6500, -1500, 6500, -1500);
                    osDelay(1);
                }
            }

            set0rount();
            while (get_rount_cnt - intia_rount_cnt < 50)
                trackxian5();
            set0rount();
            while (get_rount_cnt - intia_rount_cnt < 140)
                trackxian8();

            while (!(lukou_detect(5)))
                trackxian5();

            {
                fp32 last_angle = 0;
                angle_sum = 0;
                last_angle = INS_angle_go[0] * 180 / 3.1415;


                for (int i = 0;; i++)
                {
                    go_yaw = INS_angle_go[0] * 180 / 3.1415;
                    go_yaw_inia = last_angle - go_yaw;
                    if (rex_abs(go_yaw_inia) < 20)
                        angle_sum += go_yaw_inia;
                    last_angle = go_yaw;
                    if (rex_abs(135) < rex_abs(angle_sum))
                        break;
                    set_current(6500, -1500, 6500, -1500);
                    osDelay(1);
                }
            }
            set0rount();
            while (get_rount_cnt - intia_rount_cnt < 30)
                trackxian5();
            while (!(lukou_detect(5)))
                trackxian5();

            osDelay(50);
            stop();
            osDelay(500);
            if (tl_scan() == 0)
            {
                gomen36();
            }
            else
            {
                gomen46();
            }
        }
    }
}

void go45()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 400)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(300);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (!(lukou_detect(5)))
        trackxian5();

    osDelay(50);
    stop();
    osDelay(500);
    if (tl_scan() == 0)
    {
        gomen15();
    }
    else
    {
        turn_run(5000, 165);
        stop();
        osDelay(150);
        set0rount();
        while (get_rount_cnt - intia_rount_cnt < 30)
            trackxian5();
        while (hui[14] != 0 && hui[15] != 0)
            trackxian5();

        {
            fp32 last_angle = 0;
            angle_sum = 0;
            last_angle = INS_angle_go[0] * 180 / 3.1415;


            for (int i = 0;; i++)
            {
                go_yaw = INS_angle_go[0] * 180 / 3.1415;
                go_yaw_inia = last_angle - go_yaw;
                if (rex_abs(go_yaw_inia) < 20)
                    angle_sum += go_yaw_inia;
                last_angle = go_yaw;
                if (rex_abs(125) < rex_abs(angle_sum))
                    break;
                set_current(6500, -1500, 6500, -1500);
                osDelay(1);
            }
        }
        set0rount();
        while (get_rount_cnt - intia_rount_cnt < 30)
            trackxian5();
        while (!(lukou_detect(5)))
            trackxian5();

        osDelay(50);
        stop();
        osDelay(500);
        if (tl_scan() == 0)
        {
            gomen25();
        }
        else
        {
            turn_run(5000, 165);
            stop();
            osDelay(150);
            set0rount();
            while (get_rount_cnt - intia_rount_cnt < 30)
                trackxian5();
            while (hui[14] != 0 && hui[15] != 0)
                trackxian5();

            {
                fp32 last_angle = 0;
                angle_sum = 0;
                last_angle = INS_angle_go[0] * 180 / 3.1415;


                for (int i = 0;; i++)
                {
                    go_yaw = INS_angle_go[0] * 180 / 3.1415;
                    go_yaw_inia = last_angle - go_yaw;
                    if (rex_abs(go_yaw_inia) < 20)
                        angle_sum += go_yaw_inia;
                    last_angle = go_yaw;
                    if (rex_abs(140) < rex_abs(angle_sum))
                        break;
                    set_current(6500, -1500, 6500, -1500);
                    osDelay(1);
                }
            }

            set0rount();
            while (get_rount_cnt - intia_rount_cnt < 50)
                trackxian5();
            set0rount();
            while (get_rount_cnt - intia_rount_cnt < 140)
                trackxian8();

            while (!(lukou_detect(5)))
                trackxian5();

            {
                fp32 last_angle = 0;
                angle_sum = 0;
                last_angle = INS_angle_go[0] * 180 / 3.1415;


                for (int i = 0;; i++)
                {
                    go_yaw = INS_angle_go[0] * 180 / 3.1415;
                    go_yaw_inia = last_angle - go_yaw;
                    if (rex_abs(go_yaw_inia) < 20)
                        angle_sum += go_yaw_inia;
                    last_angle = go_yaw;
                    if (rex_abs(135) < rex_abs(angle_sum))
                        break;
                    set_current(6500, -1500, 6500, -1500);
                    osDelay(1);
                }
            }
            set0rount();
            while (get_rount_cnt - intia_rount_cnt < 30)
                trackxian5();
            while (!(lukou_detect(5)))
                trackxian5();

            osDelay(50);
            stop();
            osDelay(500);
            if (tl_scan() == 0)
            {
                gomen35();
            }
            else
            {
                gomen45();
            }
        }
    }
}

void go43()
{
    downplat();
    while (get_roll < -5)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian(1.2f);
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian(1.6f);
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian(2.0f);
    while (get_rount_cnt - intia_rount_cnt < 750)
        trackxian(2.5f);
    while (get_rount_cnt - intia_rount_cnt < 810)
        trackxian(2.0f);
    while (get_rount_cnt - intia_rount_cnt < 870)
        trackxian(1.6f);
    while (get_rount_cnt - intia_rount_cnt < 900)
        trackxian(1.2f);
    while (get_roll < 15)
        trackxian5();
    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);
    stop();
    osDelay(500);
    turn_run(5000, 170);
    stop();
    osDelay(50);
    yuyin(3);
    osDelay(50);
    wave_handc78();
}

void go315()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 280)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(150);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;

        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(125) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(50) < rex_abs(angle_sum))
                break;
            set_current(7000, 500, 7000, 500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();
    while (!(lukou_detect(5)))
        trackxian5();
    osDelay(200);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(25) < rex_abs(angle_sum))
                break;
            set_current(500, 7000, 500, 7000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian8();
    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);


    setspeed2(-2000, -2000);
    osDelay(300);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(4);
    osDelay(200);
    wave_handc78();
}

void go325()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 280)
        trackxian8();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(25) < rex_abs(angle_sum))
                break;
            set_current(7000, 500, 7000, 500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (!(lukou_detect(5)))
        trackxian5();
    osDelay(100);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(25) < rex_abs(angle_sum))
                break;
            set_current(500, 7000, 500, 7000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian8();
    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(300);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(4);
    osDelay(150);
    wave_handc78();
}

void go335()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 430)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(135) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian8();
    while (!(lukou_detect(5)))
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (!(lukou_detect(5)))
        trackxian5();
    osDelay(100);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(25) < rex_abs(angle_sum))
                break;
            set_current(500, 7000, 500, 7000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian8();
    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(300);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(4);
    osDelay(150);
    wave_handc78();
}

void go345()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 430)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(200);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(500, 7000, 500, 7000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (get_roll < 15)
        trackxian5();
    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);


    setspeed2(-2000, -2000);
    osDelay(300);

    turn_run(5000, 165);
    stop();
    osDelay(500);
    yuyin(4);
    osDelay(200);
    wave_handc78();
}


void go3men16()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;

        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;

        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    stop();
    osDelay(1000);
    while (hui[4] == 0 || hui[5] == 0 || hui[6] == 0 || hui[7] == 0 || hui[8] == 0 || hui[9] == 0 || hui[10] == 0 || hui
        [11] == 0)
        trackxian5();
    osDelay(250);
    stop();
    osDelay(150);
    turn_run(5000, 6);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 55)
        goqqb();
    stop();
    osDelay(2500);
    turn_run(5000, 8);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian3();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian3();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();
    while (get_roll < 15)
        trackxian3();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);


    setspeed2(-2000, -2000);
    osDelay(300);

    turn_run(5000, 165);
    stop();
    osDelay(150);
}

void go3men26()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(200);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;

        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 90)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(300);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    stop();
    osDelay(1000);
    while (hui[4] == 0 || hui[5] == 0 || hui[6] == 0 || hui[7] == 0 || hui[8] == 0 || hui[9] == 0 || hui[10] == 0 || hui
        [11] == 0)
        trackxian5();
    osDelay(250);
    stop();
    osDelay(150);
    turn_run(5000, 6);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 55)
        goqqb();
    stop();
    osDelay(2500);
    turn_run(5000, 8);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian3();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian3();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();
    while (get_roll < 15)
        trackxian3();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);


    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
}

void go3men36()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 400)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(135) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(300);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    stop();
    osDelay(1000);
    while (hui[4] == 0 || hui[5] == 0 || hui[6] == 0 || hui[7] == 0 || hui[8] == 0 || hui[9] == 0 || hui[10] == 0 || hui
        [11] == 0)
        trackxian5();
    osDelay(250);
    stop();
    osDelay(150);
    turn_run(5000, 6);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 55)
        goqqb();
    stop();
    osDelay(2500);
    turn_run(5000, 8);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian3();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian3();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();
    while (get_roll < 15)
        trackxian3();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);


    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
}

void go3men46()
{
    downplat();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 400)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(200);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1000, 6500, -1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1000, 6500, -1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(100);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    stop();
    osDelay(1000);
    while (hui[4] == 0 || hui[5] == 0 || hui[6] == 0 || hui[7] == 0 || hui[8] == 0 || hui[9] == 0 || hui[10] == 0 || hui
        [11] == 0)
        trackxian5();
    osDelay(250);
    stop();
    osDelay(150);
    turn_run(5000, 6);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 55)
        goqqb();
    stop();
    osDelay(2500);
    turn_run(5000, 8);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian3();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian3();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();
    while (get_roll < 15)
        trackxian3();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);


    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
}

void gomen1jia()
{
    go3men15();
    go517();
    go78();
    go86();
    go6men1jia();
}


void go3men15()
{
    downplat();
    while (get_roll < -5)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();

    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 270)
        trackxian8();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    turnright();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 290)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    turnright();

    stop();
    osDelay(50);

    //	while(data_storage[12] != 0)
    //			trackxian5();
    for (int i = 0; i < 90; i++)
    {
        trackxian5();
        osDelay(1);
    }
    stop();
    osDelay(100);
    yuyin(10);
    setspeed2(-3000, -3000);
    osDelay(950);
    stop();
    osDelay(50);
    turn_run(3000, 85);
    while (!lukou_detect(5))
        trackxian5();

    turnleft();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();

    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 230)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 270)
        trackxian5();

    while (!lukou_detect(5))
        trackxian5();

    turnleft();

    while (!lukou_detect(5))
        trackxian5();

    turnleft();

    stop();
    osDelay(50);

    //	while(data_storage[12] != 0)
    //			trackxian5();


    for (int i = 0; i < 90; i++)
    {
        trackxian5();
        osDelay(1);
    }
    stop();
    osDelay(50);
    yuyin(12);
    setspeed2(-3000, -3000);
    osDelay(950);
    stop();
    osDelay(50);
    turn_run(3000, -85);
    while (!lukou_detect(5))
        trackxian5();
    turnright();

    set0rount();
    while(get_rount_cnt-intia_rount_cnt<40)
        trackxian8();
    while(get_rount_cnt-intia_rount_cnt<60)
        trackxian12();
    while(get_rount_cnt-intia_rount_cnt<200)
        trackxian15();
    while(get_rount_cnt-intia_rount_cnt<230)
        trackxian12();
    while(get_rount_cnt-intia_rount_cnt<260)
        trackxian8();
    while (get_roll < 15)
        trackxian5();

    goplat();
    yuyin(5);

}

void go517()
{
    downplat();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian8();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 57.2974;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 57.2974;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(-1000, 8000, -1000, 8000);
            osDelay(1);
        }
        stop();
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 130)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian10();

    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian8();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(95) < rex_abs(angle_sum))
                break;
            set_current(-2000, 8000, -2000, 8000);
            osDelay(1);
        }
    }

    while (hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 && hui[14] != 0 &&
        hui[15] != 0)
        set_current(-1000, 8000, -1000, 8000);

    intia_rount_cnt = get_rount_cnt;
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();

    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();

    while (get_roll < 15)
        trackxian5();

    intia_rount_cnt = get_rount_cnt;
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian3();

    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian8();

    while (!lukou_detect(5))
        trackxian5();
    turnright();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();

    while (!lukou_detect(5))
        trackxian5();
    turnright();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian10();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian8();
    while (get_roll < 15)
        trackxian5();


    gohill();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 130)
        trackxian8();

    while (!(lukou_detect(5)))
        trackxian5();

    turnright();

    while (!(lukou_detect(5)))
        trackxian5();

    turnright();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian5();

    while (get_rount_cnt - intia_rount_cnt < 280)
        trackxian3();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian5();

    while (data_storage[12] != 0)
        trackxian3();

    //servo_pwm_set(1750, 2);

    setspeed2(3000, 3000);
    osDelay(500);
    setspeed2(-2000, -2000);
    osDelay(250);
    stop();
    osDelay(50);
    ////servo_pwm_set(2300, 0);
    osDelay(150);
    //servo_pwm_set(700, 1);


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 57.2974;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 57.2974;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 45)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(166) < rex_abs(angle_sum))
                break;
            set_current(-5000, 5000, -5000, 5000);
            osDelay(1);
        }
        stop();
    }

    stop();
    osDelay(50);

    yuyin(6);
}


void go6men1jia()
{
    downplat();

    while (!lukou_detect(4))
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(75) < rex_abs(angle_sum))
                break;
            set_current(-1000, 8000, -1000, 8000);
            osDelay(1);
        }
    }

    set0rount();

    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();


    while (hui[0] == 0 || hui[1] == 0 || hui[2] == 0 || hui[3] == 0 || hui[4] == 0 || hui[5] == 0 || hui[6] == 0 || hui[
            7] == 0 || hui[8] == 0 || hui[9] == 0 || hui[10] == 0 || hui[11] == 0 || hui[12] == 0 || hui[13] == 0 || hui
        [14]
        == 0 || hui[15] == 0)
        trackxian5();
    osDelay(120);


    fp32 last_angle = 0;
    angle_sum = 0;
    last_angle = INS_angle_go[0] * 57.2974;


    for (int i = 0;; i++)
    {
        go_yaw = INS_angle_go[0] * 57.2974;
        go_yaw_inia = last_angle - go_yaw;
        if (rex_abs(go_yaw_inia) < 45)
            angle_sum += go_yaw_inia;
        last_angle = go_yaw;
        if (rex_abs(8) < rex_abs(angle_sum))
            break;

        set_current(-1500, 3000, -1500, 3000);
        osDelay(1);
    }
    stop();


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 63)
        gobri();
    setspeed2(0, 5000);
    osDelay(100);
    stop();
    osDelay(1000);
    while (hui[7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 &&
        hui[14] != 0 && hui[15] != 0)
        setspeed2(-500, 5000);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 35)
        trackxian3();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(15) < rex_abs(angle_sum))
                break;
            set_current(5000, 8000, 5000, 8000);
            osDelay(1);
        }
    }


    //	while(hui[0] != 0 && hui[1] != 0 && data_storage[13] != 0)
    //		trackxian5();

    //	setspeed2(7000,5000);
    //	osDelay(50);


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 90)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian8();
    while (!lukou_detect(5))
        trackxian5();

    turnleft();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian15();
    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 190)
        trackxian10();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(70) < rex_abs(angle_sum))
                break;
            set_current(8000, 0, 8000, 0);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian8();
    osDelay(50);


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(1000, 8000, 1000, 8000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian3();

    while (!lukou_detect(4))
        trackxian5();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(25) < rex_abs(angle_sum))
                break;
            set_current(8000, 5000, 8000, 5000);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian8();

    while (get_roll > 5)
        trackxian5();


    setspeed2(3000, 3000);
    osDelay(300);
    stop();
    osDelay(200);

    turn_run(5000, 165);

    yuyin(9);
    stop();
    osDelay(200);
    //servo_pwm_set(1750, 2);
}

void xingomen1jia()
{
    downplat();
    intia_rount_cnt = get_rount_cnt;

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 190)
        trackxian10();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    turnright();


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 220)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 240)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian10();

    while (get_rount_cnt - intia_rount_cnt < 280)
        trackxian8();

    while (!lukou_detect(5))
        trackxian5();

    turnright();

    stop();
    osDelay(50);


    for (int i = 0; i < 300; i++)
    {
        trackxian5();
        osDelay(1);
    }

    yuyin(8);
    stop();
    osDelay(100);
    while (data_storage[16] != 0)
        setspeed2(-5000, -5000);
    stop();
    osDelay(100);

    turn_run(5000, 80);
    stop();
    osDelay(150);

    while (!lukou_detect(5))
        trackxian5();

    turnleft();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 220)
        trackxian10();

    while (get_rount_cnt - intia_rount_cnt < 230)
        trackxian8();

    while (!lukou_detect(5))
        trackxian5();

    turnleft();

    while (!lukou_detect(5))
        trackxian5();

    turnleft();

    stop();
    osDelay(50);


    for (int i = 0; i < 300; i++)
    {
        trackxian5();
        osDelay(1);
    }

    yuyin(8);
    stop();
    osDelay(100);

    while (data_storage[16] != 0)
        setspeed2(-5000, -5000);
    stop();
    osDelay(200);

    turn_run(5000, -70);
    stop();
    osDelay(150);

    while (!lukou_detect(5))
        trackxian5();

    turnright();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 220)
        trackxian12();


    while (get_roll < 15)
        trackxian8();


    goplat();
    yuyin(4);

    go517();
    go78();
    go86();
    go6men1jia();
}

void go3men25()
{
    downplat();
    while (get_roll < -10)
        trackxian5();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();

    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();

    while(get_rount_cnt-intia_rount_cnt<170)
    	trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 240)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(6000, -1000, 6000, -1000);
            osDelay(1);
        }
    }


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();

    // while(get_rount_cnt-intia_rount_cnt<80)
    // 	trackxian12();
    //
    while(get_rount_cnt-intia_rount_cnt<240)
    	trackxian15();
    //
    //  while(get_rount_cnt-intia_rount_cnt<220)
    // 	trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 270)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 300)
        trackxian8();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(20);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(-1000, 6000, -1000, 6000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();

    while(get_rount_cnt-intia_rount_cnt<60)
    	trackxian8();

    while(get_rount_cnt-intia_rount_cnt<200)
    	trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 230)
        trackxian8();

    while (get_roll < 15)
        trackxian5();

    goplat();
    yuyin(5);
}


void go58()
{
    downplat();
    servo_pwm_set(700, 0);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(500, 7000, 500, 7000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(150);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(20) < rex_abs(angle_sum))
                break;
            set_current(7000, 500, 7000, 500);
            osDelay(1);
        }
    }
    servo_pwm_set(2000, 0);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(100);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }
    servo_pwm_set(700, 0);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian5();
    while (!(lukou_detect(5)))
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 270)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian5();
    while (!(lukou_detect(5)))
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(100);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(110) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(150) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 90)
        trackxian3();
    while (data_storage[12] != 0)
        trackxian3();

    setspeed2(3000, 3000);
    osDelay(150);
    setspeed2(-2000, -2000);
    osDelay(300);
    stop();
    osDelay(1000);
    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(7);
    osDelay(150);
    wave_hand();
}

void go86()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (!(lukou_detect(4)))
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(145) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(100);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(105) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(-1500, 3000, -1500, 3000);
            osDelay(1);
        }
    }
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian3();
    while (hui[4] == 0 || hui[5] == 0 || hui[6] == 0 || hui[7] == 0 || hui[8] == 0 || hui[9] == 0 || hui[10] == 0 || hui
        [11] == 0)
        trackxian5();
    osDelay(250);
    stop();
    osDelay(150);
    turn_run(5000, 6);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 75)
        goqqb();
    stop();
    osDelay(2500);
    turn_run(5000, 8);
    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian3();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian3();
    osDelay(100);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian3();
    while (get_roll < 15)
        trackxian3();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);


    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 168);
    stop();
    osDelay(150);
    yuyin(5);
    osDelay(150);
    wave_hand();
}

void go85()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(145) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(100);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(105) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(100);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian8();
    while (!(lukou_detect(5)))
        trackxian5();
    osDelay(100);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(25) < rex_abs(angle_sum))
                break;
            set_current(500, 7000, 500, 7000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian8();
    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);


    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(4);
    osDelay(150);
    wave_handc78();
}

void go8men14()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(150);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(110) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 350)
        trackxian8();
    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(3);
    osDelay(150);
    wave_handc78();
}

void go8men13()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(150);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(110) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(2);
    osDelay(150);
    wave_handc78();
}

void go8men12()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(150);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(110) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(135) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();

    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(20) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(1);
    osDelay(150);
    wave_handc78();
}

void go8men24()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(150);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(110) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(100);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(135) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 350)
        trackxian8();
    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(3);
    osDelay(150);
    wave_handc78();
}

void go8men23()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(150);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(110) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(100);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(300);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(45) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(2);
    osDelay(150);
    wave_handc78();
}

void go8men22()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(150);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(110) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(100);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(135) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(135) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();

    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(20) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(1);
    osDelay(150);
    wave_handc78();
}

void go8men34()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(150);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(110) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(100);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 280)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(200);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(3);
    osDelay(150);
    wave_handc78();
}

void go8men33()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(150);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(110) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(100);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 280)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(130) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 450)
        trackxian8();
    set0rount();
    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(2);
    osDelay(150);
    wave_handc78();
}

void go8men32()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(150);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(110) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(100);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 280)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(130) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(300);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(20) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(1);
    osDelay(150);
    wave_handc78();
}


void go8men44()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(150);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(110) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(100);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(70) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(130) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(3);
    osDelay(150);
    wave_handc78();
}

void go8men43()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(150);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(110) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(100);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(70) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(130) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 400)
        trackxian8();
    set0rount();
    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(2);
    osDelay(150);
    wave_handc78();
}

void go8men42()
{
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian1();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(140) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();
    osDelay(150);
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(110) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(100);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(70) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(130) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian8();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();
    osDelay(300);

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(40) < rex_abs(angle_sum))
                break;
            set_current(6500, -1500, 6500, -1500);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian3();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian2();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 10)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(20) < rex_abs(angle_sum))
                break;
            set_current(-1500, 6500, -1500, 6500);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian5();

    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);

    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 165);
    stop();
    osDelay(150);
    yuyin(1);
    osDelay(150);
    wave_handc78();
}

void gomen2jia()
{
    go3men25();
    go57();
    go78();
    go86();
    go6men2jia();
}


void go6men2jia()
{
    downplat();

    while (!lukou_detect(4))
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(75) < rex_abs(angle_sum))
                break;
            set_current(-1000, 8000, -1000, 8000);
            osDelay(1);
        }
    }

    set0rount();

    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();


    while (hui[0] == 0 || hui[1] == 0 || hui[2] == 0 || hui[3] == 0 || hui[4] == 0 || hui[5] == 0 || hui[6] == 0 || hui[
            7] == 0 || hui[8] == 0 || hui[9] == 0 || hui[10] == 0 || hui[11] == 0 || hui[12] == 0 || hui[13] == 0 || hui
        [14]
        == 0 || hui[15] == 0)
        trackxian5();
    osDelay(120);


    fp32 last_angle = 0;
    angle_sum = 0;
    last_angle = INS_angle_go[0] * 57.2974;


    for (int i = 0;; i++)
    {
        go_yaw = INS_angle_go[0] * 57.2974;
        go_yaw_inia = last_angle - go_yaw;
        if (rex_abs(go_yaw_inia) < 45)
            angle_sum += go_yaw_inia;
        last_angle = go_yaw;
        if (rex_abs(9) < rex_abs(angle_sum))
            break;

        set_current(-1500, 3000, -1500, 3000);
        osDelay(1);
    }
    stop();


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 65)
        gobri();
    setspeed2(0, 5000);
    osDelay(100);
    stop();
    osDelay(1000);
    while (hui[7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 &&
        hui[14] != 0 && hui[15] != 0)
        setspeed2(-500, 5000);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 35)
        trackxian3();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(15) < rex_abs(angle_sum))
                break;
            set_current(5000, 8000, 5000, 8000);
            osDelay(1);
        }
    }


    //	while(hui[0] != 0 && hui[1] != 0 && data_storage[13] != 0)
    //		trackxian5();

    //	setspeed2(7000,5000);
    //	osDelay(50);


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 90)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian10();

    while (get_rount_cnt - intia_rount_cnt < 190)
        trackxian8();
    while (!lukou_detect(5))
        trackxian5();

    turnright();

    while (!lukou_detect(5))
        trackxian5();

    turnright();

    stop();
    osDelay(50);


    for (int i = 0; i < 300; i++)
    {
        trackxian5();
        osDelay(1);
    }

    yuyin(8);
    stop();
    osDelay(100);
    while (data_storage[15] != 0)
        setspeed2(-5000, -5000);
    stop();
    osDelay(200);

    turn_run(5000, -70);

    stop();
    osDelay(200);

    while (!lukou_detect(5))
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(8000, 1000, 8000, 1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 130)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian10();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian8();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-2000, 8000, -2000, 8000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian8();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(8000, -2000, 8000, -2000);
            osDelay(1);
        }
    }

    while (hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 && hui[14] != 0 &&
        hui[15] != 0)

        set_current(5000, -2000, 5000, -2000);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();


    while (hui[0] != 0 && hui[1] != 0)
        trackxian8();
    osDelay(50);


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(1000, 8000, 1000, 8000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian3();

    while (!lukou_detect(4))
        trackxian5();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(25) < rex_abs(angle_sum))
                break;
            set_current(8000, 5000, 8000, 5000);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian8();

    while (get_roll > 5)
        trackxian5();


    setspeed2(3000, 3000);
    osDelay(300);
    stop();
    osDelay(200);

    turn_run(5000, 165);

    yuyin(9);
    stop();
    osDelay(200);
    //servo_pwm_set(1750, 2);
}

void xingomen2jia()
{
    downplat();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 190)
        trackxian8();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(26) < rex_abs(angle_sum))
                break;
            set_current(7000, 1000, 7000, 1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian8();


    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 300)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 320)
        trackxian10();

    while (get_rount_cnt - intia_rount_cnt < 340)
        trackxian8();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(5000, 8000, 5000, 8000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 230)
        trackxian12();

    while (get_roll < 15)
        trackxian8();

    goplat();
    yuyin(4);


    go57();
    go78();
    go86();
    go6men2jia();
}


void go4men45()
{
    downplat();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 230)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian8();


    while (!(lukou_detect(4)))
        trackxian5();

    turnleft();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();

    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    turnleft();

    set0rount();
    while(get_rount_cnt-intia_rount_cnt<20)
        trackxian5();
    while(get_rount_cnt-intia_rount_cnt<50)
        trackxian8();
    while(get_rount_cnt-intia_rount_cnt<100)
        trackxian10();
    while(get_rount_cnt-intia_rount_cnt<270)
        trackxian12();
    while(get_rount_cnt-intia_rount_cnt<280)
        trackxian8();


    while (get_roll < 15)
        trackxian8();


    goplat();
    yuyin(5);
}

void go6men3jia()
{
    downplat();

    while (!lukou_detect(4))
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(75) < rex_abs(angle_sum))
                break;
            set_current(-1000, 8000, -1000, 8000);
            osDelay(1);
        }
    }

    set0rount();

    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();


    while (hui[0] == 0 || hui[1] == 0 || hui[2] == 0 || hui[3] == 0 || hui[4] == 0 || hui[5] == 0 || hui[6] == 0 || hui[
            7] == 0 || hui[8] == 0 || hui[9] == 0 || hui[10] == 0 || hui[11] == 0 || hui[12] == 0 || hui[13] == 0 || hui
        [14]
        == 0 || hui[15] == 0)
        trackxian5();
    osDelay(120);


    fp32 last_angle = 0;
    angle_sum = 0;
    last_angle = INS_angle_go[0] * 57.2974;


    for (int i = 0;; i++)
    {
        go_yaw = INS_angle_go[0] * 57.2974;
        go_yaw_inia = last_angle - go_yaw;
        if (rex_abs(go_yaw_inia) < 45)
            angle_sum += go_yaw_inia;
        last_angle = go_yaw;
        if (rex_abs(9) < rex_abs(angle_sum))
            break;

        set_current(-1500, 3000, -1500, 3000);
        osDelay(1);
    }
    stop();


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 65)
        gobri();
    setspeed2(0, 5000);
    osDelay(100);
    stop();
    osDelay(1000);
    while (hui[7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 &&
        hui[14] != 0 && hui[15] != 0)
        setspeed2(-500, 5000);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 35)
        trackxian3();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(15) < rex_abs(angle_sum))
                break;
            set_current(5000, 8000, 5000, 8000);
            osDelay(1);
        }
    }


    //	while(hui[0] != 0 && hui[1] != 0 && data_storage[13] != 0)
    //		trackxian5();

    //	setspeed2(7000,5000);
    //	osDelay(50);


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 90)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian10();
    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian8();
    while (!lukou_detect(5))
        trackxian5();

    turnright();

    while (!lukou_detect(5))
        trackxian5();

    turnright();

    stop();
    osDelay(50);


    for (int i = 0; i < 300; i++)
    {
        trackxian5();
        osDelay(1);
    }

    yuyin(8);
    stop();
    osDelay(100);
    while (data_storage[16] != 0)
        setspeed2(-5000, -5000);
    stop();
    osDelay(100);

    turn_run(5000, -70);

    stop();
    osDelay(200);

    while (!lukou_detect(5))
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(28) < rex_abs(angle_sum))
                break;
            set_current(8000, 1000, 8000, 1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 270)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 290)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 310)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 320)
        trackxian8();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(87) < rex_abs(angle_sum))
                break;
            set_current(-2000, 8000, -2000, 8000);
            osDelay(1);
        }
    }


    while (hui[7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 &&
        hui[14] != 0 && hui[15] != 0)
        set_current(-2000, 5000, -2000, 5000);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(8000, -2000, 8000, -2000);
            osDelay(1);
        }
    }

    while (hui[7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 &&
        hui[14] != 0 && hui[15] != 0)
        set_current(5000, -2000, 5000, -2000);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian3();

    while (!lukou_detect(4))
        trackxian5();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(25) < rex_abs(angle_sum))
                break;
            set_current(8000, 5000, 8000, 5000);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian8();

    while (get_roll > 5)
        trackxian5();


    setspeed2(3000, 3000);
    osDelay(300);
    stop();
    osDelay(200);

    turn_run(5000, 165);

    yuyin(9);
    stop();
    osDelay(200);
    //servo_pwm_set(1750, 2);
}


void go6men4jia()
{
    downplat();

    while (!lukou_detect(4))
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(75) < rex_abs(angle_sum))
                break;
            set_current(-1000, 8000, -1000, 8000);
            osDelay(1);
        }
    }

    set0rount();

    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();


    while (hui[0] == 0 || hui[1] == 0 || hui[2] == 0 || hui[3] == 0 || hui[4] == 0 || hui[5] == 0 || hui[6] == 0 || hui[
            7] == 0 || hui[8] == 0 || hui[9] == 0 || hui[10] == 0 || hui[11] == 0 || hui[12] == 0 || hui[13] == 0 || hui
        [14]
        == 0 || hui[15] == 0)
        trackxian5();
    osDelay(120);


    fp32 last_angle = 0;
    angle_sum = 0;
    last_angle = INS_angle_go[0] * 57.2974;


    for (int i = 0;; i++)
    {
        go_yaw = INS_angle_go[0] * 57.2974;
        go_yaw_inia = last_angle - go_yaw;
        if (rex_abs(go_yaw_inia) < 45)
            angle_sum += go_yaw_inia;
        last_angle = go_yaw;
        if (rex_abs(9) < rex_abs(angle_sum))
            break;

        set_current(-1500, 3000, -1500, 3000);
        osDelay(1);
    }
    stop();


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 65)
        gobri();
    setspeed2(0, 5000);
    osDelay(100);
    stop();
    osDelay(1000);
    while (hui[7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 &&
        hui[14] != 0 && hui[15] != 0)
        setspeed2(-500, 5000);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 35)
        trackxian3();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(15) < rex_abs(angle_sum))
                break;
            set_current(5000, 8000, 5000, 8000);
            osDelay(1);
        }
    }


    //	while(hui[0] != 0 && hui[1] != 0 && data_storage[13] != 0)
    //		trackxian5();

    //	setspeed2(7000,5000);
    //	osDelay(50);


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 90)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian8();
    while (!lukou_detect(5))
        trackxian5();


    turnright();

    while (!lukou_detect(5))
        trackxian5();

    turnright();

    stop();
    osDelay(50);


    for (int i = 0; i < 300; i++)
    {
        trackxian5();
        osDelay(1);
    }

    yuyin(8);
    stop();
    osDelay(100);
    while (data_storage[16] != 0)
        setspeed2(-5000, -5000);
    stop();
    osDelay(100);

    turn_run(5000, -70);

    stop();
    osDelay(200);

    while (!lukou_detect(5))
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(8000, 1000, 8000, 1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 110)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 130)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian10();

    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian8();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(60) < rex_abs(angle_sum))
                break;
            set_current(8000, -1000, 8000, -1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 110)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 130)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian10();

    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian8();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(-2000, 8000, -2000, 8000);
            osDelay(1);
        }
    }

    while (hui[7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 &&
        hui[14] != 0 && hui[15] != 0)
        setspeed2(-500, 5000);


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 130)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian10();

    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian8();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    turnleft();


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 90)
        trackxian8();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(8000, -2000, 8000, -2000);
            osDelay(1);
        }
    }

    while (hui[7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 &&
        hui[14] != 0 && hui[15] != 0)
        set_current(5000, -2000, 5000, -2000);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian3();

    while (!lukou_detect(4))
        trackxian5();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(25) < rex_abs(angle_sum))
                break;
            set_current(8000, 5000, 8000, 5000);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian8();

    while (get_roll > 5)
        trackxian5();


    setspeed2(3000, 3000);
    osDelay(300);
    stop();
    osDelay(200);

    turn_run(5000, 165);

    yuyin(9);
    stop();
    osDelay(200);

    //servo_pwm_set(1750, 2);
}

void go4men4jia()
{
    go4men45();
    go57();
    go78();
    go86();
    go6men43();
    go3jia();
}

void xingo4men4jia()
{
    downplat();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();


    while (!(lukou_detect(4)))
        trackxian5();

    turnleft();


    set0rount();

    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 190)
        trackxian10();

    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    turnleft();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 220)
        trackxian12();


    while (get_roll < 15)
        trackxian8();


    goplat();
    yuyin(4);

    go57();
    go78();
    go86();
    go6men43();
    go3jia();
}

void go4men3jia()
{
    go4men45();
    go57();
    go78();
    go86();
    go6men33();
    go3jia();
}

void xingo4men3jia()
{
    downplat();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();


    while (!(lukou_detect(4)))
        trackxian5();

    turnleft();


    set0rount();

    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 190)
        trackxian10();

    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    turnleft();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 220)
        trackxian12();


    while (get_roll < 15)
        trackxian8();


    goplat();
    yuyin(4);

    go57();
    go78();
    go86();
    go6men3jia();
}


void go3men2jia()
{
    go3men25();
    go57();
    go78();
    go86();
    go6men1jia();
}

void xingo3men2jia()
{
    downplat();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 190)
        trackxian8();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(28) < rex_abs(angle_sum))
                break;
            set_current(8000, 1000, 8000, 1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();


    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 220)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 240)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 260)
        trackxian8();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(1000, 8000, 1000, 8000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 270)
        trackxian12();

    while (get_roll < 15)
        trackxian8();

    goplat();
    yuyin(4);

    go57();
    go78();
    go86();
    go6men1jia();
}


void gozhimen1jia()
{
    while (!lukou_detect(4))
        trackxian5();

    turnright();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian8();
    osDelay(200);
    while (hui[0] != 0 && hui[1] != 0)
        trackxian8();

    setspeed2(10000, 8000);
    osDelay(60);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian10();
    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian8();
    while (!lukou_detect(5))
        trackxian5();

    turnleft();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 130)
        trackxian15();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian10();
    while (get_rount_cnt - intia_rount_cnt < 190)
        trackxian8();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(70) < rex_abs(angle_sum))
                break;
            set_current(8000, 0, 8000, 0);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian10();

    while (!lukou_detect(4))
        trackxian8();
    osDelay(50);


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(1000, 8000, 1000, 8000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 90)
        trackxian3();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(8000, 0, 8000, 0);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian8();

    while (get_roll > 5)
        trackxian5();


    setspeed2(3000, 3000);
    osDelay(300);
    stop();
    osDelay(200);

    turn_run(5000, 165);

    yuyin(9);
    stop();
    osDelay(200);
    //servo_pwm_set(1750, 2);
}


void gozhimen2jia()
{
    while (!lukou_detect(4))
        trackxian5();

    turnright();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian8();
    osDelay(200);
    while (hui[0] != 0 && hui[1] != 0)
        trackxian8();

    setspeed2(10000, 8000);
    osDelay(60);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian8();
    while (!lukou_detect(6))
        trackxian5();

    turnright();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian8();

    turnright();

    while (data_storage[12] != 0)
        trackxian5();

    yuyin(8);

    for (int i = 0; i < 200; i++)
    {
        trackxian5();
        osDelay(1);
    }
    stop();
    osDelay(100);
    while (data_storage[16] != 0)
        setspeed2(-5000, -5000);
    stop();
    osDelay(200);

    turn_run(5000, -70);

    stop();
    osDelay(200);

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(8000, 1000, 8000, 1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian8();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-2000, 8000, -2000, 8000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian8();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(8000, -2000, 8000, -2000);
            osDelay(1);
        }
    }

    while (hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 && hui[14] != 0 &&
        hui[15] != 0)

        set_current(5000, -2000, 5000, -2000);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian12();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian8();
    osDelay(50);


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(1000, 8000, 1000, 8000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 90)
        trackxian3();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(8000, 0, 8000, 0);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian8();

    while (get_roll > 5)
        trackxian5();


    setspeed2(3000, 3000);
    osDelay(300);
    stop();
    osDelay(200);

    turn_run(5000, 165);

    yuyin(9);
    stop();
    osDelay(200);
    //servo_pwm_set(1750, 2);
}


void gozhimen3jia()
{
    while (!lukou_detect(4))
        trackxian5();

    turnright();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian8();
    osDelay(200);
    while (hui[0] != 0 && hui[1] != 0)
        trackxian8();

    setspeed2(10000, 8000);
    osDelay(60);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian8();
    while (!lukou_detect(5))
        trackxian5();


    turnright();

    while (!lukou_detect(5))
        trackxian8();

    turnright();

    while (data_storage[12] != 0)
        trackxian5();

    yuyin(8);

    for (int i = 0; i < 200; i++)
    {
        trackxian5();
        osDelay(1);
    }
    stop();
    osDelay(100);
    while (data_storage[16] != 0)
        setspeed2(-5000, -5000);
    stop();
    osDelay(100);

    turn_run(5000, -70);

    stop();
    osDelay(200);

    while (hui[14] != 0 && hui[15] != 0)
        trackxian8();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(8000, 1000, 8000, 1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 280)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 300)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 320)
        trackxian8();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(-2000, 8000, -2000, 8000);
            osDelay(1);
        }
    }


    while (hui[7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 &&
        hui[14] != 0 && hui[15] != 0)
        set_current(-2000, 5000, -2000, 5000);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(8000, -2000, 8000, -2000);
            osDelay(1);
        }
    }

    while (hui[7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 &&
        hui[14] != 0 && hui[15] != 0)
        set_current(5000, -2000, 5000, -2000);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 90)
        trackxian3();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(8000, 0, 8000, 0);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian8();

    while (get_roll > 5)
        trackxian5();


    setspeed2(3000, 3000);
    osDelay(300);
    stop();
    osDelay(200);

    turn_run(5000, 165);

    yuyin(9);
    stop();
    osDelay(200);
    //servo_pwm_set(1750, 2);
}


void gozhimen4jia()
{
    while (!lukou_detect(4))
        trackxian5();

    turnright();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian8();
    osDelay(200);
    while (hui[0] != 0 && hui[1] != 0)
        trackxian8();

    setspeed2(10000, 8000);
    osDelay(60);


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian8();
    while (!lukou_detect(5))
        trackxian5();


    turnright();

    while (!lukou_detect(5))
        trackxian8();

    turnright();

    while (data_storage[12] != 0)
        trackxian5();

    yuyin(8);

    for (int i = 0; i < 200; i++)
    {
        trackxian5();
        osDelay(1);
    }
    stop();
    osDelay(100);
    while (data_storage[16] != 0)
        setspeed2(-5000, -5000);
    stop();
    osDelay(100);

    turn_run(5000, -70);

    stop();
    osDelay(200);

    while (hui[14] != 0 && hui[15] != 0)
        trackxian8();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(8000, 1000, 8000, 1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 110)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 130)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian8();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(60) < rex_abs(angle_sum))
                break;
            set_current(8000, -1000, 8000, -1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 110)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 130)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian8();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(-2000, 8000, -2000, 8000);
            osDelay(1);
        }
    }

    while (hui[7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 &&
        hui[14] != 0 && hui[15] != 0)
        setspeed2(-500, 5000);


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 130)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian8();

    while (!lukou_detect(5))
        trackxian5();

    turnleft();


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(8000, -2000, 8000, -2000);
            osDelay(1);
        }
    }

    while (hui[7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 &&
        hui[14] != 0 && hui[15] != 0)
        set_current(5000, -2000, 5000, -2000);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 90)
        trackxian3();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(8000, 0, 8000, 0);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian8();

    while (get_roll > 5)
        trackxian5();


    setspeed2(3000, 3000);
    osDelay(300);
    stop();
    osDelay(200);

    turn_run(5000, 165);

    yuyin(9);
    stop();
    osDelay(200);
    //servo_pwm_set(1750, 2);
}


void go4zhi3()
{
    //servo_pwm_set(1300, 2);
    //servo_pwm_set(1500, 1);
    //servo_pwm_set(1500, 0);
    downplat();
    while (hui[14] != 0 && hui[15] != 0)
        trackxian8();

    intia_rount_cnt = get_rount_cnt;
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 350)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 380)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 420)
        trackxian10();


    while (hui[0] != 0 && hui[1] != 0)
        trackxian8();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(23) < rex_abs(angle_sum))
                break;
            set_current(10000, 7000, 10000, 7000);
            osDelay(1);
        }
    }

    set0rount();

    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 190)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 210)
        trackxian10();


    while (data_storage[12] != 0)
        trackxian8();
    setspeed2(3000, 3000);
    osDelay(400);

    setspeed2(-2000, -2000);
    osDelay(200);

    turn_run(5000, 167);

    stop();
    osDelay(100);

    yuyin(8);

    set0rount();

    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 210)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 230)
        trackxian10();

    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian8();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(8000, -2000, 8000, -2000);
            osDelay(1);
        }
    }

    while (hui[0] != 0 && hui[1] != 0 && hui[2] != 0 && hui[3] != 0 && hui[4] != 0 && hui[5] != 0 && hui[6] != 0)
        set_current(5000, -2000, 5000, -2000);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian10();

    while (get_roll < 15)
        trackxian8();

    goplat();

    yuyin(2);
}


void gomen3jia()
{
    downplat();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();


    while (!(lukou_detect(4)))
        trackxian5();

    turnleft();

    set0rount();

    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    stop();
    osDelay(700);

    turn_run(6000, 165);

    stop();
    osDelay(150);
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
        trackxian5();
    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(110) < rex_abs(angle_sum))
                break;
            set_current(-1000, 8000, -1000, 8000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();

    while (!lukou_detect(5))
        trackxian8();
    osDelay(100);
    stop();
    osDelay(500);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();

    while (!lukou_detect(4))
        trackxian5();

    turnleft();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian10();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian8();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(5000, 8000, 5000, 8000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 230)
        trackxian12();

    while (get_roll < 15)
        trackxian8();

    goplat();
    yuyin(4);
    go57();
    go78();
    go86();
    go6men33();
    go3jia();
}


void go6men43()
{
    downplat();

    while (!lukou_detect(4))
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(75) < rex_abs(angle_sum))
                break;
            set_current(-1000, 8000, -1000, 8000);
            osDelay(1);
        }
    }

    set0rount();

    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();


    while (hui[0] == 0 || hui[1] == 0 || hui[2] == 0 || hui[3] == 0 || hui[4] == 0 || hui[5] == 0 || hui[6] == 0 || hui[
            7] == 0 || hui[8] == 0 || hui[9] == 0 || hui[10] == 0 || hui[11] == 0 || hui[12] == 0 || hui[13] == 0 || hui
        [14]
        == 0 || hui[15] == 0)
        trackxian5();
    osDelay(120);


    fp32 last_angle = 0;
    angle_sum = 0;
    last_angle = INS_angle_go[0] * 57.2974;


    for (int i = 0;; i++)
    {
        go_yaw = INS_angle_go[0] * 57.2974;
        go_yaw_inia = last_angle - go_yaw;
        if (rex_abs(go_yaw_inia) < 45)
            angle_sum += go_yaw_inia;
        last_angle = go_yaw;
        if (rex_abs(9) < rex_abs(angle_sum))
            break;

        set_current(-1500, 3000, -1500, 3000);
        osDelay(1);
    }
    stop();


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 65)
        gobri();
    setspeed2(0, 5000);
    osDelay(100);
    stop();
    osDelay(1000);
    while (hui[7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 &&
        hui[14] != 0 && hui[15] != 0)
        setspeed2(-500, 5000);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 35)
        trackxian3();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(15) < rex_abs(angle_sum))
                break;
            set_current(5000, 8000, 5000, 8000);
            osDelay(1);
        }
    }


    //	while(hui[0] != 0 && hui[1] != 0 && data_storage[13] != 0)
    //		trackxian5();

    //	setspeed2(7000,5000);
    //	osDelay(50);


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 90)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian8();
    while (!lukou_detect(5))
        trackxian5();


    turnright();

    while (!lukou_detect(5))
        trackxian5();

    turnright();

    stop();
    osDelay(50);


    for (int i = 0; i < 300; i++)
    {
        trackxian5();
        osDelay(1);
    }

    yuyin(8);
    stop();
    osDelay(100);
    while (data_storage[16] != 0)
        setspeed2(-5000, -5000);
    stop();
    osDelay(100);

    turn_run(5000, -70);

    stop();
    osDelay(200);

    while (!lukou_detect(5))
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(8000, 1000, 8000, 1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 110)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 130)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian10();

    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian8();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(60) < rex_abs(angle_sum))
                break;
            set_current(8000, -1000, 8000, -1000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 110)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 130)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian10();

    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian8();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(90) < rex_abs(angle_sum))
                break;
            set_current(-2000, 8000, -2000, 8000);
            osDelay(1);
        }
    }

    while (hui[7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 &&
        hui[14] != 0 && hui[15] != 0)
        setspeed2(-500, 5000);


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 130)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian10();

    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian8();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    turnleft();


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 400)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 460)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 500)
        trackxian10();

    while (get_roll < 15)
        trackxian8();

    goplat();
    yuyin(2);
}

void go3jia()
{
    downplat();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 90)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 240)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 280)
        trackxian10();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian8();
    osDelay(50);


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(30) < rex_abs(angle_sum))
                break;
            set_current(1000, 8000, 1000, 8000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 50)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian3();

    while (!lukou_detect(4))
        trackxian5();


    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(25) < rex_abs(angle_sum))
                break;
            set_current(8000, 5000, 8000, 5000);
            osDelay(1);
        }
    }

    while (get_roll < 15)
        trackxian8();

    while (get_roll > 5)
        trackxian5();


    setspeed2(3000, 3000);
    osDelay(300);
    stop();
    osDelay(200);

    turn_run(5000, 165);

    yuyin(9);
    stop();
    osDelay(200);
    //servo_pwm_set(1750, 2);
}

void go6men33()
{
    downplat();

    while (!lukou_detect(4))
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(75) < rex_abs(angle_sum))
                break;
            set_current(-1000, 8000, -1000, 8000);
            osDelay(1);
        }
    }

    set0rount();

    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();


    while (hui[0] == 0 || hui[1] == 0 || hui[2] == 0 || hui[3] == 0 || hui[4] == 0 || hui[5] == 0 || hui[6] == 0 || hui[
            7] == 0 || hui[8] == 0 || hui[9] == 0 || hui[10] == 0 || hui[11] == 0 || hui[12] == 0 || hui[13] == 0 || hui
        [14]
        == 0 || hui[15] == 0)
        trackxian5();
    osDelay(120);


    fp32 last_angle = 0;
    angle_sum = 0;
    last_angle = INS_angle_go[0] * 57.2974;


    for (int i = 0;; i++)
    {
        go_yaw = INS_angle_go[0] * 57.2974;
        go_yaw_inia = last_angle - go_yaw;
        if (rex_abs(go_yaw_inia) < 45)
            angle_sum += go_yaw_inia;
        last_angle = go_yaw;
        if (rex_abs(9) < rex_abs(angle_sum))
            break;

        set_current(-1500, 3000, -1500, 3000);
        osDelay(1);
    }
    stop();


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 65)
        gobri();
    setspeed2(0, 5000);
    osDelay(100);
    stop();
    osDelay(1000);
    while (hui[7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 &&
        hui[14] != 0 && hui[15] != 0)
        setspeed2(-500, 5000);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 35)
        trackxian3();


    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(15) < rex_abs(angle_sum))
                break;
            set_current(5000, 8000, 5000, 8000);
            osDelay(1);
        }
    }


    //	while(hui[0] != 0 && hui[1] != 0 && data_storage[13] != 0)
    //		trackxian5();

    //	setspeed2(7000,5000);
    //	osDelay(50);


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 90)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 170)
        trackxian10();
    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian8();
    while (!lukou_detect(5))
        trackxian5();

    turnright();

    while (!lukou_detect(5))
        trackxian5();

    turnright();

    stop();
    osDelay(50);


    for (int i = 0; i < 300; i++)
    {
        trackxian5();
        osDelay(1);
    }

    yuyin(8);
    stop();
    osDelay(100);
    while (data_storage[16] != 0)
        setspeed2(-5000, -5000);
    stop();
    osDelay(100);

    turn_run(5000, -70);

    stop();
    osDelay(200);

    while (!lukou_detect(5))
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(28) < rex_abs(angle_sum))
                break;
            set_current(8000, 1000, 8000, 1000);
            osDelay(1);
        }
    }
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian12();
    while (get_rount_cnt - intia_rount_cnt < 280)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 300)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 320)
        trackxian10();

    while (get_rount_cnt - intia_rount_cnt < 330)
        trackxian8();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(80) < rex_abs(angle_sum))
                break;
            set_current(-2000, 8000, -2000, 8000);
            osDelay(1);
        }
    }


    while (hui[7] != 0 && hui[8] != 0 && hui[9] != 0 && hui[10] != 0 && hui[11] != 0 && hui[12] != 0 && hui[13] != 0 &&
        hui[14] != 0 && hui[15] != 0)
        set_current(-2000, 5000, -2000, 5000);
    stop();
    osDelay(50);

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 420)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 460)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 500)
        trackxian10();

    while (get_roll < 15)
        trackxian8();

    goplat();
    yuyin(2);
}

void go3men1()
{
    downplat();
    while (get_roll < -5)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();

    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 220)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian8();

    // while (get_rount_cnt - intia_rount_cnt < 250)
    //     trackxian5();

    while (hui[14] != 0 && hui[15] != 0)
        trackxian5();

    turnright();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian5();

    while (!(lukou_detect(4)))
        trackxian5();
    stop();
    osDelay(50);
}

void xingomen3jia()
{
    downplat();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 160)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 180)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian8();


    while (!(lukou_detect(4)))
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(22) < rex_abs(angle_sum))
                break;
            set_current(3000, 8000, 3000, 8000);
            osDelay(1);
        }
    }


    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 70)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian10();

    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian8();


    while (!lukou_detect(4))
        trackxian5();

    turnleft();

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 120)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 140)
        trackxian10();
    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian8();

    while (hui[0] != 0 && hui[1] != 0)
        trackxian5();

    {
        fp32 last_angle = 0;
        angle_sum = 0;
        last_angle = INS_angle_go[0] * 180 / 3.1415;


        for (int i = 0;; i++)
        {
            go_yaw = INS_angle_go[0] * 180 / 3.1415;
            go_yaw_inia = last_angle - go_yaw;
            if (rex_abs(go_yaw_inia) < 20)
                angle_sum += go_yaw_inia;
            last_angle = go_yaw;
            if (rex_abs(35) < rex_abs(angle_sum))
                break;
            set_current(5000, 8000, 5000, 8000);
            osDelay(1);
        }
    }

    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian8();

    while (get_rount_cnt - intia_rount_cnt < 60)
        trackxian12();

    while (get_rount_cnt - intia_rount_cnt < 200)
        trackxian15();

    while (get_rount_cnt - intia_rount_cnt < 230)
        trackxian12();

    while (get_roll < 15)
        trackxian8();

    goplat();
    yuyin(4);

    go57();
    go78();
    go86();
    go6men33();
    go3jia();
}
