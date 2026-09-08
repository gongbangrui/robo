
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

/*
 * 机器人任务运行逻辑总览
 * ======================
 *
 * 本文件保留了比赛现场调试形成的“阻塞式动作序列”结构。run_task() 是唯一的任务入口，
 * 它根据红绿灯识别结果选择任务门，并依次调用各段路线函数。每个路线函数在满足传感器
 * 或里程条件前不会返回，因此调用顺序本身就是机器人的任务状态机。
 *
 * 控制链路分为四层：
 * 1. sensor_update() 轮询 16 路灰度和 3 路红外，并转换成旧任务代码使用的数据格式；
 * 2. trackxian*() 根据灰度位图给左右轮组设置不同转速，数字后缀表示大致基础转速；
 * 3. 转向、平台、桥面、舵机和语音函数封装单个阻塞动作；
 * 4. go*()/gomen*() 按赛道编号组合动作，run_task() 再组合成完整比赛流程。
 *
 * 路线函数常用的退出条件：
 * - get_rount_cnt 的差值：底盘累计行程达到经验阈值；
 * - hui[] 某些通道变黑：车头到达边线、弯道或路口；
 * - lukou_detect()：连续两次确认至少 det 路灰度触发，抑制瞬时噪声；
 * - data_storage[12..14]：前、左、右红外触发；
 * - get_roll：车体进入或离开坡面；
 * - angle_sum：积分相邻航向角增量，跨越正负 180 度时仍能累计实际转角。
 *
 * 注意：大量阈值、延时和轮速均是与实体赛道绑定的标定值。函数虽然采用普通 C 调用，
 * 实际语义是“执行到某个物理事件再进入下一状态”，阅读时应将每个 while 循环视为一个
 * 独立的行驶阶段。除 turn_run() 的 10 秒保护外，多数等待没有超时，传感器未触发会使
 * 当前任务一直停留在该阶段。
 */

/* INS 输出为弧度；任务层统一乘以 180/pi 转为角度。三个宏直接读取共享姿态数组。 */
#define get_yaw    INS_angle_go[0]*57.29578f
#define get_pitch  INS_angle_go[1]*57.29578f
#define get_roll   INS_angle_go[2]*57.29578f

/* 兼容旧任务代码的传感器别名；STOP 会停车并永久挂起当前任务流程。 */
#define sensor data_storage
#define STOP  stop(); while(1) osDelay(1);

/* 底盘累计圈数/里程计数的兼容名称，路线中仅使用相对差值。 */
#define get_rount_cnt chassis_round_cnt()


/* 灰度阵列触发通道达到该数量时，将兼容位 R4/R5 判定为有效。 */
#define R4_BLACK_CNT 14
#define R5_BLACK_CNT 14

/*
 * 可调参数索引（不改变默认值）：
 * - R4_BLACK_CNT/R5_BLACK_CNT：灰度“全黑”判定门槛，受传感器安装高度、线宽和阈值影响；
 *   误触发时提高，漏检时降低，建议每次只调整 1~2 路计数并现场记录。
 * - TRACK_PID_KP/KI/KD：连续循线的比例、积分、微分增益；先调 KP 消除跟线迟钝，再用 KD
 *   抑制摆动，最后用很小的 KI 消除长期偏差。TRACK_PID_MAX_WZ 限制角速度，MAX_IOUT
 *   限制积分累积，二者过大都可能导致高速出弯过冲。
 * - trackxian1/2/3/5/8/10/12/15 查表中的轮速：单位是底盘 RPM 目标值，数字越大越快；
 *   同一 case 的左右差值决定转向强度。应先确认电机额定转速和电池电压，再整体缩放。
 * - 路线中的 < 100、< 250 等里程阈值：底盘累计圈数/编码器计数，代表物理距离；需要按
 *   轮径、编码器分辨率或赛道实测距离重新标定。
 * - turn_run/turn2 传入的 speed、angle，以及各路线中的 20/35/80/130 等角度：分别是
 *   原地转向轮速和目标航向变化；角度误差大时优先检查 INS 零偏与正负方向，再修改数值。
 * - osDelay(50/100/200/...)：动作之间的稳定、机构到位或倒车持续时间，必须结合舵机响应和
 *   车体惯性调整；过短会导致动作未完成就进入下一状态，过长会降低比赛速度。
 * - 语音编号、舵机脉宽和红外通道索引与硬件接线绑定，除非同时更换硬件映射，否则不要改动。
 */


/*
 * 旧版任务层的传感器数据镜像。
 * hui[0..15]：单路灰度状态，0 表示检测到黑线，1 表示未检测到黑线；
 * HUI_data：灰度硬件 bitmask 按位取反后的 16 位模式，供查表式循线使用；
 * data_storage[12]：前红外，data_storage[13]：左红外，data_storage[14]：右红外；
 * data_storage[15]/[16]：由灰度触发数量模拟的 R4/R5 全黑检测位。
 * pt*、hld、zsp 和 xiansuo* 是旧视觉/线索协议保留的全局状态，本文件当前流程未写入它们。
 */
uint8_t data_storage[27] = {1};
uint8_t hui[16] = {0};
uint16_t HUI_data = 0;
uint8_t pt1 = 0, pt2 = 0, pt3 = 0;
uint8_t hld = 0x30;
uint8_t zsp = 0x31;
uint8_t xiansuo1 = 0, xiansuo2 = 0x32;
;


/*
 * 任务控制共享状态。
 * INS_angle_go 指向 INS 任务维护的欧拉角数组；go_yaw_inia 是本次采样与上次采样的航向差；
 * angle_sum 累计一次转向过程的航向变化；v_r/v_l 保存转向时的右、左轮组命令；
 * set_speed_rpm 按电机编号保存最终下发值；intia_rount_cnt 保存某一行驶阶段的里程起点。
 * 其余变量为旧接口兼容字段，仍由 run_task.h 对外公开。
 */
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


/*
 * 刷新任务层传感器快照。
 * 灰度驱动的 bitmask 中 1 表示硬件检测有效，旧路线代码却约定 hui[i]==0 表示压到黑线，
 * 因此这里同时生成取反的 HUI_data 和反相后的 hui[]。随后更新三路红外，并用灰度触发
 * 总数生成两个兼容全黑信号。调用者每次决策前必须执行本函数，否则使用的是旧快照。
 */
extern void sensor_update(void)
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`gs` 是灰度驱动只读快照；`bm` 是原始 16 位灰度 bitmask；`ir` 是红外驱动只读快照；`i` 仅为循环计数器；`HUI_data` 是反相后的 16
     *   路灰度位图。
     * - 检测条件：`data_storage[12、13、14、15、16]`（前红外；左红外；右红外；R4 全黑兼容位；R5
     *   全黑兼容位）。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：for 上限 `16`（循环次数，不自动等同距离）。
     * - 专项说明：`i=0..15` 是固定灰度通道扫描范围；`R4_BLACK_CNT/R5_BLACK_CNT` 是由 line_count 生成兼容全黑位的可调门槛。
     */
    /* gs 是灰度驱动的只读快照；bm 保留硬件原始位，后续所有兼容字段都由它派生。 */
    gray_sensor_poll();
    const gray_sensor_t* gs = get_gray_sensor_point();
    uint16_t bm = gs->bitmask;
    HUI_data = (uint16_t)(~bm);
    /* i 为灰度通道索引 0~15；右移和按位与提取每一路，再按旧逻辑反相。 */
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


/* 连续位置式循线 PID 参数。输出作为底盘角速度 wz，限制积分项以减小长时间偏线后的过冲。 */
#define TRACK_PID_KP 3.0f
#define TRACK_PID_KI 0.001f
#define TRACK_PID_KD 0.4f
#define TRACK_PID_MAX_WZ   4.0f
#define TRACK_PID_MAX_IOUT 0.3f
static pid_type_def track_pid;

/*
 * 采用灰度驱动提供的连续位置误差循线。
 * vx 是期望前向速度；当至少四路同时检测到线时，将其视作宽线/路口并保持直行，避免
 * 多通道触发时位置估计跳变。普通单线场景下，以 -position 为反馈计算角速度并交给底盘层。
 */
void trackxian(fp32 vx)
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：`vx` 是期望前向线速度，单位 m/s；直接交给 chassis_set_velocity()，可按直道、弯道和电池状态调节。
     * - 变量/状态：`gs` 是灰度驱动只读快照。
     * - 专项说明：`gs->line_count >= 4` 将宽线/路口视为直行；`4` 是可调判定门槛，`-gs->position` 是 PID 反馈方向，符号不可随意翻转。
     */
    /* vx 是前向线速度；gs->position 为归一化横向误差，PID 输出直接作为 wz。 */
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

/* 返回单精度浮点数的绝对值，供角度增量和累计转角比较使用。 */
fp32 rex_abs(fp32 a)
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：`a` 是待取绝对值的单精度浮点数；仅参与数学比较，不是现场标定参数。
     */
    if (a < 0)
        a = -a;
    return a;
}


/*
 * 本文件采用先声明、后定义的历史布局。以下原型既包含当前主流程使用的路线，也包含
 * 备用任务路线；命名中的数字通常表示赛道区域，men 表示任务门，jia 表示附加任务路线。
 *
 * 路线函数体中的局部变量遵循固定约定：
 * - intia_rount_cnt：每个阶段开始时由 set0rount() 写入，避免把上一阶段的里程带入本阶段；
 * - get_rount_cnt-intia_rount_cnt：当前阶段已行驶的相对计数，while 上限就是该段距离标定；
 * - hui[k]：等待指定灰度通道压线，条件中的 && 表示所有通道仍未触发，任一触发即继续；
 * - data_storage[12..16]：等待前/侧红外或 R4/R5 全黑，通常用于目标、坡面或平台边缘定位；
 * - last_angle、go_yaw、go_yaw_inia、angle_sum：原地转向的局部积分器。last_angle 保存上一
 *   次航向，go_yaw 是当前航向，go_yaw_inia 是差值，angle_sum 是累计角度；for(;;) 的
 *   break 条件就是目标角度，20/45 度是异常采样过滤阈值，可随 IMU 噪声调整；
 * - for(int i=0; i<N; i++) 中的 i 只用于重复 N 次 1 ms 循线，N 是时间而非距离；
 * - setspeed2(a,b) 中 a/b 是左右轮组 RPM，负值代表倒车；不对称值用于纠偏或原地转向；
 * - servo_pwm_set(pulse,ch) 的 pulse 单位为微秒，1500 通常为中位，500/2500 为两端，
 *   具体范围必须服从舵机规格，修改前应先限制机械行程；
 * - yuyin(n) 的 n 是语音轨道编号，属于比赛交互协议，不能当作速度或路线编号理解。
 *
 * 调整路线时建议遵循“先距离、再角度、后速度、最后延时”的顺序：距离错误会造成后续
 * 所有触发点整体偏移；角度错误会改变进入支路的方向；速度和延时只影响通过时机和惯性。
 */
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

/* 将左右轮组的目标转速分别复制到前后电机，形成标准差速驱动命令。 */
void setspeed2(int speed1, int speed2)
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：`speed1`/`speed2` 分别写入左侧 m1/m3 和右侧 m2/m4 的 RPM 目标；正值前进、负值倒车，差值决定转向强度。
     */
    /* speed1/speed2：左、右轮组目标 RPM；正值前进、负值倒车。数值可按电池电压和地面
     * 摩擦调整，必须保持同一轮组前后电机使用相同值。 */
    set_current(speed1, speed2, speed1, speed2);
}

/* 7→8 特定路段的差速补偿：后侧两个电机在对应轮组目标上额外减 1000。 */
void setspeed78(int speed1, int speed2)
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：`speed1`/`speed2` 分别写入左侧 m1/m3 和右侧 m2/m4 的 RPM 目标；正值前进、负值倒车，差值决定转向强度。
     * - 专项说明：后侧 m3/m4 在基准 RPM 上减去 `1000`，这是桥面段四轮补偿量，可按实际轮速差调整；电机归属不可换位。
     */
    /* speed1/speed2 是 7→8 段左右轮组基准 RPM；后轮各减 1000 用于补偿桥面负载，
     * 该补偿量可依据四轮实际速度差重新标定。 */
    set_current(speed1, speed2, speed1-1000, speed2-1000);
}

/*
 * 固定轮速向一个方向转约 80 度。
 * 每毫秒读取一次航向，相邻采样差小于 20 度才纳入 angle_sum，以过滤姿态突跳。该函数
 * 只结束轮速循环，不主动下发零速，调用路线需紧接 stop() 或下一条运动命令。
 */
void turnleft()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`angle_sum` 累加有效航向增量以判断目标转角；`go_yaw_inia`
     *   是相邻航向采样差。
     * - 数值：积分目标角 `80` 度；航向跳变过滤 `20` 度；四轮 RPM `-1000/8000/-1000/8000`（m1/m2/m3/m4）；时序 `1` ms。
     * - 可调项：目标转角、IMU 跳变过滤门槛、RPM/循线档、动作时序均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     * - 专项说明：`180/3.1415` 是弧度转角度近似系数，不是赛道参数；`20` 是单次 IMU 跳变过滤阈值，转向 RPM 和目标角可调。
     */
    /* last_angle/go_yaw/go_yaw_inia/angle_sum 均为角度值（度）；80 是目标转角，20 是
     * 单次采样滤波阈值，-1000/8000 是左右轮反向转向 RPM，均可按车体转向半径调整。 */
    {
        fp32 last_angle = 0;
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

/* 与 turnleft() 对称的约 80 度转向，左右轮组命令方向相反；返回前同样不会主动停车。 */
void turnright()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`angle_sum` 累加有效航向增量以判断目标转角；`go_yaw_inia`
     *   是相邻航向采样差。
     * - 数值：积分目标角 `80` 度；航向跳变过滤 `20` 度；四轮 RPM `8000/-1000/8000/-1000`（m1/m2/m3/m4）；时序 `1` ms。
     * - 可调项：目标转角、IMU 跳变过滤门槛、RPM/循线档、动作时序均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     * - 专项说明：`180/3.1415` 是弧度转角度近似系数，不是赛道参数；`20` 是单次 IMU 跳变过滤阈值，转向 RPM 和目标角可调。
     */
    /* 与 turnleft() 参数含义相同，轮速符号镜像后实现相反方向转动。 */
    {
        fp32 last_angle = 0;
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

/*
 * 回到终点/停车区：循线检测上坡和回平，短距离前冲后停车，再掉头约 165 度、播放 9 号语音。
 * 两个 roll 阈值依次确认“已经上坡”和“已经回到近水平面”，防止只凭里程误判终点位置。
 */
void gohome()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 检测条件：横滚角 `get_roll<15、>2` 度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：通用转向 `5000 RPM / 165 度`；左右轮组 RPM `3000/3000`；循线档 `trackxian5`；时序 `300、200` ms；语音轨道 `9`。
     * - 可调项：目标转角、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/*
 * 对路口进行双采样消抖。
 * 每次统计 16 路中值为 0 的通道数；若不少于 det，则记为一次命中。首次命中后等待 10 ms
 * 再统计，只有两次都命中才返回 1。函数本身不刷新传感器，因此它验证的是 hui[] 当前快照；
 * 通常由紧邻的 trackxian*() 在循环的上一次迭代中更新。
 */
int lukou_detect(int det)
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：`det` 是一次采样判为黑线所需的最少灰度通道数；应按线宽、传感器高度和噪声调整。
     * - 变量/状态：`num` 统计当前黑线通道数；`lukou_num` 统计双采样命中次数；`i` 仅为循环计数器；`j` 遍历 0..15 灰度通道。
     * - 数值：时序 `10` ms；for 上限 `2、16`（循环次数，不自动等同距离）。
     * - 可调项：动作时序均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     * - 专项说明：固定扫描两次（`i<2`）并遍历 16 路（`j<16`）；`10 ms` 是两次确认间隔，`det` 才是主要现场门槛。
     */
    /* det 是一次路口判定所需的黑线通道数；i=0/1 表示连续两次采样，j=0..15 遍历灰度通道，
     * 10 ms 是两次采样间隔。det 和间隔应根据赛道线宽及传感器噪声调节。 */
    int num = 0;
    int lukou_num = 0;
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 16; j++)
        {
            if (hui[j] == 0)
                num++;
        }
        if (num >= (det))
        {
            lukou_num++;
            if (lukou_num == 1) osDelay(10);
        }
        num = 0;
    }
    if (lukou_num >= 2) return 1;

    return 0;
}

/*
 * 低速离散循线，基础轮速 1000。
 * HUI_data 表示 16 路灰度的组合模式：中部命中时两侧同速，黑线越偏向一侧，就越降低
 * 对应内侧轮组速度以增大转向量；未匹配到已标定模式时保持直行。其余 trackxianN()
 * 使用相同位图查表策略，仅基础速度和不同偏差等级下的差速量不同。
 */
void trackxian1()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`HUI_data` 是反相后的 16 路灰度位图。
     * - 数值：左右轮组 RPM `1000/1000、1000/500、500/1000、1000/0、0/1000`。
     * - 可调项：RPM/循线档均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     * - 专项说明：本函数先刷新传感器。 各 `case 0x....` 是硬件灰度位图编码，通常固定；每个 case 的 RPM 差速表才是可调标定值。
     */
    /* HUI_data 的每个 case 是一组已标定的左右轮 RPM；这里只做一次查表和下发。 */
    sensor_update();

    switch (HUI_data)
    {

    case 0xFE7F: setspeed2(1000, 1000);
        break;
    case 0xFC7F: setspeed2(1000, 500);
        break;
    case 0xFE3F: setspeed2(500, 1000);
        break;
    case 0xFEFF: setspeed2(1000, 500);
        break;
    case 0xFCFF: setspeed2(1000, 500);
        break;
    case 0xFF7F: setspeed2(500, 1000);
        break;
    case 0xFF3F: setspeed2(500, 1000);
        break;
    case 0xF8FF: setspeed2(1000, 500);
        break;
    case 0xFF1F: setspeed2(500, 1000);
        break;

    case 0xFDFF: setspeed2(1000, 0);
        break;
    case 0xF9FF: setspeed2(1000, 0);
        break;
    case 0xFFBF: setspeed2(0, 1000);
        break;
    case 0xFF9F: setspeed2(0, 1000);
        break;
    case 0xF1FF: setspeed2(1000, 0);
        break;
    case 0xFF8F: setspeed2(0, 1000);
        break;

    case 0xFBFF: setspeed2(1000, 0);
        break;
    case 0xF3FF: setspeed2(1000, 0);
        break;
    case 0xFFDF: setspeed2(0, 1000);
        break;
    case 0xFFCF: setspeed2(0, 1000);
        break;
    case 0xE3FF: setspeed2(1000, 0);
        break;
    case 0xFFC7: setspeed2(0, 1000);
        break;

    case 0xF7FF: setspeed2(1000, 0);
        break;
    case 0xE7FF: setspeed2(1000, 0);
        break;
    case 0xFFEF: setspeed2(0, 1000);
        break;
    case 0xFFE7: setspeed2(0, 1000);
        break;
    case 0xC7FF: setspeed2(1000, 0);
        break;
    case 0xFFE3: setspeed2(0, 1000);
        break;

    case 0xEFFF: setspeed2(1000, 0);
        break;
    case 0xCFFF: setspeed2(1000, 0);
        break;
    case 0xFFF7: setspeed2(0, 1000);
        break;
    case 0xFFF3: setspeed2(0, 1000);
        break;
    case 0x8FFF: setspeed2(1000, 0);
        break;
    case 0xFFF1: setspeed2(0, 1000);
        break;

    case 0xFFFB: setspeed2(0, 1000);
        break;
    case 0xFFF9: setspeed2(0, 1000);
        break;
    case 0xDFFF: setspeed2(1000, 0);
        break;
    case 0x9FFF: setspeed2(1000, 0);
        break;
    case 0xFFF8: setspeed2(0, 1000);
        break;
    case 0x1FFF: setspeed2(1000, 0);
        break;

    case 0xFFFD: setspeed2(0, 1000);
        break;
    case 0xFFFC: setspeed2(0, 1000);
        break;
    case 0xBFFF: setspeed2(1000, 0);
        break;
    case 0x3FFF: setspeed2(1000, 0);
        break;
    case 0xFFFE: setspeed2(0, 1000);
        break;
    case 0x7FFF: setspeed2(1000, 0);
        break;
    default: setspeed2(1000, 1000);
        break;
    }
}




/* 中低速循线，基础轮速 3000；用于普通直线和坡道前后的稳定过渡。 */
void trackxian3()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`HUI_data` 是反相后的 16 路灰度位图。
     * - 数值：左右轮组 RPM
     *   `3000/3000、3000/2500、2500/3000、3000/2000、2000/3000、3000/1500、1500/3000、3000/1000、1000/3000、3000/500、500/3000、0/3000、3000/0`。
     * - 可调项：RPM/循线档均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     * - 专项说明：本函数先刷新传感器。 各 `case 0x....` 是硬件灰度位图编码，通常固定；每个 case 的 RPM 差速表才是可调标定值。
     */
    /* 每个 case 的两个整数是左右轮 RPM；3000 为直线基础速度，差值 500~2500 为偏线
     * 修正量。需要更快通过时整体提高，出现摆动时优先减小差值。 */
    sensor_update();

    switch (HUI_data)
    {

    case 0xFE7F: setspeed2(3000, 3000);
        break;
    case 0xFC7F: setspeed2(3000, 2500);
        break;
    case 0xFE3F: setspeed2(2500, 3000);
        break;
    case 0xFEFF: setspeed2(3000, 2500);
        break;
    case 0xFCFF: setspeed2(3000, 2500);
        break;
    case 0xFF7F: setspeed2(2500, 3000);
        break;
    case 0xFF3F: setspeed2(2500, 3000);
        break;
    case 0xF8FF: setspeed2(3000, 2500);
        break;
    case 0xFF1F: setspeed2(2500, 3000);
        break;

    case 0xFDFF: setspeed2(3000, 2000);
        break;
    case 0xF9FF: setspeed2(3000, 2000);
        break;
    case 0xFFBF: setspeed2(2000, 3000);
        break;
    case 0xFF9F: setspeed2(2000, 3000);
        break;
    case 0xF1FF: setspeed2(3000, 2000);
        break;
    case 0xFF8F: setspeed2(2000, 3000);
        break;

    case 0xFBFF: setspeed2(3000, 1500);
        break;
    case 0xF3FF: setspeed2(3000, 1500);
        break;
    case 0xFFDF: setspeed2(1500, 3000);
        break;
    case 0xFFCF: setspeed2(1500, 3000);
        break;
    case 0xE3FF: setspeed2(3000, 1500);
        break;
    case 0xFFC7: setspeed2(1500, 3000);
        break;

    case 0xF7FF: setspeed2(3000, 1000);
        break;
    case 0xE7FF: setspeed2(3000, 1000);
        break;
    case 0xFFEF: setspeed2(1000, 3000);
        break;
    case 0xFFE7: setspeed2(1000, 3000);
        break;
    case 0xC7FF: setspeed2(3000, 1000);
        break;
    case 0xFFE3: setspeed2(1000, 3000);
        break;

    case 0xEFFF: setspeed2(3000, 500);
        break;
    case 0xCFFF: setspeed2(3000, 500);
        break;
    case 0xFFF7: setspeed2(500, 3000);
        break;
    case 0xFFF3: setspeed2(500, 3000);
        break;
    case 0x8FFF: setspeed2(3000, 500);
        break;
    case 0xFFF1: setspeed2(500, 3000);
        break;

    case 0xFFFB: setspeed2(0, 3000);
        break;
    case 0xFFF9: setspeed2(0, 3000);
        break;
    case 0xDFFF: setspeed2(3000, 0);
        break;
    case 0x9FFF: setspeed2(3000, 0);
        break;
    case 0xFFF8: setspeed2(0, 3000);
        break;
    case 0x1FFF: setspeed2(3000, 0);
        break;

    case 0xFFFD: setspeed2(0, 3000);
        break;
    case 0xFFFC: setspeed2(0, 3000);
        break;
    case 0xBFFF: setspeed2(3000, 0);
        break;
    case 0x3FFF: setspeed2(3000, 0);
        break;
    case 0xFFFE: setspeed2(0, 3000);
        break;
    case 0x7FFF: setspeed2(3000, 0);
        break;
    default: setspeed2(3000, 3000);
        break;
    }
}


/* 过渡速度循线，基础轮速 2000；常用于转弯后重新压线和短距离校正。 */
void trackxian2()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`HUI_data` 是反相后的 16 路灰度位图。
     * - 数值：左右轮组 RPM
     *   `2000/2000、2000/1500、1500/2000、2000/1000、1000/2000、2000/500、500/2000、0/2000、2000/0`。
     * - 可调项：RPM/循线档均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     * - 专项说明：本函数先刷新传感器。 各 `case 0x....` 是硬件灰度位图编码，通常固定；每个 case 的 RPM 差速表才是可调标定值。
     */
    /* 2000 为低速基础 RPM，适合转弯后重新捕线；查表差速可按弯道半径和车速重新标定。 */
    sensor_update();

    switch (HUI_data)
    {

    case 0xFE7F: setspeed2(2000, 2000);
        break;
    case 0xFC7F: setspeed2(2000, 1500);
        break;
    case 0xFE3F: setspeed2(1500, 2000);
        break;
    case 0xFEFF: setspeed2(2000, 1500);
        break;
    case 0xFCFF: setspeed2(2000, 1500);
        break;
    case 0xFF7F: setspeed2(1500, 2000);
        break;
    case 0xFF3F: setspeed2(1500, 2000);
        break;
    case 0xF8FF: setspeed2(2000, 1500);
        break;
    case 0xFF1F: setspeed2(1500, 2000);
        break;

    case 0xFDFF: setspeed2(2000, 1000);
        break;
    case 0xF9FF: setspeed2(2000, 1000);
        break;
    case 0xFFBF: setspeed2(1000, 2000);
        break;
    case 0xFF9F: setspeed2(1000, 2000);
        break;
    case 0xF1FF: setspeed2(2000, 1000);
        break;
    case 0xFF8F: setspeed2(1000, 2000);
        break;

    case 0xFBFF: setspeed2(2000, 500);
        break;
    case 0xF3FF: setspeed2(2000, 500);
        break;
    case 0xFFDF: setspeed2(500, 2000);
        break;
    case 0xFFCF: setspeed2(500, 2000);
        break;
    case 0xE3FF: setspeed2(2000, 500);
        break;
    case 0xFFC7: setspeed2(500, 2000);
        break;

    case 0xF7FF: setspeed2(2000, 500);
        break;
    case 0xE7FF: setspeed2(2000, 500);
        break;
    case 0xFFEF: setspeed2(500, 2000);
        break;
    case 0xFFE7: setspeed2(500, 2000);
        break;
    case 0xC7FF: setspeed2(2000, 500);
        break;
    case 0xFFE3: setspeed2(500, 2000);
        break;

    case 0xEFFF: setspeed2(2000, 500);
        break;
    case 0xCFFF: setspeed2(2000, 500);
        break;
    case 0xFFF7: setspeed2(500, 2000);
        break;
    case 0xFFF3: setspeed2(500, 2000);
        break;
    case 0x8FFF: setspeed2(2000, 500);
        break;
    case 0xFFF1: setspeed2(500, 2000);
        break;

    case 0xFFFB: setspeed2(0, 2000);
        break;
    case 0xFFF9: setspeed2(0, 2000);
        break;
    case 0xDFFF: setspeed2(2000, 0);
        break;
    case 0x9FFF: setspeed2(2000, 0);
        break;
    case 0xFFF8: setspeed2(0, 2000);
        break;
    case 0x1FFF: setspeed2(2000, 0);
        break;

    case 0xFFFD: setspeed2(0, 2000);
        break;
    case 0xFFFC: setspeed2(0, 2000);
        break;
    case 0xBFFF: setspeed2(2000, 0);
        break;
    case 0x3FFF: setspeed2(2000, 0);
        break;
    case 0xFFFE: setspeed2(0, 2000);
        break;
    case 0x7FFF: setspeed2(2000, 0);
        break;
    default: setspeed2(2000, 2000);
        break;
    }
}


/* 常用中速循线，基础轮速 5000；绝大多数路线的默认巡航档位。 */
void trackxian5()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`HUI_data` 是反相后的 16 路灰度位图。
     * - 数值：左右轮组 RPM
     *   `5000/5000、5000/4000、4000/5000、5000/3500、3500/5000、5000/3000、3000/5000、5000/2000、2000/5000、1000/5000、5000/1000、0/5000、5000/0`。
     * - 可调项：RPM/循线档均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     * - 专项说明：本函数先刷新传感器。 各 `case 0x....` 是硬件灰度位图编码，通常固定；每个 case 的 RPM 差速表才是可调标定值。
     */
    /* 5000 为常用巡航 RPM；左右速度差越大，转向越急。该表是现场调参的主要位置之一。 */
    sensor_update();

    switch (HUI_data)
    {

    case 0xFE7F: setspeed2(5000, 5000);
        break;
    case 0xFC7F: setspeed2(5000, 4000);
        break;
    case 0xFE3F: setspeed2(4000, 5000);
        break;
    case 0xFEFF: setspeed2(5000, 3500);
        break;
    case 0xFCFF: setspeed2(5000, 3500);
        break;
    case 0xFF7F: setspeed2(3500, 5000);
        break;
    case 0xFF3F: setspeed2(3500, 5000);
        break;
    case 0xF8FF: setspeed2(5000, 3500);
        break;
    case 0xFF1F: setspeed2(3500, 5000);
        break;

    case 0xFDFF: setspeed2(5000, 3000);
        break;
    case 0xF9FF: setspeed2(5000, 3000);
        break;
    case 0xFFBF: setspeed2(3000, 5000);
        break;
    case 0xFF9F: setspeed2(3000, 5000);
        break;
    case 0xF1FF: setspeed2(5000, 3000);
        break;
    case 0xFF8F: setspeed2(3000, 5000);
        break;

    case 0xFBFF: setspeed2(5000, 3000);
        break;
    case 0xF3FF: setspeed2(5000, 3000);
        break;
    case 0xFFDF: setspeed2(3000, 5000);
        break;
    case 0xFFCF: setspeed2(3000, 5000);
        break;
    case 0xE3FF: setspeed2(5000, 3000);
        break;
    case 0xFFC7: setspeed2(3000, 5000);
        break;

    case 0xF7FF: setspeed2(5000, 2000);
        break;
    case 0xE7FF: setspeed2(5000, 2000);
        break;
    case 0xFFEF: setspeed2(2000, 5000);
        break;
    case 0xFFE7: setspeed2(2000, 5000);
        break;
    case 0xC7FF: setspeed2(5000, 2000);
        break;
    case 0xFFE3: setspeed2(2000, 5000);
        break;

    case 0xEFFF: setspeed2(5000, 2000);
        break;
    case 0xCFFF: setspeed2(5000, 2000);
        break;
    case 0xFFF7: setspeed2(2000, 5000);
        break;
    case 0xFFF3: setspeed2(2000, 5000);
        break;
    case 0x8FFF: setspeed2(5000, 2000);
        break;
    case 0xFFF1: setspeed2(2000, 5000);
        break;

    case 0xFFFB: setspeed2(1000, 5000);
        break;
    case 0xFFF9: setspeed2(1000, 5000);
        break;
    case 0xDFFF: setspeed2(5000, 1000);
        break;
    case 0x9FFF: setspeed2(5000, 1000);
        break;
    case 0xFFF8: setspeed2(1000, 5000);
        break;
    case 0x1FFF: setspeed2(5000, 1000);
        break;

    case 0xFFFD: setspeed2(0, 5000);
        break;
    case 0xFFFC: setspeed2(0, 5000);
        break;
    case 0xBFFF: setspeed2(5000, 0);
        break;
    case 0x3FFF: setspeed2(5000, 0);
        break;
    case 0xFFFE: setspeed2(0, 5000);
        break;
    case 0x7FFF: setspeed2(5000, 0);
        break;

    default: setspeed2(5000, 5000);
        break;
    }
}
/* 7→8 路段专用循线，查表速度与 trackxian5 相同，但通过 setspeed78 做电机补偿。 */
void trackxian78()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`HUI_data` 是反相后的 16 路灰度位图。
     * - 数值：左右轮组 RPM
     *   `5000/5000、5000/4000、4000/5000、5000/3500、3500/5000、5000/3000、3000/5000、5000/2000、2000/5000、1000/5000、5000/1000、0/5000、5000/0`。
     * - 可调项：RPM/循线档均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     * - 专项说明：本函数先刷新传感器。 各 `case 0x....` 是硬件灰度位图编码，通常固定；每个 case 的 RPM 差速表才是可调标定值。
     */
    /* 速度表与 trackxian5 相近，但通过 setspeed78 对后轮做固定补偿；补偿值在该函数外调整。 */
    sensor_update();

    switch (HUI_data)
    {

    case 0xFE7F: setspeed78(5000, 5000);
        break;
    case 0xFC7F: setspeed78(5000, 4000);
        break;
    case 0xFE3F: setspeed78(4000, 5000);
        break;
    case 0xFEFF: setspeed78(5000, 3500);
        break;
    case 0xFCFF: setspeed78(5000, 3500);
        break;
    case 0xFF7F: setspeed78(3500, 5000);
        break;
    case 0xFF3F: setspeed78(3500, 5000);
        break;
    case 0xF8FF: setspeed78(5000, 3500);
        break;
    case 0xFF1F: setspeed78(3500, 5000);
        break;

    case 0xFDFF: setspeed78(5000, 3000);
        break;
    case 0xF9FF: setspeed78(5000, 3000);
        break;
    case 0xFFBF: setspeed78(3000, 5000);
        break;
    case 0xFF9F: setspeed78(3000, 5000);
        break;
    case 0xF1FF: setspeed78(5000, 3000);
        break;
    case 0xFF8F: setspeed78(3000, 5000);
        break;

    case 0xFBFF: setspeed78(5000, 3000);
        break;
    case 0xF3FF: setspeed78(5000, 3000);
        break;
    case 0xFFDF: setspeed78(3000, 5000);
        break;
    case 0xFFCF: setspeed78(3000, 5000);
        break;
    case 0xE3FF: setspeed78(5000, 3000);
        break;
    case 0xFFC7: setspeed78(3000, 5000);
        break;

    case 0xF7FF: setspeed78(5000, 2000);
        break;
    case 0xE7FF: setspeed78(5000, 2000);
        break;
    case 0xFFEF: setspeed78(2000, 5000);
        break;
    case 0xFFE7: setspeed78(2000, 5000);
        break;
    case 0xC7FF: setspeed78(5000, 2000);
        break;
    case 0xFFE3: setspeed78(2000, 5000);
        break;

    case 0xEFFF: setspeed78(5000, 2000);
        break;
    case 0xCFFF: setspeed78(5000, 2000);
        break;
    case 0xFFF7: setspeed78(2000, 5000);
        break;
    case 0xFFF3: setspeed78(2000, 5000);
        break;
    case 0x8FFF: setspeed78(5000, 2000);
        break;
    case 0xFFF1: setspeed78(2000, 5000);
        break;

    case 0xFFFB: setspeed78(1000, 5000);
        break;
    case 0xFFF9: setspeed78(1000, 5000);
        break;
    case 0xDFFF: setspeed78(5000, 1000);
        break;
    case 0x9FFF: setspeed78(5000, 1000);
        break;
    case 0xFFF8: setspeed78(1000, 5000);
        break;
    case 0x1FFF: setspeed78(5000, 1000);
        break;

    case 0xFFFD: setspeed78(0, 5000);
        break;
    case 0xFFFC: setspeed78(0, 5000);
        break;
    case 0xBFFF: setspeed78(5000, 0);
        break;
    case 0x3FFF: setspeed78(5000, 0);
        break;
    case 0xFFFE: setspeed78(0, 5000);
        break;
    case 0x7FFF: setspeed78(5000, 0);
        break;

    default: setspeed78(5000, 5000);
        break;
    }
}
/* 高速循线，基础轮速 8000；仅在直线较长且需要快速通过的里程区间使用。 */
void trackxian8()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`HUI_data` 是反相后的 16 路灰度位图。
     * - 数值：左右轮组 RPM
     *   `8000/8000、8000/7000、7000/8000、8000/6500、6500/8000、8000/6000、6000/8000、8000/5000、5000/8000、8000/4000、4000/8000、8000/3000、3000/8000、8000/1000、1000/8000`。
     * - 可调项：RPM/循线档均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     * - 专项说明：本函数先刷新传感器。 各 `case 0x....` 是硬件灰度位图编码，通常固定；每个 case 的 RPM 差速表才是可调标定值。
     */
    /* 8000 为高速直线 RPM；高速时应同步检查 MAX_WHEEL_SPEED、轮胎打滑和制动距离。 */
    sensor_update();

    switch (HUI_data)
    {

    case 0xFE7F: setspeed2(8000, 8000);
        break;
    case 0xFC7F: setspeed2(8000, 7000);
        break;
    case 0xFE3F: setspeed2(7000, 8000);
        break;
    case 0xFEFF: setspeed2(8000, 7000);
        break;
    case 0xFCFF: setspeed2(8000, 6500);
        break;
    case 0xFF7F: setspeed2(6500, 8000);
        break;
    case 0xFF3F: setspeed2(6500, 8000);
        break;
    case 0xF8FF: setspeed2(8000, 6500);
        break;
    case 0xFF1F: setspeed2(6500, 8000);
        break;

    case 0xFDFF: setspeed2(8000, 6000);
        break;
    case 0xF9FF: setspeed2(8000, 6000);
        break;
    case 0xFFBF: setspeed2(6000, 8000);
        break;
    case 0xFF9F: setspeed2(6000, 8000);
        break;
    case 0xF1FF: setspeed2(8000, 6000);
        break;
    case 0xFF8F: setspeed2(6000, 8000);
        break;

    case 0xFBFF: setspeed2(8000, 5000);
        break;
    case 0xF3FF: setspeed2(8000, 5000);
        break;
    case 0xFFDF: setspeed2(5000, 8000);
        break;
    case 0xFFCF: setspeed2(5000, 8000);
        break;
    case 0xE3FF: setspeed2(8000, 5000);
        break;
    case 0xFFC7: setspeed2(5000, 8000);
        break;

    case 0xF7FF: setspeed2(8000, 4000);
        break;
    case 0xE7FF: setspeed2(8000, 4000);
        break;
    case 0xFFEF: setspeed2(4000, 8000);
        break;
    case 0xFFE7: setspeed2(4000, 8000);
        break;
    case 0xC7FF: setspeed2(8000, 4000);
        break;
    case 0xFFE3: setspeed2(4000, 8000);
        break;

    case 0xEFFF: setspeed2(8000, 3000);
        break;
    case 0xCFFF: setspeed2(8000, 3000);
        break;
    case 0xFFF7: setspeed2(3000, 8000);
        break;
    case 0xFFF3: setspeed2(3000, 8000);
        break;
    case 0x8FFF: setspeed2(8000, 3000);
        break;
    case 0xFFF1: setspeed2(3000, 8000);
        break;

    case 0xFFFB: setspeed2(8000, 1000);
        break;
    case 0xFFF9: setspeed2(1000, 8000);
        break;
    case 0xDFFF: setspeed2(8000, 1000);
        break;
    case 0x9FFF: setspeed2(8000, 1000);
        break;
    case 0xFFF8: setspeed2(1000, 8000);
        break;
    case 0x1FFF: setspeed2(8000, 4000);
        break;

    case 0xFFFD: setspeed2(1000, 8000);
        break;
    case 0xFFFC: setspeed2(1000, 8000);
        break;
    case 0xBFFF: setspeed2(8000, 1000);
        break;
    case 0x3FFF: setspeed2(8000, 1000);
        break;
    case 0xFFFE: setspeed2(1000, 8000);
        break;
    case 0x7FFF: setspeed2(8000, 1000);
        break;
    default: setspeed2(8000, 8000);
        break;
    }
}


/*
 * 更高速循线，基础轮速 10000；用于宽直道，仍保留灰度偏差对应的差速。
 * 函数本身不调用 sensor_update()，依赖调用前已有的 HUI_data 快照；通常由紧邻的
 * trackxian5/8 或路线中的显式 sensor_update() 提供采样。
 */
void trackxian10()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`HUI_data` 是反相后的 16 路灰度位图。
     * - 数值：左右轮组 RPM
     *   `10000/10000、10000/9500、9500/10000、10000/9000、9000/10000、10000/8000、8000/10000、10000/7500、7500/10000、10000/6500、6500/10000、10000/6000、6000/10000、5000/10000、10000/5000、7000/10000、10000/7000`。
     * - 可调项：RPM/循线档均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     * - 专项说明：本函数复用已有 HUI_data 快照，调用者必须保证快照新鲜。 各 `case 0x....` 是硬件灰度位图编码，通常固定；每个 case 的 RPM
     *   差速表才是可调标定值。
     */
    /* 10000 为更高速档位；本函数复用当前 HUI_data，不采样，调用者需确保快照足够新。 */
    switch (HUI_data)
    {

    case 0xFE7F: setspeed2(10000, 10000);
        break;
    case 0xFC7F: setspeed2(10000, 9500);
        break;
    case 0xFE3F: setspeed2(9500, 10000);
        break;
    case 0xFEFF: setspeed2(10000, 9000);
        break;
    case 0xFCFF: setspeed2(10000, 9000);
        break;
    case 0xFF7F: setspeed2(9000, 10000);
        break;
    case 0xFF3F: setspeed2(9000, 10000);
        break;
    case 0xF8FF: setspeed2(10000, 9000);
        break;
    case 0xFF1F: setspeed2(9000, 10000);
        break;

    case 0xFDFF: setspeed2(10000, 8000);
        break;
    case 0xF9FF: setspeed2(10000, 8000);
        break;
    case 0xFFBF: setspeed2(8000, 10000);
        break;
    case 0xFF9F: setspeed2(8000, 10000);
        break;
    case 0xF1FF: setspeed2(10000, 8000);
        break;
    case 0xFF8F: setspeed2(8000, 10000);
        break;

    case 0xFBFF: setspeed2(10000, 7500);
        break;
    case 0xF3FF: setspeed2(10000, 7500);
        break;
    case 0xFFDF: setspeed2(7500, 10000);
        break;
    case 0xFFCF: setspeed2(7500, 10000);
        break;
    case 0xE3FF: setspeed2(10000, 7500);
        break;
    case 0xFFC7: setspeed2(7500, 10000);
        break;

    case 0xF7FF: setspeed2(10000, 6500);
        break;
    case 0xE7FF: setspeed2(10000, 6500);
        break;
    case 0xFFEF: setspeed2(6500, 10000);
        break;
    case 0xFFE7: setspeed2(6500, 10000);
        break;
    case 0xC7FF: setspeed2(10000, 6500);
        break;
    case 0xFFE3: setspeed2(6500, 10000);
        break;

    case 0xEFFF: setspeed2(10000, 6000);
        break;
    case 0xCFFF: setspeed2(10000, 6000);
        break;
    case 0xFFF7: setspeed2(6000, 10000);
        break;
    case 0xFFF3: setspeed2(6000, 10000);
        break;
    case 0x8FFF: setspeed2(10000, 6000);
        break;
    case 0xFFF1: setspeed2(6000, 10000);
        break;

    case 0xFFFB: setspeed2(5000, 10000);
        break;
    case 0xFFF9: setspeed2(5000, 10000);
        break;
    case 0xDFFF: setspeed2(10000, 5000);
        break;
    case 0x9FFF: setspeed2(10000, 5000);
        break;
    case 0xFFF8: setspeed2(5000, 10000);
        break;
    case 0x1FFF: setspeed2(10000, 5000);
        break;

    case 0xFFFD: setspeed2(7000, 10000);
        break;
    case 0xFFFC: setspeed2(7000, 10000);
        break;
    case 0xBFFF: setspeed2(10000, 7000);
        break;
    case 0x3FFF: setspeed2(10000, 7000);
        break;
    case 0xFFFE: setspeed2(5000, 10000);
        break;
    case 0x7FFF: setspeed2(10000, 5000);
        break;
    default: setspeed2(10000, 10000);
        break;
    }
}


/* 高速循线，基础轮速 12000；每次调用先刷新传感器，适合确认姿态稳定后的长直线区间。 */
void trackxian12()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`HUI_data` 是反相后的 16 路灰度位图。
     * - 数值：左右轮组 RPM
     *   `12000/12000、12000/11500、11500/12000、12000/11000、11000/12000、12000/10500、10500/12000、12000/10000、10000/12000、12000/9000、9000/12000、8000/12000、12000/8000、7000/12000、12000/7000、5000/12000、12000/5000`。
     * - 可调项：RPM/循线档均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     * - 专项说明：本函数先刷新传感器。 各 `case 0x....` 是硬件灰度位图编码，通常固定；每个 case 的 RPM 差速表才是可调标定值。
     */
    /* 12000 为高速档位；sensor_update() 在查表前刷新灰度和红外，适合连续高速循线。 */
    sensor_update();

    switch (HUI_data)
    {

    case 0xFE7F: setspeed2(12000, 12000);
        break;
    case 0xFC7F: setspeed2(12000, 12000);
        break;
    case 0xFE3F: setspeed2(12000, 12000);
        break;
    case 0xFEFF: setspeed2(12000, 11500);
        break;
    case 0xFCFF: setspeed2(12000, 11500);
        break;
    case 0xFF7F: setspeed2(11500, 12000);
        break;
    case 0xFF3F: setspeed2(11500, 12000);
        break;
    case 0xF8FF: setspeed2(12000, 11500);
        break;
    case 0xFF1F: setspeed2(11500, 12000);
        break;

    case 0xFDFF: setspeed2(12000, 11000);
        break;
    case 0xF9FF: setspeed2(12000, 11000);
        break;
    case 0xFFBF: setspeed2(11000, 12000);
        break;
    case 0xFF9F: setspeed2(11000, 12000);
        break;
    case 0xF1FF: setspeed2(12000, 11000);
        break;
    case 0xFF8F: setspeed2(11000, 12000);
        break;

    case 0xFBFF: setspeed2(12000, 10500);
        break;
    case 0xF3FF: setspeed2(12000, 10500);
        break;
    case 0xFFDF: setspeed2(10500, 12000);
        break;
    case 0xFFCF: setspeed2(10500, 12000);
        break;
    case 0xE3FF: setspeed2(12000, 10500);
        break;
    case 0xFFC7: setspeed2(10500, 12000);
        break;

    case 0xF7FF: setspeed2(12000, 10000);
        break;
    case 0xE7FF: setspeed2(12000, 10000);
        break;
    case 0xFFEF: setspeed2(10000, 12000);
        break;
    case 0xFFE7: setspeed2(10000, 12000);
        break;
    case 0xC7FF: setspeed2(12000, 10000);
        break;
    case 0xFFE3: setspeed2(10000, 12000);
        break;

    case 0xEFFF: setspeed2(12000, 9000);
        break;
    case 0xCFFF: setspeed2(12000, 9000);
        break;
    case 0xFFF7: setspeed2(9000, 12000);
        break;
    case 0xFFF3: setspeed2(9000, 12000);
        break;
    case 0x8FFF: setspeed2(12000, 9000);
        break;
    case 0xFFF1: setspeed2(9000, 12000);
        break;

    case 0xFFFB: setspeed2(8000, 12000);
        break;
    case 0xFFF9: setspeed2(8000, 12000);
        break;
    case 0xDFFF: setspeed2(12000, 8000);
        break;
    case 0x9FFF: setspeed2(12000, 8000);
        break;
    case 0xFFF8: setspeed2(8000, 12000);
        break;
    case 0x1FFF: setspeed2(12000, 8000);
        break;

    case 0xFFFD: setspeed2(7000, 12000);
        break;
    case 0xFFFC: setspeed2(7000, 12000);
        break;
    case 0xBFFF: setspeed2(12000, 7000);
        break;
    case 0x3FFF: setspeed2(12000, 7000);
        break;
    case 0xFFFE: setspeed2(5000, 12000);
        break;
    case 0x7FFF: setspeed2(12000, 5000);
        break;
    default: setspeed2(12000, 12000);
        break;
    }
}


/* 最高标定速度循线，基础轮速 15000；每次调用先刷新传感器，仅用于明确标定的高速直段。 */
void trackxian15()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`HUI_data` 是反相后的 16 路灰度位图。
     * - 数值：左右轮组 RPM
     *   `15000/15000、15000/14500、14500/15000、15000/14000、14000/15000、15000/13500、13500/15000、15000/12500、12500/15000、15000/11500、11500/15000、10000/15000、15000/10000、8000/15000、15000/8000、7000/15000、15000/7000`。
     * - 可调项：RPM/循线档均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     * - 专项说明：本函数先刷新传感器。 各 `case 0x....` 是硬件灰度位图编码，通常固定；每个 case 的 RPM 差速表才是可调标定值。
     */
    /* 15000 为最高标定 RPM；仅用于实测稳定的直段，修改前应确认电机和底盘允许的峰值。 */
    sensor_update();

    switch (HUI_data)
    {

    case 0xFE7F: setspeed2(15000, 15000);
        break;
    case 0xFC7F: setspeed2(15000, 15000);
        break;
    case 0xFE3F: setspeed2(15000, 15000);
        break;
    case 0xFEFF: setspeed2(15000, 14500);
        break;
    case 0xFCFF: setspeed2(15000, 14500);
        break;
    case 0xFF7F: setspeed2(14500, 15000);
        break;
    case 0xFF3F: setspeed2(14500, 15000);
        break;
    case 0xF8FF: setspeed2(15000, 14500);
        break;
    case 0xFF1F: setspeed2(14500, 15000);
        break;

    case 0xFDFF: setspeed2(15000, 14000);
        break;
    case 0xF9FF: setspeed2(15000, 14000);
        break;
    case 0xFFBF: setspeed2(14000, 15000);
        break;
    case 0xFF9F: setspeed2(14000, 15000);
        break;
    case 0xF1FF: setspeed2(15000, 14000);
        break;
    case 0xFF8F: setspeed2(14000, 15000);
        break;

    case 0xFBFF: setspeed2(15000, 13500);
        break;
    case 0xF3FF: setspeed2(15000, 13500);
        break;
    case 0xFFDF: setspeed2(13500, 15000);
        break;
    case 0xFFCF: setspeed2(13500, 15000);
        break;
    case 0xE3FF: setspeed2(15000, 13500);
        break;
    case 0xFFC7: setspeed2(13500, 15000);
        break;

    case 0xF7FF: setspeed2(15000, 12500);
        break;
    case 0xE7FF: setspeed2(15000, 12500);
        break;
    case 0xFFEF: setspeed2(12500, 15000);
        break;
    case 0xFFE7: setspeed2(12500, 15000);
        break;
    case 0xC7FF: setspeed2(15000, 12500);
        break;
    case 0xFFE3: setspeed2(12500, 15000);
        break;

    case 0xEFFF: setspeed2(15000, 11500);
        break;
    case 0xCFFF: setspeed2(15000, 11500);
        break;
    case 0xFFF7: setspeed2(11500, 15000);
        break;
    case 0xFFF3: setspeed2(11500, 15000);
        break;
    case 0x8FFF: setspeed2(15000, 11500);
        break;
    case 0xFFF1: setspeed2(11500, 15000);
        break;

    case 0xFFFB: setspeed2(10000, 15000);
        break;
    case 0xFFF9: setspeed2(10000, 15000);
        break;
    case 0xDFFF: setspeed2(15000, 10000);
        break;
    case 0x9FFF: setspeed2(15000, 10000);
        break;
    case 0xFFF8: setspeed2(10000, 15000);
        break;
    case 0x1FFF: setspeed2(15000, 10000);
        break;

    case 0xFFFD: setspeed2(8000, 15000);
        break;
    case 0xFFFC: setspeed2(8000, 15000);
        break;
    case 0xBFFF: setspeed2(15000, 8000);
        break;
    case 0x3FFF: setspeed2(15000, 8000);
        break;
    case 0xFFFE: setspeed2(7000, 15000);
        break;
    case 0x7FFF: setspeed2(15000, 7000);
        break;
    default: setspeed2(15000, 15000);
        break;
    }
}

/* 上坡段：记录当前里程后以低速循线前进约 80 个里程计单位，供桥/坡动作复用。 */
void gohill()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程。
     * - 数值：相对里程阈值 `80`（round_cnt 计数）；循线档 `trackxian3`。
     * - 可调项：里程阈值、RPM/循线档均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     * - 专项说明：`80` 是从当前轮次计数开始的相对行程，不是毫秒；需按轮径、编码器分辨率和坡道位置重新标定。
     */
    intia_rount_cnt = get_rount_cnt;
    while (get_rount_cnt - intia_rount_cnt < 80)
        trackxian3();
}

/*
 * 平台/桥面收尾动作：前红外触发前持续循线，短暂前进后倒车卸载，再挥手；随后以航向
 * 积分原地左转约 167 度并停车。该函数是多个“到达任务平台”路线的公共尾段。
 */
void goplat()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`angle_sum` 累加有效航向增量以判断目标转角；`go_yaw_inia`
     *   是相邻航向采样差。
     * - 检测条件：`data_storage[12]`（前红外）。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：积分目标角 `167` 度；航向跳变过滤 `45` 度；左右轮组 RPM `3000/3000、-2000/-2000`；四轮 RPM
     *   `-6000/6000/-6000/6000`（m1/m2/m3/m4）；循线档 `trackxian5`；时序 `100、200、1、50` ms。
     * - 可调项：目标转角、IMU 跳变过滤门槛、RPM/循线档、动作时序均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     * - 专项说明：平台段先等待前红外，再执行前冲、倒车、挥手和掉头；`167` 度和 `-6000/6000` RPM 是平台出口定向标定值。
     */
    /* data_storage[12] 非 0 表示前方尚未检测到平台目标；3000/3000 前冲 100 ms 后以
     * -2000/-2000 倒车 200 ms 释放车体压力。167 是平台出口的实测掉头角度。 */
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


/*
 * 比赛主任务。
 * 启动时绑定 INS 共享姿态并初始化循线 PID，等待 3 秒让传感器和底盘稳定。主循环先等待
 * 前红外从“有障碍”变为“无遮挡”，播放起始语音、抬手，然后执行 1→2→4→3→门1 的公共
 * 路线。tl_scan() 决定进入哪一扇任务门：绿灯时执行 1 号门路径，否则依次经过门 1→2、
 * 门 2→3 或门 3→4→5 进行分支选择。每条分支最终经过 5→7→8→直道和对应终点门函数。
 * 函数内部没有返回路径，最外层 for(;;) 在一轮完成后重新等待前红外并开始下一轮。
 */
void run_task(void const* argument)
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：`argument` 是 CMSIS-RTOS 任务入口预留指针，当前未解引用；保持接口类型即可，通常无需设置。
     * - 变量/状态：`pid_params[3]` 依次保存 Kp/Ki/Kd。
     * - 检测条件：`data_storage[12]`（前红外）。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：时序 `3000、50、500` ms；语音轨道 `1`。
     * - 可调项：动作时序均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     * - 专项说明：`pid_params[0..2]` 依次为 Kp、Ki、Kd；`tl_scan()==1` 是绿灯分支结果，`3000 ms`
     *   是上电稳定等待，均应与实际模块初始化时序核对。
     */
    /* INS 任务持有角度数据，本任务只保存只读指针；PID 参数顺序为 Kp、Ki、Kd。 */
    INS_angle_go = get_INS_angle_point();
    v_r = v_l = 0;
    const fp32 pid_params[3] = {TRACK_PID_KP, TRACK_PID_KI, TRACK_PID_KD};
    PID_init(&track_pid, PID_POSITION, pid_params, TRACK_PID_MAX_WZ, TRACK_PID_MAX_IOUT);
    osDelay(3000);














    for (;;)
    {

















        /* 等待前红外出现一次完整的“遮挡→释放”沿，将其作为人工/机构启动信号。 */
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

        /* 公共前半程：从 1 区经 2、4、3 区到达第一个交通灯判断点。 */
        go12();
        go24();
        go43();
        go3men1();
        if (tl_scan()==1)
        {
            /* 第一个判断点为绿灯：完成 1 号门任务，再走公共 5→7→8 通道到 1 号终点门。 */
            gomen15();
            go57();
            go78();
            go8zhi();
            gozhimen1();

            /* 终点处再次等待红外启动沿，开始第二段指定的 1→2→3→门1→5 路线。 */
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
            /* 第一个判断点未识别为绿灯，转移到 2 号门继续判定。 */
            gomen1_2();
            if (tl_scan()==1)
            {
                /* 第二个判断点为绿灯：执行 2 号门任务，并从 2 号终点门结束该轮。 */
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
                /* 第二个判断点仍非绿灯，继续转移到 3 号门。 */
                gomen2_3();
                if (tl_scan()==1)
                {
                    /* 第三个判断点为绿灯：执行 3 号门任务，随后走 3 号终点门路线。 */
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
                    /* 三次判断均未命中绿灯：按兜底路线穿过 4 号门并完成 5 号平台任务。 */
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

/*
 * 通用原地转向。
 * speed 为两侧轮组转速绝对值，angle 的绝对值为目标累计角度；angle>0 时右轮反转、左轮
 * 正转，angle<0 时相反。只接受小于 45 度的相邻航向变化，既过滤异常跳变，也能在航向
 * 跨越 ±180 度时继续积分。达到目标或超过 10 秒后退出，最后主动下发零速。
 */
void turn_run(int speed, fp32 angle)
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：`speed` 是两侧轮组原地转向的 RPM 绝对值；`angle` 是目标累计航向角（度），正负号决定方向；两者都可现场标定。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`t0` 保存转向起始系统 tick；`i` 仅为循环计数器；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 数值：航向跳变过滤 `45` 度；时序 `1` ms。
     * - 可调项：IMU 跳变过滤门槛、动作时序均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     * - 专项说明：`57.2974` 是弧度转角度近似系数；`10000` tick 是 10 s 超时保护（当前 tick=1 kHz），可按安全策略调整。
     */
    /* last_angle/go_yaw/go_yaw_inia/angle_sum 构成航向积分状态；t0 用于 10000 tick 超时。
     * speed 只影响转向快慢，angle 决定方向和目标角度，二者是路线最常调的参数。 */
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

/*
 * 旧版通用原地转向实现。与 turn_run() 的轮速方向约定相同，但角度换算使用 180/3.1415、
 * 单次增量过滤阈值为 20 度，且没有超时保护；保留供历史路线兼容。
 */
void turn2(int speed, fp32 angle)
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：`speed` 是两侧轮组原地转向的 RPM 绝对值；`angle` 是目标累计航向角（度），正负号决定方向；两者都可现场标定。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`angle_sum` 累加有效航向增量以判断目标转角；`go_yaw_inia`
     *   是相邻航向采样差。
     * - 数值：航向跳变过滤 `20` 度；时序 `1` ms。
     * - 可调项：IMU 跳变过滤门槛、动作时序均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     * - 专项说明：`180/3.1415` 是弧度转角度近似系数，不是赛道参数；`20` 是单次 IMU 跳变过滤阈值，转向 RPM 和目标角可调。
     */
    /* 旧接口没有 t0 超时保护；若 IMU 不更新或角度方向错误，for(;;) 可能永久阻塞。 */
    fp32 last_angle = 0;
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


/* 立即将四个电机目标转速清零；该函数只发命令，不包含延时或制动完成确认。 */
void stop(void)
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 数值：四轮 RPM `0/0/0/0`（m1/m2/m3/m4）。
     * - 可调项：RPM/循线档均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     * - 专项说明：四个 `0` 是明确的停车命令，不应改成非零“缓停”值；如需制动策略，应另行修改底盘层而非本函数。
     */
    set_current(0, 0, 0, 0);
}


/* 保存当前底盘里程计数，后续路线用 get_rount_cnt-intia_rount_cnt 判断相对行程。 */
void set0rount()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程。
     * - 专项说明：该函数只写入当前 motor0 的累计 round_cnt 快照；调用位置决定后续所有相对里程的零点。
     */
    intia_rount_cnt = get_rount_cnt;
}


/* 下平台的公共短段：从当前里程起，以基础轮速 3000 循线前进 40 个计数。 */
void downplat()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程。
     * - 数值：相对里程阈值 `40`（round_cnt 计数）；循线档 `trackxian3`。
     * - 可调项：里程阈值、RPM/循线档均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     * - 专项说明：`40` 是下平台后的短距离补偿计数；赛道平台边缘位置变化时可单独调整。
     */
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 40)
        trackxian3();
}


/*
 * 桥面红外纠偏动作。
 * 仅左侧红外触发时提高左轮组速度，仅右侧触发时提高右轮组速度，其余情况直行；每次
 * 命令保持 130 ms。调用者通常在循环中反复调用，以侧向红外状态修正车身位置。
 */
void gobri()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 检测条件：`data_storage[14、13]`（右红外；左红外）。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：四轮 RPM `3000/2000/3000/2000、2000/3000/2000/3000、3700/3700/3700/3700`（m1/m2/m3/m4）；时序
     *   `130` ms。
     * - 可调项：RPM/循线档、动作时序均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
    /* data_storage[13]/[14] 分别为左右红外；130 ms 是一次纠偏动作的保持时间，
     * 3000/2000 的差速大小决定修正强度。 */
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


/* gobri() 的快速强纠偏版本：使用更大的左右差速，每次只保持 10 ms。 */
void gobri1()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 检测条件：`data_storage[14、13]`（右红外；左红外）。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：四轮 RPM `5000/2000/5000/2000、2000/5000/2000/5000、5000/5000/5000/5000`（m1/m2/m3/m4）；时序 `10`
     *   ms。
     * - 可调项：RPM/循线档、动作时序均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     * - 专项说明：两个 `if` 是并列语句，`else` 仅与第二个 `if` 配对；这是现有控制流程，注释不改变其可能在一次调用内再次下发直行命令的行为。
     */
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

/* 桥面低速微调版本：依据左右红外设置 3000/1000 差速，每次命令保持 30 ms。 */
void goqqb()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 检测条件：`data_storage[14、13]`（右红外；左红外）。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：四轮 RPM `3000/1000/3000/1000、1000/3000/1000/3000、3000/3000/3000/3000`（m1/m2/m3/m4）；时序 `30`
     *   ms。
     * - 可调项：RPM/循线档、动作时序均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/*
 * 四电机转速命令的唯一汇合点。参数顺序与 chassis_set_motor_rpm() 的电机编号一致，
 * 路线层通常让 motor1/motor3 同速、motor2/motor4 同速，从而形成左右轮组差速。
 */
void set_current(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：`motor1`、`motor2`、`motor3`、`motor4` 按旧 CAN 序对应左前、右前、左后、右后电机的 RPM 目标；编号固定，数值可调。
     * - 变量/状态：`set_speed_rpm[4]` 是待下发的四电机 RPM 数组。
     */
    set_speed_rpm[0] = motor1;
    set_speed_rpm[1] = motor2;
    set_speed_rpm[2] = motor3;
    set_speed_rpm[3] = motor4;
    chassis_set_motor_rpm(set_speed_rpm);
}

/* 舵机挥手动作：依次驱动通道 1 和通道 3，在两个端点间摆动并留出机械到位时间。 */
void wave_hand(void)
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 数值：时序 `200` ms；舵机 `1500 us / 通道 1、1500 us / 通道 3、500 us / 通道 3、2500 us / 通道 1`。
     * - 可调项：动作时序、舵机脉宽与到位等待均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     * - 专项说明：`1500 us` 通常为中位，`500/2500 us` 为端点；脉宽与 `200 ms` 到位等待可微调，但不得越过机构限位。
     */
    /* 200 ms 是舵机到位等待；1500 为中位，500 和 2500 为两端位置，均可按机构行程微调。 */
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

/* 7→8 路线使用的挥手动作；当前脉宽和时序与 wave_hand() 完全一致，保留独立调参入口。 */
void wave_handc78(void)
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 数值：时序 `200` ms；舵机 `1500 us / 通道 1、1500 us / 通道 3、500 us / 通道 3、2500 us / 通道 1`。
     * - 可调项：动作时序、舵机脉宽与到位等待均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     * - 专项说明：`1500 us` 通常为中位，`500/2500 us` 为端点；脉宽与 `200 ms` 到位等待可微调，但不得越过机构限位。
     */
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

/* 抬手机构动作：通道 1、3 同步回中，再分别运动到目标端点。 */
void up_hand(void)
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 数值：时序 `200` ms；舵机 `1500 us / 通道 1、1500 us / 通道 3、500 us / 通道 3、2500 us / 通道 1`。
     * - 可调项：动作时序、舵机脉宽与到位等待均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     * - 专项说明：`1500 us` 通常为中位，`500/2500 us` 为端点；脉宽与 `200 ms` 到位等待可微调，但不得越过机构限位。
     */
    /* 抬手只使用通道 1、3；两个通道先同步回中，再同时运动到端点。 */
    osDelay(200);
    servo_pwm_set(1500, 1);
    servo_pwm_set(1500, 3);
    osDelay(200);
    servo_pwm_set(500, 3);
    servo_pwm_set(2500, 1);
    osDelay(200);
}


/* 播放编号 a 对应的语音轨道；强制转换为语音模块公开的枚举类型。 */
void yuyin(int a)
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：`a` 是语音模块轨道号；属于比赛交互协议，只有同步更换语音素材表时才调整。
     * - 专项说明：语音编号不是速度或赛道编号；它必须与 voice_track_t 枚举和实际 SD/模块素材表保持一致。
     */
    /* a 由路线语义决定，例如 1 是启动提示，5/9 等是任务完成提示。 */
    voice_module_play((voice_track_t)a);
}


/*
 * 1→2 主路线：下平台后先等待车体上坡/回平，再以里程 120 的中速循线到达红外目标。
 * 到达后倒车卸载、挥手、右转约 170 度并播放 2 号语音，完成一次门区任务。
 */
void go12()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程。
     * - 检测条件：`data_storage[12]`（前红外）；横滚角 `get_roll<15、>-10、>10`
     *   度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `120`（round_cnt 计数）；通用转向 `5000 RPM / 170 度`；左右轮组 RPM `3000/3000、-2000/-2000`；循线档
     *   `trackxian`；时序 `100、200、150` ms；语音轨道 `2`。
     * - 可调项：里程阈值、目标转角、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
    downplat();

    while (get_roll < 15)
        trackxian(1.1f);



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

/*
 * 2→4 路线：通过灰度边沿和多段里程速度（5→8→12→8→5）穿过长直道，随后依据前红外
 * 到达平台，倒车、挥手、右转并播放 4 号语音。中间两次航向积分分别用于进入和离开支路。
 */
void go24()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[13、14、0、1]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角 `get_roll<15`
     *   度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `20、50、30、60、390、420、450`（round_cnt 计数）；积分目标角 `35、30` 度；航向跳变过滤 `20` 度；通用转向 `5000
     *   RPM / 170 度`；左右轮组 RPM `3000/3000、-2000/-2000`；四轮 RPM
     *   `7000/500/7000/500、-1500/7000/-1500/7000`（m1/m2/m3/m4）；循线档
     *   `trackxian5、trackxian3、trackxian8、trackxian12`；时序 `1、100、150、300、50` ms；语音轨道 `4`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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
/* 2→3 路线：与 go24 共用前半段，第二个灰度边沿处改为约 130 度转向，最终到达 3 号平台。 */
void go23()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[13、14、15]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角 `get_roll<15`
     *   度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `20、50、30、60、400、410、430`（round_cnt 计数）；积分目标角 `35、130` 度；航向跳变过滤 `20` 度；通用转向 `5000
     *   RPM / 170 度`；左右轮组 RPM `3000/3000、-2000/-2000`；四轮 RPM
     *   `7000/500/7000/500、7000/-2000/7000/-2000`（m1/m2/m3/m4）；循线档
     *   `trackxian5、trackxian3、trackxian8、trackxian12`；时序 `1、150、300、50` ms；语音轨道 `3`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 3→4 直连路线：连续经过多个灰度边沿，按 8/10/12 档循线，检测坡面回平后前进、倒车，
 * 以 165 度转向离开并播放 2 号语音，再挥手结束。 */
void go34()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程。
     * - 检测条件：灰度 `hui[15、14、0、1]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角 `get_roll<15`
     *   度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `20`（round_cnt 计数）；通用转向 `5000 RPM / 165 度`；左右轮组 RPM `3000/3000、-2000/-2000`；循线档
     *   `trackxian5、trackxian8、trackxian10、trackxian12`；时序 `150、200、100` ms；语音轨道 `2`。
     * - 可调项：里程阈值、目标转角、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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










    while (get_roll < 15)
        trackxian5();
    while (data_storage[12] != 0)
        setspeed2(3000, 3000);
    osDelay(150);


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

/* 4→门1 路线：按里程通过高速直道，在右侧灰度边沿处积分约 80 度转入门区，继续循线到
 * 目标黑线后停车，供 run_task() 的首个绿灯分支使用。 */
void go4men1()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[14、15、5、6、7、8、9、10]`（0 表示压到黑线）。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `40、280、100`（round_cnt 计数）；积分目标角 `80` 度；航向跳变过滤 `20` 度；四轮 RPM
     *   `6500/500/6500/500`（m1/m2/m3/m4）；循线档 `trackxian5、trackxian8`；时序 `1、100` ms。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 门1→门2 转移：先右转 170 度脱离当前门，再以 140 度定向进入下一通道，最后用路口检测
 * det=4 确认已经到达门2，停车等待交通灯判定。 */
void gomen1_2()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[14、13]`（0 表示压到黑线）；路口确认 det=`4`。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `80、60`（round_cnt 计数）；积分目标角 `140` 度；航向跳变过滤 `20` 度；通用转向 `5000 RPM / 170 度`；四轮 RPM
     *   `7000/-2500/7000/-2500`（m1/m2/m3/m4）；循线档 `trackxian5`；时序 `100、1、700` ms。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 门2→门3 转移：两次约 135 度航向转动夹着 5/8/12 档循线，末端以 det=4 路口确认门3。 */
void gomen2_3()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[13、14、15]`（0 表示压到黑线）；路口确认 det=`4`。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `30、70、230、250、270、60`（round_cnt 计数）；积分目标角 `135` 度；航向跳变过滤 `20` 度；通用转向 `5000 RPM /
     *   170 度`；四轮 RPM `7000/-2500/7000/-2500`（m1/m2/m3/m4）；循线档
     *   `trackxian5、trackxian8、trackxian12`；时序 `100、1、700` ms。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
    turn_run(5000, 170);
    stop();
    osDelay(100);
    set0rount();



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

/* 门3→门4 转移：右转后沿里程前进，在左侧灰度边沿处完成约 130 度定向，抵达门4入口。 */
void gomen3_4()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[14、15]`（0 表示压到黑线）。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `70、80`（round_cnt 计数）；积分目标角 `130` 度；航向跳变过滤 `20` 度；通用转向 `5000 RPM / 170 度`；四轮 RPM
     *   `6000/-1500/6000/-1500`（m1/m2/m3/m4）；循线档 `trackxian5`；时序 `100、50、1` ms。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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



}

/* 门3→门4→门5 兜底长路线：完成两段定角和多档循线后进入平台，调用 goplat() 收尾并播放 5 号语音。 */
void gomen3_men4_5()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[14、15、0、1]`（0 表示压到黑线）；横滚角 `get_roll<15` 度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `40、30、160、200、70、210、250、280`（round_cnt 计数）；积分目标角 `130、80` 度；航向跳变过滤 `20` 度；通用转向
     *   `5000 RPM / 170 度`；四轮 RPM `6000/-2000/6000/-2000、-2000/7000/-2000/7000`（m1/m2/m3/m4）；循线档
     *   `trackxian5、trackxian8、trackxian12`；时序 `100、1` ms；语音轨道 `5`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 门1→5 任务路线：在门区内连续三次路口检测和转向，执行一次倒车/语音动作后回到主线，
 * 再以 5/8/12/15 档到达平台并调用 goplat()。 */
void gomen15()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程。
     * - 检测条件：灰度 `hui[14、15]`（0 表示压到黑线）；横滚角 `get_roll<15` 度；路口确认
     *   det=`5`。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `40、120、160、30、80、260、270、60、200、230`（round_cnt 计数）；通用转向 `5000 RPM / 85 度、5000 RPM
     *   / -85 度`；左右轮组 RPM `-3000/-3000`；循线档 `trackxian5、trackxian8、trackxian12、trackxian15`；时序
     *   `50、1、900、800` ms；for 上限 `80`（循环次数，不自动等同距离）；语音轨道 `10、12、5`。
     * - 可调项：里程阈值、目标转角、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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


    while (get_roll < 15)
        trackxian8();

    goplat();
    yuyin(5);
}

/* 门2→5 任务路线：里程分段进入支路，约 35 度修正方向，使用 5/8/12 档通过直段后平台收尾。 */
void gomen25()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1]`（0 表示压到黑线）；横滚角 `get_roll<15` 度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `30、70、230、260、300、80、210、250、280`（round_cnt 计数）；积分目标角 `35` 度；航向跳变过滤 `20` 度；四轮 RPM
     *   `-1500/7500/-1500/7500`（m1/m2/m3/m4）；循线档 `trackxian5、trackxian8、trackxian12`；时序 `100、1`
     *   ms；语音轨道 `5`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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


    while (get_rount_cnt - intia_rount_cnt < 250)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 280)
        trackxian5();
    while (get_roll < 15)
        trackxian5();


    goplat();
    yuyin(5);
}

/* 门3→5 任务路线：先左转进入往返支路，再执行倒车、语音和两次 85 度掉头，最后返回平台。 */
void gomen35()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1、15、14]`（0 表示压到黑线）；横滚角 `get_roll<15` 度；路口确认
     *   det=`5`。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `30、100、120、50、70、240、270、290、80、200、280`（round_cnt 计数）；积分目标角 `40、80` 度；航向跳变过滤
     *   `20` 度；通用转向 `3000 RPM / 85 度、3000 RPM / -85 度`；左右轮组 RPM `-3000/-3000`；四轮 RPM
     *   `-1500/6500/-1500/6500、6500/-1500/6500/-1500、-2000/6500/-2000/6500`（m1/m2/m3/m4）；循线档
     *   `trackxian5、trackxian8、trackxian12`；时序 `100、1、50、800、700` ms；for 上限
     *   `70、80`（循环次数，不自动等同距离）；语音轨道 `10、12、5`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while (get_rount_cnt - intia_rount_cnt < 100)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 120)
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


    while (get_rount_cnt - intia_rount_cnt < 240)
        trackxian8();
    while (get_rount_cnt - intia_rount_cnt < 280)
        trackxian5();
    while (get_roll < 15)
        trackxian5();


    goplat();
    yuyin(5);
}
/* 4→门3→5 路线：从 4 区进入门3方向，重复门区往返动作，使用 det=5 路口确认并在平台结束。 */
void go4men35()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1、14、15]`（0 表示压到黑线）；横滚角 `get_roll<-5、<15` 度；路口确认
     *   det=`5`。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `30、70、220、250、280、270、300、320、50、240、290、20、100`（round_cnt 计数）；积分目标角 `35、40、80`
     *   度；航向跳变过滤 `20` 度；通用转向 `3000 RPM / 85 度、3000 RPM / -85 度`；左右轮组 RPM
     *   `-3000/-3000、-3000/-3100`；四轮 RPM
     *   `-1500/6500/-1500/6500、6500/-1500/6500/-1500、-2000/6500/-2000/6500`（m1/m2/m3/m4）；循线档
     *   `trackxian5、trackxian8、trackxian12、trackxian10`；时序 `1、50、100、520、700` ms；for 上限
     *   `70、80`（循环次数，不自动等同距离）；语音轨道 `10、5`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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
/* 门4→5 任务路线：沿长直道进入门区，完成定角、倒车和语音交互，再循线回到平台。 */
void gomen45()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1、14、15]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角 `get_roll<15`
     *   度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `100、200`（round_cnt 计数）；积分目标角 `80` 度；航向跳变过滤 `20` 度；通用转向 `5000 RPM / 165 度`；左右轮组
     *   RPM `3000/3000、-2000/-2000`；四轮 RPM `-500/6500/-500/6500`（m1/m2/m3/m4）；循线档
     *   `trackxian5、trackxian8`；时序 `1、150、200、100` ms；语音轨道 `2`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/*
 * 5→7 长路线：下平台后连续通过多个灰度边沿和坡面，以 50/140/80 度等标定角度切换赛道；
 * 末端由前红外定位，执行倒车、挥手、约 169 度掉头并播放 7 号语音。
 */
void go57()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[1、2、0、14、15]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角 `get_roll<15`
     *   度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `70、40、130、100、80、150、20、50、60`（round_cnt 计数）；积分目标角 `50、140、80` 度；航向跳变过滤 `20、45`
     *   度；通用转向 `5000 RPM / 169 度`；左右轮组 RPM `3000/3000、-2000/-2000`；四轮 RPM
     *   `500/7000/500/7000、-3000/5500/-3000/5500、6500/-1500/6500/-1500`（m1/m2/m3/m4）；循线档
     *   `trackxian5、trackxian8、trackxian3`；时序 `1、200、150、300、50` ms；语音轨道 `7`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/*
 * 7→8 路线：先用 roll 和全灰度状态越过坡顶，再调用带电机补偿的 trackxian78() 下坡；
 * 中途按里程分段通过桥区，前红外定位后挥手、反向转约 168 度并播放 8 号语音。
 */
void go78()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程。
     * - 检测条件：灰度 `hui[0、1、2、3、4、5、6、7、8、9、10、11、12、13、14、15]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角
     *   `get_roll>-5、<-5、<15` 度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `70、160、30、100、90、150、210`（round_cnt 计数）；通用转向 `5000 RPM / -168 度`；左右轮组 RPM
     *   `3000/3000、-2000/-2000`；四轮 RPM `3000/3000/3000/3000`（m1/m2/m3/m4）；循线档
     *   `trackxian78、trackxian5、trackxian3、trackxian8`；时序 `100、150、200、50` ms；语音轨道 `8`。
     * - 可调项：里程阈值、目标转角、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */



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


    while (get_rount_cnt - intia_rount_cnt < 150)
        trackxian8();
    while (get_roll < 15)
        trackxian5();
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 210)
        trackxian5();
    while (data_storage[12] != 0)
        trackxian3();



    setspeed2(3000, 3000);
    osDelay(150);
    setspeed2(-2000, -2000);
    osDelay(200);
    stop();
    wave_handc78();
    osDelay(50);




    turn_run(5000, -168);
    stop();
    osDelay(100);
    yuyin(8);
    osDelay(50);
}

/*
 * 8 区→直道公共路线：依次越过坡面、进入长直道并在多个左右灰度边沿完成定角转向；
 * 到达前红外目标后播放 11 号语音、倒车脱离，再转约 85 度朝向终点门路线。
 */
void go8zhi()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1、2、3、4、5、6、7、8、9、10、11、12、13、14、15]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角
     *   `get_roll>-5、<-15` 度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `20、60、30、170、180、50、110、40、80、120、140`（round_cnt 计数）；积分目标角 `140、110、80` 度；航向跳变过滤
     *   `20、45` 度；通用转向 `5000 RPM / 85 度`；左右轮组 RPM `3000/3000、-3000/-3000`；四轮 RPM
     *   `3000/3000/3000/3000、2000/2000/2000/2000、-4000/7000/-4000/7000、-2000/8000/-2000/8000、7000/-1000/7000/-1000`（m1/m2/m3/m4）；循线档
     *   `trackxian3、trackxian78、trackxian5、trackxian8、trackxian12`；时序 `100、200、1、50、150、500、800`
     *   ms；语音轨道 `11`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */



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




    set0rount();


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



    setspeed2(-3000, -3000);
    osDelay(800);
    stop();
    turn_run(5000, 85);
    stop();
    osDelay(100);
}

/* 直道→1 号终点门：多段 80 度转向连接高速直线，末段用 roll 判定坡面进出，停车挥手、
 * 掉头约 170 度并播放 9 号语音。 */
void gozhimen1()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[14、15、0、1]`（0 表示压到黑线）；横滚角 `get_roll<15、>5`
     *   度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `60、30、70、260、290、320、160、190、230、50、100、120`（round_cnt 计数）；积分目标角 `80、30、35`
     *   度；航向跳变过滤 `20` 度；通用转向 `5000 RPM / 170 度`；左右轮组 RPM `3000/3000`；四轮 RPM
     *   `7000/-1000/7000/-1000、-1000/7000/-1000/7000、1000/8000/1000/8000、8000/-1000/8000/-1000`（m1/m2/m3/m4）；循线档
     *   `trackxian5、trackxian8、trackxian12、trackxian3`；时序 `1、100、150、250、200` ms；语音轨道 `9`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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
/* 直道→1 号终点门的 3 区来向变体；里程和转角针对从 go3men15() 接入时的姿态标定。 */
void gozhimen13()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[14、15、0、1]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角 `get_roll<15、<-5、>15`
     *   度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值
     *   `20、30、70、260、290、320、150、180、220、100、560、590、620、650、50、330、360、400、90、110`（round_cnt
     *   计数）；积分目标角 `80、140、35` 度；航向跳变过滤 `20` 度；通用转向 `5000 RPM / 168 度、5000 RPM / 170 度`；左右轮组 RPM
     *   `3000/3000、-2000/-2000`；四轮 RPM
     *   `7000/-1000/7000/-1000、-1000/7000/-1000/7000、6000/-1000/6000/-1000、6000/-2000/6000/-2000`（m1/m2/m3/m4）；循线档
     *   `trackxian8、trackxian5、trackxian12、trackxian15、trackxian3`；时序 `1、120、200、50、150、300` ms；语音轨道
     *   `4、9`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 20)
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
/* 直道→2 号终点门：沿门2标定的灰度边沿和长直线路径到达坡面，完成终点停车与语音。 */
void gozhimen2()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[14、15、0、1]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角 `get_roll<15、>5`
     *   度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `30、50、70、260、290、320、20、180、210、280、350、100、120`（round_cnt 计数）；积分目标角
     *   `80、40、130、30、35` 度；航向跳变过滤 `20` 度；通用转向 `5000 RPM / 85 度、5000 RPM / -85 度、5000 RPM / 170
     *   度`；左右轮组 RPM `3000/3000、-3000/-3000`；四轮 RPM
     *   `7000/-1000/7000/-1000、-500/6500/-500/6500、-1500/6500/-1500/6500、7500/-2500/7500/-2500、-1000/7000/-1000/7000、8000/-1000/8000/-1000`（m1/m2/m3/m4）；循线档
     *   `trackxian5、trackxian8、trackxian12、trackxian3`；时序 `1、150、50、800、100、700、250、200` ms；语音轨道
     *   `10、12、9`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while(get_rount_cnt-intia_rount_cnt<50)
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
/* 直道→2 号终点门的 4 区来向变体；包含门区往返段及更长的 15 档高速里程区间。 */
void gozhimen24()

{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[14、15、0、1]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角 `get_roll<15、<-5、>10`
     *   度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值
     *   `30、80、70、260、290、320、20、180、210、280、50、350、100、450、480、520、550、60、360、400`（round_cnt
     *   计数）；积分目标角 `80、40、130、135、30` 度；航向跳变过滤 `20` 度；通用转向 `5000 RPM / 85 度、5000 RPM / -85 度、5000 RPM
     *   / 170 度`；左右轮组 RPM `3000/3000、-3000/-3000、-3000/-3100、-2000/-2000`；四轮 RPM
     *   `7000/-1000/7000/-1000、-500/6500/-500/6500、-1500/6500/-1500/6500、7500/-2500/7500/-2500、7000/-2000/7000/-2000、8000/1000/8000/1000`（m1/m2/m3/m4）；循线档
     *   `trackxian5、trackxian8、trackxian12、trackxian15、trackxian3`；时序
     *   `1、150、100、400、700、120、200、500、50、250` ms；语音轨道 `10、4、9`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
    set0rount();
    while (get_rount_cnt - intia_rount_cnt < 30)
        trackxian5();
    while(get_rount_cnt-intia_rount_cnt<80)
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
/* 直道→3 号终点门：按 3 号门几何连续定角和循线，红外定位平台后完成掉头、语音与挥手。 */
void gozhimen3()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[14、15、1、0]`（0 表示压到黑线）；横滚角 `get_roll<15、>5`
     *   度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `80、30、70、260、290、320、330、370、400、100、50`（round_cnt 计数）；积分目标角 `80、40、130、30`
     *   度；航向跳变过滤 `20` 度；通用转向 `5000 RPM / 170 度`；左右轮组 RPM `3000/3000`；四轮 RPM
     *   `7000/-1000/7000/-1000、-1000/7000/-1000/7000、-2000/6000/-2000/6000、6000/-2000/6000/-2000、8000/2000/8000/2000`（m1/m2/m3/m4）；循线档
     *   `trackxian5、trackxian8、trackxian12、trackxian3`；时序 `1、20、100、300、200` ms；语音轨道 `9`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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
/* 直道→3 号终点门的二次任务变体；到达一次终点后再次下平台，走回程标定段并最终停车。 */
void gozhimen33()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[14、15、1、0]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角 `get_roll<15、<-5、>5`
     *   度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `80、30、70、220、260、300、330、370、400、100、440、470、500、60、320、360、50`（round_cnt
     *   计数）；积分目标角 `80、40、130、30` 度；航向跳变过滤 `20` 度；通用转向 `5000 RPM / 170 度`；左右轮组 RPM
     *   `3000/3000、-2000/-2000`；四轮 RPM
     *   `7000/-1000/7000/-1000、-1000/7000/-1000/7000、-2000/6000/-2000/6000、1000/6000/1000/6000、8000/2000/8000/2000`（m1/m2/m3/m4）；循线档
     *   `trackxian5、trackxian8、trackxian12、trackxian15、trackxian3`；时序 `1、20、120、200、500、100、300`
     *   ms；语音轨道 `3、9`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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
/* 直道→4 号终点门：穿过门4的多段折线路径，途中执行两次任务点语音/倒车动作，最终上坡停车。 */
void gozhimen4()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[14、15、0、1]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角 `get_roll<15、>5`
     *   度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `30、80、250、300、50、100、240、45、60、270、90、110、160、180`（round_cnt 计数）；积分目标角
     *   `80、130、35` 度；航向跳变过滤 `20` 度；通用转向 `5000 RPM / 80 度、5000 RPM / -80 度、5000 RPM / 170 度`；左右轮组
     *   RPM `3000/3000、-3000/-3000`；四轮 RPM
     *   `5500/-1500/5500/-1500、7000/-1000/7000/-1000、6500/-500/6500/-500、-500/6500/-500/6500、-1000/6000/-1000/6000、5000/-2000/5000/-2000`（m1/m2/m3/m4）；循线档
     *   `trackxian5、trackxian8、trackxian12、trackxian10、trackxian3`；时序 `1、150、50、800、100、700、300、200`
     *   ms；语音轨道 `10、9`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
	set0rount();
	while(get_rount_cnt-intia_rount_cnt<30)
		trackxian5();
	while(get_rount_cnt-intia_rount_cnt<80)
		trackxian8();


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
/* 直道→4 号终点门的 3 区来向变体；主体与 gozhimen4 相似，部分倒车时间和高速里程不同。 */
void gozhimen43()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[14、15、0、1]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角 `get_roll<15、<-5、>5`
     *   度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `30、80、250、300、50、100、240、45、60、270、70、460、490、520、550、320、360、400、120`（round_cnt
     *   计数）；积分目标角 `80、40、30` 度；航向跳变过滤 `20` 度；通用转向 `5000 RPM / 80 度、5000 RPM / -80 度、5000 RPM / 170
     *   度`；左右轮组 RPM `3000/3000、-3000/-3000、-2000/-2000`；四轮 RPM
     *   `5500/-1500/5500/-1500、7000/-1000/7000/-1000、6500/-500/6500/-500、-500/6500/-500/6500、-1000/6000/-1000/6000、1000/6000/1000/6000`（m1/m2/m3/m4）；循线档
     *   `trackxian5、trackxian8、trackxian12、trackxian10、trackxian15、trackxian3`；时序
     *   `1、150、500、100、700、50、120、200、300` ms；语音轨道 `10、3`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
	set0rount();
	while(get_rount_cnt-intia_rount_cnt<30)
		trackxian5();
	while(get_rount_cnt-intia_rount_cnt<80)
		trackxian8();


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
/* 3→6 路线：下平台后经 8 档直线和多个灰度边沿转弯，最终以桥面纠偏进入 6 区。 */
void go36()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[14、15]`（0 表示压到黑线）；路口确认 det=`5、4`。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `40、280、30、50、140`（round_cnt 计数）；积分目标角 `80、125、140、135` 度；航向跳变过滤 `20` 度；通用转向 `5000
     *   RPM / 165 度`；四轮 RPM `6500/-1500/6500/-1500`（m1/m2/m3/m4）；循线档 `trackxian5、trackxian8`；时序
     *   `1、50、500、150` ms。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 3→5 路线：按坡度和分级速度通过长直段，在路口定向后抵达 5 区平台。 */
void go35()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程。
     * - 检测条件：灰度 `hui[14、15]`（0 表示压到黑线）；横滚角 `get_roll<-5、<15` 度；路口确认
     *   det=`5`。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `30、60、160、210、240、70、220、250、290、80、280、20、50、100、270`（round_cnt 计数）；通用转向 `3000
     *   RPM / 85 度、3000 RPM / -86 度`；左右轮组 RPM `-3000/-3000`；循线档
     *   `trackxian5、trackxian8、trackxian12、trackxian、trackxian10`；时序 `50、1、100、800` ms；for 上限
     *   `50`（循环次数，不自动等同距离）；语音轨道 `10、5`。
     * - 可调项：里程阈值、目标转角、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 4→6 路线：400 个计数高速直行后进入折线区，利用左右灰度边沿完成转向并到达 6 区。 */
void go46()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1、14、15]`（0 表示压到黑线）；路口确认 det=`5`。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `50、400、30、140`（round_cnt 计数）；积分目标角 `80、125、140、135` 度；航向跳变过滤 `20` 度；通用转向 `5000
     *   RPM / 165 度`；四轮 RPM `-1500/6500/-1500/6500、6500/-1500/6500/-1500`（m1/m2/m3/m4）；循线档
     *   `trackxian5、trackxian8`；时序 `1、50、500、150` ms。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 4→5 路线：与 go46 共用长直段，按 5 区方向的转角标定进入平台。 */
void go45()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1、14、15]`（0 表示压到黑线）；路口确认 det=`5`。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `50、400、30、140`（round_cnt 计数）；积分目标角 `80、125、140、135` 度；航向跳变过滤 `20` 度；通用转向 `5000
     *   RPM / 165 度`；四轮 RPM `-1500/6500/-1500/6500、6500/-1500/6500/-1500`（m1/m2/m3/m4）；循线档
     *   `trackxian5、trackxian8`；时序 `300、1、50、500、150` ms。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/*
 * 4→3 主流程路线。该段使用连续 PID 循线并将 vx 从 1.2 逐级升到 2.5，再逐级降速；
 * 这种速度包络用于长直道平顺加减速，末端由灰度和坡度条件定位到 3 区。
 */
void go43()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程。
     * - 检测条件：`data_storage[12]`（前红外）；横滚角 `get_roll<-5、<15` 度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `30、70、100、750、810、870、900`（round_cnt 计数）；通用转向 `5000 RPM / 170 度`；左右轮组 RPM
     *   `3000/3000、-2000/-2000`；循线档 `trackxian5、trackxian`；时序 `150、200、500、50` ms；语音轨道 `3`。
     * - 可调项：里程阈值、目标转角、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 3 区→1 号门→5 区的标定路线；数字 315 表示路线的区域/门序列，而非单个数值。 */
void go315()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[14、15、0、1]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角 `get_roll<15` 度；路口确认
     *   det=`5`。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `40、280、50、150、70、170`（round_cnt 计数）；积分目标角 `80、125、50、25` 度；航向跳变过滤 `20` 度；通用转向
     *   `5000 RPM / 165 度`；左右轮组 RPM `3000/3000、-2000/-2000`；四轮 RPM
     *   `6500/-1500/6500/-1500、-1500/6500/-1500/6500、7000/500/7000/500、500/7000/500/7000`（m1/m2/m3/m4）；循线档
     *   `trackxian5、trackxian8`；时序 `1、150、200、300` ms；语音轨道 `4`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 3 区→2 号门→5 区的标定路线，使用门2对应的里程和转角参数。 */
void go325()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[14、15]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角 `get_roll<15` 度；路口确认
     *   det=`5`。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `40、280、70、200、50、170`（round_cnt 计数）；积分目标角 `25` 度；航向跳变过滤 `20` 度；通用转向 `5000 RPM /
     *   165 度`；左右轮组 RPM `3000/3000、-2000/-2000`；四轮 RPM
     *   `7000/500/7000/500、500/7000/500/7000`（m1/m2/m3/m4）；循线档 `trackxian5、trackxian8`；时序
     *   `1、100、150、300` ms；语音轨道 `4`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 3 区→3 号门→5 区的标定路线，长直道长度和门区切入角针对门3。 */
void go335()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[14、15]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角 `get_roll<15` 度；路口确认
     *   det=`5`。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `50、430、60、100、70、160`（round_cnt 计数）；积分目标角 `135、90、25` 度；航向跳变过滤 `20` 度；通用转向 `5000
     *   RPM / 165 度`；左右轮组 RPM `3000/3000、-2000/-2000`；四轮 RPM
     *   `6500/-1500/6500/-1500、-1500/6500/-1500/6500、500/7000/500/7000`（m1/m2/m3/m4）；循线档
     *   `trackxian5、trackxian8`；时序 `1、100、150、300` ms；语音轨道 `4`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 3 区→4 号门→5 区的标定路线，沿门4方向完成任务后进入平台。 */
void go345()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[14、15、0、1]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角 `get_roll<15`
     *   度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `50、430、40、120、60、100`（round_cnt 计数）；积分目标角 `80` 度；航向跳变过滤 `20` 度；通用转向 `5000 RPM /
     *   165 度`；左右轮组 RPM `3000/3000、-2000/-2000`；四轮 RPM
     *   `6500/-1500/6500/-1500、500/7000/500/7000`（m1/m2/m3/m4）；循线档 `trackxian5、trackxian8`；时序
     *   `200、1、150、300、500` ms；语音轨道 `4`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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


/* 3 区→1 号门→6 区路线：执行门1任务点动作后转入 6 区通道。 */
void go3men16()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[14、15、4、5、6、7、8、9、10、11、0、1]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角
     *   `get_roll<15` 度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `50、150、100、55、20`（round_cnt 计数）；积分目标角 `80、140` 度；航向跳变过滤 `20` 度；通用转向 `5000 RPM / 6
     *   度、5000 RPM / 8 度、5000 RPM / 165 度`；左右轮组 RPM `3000/3000、-2000/-2000`；四轮 RPM
     *   `6500/-1500/6500/-1500、-1500/6500/-1500/6500`（m1/m2/m3/m4）；循线档
     *   `trackxian5、trackxian8、trackxian3`；时序 `1、1000、250、150、2500、300` ms。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 3 区→2 号门→6 区路线：结构与门1版本相同，使用门2专属里程和转角。 */
void go3men26()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[14、15、0、1、4、5、6、7、8、9、10、11]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角
     *   `get_roll<15` 度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `50、150、40、90、55、20`（round_cnt 计数）；积分目标角 `30、90、140、80` 度；航向跳变过滤 `20` 度；通用转向 `5000
     *   RPM / 6 度、5000 RPM / 8 度、5000 RPM / 165 度`；左右轮组 RPM `3000/3000、-2000/-2000`；四轮 RPM
     *   `6500/-1500/6500/-1500、-1500/6500/-1500/6500`（m1/m2/m3/m4）；循线档
     *   `trackxian5、trackxian8、trackxian3`；时序 `200、1、300、1000、250、150、2500` ms。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 3 区→3 号门→6 区路线：先通过较长高速段，再执行门3任务点和回程。 */
void go3men36()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[14、15、4、5、6、7、8、9、10、11、0、1]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角
     *   `get_roll<15` 度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `50、400、250、55、20`（round_cnt 计数）；积分目标角 `135、30、140、80` 度；航向跳变过滤 `20` 度；通用转向 `5000
     *   RPM / 6 度、5000 RPM / 8 度、5000 RPM / 165 度`；左右轮组 RPM `3000/3000、-2000/-2000`；四轮 RPM
     *   `6500/-1500/6500/-1500、-1500/6500/-1500/6500`（m1/m2/m3/m4）；循线档
     *   `trackxian5、trackxian8、trackxian3`；时序 `1、300、1000、250、150、2500、200` ms。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 3 区→4 号门→6 区路线：沿门4标定支路完成任务后驶往 6 区。 */
void go3men46()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[14、15、4、5、6、7、8、9、10、11、0、1]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角
     *   `get_roll<15` 度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `50、400、60、260、100、55、20`（round_cnt 计数）；积分目标角 `80、140` 度；航向跳变过滤 `20` 度；通用转向 `5000
     *   RPM / 6 度、5000 RPM / 8 度、5000 RPM / 165 度`；左右轮组 RPM `3000/3000、-2000/-2000`；四轮 RPM
     *   `6500/-1500/6500/-1500、6500/-1000/6500/-1000、-1500/6500/-1500/6500`（m1/m2/m3/m4）；循线档
     *   `trackxian5、trackxian8、trackxian3`；时序 `200、1、100、1000、250、150、2500` ms。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 附加任务“门1甲”的总编排：串联 3→门1→5、5→1→7、7→8、8→6 和 6→门1甲。 */
void gomen1jia()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 流程/调参：本函数不维护局部变量或直接标定值，依次调用 go3men15→go517→go78→go86→go6men1jia；
     *   应在对应子路线内调整里程、转角、RPM 与时序，避免在本编排器中重复设定同类参数。
     */
    go3men15();
    go517();
    go78();
    go86();
    go6men1jia();
}


/* 3 区→门1→5 路线：在门区完成两处语音/倒车任务，再回到 5 区平台。 */
void go3men15()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程。
     * - 检测条件：灰度 `hui[14、15]`（0 表示压到黑线）；横滚角 `get_roll<-5、<15` 度；路口确认
     *   det=`5`。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `30、60、160、250、270、40、80、200、290、230、260`（round_cnt 计数）；通用转向 `3000 RPM / 85 度、3000
     *   RPM / -85 度`；左右轮组 RPM `-3000/-3000`；循线档 `trackxian5、trackxian8、trackxian12、trackxian15`；时序
     *   `50、1、100、950` ms；for 上限 `90`（循环次数，不自动等同距离）；语音轨道 `10、12、5`。
     * - 可调项：里程阈值、目标转角、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 5→1→7 复合路线：穿过 1 号支路及多个路口，跨越坡面后在 7 区前完成平台动作。 */
void go517()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1、8、9、10、11、12、13、14、15]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角 `get_roll<15`
     *   度；路口确认 det=`5`。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `50、70、130、150、170、30、80、40、100、120、280`（round_cnt 计数）；积分目标角 `30、95、166` 度；航向跳变过滤
     *   `45、20` 度；左右轮组 RPM `3000/3000、-2000/-2000`；四轮 RPM
     *   `-1000/8000/-1000/8000、-2000/8000/-2000/8000、-5000/5000/-5000/5000`（m1/m2/m3/m4）；循线档
     *   `trackxian8、trackxian5、trackxian12、trackxian10、trackxian3`；时序 `1、500、250、50、150` ms；语音轨道
     *   `6`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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



    setspeed2(3000, 3000);
    osDelay(500);
    setspeed2(-2000, -2000);
    osDelay(250);
    stop();
    osDelay(50);

    osDelay(150);



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


/* 6 区→门1甲终点：路口切入后用桥面红外纠偏，通过多段高速路线并以 gohome() 等价姿态收尾。 */
void go6men1jia()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1、2、3、4、5、6、7、8、9、10、11、12、13、14、15]`（0 表示压到黑线）；横滚角 `get_roll<15、>5` 度；路口确认
     *   det=`4、5`。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `30、63、35、60、90、140、160、40、150、170、190、200、50`（round_cnt 计数）；积分目标角
     *   `75、8、15、70、30、25` 度；航向跳变过滤 `20、45` 度；通用转向 `5000 RPM / 165 度`；左右轮组 RPM
     *   `0/5000、-500/5000、3000/3000`；四轮 RPM
     *   `-1000/8000/-1000/8000、-1500/3000/-1500/3000、5000/8000/5000/8000、8000/0/8000/0、1000/8000/1000/8000、8000/5000/8000/5000`（m1/m2/m3/m4）；循线档
     *   `trackxian5、trackxian3、trackxian8、trackxian12、trackxian15、trackxian10`；时序
     *   `1、120、100、1000、50、300、200` ms；语音轨道 `9`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

}

/* 新版门1甲总路线：增加 15 档高速段与全黑位倒车定位，完成往返后接续 5→7→8→6。 */
void xingomen1jia()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程。
     * - 检测条件：灰度 `hui[14、15]`（0 表示压到黑线）；`data_storage[16]`（R5 全黑兼容位）；横滚角 `get_roll<15` 度；路口确认
     *   det=`5`。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `40、80、150、170、190、200、30、50、220、240、260、280、100、180、230`（round_cnt 计数）；通用转向 `5000
     *   RPM / 80 度、5000 RPM / -70 度`；左右轮组 RPM `-5000/-5000`；循线档
     *   `trackxian8、trackxian12、trackxian15、trackxian10、trackxian5`；时序 `50、1、100、150、200` ms；for 上限
     *   `300`（循环次数，不自动等同距离）；语音轨道 `8、4`。
     * - 可调项：里程阈值、目标转角、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 3 区→门2→5 路线：经门2支路的两次定角和高速直段后调用 goplat()。 */
void go3men25()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[14、15、0、1]`（0 表示压到黑线）；横滚角 `get_roll<-10、<15`
     *   度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `30、70、170、200、240、60、270、300、230`（round_cnt 计数）；积分目标角 `40、35` 度；航向跳变过滤 `20` 度；四轮
     *   RPM `6000/-1000/6000/-1000、-1000/6000/-1000/6000`（m1/m2/m3/m4）；循线档
     *   `trackxian5、trackxian8、trackxian12、trackxian15`；时序 `1、20` ms；语音轨道 `5`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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




    while(get_rount_cnt-intia_rount_cnt<240)
    	trackxian15();




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


/* 5→8 路线：先设置舵机通道 0，再穿过连续折线、坡面和桥区到达 8 区。 */
void go58()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1、14、15]`（0 表示压到黑线）；`data_storage[12]`（前红外）；路口确认
     *   det=`5`。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `70、40、80、50、150、100、270、140、90`（round_cnt 计数）；积分目标角 `40、20、80、110、150` 度；航向跳变过滤
     *   `20` 度；通用转向 `5000 RPM / 165 度`；左右轮组 RPM `3000/3000、-2000/-2000`；四轮 RPM
     *   `500/7000/500/7000、7000/500/7000/500、6500/-1500/6500/-1500、-1500/6500/-1500/6500`（m1/m2/m3/m4）；循线档
     *   `trackxian5、trackxian8、trackxian3`；时序 `1、150、100、300、1000` ms；语音轨道 `7`；舵机 `700 us / 通道
     *   0、2000 us / 通道 0`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛、舵机脉宽与到位等待均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 8→6 路线：先以 1 档低速脱离 8 区，再经路口定角、桥面纠偏和坡度检测进入 6 区。 */
void go86()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1、14、15、4、5、6、7、8、9、10、11]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角
     *   `get_roll<15` 度；路口确认 det=`4`。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `250、40、50、60、260、150、30、75、20`（round_cnt 计数）；积分目标角 `145、105、80、30` 度；航向跳变过滤 `20`
     *   度；通用转向 `5000 RPM / 6 度、5000 RPM / 8 度、5000 RPM / 168 度`；左右轮组 RPM `3000/3000、-2000/-2000`；四轮
     *   RPM `-1500/6500/-1500/6500、6500/-1500/6500/-1500、-1500/3000/-1500/3000`（m1/m2/m3/m4）；循线档
     *   `trackxian1、trackxian5、trackxian8、trackxian3`；时序 `1、100、150、250、2500、200` ms；语音轨道 `5`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 8→5 路线：与 go86 共用起始低速段，按 5 区方向的灰度边沿和转角进入平台。 */
void go85()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1、14、15]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角 `get_roll<15` 度；路口确认
     *   det=`5`。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `250、40、50、60、260、200、150、170`（round_cnt 计数）；积分目标角 `145、105、80、40、90、25` 度；航向跳变过滤
     *   `20` 度；通用转向 `5000 RPM / 165 度`；左右轮组 RPM `3000/3000、-2000/-2000`；四轮 RPM
     *   `-1500/6500/-1500/6500、6500/-1500/6500/-1500、500/7000/500/7000`（m1/m2/m3/m4）；循线档
     *   `trackxian1、trackxian5、trackxian8`；时序 `1、100、150、200` ms；语音轨道 `4`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 8 区→1 号门→4 区路线；后两位 14 表示任务门编号与目标区域。 */
void go8men14()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1、14、15]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角 `get_roll<15`
     *   度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `250、40、50、60、260、200、170、100、350`（round_cnt 计数）；积分目标角 `140、110、80` 度；航向跳变过滤 `20`
     *   度；通用转向 `5000 RPM / 165 度`；左右轮组 RPM `3000/3000、-2000/-2000`；四轮 RPM
     *   `-1500/6500/-1500/6500、6500/-1500/6500/-1500`（m1/m2/m3/m4）；循线档
     *   `trackxian1、trackxian5、trackxian8`；时序 `1、150、200` ms；语音轨道 `3`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 8 区→1 号门→3 区路线；门区任务相同，离开后的行程和转角按 3 区标定。 */
void go8men13()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1、14、15]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角 `get_roll<15`
     *   度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `250、40、50、60、260、200、170`（round_cnt 计数）；积分目标角 `140、110、80` 度；航向跳变过滤 `20` 度；通用转向
     *   `5000 RPM / 165 度`；左右轮组 RPM `3000/3000、-2000/-2000`；四轮 RPM
     *   `-1500/6500/-1500/6500、6500/-1500/6500/-1500`（m1/m2/m3/m4）；循线档
     *   `trackxian1、trackxian5、trackxian8`；时序 `1、150、200` ms；语音轨道 `2`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 8 区→1 号门→2 区路线；通过 1 号门任务点后转向 2 区出口。 */
void go8men12()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1、14、15]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角 `get_roll<15`
     *   度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `250、40、50、60、260、200、170、100、20、80、10`（round_cnt 计数）；积分目标角 `140、110、80、135、20`
     *   度；航向跳变过滤 `20` 度；通用转向 `5000 RPM / 165 度`；左右轮组 RPM `3000/3000、-2000/-2000`；四轮 RPM
     *   `-1500/6500/-1500/6500、6500/-1500/6500/-1500`（m1/m2/m3/m4）；循线档
     *   `trackxian1、trackxian5、trackxian8、trackxian3、trackxian2`；时序 `1、150、200` ms；语音轨道 `1`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 8 区→2 号门→4 区路线；使用门2入口、任务点及 4 区出口的完整标定序列。 */
void go8men24()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1、14、15]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角 `get_roll<15`
     *   度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `250、40、50、60、260、200、100、350`（round_cnt 计数）；积分目标角 `140、110、80、40、90、135` 度；航向跳变过滤
     *   `20` 度；通用转向 `5000 RPM / 165 度`；左右轮组 RPM `3000/3000、-2000/-2000`；四轮 RPM
     *   `-1500/6500/-1500/6500、6500/-1500/6500/-1500`（m1/m2/m3/m4）；循线档
     *   `trackxian1、trackxian5、trackxian8`；时序 `1、150、100、200` ms；语音轨道 `3`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 8 区→2 号门→3 区路线；末段从门2支路接入 3 区。 */
void go8men23()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1、14、15]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角 `get_roll<15`
     *   度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `250、40、50、60、260、200、100`（round_cnt 计数）；积分目标角 `140、110、80、40、90、45` 度；航向跳变过滤 `20`
     *   度；通用转向 `5000 RPM / 165 度`；左右轮组 RPM `3000/3000、-2000/-2000`；四轮 RPM
     *   `-1500/6500/-1500/6500、6500/-1500/6500/-1500`（m1/m2/m3/m4）；循线档
     *   `trackxian1、trackxian5、trackxian8`；时序 `1、150、100、300、200` ms；语音轨道 `2`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 8 区→2 号门→2 区路线；门区往返后沿 2 区出口离开。 */
void go8men22()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1、14、15]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角 `get_roll<15`
     *   度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `250、40、50、60、260、200、100、20、80、10`（round_cnt 计数）；积分目标角 `140、110、80、40、90、135、20`
     *   度；航向跳变过滤 `20` 度；通用转向 `5000 RPM / 165 度`；左右轮组 RPM `3000/3000、-2000/-2000`；四轮 RPM
     *   `-1500/6500/-1500/6500、6500/-1500/6500/-1500`（m1/m2/m3/m4）；循线档
     *   `trackxian1、trackxian5、trackxian8、trackxian3、trackxian2`；时序 `1、150、100、200` ms；语音轨道 `1`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 8 区→3 号门→4 区路线；包含 3 号门任务点及通往 4 区的长直段。 */
void go8men34()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1、14、15]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角 `get_roll<15`
     *   度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `250、40、50、60、260、200、280`（round_cnt 计数）；积分目标角 `140、110、80、40` 度；航向跳变过滤 `20`
     *   度；通用转向 `5000 RPM / 165 度`；左右轮组 RPM `3000/3000、-2000/-2000`；四轮 RPM
     *   `-1500/6500/-1500/6500、6500/-1500/6500/-1500`（m1/m2/m3/m4）；循线档
     *   `trackxian1、trackxian5、trackxian8`；时序 `1、150、100、200` ms；语音轨道 `3`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 8 区→3 号门→3 区路线；按门3回到 3 区的方向完成转角和里程控制。 */
void go8men33()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1、14、15]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角 `get_roll<15`
     *   度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `250、40、50、60、260、200、280、450`（round_cnt 计数）；积分目标角 `140、110、80、40、130` 度；航向跳变过滤
     *   `20` 度；通用转向 `5000 RPM / 165 度`；左右轮组 RPM `3000/3000、-2000/-2000`；四轮 RPM
     *   `-1500/6500/-1500/6500、6500/-1500/6500/-1500`（m1/m2/m3/m4）；循线档
     *   `trackxian1、trackxian5、trackxian8`；时序 `1、150、100、200` ms；语音轨道 `2`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 8 区→3 号门→2 区路线；门3任务完成后沿 2 区方向退出。 */
void go8men32()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1、14、15]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角 `get_roll<15`
     *   度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `250、40、50、60、260、200、280、80、10`（round_cnt 计数）；积分目标角 `140、110、80、40、130、20`
     *   度；航向跳变过滤 `20` 度；通用转向 `5000 RPM / 165 度`；左右轮组 RPM `3000/3000、-2000/-2000`；四轮 RPM
     *   `-1500/6500/-1500/6500、6500/-1500/6500/-1500`（m1/m2/m3/m4）；循线档
     *   `trackxian1、trackxian5、trackxian8、trackxian3、trackxian2`；时序 `1、150、100、300、200` ms；语音轨道 `1`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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


/* 8 区→4 号门→4 区路线；使用门4任务点和 4 区出口的标定参数。 */
void go8men44()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1、14、15]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角 `get_roll<15`
     *   度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `250、40、50、60、260、200、100、180`（round_cnt 计数）；积分目标角 `140、110、80、40、70、130` 度；航向跳变过滤
     *   `20` 度；通用转向 `5000 RPM / 165 度`；左右轮组 RPM `3000/3000、-2000/-2000`；四轮 RPM
     *   `-1500/6500/-1500/6500、6500/-1500/6500/-1500`（m1/m2/m3/m4）；循线档
     *   `trackxian1、trackxian5、trackxian8`；时序 `1、150、100、200` ms；语音轨道 `3`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 8 区→4 号门→3 区路线；从门4支路转入 3 区。 */
void go8men43()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1、14、15]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角 `get_roll<15`
     *   度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `250、40、50、60、260、200、100、180、400`（round_cnt 计数）；积分目标角 `140、110、80、40、70、130`
     *   度；航向跳变过滤 `20` 度；通用转向 `5000 RPM / 165 度`；左右轮组 RPM `3000/3000、-2000/-2000`；四轮 RPM
     *   `-1500/6500/-1500/6500、6500/-1500/6500/-1500`（m1/m2/m3/m4）；循线档
     *   `trackxian1、trackxian5、trackxian8`；时序 `1、150、100、200` ms；语音轨道 `2`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 8 区→4 号门→2 区路线；完成门4动作后从 2 区出口离开。 */
void go8men42()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1、14、15]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角 `get_roll<15`
     *   度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `250、40、50、60、260、200、100、180、80、10`（round_cnt 计数）；积分目标角 `140、110、80、40、70、130、20`
     *   度；航向跳变过滤 `20` 度；通用转向 `5000 RPM / 165 度`；左右轮组 RPM `3000/3000、-2000/-2000`；四轮 RPM
     *   `-1500/6500/-1500/6500、6500/-1500/6500/-1500`（m1/m2/m3/m4）；循线档
     *   `trackxian1、trackxian5、trackxian8、trackxian3、trackxian2`；时序 `1、150、100、300、200` ms；语音轨道 `1`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 附加任务“门2甲”的总编排：3→门2→5→7→8→6→门2甲。 */
void gomen2jia()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 流程/调参：本函数只串联 go3men25→go57→go78→go86→go6men2jia；没有独立阈值，
     *   所有可调参数归属到被调用的具体路线函数。
     */
    go3men25();
    go57();
    go78();
    go86();
    go6men2jia();
}


/* 6 区→门2甲终点路线：通过路口切入、桥面纠偏和高速直段抵达门2附加任务点。 */
void go6men2jia()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1、2、3、4、5、6、7、8、9、10、11、12、13、14、15]`（0 表示压到黑线）；`data_storage[15]`（R4
     *   全黑兼容位）；横滚角 `get_roll<15、>5` 度；路口确认 det=`4、5`。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `30、65、35、60、90、160、180、190、130、140、150、50`（round_cnt 计数）；积分目标角
     *   `75、9、15、30、80、90、25` 度；航向跳变过滤 `20、45` 度；通用转向 `5000 RPM / -70 度、5000 RPM / 165 度`；左右轮组 RPM
     *   `0/5000、-500/5000、-5000/-5000、3000/3000`；四轮 RPM
     *   `-1000/8000/-1000/8000、-1500/3000/-1500/3000、5000/8000/5000/8000、8000/1000/8000/1000、-2000/8000/-2000/8000、8000/-2000/8000/-2000、5000/-2000/5000/-2000、1000/8000/1000/8000、8000/5000/8000/5000`（m1/m2/m3/m4）；循线档
     *   `trackxian5、trackxian3、trackxian8、trackxian12、trackxian10`；时序 `1、120、100、1000、50、200、300`
     *   ms；for 上限 `300`（循环次数，不自动等同距离）；语音轨道 `8、9`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

}

/* 新版门2甲总路线：采用更长的 15 档高速段和新的任务点倒车定位参数。 */
void xingomen2jia()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[14、15、0、1]`（0 表示压到黑线）；横滚角 `get_roll<15` 度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `40、80、150、170、190、50、260、300、320、340、60、200、230`（round_cnt 计数）；积分目标角 `26、35`
     *   度；航向跳变过滤 `20` 度；四轮 RPM `7000/1000/7000/1000、5000/8000/5000/8000`（m1/m2/m3/m4）；循线档
     *   `trackxian8、trackxian12、trackxian15、trackxian5、trackxian10`；时序 `1` ms；语音轨道 `4`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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


/* 4 区→门4→5 区连接段；主流程第四分支用它从 4 号任务门进入公共 5 区路线。 */
void go4men45()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程。
     * - 检测条件：灰度 `hui[0、1]`（0 表示压到黑线）；横滚角 `get_roll<15` 度；路口确认
     *   det=`4`。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `30、60、200、230、250、20、50、170、100、270、280`（round_cnt 计数）；循线档
     *   `trackxian8、trackxian12、trackxian15、trackxian5、trackxian10`；语音轨道 `5`。
     * - 可调项：里程阈值、RPM/循线档、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 6 区→门3甲终点路线：门3附加任务的路口、桥面和终点姿态标定。 */
void go6men3jia()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1、2、3、4、5、6、7、8、9、10、11、12、13、14、15]`（0 表示压到黑线）；`data_storage[16]`（R5
     *   全黑兼容位）；横滚角 `get_roll<15、>5` 度；路口确认 det=`4、5`。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `30、65、35、60、90、150、170、180、100、270、290、310、320、20、80、50、160`（round_cnt 计数）；积分目标角
     *   `75、9、15、28、87、90、25` 度；航向跳变过滤 `20、45` 度；通用转向 `5000 RPM / -70 度、5000 RPM / 165 度`；左右轮组 RPM
     *   `0/5000、-500/5000、-5000/-5000、3000/3000`；四轮 RPM
     *   `-1000/8000/-1000/8000、-1500/3000/-1500/3000、5000/8000/5000/8000、8000/1000/8000/1000、-2000/8000/-2000/8000、-2000/5000/-2000/5000、8000/-2000/8000/-2000、5000/-2000/5000/-2000、8000/5000/8000/5000`（m1/m2/m3/m4）；循线档
     *   `trackxian5、trackxian3、trackxian8、trackxian12、trackxian10、trackxian15`；时序
     *   `1、120、100、1000、50、200、300` ms；for 上限 `300`（循环次数，不自动等同距离）；语音轨道 `8、9`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

}


/* 6 区→门4甲终点路线：门4附加任务的路口、桥面和终点姿态标定。 */
void go6men4jia()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1、2、3、4、5、6、7、8、9、10、11、12、13、14、15]`（0 表示压到黑线）；`data_storage[16]`（R5
     *   全黑兼容位）；横滚角 `get_roll<15、>5` 度；路口确认 det=`4、5`。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `30、65、35、60、90、140、160、50、110、130、150、170、180`（round_cnt 计数）；积分目标角
     *   `75、9、15、30、60、90、25` 度；航向跳变过滤 `20、45` 度；通用转向 `5000 RPM / -70 度、5000 RPM / 165 度`；左右轮组 RPM
     *   `0/5000、-500/5000、-5000/-5000、3000/3000`；四轮 RPM
     *   `-1000/8000/-1000/8000、-1500/3000/-1500/3000、5000/8000/5000/8000、8000/1000/8000/1000、8000/-1000/8000/-1000、-2000/8000/-2000/8000、8000/-2000/8000/-2000、5000/-2000/5000/-2000、8000/5000/8000/5000`（m1/m2/m3/m4）；循线档
     *   `trackxian5、trackxian3、trackxian8、trackxian12、trackxian15、trackxian10`；时序
     *   `1、120、100、1000、50、200、300` ms；for 上限 `300`（循环次数，不自动等同距离）；语音轨道 `8、9`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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


}

/* 附加任务“门4甲”的总编排：4→门4→5→7→8→6→门4甲→3 号回程。 */
void go4men4jia()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 流程/调参：按 go4men45→go57→go78→go86→go6men43→go3jia 执行；本函数没有局部计数或
     *   直接控制数值，标定应落在对应的分段路线中。
     */
    go4men45();
    go57();
    go78();
    go86();
    go6men43();
    go3jia();
}

/* 新版门4甲路线：15 档高速直段和新的门4甲终点标定，供调试时替换旧路线。 */
void xingo4men4jia()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程。
     * - 检测条件：灰度 `hui[0、1]`（0 表示压到黑线）；横滚角 `get_roll<15` 度；路口确认
     *   det=`4`。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `30、60、160、180、200、20、40、150、170、190、80、220`（round_cnt 计数）；循线档
     *   `trackxian8、trackxian12、trackxian15、trackxian5、trackxian10`；语音轨道 `4`。
     * - 可调项：里程阈值、RPM/循线档、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 附加任务“门3甲”的总编排：4→门4→5→7→8→6→门3甲→3 号回程。 */
void go4men3jia()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 流程/调参：按 go4men45→go57→go78→go86→go6men33→go3jia 执行；它仅决定任务组合，
     *   不拥有独立的里程、角度或 RPM 参数。
     */
    go4men45();
    go57();
    go78();
    go86();
    go6men33();
    go3jia();
}

/* 新版门3甲路线：与旧版任务序列相同，重新标定高速里程和门区转角。 */
void xingo4men3jia()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程。
     * - 检测条件：灰度 `hui[0、1]`（0 表示压到黑线）；横滚角 `get_roll<15` 度；路口确认
     *   det=`4`。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `30、60、160、180、200、20、40、150、170、190、80、220`（round_cnt 计数）；循线档
     *   `trackxian8、trackxian12、trackxian15、trackxian5、trackxian10`；语音轨道 `4`。
     * - 可调项：里程阈值、RPM/循线档、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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


/* 3 区→门2→5→7→8→6→门1甲的组合路线，供门2甲方案的前置路径复用。 */
void go3men2jia()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 流程/调参：按 go3men25→go57→go78→go86→go6men1jia 串联；当前函数无本地控制变量，
     *   调参时应定位到具体子路线，避免重复覆盖其已标定的参数。
     */
    go3men25();
    go57();
    go78();
    go86();
    go6men1jia();
}

/* 新版 3 区→门2 甲路线：以更高档位通过直道，并使用新的门区定位参数。 */
void xingo3men2jia()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[14、15、0、1]`（0 表示压到黑线）；横滚角 `get_roll<15` 度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `40、80、150、170、190、30、60、220、240、260、250、270`（round_cnt 计数）；积分目标角 `28、35` 度；航向跳变过滤
     *   `20` 度；四轮 RPM `8000/1000/8000/1000、1000/8000/1000/8000`（m1/m2/m3/m4）；循线档
     *   `trackxian8、trackxian12、trackxian15、trackxian5`；时序 `1` ms；语音轨道 `4`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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


/* 直道→门1甲终点：det=4 路口确认后连续两次高速寻线，执行语音、倒车和 80 度转向。 */
void gozhimen1jia()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1、14、15]`（0 表示压到黑线）；横滚角 `get_roll<15、>5` 度；路口确认
     *   det=`4、5`。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `80、120、140、160、40、60、130、150、170、190、20、50、30、90`（round_cnt 计数）；积分目标角 `70、30`
     *   度；航向跳变过滤 `20` 度；通用转向 `5000 RPM / 165 度`；左右轮组 RPM `10000/8000、3000/3000`；四轮 RPM
     *   `8000/0/8000/0、1000/8000/1000/8000`（m1/m2/m3/m4）；循线档
     *   `trackxian5、trackxian8、trackxian12、trackxian10、trackxian15、trackxian3`；时序 `200、60、1、50、300`
     *   ms；语音轨道 `9`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

}


/* 直道→门2甲终点：结构同 gozhimen1jia，门2方向的里程与角度参数独立标定。 */
void gozhimen2jia()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1、14、15、8、9、10、11、12、13]`（0 表示压到黑线）；`data_storage[12、16]`（前红外；R5 全黑兼容位）；横滚角
     *   `get_roll<15、>5` 度；路口确认 det=`4、6`。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `80、120、160、30、140、150、20、50、90`（round_cnt 计数）；积分目标角 `30、80、90` 度；航向跳变过滤 `20`
     *   度；通用转向 `5000 RPM / -70 度、5000 RPM / 165 度`；左右轮组 RPM `10000/8000、-5000/-5000、3000/3000`；四轮
     *   RPM
     *   `8000/1000/8000/1000、-2000/8000/-2000/8000、8000/-2000/8000/-2000、5000/-2000/5000/-2000、1000/8000/1000/8000、8000/0/8000/0`（m1/m2/m3/m4）；循线档
     *   `trackxian5、trackxian8、trackxian12、trackxian3`；时序 `200、60、1、100、50、300` ms；for 上限
     *   `200`（循环次数，不自动等同距离）；语音轨道 `8、9`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

}


/* 直道→门3甲终点：通过门3附加任务点后按终点姿态完成停车。 */
void gozhimen3jia()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1、14、15、7、8、9、10、11、12、13]`（0 表示压到黑线）；`data_storage[12、16]`（前红外；R5 全黑兼容位）；横滚角
     *   `get_roll<15、>5` 度；路口确认 det=`4、5`。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `80、120、160、50、280、300、320、20、100、30、90`（round_cnt 计数）；积分目标角 `30、90` 度；航向跳变过滤 `20`
     *   度；通用转向 `5000 RPM / -70 度、5000 RPM / 165 度`；左右轮组 RPM `10000/8000、-5000/-5000、3000/3000`；四轮
     *   RPM
     *   `8000/1000/8000/1000、-2000/8000/-2000/8000、-2000/5000/-2000/5000、8000/-2000/8000/-2000、5000/-2000/5000/-2000、8000/0/8000/0`（m1/m2/m3/m4）；循线档
     *   `trackxian5、trackxian8、trackxian12、trackxian15、trackxian3`；时序 `200、60、1、100、300` ms；for 上限
     *   `200`（循环次数，不自动等同距离）；语音轨道 `8、9`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

}


/* 直道→门4甲终点：通过门4附加任务点后按终点姿态完成停车。 */
void gozhimen4jia()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1、14、15、7、8、9、10、11、12、13]`（0 表示压到黑线）；`data_storage[12、16]`（前红外；R5 全黑兼容位）；横滚角
     *   `get_roll<15、>5` 度；路口确认 det=`4、5`。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `80、120、160、30、50、110、130、140、150、170、20、100、90`（round_cnt 计数）；积分目标角 `30、60、90`
     *   度；航向跳变过滤 `20` 度；通用转向 `5000 RPM / -70 度、5000 RPM / 165 度`；左右轮组 RPM
     *   `10000/8000、-5000/-5000、-500/5000、3000/3000`；四轮 RPM
     *   `8000/1000/8000/1000、8000/-1000/8000/-1000、-2000/8000/-2000/8000、8000/-2000/8000/-2000、5000/-2000/5000/-2000、8000/0/8000/0`（m1/m2/m3/m4）；循线档
     *   `trackxian5、trackxian8、trackxian12、trackxian15、trackxian3`；时序 `200、60、1、100、300` ms；for 上限
     *   `200`（循环次数，不自动等同距离）；语音轨道 `8、9`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

}


/* 4 区→直道→3 区：先以 8/12 档通过长直段，再经连续门区转向回到 3 区。 */
void go4zhi3()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[14、15、0、1、2、3、4、5、6]`（0 表示压到黑线）；`data_storage[12]`（前红外）；横滚角 `get_roll<15`
     *   度。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `100、350、380、420、50、70、170、190、210、80、230、250、140、160、180`（round_cnt 计数）；积分目标角
     *   `23、80` 度；航向跳变过滤 `20` 度；通用转向 `5000 RPM / 167 度`；左右轮组 RPM `3000/3000、-2000/-2000`；四轮 RPM
     *   `10000/7000/10000/7000、8000/-2000/8000/-2000、5000/-2000/5000/-2000`（m1/m2/m3/m4）；循线档
     *   `trackxian8、trackxian12、trackxian15、trackxian10、trackxian5`；时序 `1、400、200、100` ms；语音轨道
     *   `8、2`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */



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


/* 附加任务“门3甲”的总编排：3→门3→5→7→8→6→门3甲。 */
void gomen3jia()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1]`（0 表示压到黑线）；横滚角 `get_roll<15` 度；路口确认
     *   det=`4、5`。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `30、60、160、180、200、50、20、120、140、150、40、230`（round_cnt 计数）；积分目标角 `110、35` 度；航向跳变过滤
     *   `20` 度；通用转向 `6000 RPM / 165 度`；四轮 RPM
     *   `-1000/8000/-1000/8000、5000/8000/5000/8000`（m1/m2/m3/m4）；循线档
     *   `trackxian8、trackxian12、trackxian15、trackxian5、trackxian10`；时序 `700、150、1、100、500` ms；语音轨道
     *   `4`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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


/* 6 区→门4→3 区：门4方向附加任务完成后回到 3 区，末段按终点平台动作收尾。 */
void go6men43()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1、2、3、4、5、6、7、8、9、10、11、12、13、14、15]`（0 表示压到黑线）；`data_storage[16]`（R5
     *   全黑兼容位）；横滚角 `get_roll<15` 度；路口确认 det=`4、5`。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `30、65、35、60、90、140、160、50、110、130、150、170、180、80、400、460、500`（round_cnt 计数）；积分目标角
     *   `75、9、15、30、60、90` 度；航向跳变过滤 `20、45` 度；通用转向 `5000 RPM / -70 度`；左右轮组 RPM
     *   `0/5000、-500/5000、-5000/-5000`；四轮 RPM
     *   `-1000/8000/-1000/8000、-1500/3000/-1500/3000、5000/8000/5000/8000、8000/1000/8000/1000、8000/-1000/8000/-1000、-2000/8000/-2000/8000`（m1/m2/m3/m4）；循线档
     *   `trackxian5、trackxian3、trackxian8、trackxian12、trackxian15、trackxian10`；时序
     *   `1、120、100、1000、50、200` ms；for 上限 `300`（循环次数，不自动等同距离）；语音轨道 `8、2`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 3 号附加任务公共段：从 3 区高速进入任务门，执行终点动作后为组合路线提供回程。 */
void go3jia()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1]`（0 表示压到黑线）；横滚角 `get_roll<15、>5` 度；路口确认
     *   det=`4`。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `50、90、200、240、280、160`（round_cnt 计数）；积分目标角 `30、25` 度；航向跳变过滤 `20` 度；通用转向 `5000 RPM
     *   / 165 度`；左右轮组 RPM `3000/3000`；四轮 RPM
     *   `1000/8000/1000/8000、8000/5000/8000/5000`（m1/m2/m3/m4）；循线档
     *   `trackxian8、trackxian12、trackxian15、trackxian10、trackxian5、trackxian3`；时序 `50、1、300、200`
     *   ms；语音轨道 `9`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

}

/* 6 区→门3→3 区：与 go6men43 对称的门3方向回程。 */
void go6men33()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1、2、3、4、5、6、7、8、9、10、11、12、13、14、15]`（0 表示压到黑线）；`data_storage[16]`（R5
     *   全黑兼容位）；横滚角 `get_roll<15` 度；路口确认 det=`4、5`。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `30、65、35、60、90、150、170、180、100、280、300、320、330、80、420、460、500`（round_cnt
     *   计数）；积分目标角 `75、9、15、28、80` 度；航向跳变过滤 `20、45` 度；通用转向 `5000 RPM / -70 度`；左右轮组 RPM
     *   `0/5000、-500/5000、-5000/-5000`；四轮 RPM
     *   `-1000/8000/-1000/8000、-1500/3000/-1500/3000、5000/8000/5000/8000、8000/1000/8000/1000、-2000/8000/-2000/8000、-2000/5000/-2000/5000`（m1/m2/m3/m4）；循线档
     *   `trackxian5、trackxian3、trackxian8、trackxian12、trackxian10、trackxian15`；时序
     *   `1、120、100、1000、50、200` ms；for 上限 `300`（循环次数，不自动等同距离）；语音轨道 `8、2`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 3 区→门1路线：从坡面进入门1，按 5/8/12 档通过直道并在门前定角停车。 */
void go3men1()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程。
     * - 检测条件：灰度 `hui[14、15]`（0 表示压到黑线）；横滚角 `get_roll<-5` 度；路口确认
     *   det=`4`。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `30、60、220、250、80`（round_cnt 计数）；循线档 `trackxian5、trackxian8、trackxian12`；时序 `50`
     *   ms。
     * - 可调项：里程阈值、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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

/* 新版门3甲组合路线：采用新版高速门区参数，完成 3 区进入和附加任务编排。 */
void xingomen3jia()
{
    /*
     * 函数参数、状态与可调参数：
     * - 形参：无显式形参；由调用顺序和共享的底盘、IMU、传感器状态决定当前动作。
     * - 变量/状态：`last_angle` 保存上一次航向角（度）；`i` 仅为循环计数器；`intia_rount_cnt` 记录阶段里程零点，差值是当前段行程；`angle_sum`
     *   累加有效航向增量以判断目标转角；`go_yaw_inia` 是相邻航向采样差。
     * - 检测条件：灰度 `hui[0、1]`（0 表示压到黑线）；横滚角 `get_roll<15` 度；路口确认
     *   det=`4`。通道索引由接线和安装位置决定，通常固定；里程、姿态或路口门槛可现场重标定。
     * - 数值：相对里程阈值 `30、60、160、180、200、40、70、100、120、140、150、230`（round_cnt 计数）；积分目标角 `22、35` 度；航向跳变过滤
     *   `20` 度；四轮 RPM `3000/8000/3000/8000、5000/8000/5000/8000`（m1/m2/m3/m4）；循线档
     *   `trackxian8、trackxian12、trackxian15、trackxian5、trackxian10`；时序 `1` ms；语音轨道 `4`。
     * - 可调项：里程阈值、目标转角、IMU
     *   跳变过滤门槛、RPM/循线档、动作时序、姿态/路口门槛均应按车辆、赛道和电池状态逐项标定；先校里程和转角，再调速度与延时。语音编号、通道索引、电机编号及位图编码通常固定。
     */
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
