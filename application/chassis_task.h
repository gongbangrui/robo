/**
 * chassis_task.h — differential-drive chassis control (4 motors, skid-steer).
 *
 * Left side  = motors 0,1 (CAN IDs 0x201, 0x202)
 * Right side = motors 2,3 (CAN IDs 0x203, 0x204)
 *
 * Kinematics (skid-steer, rubber tyres):
 *   v_left  = vx - wz * WHEEL_BASE / 2
 *   v_right = vx + wz * WHEEL_BASE / 2
 *
 * No lateral (vy) motion — rubber tyres can't crab sideways.
 */
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "struct_typedef.h"
#include "pid.h"

#include "user_lib.h"

#define CHASSIS_TASK_INIT_TIME 357

/* ---- wheel geometry ---- */
#define WHEEL_BASE         0.35f   /* track width, left-to-right centre distance (m) */
#define MOTOR_DISTANCE_TO_CENTER (WHEEL_BASE / 2.0f)

/* ---- motor RPM → wheel speed (m/s) ---- */
/* M2006 gear ratio 36:1, wheel diameter 108mm → π*0.108/(36*60) ≈ 1.57e-4 */
#define M2006_MOTOR_RPM_TO_VECTOR       0.00015707963267948966f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M2006_MOTOR_RPM_TO_VECTOR

/* ---- speed → chassis feedback (tune with wheel geometry) ---- */
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ (MOTOR_SPEED_TO_CHASSIS_SPEED_VX / MOTOR_DISTANCE_TO_CENTER)

/* ---- control timing ---- */
#define CHASSIS_CONTROL_TIME_MS   2
#define CHASSIS_CONTROL_TIME      0.002f
#define CHASSIS_CONTROL_FREQUENCE 500.0f

/* ---- speed limits ---- */
#define MAX_WHEEL_SPEED            4.0f
#define NORMAL_MAX_CHASSIS_SPEED_X 4.0f
#define NORMAL_MAX_CHASSIS_SPEED_WZ (NORMAL_MAX_CHASSIS_SPEED_X / MOTOR_DISTANCE_TO_CENTER)

/* ---- motor PID ---- */
#define M3505_MOTOR_SPEED_PID_KP      15000.0f
#define M3505_MOTOR_SPEED_PID_KI      10.0f
#define M3505_MOTOR_SPEED_PID_KD      0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT 10000.0f
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

typedef enum {
    CHASSIS_VECTOR_NO_FOLLOW_YAW,  /* rotation speed control (vx + wz) */
    CHASSIS_VECTOR_RAW,            /* current passed directly to CAN */
    CHASSIS_VECTOR_RPM,            /* 4-wheel independent RPM (via speed PID) */
} chassis_mode_e;

typedef struct {
    const fp32 *chassis_INS_angle;
    chassis_mode_e chassis_mode;
    chassis_mode_e last_chassis_mode;

    /* Motor feedback arrays (indexed 0-3) */
    fp32 motor_accel[4];
    fp32 motor_speed[4];
    fp32 motor_speed_set[4];
    int16_t motor_give_current[4];

    pid_type_def motor_speed_pid[4];

    fp32 vx;        /* measured forward speed (m/s) */
    fp32 wz;        /* measured angular velocity (rad/s) */
    fp32 vx_set;    /* commanded forward speed (m/s) */
    fp32 wz_set;    /* commanded angular velocity (rad/s) */

    fp32 vx_max_speed;
    fp32 vx_min_speed;

    fp32 chassis_yaw;
    fp32 chassis_pitch;
    fp32 chassis_roll;

    /* RPM 直通模式 (CHASSIS_VECTOR_RPM) */
    int16_t motor_rpm_set[4];   /* 旧参数序: {m1, m2, m3, m4} */
    uint8_t rpm_mode;           /* 1 = RPM 直通激活 */
} chassis_move_t;

extern void chassis_task(void const *pvParameters);
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief  Set chassis velocity command.
 *         This is the public API for other tasks to command the chassis.
 * @param  vx: forward speed (m/s)
 * @param  wz: angular velocity (rad/s)
 */
extern void chassis_set_velocity(fp32 vx, fp32 wz);

/**
 * @brief  Set 4-wheel independent RPM targets (via speed PID → current).
 *         RPM 直通模式: 4 轮独立转速目标, 经速度 PID 转电流。
 *         rpm 数组为旧参数序 {m1, m2, m3, m4}:
 *           m1 = 左前, m2 = 右前, m3 = 左后, m4 = 右后 (旧 CAN 序)
 *         内部按新电机序 (左前,左后,右前,右后) 与反装符号重映射。
 */
extern void chassis_set_motor_rpm(const int16_t rpm[4]);

/**
 * @brief  Get current chassis velocity (for monitoring).
 * @param  vx: pointer to store forward speed
 * @param  wz: pointer to store angular velocity
 */
extern void chassis_get_velocity(fp32 *vx, fp32 *wz);

extern chassis_move_t chassis_move;

#endif
