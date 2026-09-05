/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_task.h"
#include "chassis_behaviour.h"

#include <math.h>
#include "cmsis_os.h"

#include "pid.h"

#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"

/* Internal motor data — depends on CAN_receive.h, kept out of the public header */
static const motor_measure_t *chassis_motor_measure_ptrs[4];

/**
  * @brief          "chassis_move" valiable initialization, include pid initialization, remote control data point initialization, 3508 chassis motors
  *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
  * @param[out]     chassis_move_init: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief 初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param [out]     chassis_move_init: "chassis_move"变量指针.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init);

/**
  * @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
  * @param[out]     chassis_move_mode: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief 初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param [out]     chassis_move_mode: "chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode);

/**
  * @brief          when chassis mode change, some param should be changed, suan as chassis yaw_set should be now chassis yaw
  * @param[out]     chassis_move_transit: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          底盘模式改变，有些参数需要改变，例如底盘控制yaw角度设定值应该变成当前底盘yaw角度
  * @param[out]     chassis_move_transit:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
/**
  * @brief 初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief 初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param [out]     chassis_move_update: "chassis_move"变量指针.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
/**
  * @brief          set chassis control set-point, three movement control value is set by "chassis_behaviour_control_set".
  *                 
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          
  * @param [out]     chassis_move_update: "chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
/**
  * @brief          control loop, according to control set-point, calculate motor current, 
  *                 motor current will be sentto motor
  * @param[out]     chassis_move_control_loop: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif



//底盘运动数据
chassis_move_t chassis_move;

/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief 初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param [in]      pvParameters:  空
  * @retval         none
  */
void chassis_task(void const *pvParameters)
{
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    chassis_init(&chassis_move);

    while (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE)
        || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE))
    {
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    }

    for (;;)
    {
        chassis_set_mode(&chassis_move);
        chassis_mode_change_control_transit(&chassis_move);
        chassis_feedback_update(&chassis_move);
        chassis_set_contorl(&chassis_move);
        chassis_control_loop(&chassis_move);

        if (!(toe_is_error(CHASSIS_MOTOR1_TOE) && toe_is_error(CHASSIS_MOTOR2_TOE)
           && toe_is_error(CHASSIS_MOTOR3_TOE) && toe_is_error(CHASSIS_MOTOR4_TOE)))
        {
            CAN_cmd_chassis(chassis_move.motor_give_current[0],
                            chassis_move.motor_give_current[1],
                            chassis_move.motor_give_current[2],
                            chassis_move.motor_give_current[3]);
        }

        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
  * @brief          "chassis_move" valiable initialization, include pid initialization, remote control data point initialization, 3508 chassis motors
  *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
  * @param[out]     chassis_move_init: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief 初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param [out]     chassis_move_init: "chassis_move"变量指针.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
        return;

    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};

    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();

    for (uint8_t i = 0; i < 4; i++)
    {
        chassis_motor_measure_ptrs[i] = get_chassis_motor_measure_point(i);
        PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }

    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_feedback_update(chassis_move_init);
}

/**
  * @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
  * @param[out]     chassis_move_mode: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief 初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param [out]     chassis_move_mode: "chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
    //in file "chassis_behaviour.c"
    chassis_behaviour_mode_set(chassis_move_mode);
}

/**
  * @brief          when chassis mode change, some param should be changed, suan as chassis yaw_set should be now chassis yaw
  * @param[out]     chassis_move_transit: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          底盘模式改变，有些参数需要改变，例如底盘控制yaw角度设定值应该变成当前底盘yaw角度
  * @param[out]     chassis_move_transit:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL)
        return;
    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

/**
  * @brief 初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief 初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param [out]     chassis_move_update: "chassis_move"变量指针.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
        return;

    for (uint8_t i = 0; i < 4; i++)
    {
        chassis_move_update->motor_speed[i] = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_motor_measure_ptrs[i]->speed_rpm;
        chassis_move_update->motor_accel[i] = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
    }

    /* Skid-steer: left = motors 0,1  right = motors 2,3 */
    chassis_move_update->vx = (chassis_move_update->motor_speed[0]
                             + chassis_move_update->motor_speed[1]
                             + chassis_move_update->motor_speed[2]
                             + chassis_move_update->motor_speed[3]) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->wz = (-chassis_move_update->motor_speed[0]
                              - chassis_move_update->motor_speed[1]
                              + chassis_move_update->motor_speed[2]
                              + chassis_move_update->motor_speed[3]) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ;

    chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET));
    chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET));
    chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);
}
/**
  * @brief          accroding to the channel value of remote control, calculate chassis vertical and horizontal speed set-point
  *                 
  * @param[out]     vx_set: vertical speed set-point
  * @param[out]     vy_set: horizontal speed set-point
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief 初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  *                 
  * @param [out]     vx_set:  纵向速度指针
  * @param [out]     vy_set:  横向速度指针
  * @param [out]     chassis_move_rc_to_vector: "chassis_move"变量指针.
  * @retval         none
  */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
        return;

    *vx_set = chassis_move_rc_to_vector->vx_set;
    *vy_set = 0.0f;   /* rubber tyres — no lateral motion */
}
/**
  * @brief          set chassis control set-point, three movement control value is set by "chassis_behaviour_control_set".
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief 初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param [out]     chassis_move_update: "chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
    if (chassis_move_control == NULL)
        return;

    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);

    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        chassis_move_control->wz_set = angle_set;
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
    }
    else /* CHASSIS_VECTOR_RAW */
    {
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->wz_set = angle_set;
    }
}

/**
 * Skid-steer speed decomposition (4 rubber-tyre wheels).
 * Left side  = motors 0,1  — same speed.
 * Right side = motors 2,3  — same speed.
 */
static void chassis_vector_to_wheel_speed(const fp32 vx_set, const fp32 wz_set, fp32 wheel_speed[4])
{
    const fp32 v_left  = vx_set - wz_set * MOTOR_DISTANCE_TO_CENTER;
    const fp32 v_right = vx_set + wz_set * MOTOR_DISTANCE_TO_CENTER;

    /* 左半边: motors 0,1 — 正 RPM = 前进 */
    wheel_speed[0] =  v_left;
    wheel_speed[1] =  v_left;
    /* 右半边: motors 2,3 — 反装，负 RPM = 前进 */
    wheel_speed[2] = -v_right;
    wheel_speed[3] = -v_right;
}


/**
  * @brief          control loop, according to control set-point, calculate motor current, 
  *                 motor current will be sentto motor
  * @param[out]     chassis_move_control_loop: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 wheel_speed[4] = {0};
    chassis_vector_to_wheel_speed(chassis_move_control_loop->vx_set,
                                  chassis_move_control_loop->wz_set, wheel_speed);

    if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        for (uint8_t i = 0; i < 4; i++)
            chassis_move_control_loop->motor_give_current[i] = (int16_t)wheel_speed[i];
        return;
    }

    /* RPM 直通: 4 轮独立转速 (旧参数序 m1..m4), 经速度 PID 转电流。
     * 旧序: m1=左前, m2=右前, m3=左后, m4=右后; 右轮反装 (负=前进)。
     * 新序: motor0=左前, motor1=左后, motor2=右前, motor3=右后。 */
    if (chassis_move_control_loop->rpm_mode)
    {
        const fp32 k = M2006_MOTOR_RPM_TO_VECTOR;
        const int16_t *r = chassis_move_control_loop->motor_rpm_set;
        chassis_move_control_loop->motor_speed_set[0] =  (fp32)r[0] * k;
        chassis_move_control_loop->motor_speed_set[1] =  (fp32)r[2] * k;
        chassis_move_control_loop->motor_speed_set[2] = -(fp32)r[1] * k;
        chassis_move_control_loop->motor_speed_set[3] = -(fp32)r[3] * k;

        for (uint8_t i = 0; i < 4; i++)
        {
            PID_calc(&chassis_move_control_loop->motor_speed_pid[i],
                     chassis_move_control_loop->motor_speed[i],
                     chassis_move_control_loop->motor_speed_set[i]);
            chassis_move_control_loop->motor_give_current[i] =
                (int16_t)chassis_move_control_loop->motor_speed_pid[i].out;
        }
        return;
    }

    /* Speed limit: scale all wheels if any exceeds MAX_WHEEL_SPEED */
    fp32 max_vector = 0.0f;
    for (uint8_t i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_speed_set[i] = wheel_speed[i];
        fp32 t = fabs(wheel_speed[i]);
        if (max_vector < t) max_vector = t;
    }

    if (max_vector > MAX_WHEEL_SPEED)
    {
        fp32 rate = MAX_WHEEL_SPEED / max_vector;
        for (uint8_t i = 0; i < 4; i++)
            chassis_move_control_loop->motor_speed_set[i] *= rate;
    }

    /* Motor speed PID → current */
    for (uint8_t i = 0; i < 4; i++)
    {
        PID_calc(&chassis_move_control_loop->motor_speed_pid[i],
                 chassis_move_control_loop->motor_speed[i],
                 chassis_move_control_loop->motor_speed_set[i]);
        chassis_move_control_loop->motor_give_current[i] =
            (int16_t)chassis_move_control_loop->motor_speed_pid[i].out;
    }
}

/* ---- Public API ---- */

void chassis_set_velocity(fp32 vx, fp32 wz)
{
    chassis_move.rpm_mode = 0;   /* 退出 RPM 直通, 回到速度模式 */
    chassis_move.vx_set = vx;
    chassis_move.wz_set = wz;
}

void chassis_set_motor_rpm(const int16_t rpm[4])
{
    for (uint8_t i = 0; i < 4; i++)
        chassis_move.motor_rpm_set[i] = rpm[i];
    chassis_move.rpm_mode = 1;
    chassis_move.chassis_mode = CHASSIS_VECTOR_RPM;
}

void chassis_get_velocity(fp32 *vx, fp32 *wz)
{
    if (vx) *vx = chassis_move.vx;
    if (wz) *wz = chassis_move.wz;
}
