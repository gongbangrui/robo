/**
 ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_behaviour.c/h
  * @brief      chassis behaviour mode control (no remote — fixed autonomous mode).
  *             底盘行为模式控制 (无遥控 — 固定自主模式)
  * @note
  * @history
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *  V2.0.0     Apr-27-2026     smart_car       stripped RC, fixed autonomous mode
  *
  @verbatim
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "chassis_behaviour.h"
#include <stddef.h>

chassis_behaviour_e chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;


void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
        return;

    /* RPM 直通模式由 run_task 独占, 不被 behaviour 覆盖 */
    if (chassis_move_mode->rpm_mode) {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RPM;
        return;
    }

    chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
    chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
}


void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set,
                                   chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL ||
        chassis_move_rc_to_vector == NULL)
        return;

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
    *angle_set = chassis_move_rc_to_vector->wz_set;
}
