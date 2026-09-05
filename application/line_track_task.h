/**
 * line_track_task.h
 * Line-following state machine.
 *
 * State machine overview (5 states):
 *
 *   LOST_LINE → TRACKING → APPROACH → CROSS → TRACKING
 *   任意状态 ──[STOP]──→ STOP
 *
 * CROSS exit:
 *   转弯 (yaw_threshold > 0): gyro yaw → TRACKING
 *   直行 (yaw_threshold == 0): sensor line reacquire → TRACKING
 */
#ifndef LINE_TRACK_TASK_H
#define LINE_TRACK_TASK_H

#include "nav_types.h"
#include "struct_typedef.h"

#define LT_KP 3.0f
#define LT_KI 0.1f
#define LT_KD 0.8f
#define LT_MAX_WZ 4.0f
#define LT_BASE_SPEED 0.8f
#define LT_SLOW_SPEED 0.2f

#define LT_CONTROL_TIME_MS 5
#define LT_CONTROL_TIME 0.005f

typedef enum {
  LT_STATE_LOST_LINE = 0,
  LT_STATE_TRACKING = 1,
  LT_STATE_INTERSECTION_APPROACH = 2,
  LT_STATE_INTERSECTION_CROSS = 3,
  LT_STATE_STOP = 4,
} lt_state_t;

typedef struct {
  fp32 vx_set;
  fp32 vy_set;
  fp32 wz_set;
} chassis_cmd_t;

extern void line_track_task(void const *pvParameters);
extern void line_track_set_nav_action(navigate_action_t action);
extern void line_track_set_nav_step(navigate_step_t step);
extern uint8_t line_track_got_exit_event(void);
extern navigate_action_t line_track_get_pending_action(void);
extern fp32 line_track_get_pending_dvx(void); /* 当前步相对 base_speed 偏移 */
extern const char *line_track_state_name(void);
extern fp32 line_track_get_position(void);
extern void line_track_get_pid(fp32 *kp, fp32 *ki, fp32 *kd);
extern void line_track_set_pid(fp32 kp, fp32 ki, fp32 kd);
extern fp32 line_track_get_base_speed(void);
extern void line_track_set_base_speed(fp32 speed);
extern fp32 line_track_get_slow_speed(void);
extern void line_track_set_slow_speed(fp32 speed);
extern void line_track_set_trace(uint8_t enable);
extern uint8_t line_track_get_trace(void);

/**
 * @brief  计算动作对应死推算参数 (vx 用 speed/slow, wz 和 yaw_th 按度数计算).
 */
extern void get_deadreckon_params(navigate_action_t action, fp32 *vx, fp32 *wz,
                                  fp32 *yaw_th);

#endif
