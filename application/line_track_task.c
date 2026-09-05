/**
 * line_track_task.c
 * 循线状态机核心（5ms 控制周期）
 *
 * 状态机流转 (5 状态):
 *   LOST_LINE → TRACKING → APPROACH → CROSS → TRACKING
 *   任意状态 ──[STOP]──→ STOP
 *
 * 输入:  gray_sensor.position (-1.0 左 … +1.0 右), online, line_count
 * 输入:  navigate_action (由 navigate_task 设定)
 * 输出: chassis_cmd { vx, vy, wz } 由 chassis_task 消费
 *
 * CROSS 退出策略:
 *   转弯 (yaw_threshold > 0): 仅靠陀螺仪 yaw 角，不看传感器线
 *   直行 (yaw_threshold == 0): 靠传感器线重获
 */
#include "line_track_task.h"
#include "INS_task.h"
#include "actions.h"
#include "chassis_task.h"
#include "cmsis_os.h"
#include "cross_exec.h"
#include "debug_console.h"
#include "gray_sensor.h"
#include "ir_detect_exec.h"
#include "ir_sensor.h"
#include "lt_utils.h"
#include "pid.h"

#include <math.h>
#include <string.h>

/* ---- 内部状态 ---- */

static lt_state_t state = LT_STATE_LOST_LINE;
static navigate_action_t pending_action = NONE;
static uint8_t pending_action_param = 0;
static fp32 pending_dvx = 0;   /* 当前步相对 base_speed 的速度偏移 */
static uint8_t exit_event = 0;

static pid_type_def line_pid;

static uint32_t state_entry_ticks = 0;
static fp32 deadreckon_vx = 0;
static fp32 deadreckon_wz = 0;
static fp32 cross_yaw_start = 0;
static fp32 deadreckon_yaw_threshold = 0;
static uint8_t approach_confirm_cnt = 0;
static uint8_t edge_confirm_cnt = 0;
static uint8_t just_crossed = 0;
static uint8_t climb_lost_cnt = 0;  /* CLIMB: 连续丢线帧数 (消抖) */
static uint8_t ir_hold_cnt = 0;     /* IR: 前红外连续触发帧数 (消抖) */

/* ---- FSM trace 调试 ---- */

static uint8_t trace_enabled = 0;
static uint32_t trace_start_ms = 0;

static const char *state_names[] = {
    "LOST_LINE", "TRACKING", "APPROACH", "CROSS", "STOP",
};

void line_track_set_trace(uint8_t enable) {
  trace_enabled = enable;
  trace_start_ms = osKernelSysTick();
  debug_console_printf("[TRACE] FSM trace %s\r\n", enable ? "ON" : "OFF");
}

uint8_t line_track_get_trace(void) { return trace_enabled; }

/* ---- 辅助函数 ---- */

static void set_state(lt_state_t new_state, const char *reason) {
  if (trace_enabled) {
    const gray_sensor_t *gs = get_gray_sensor_point();
    uint32_t elapsed = osKernelSysTick() - trace_start_ms;
    debug_console_printf("[TRACE] %6lu ms  %-10s -> %-10s  reason=%-20s "
                         "pos=%+.2f bm=0x%04X lc=%d\r\n",
                         elapsed, state_names[state], state_names[new_state],
                         reason, gs ? gs->position : 0.0f, gs ? gs->bitmask : 0,
                         gs ? gs->line_count : 0);
  }
  state = new_state;
  state_entry_ticks = osKernelSysTick();
}

static uint32_t time_in_state(void) {
  return osKernelSysTick() - state_entry_ticks;
}

const char *line_track_state_name(void) { return state_names[state]; }
fp32 line_track_get_position(void) {
  const gray_sensor_t *gs = get_gray_sensor_point();
  return gs ? gs->position : 0.0f;
}

/* 路口分析状态（仅 APPROACH 使用） */
static bitmask_analysis_t intersection_analysis;
static uint8_t expected_branch;

static const char *classify_intersection(const bitmask_analysis_t *a,
                                         navigate_action_t action,
                                         uint8_t *exp_branch) {
  action_dir_t dir = get_action_direction(action);
  if (dir != DIR_NONE) {
    *exp_branch = dir;
  } else if (action == AUTO) {
    if (a->left_branch && !a->right_branch)
      *exp_branch = DIR_LEFT;
    else if (a->right_branch && !a->left_branch)
      *exp_branch = DIR_RIGHT;
    else
      *exp_branch = DIR_NONE;
  } else {
    *exp_branch = DIR_NONE;
  }
  if (a->is_wide_blob)
    return "BLOB";
  if (a->left_branch && a->right_branch)
    return "CROSS";
  if (a->left_branch)
    return "T_LEFT";
  if (a->right_branch)
    return "T_RIGHT";
  if (a->cluster_count >= 2)
    return "MULTI";
  return "SINGLE";
}

/* ---- 运行时可调参数 ---- */

static fp32 current_base_speed = LT_BASE_SPEED;
static fp32 current_slow_speed = LT_SLOW_SPEED;

/* ---- 死推算参数表 ---- */

typedef struct {
  fp32 vx, wz, yaw_th;
} dr_param_t;

#define D(deg) DEG2RAD(deg)
#define VX_SLOW (-1.0f) /* 哨兵: current_slow_speed */
#define VX_BASE (-2.0f) /* 哨兵: current_base_speed */
#define CLIMB_LOST_FRAMES 10 /* CLIMB: 连续丢线帧数才触发死推 (防起步抖动误触发) */
#define IR_HOLD_FRAMES 5      /* IR: 前红外连续触发帧数 (~25ms) */

static const dr_param_t DR_TABLE[] = {
    [FORWARD] = {VX_BASE},
    [BACKWARD] = {VX_SLOW},
    [STOP] = {0.0f},
    [LEFT_10] = {VX_SLOW, +2.0f, D(5)},
    [LEFT_20] = {VX_SLOW, +2.0f, D(15)},
    [LEFT_30] = {VX_SLOW, +2.0f, D(25)},
    [LEFT_40] = {VX_SLOW, +2.0f, D(35)},
    [LEFT_50] = {VX_SLOW, +2.0f, D(45)},
    [LEFT_60] = {VX_SLOW, +2.0f, D(55)},
    [LEFT_70] = {VX_SLOW, +2.0f, D(65)},
    [LEFT_80] = {VX_SLOW, +2.0f, D(75)},
    [LEFT_90] = {VX_SLOW, +2.0f, D(85)},
    [LEFT_100] = {VX_SLOW, +2.0f, D(95)},
    [LEFT_110] = {VX_SLOW, +2.0f, D(105)},
    [LEFT_120] = {VX_SLOW, +2.0f, D(115)},
    [LEFT_130] = {VX_SLOW, +2.0f, D(125)},
    [LEFT_140] = {VX_SLOW, +2.0f, D(135)},
    [LEFT_150] = {VX_SLOW, +2.0f, D(145)},
    [LEFT_160] = {VX_SLOW, +2.0f, D(155)},
    [LEFT_170] = {VX_SLOW, +2.0f, D(165)},
    [LEFT_180] = {VX_SLOW, +2.0f, D(175)},
    [RIGHT_10] = {VX_SLOW, -2.0f, D(5)},
    [RIGHT_20] = {VX_SLOW, -2.0f, D(15)},
    [RIGHT_30] = {VX_SLOW, -2.0f, D(25)},
    [RIGHT_40] = {VX_SLOW, -2.0f, D(35)},
    [RIGHT_50] = {VX_SLOW, -2.0f, D(45)},
    [RIGHT_60] = {VX_SLOW, -2.0f, D(55)},
    [RIGHT_70] = {VX_SLOW, -2.0f, D(65)},
    [RIGHT_80] = {VX_SLOW, -2.0f, D(75)},
    [RIGHT_90] = {VX_SLOW, -2.0f, D(85)},
    [RIGHT_100] = {VX_SLOW, -2.0f, D(95)},
    [RIGHT_110] = {VX_SLOW, -2.0f, D(105)},
    [RIGHT_120] = {VX_SLOW, -2.0f, D(115)},
    [RIGHT_130] = {VX_SLOW, -2.0f, D(125)},
    [RIGHT_140] = {VX_SLOW, -2.0f, D(135)},
    [RIGHT_150] = {VX_SLOW, -2.0f, D(145)},
    [RIGHT_160] = {VX_SLOW, -2.0f, D(155)},
    [RIGHT_170] = {VX_SLOW, -2.0f, D(165)},
    [RIGHT_180] = {VX_SLOW, -2.0f, D(175)},
    [SLOW] = {0.10f},
    [FAST] = {0.80f},
    [AUTO] = {VX_SLOW},
    [BRIDGE] = {VX_SLOW, 0.0f, 0.0f},
    [MOVE] = {0.15f, 0.0f, 0.0f},
    [TRAFFIC] = {VX_SLOW, 0.0f, 0.0f},
    [QRSCAN] = {VX_SLOW, 0.0f, 0.0f},
    [DIGIT] = {VX_SLOW, 0.0f, 0.0f},
    [RUN_PATH] = {VX_SLOW, 0.0f, 0.0f},
    [NONE] = {VX_SLOW},
    [LINE_END] = {VX_BASE},
};

void get_deadreckon_params(navigate_action_t a, fp32 *vx, fp32 *wz,
                           fp32 *yaw_th) {
  unsigned i = (unsigned)a;
  if (i >= sizeof(DR_TABLE) / sizeof(DR_TABLE[0]))
    i = NONE;
  dr_param_t p = DR_TABLE[i];
  *vx = (p.vx == VX_SLOW)   ? current_slow_speed
        : (p.vx == VX_BASE) ? current_base_speed
                            : p.vx;
  *wz = p.wz;
  *yaw_th = p.yaw_th;

  /* 仅直行类 (CAT_FWD) 应用当前步的相对速度偏移 */
  if (navigate_action_category(a) == CAT_FWD) {
    *vx += pending_dvx;
    if (*vx < 0.05f) *vx = 0.05f;
  }
}

static void trace_intersection_info(const char *itype) {
  if (trace_enabled) {
    debug_console_printf("[TRACE]   itype=%s exp_b=%d act=%d "
                         "dr_vx=%.2f dr_wz=%.2f yaw_th=%.0f\r\n",
                         itype, expected_branch, pending_action, deadreckon_vx,
                         deadreckon_wz, RAD2DEG(deadreckon_yaw_threshold));
  }
}

void line_track_set_nav_action(navigate_action_t action) {
  pending_action = action;
  pending_action_param = 0;
  pending_dvx = 0;
}

void line_track_set_nav_step(navigate_step_t step) {
  pending_action = step.action;
  pending_action_param = step.param;
  pending_dvx = step.dvx;
}

/* 当前步相对 base_speed 的速度偏移 (供 executor 查询) */
fp32 line_track_get_pending_dvx(void) { return pending_dvx; }

uint8_t line_track_got_exit_event(void) {
  uint8_t v = exit_event;
  exit_event = 0;
  return v;
}

/* ================================================================
 * 核心状态机 (5 状态)
 * ================================================================ */

void line_track_task(void const *pvParameters) {
  (void)pvParameters;
  const gray_sensor_t *gs;

  const fp32 pid_params[3] = {LT_KP, LT_KI, LT_KD};
  PID_init(&line_pid, PID_POSITION, pid_params, LT_MAX_WZ, 0.3f);

  /* 初始化路口执行器分发系统 */
  cross_exec_init();

  osDelay(200);

  fp32 cmd_vx = 0, cmd_wz = 0;

  for (;;) {
    if (actions_is_active()) {
      osDelay(LT_CONTROL_TIME_MS);
      continue;
    }
    if (!gray_sensor_poll()) {
      chassis_set_velocity(0, 0);
      osDelay(LT_CONTROL_TIME_MS);
      continue;
    }

    /* 红外传感器轮询（纳秒级，不阻塞） */
    ir_sensor_poll();

    gs = get_gray_sensor_point();

    if (pending_action == STOP && state != LT_STATE_STOP)
      set_state(LT_STATE_STOP, "immediate_stop");

    fp32 pos = gs->position;

    switch (state) {

    case LT_STATE_LOST_LINE:
      cmd_vx = 0;
      cmd_wz = 0;
      if (is_any_line(gs)) {
        set_state(LT_STATE_TRACKING, "line_detected");
      } else if (pending_action == BRIDGE) {
        cmd_vx = current_slow_speed;
        fp32 p = get_INS_angle_point()[INS_PITCH_ADDRESS_OFFSET];
        if (p > DEG2RAD(15.0f)) {
          set_state(LT_STATE_INTERSECTION_CROSS, "bridge_pitch");
        } else {
          bitmask_analysis_t a;
          analyze_bitmask(gs->bitmask, &a);
          if (approaching_intersection(&a))
            set_state(LT_STATE_INTERSECTION_APPROACH, "bridge_ramp");
        }
      } else if (pending_action == PLATFORM) {
        cmd_vx = current_slow_speed;
        set_state(LT_STATE_INTERSECTION_CROSS, "platform_line_lost");
      } else if (pending_action == LINE_END) {
        exit_event = 1;
      }
      break;

    case LT_STATE_TRACKING:
      cmd_vx = current_base_speed + pending_dvx;
      if (cmd_vx < 0.05f) cmd_vx = 0.05f;
      PID_calc(&line_pid, -pos, 0.0f);
      cmd_wz = line_pid.out;

      if (just_crossed) {
        if (line_is_centered(gs) || time_in_state() > 500) {
          just_crossed = 0;
        }
      } else {
        if (is_any_line(gs)) climb_lost_cnt = 0;
        bitmask_analysis_t a;
        analyze_bitmask(gs->bitmask, &a);

        /* IR: 循线中前红外触发 → 进 CROSS 停下 (电平配置在 ir_detect_exec) */
        if (pending_action == IR) {
          if (ir_detect_triggered()) {
            if (++ir_hold_cnt >= IR_HOLD_FRAMES) {
              ir_hold_cnt = 0;
              set_state(LT_STATE_INTERSECTION_CROSS, "ir_triggered");
            }
          } else {
            ir_hold_cnt = 0;
          }
        } else if (is_turn_action(pending_action)) {
          /* 转弯: 循线保持到触发瞬间, 不进 APPROACH (复刻旧 trackxian5 结构).
           * 干扰线不满足触发条件就继续循线 → 到达路口的位置/姿态一致 */
          if (turn_trigger_reached(gs, pending_action))
            edge_confirm_cnt++;
          else
            edge_confirm_cnt = 0;
          if (edge_confirm_cnt >= 2) {
            edge_confirm_cnt = 0;
            const char *itype =
                classify_intersection(&a, pending_action, &expected_branch);
            get_deadreckon_params(pending_action, &deadreckon_vx,
                                  &deadreckon_wz, &deadreckon_yaw_threshold);
            trace_intersection_info(itype);
            cross_yaw_start = get_INS_angle_point()[INS_YAW_ADDRESS_OFFSET];
            set_state(LT_STATE_INTERSECTION_CROSS, "turn_trigger");
          }
        } else if (approaching_intersection(&a)) {
          climb_lost_cnt = 0;
          set_state(LT_STATE_INTERSECTION_APPROACH, "intersection_detected");
        } else if (!is_any_line(gs) && time_in_state() > 100) {
          edge_confirm_cnt = 0;
          if (pending_action == BRIDGE) {
            cmd_vx = current_slow_speed;
            cmd_wz = 0.0f;
            fp32 p = get_INS_angle_point()[INS_PITCH_ADDRESS_OFFSET];
            if (p > DEG2RAD(15.0f))
              set_state(LT_STATE_INTERSECTION_CROSS, "bridge_pitch");
          } else if (pending_action == PLATFORM) {
            cmd_vx = current_slow_speed;
            cmd_wz = 0.0f;
            set_state(LT_STATE_INTERSECTION_CROSS, "platform_line_lost");
          } else if (pending_action == CLIMB) {
            /* 连续丢线消抖: 防起步抖动/传感器误报, 真丢线才死推越障 */
            if (++climb_lost_cnt >= CLIMB_LOST_FRAMES) {
              climb_lost_cnt = 0;
              cmd_vx = current_slow_speed;
              cmd_wz = 0.0f;
              set_state(LT_STATE_INTERSECTION_CROSS, "climb_line_lost");
            }
          } else if (pending_action == LINE_END) {
            exit_event = 1;
          } else {
            set_state(LT_STATE_LOST_LINE, "line_lost");
          }
        }
      }
      break;

    case LT_STATE_INTERSECTION_APPROACH:
      /* 进路口关闭循线PID，直行不降速，转弯降速 */
      cmd_wz = 0.0f;
      if (navigate_action_category(pending_action) == CAT_FWD) {
        cmd_vx = current_base_speed + pending_dvx;
        if (cmd_vx < 0.05f) cmd_vx = 0.05f;
      } else {
        cmd_vx = current_slow_speed;
      }
      {
        bitmask_analysis_t a;
        analyze_bitmask(gs->bitmask, &a);

        if (approaching_intersection(&a)) {
          approach_confirm_cnt++;
          if (approach_confirm_cnt == 1) {
            intersection_analysis = a;
            classify_intersection(&a, pending_action, &expected_branch);
            get_deadreckon_params(pending_action, &deadreckon_vx,
                                  &deadreckon_wz, &deadreckon_yaw_threshold);
          }
        } else {
          approach_confirm_cnt = 0;
        }

        if (is_all_black(gs)) {
          intersection_analysis = a;
          const char *itype =
              classify_intersection(&a, pending_action, &expected_branch);
          get_deadreckon_params(pending_action, &deadreckon_vx, &deadreckon_wz,
                                &deadreckon_yaw_threshold);
          trace_intersection_info(itype);
          cross_yaw_start = get_INS_angle_point()[INS_YAW_ADDRESS_OFFSET];
          approach_confirm_cnt = 0;
          edge_confirm_cnt = 0;
          set_state(LT_STATE_INTERSECTION_CROSS, "all_black");
        } else if (approach_confirm_cnt >= 6) {
          /* 直行: 6帧消抖 → CROSS */
          trace_intersection_info(classify_intersection(
              &intersection_analysis, pending_action, &expected_branch));
          cross_yaw_start = get_INS_angle_point()[INS_YAW_ADDRESS_OFFSET];
          approach_confirm_cnt = 0;
          set_state(LT_STATE_INTERSECTION_CROSS, "turn_confirmed");
        } else if (!approaching_intersection(&a) && line_is_centered(gs)) {
          approach_confirm_cnt = 0;
          edge_confirm_cnt = 0;
          set_state(LT_STATE_TRACKING, "false_alarm");
        }
      }
      break;

    case LT_STATE_INTERSECTION_CROSS: {
      const cross_executor_t *exe = cross_exec_lookup(pending_action);
      if (!exe) {
        /* 无匹配 executor — 回退到直行 */
        exe = cross_exec_lookup(FORWARD);
      }
      if (!g_cross_ctx.entered) {
        g_cross_ctx.entered = 1;
        g_cross_ctx.action_param = pending_action_param;
        exe->enter(&g_cross_ctx, pending_action, gs, &cmd_vx, &cmd_wz);
      }

      cross_exec_result_t res = exe->exec(&g_cross_ctx, &cmd_vx, &cmd_wz);

      if (res == CROSS_EXEC_DONE) {
        exe->exit(&g_cross_ctx);
        g_cross_ctx.entered = 0;
        if (exe->signal_navigator) {
          just_crossed = 1;
        }
        /* 先置 exit_event 再唤醒 cross(): 保证 actions 流可立即取走事件,
         * 防止 navigate_task 用残留路径推进 (第二次动作被吞 → 锁死) */
        exit_event = 1;
        actions_signal_cross_done();
        set_state(LT_STATE_TRACKING, "cross_done");
      } else if (res == CROSS_EXEC_ABORT) {
        exe->exit(&g_cross_ctx);
        g_cross_ctx.entered = 0;
        actions_signal_cross_done();
        set_state(LT_STATE_LOST_LINE, "cross_abort");
      }
      /* CROSS_EXEC_RUNNING: executor 已设置 cmd_vx/cmd_wz */
    } break;

    case LT_STATE_STOP:
      cmd_vx = 0;
      cmd_wz = 0;
      if (pending_action != STOP &&
          pending_action != NONE)
        set_state(LT_STATE_LOST_LINE, "restart");
      break;
    }

    /* 低通滤波平滑速度指令 */
    chassis_set_velocity(cmd_vx, cmd_wz);
    osDelay(LT_CONTROL_TIME_MS);
  }
}

/* ---- 公共 API ---- */
navigate_action_t line_track_get_pending_action(void) { return pending_action; }

static fp32 current_kp = LT_KP, current_ki = LT_KI, current_kd = LT_KD;

void line_track_get_pid(fp32 *kp, fp32 *ki, fp32 *kd) {
  if (kp)
    *kp = current_kp;
  if (ki)
    *ki = current_ki;
  if (kd)
    *kd = current_kd;
}

void line_track_set_pid(fp32 kp, fp32 ki, fp32 kd) {
  current_kp = kp;
  current_ki = ki;
  current_kd = kd;
  const fp32 pid_params[3] = {kp, ki, kd};
  PID_init(&line_pid, PID_POSITION, pid_params, LT_MAX_WZ, 0.3f);
}

fp32 line_track_get_base_speed(void) { return current_base_speed; }
fp32 line_track_get_slow_speed(void) { return current_slow_speed; }

void line_track_set_base_speed(fp32 speed) {
  if (speed > 0.0f && speed <= 1.5f)
    current_base_speed = speed;
}

void line_track_set_slow_speed(fp32 speed) {
  if (speed > 0.0f && speed <= 1.5f)
    current_slow_speed = speed;
}
