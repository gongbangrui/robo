/**
 * bridge_exec.c
 * 过桥执行器 — 全程 IR 纠偏 + 符号 pitch 判据 (不依赖 entry_pitch)。
 *
 * pitch 轨迹: 地面~0 → 上坡 +20° → 桥面 ~0 → 下坡 -20° → 地面 ~0
 *
 * 状态机:
 *   CROSSING  → 检测上坡→桥面→下坡→地面, 全程 IR
 *   RECOVER   → 地面持续确认 (或见线兜底) → DONE
 */
#include "INS_task.h"
#include "cmsis_os.h"
#include "cross_exec.h"
#include "gray_sensor.h"
#include "ir_sensor.h"
#include "lt_utils.h"
#include "voice_module.h"

/* ---- 可调参数 ---- */
#define BRIDGE_BASE_SPEED 0.5f          /* 过桥基础速度 (m/s) */
#define BRIDGE_STEER_GAIN 0.8f          /* 红外纠偏增益 (rad/s) */
#define BRIDGE_PITCH_CLIMB_DEG 15.0f    /* pitch 超此值 = 上坡 (误检阈值) */
#define BRIDGE_PITCH_DESCEND_DEG -15.0f /* pitch 低于此 = 下坡 */
#define BRIDGE_PITCH_LEVEL_DEG 3.0f     /* |pitch| 此内 = 水平 */
#define BRIDGE_LINE_LEVEL_DEG 8.0f      /* +-此内 + 见线 = 兜底退出 */
#define BRIDGE_LEVEL_HOLD_MS 500        /* 水平持续确认时间 (ms) */

/* ---- 阶段枚举 ---- */
typedef enum {
  BRIDGE_STAGE_CROSSING = 0, /* 上坡→桥面→下坡, 全程 IR */
  BRIDGE_STAGE_RECOVER = 1,  /* 地面确认 (或见线) */
  BRIDGE_STAGE_DONE = 2,     /* 完成 */
} bridge_stage_t;

/* ---- 私有状态 ---- */
typedef struct {
  bridge_stage_t stage;
  uint32_t level_start_ms;
  uint32_t entry_ms;
  uint8_t seen_climb;   /* pitch 曾 > +CLIMB_DEG (已上坡) */
  uint8_t seen_bridge;  /* 上坡后 pitch 回到水平 (桥面) */
  uint8_t seen_descend; /* 桥面后 pitch < DESCEND_DEG (已下坡) */
} bridge_priv_t;

static bridge_priv_t bridge_state;

/* ---- IR 纠偏: 左侧见红→右修, 右侧见红→左修 ---- */
static fp32 bridge_ir_steer(const ir_sensor_t *ir) {
  fp32 steer = 0.0f;
  if (!ir->left)
    steer -= BRIDGE_STEER_GAIN;
  if (!ir->right)
    steer += BRIDGE_STEER_GAIN;
  return steer;
}

/* ---- executor 接口 ---- */

static void bridge_enter(cross_exec_ctx_t *ctx, navigate_action_t action,
                         const gray_sensor_t *gs, fp32 *vx, fp32 *wz) {
  (void)action;
  (void)gs;

  bridge_state.stage = BRIDGE_STAGE_CROSSING;
  bridge_state.level_start_ms = 0;
  bridge_state.entry_ms = osKernelSysTick();
  bridge_state.seen_climb = 0;
  bridge_state.seen_bridge = 0;
  bridge_state.seen_descend = 0;

  voice_module_play(VOICE_TRACK_BRIDGE_ENTER);

  *vx = BRIDGE_BASE_SPEED;
  *wz = bridge_ir_steer(get_ir_sensor_point());

  ctx->priv = &bridge_state;
}

static cross_exec_result_t bridge_exec_fn(cross_exec_ctx_t *ctx, fp32 *vx,
                                          fp32 *wz) {
  (void)ctx;
  const ir_sensor_t *ir = get_ir_sensor_point();
  const gray_sensor_t *gs = get_gray_sensor_point();
  uint32_t now = osKernelSysTick();
  fp32 pitch = get_INS_angle_point()[INS_PITCH_ADDRESS_OFFSET];

  /* IR 纠偏在上坡和桥面启用, 下坡 (pitch < -15°) 直行 */
  *vx = BRIDGE_BASE_SPEED;
  if (pitch > DEG2RAD(-15.0f))
    *wz = bridge_ir_steer(ir);
  else
    *wz = 0.0f;

  /* 下过坡后见到线 → 立即退出 */
  if (bridge_state.seen_descend && is_any_line(gs))
    bridge_state.stage = BRIDGE_STAGE_DONE;

  switch (bridge_state.stage) {

  case BRIDGE_STAGE_CROSSING:
    if (pitch > DEG2RAD(BRIDGE_PITCH_CLIMB_DEG))
      bridge_state.seen_climb = 1;
    /* 桥面检测: 上坡后 pitch 回到水平 */
    if (bridge_state.seen_climb && pitch < DEG2RAD(BRIDGE_PITCH_LEVEL_DEG) &&
        pitch > -DEG2RAD(BRIDGE_PITCH_LEVEL_DEG)) {
      bridge_state.seen_bridge = 1;
    }
    /* 下坡检测: 桥面后 pitch 低于负阈值 */
    if (bridge_state.seen_bridge && pitch < DEG2RAD(BRIDGE_PITCH_DESCEND_DEG)) {
      bridge_state.seen_descend = 1;
    }
    /* 地面检测: 下坡后 pitch 回到水平 → RECOVER */
    if (bridge_state.seen_descend && pitch < DEG2RAD(BRIDGE_PITCH_LEVEL_DEG) &&
        pitch > -DEG2RAD(BRIDGE_PITCH_LEVEL_DEG)) {
      bridge_state.stage = BRIDGE_STAGE_RECOVER;
      bridge_state.level_start_ms = now;
    }
    break;

  case BRIDGE_STAGE_RECOVER:
    if (pitch > DEG2RAD(BRIDGE_PITCH_LEVEL_DEG) ||
        pitch < -DEG2RAD(BRIDGE_PITCH_LEVEL_DEG))
      bridge_state.stage = BRIDGE_STAGE_CROSSING;
    break;

  case BRIDGE_STAGE_DONE:
    *vx = 0.0f;
    *wz = 0.0f;
    voice_module_play(VOICE_TRACK_BRIDGE_DONE);
    return CROSS_EXEC_DONE;
  }

  if (now - bridge_state.entry_ms > 30000) {
    bridge_state.stage = BRIDGE_STAGE_DONE;
  }

  return CROSS_EXEC_RUNNING;
}

static void bridge_exit(cross_exec_ctx_t *ctx) { (void)ctx; }

const cross_executor_t bridge_exec = {
    .name = "bridge",
    .enter = bridge_enter,
    .exec = bridge_exec_fn,
    .exit = bridge_exit,
    .signal_navigator = 1,
};
