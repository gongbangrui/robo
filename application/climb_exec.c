/**
 * climb_exec.c
 * 爬坡越障执行器 — 循线爬"山"，山顶丢线后死推固定距离越过。
 *
 * 流程: TRACKING 循线 → 丢线 → CROSS → 死推 CLIMB_DIST_CM 越顶 → 续循。
 * 无需陀螺仪 (纯距离积分)。见线需持续消抖才提前结束, 忽略山顶残留短线段。
 *
 * 路径用法: {CLIMB}
 */
#include "chassis_task.h"
#include "cmsis_os.h"
#include "cross_exec.h"
#include "gray_sensor.h"
#include "lt_utils.h"

#define CLIMB_DIST_CM 80.0f    /* 死推距离 (cm) */
#define CLIMB_SPEED 0.8f       /* 死推速度 (m/s) */
#define CLIMB_TIMEOUT_MS 10000 /* 兜底超时 */
#define CLIMB_LINE_HOLD_FRAMES 10 /* 见线消抖帧数 (~50ms), 忽略山顶残留短线段 */

typedef struct {
  uint32_t enter_ms;
  fp32 target_m;
  fp32 traveled_m;
  uint8_t line_hold_cnt; /* 连续见线帧数 (消抖) */
} climb_priv_t;

static climb_priv_t climb_state;

static void climb_enter(cross_exec_ctx_t *ctx, navigate_action_t action,
                        const gray_sensor_t *gs, fp32 *vx, fp32 *wz) {
    (void)action;
    climb_state.enter_ms      = osKernelSysTick();
    climb_state.target_m      = CLIMB_DIST_CM * 0.01f;
    climb_state.traveled_m    = 0.0f;
    climb_state.line_hold_cnt = 0;

    *vx = CLIMB_SPEED;
    *wz = 0.0f;
    ctx->priv = &climb_state;
}

static cross_exec_result_t climb_exec_fn(cross_exec_ctx_t *ctx, fp32 *vx,
                                         fp32 *wz) {
    (void)ctx;
    uint32_t now = osKernelSysTick();

    *vx = CLIMB_SPEED;
    *wz = 0.0f;

    /* 死推: 重新见线需持续消抖才结束, 忽略山顶残留短线段 */
    if (is_any_line(get_gray_sensor_point())) {
        if (++climb_state.line_hold_cnt >= CLIMB_LINE_HOLD_FRAMES) {
            *vx = 0.0f;
            return CROSS_EXEC_DONE;
        }
    } else {
        climb_state.line_hold_cnt = 0;
    }

    /* 死推距离积分 */
    climb_state.traveled_m += CLIMB_SPEED * 0.005f;  /* 5ms 周期 */
    if (climb_state.traveled_m >= climb_state.target_m) {
        *vx = 0.0f;
        return CROSS_EXEC_DONE;
    }
    /* 兜底超时 */
    if (now - climb_state.enter_ms > CLIMB_TIMEOUT_MS) {
        *vx = 0.0f;
        return CROSS_EXEC_DONE;
    }
    return CROSS_EXEC_RUNNING;
}

static void climb_exit(cross_exec_ctx_t *ctx) { (void)ctx; }

const cross_executor_t climb_exec = {
    .name = "climb",
    .enter = climb_enter,
    .exec = climb_exec_fn,
    .exit = climb_exit,
    .signal_navigator = 1,
};
