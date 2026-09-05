/**
 * forward_exec.c
 * 直行执行器 — 过路口时靠传感器线重获判出。
 *
 * 从原 line_track_task.c CROSS 状态中提取，逻辑不变。
 * 适用于所有 yaw_threshold == 0 的直行类动作（FORWARD, FORK_CENTER, AUTO_SELECT 等）。
 */
#include "cross_exec.h"
#include "lt_utils.h"
#include "line_track_task.h"  /* get_deadreckon_params */
#include "INS_task.h"
#include "chassis_task.h"
#include "gray_sensor.h"
#include "cmsis_os.h"

/* ---- 直行私有状态 ---- */
typedef struct {
    fp32     deadreckon_vx;    /* 死推算前向速度 */
    fp32     deadreckon_wz;    /* 死推算角速度 (直行为 0) */
    uint8_t  expected_branch;  /* 期望分支方向: DIR_LEFT / DIR_RIGHT / DIR_NONE */
    uint32_t entry_ms;         /* 进入时间 (ms) */
} forward_priv_t;

static forward_priv_t forward_state;

/* ---- executor 接口 ---- */

static void forward_enter(cross_exec_ctx_t *ctx, navigate_action_t action,
                          const gray_sensor_t *gs, fp32 *vx, fp32 *wz)
{
    (void)gs;

    forward_state.entry_ms = osKernelSysTick();

    /* 从 DR_TABLE 获取死推算参数 */
    extern void get_deadreckon_params(navigate_action_t a,
                                       fp32 *vx_out, fp32 *wz_out, fp32 *yaw_th);
    fp32 yaw_th_unused;
    get_deadreckon_params(action,
                          &forward_state.deadreckon_vx,
                          &forward_state.deadreckon_wz,
                          &yaw_th_unused);

    /* 确定期望分支方向 */
    action_dir_t dir = get_action_direction(action);
    forward_state.expected_branch = (uint8_t)dir;

    /* 设置初始底盘指令 */
    *vx = forward_state.deadreckon_vx;
    *wz = forward_state.deadreckon_wz;

    ctx->priv = &forward_state;
}

static cross_exec_result_t forward_exec_fn(cross_exec_ctx_t *ctx,
                                            fp32 *vx, fp32 *wz)
{
    (void)ctx;
    const gray_sensor_t *gs = get_gray_sensor_point();

    /* 持续输出速度 */
    *vx = forward_state.deadreckon_vx;
    *wz = forward_state.deadreckon_wz;

    /* 丢线检测 */
    if (gs->line_count == 0) {
        *wz = 0.0f;
        return CROSS_EXEC_ABORT;
    }

    /* 超时保护 (10s) */
    if (osKernelSysTick() - forward_state.entry_ms > 10000) {
        *wz = 0.0f;
        return CROSS_EXEC_DONE;
    }

    /* 全黑时还在路口中间，不算线重获 */
    if (is_all_black(gs)) {
        return CROSS_EXEC_RUNNING;
    }

    /* 传感器线重获判出 — 根据期望分支方向判断 */
    bitmask_analysis_t cross_a;
    analyze_bitmask(gs->bitmask, &cross_a);
    uint8_t line_reacquired = 0;

    if (forward_state.expected_branch == DIR_LEFT) {
        line_reacquired = (cross_a.left_branch || cross_a.cluster_count >= 1)
                          && gs->position < -0.1f;
    } else if (forward_state.expected_branch == DIR_RIGHT) {
        line_reacquired = (cross_a.right_branch || cross_a.cluster_count >= 1)
                          && gs->position > 0.1f;
    } else {
        /* 直行: 跨过路口 — 通道数降到3以下退出 (至少等多走一会) */
        if (osKernelSysTick() - forward_state.entry_ms < 300)
            return CROSS_EXEC_RUNNING;
        line_reacquired = (gs->line_count <= 3);
    }

    if (line_reacquired) {
        return CROSS_EXEC_DONE;
    }

    return CROSS_EXEC_RUNNING;
}

static void forward_exit(cross_exec_ctx_t *ctx)
{
    (void)ctx;
    /* 直行执行器无需清理 */
}

const cross_executor_t forward_exec = {
    .name             = "forward",
    .enter            = forward_enter,
    .exec             = forward_exec_fn,
    .exit             = forward_exit,
    .signal_navigator = 1,    /* 直行完成 → 推进路径步 */
};
