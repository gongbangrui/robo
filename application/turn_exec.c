/**
 * turn_exec.c
 * 转弯执行器 — 基于陀螺仪 yaw 的死推算。
 *
 * 从原 line_track_task.c CROSS 状态中提取，逻辑不变。
 * 适用于所有 yaw_threshold > 0 的转弯动作。
 */
#include "cross_exec.h"
#include "lt_utils.h"
#include "line_track_task.h"  /* DR_TABLE, get_deadreckon_params 的声明 */
#include "INS_task.h"
#include "chassis_task.h"
#include "gray_sensor.h"
#include "cmsis_os.h"

#ifndef PI
#define PI 3.14159265358979323846f
#endif

/* ---- 转弯私有状态 ---- */
typedef struct {
    fp32     cross_yaw_start;          /* 上一帧 yaw (逐帧累加用) */
    fp32     deadreckon_yaw_threshold;  /* yaw 累积阈值 (rad) */
    fp32     deadreckon_vx;            /* 死推算前向速度 */
    fp32     deadreckon_wz;            /* 死推算角速度 */
    fp32     accumulated_yaw;           /* 已累积的偏航角 */
    uint32_t entry_ms;                 /* 进入时间 (ms) */
} turn_priv_t;

static turn_priv_t turn_state;

/* ---- 前向声明 —— 来自 line_track_task.c 的 DR_TABLE 接口 ---- */
/* DR_TABLE 和 get_deadreckon_params 的声明见下方。
 * 实际实现保留在 line_track_task.c 中（保持兼容性）。
 * turn_exec 通过 line_track_task.h 暴露的新接口获取参数。 */

/* ---- executor 接口 ---- */

static void turn_enter(cross_exec_ctx_t *ctx, navigate_action_t action,
                       const gray_sensor_t *gs, fp32 *vx, fp32 *wz)
{
    (void)gs;

    turn_state.entry_ms = osKernelSysTick();
    turn_state.accumulated_yaw = 0.0f;

    /* 从 DR_TABLE 获取死推算参数 */
    extern void get_deadreckon_params(navigate_action_t a,
                                       fp32 *vx_out, fp32 *wz_out, fp32 *yaw_th);
    get_deadreckon_params(action,
                          &turn_state.deadreckon_vx,
                          &turn_state.deadreckon_wz,
                          &turn_state.deadreckon_yaw_threshold);

    /* 记录 yaw 参考起点 */
    turn_state.cross_yaw_start = get_INS_angle_point()[INS_YAW_ADDRESS_OFFSET];

    /* 设置初始底盘指令 */
    *vx = turn_state.deadreckon_vx;
    *wz = turn_state.deadreckon_wz;

    ctx->priv = &turn_state;
}

static cross_exec_result_t turn_exec_fn(cross_exec_ctx_t *ctx,
                                         fp32 *vx, fp32 *wz)
{
    (void)ctx;

    /* 持续输出死推算速度 (丢线也继续，纯靠陀螺仪) */
    *vx = turn_state.deadreckon_vx;
    *wz = turn_state.deadreckon_wz;

    /* 超时保护 (10s) */
    if (osKernelSysTick() - turn_state.entry_ms > 10000) {
        *wz = 0.0f;
        return CROSS_EXEC_DONE;
    }

    /* yaw 逐帧累加判出 */
    fp32 yaw_now = get_INS_angle_point()[INS_YAW_ADDRESS_OFFSET];
    fp32 delta   = yaw_now - turn_state.cross_yaw_start;
    if (delta > PI)       delta -= 2.0f * PI;
    else if (delta < -PI) delta += 2.0f * PI;
    turn_state.accumulated_yaw += delta;
    turn_state.cross_yaw_start  = yaw_now;

    if (turn_state.accumulated_yaw >= turn_state.deadreckon_yaw_threshold ||
        turn_state.accumulated_yaw <= -turn_state.deadreckon_yaw_threshold) {
        return CROSS_EXEC_DONE;
    }

    return CROSS_EXEC_RUNNING;
}

static void turn_exit(cross_exec_ctx_t *ctx)
{
    (void)ctx;
    /* 转弯执行器无需清理 */
}

const cross_executor_t turn_exec = {
    .name             = "turn",
    .enter            = turn_enter,
    .exec             = turn_exec_fn,
    .exit             = turn_exit,
    .signal_navigator = 1,    /* 转弯完成 → 推进路径步 */
};
