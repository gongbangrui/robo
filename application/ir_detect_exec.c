/**
 * ir_detect_exec.c
 * 前红外检测执行器 — 循线中前红外触发后, 进 CROSS 停下结束。
 * 用于上坡到平台边缘"循线前进, 前红外检测到障碍即停"的场景。
 *
 * 路径用法: {IR}
 */
#include "chassis_task.h"
#include "cmsis_os.h"
#include "cross_exec.h"
#include "ir_sensor.h"

/* ---- 触发电平配置 (0/1) ----
 * IR_TRIGGER_LEVEL = 1  → front==1 触发 (有障碍)
 * IR_TRIGGER_LEVEL = 0  → front==0 触发 (无障碍/无遮挡) */
#define IR_TRIGGER_LEVEL 0

/* 查询前红外是否满足触发条件 (电平已按 IR_TRIGGER_LEVEL 反转) */
uint8_t ir_detect_triggered(void) {
    ir_sensor_poll();
    uint8_t front = get_ir_sensor_point()->front;
    return (IR_TRIGGER_LEVEL) ? front : !front;
}

static void ir_enter(cross_exec_ctx_t *ctx, navigate_action_t action,
                     const gray_sensor_t *gs, fp32 *vx, fp32 *wz) {
    (void)action;
    (void)gs;
    *vx = 0.0f;
    *wz = 0.0f;
    ctx->priv = NULL;
}

static cross_exec_result_t ir_exec_fn(cross_exec_ctx_t *ctx, fp32 *vx,
                                      fp32 *wz) {
    (void)ctx;
    *vx = 0.0f;
    *wz = 0.0f;
    return CROSS_EXEC_DONE;
}

static void ir_exit(cross_exec_ctx_t *ctx) { (void)ctx; }

const cross_executor_t ir_detect_exec = {
    .name = "ir_detect",
    .enter = ir_enter,
    .exec = ir_exec_fn,
    .exit = ir_exit,
    .signal_navigator = 1,
};
