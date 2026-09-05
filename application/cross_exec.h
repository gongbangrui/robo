/**
 * cross_exec.h
 * 路口执行器分发接口 — 线跟踪状态机用。
 *
 * 每种导航动作类别（转弯、直行、上平台、过桥）都有独立的 executor，
 * 各自实现 enter / exec / exit 回调。CROSS 状态根据 pending_action 分发给对应
 * executor，不再用 monolithic if-else 链。
 */
#ifndef CROSS_EXEC_H
#define CROSS_EXEC_H

#include "nav_types.h"
#include "struct_typedef.h"
#include "gray_sensor.h"    /* gray_sensor_t */

/* ---- executor 返回码 ---- */
typedef enum {
    CROSS_EXEC_RUNNING = 0,   /* executor 仍在执行，保持 CROSS 状态 */
    CROSS_EXEC_DONE    = 1,   /* 成功完成 → TRACKING */
    CROSS_EXEC_ABORT   = 2,   /* 不可恢复 → LOST_LINE */
    CROSS_EXEC_RETRACK = 3,   /* 退回 TRACKING，不推进路径，等下次触发 */
} cross_exec_result_t;

/* ---- 每个 executor 的上下文 ---- */
typedef struct {
    void   *priv;          /* executor 私有状态块（静态分配） */
    uint8_t entered;       /* 首次进入 CROSS 时由 dispatcher 置 1 */
    uint8_t action_param;  /* navigate_step_t.param，透传给 executor */
} cross_exec_ctx_t;

/* ---- executor 虚表 ---- */
typedef struct cross_executor_s {
    const char *name;   /* 调试标签 */

    /* 进入 CROSS 时调用一次。初始化 priv 状态，设置初始底盘指令。 */
    void (*enter)(cross_exec_ctx_t *ctx, navigate_action_t action,
                  const gray_sensor_t *gs, fp32 *vx, fp32 *wz);

    /* 在 CROSS 期间每 5ms 调用一次。返回 RUNNING / DONE / ABORT。 */
    cross_exec_result_t (*exec)(cross_exec_ctx_t *ctx, fp32 *vx, fp32 *wz);

    /* 退出时调用一次（无论 DONE 还是 ABORT）。清理 / 停用执行器。 */
    void (*exit)(cross_exec_ctx_t *ctx);

    /* 为 1 时，DONE 后通过 just_crossed / exit_event 通知 navigate_task 推进路径。
     * 转弯、直行、上平台设为 1。不消耗路径步的动作设为 0。 */
    uint8_t signal_navigator;
} cross_executor_t;

/* ---- dispatcher API ---- */

/* 注册所有 executor。启动时调用一次（从 line_track_task init）。 */
void cross_exec_init(void);

/* 根据导航动作查找对应 executor。
 * 对不需要 CROSS 的动作（STOP、NONE）返回 NULL。 */
const cross_executor_t *cross_exec_lookup(navigate_action_t action);

/* 当前活跃 executor 的全局上下文（同一时间只有一个）。 */
extern cross_exec_ctx_t g_cross_ctx;

#endif
