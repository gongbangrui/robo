/**
 * cross_exec.c
 * 路口执行器注册表和分发器。
 *
 * navigate_action_t → navigate_category_t → cross_executor_t 的映射。
 * 进入 CROSS 时查找活跃 executor，之后每 5ms 调用其 exec() 直到返回 DONE 或 ABORT。
 */
#include "cross_exec.h"
#include <stddef.h>   /* NULL */

/* ---- 全局上下文（同一时间只有一个活跃 executor） ---- */
cross_exec_ctx_t g_cross_ctx;

/* ---- executor 表（按 navigate_category_t 索引） ---- */
#define MAX_CATEGORIES 9

static cross_executor_t g_executors[MAX_CATEGORIES];

/* 前向声明 — 每个 executor 定义在各自的 .c 文件中 */
extern const cross_executor_t turn_exec;
extern const cross_executor_t forward_exec;
extern const cross_executor_t platform_exec;
extern const cross_executor_t bridge_exec;
extern const cross_executor_t vision_exec;
extern const cross_executor_t move_exec;
extern const cross_executor_t climb_exec;
extern const cross_executor_t ir_detect_exec;

void cross_exec_init(void)
{
    /* 清零 */
    for (uint8_t i = 0; i < MAX_CATEGORIES; i++) {
        g_executors[i].name             = NULL;
        g_executors[i].enter            = NULL;
        g_executors[i].exec             = NULL;
        g_executors[i].exit             = NULL;
        g_executors[i].signal_navigator = 0;
    }

    /* 注册已知 executor */
    g_executors[CAT_TURN]     = turn_exec;
    g_executors[CAT_FWD]  = forward_exec;
    g_executors[CAT_PLAT] = platform_exec;
    g_executors[CAT_BRDG]   = bridge_exec;
    g_executors[CAT_VIS]   = vision_exec;
    g_executors[CAT_MOVE]     = move_exec;
    g_executors[CAT_CLIMB]    = climb_exec;
    g_executors[CAT_IR]       = ir_detect_exec;

    g_cross_ctx.priv        = NULL;
    g_cross_ctx.entered     = 0;
    g_cross_ctx.action_param = 0;
}

const cross_executor_t *cross_exec_lookup(navigate_action_t action)
{
    navigate_category_t cat = navigate_action_category(action);
    if (cat >= MAX_CATEGORIES) return NULL;
    if (g_executors[cat].exec == NULL) return NULL;
    return &g_executors[cat];
}
