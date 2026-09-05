/**
 * navigate_task.c
 * Path-planning layer — table-driven, not if-else.
 *
 * The path is loaded once at startup (or at any time to change routes).
 * Each time line_track_task signals an intersection-exit event,
 * this task advances its step index and issues the next action.
 */
#include "navigate_task.h"
#include "cmsis_os.h"
#include "debug_console.h"
#include "line_track_task.h"
#include "paths.h"
#include <string.h>

/* ---- semaphore for run() blocking ---- */
static SemaphoreHandle_t s_path_done_sem;

void navigate_init_semaphore(void) {
    s_path_done_sem = xSemaphoreCreateBinary();
}

/* ---- path table ---- */

#define MAX_PATH_LENGTH 32

static navigate_step_t g_path[MAX_PATH_LENGTH];
static uint8_t           g_path_len  = 0;
static uint8_t           g_step      = 0;

/* ---- sub-path (ref_name 引用) ---- */
#define SUB_PATH_MAX 16
static navigate_step_t g_sub_path[SUB_PATH_MAX];
static uint8_t         g_sub_len;
static uint8_t         g_sub_step;
static uint8_t         g_sub_active;
static uint8_t         g_sub_resume_step;

static void handle_step(void);

/* ---- public ---- */

void navigate_load_path(const navigate_step_t *path, uint8_t length)
{
    if (length > MAX_PATH_LENGTH) length = MAX_PATH_LENGTH;
    for (uint8_t i = 0; i < length; i++) g_path[i] = path[i];
    g_path_len = length;
    g_step     = 0;
    g_sub_active = 0;
    if (g_path_len > 0) line_track_set_nav_step(g_path[0]);
}

void navigate_load_action_path(const navigate_action_t *path, uint8_t length)
{
    if (length > MAX_PATH_LENGTH) length = MAX_PATH_LENGTH;
    for (uint8_t i = 0; i < length; i++) {
        g_path[i].action   = path[i];
        g_path[i].param    = 0;
        g_path[i].ref_name = NULL;
        g_path[i].dvx      = 0.0f;
    }
    g_path_len = length;
    g_step     = 0;
    g_sub_active = 0;
    if (g_path_len > 0) line_track_set_nav_step(g_path[0]);
}

uint8_t navigate_get_current_step(void)  { return g_step; }
uint8_t navigate_get_path_length(void)   { return g_path_len; }

/* ---- run() — 阻塞执行命名路径 ---- */

void run(const char *name) {
    const path_entry_t *pe = path_find(name);
    if (!pe) {
        debug_console_printf("[PATH-ERR] run(\"%s\") 路径不存在! "
                             "用 'path' 命令查看可用路径.\r\n", name);
        return;
    }
    navigate_load_path(pe->steps, pe->length);
    xSemaphoreTake(s_path_done_sem, portMAX_DELAY);
}

void run_path(const navigate_step_t *path, uint8_t length) {
    navigate_load_path(path, length);
    xSemaphoreTake(s_path_done_sem, portMAX_DELAY);
}

void navigate_stop(void) {
    g_path_len = 0;
    g_step = 0;
    g_sub_active = 0;
    if (s_path_done_sem) xSemaphoreGive(s_path_done_sem);
}

/* ---- task ---- */

void navigate_task(void const *pvParameters)
{
    (void)pvParameters;
    osDelay(500);

    /* 启动校验: 所有路径的 ref_name 引用是否有效 */
    {
        uint8_t errs = path_check_all();
        if (errs)
            debug_console_printf("[PATH-CHECK] %u 个无效引用!\r\n", errs);
        else
            debug_console_puts("[PATH-CHECK] 所有路径引用 OK\r\n");
    }

    for (;;) {
        if (line_track_got_exit_event()) {

            if (g_sub_active) {
                g_sub_step++;
                if (g_sub_step < g_sub_len &&
                    g_sub_path[g_sub_step].action != STOP) {
                    line_track_set_nav_step(g_sub_path[g_sub_step]);
                } else {
                    g_sub_active = 0;
                    g_step = g_sub_resume_step;
                    goto advance_parent;
                }
            } else {
                g_step++;
            advance_parent:
                if (g_step < g_path_len && g_path[g_step].action != STOP) {
                    handle_step();
                } else {
                    line_track_set_nav_action(STOP);
                    if (s_path_done_sem) xSemaphoreGive(s_path_done_sem);
                }
            }
        }

        osDelay(NAVIGATE_CONTROL_TIME_MS);
    }
}

static void handle_step(void)
{
    if (g_path[g_step].ref_name) {
        const path_entry_t *pe = path_find(g_path[g_step].ref_name);
        if (pe) {
            uint8_t len = pe->length > SUB_PATH_MAX ? SUB_PATH_MAX : pe->length;
            const navigate_step_t *src = pe->steps;
            for (uint8_t i = 0; i < len; i++) g_sub_path[i] = src[i];
            g_sub_len = len; g_sub_step = 0; g_sub_active = 1;
            g_sub_resume_step = g_step + 1;
            if (len > 0) line_track_set_nav_step(g_sub_path[0]);
            return;
        }
        debug_console_printf("[PATH-ERR] 路径第%d步引用不存在: \"%s\". "
                             "已强制停止.\r\n", g_step, g_path[g_step].ref_name);
        line_track_set_nav_action(STOP);
        if (s_path_done_sem) xSemaphoreGive(s_path_done_sem);
        return;
    }
    line_track_set_nav_step(g_path[g_step]);
}
