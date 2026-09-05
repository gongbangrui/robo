/**
 * navigate_task.h
 * Path-planning layer. Drives line_track_task by telling it
 * what action to take at each intersection.
 *
 * The path is a data table (NOT code). Change the array to change the route.
 */
#ifndef NAVIGATE_TASK_H
#define NAVIGATE_TASK_H

#include "struct_typedef.h"
#include "nav_types.h"

#define NAVIGATE_CONTROL_TIME_MS 20

/**
 * @brief  Navigate FreeRTOS task entry (20ms polling period).
 */
extern void navigate_task(void const *pvParameters);

/**
 * @brief  Load a path table from application code.
 *         Path format: array of navigate_action_t, terminated by STOP.
 *         Example: {FORWARD, TURN_LEFT, FORWARD, TURN_RIGHT, STOP}
 */
extern void navigate_load_path(const navigate_step_t *path, uint8_t length);

/* navigate_init + run are in the bottom of the file */

/**
 * @brief  Get current path step index (for OLED debug).
 */
extern uint8_t navigate_get_current_step(void);

/**
 * @brief  Get total path length (for OLED debug).
 */
extern uint8_t navigate_get_path_length(void);

/* 初始化信号量 (run() 依赖), freertos.c 启动时调用一次 */
extern void navigate_init_semaphore(void);

/* run("name") — 加载并阻塞执行命名路径, 完成后返回 */
extern void run(const char *name);
extern void run_path(const navigate_step_t *path, uint8_t length);

/* 清空路径表 + 解锁 run(), 供 stop 命令使用 */
extern void navigate_stop(void);

#endif
