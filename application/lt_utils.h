/**
 * lt_utils.h
 * Shared utilities for line-tracking state machine and cross executors.
 *
 * Functions that were previously static in line_track_task.c are exposed
 * here so that cross executors (turn_exec, forward_exec, etc.) can use
 * them without duplication.
 */
#ifndef LT_UTILS_H
#define LT_UTILS_H

#include "struct_typedef.h"
#include "nav_types.h"
#include "gray_sensor.h"    /* gray_sensor_t 完整定义 */

/* ---- 角度工具 ---- */
#define DEG2RAD(deg) ((deg) * 0.0174532925f)
#define RAD2DEG(rad) ((rad) * 57.29578f)

fp32 normalize_angle(fp32 a);

/* ---- 位掩码簇分析 ---- */

#define ZONE_LEFT_START   0
#define ZONE_LEFT_END     4
#define ZONE_CENTER_START 5
#define ZONE_CENTER_END  10
#define ZONE_RIGHT_START 11
#define ZONE_RIGHT_END   15

#define MAX_CLUSTERS 4

typedef enum {
    DIR_NONE  = 0,
    DIR_LEFT  = 1,
    DIR_RIGHT = 2,
} action_dir_t;

typedef struct {
    uint8_t cluster_count;
    uint8_t cluster_start[MAX_CLUSTERS];
    uint8_t cluster_width[MAX_CLUSTERS];
    uint8_t left_branch;
    uint8_t right_branch;
    uint8_t center_cluster_idx;
    uint8_t is_wide_blob;
} bitmask_analysis_t;

void analyze_bitmask(uint16_t mask, bitmask_analysis_t *out);

/* ---- 线状态判断 ---- */
uint8_t line_is_centered(const gray_sensor_t *gs);
uint8_t approaching_intersection(const bitmask_analysis_t *a);
uint8_t is_all_black(const gray_sensor_t *gs);
uint8_t is_any_line(const gray_sensor_t *gs);

/* 转弯精准触发: 目标侧最边缘 2 探头同时压黑 (支线到达车轴线正下方).
 * 左转 = ch0&ch1, 右转 = ch14&ch15. 对齐旧 run_task 的 hui[0]&hui[1]. */
uint8_t edge_branch_reached(const gray_sensor_t *gs, action_dir_t dir);

/* 直角弯 (LEFT_90/RIGHT_90) 触发: 对应侧半边 (8 路) 压黑探头数下限.
 * 8 = 全亮 (直角横线与探头排平行, 到达瞬间同时全亮; 160°等斜线只能
 * 渐进亮 3~7 个, 全亮条件可滤掉). 个别探头脏/不稳时可降至 7 容错. */
#define TURN90_SIDE_MIN 8

/* 按动作分派转弯触发条件:
 *   LEFT_90/RIGHT_90 → 对应侧半边压黑数 >= TURN90_SIDE_MIN
 *   其他转弯动作     → edge_branch_reached (边缘双探头) */
uint8_t turn_trigger_reached(const gray_sensor_t *gs, navigate_action_t action);

/* ---- 动作方向 ---- */
action_dir_t get_action_direction(navigate_action_t action);
uint8_t      is_turn_action(navigate_action_t action);

#endif
