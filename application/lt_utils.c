/**
 * lt_utils.c
 * 线跟踪共享工具函数实现。
 *
 * 这些函数原为 line_track_task.c 中的 static 函数，
 * 提取到此文件供 line_track_task 和各 cross executor 共用。
 */
#include "lt_utils.h"
#include <string.h>

/* ---- 角度归一化 ---- */
fp32 normalize_angle(fp32 a) {
  while (a > DEG2RAD(180))
    a -= DEG2RAD(360);
  while (a < DEG2RAD(-180))
    a += DEG2RAD(360);
  return a;
}

/* ---- 位掩码簇分析 ---- */

static void classify_cluster(bitmask_analysis_t *out, uint8_t idx) {
  uint8_t start = out->cluster_start[idx];
  uint8_t end = start + out->cluster_width[idx] - 1;
  if (start <= ZONE_CENTER_END && end >= ZONE_CENTER_START)
    out->center_cluster_idx = idx;
  if (end <= ZONE_LEFT_END && out->cluster_width[idx] >= 2)
    out->left_branch = 1;
  if (start >= ZONE_RIGHT_START && out->cluster_width[idx] >= 2)
    out->right_branch = 1;
}

void analyze_bitmask(uint16_t mask, bitmask_analysis_t *out) {
  memset(out, 0, sizeof(*out));
  out->center_cluster_idx = 0xFF;
  uint8_t in_cluster = 0, idx = 0;
  for (uint8_t ch = 0; ch < 16; ch++) {
    uint8_t bit = (mask >> ch) & 1;
    if (bit && !in_cluster) {
      if (idx < MAX_CLUSTERS) {
        out->cluster_start[idx] = ch;
        out->cluster_width[idx] = 1;
      }
      in_cluster = 1;
    } else if (bit && in_cluster) {
      if (idx < MAX_CLUSTERS)
        out->cluster_width[idx]++;
    } else if (!bit && in_cluster) {
      if (idx < MAX_CLUSTERS) {
        classify_cluster(out, idx);
        idx++;
      }
      in_cluster = 0;
    }
  }
  if (in_cluster && idx < MAX_CLUSTERS) {
    classify_cluster(out, idx);
    idx++;
  }
  out->cluster_count = idx;
  if (out->cluster_count == 1 && out->cluster_width[0] >= 6)
    out->is_wide_blob = 1;
}

/* ---- 线状态判断 ---- */

uint8_t line_is_centered(const gray_sensor_t *gs) {
  if (gs->line_count < 1 || gs->line_count > 10)
    return 0;
  fp32 p = gs->position;
  return (p > -0.25f && p < 0.25f);
}

uint8_t approaching_intersection(const bitmask_analysis_t *a) {
  return a->cluster_count >= 2 || a->is_wide_blob;
}

uint8_t is_all_black(const gray_sensor_t *gs) { return gs->line_count >= 14; }

uint8_t is_any_line(const gray_sensor_t *gs) { return gs->line_count >= 1; }

uint8_t edge_branch_reached(const gray_sensor_t *gs, action_dir_t dir) {
  if (dir == DIR_LEFT)
    return (gs->bitmask & 0x0003) == 0x0003;
  if (dir == DIR_RIGHT)
    return (gs->bitmask & 0xC000) == 0xC000;
  return 0;
}

static uint8_t bit_count8(uint8_t v) {
  uint8_t n = 0;
  while (v) {
    v &= (uint8_t)(v - 1);
    n++;
  }
  return n;
}

uint8_t turn_trigger_reached(const gray_sensor_t *gs,
                             navigate_action_t action) {
  action_dir_t dir = get_action_direction(action);

  if (action == LEFT_90 || action == RIGHT_90) {
    uint8_t side = (dir == DIR_LEFT)
                       ? bit_count8((uint8_t)(gs->bitmask & 0x00FF))
                       : bit_count8((uint8_t)((gs->bitmask >> 8) & 0xFF));
    return side >= TURN90_SIDE_MIN;
  }
  return edge_branch_reached(gs, dir);
}

/* ---- 动作方向分类 ---- */

action_dir_t get_action_direction(navigate_action_t action) {
  unsigned a = (unsigned)action;
  if (a >= 10 && a <= 199)
    return DIR_LEFT;
  if (a >= 210 && a <= 399)
    return DIR_RIGHT;
  return DIR_NONE;
}

uint8_t is_turn_action(navigate_action_t action) {
  unsigned a = (unsigned)action;
  return (a >= 10 && a <= 199) || (a >= 210 && a <= 399);
}
