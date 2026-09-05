/**
 * paths.c — 路径注册表
 *
 * 语法: { LEFT_90 }  普通动作
 *       { .action = PLATFORM, .param = 1 }  带参
 *       { .ref_name = "basic" }  引用子路径
 */
#include "paths.h"
#include "nav_types.h"
#include "uart_debug.h"
#include <string.h>

/*
 * ════════════════════════════════════════════════════════════
 * 基础训练路径
 * ════════════════════════════════════════════════════════════ */

static const navigate_step_t s_basic[] = {
    {FORWARD}, {LEFT_90}, {FORWARD}, {STOP}};
static const navigate_step_t s_sharp[] = {
    {FORWARD}, {LEFT_30}, {FORWARD}, {STOP}};
static const navigate_step_t s_obtuse[] = {
    {FORWARD}, {RIGHT_150}, {FORWARD}, {STOP}};
static const navigate_step_t s_uturn[] = {
    {FORWARD}, {LEFT_180}, {FORWARD}, {STOP}};
static const navigate_step_t s_fork[] = {
    {FORWARD}, {LEFT_90}, {FORWARD}, {STOP}};
static const navigate_step_t s_complex[] = {
    {FORWARD},  {LEFT_30}, {FORWARD}, {RIGHT_150}, {FORWARD},
    {LEFT_180}, {FORWARD}, {LEFT_90}, {FORWARD},   {STOP},
};

/*
 * ════════════════════════════════════════════════════════════
 * 平台 / 桥
 * ════════════════════════════════════════════════════════════ */

static const navigate_step_t s_platform[] = {
    {FORWARD},
    {.action = PLATFORM, .param = 1},
    {FORWARD},
    {STOP},
};
static const navigate_step_t s_bridge[] = {
    {FORWARD},
    {BRIDGE},
    {FORWARD},
    {STOP},
};

/*
 * ════════════════════════════════════════════════════════════
 * 综合演示
 * ════════════════════════════════════════════════════════════ */

static const navigate_step_t s_demo[] = {
    {.ref_name = "basic"},
    {.ref_name = "basic"},
    {FORWARD},
    {LEFT_90},
    {FORWARD},
    {TRAFFIC},
    {FORWARD},
    {RIGHT_90},
    {BRIDGE},
    {FORWARD},
    {.action = PLATFORM, .param = 1},
    {.ref_name = "sharp"},
    {STOP},
};

static const navigate_step_t s_demo_vision[] = {
    {FORWARD},
    {LEFT_90},
    {FORWARD},
    {.action = PLATFORM, .param = 2},
    {QRSCAN},
    {FORWARD},
    {.action = PLATFORM, .param = 3},
    {DIGIT},
    {FORWARD},
    {STOP},
};

/*
 * ════════════════════════════════════════════════════════════
 * 红绿灯路由示例
 * ════════════════════════════════════════════════════════════ */
/* 爬坡越障测试: 循线 → 山顶丢线 → 死推越顶 → 续循 */
static const navigate_step_t s_climb_test[] = {
    {FORWARD},
    {CLIMB},
    {FORWARD},
    {STOP},
};

/* dvx 速度偏移测试: 各步不同速度 */
static const navigate_step_t s_dvx_test[] = {
    {FORWARD, .dvx = +0.3f}, /* 快速 */
    {FORWARD, .dvx = 0.0f},  /* 基准 */
    {FORWARD, .dvx = -0.2f}, /* 慢速 */
    {STOP},
};
static const navigate_step_t s_tl_green_to_p4[] = {
    {TRAFFIC},
    {FORWARD},
    {RIGHT_90},
    {FORWARD},
    {.action = PLATFORM, .param = 4},
    {STOP},
};
static const navigate_step_t s_tl_red_to_p3[] = {
    {TRAFFIC},
    {FORWARD},
    {LEFT_90},
    {FORWARD},
    {.action = PLATFORM, .param = 3},
    {STOP},
};

/*
 * ════════════════════════════════════════════════════════════
 * 用户自定义路径
 * ════════════════════════════════════════════════════════════ */

static const navigate_step_t s_p1_p2[] = {
    {FORWARD},
    {BRIDGE},
    {FORWARD},
    {LINE_END},
};

static const navigate_step_t s_p2_z1[] = {
    {RIGHT_40}, {CLIMB}, {LEFT_40}, {FORWARD}, {LEFT_40}, {IR},
};

static const navigate_step_t s_z1_p3[] = {
    {RIGHT_40}, {CLIMB}, {LEFT_40}, {FORWARD}, {LEFT_40}, {IR},
};

static const navigate_step_t s_p2_men4[] = {{RIGHT_40}, {FORWARD},  {LEFT_40},
                                            {FORWARD},  {RIGHT_90}, {FORWARD}};
static const navigate_step_t s_men4_men3[] = {
    {BACKWARD}, {RIGHT_50}, {FORWARD}};
static const navigate_step_t s_men3_men1[] = {
    {BACKWARD}, {RIGHT_40}, {FORWARD}, {LEFT_90}, {FORWARD}};
static const navigate_step_t s_men1_men2[] = {{BACKWARD}, {LEFT_50}, {FORWARD}};
static const navigate_step_t s_jd1_p5[] = {{FORWARD}, {LEFT_40},  {FORWARD},
                                           {FORWARD}, {PLATFORM}, {LEFT_180}};
static const navigate_step_t s_p5_jd4[] = {{LEFT_150}, {FORWARD},  {RIGHT_90},
                                           {FORWARD},  {RIGHT_90}, {FORWARD}};
static const navigate_step_t s_jd4_p7[] = {
    {LEFT_90}, {FORWARD}, {LEFT_90}, {PLATFORM}, {LEFT_180}};
static const navigate_step_t s_p5_jd2[] = {{LEFT_30}, {FORWARD},  {RIGHT_30},
                                           {FORWARD}, {RIGHT_90}, {FORWARD}};
static const navigate_step_t s_p60_jd5[] = {{FORWARD}, {LEFT_90},   {FORWARD},
                                            {FORWARD}, {LEFT_90},   {FORWARD},
                                            {LEFT_90}, {RIGHT_120}, {FORWARD}};
static const navigate_step_t s_p60_jd4[] = {
    {FORWARD}, {RIGHT_90}, {FORWARD}, {LEFT_90}, {FORWARD}};
static const navigate_step_t s_p7_jd6[] = {{RIGHT_90}, {FORWARD}, {RIGHT_90},
                                           {FORWARD},  {LEFT_90}, {FORWARD},
                                           {LEFT_90},  {FORWARD}};
static const navigate_step_t s_jd6_jd2[] = {{FORWARD}, {RIGHT_90}, {FORWARD}};
static const navigate_step_t s_jd7_p2[] = {{FORWARD}, {LEFT_150}, {FORWARD},
                                           {LEFT_40}, {FORWARD},  {PLATFORM},
                                           {LEFT_180}};
static const navigate_step_t s_jd8_p2[] = {{FORWARD}, {RIGHT_40}, {FORWARD},
                                           {LEFT_40}, {FORWARD},  {PLATFORM},
                                           {LEFT_180}};
static const navigate_step_t s_p8_p60[] = {
    {LEFT_150}, {FORWARD}, {LEFT_120}, {FORWARD},  {RIGHT_90}, {FORWARD},
    {RIGHT_90}, {FORWARD}, {FORWARD},  {RIGHT_90}, {FORWARD}};
static const navigate_step_t s_p2_men1[] = {{RIGHT_40}, {FORWARD}, {LEFT_140},
                                            {FORWARD},  {LEFT_90}, {FORWARD}};
static const navigate_step_t s_men1_men2b[] = {
    {BACKWARD}, {LEFT_50}, {FORWARD}, {RIGHT_90}, {FORWARD}};
static const navigate_step_t s_men2_men3[] = {
    {BACKWARD}, {LEFT_40}, {FORWARD}, {FORWARD}, {RIGHT_140}, {FORWARD}};
static const navigate_step_t s_men3_men4b[] = {
    {BACKWARD}, {LEFT_50}, {FORWARD}};
static const navigate_step_t s_men3_men2[] = {
    {BACKWARD}, {RIGHT_40}, {FORWARD}, {FORWARD}, {LEFT_140}};
static const navigate_step_t s_men2_men1[] = {
    {BACKWARD}, {RIGHT_50}, {FORWARD}};

/*
 * ════════════════════════════════════════════════════════════
 * 注册表
 * ════════════════════════════════════════════════════════════ */

#define ENTRY(name) {#name, s_##name, sizeof(s_##name) / sizeof(s_##name[0])}

const path_entry_t g_path_registry[] = {
    ENTRY(basic),       ENTRY(sharp),          ENTRY(obtuse),
    ENTRY(uturn),       ENTRY(fork),           ENTRY(complex),
    ENTRY(platform),    ENTRY(bridge),         ENTRY(demo),
    ENTRY(demo_vision), ENTRY(p1_p2),          ENTRY(p2_z1),
    ENTRY(z1_p3),       ENTRY(tl_green_to_p4), ENTRY(tl_red_to_p3),
    ENTRY(p2_men4),     ENTRY(men4_men3),      ENTRY(men3_men1),
    ENTRY(men1_men2),   ENTRY(jd1_p5),         ENTRY(p5_jd4),
    ENTRY(jd4_p7),      ENTRY(p5_jd2),         ENTRY(p60_jd5),
    ENTRY(p60_jd4),     ENTRY(p7_jd6),         ENTRY(jd6_jd2),
    ENTRY(jd7_p2),      ENTRY(jd8_p2),         ENTRY(p8_p60),
    ENTRY(p2_men1),     ENTRY(men1_men2b),     ENTRY(men2_men3),
    ENTRY(men3_men4b),  ENTRY(men3_men2),      ENTRY(men2_men1),
    ENTRY(climb_test),  ENTRY(dvx_test),       {NULL, NULL, 0},
};

const path_entry_t *path_find(const char *name) {
  for (const path_entry_t *p = g_path_registry; p->name; p++)
    if (strcmp(p->name, name) == 0)
      return p;
  return NULL;
}

/* 校验所有路径及子路径引用是否有效。返回错误数。 */
uint8_t path_check_all(void) {
  uint8_t errs = 0;
  for (const path_entry_t *p = g_path_registry; p->name; p++) {
    for (uint8_t i = 0; i < p->length; i++) {
      const char *ref = p->steps[i].ref_name;
      if (ref && !path_find(ref)) {
        errs++;
        uart_printf("[PATH-ERR] 路径 %s 第%d步引用不存在: \"%s\"\r\n", p->name,
                    i, ref);
      }
    }
  }
  return errs;
}
