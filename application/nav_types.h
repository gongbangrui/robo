/**
 * nav_types.h
 * Navigation action enum and path step type.
 */
#ifndef NAV_TYPES_H
#define NAV_TYPES_H

#include "struct_typedef.h"

/*
 * ── 值域契约 (勿破坏!) ──────────────────────────────────────
 *   0..9     控制动作
 *   10..199  左转, 值 = 目标度数  (is_turn_action / category 按区间判定)
 *   200..209 功能动作
 *   210..399 右转, 值-200 = 目标度数
 *
 * 任何新动作严禁落入 [10,199] / [210,399], 否则会与 LEFT_x/RIGHT_x
 * 重值: DR_TABLE 指定初始化器同下标互相覆写, 区间判定误判为转弯.
 * ──────────────────────────────────────────────────────────── */
typedef enum {
    FORWARD  = 0,                            /* 直行过路口 */
    IR      = 1,                            /* 前红外触发即结束 (front==1) */
    BACKWARD = 4,                            /* 倒车过路口 */
    STOP     = 3,                            /* 终止 */
    AUTO     = 5,                            /* 自动选向 */
    PLATFORM = 6,                            /* 上平台 */
    RUN_PATH = 7,                            /* 子路径引用 */
    LINE_END = 8,                            /* 循线直到丢线 */
    CLIMB    = 9,                            /* 爬坡越障: 循线→丢线→死推→续循 */

    LEFT_10  = 10,  RIGHT_10  = 210,         /* 10° */
    LEFT_20  = 20,  RIGHT_20  = 220,
    LEFT_30  = 30,  RIGHT_30  = 230,
    LEFT_40  = 40,  RIGHT_40  = 240,
    LEFT_50  = 50,  RIGHT_50  = 250,
    LEFT_60  = 60,  RIGHT_60  = 260,
    LEFT_70  = 70,  RIGHT_70  = 270,
    LEFT_80  = 80,  RIGHT_80  = 280,
    LEFT_90  = 90,  RIGHT_90  = 290,
    LEFT_100 = 100, RIGHT_100 = 300,
    LEFT_110 = 110, RIGHT_110 = 310,
    LEFT_120 = 120, RIGHT_120 = 320,
    LEFT_130 = 130, RIGHT_130 = 330,
    LEFT_140 = 140, RIGHT_140 = 340,
    LEFT_150 = 150, RIGHT_150 = 350,
    LEFT_160 = 160, RIGHT_160 = 360,
    LEFT_170 = 170, RIGHT_170 = 370,
    LEFT_180 = 180, RIGHT_180 = 380,

    SLOW    = 200,                           /* 减速 */
    FAST    = 201,                           /* 加速 */
    TRAFFIC = 202,                           /* 红绿灯 */
    QRSCAN  = 203,                           /* 二维码 */
    DIGIT   = 204,                           /* 数字识别 */
    BRIDGE  = 205,                           /* 过桥 */
    MOVE    = 206,                           /* 指定距离 (param=cm, 负=倒退) */
    NONE    = 207,                           /* 无动作 */
} navigate_action_t;

typedef struct {
    navigate_action_t action;
    uint8_t           param;
    const char       *ref_name;
    fp32              dvx;   /* 相对 base_speed 的速度偏移 (m/s), 0=默认 */
} navigate_step_t;

typedef enum {
    CAT_NONE = 0,
    CAT_TURN = 1,
    CAT_FWD  = 2,
    CAT_PLAT = 3,
    CAT_BRDG = 4,
    CAT_VIS  = 5,
    CAT_MOVE = 6,
    CAT_CLIMB = 7,
    CAT_IR   = 8,
} navigate_category_t;

navigate_category_t navigate_action_category(navigate_action_t action);

#endif
