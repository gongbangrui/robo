/**
 * paths.h — 路径注册表接口
 * =======================
 * 所有命名路径集中定义在这里，debug_console 和 main.c 共用。
 *
 * ── 语法速查 ─────────────────────────────────────────────────
 *
 * 普通动作  →  { LEFT_90 }   { FORWARD }   { STOP }
 * 带参动作  →  { .action = PLATFORM, .param = 1 }
 *              { .action = MOVE, .param = 30 }
 * 引用路径  →  { .ref_name = "basic" }
 * 速度偏移  →  { FORWARD, .dvx = +0.2 }  该步速度 = 基准 + 0.2 (仅直行循线)
 *              { FORWARD, .dvx = -0.1 }  该步速度 = 基准 - 0.1
 *
 * ── 动作一览 ─────────────────────────────────────────────────
 *
 * 直行/倒车/停止:
 *   FORWARD         直行通过路口
 *   BACKWARD        倒车通过路口
 *   STOP            路径终止
 *
 * 左转 (10°~180°, 18 个):
 *   LEFT_10  LEFT_20  LEFT_30  LEFT_40  LEFT_50  LEFT_60
 *   LEFT_70  LEFT_80  LEFT_90  LEFT_100 LEFT_110 LEFT_120
 *   LEFT_130 LEFT_140 LEFT_150 LEFT_160 LEFT_170 LEFT_180
 *
 * 右转 (10°~180°, 18 个):
 *   RIGHT_10  RIGHT_20  RIGHT_30  RIGHT_40  RIGHT_50  RIGHT_60
 *   RIGHT_70  RIGHT_80  RIGHT_90  RIGHT_100 RIGHT_110 RIGHT_120
 *   RIGHT_130 RIGHT_140 RIGHT_150 RIGHT_160 RIGHT_170 RIGHT_180
 *
 * 速度控制:
 *   SLOW            减速
 *   FAST            加速
 *
 * 距离移动:
 *   MOVE            指定距离 (param=cm, 负值=倒退)
 *
 * 平台 / 桥:
 *   PLATFORM        上平台 (param: 低4位=语音编号, 0x10=数字识别, 0x20=QR)
 *   BRIDGE          过桥
 *   CLIMB           爬坡越障: 循线→山顶丢线→死推20cm→续循 (纯距离, 不用陀螺仪)
 *
 * 视觉:
 *   TRAFFIC         红绿灯检测
 *   QRSCAN          二维码扫描
 *   DIGIT           数字识别
 *
 * 其他:
 *   AUTO            路口自动选向
 *   NONE            无动作
 *
 * ── 函数 ─────────────────────────────────────────────────────
 *
 * path_find("name")  → 按名字查找路径, 未找到返回 NULL
 * g_path_registry[]  → 编译期注册表, 以 {NULL,NULL,0} 终止
 *
 * ── 示例 ─────────────────────────────────────────────────────
 *
 * 路径定义 (paths.c):
 *
 *   static const navigate_step_t s_basic[] = {
 *       { FORWARD }, { LEFT_90 },  { FORWARD }, { STOP },
 *   };
 *
 * 路径引用 (main.c):
 *
 *   static const navigate_step_t r[] = {
 *       { .ref_name = "basic" }, { .ref_name = "basic" }, { STOP },
 *   };
 * =============================================================
 */
#ifndef PATHS_H
#define PATHS_H

#include "nav_types.h"

typedef struct {
    const char             *name;
    const navigate_step_t  *steps;
    uint8_t                 length;
} path_entry_t;

extern const path_entry_t g_path_registry[];
const path_entry_t *path_find(const char *name);
uint8_t path_check_all(void);   /* 校验所有路径引用, 返回错误数 */

#endif
