/**
 * actions.h
 * 高层动作 API — 阻塞式函数调用，替代路径表 + 状态机嵌套。
 *
 * 两类动作:
 * ① 有线动作 — 需要路口（灰度传感器），进入 CROSS 状态执行，阻塞到底。
 * ② 无线动作 — 无需路口，暂停线跟踪任务直接控制底盘，阻塞到底。
 * ③ 瞬时动作 — 立即执行返回，不阻塞。
 *
 * 用法:
 *   cross(FORWARD);   // 直行过路口，阻塞
 *   turn(90);         // 原地右转 90°，阻塞
 *   find_line(3000);  // 前进寻线，阻塞
 *   speed(0.8f);      // 瞬时调速
 */
#ifndef ACTIONS_H
#define ACTIONS_H

#include "nav_types.h"
#include "struct_typedef.h"
#include "FreeRTOS.h"
#include "semphr.h"

/* ---- 初始化 ---- */
void actions_init(void);

/* ---- 任务忙标志 (防调试命令冲突) ---- */
extern volatile uint8_t g_mission_active;

/* ---- 底盘控制权（线跟踪任务暂停标志，actions.c 内部使用） ---- */
uint8_t actions_is_active(void);
void    actions_signal_cross_done(void);
void    actions_force_stop(void);

/* ================================================================
 * ① 有线动作 — 需要路口，阻塞
 * ================================================================ */
void cross(navigate_action_t action);

/* ================================================================
 * ② 无线动作 — 无需路口，阻塞
 * ================================================================ */
void move(int16_t mm, float spd, uint32_t timeout_ms);
void turn(int16_t deg);
int  find_line(uint32_t timeout_ms);
void wait_ir(uint32_t timeout_ms);
int  wait_ir_trigger(uint32_t timeout_ms, uint32_t stable_ms);
int  bridge(void);   /* 循线爬坡→桥面IR纠偏→pitch回平退出; 0=成功 -1=爬坡超时 */

/* ================================================================
 * ③ 瞬时动作 — 立即返回
 * ================================================================ */
void speed(float v);
void voice(uint8_t track);
void servo(uint8_t ch, uint16_t pulse_us);

/* ================================================================
 * ④ 视觉 — 阻塞等结果
 * ================================================================ */
/* 红绿灯检测。返回 0=红, 1=绿, -1=超时。 */
int tl_scan(void);
/* 扫码。成功返回 1, 超时返回 0。
 * 若 qr 非 NULL, 填充 3 字节 (ASCII 字符)。 */
int qr_scan(uint8_t qr[3]);
int digit_read(void);

#endif
