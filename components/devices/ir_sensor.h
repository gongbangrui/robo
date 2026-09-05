/**
 * ir_sensor.h
 * 红外传感器设备驱动 — 前/左/右三路红外传感器数据访问。
 *
 * 硬件无关层。底层 GPIO 操作由 bsp_ir_sensor.h/c 提供。
 * ir_sensor_poll() 在 5ms 循环中调用，纯 GPIO 读取，纳秒级无阻塞。
 */
#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include "struct_typedef.h"

/* ---- 传感器数据结构 ---- */
typedef struct {
    uint8_t front;   /* 前红外:   0=未触发, 1=触发（检测到障碍/平台边缘） */
    uint8_t left;    /* 左红外:   0=未触发, 1=触发（检测到红色桥边缘） */
    uint8_t right;   /* 右红外:   0=未触发, 1=触发（检测到红色桥边缘） */
    uint8_t extra;   /* 备用红外: 0=未触发, 1=触发 */
} ir_sensor_t;

/* ---- API ---- */

/* 读取三路红外传感器，更新内部数据结构。
 * 在 line_track_task 5ms 循环中调用。 */
void ir_sensor_poll(void);

/* 获取只读传感器数据指针。 */
const ir_sensor_t *get_ir_sensor_point(void);

#endif
