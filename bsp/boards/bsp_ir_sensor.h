/**
 * bsp_ir_sensor.h
 * 红外传感器 BSP 层 — GPIO 输入，3 路。
 *
 * CubeMX 配置: GPIOE, 输入模式, 无上下拉。
 * IR1=PE9 (前), IR2=PE11 (左), IR3=PE13 (右)
 */
#ifndef BSP_IR_SENSOR_H
#define BSP_IR_SENSOR_H

#include "struct_typedef.h"

/* 前红外 — 检测平台边缘/障碍物 */
#define IR_FRONT_PORT   GPIOE
#define IR_FRONT_PIN    GPIO_PIN_9

/* 左红外 — 检测桥左侧红色边缘 */
#define IR_LEFT_PORT    GPIOE
#define IR_LEFT_PIN     GPIO_PIN_11

/* 右红外 — 检测桥右侧红色边缘 */
#define IR_RIGHT_PORT   GPIOE
#define IR_RIGHT_PIN    GPIO_PIN_13

uint8_t bsp_ir_front_read(void);
uint8_t bsp_ir_left_read(void);
uint8_t bsp_ir_right_read(void);

#endif
