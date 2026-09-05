/**
 * bsp_voice.h
 * 语音模块 BSP 层 — 5 路 GPIO 并行输出。
 *
 * CubeMX 配置: GPIOB PB12-PB15 + GPIOF PF0, 推挽输出, 上拉。
 * D0=PB12, D1=PB13, D2=PB14, D3=PB15, D4=PF0
 * 输出 5bit 二进制值选曲 (00000-11111, 共 32 轨)，低电平有效。
 */
#ifndef BSP_VOICE_H
#define BSP_VOICE_H

#include "struct_typedef.h"

#define VOICE_D0_PORT   GPIOB
#define VOICE_D0_PIN    GPIO_PIN_12
#define VOICE_D1_PORT   GPIOB
#define VOICE_D1_PIN    GPIO_PIN_13
#define VOICE_D2_PORT   GPIOB
#define VOICE_D2_PIN    GPIO_PIN_14
#define VOICE_D3_PORT   GPIOB
#define VOICE_D3_PIN    GPIO_PIN_15
#define VOICE_D4_PORT   GPIOF
#define VOICE_D4_PIN    GPIO_PIN_0

/* 触发脉冲引脚（可选） */
/* #define VOICE_TRIG_PORT  GPIOX */
/* #define VOICE_TRIG_PIN   GPIO_PIN_X */

void bsp_voice_write(uint8_t data);

#endif
