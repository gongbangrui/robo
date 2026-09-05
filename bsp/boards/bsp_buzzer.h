/**
 * bsp_buzzer.h
 * 蜂鸣器 — TIM4 CH3, PD14, PWM 无源蜂鸣器。
 */
#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H
#include "struct_typedef.h"

/* 初始化 (启动 PWM) */
extern void buzzer_init(void);

/* 单音 */
extern void buzzer_on(uint16_t psc, uint16_t pwm);
extern void buzzer_off(void);

/* 旋律: Notes 数组 {(psc, dur_ms), ...}, count=音符数, gap_ms=音符间隔 (阻塞) */
extern void buzzer_play(const uint16_t notes[][2], uint8_t count, uint8_t gap_ms);

/* 预定义音效 */
extern void buzzer_init_done(void);     /* 初始化完成: 升调三连音 */
extern void buzzer_low_battery(void);   /* 低电压: 急促四短鸣 */
extern void buzzer_cali_mag_done(void); /* 磁力计校准完成: 降调五连音 */

/* 音符宏 — psc=(84MHz / (65536*freq))-1, pwm=占空比50% */
#define NOTE_C5  10   /* 523Hz */
#define NOTE_D5  9    /* 587Hz */
#define NOTE_E5  8    /* 659Hz */
#define NOTE_G5  6    /* 784Hz */
#define NOTE_A5  5    /* 880Hz */
#define NOTE_C6  3    /* 1047Hz */

#endif
