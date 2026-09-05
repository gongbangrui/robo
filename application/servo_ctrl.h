/**
 * servo_ctrl.h
 * 舵机控制 — 应用层封装，基于 bsp_servo_pwm.c。
 *
 * 提供命名动作（平台推杆/回位），以及调试用的原始 PWM 设置接口。
 *
 * PWM 频率 50Hz（周期 20000us），典型舵机脉冲范围 500-2500us。
 * 用户需根据实际舵机规格调整 SERVO_PLATFORM_ACTIVE_US / HOME_US。
 */
#ifndef SERVO_CTRL_H
#define SERVO_CTRL_H

#include "struct_typedef.h"

/* ---- 舵机通道分配 ---- */
/* ch0: TIM1 CH4 — PE14  (平台舵机2)
 * ch1: TIM8 CH1 — PC6   (平台舵机1)
 * ch2: TIM8 CH2 — PI6
 * ch3: TIM8 CH3 — PI7 */
#define SERVO_CH_PLATFORM   1
#define SERVO_CH_PLATFORM2  0

/* ---- 脉冲宽度配置 (us) — 用户按实际舵机规格调整 ---- */
#define SERVO_PLATFORM_HOME_US   800   /* 回位脉冲 (us) */
#define SERVO_PLATFORM_ACTIVE_US 800   /* 推杆脉冲 (us) */
#define SERVO_PLATFORM2_HOME_US   1500
#define SERVO_PLATFORM2_ACTIVE_US 2000

/* ---- API ---- */

/* 初始化舵机（启动 PWM）。 */
void servo_ctrl_init(void);


/* 调试用：设置指定通道的原始脉宽 (us)。
 * ch: 通道号 (0-3)
 * pulse_us: 脉宽微秒 (典型 500-2500) */
void servo_set_raw(uint8_t ch, uint16_t pulse_us);

#endif
