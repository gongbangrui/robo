/**
 * servo_ctrl.c
 * 舵机控制实现 — TIM8 CH1/CH2/CH3 + TIM1 CH4。
 */
#include "servo_ctrl.h"
#include "bsp_servo_pwm.h"
#include "tim.h"    /* htim8, htim1 */

/* 将微秒脉宽转换为定时器比较值。
 * 50Hz PWM: 预分频后 1MHz → 周期 20000 tick = 20ms。
 * pulse_us = 1500 → 比较值 = 1500。 */
static uint16_t us_to_compare(uint16_t pulse_us)
{
    return pulse_us;
}

void servo_ctrl_init(void)
{
    /* 启动 TIM8 三个 PWM 通道 (50Hz) */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);  /* PC6 — 舵机 0 */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);  /* PI6 — 舵机 1 */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);  /* PI7 — 舵机 2 */

    /* 启动 TIM1 CH4 PWM (PE14 — 舵机 3) */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    servo_set_raw(0, 1300);
    servo_set_raw(1, 2500);
    servo_set_raw(3, 500);
}

void servo_set_raw(uint8_t ch, uint16_t pulse_us)
{
    if (ch > 3) return;
    servo_pwm_set(us_to_compare(pulse_us), ch);
}
