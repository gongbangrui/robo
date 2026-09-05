/**
 * bsp_servo_pwm.c
 * 舵机 PWM 输出 (50Hz)。
 *
 * 通道映射:
 *   ch0: TIM1 CH4 — PE14  (新)
 *   ch1: TIM8 CH1 — PC6
 *   ch2: TIM8 CH2 — PI6
 *   ch3: TIM8 CH3 — PI7
 */
#include "bsp_servo_pwm.h"
#include "tim.h"

void servo_pwm_set(uint16_t pwm, uint8_t i)
{
    switch (i) {
    case 0:
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, pwm);
        break;
    case 1:
        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwm);
        break;
    case 2:
        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwm);
        break;
    case 3:
        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, pwm);
        break;
    default:
        break;
    }
}
