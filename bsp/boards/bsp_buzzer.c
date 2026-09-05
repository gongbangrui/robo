/**
 * bsp_buzzer.c
 * 蜂鸣器 PWM 驱动 + 旋律播放 (TIM4 CH3, PD14)。
 * TIM4 prescaler=0 → 计数器 ~1.3kHz, 0%占空比时人耳听不到。
 */
#include "bsp_buzzer.h"
#include "tim.h"
#include "cmsis_os.h"

#define PWM_VOLUME  32767  /* 50% 占空比 */

void buzzer_init(void)
{
    /* 启动 PWM，CCR=0 时占空比 0%，无声 */
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}

void buzzer_on(uint16_t psc, uint16_t pwm)
{
    __HAL_TIM_PRESCALER(&htim4, psc);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);
}

void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}

void buzzer_play(const uint16_t notes[][2], uint8_t count, uint8_t gap_ms)
{
    for (uint8_t i = 0; i < count; i++) {
        buzzer_on(notes[i][0], PWM_VOLUME);
        osDelay(notes[i][1]);
        buzzer_off();
        if (gap_ms > 0 && i < count - 1)
            osDelay(gap_ms);
    }
}

/* 初始化完成: C5→E5→G5→C6 升调 */
void buzzer_init_done(void)
{
    const uint16_t notes[][2] = {
        {NOTE_C5, 100}, {NOTE_E5, 100}, {NOTE_G5, 100}, {NOTE_C6, 200},
    };
    buzzer_play(notes, 4, 50);
}

/* 低电压报警: G5 急促四短鸣 */
void buzzer_low_battery(void)
{
    const uint16_t notes[][2] = {
        {NOTE_G5, 80}, {NOTE_G5, 80}, {NOTE_G5, 80}, {NOTE_G5, 120},
    };
    buzzer_play(notes, 4, 60);
}

/* 磁力计校准完成: C6→A5→G5→E5→C5 降调五连音 */
void buzzer_cali_mag_done(void)
{
    const uint16_t notes[][2] = {
        {NOTE_C6, 80}, {NOTE_A5, 80}, {NOTE_G5, 80}, {NOTE_E5, 80}, {NOTE_C5, 150},
    };
    buzzer_play(notes, 5, 40);
}
