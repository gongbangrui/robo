/**
 * bsp_voice.c
 * 语音模块 BSP 实现 — 5 路 GPIO 并行输出, 低电平有效编码。
 */
#include "bsp_voice.h"
#include "main.h"

void bsp_voice_write(uint8_t data)
{
    /* 低电平有效: bit=1 → RESET, bit=0 → SET */
    HAL_GPIO_WritePin(VOICE_D0_PORT, VOICE_D0_PIN,
                      (data & 0x01) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(VOICE_D1_PORT, VOICE_D1_PIN,
                      (data & 0x02) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(VOICE_D2_PORT, VOICE_D2_PIN,
                      (data & 0x04) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(VOICE_D3_PORT, VOICE_D3_PIN,
                      (data & 0x08) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(VOICE_D4_PORT, VOICE_D4_PIN,
                      (data & 0x10) ? GPIO_PIN_RESET : GPIO_PIN_SET);

#ifdef VOICE_TRIG_PIN
    HAL_GPIO_WritePin(VOICE_TRIG_PORT, VOICE_TRIG_PIN, GPIO_PIN_SET);
    for (volatile int i = 0; i < 10; i++) { __NOP(); }
    HAL_GPIO_WritePin(VOICE_TRIG_PORT, VOICE_TRIG_PIN, GPIO_PIN_RESET);
#endif
}
