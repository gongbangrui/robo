/**
 * voice_module.c
 * 语音模块设备驱动实现 — 5 路 GPIO 并行输出选曲。
 */
#include "voice_module.h"
#include "bsp_voice.h"
#include "cmsis_os.h"

void voice_module_init(void)
{
    /* 低电平有效编码: 上电默认全高 (无选曲) */
    bsp_voice_write(0x00);
}

void voice_module_play(voice_track_t track)
{
    /* 设置 D0-D4 = track 的低 5 位 (低电平有效) */
    bsp_voice_write((uint8_t)track & 0x1F);
    osDelay(100);
    /* 恢复默认高电平 */
    bsp_voice_write(0x00);
}
