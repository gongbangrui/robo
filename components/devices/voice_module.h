/**
 * voice_module.h
 * 语音模块设备驱动 — 5 路 GPIO 并行选曲（00000-11111，32 个音轨）。
 *
 * 硬件无关层。底层 GPIO 由 bsp_voice.h/c 提供。
 * 仅在状态转换时调用，不在 5ms 热路径中。
 *
 * 若模块需要触发脉冲（Latch 信号），在 bsp_voice.c 中实现。
 */
#ifndef VOICE_MODULE_H
#define VOICE_MODULE_H

#include "struct_typedef.h"

/* ---- 预定义音轨 ---- */
typedef enum {
    VOICE_TRACK_PLATFORM_ENTER = 1,   /* "上平台开始" */
    VOICE_TRACK_PLATFORM_DONE  = 2,   /* "上平台完成" */
    VOICE_TRACK_BRIDGE_ENTER   = 3,   /* "过桥开始" */
    VOICE_TRACK_BRIDGE_DONE    = 4,   /* "过桥完成" */
    VOICE_TRACK_NUMBER_BASE    = 10,  /* 数字播报基址: 10 + digit */
} voice_track_t;

/* ---- API ---- */

/* 初始化语音模块（GPIO 输出模式已在 CubeMX 中配置）。 */
void voice_module_init(void);

/* 播放指定音轨。
 * 实现：设置 5 路 GPIO = (track & 0x1F)，可选触发脉冲。 */
void voice_module_play(voice_track_t track);

#endif
