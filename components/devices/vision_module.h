/**
 * vision_module.h
 * 视觉模块设备驱动。
 *
 * 旧接口（platform_exec）: bsp_vision UART 中断接收 ASCII 数字。
 * 新接口（vision_exec）: USART6 协议 — MCU 主动发送命令帧。
 *
 * USART6 协议:  命令帧 0xA5 <cmd> 0x5A  /  应答帧 <cmd> <data...>
 */
#ifndef VISION_MODULE_H
#define VISION_MODULE_H

#include "struct_typedef.h"

/* ---- 视觉结果存储 ---- */
#define VISION_MAX_RESULTS 8

typedef struct {
    uint8_t cmd;
    uint8_t data[4];
    uint8_t len;
} vision_stored_t;

extern vision_stored_t g_vision_results[VISION_MAX_RESULTS];
extern uint8_t g_vision_result_count;

/* ---- 旧接口 (platform_exec) ---- */
typedef struct {
    uint8_t number;   /* 0-9 或 0xFF 无效 */
    uint8_t fresh;
    uint8_t online;
} vision_result_t;

void vision_module_init(void);
uint8_t vision_module_poll(void);
const vision_result_t *get_vision_result_point(void);
void vision_module_reset(void);

/* ---- USART6 协议 (vision_exec) ---- */
void vision_module_begin_cmd(void);
void vision_module_send_cmd(uint8_t cmd);
void vision_module_on_usart6_byte(uint8_t byte);
uint8_t vision_module_has_response(void);
uint8_t vision_module_get_response(uint8_t data[4]);
void vision_result_store(uint8_t cmd, const uint8_t *data, uint8_t len);

/* ---- 结果查询 (路由决策用) ---- */
void vision_result_clear(void);
uint8_t vision_result_get_cmd(uint8_t idx);
uint8_t vision_result_get_byte(uint8_t idx, uint8_t n);
uint8_t vision_find_first_green(void);
uint8_t vision_last_tl_green(void);
uint8_t vision_last_tl_green_idx(void);
uint8_t vision_qr_get_target(uint8_t qr_idx, uint8_t n);

/* ---- 可调参数 ---- */
extern uint32_t g_vision_timeout_ms;   /* 视觉应答超时 (ms), 默认 3000 */

#endif
