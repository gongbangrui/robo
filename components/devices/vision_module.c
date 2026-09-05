/**
 * vision_module.c
 * 视觉模块驱动 — USART6 协议 + 旧 platform 接口。
 */
#include "vision_module.h"
#include "bsp_vision.h"
#include "detect_task.h"
#include "uart_debug.h"
#include <string.h>

/* ---- 旧接口 (platform_exec) ---- */
static vision_result_t vis_data;

void vision_module_on_rx_byte(uint8_t byte) {
  if (byte >= '0' && byte <= '9') {
    vis_data.number = byte - '0';
    vis_data.fresh = 1;
    vis_data.online = 1;
    detect_hook(VISION_MODULE_TOE);
  }
}

void vision_module_init(void) {
  memset(&vis_data, 0, sizeof(vis_data));
  vis_data.number = 0xFF;
  bsp_vision_init();
}

uint8_t vision_module_poll(void) {
  uint8_t byte;
  while (bsp_vision_getchar(&byte))
    vision_module_on_rx_byte(byte);
  return vis_data.fresh;
}

const vision_result_t *get_vision_result_point(void) { return &vis_data; }
void vision_module_reset(void) { vis_data.fresh = 0; }

/* ---- 全局结果存储 ---- */
vision_stored_t g_vision_results[VISION_MAX_RESULTS];

/* ---- 可调参数 ---- */
uint32_t g_vision_timeout_ms = 3000; /* 视觉应答超时, 默认 3s */
uint8_t g_vision_result_count;

void vision_result_store(uint8_t cmd, const uint8_t *data, uint8_t len) {
  if (g_vision_result_count >= VISION_MAX_RESULTS)
    return;
  if (len > 4)
    len = 4;
  g_vision_results[g_vision_result_count].cmd = cmd;
  for (uint8_t i = 0; i < len; i++)
    g_vision_results[g_vision_result_count].data[i] = data[i];
  g_vision_results[g_vision_result_count].len = len;
  g_vision_result_count++;
}

/* ---- USART6 协议 ---- */
static uint8_t vis_rx_buf[16];
static uint8_t vis_rx_idx, vis_rsp_ready, vis_rsp_cmd;
static uint8_t vis_rsp_data[4], vis_rsp_len, vis_active;

void vision_module_begin_cmd(void) {
  vis_rx_idx = 0;
  vis_rsp_ready = 0;
  vis_rsp_len = 0;
  vis_active = 1;
}

void vision_module_send_cmd(uint8_t cmd) {
  vision_module_begin_cmd();
  uint8_t f[3] = {0xA5, cmd, 0x5A};
  for (int i = 0; i < 3; i++)
    uart_putchar(f[i]);
}

void vision_module_on_usart6_byte(uint8_t byte) {
  if (!vis_active)
    return;
  if (vis_rx_idx == 0) {
    vis_rsp_cmd = byte;
    vis_rsp_len = 0;
    vis_rx_idx = 1;
  } else if (vis_rsp_len < 4)
    vis_rsp_data[vis_rsp_len++] = byte;
}

uint8_t vision_module_has_response(void) {
  if (vis_rsp_ready)
    return 1;
  if (vis_rsp_len == 0)
    return 0;
  uint8_t exp = (vis_rsp_cmd == 0x02) ? 3 : 1;
  if (vis_rsp_len >= exp) {
    vis_rsp_ready = 1;
    return 1;
  }
  return 0;
}

uint8_t vision_module_get_response(uint8_t data[4]) {
  if (!vis_rsp_ready)
    return 0;
  for (uint8_t i = 0; i < vis_rsp_len && i < 4; i++)
    data[i] = vis_rsp_data[i];
  vis_rsp_ready = 0;
  return vis_rsp_len;
}

/* ---- 结果查询 (路由决策用) ---- */

void vision_result_clear(void) { g_vision_result_count = 0; }

uint8_t vision_result_get_cmd(uint8_t idx) {
  if (idx >= g_vision_result_count)
    return 0xFF;
  return g_vision_results[idx].cmd;
}

uint8_t vision_result_get_byte(uint8_t idx, uint8_t n) {
  if (idx >= g_vision_result_count || n >= g_vision_results[idx].len)
    return 0xFF;
  return g_vision_results[idx].data[n];
}

uint8_t vision_find_first_green(void) {
  for (uint8_t i = 0; i < g_vision_result_count; i++) {
    if (g_vision_results[i].cmd == 0x01 && g_vision_results[i].len >= 1 &&
        g_vision_results[i].data[0] == 1)
      return i;
  }
  return 0xFF; /* not found */
}

uint8_t vision_qr_get_target(uint8_t qr_idx, uint8_t n) {
  uint8_t cnt = 0;
  for (uint8_t i = 0; i < g_vision_result_count; i++) {
    if (g_vision_results[i].cmd == 0x02) {
      if (cnt == qr_idx)
        return vision_result_get_byte(i, n);
      cnt++;
    }
  }
  return 0xFF;
}

uint8_t vision_last_tl_green(void) {
  for (int i = (int)g_vision_result_count - 1; i >= 0; i--) {
    if (g_vision_results[i].cmd != 0x01)
      continue;
    return (g_vision_results[i].data[0] == 1) ? 1 : 0;
  }
  return 0;
}

uint8_t vision_last_tl_green_idx(void) {
  for (int i = (int)g_vision_result_count - 1; i >= 0; i--) {
    if (g_vision_results[i].cmd != 0x01)
      continue;
    return (g_vision_results[i].data[0] == 1) ? (uint8_t)i : 0xFF;
  }
  return 0xFF;
}
