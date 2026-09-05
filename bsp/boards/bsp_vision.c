/**
 * bsp_vision.c
 * 视觉模块 BSP 实现 — 标准 UART 中断接收 + 环形缓冲区。
 *
 * 参考: application/uart_debug.c 的实现模式。
 * 使用 USART3 (DBUS 接口)，需外部反向器恢复信号极性。
 */
#include "bsp_vision.h"
#include <string.h>

/* ---- 环形缓冲区 ---- */
static uint8_t  rx_ring[VISION_RX_BUF_SIZE];
static volatile uint8_t rx_head = 0;
static uint8_t  rx_tail = 0;
static volatile uint8_t rx_byte;   /* HAL_UART_Receive_IT 单字节缓冲 */

void bsp_vision_init(void)
{
    memset(rx_ring, 0, sizeof(rx_ring));
    rx_head = 0;
    rx_tail = 0;

    /* 启动单字节中断接收 */
    HAL_UART_Receive_IT(&VISION_UART, (uint8_t *)&rx_byte, 1);
}

uint8_t bsp_vision_getchar(uint8_t *byte)
{
    if (rx_head == rx_tail) {
        return 0;
    }
    *byte = rx_ring[rx_tail];
    rx_tail = (rx_tail + 1) & (VISION_RX_BUF_SIZE - 1);
    return 1;
}

/* ---- UART 中断处理（由 HAL_UART_RxCpltCallback 统一分发） ---- */

void bsp_vision_uart_isr(UART_HandleTypeDef *huart)
{
    if (huart != &VISION_UART) return;

    uint8_t next_head = (rx_head + 1) & (VISION_RX_BUF_SIZE - 1);
    if (next_head != rx_tail) {
        rx_ring[rx_head] = rx_byte;
        rx_head = next_head;
    }
    /* 重新启动单字节接收 */
    HAL_UART_Receive_IT(&VISION_UART, (uint8_t *)&rx_byte, 1);
}
