#include "uart_debug.h"
#include "bsp_vision.h"
#include "vision_module.h"

extern volatile uint8_t g_vision_mode;

#include <stdio.h>
#include <stdarg.h>

/* ---- Interrupt-based RX ring buffer ---- */
#define UART_RX_BUF_SIZE 64
#define UART_RX_BUF_MASK (UART_RX_BUF_SIZE - 1)

static uint8_t  rx_ring[UART_RX_BUF_SIZE];
static volatile uint8_t rx_head = 0;   /* ISR writes here */
static volatile uint8_t rx_tail = 0;   /* main reads from here */
static uint8_t  rx_byte;               /* single-byte buffer for HAL receive_IT */

void uart_debug_init(void)
{
    rx_head = 0;
    rx_tail = 0;
    /* Arm single-byte interrupt reception */
    HAL_UART_Receive_IT(&DEBUG_UART, &rx_byte, 1);
}

/* ---- HAL callback: called from USARTx_IRQHandler context ---- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* 调试串口 (USART6) */
    if (huart == &DEBUG_UART) {
        if (g_vision_mode) {
            vision_module_on_usart6_byte(rx_byte);
        } else {
            uint8_t next = (rx_head + 1) & UART_RX_BUF_MASK;
            if (next != rx_tail) { rx_ring[rx_head] = rx_byte; rx_head = next; }
        }
        HAL_UART_Receive_IT(huart, &rx_byte, 1);
    }

    /* 视觉模块 UART */
    bsp_vision_uart_isr(huart);
}

/* ---- TX: polling (unchanged) ---- */

void uart_printf(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    int len;

    va_start(ap, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (len <= 0) return;
    if ((unsigned)len >= sizeof(buf)) len = (int)sizeof(buf) - 1;

    HAL_UART_Transmit(&DEBUG_UART, (uint8_t *)buf, (uint16_t)len, 100);
}

void uart_putchar(uint8_t ch)
{
    HAL_UART_Transmit(&DEBUG_UART, &ch, 1, 10);
}

/* ---- RX: read from ring buffer ---- */

uint8_t uart_getchar(void)
{
    if (rx_head == rx_tail) {
        return 0;   /* empty */
    }
    uint8_t ch = rx_ring[rx_tail];
    rx_tail = (rx_tail + 1) & UART_RX_BUF_MASK;
    return ch;
}
