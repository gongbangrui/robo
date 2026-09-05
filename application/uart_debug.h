#ifndef UART_DEBUG_H
#define UART_DEBUG_H

#include "usart.h"

/* Debug UART — TX on PG14 (USART6, 115200 8N1). */
#define DEBUG_UART  huart6

/**
 * @brief  Initialize debug UART interrupt-based RX.
 *         Must be called once before using uart_getchar().
 */
void uart_debug_init(void);

/**
 * @brief  Send formatted string to debug UART.
 */
void uart_printf(const char *fmt, ...);

/**
 * @brief  Send single character to debug UART.
 * @param  ch: character to send
 */
void uart_putchar(uint8_t ch);

/**
 * @brief  Receive single character from debug UART (non-blocking).
 * @return Received character, or 0 if no data available
 */
uint8_t uart_getchar(void);

#endif
