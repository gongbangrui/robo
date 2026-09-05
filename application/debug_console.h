/**
 * debug_console.h
 * Debug command console for testing and tuning.
 *
 * Provides interactive command interface via USART6/USB CDC for:
 * - Real-time sensor data display (gray sensor, IMU, motors)
 * - Parameter tuning (PID, speed, dead-reckon)
 * - Path selection and control
 * - System status monitoring
 *
 * Usage:
 *   1. Connect to USART6 (PG14, 115200 8N1) or USB CDC
 *   2. Type "help" to see available commands
 *   3. Commands are case-insensitive
 */
#ifndef DEBUG_CONSOLE_H
#define DEBUG_CONSOLE_H

#include "struct_typedef.h"

/* Console input buffer size */
#define DEBUG_CMD_BUF_SIZE  64

/* Console output buffer size */
#define DEBUG_LINE_BUF_SIZE 256

/**
 * @brief  Debug console FreeRTOS task entry.
 *         Receives commands from USART6 and executes them.
 */
extern void debug_console_task(void const *pvParameters);

/**
 * @brief  Send a character to debug console (for printf retarget).
 *         Called from uart_printf or direct putchar.
 */
extern void debug_console_putchar(uint8_t ch);

/**
 * @brief  Send a string to debug console.
 */
extern void debug_console_puts(const char *str);

/**
 * @brief  Send formatted string to debug console (printf-style).
 */
extern void debug_console_printf(const char *fmt, ...);

/**
 * @brief  Check if console has pending command.
 * @return 1 if command ready, 0 otherwise
 */
extern uint8_t debug_console_has_cmd(void);

/**
 * @brief  Get pointer to command buffer.
 * @return Pointer to null-terminated command string
 */
extern const char *debug_console_get_cmd(void);

/**
 * @brief  Clear command buffer.
 */
extern void debug_console_clear_cmd(void);
extern void debug_console_vision_mode(uint8_t enable);

#endif
