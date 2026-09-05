/**
 * usb_cdc_task.h
 * USB CDC FreeRTOS task — self-contained TX/RX via ring buffers + semaphores.
 *
 * Architecture:
 *   [printf/_write] → __io_putchar → tx_fifo → tx_sem → usb_cdc_task
 *                                                           |
 *                                                           v
 *                                                    CDC_Transmit_FS
 *                                                           |
 *                                                           v (ISR)
 *   [scanf/_read]   ← __io_getchar ← rx_fifo ← CDC_Receive_FS → usb_cdc_on_rx()
 *                                                     |
 *                                                     v (ISR)
 *                                              usb_cdc_on_tx_complete()
 *
 * Key design decisions:
 *   - Single USB task drains TX FIFO and waits for hardware completion.
 *   - usb_printf is dual-mode: direct TX before scheduler starts, FIFO after.
 *   - __io_putchar writes byte to FIFO, signals on newline (batched USB transfers).
 *   - RX is non-blocking; __io_getchar returns 0 when empty.
 *   - CDC callbacks in usbd_cdc_if.c are just thunks into this module.
 */
#ifndef USB_CDC_TASK_H
#define USB_CDC_TASK_H

#include <stdint.h>

/**
 * @brief  USB CDC FreeRTOS task entry.
 *         Self-initializes semaphores/FIFOs, then drains TX FIFO.
 *         Create with osPriorityNormal, stack 256.
 */
void usb_cdc_task(void const *argument);

/**
 * @brief  printf-like output over USB CDC. Works before and after scheduler.
 *         Thread-safe: multiple tasks can call concurrently.
 */
void usb_printf(const char *fmt, ...);

/**
 * @brief  Retarget __io_putchar for newlib _write → printf.
 *         Each byte → TX FIFO, signals task on '\n'.
 */
int __io_putchar(int ch);

/**
 * @brief  Retarget __io_getchar for newlib _read → scanf/getchar.
 *         Non-blocking: returns 0 when no data.
 */
int __io_getchar(void);

/* ---- ISR callbacks (called from usbd_cdc_if.c) ---- */

/**
 * @brief  Called from CDC_TransmitCplt_FS (ISR context).
 *         Signals tx_done_sem to unblock the USB task.
 */
void usb_cdc_on_tx_complete(void);

/**
 * @brief  Called from CDC_Receive_FS (ISR context).
 *         Copies received data into RX FIFO.
 */
void usb_cdc_on_rx(uint8_t *data, uint32_t len);

#endif
