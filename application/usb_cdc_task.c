/**
 * usb_cdc_task.c
 * USB CDC FreeRTOS task implementation.
 *
 * Data flow:
 *   Producer (any task)          USB CDC Task           USB ISR
 *   ─────────────────            ────────────           ───────
 *   usb_printf()                 wait(tx_sem)           CDC_TransmitCplt_FS
 *     write(fmt_buf, tx_fifo)      read(tx_fifo, chunk)   └─ usb_cdc_on_tx_complete()
 *     signal(tx_sem)               CDC_Transmit_FS(chunk)       └─ signal(tx_done_sem)
 *                                   wait(tx_done_sem)     CDC_Receive_FS
 *                                   ...loop...             └─ usb_cdc_on_rx(data, len)
 *                                                               └─ write(rx_fifo)
 */

#include "usb_cdc_task.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "semphr.h"

#include "fifo.h"
#include "usbd_cdc_if.h"

/* ===== Private state ===== */

static volatile uint8_t cdc_tx_complete = 1;  /* for pre-scheduler direct TX */

/* Ring buffers */
static char     tx_fifo_buf[1024];
static fifo_s_t tx_fifo;
static char     rx_fifo_buf[256];
static fifo_s_t rx_fifo;

/* DMA-safe chunk buffer */
static uint8_t  usb_tx_chunk[APP_TX_DATA_SIZE];

/* Binary semaphores */
static SemaphoreHandle_t tx_sem;
static SemaphoreHandle_t tx_done_sem;

/* ===== Public API ===== */

/**
 * @brief  USB CDC task — self-initializing, drains TX FIFO and sends over USB.
 *
 *         Init phase (runs first):
 *           - Create binary semaphores (tx_sem, tx_done_sem)
 *           - Init ring buffers
 *           - Wait for USB enumeration
 *           - Send boot banner
 *
 *         Loop:
 *           - Block on tx_sem until data arrives
 *           - Drain TX FIFO in chunk-sized transfers
 *           - Block on tx_done_sem for each chunk's hardware completion
 */
void usb_cdc_task(void const *argument)
{
    (void)argument;

    /* ---- Init ---- */
    tx_sem      = xSemaphoreCreateBinary();
    tx_done_sem = xSemaphoreCreateBinary();
    fifo_s_init(&tx_fifo, tx_fifo_buf, sizeof(tx_fifo_buf));
    fifo_s_init(&rx_fifo, rx_fifo_buf, sizeof(rx_fifo_buf));

    /* Wait for USB enumeration (host SET_CONFIGURATION). */
    osDelay(800);

    /* Prime: push boot banner into the FIFO so it follows the normal path. */
    {
        const char *msg = "\r\n=== smart_car boot ===\r\n";
        fifo_s_puts(&tx_fifo, (char *)msg, (int)strlen(msg));
        xSemaphoreGive(tx_sem);
    }

    /* ---- Main loop ---- */
    for (;;) {
        /* Block until a producer signals data. */
        xSemaphoreTake(tx_sem, portMAX_DELAY);

        /* Drain the TX FIFO. */
        while (!fifo_s_isempty(&tx_fifo)) {
            int len = fifo_s_gets(&tx_fifo, (char *)usb_tx_chunk,
                                  (int)sizeof(usb_tx_chunk));
            if (len > 0) {
                /* Wait until USB is ready before sending. */
                uint32_t tout = 500000U;
                while (!cdc_tx_complete && --tout) { __NOP(); }

                cdc_tx_complete = 0;
                if (CDC_Transmit_FS(usb_tx_chunk, (uint16_t)len) == USBD_OK) {
                    /* Wait for hardware TX completion. */
                    xSemaphoreTake(tx_done_sem, portMAX_DELAY);
                }
            }
        }
    }
}

/**
 * @brief  printf over USB CDC.
 *
 *         Dual-mode:
 *         - Before FreeRTOS starts: direct blocking TX (for boot messages).
 *         - After FreeRTOS starts:  push to TX FIFO, wake USB task.
 *
 *         Thread-safe: multiple tasks can call concurrently.
 */
void usb_printf(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    int len;

    va_start(ap, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (len <= 0) return;
    if ((unsigned)len >= sizeof(buf)) len = (int)sizeof(buf) - 1;

    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        /* RTOS mode: push to FIFO, signal the USB task. */
        fifo_s_puts(&tx_fifo, buf, len);
        xSemaphoreGive(tx_sem);
    } else {
        /* Bare-metal boot mode: direct TX with retry and longer timeout. */
        uint32_t tout;
        for (int retry = 0; retry < 10; retry++) {
            tout = 500000U;
            while (!cdc_tx_complete && --tout) { __NOP(); }
            if (cdc_tx_complete) break;
        }
        cdc_tx_complete = 0;
        CDC_Transmit_FS((uint8_t *)buf, (uint16_t)len);
        tout = 500000U;
        while (!cdc_tx_complete && --tout) { __NOP(); }
    }
}

/**
 * @brief  Retarget __io_putchar for printf() via newlib _write().
 *
 *         Bytes go into the TX FIFO one at a time. The USB task
 *         is signalled on '\n' so a full line triggers one transfer.
 *         Before scheduler: direct byte-by-byte TX.
 */
int __io_putchar(int ch)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        fifo_s_put(&tx_fifo, (char)ch);
        if (ch == '\n') {
            xSemaphoreGive(tx_sem);
        }
    } else {
        /* Bare-metal: direct TX. */
        uint8_t byte = (uint8_t)ch;
        uint32_t tout;
        for (int retry = 0; retry < 10; retry++) {
            tout = 500000U;
            while (!cdc_tx_complete && --tout) { __NOP(); }
            if (cdc_tx_complete) break;
        }
        cdc_tx_complete = 0;
        CDC_Transmit_FS(&byte, 1);
        tout = 500000U;
        while (!cdc_tx_complete && --tout) { __NOP(); }
    }
    return (unsigned char)ch;
}

/**
 * @brief  Retarget __io_getchar for scanf() via newlib _read().
 *         Non-blocking: returns 0 when no data is available.
 */
int __io_getchar(void)
{
    if (fifo_s_isempty(&rx_fifo)) {
        return 0;
    }
    return (unsigned char)fifo_s_get(&rx_fifo);
}

/* ===== ISR callbacks (called from usbd_cdc_if.c) ===== */

/**
 * @brief  Called from CDC_TransmitCplt_FS (USB ISR).
 *         Signals the USB task that the hardware TX is done.
 */
void usb_cdc_on_tx_complete(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    cdc_tx_complete = 1;
    if (tx_done_sem != NULL) {
        xSemaphoreGiveFromISR(tx_done_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
 * @brief  Called from CDC_Receive_FS (USB ISR).
 *         Copies received data into the RX FIFO.
 */
void usb_cdc_on_rx(uint8_t *data, uint32_t len)
{
    if (len > 0) {
        fifo_s_puts_noprotect(&rx_fifo, (char *)data, (int)len);
    }
}
