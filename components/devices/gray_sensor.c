/**
 * gray_sensor.c
 * 16-channel 5th-gen grayscale sensor — UART request-response driver.
 *
 * DMA idle-line reception on USART1 (DMA2_Stream5, channel 4).
 * One-time config: sends "4C ID 01 03" to set 全输出 (all-output) mode.
 *
 * Protocol:
 *   Host query:    0x57  ID          (2 bytes)
 *   Sensor reply:  0x75  Data[37]  Tail(0x26)   (39-41 bytes, idle-delimited)
 */
#include "gray_sensor.h"
#include "usart.h"
#include "cmsis_os.h"
#include "string.h"
#include "detect_task.h"

/* DMA handle is global in usart.c (CubeMX-generated). */
extern DMA_HandleTypeDef hdma_usart1_rx;

/* ---- RX buffer and synchronization ---- */
static uint8_t         rx_buf[GRAY_RX_BUF_SIZE];
static volatile uint8_t rx_frame_ready;   /* ISR sets → 1 when valid frame decoded */
static volatile uint8_t rx_frame_valid;   /* ISR sets → 1=valid, 0=bad frame */
static uint8_t         sensor_id = 0x01;

/* ---- sensor data ---- */
static gray_sensor_t sensor_data;

/* ---- internal prototypes ---- */
static void parse_frame(const uint8_t *data, uint16_t len);
static void sensor_config(void);

/* ================================================================
 * HAL callback: called from HAL_UART_IRQHandler in ISR context
 * when USART1 idle-line is detected after DMA reception.
 * ================================================================ */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart != &huart1) return;

    /* Minimum frame: 0x75 + 5 data bytes + 0x26 = 7 bytes. */
    if (Size < 7) {
        /* Echo-only (0x57 ID).  DMA is circular — it keeps running.
         * Do NOT set rx_frame_ready; poll() will keep waiting for
         * the response that follows after the sensor's processing gap. */
        return;
    }

    /* Search for 0x75 header.  The sensor echoes the query (0x57 ID)
     * before the response, so the real header is near the start.
     * Problem: 0x75 can appear as an analog data byte inside the frame.
     * Fix: validate that the remaining bytes from the candidate header
     * form a full frame (≥39 bytes) with a valid tail at the expected
     * position.  A false 0x75 deep in the payload won't have enough
     * bytes behind it. */
    uint16_t hdr = 0;
    while (hdr < Size - 6 && rx_buf[hdr] != 0x75)
        hdr++;
    /* Remaining bytes from hdr must be ≥ 39 (header + 37 data + tail). */
    if (hdr > Size - 39) {
        rx_frame_valid = 0;
        rx_frame_ready = 1;
        return;
    }

    /* Validate tail byte at the expected position (header + 38). */
    if (rx_buf[hdr + 38] != 0x26) {
        rx_frame_valid = 0;
        rx_frame_ready = 1;
        return;
    }

    parse_frame(rx_buf + hdr, 39);
    rx_frame_valid = 1;
    rx_frame_ready = 1;
}

/* ---- frame parser ---- */
static void parse_frame(const uint8_t *data, uint16_t len)
{
    (void)len;

    /* Digital bitmap */
    sensor_data.bitmask = (uint16_t)data[1] | ((uint16_t)data[2] << 8);

    /* Position info byte (Data2) */
    uint8_t info       = data[3];
    uint8_t line_cnt   = info & 0x1F;
    uint8_t sign       = (info >> 5) & 0x01;
    uint8_t out_line_bit = (info >> 6) & 0x01;

    sensor_data.line_count = line_cnt;
    if (line_cnt == 0) {
        sensor_data.out_line = 2;   /* no line */
    } else {
        sensor_data.out_line = out_line_bit;  /* 0=left-out, 1=right-out */
    }

    /* Offset (Data3 high, Data4 low) */
    uint16_t offset_mag = ((uint16_t)data[4] << 8) | data[5];
    sensor_data.offset_raw = sign ? (int16_t)offset_mag : -(int16_t)offset_mag;

    /* Normalize position: -1 (left) … 0 (center) … +1 (right) */
    sensor_data.position = (fp32)sensor_data.offset_raw / GRAY_OFFSET_SCALE;
    if (sensor_data.position > 1.0f)
        sensor_data.position = 1.0f;
    if (sensor_data.position < -1.0f)
        sensor_data.position = -1.0f;

    sensor_data.online = 1;
}

/* ---- one-time sensor config: set 全输出 mode ---- */
static void sensor_config(void)
{
    /* Send: 0x4C ID 0x01 0x03  →  set output type = 全输出 (digital+offset+analog) */
    uint8_t cmd[] = { 0x4C, sensor_id, 0x01, 0x03 };
    HAL_UART_Transmit(&huart1, cmd, sizeof(cmd), 10);
    /* Sensor does not ACK config commands.  Short delay to let it apply. */
    osDelay(10);
}

/* ---- public API ---- */

void gray_sensor_init(uint8_t id)
{
    sensor_id = id;
    memset(&sensor_data, 0, sizeof(sensor_data));
    memset(rx_buf, 0, sizeof(rx_buf));
    rx_frame_ready = 0;
    rx_frame_valid = 0;

    /* DMA2_Stream5 is initialized by CubeMX in HAL_UART_MspInit().
     * Just start idle-line DMA reception on the global hdma_usart1_rx. */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buf, GRAY_RX_BUF_SIZE);

    /* Disable half-transfer interrupt (only need idle-line). */
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

    /* Configure sensor for 全输出 mode. */
    sensor_config();
}

/**
 * gray_sensor_poll — send query, wait for response, parse result.
 *
 * Call from line_track_task at 5 ms interval.
 * Blocks ~5 ms worst case (TX + sensor response at 115200).
 *
 * Returns 1 when a fresh valid frame was decoded (read via get_gray_sensor_point).
 * Returns 0 on timeout (sensor offline, wiring, or bad frame).
 */
uint8_t gray_sensor_poll(void)
{
    /* Clear stale flag. */
    rx_frame_ready = 0;
    rx_frame_valid = 0;

    /* Send query: 0x57 + ID.  Blocking TX — 2 bytes at 115200 ≈ 0.17 ms. */
    uint8_t query[] = { 0x57, sensor_id };
    if (HAL_UART_Transmit(&huart1, query, sizeof(query), 10) != HAL_OK) {
        sensor_data.online = 0;
        return 0;
    }

    /* Wait for idle-line ISR to parse the response.
     * Sensor replies in ~3.6 ms at 115200.  Use short yields so other
     * higher-priority tasks (INS, chassis) can run. */
    uint32_t start = osKernelSysTick();
    while (!rx_frame_ready) {
        if (osKernelSysTick() - start > 6) {   /* 6 ms timeout */
            sensor_data.online = 0;
            return 0;
        }
        osDelay(1);
    }

    if (!rx_frame_valid) {
        sensor_data.online = 0;
        return 0;
    }

    detect_hook(GRAY_SENSOR_TOE);
    sensor_data.frame_ready = 1;
    return 1;
}

const gray_sensor_t *get_gray_sensor_point(void)
{
    return &sensor_data;
}
