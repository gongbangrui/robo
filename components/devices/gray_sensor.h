/**
 * gray_sensor.h
 * 16-channel 5th-gen grayscale sensor — UART request-response driver.
 *
 * Protocol (全输出 mode, 41-byte frame):
 *   Host sends:  0x57 + ID
 *   Sensor responds: 0x75 + 37 data bytes + 0x26 (tail)
 *
 * DMA idle-line reception on USART1.  Call gray_sensor_poll() from
 * line_track_task (5 ms) — it sends the query blocks ~4 ms for the
 * response and returns 1 when fresh data is available.
 */
#ifndef GRAY_SENSOR_H
#define GRAY_SENSOR_H

#include "struct_typedef.h"

#define GRAY_CHANNELS     16
#define GRAY_FRAME_LEN    41        /* 全输出 frame = header(1) + data(37) + tail(1) + 2 reserved */
#define GRAY_RX_BUF_SIZE  64        /* DMA circular buffer (power of 2, > GRAY_FRAME_LEN) */
#define GRAY_OFFSET_SCALE 4000.0f   /* normalize offset_raw → position [-1, +1] */

typedef struct {
    uint16_t bitmask;         /* digital bitmap: bit i = channel i+1 on line */
    uint8_t  line_count;      /* channels seeing the line (0-16) */
    uint8_t  out_line;        /* 0=left-out, 1=right-out, 2=no-line */
    int16_t  offset_raw;      /* raw offset (negative=left, positive=right) */
    fp32     position;        /* normalized: -1.0=left … 0=center … +1.0=right */
    uint8_t  online;          /* last poll succeeded */
    uint8_t  frame_ready;     /* consumer clears after reading */
} gray_sensor_t;

/**
 * @brief  Initialize gray sensor DMA RX and start reception.
 * @param  id: sensor ID (0x01 default)
 */
extern void gray_sensor_init(uint8_t id);

/**
 * @brief  Send query, wait for response, parse frame.
 *         Blocks ~4 ms (UART TX + sensor response).
 * @retval 1 = fresh frame decoded; 0 = timeout / invalid frame.
 */
extern uint8_t gray_sensor_poll(void);

/**
 * @brief  Returns pointer to latest decoded sensor data (read-only).
 */
extern const gray_sensor_t *get_gray_sensor_point(void);

#endif
