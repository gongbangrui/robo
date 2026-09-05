/**
 * led_flow_task.c — RGB LED status indication.
 *
 * Phases:
 *   BOOT   (0–5 s)  rainbow colour cycle
 *   NORMAL           green slow breathing pulse
 *   ERROR            colour-coded blink by error type
 *
 * Error colour schema (all motors equal — any offline = no-go):
 *   Red fast    — any chassis motor offline
 *   Yellow      — IMU sensor error (gyro / accel / mag)
 *   Blue        — gray sensor offline
 *   White       — multiple error sources at once
 *
 * Hardware: TIM5 CH1=Blue CH2=Green CH3=Red, driven by aRGB_led_show(0xAARRGGBB).
 */
#include "led_flow_task.h"
#include "bsp_led.h"
#include "detect_task.h"
#include "cmsis_os.h"

/* ---- timing (ms) ---- */
#define LED_TICK_MS          20
#define BOOT_DURATION_MS     5000
#define BOOT_COLOR_CYCLE_MS  300

#define NORMAL_CYCLE_MS      2000

#define ERR_ON_FAST          120
#define ERR_OFF_FAST         180
#define ERR_ON_NORMAL        250
#define ERR_OFF_NORMAL        350

/* ---- palette (0xAARRGGBB) ---- */
#define C_BLACK    0xFF000000u
#define C_RED      0xFFFF0000u
#define C_GREEN    0xFF00FF00u
#define C_BLUE     0xFF0000FFu
#define C_YELLOW   0xFFFFFF00u
#define C_WHITE    0xFFFFFFFFu

/* ---- helpers ---- */

static uint32_t mkcolor(uint8_t r, uint8_t g, uint8_t b)
{
    return 0xFF000000u | ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

/* 0 → 1 → 0 over period_ms */
static uint8_t breathe(uint32_t phase_ms, uint32_t period_ms)
{
    uint32_t p = phase_ms % period_ms;
    uint32_t half = period_ms / 2;
    if (p < half)
        return (uint8_t)((uint32_t)255 * p / half);
    else
        return (uint8_t)((uint32_t)255 * (period_ms - p) / half);
}

/* ---- error detection ---- */

/* Returns bitmask:
 *   bit 0 — any chassis motor
 *   bit 1 — IMU (gyro/accel/mag)
 *   bit 2 — gray sensor
 */
static uint8_t led_error_mask(void)
{
    uint8_t m = 0;

    if (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE)
     || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE))
        m |= 1;

    if (toe_is_error(BOARD_GYRO_TOE) || toe_is_error(BOARD_ACCEL_TOE)
     || toe_is_error(BOARD_MAG_TOE)     || toe_is_error(RM_IMU_TOE))
        m |= 2;

    if (toe_is_error(GRAY_SENSOR_TOE))
        m |= 4;

    return m;
}

/* ---- task ---- */

void led_RGB_flow_task(void const *argument)
{
    (void)argument;

    static const uint32_t boot_palette[] = {
        0xFF000000u, 0xFFFF0000u, 0xFF00FF00u, 0xFF0000FFu,
        0xFFFFFFFFu, 0xFFFFFF00u, 0xFFFF00FFu, 0xFF00FFFFu,
    };
    enum { BOOT_COUNT = sizeof(boot_palette) / sizeof(boot_palette[0]) };

    enum { PHASE_BOOT, PHASE_NORMAL, PHASE_ERROR } phase = PHASE_BOOT;
    uint32_t tick_ms = 0;

    for (;;) {
        uint8_t err = led_error_mask();

        if (phase == PHASE_BOOT && tick_ms > BOOT_DURATION_MS)
            phase = PHASE_NORMAL;
        if (err && phase != PHASE_BOOT)
            phase = PHASE_ERROR;
        else if (!err && phase == PHASE_ERROR)
            phase = PHASE_NORMAL;

        uint32_t color = C_BLACK;

        switch (phase) {

        case PHASE_BOOT:
            color = boot_palette[(tick_ms / BOOT_COLOR_CYCLE_MS) % BOOT_COUNT];
            break;

        case PHASE_NORMAL:
            color = mkcolor(0, breathe(tick_ms, NORMAL_CYCLE_MS), 0);
            break;

        case PHASE_ERROR: {
            uint32_t on = ERR_ON_NORMAL, off = ERR_OFF_NORMAL;

            /* Pick colour: white if multiple, otherwise specific */
            if (err == 1)      { color = C_RED;    on = ERR_ON_FAST; off = ERR_OFF_FAST; }
            else if (err == 2) { color = C_YELLOW; }
            else if (err == 4) { color = C_BLUE;   }
            else               { color = C_WHITE;  }  /* mixed */

            uint32_t period = on + off;
            if ((tick_ms % period) >= on)
                color = C_BLACK;
            break;
        }
        }

        aRGB_led_show(color);
        tick_ms += LED_TICK_MS;
        osDelay(LED_TICK_MS);
    }
}
