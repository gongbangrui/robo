/**
 * platform_exec.c
 * 上平台执行器 — 死推算前进/后退 → 舵机 → 视觉 → 180°掉头 → 退出寻线。
 *
 * param 编码:
 *   低 4 位 = 语音编号
 *   高 4 位 = 模式: 0x00=仅语音 0x10=数字 0x20=QR
 *
 * param == 8: 平台8特判（两次上下）
 *
 * 阶段:
 *   DEAD_RECKON → SERVO → VISION → TURN → EXIT
 */
#include "cross_exec.h"
#include "lt_utils.h"
#include "gray_sensor.h"
#include "ir_sensor.h"
#include "servo_ctrl.h"
#include "voice_module.h"
#include "vision_module.h"
#include "debug_console.h"
#include "INS_task.h"
#include "cmsis_os.h"

/* 可调参数 */
#define PLAT_FWD_MM        250    /* 死推算前进距离 (mm) */
#define PLAT_FWD_SPEED     0.15f  /* 前进速度 (m/s) */
#define PLAT_DRIVE_TIMEOUT_MS 5000 /* 死推算超时 */
#define PLAT_SERVO_HOLD_MS  800   /* 舵机保持时间 */
#define PLAT_VISION_TIMEOUT_MS 3000 /* 视觉超时 */
#define PLAT_TURN_SPEED     2.0f  /* 掉头角速度 (rad/s) */
#define PLAT_EXIT_TIMEOUT_MS 3000 /* 退出寻线超时 */
#define PLAT_EXIT_SPEED     0.10f /* 退出慢速 */
#define PLAT_TURN_FWD_MM    500   /* 掉头后前进距离 (mm) */

#ifndef PI
#define PI 3.14159265358979323846f
#endif

/* 阶段 */
typedef enum {
    STAGE_DEAD_RECKON = 0,
    STAGE_SERVO       = 1,
    STAGE_VISION      = 2,
    STAGE_TURN        = 3,
    STAGE_POST_TURN   = 4,  /* 掉头后死推算前进 */
    STAGE_EXIT        = 5,
} plat_stage_t;

typedef struct {
    plat_stage_t stage;
    uint32_t     enter_ms;
    uint8_t      digit;
    uint8_t      mode;           /* 0=语音 1=数字 2=QR */
    uint8_t      line_was_lost;  /* 丢过线标志 */
    fp32         turn_yaw_start;
    uint8_t      plat8_level;    /* 平台8: 0=first, 1=second */
} plat_priv_t;

static plat_priv_t ps;

static void plat_enter(cross_exec_ctx_t *ctx, navigate_action_t action,
                       const gray_sensor_t *gs, fp32 *vx, fp32 *wz)
{
    (void)action; (void)gs;
    uint8_t p     = ctx->action_param & 0x7F;  /* 去掉 bit7 */
    uint8_t voice = p & 0x0F;
    uint8_t mode  = (p >> 4) & 0x03;

    /* 首次进入才播语音和初始化 */
    if (ps.stage == STAGE_DEAD_RECKON && ps.enter_ms == 0) {
        voice_module_play(voice ? (voice_track_t)voice : VOICE_TRACK_PLATFORM_ENTER);
        ps.mode  = mode;
        ps.digit = 0;
        ps.line_was_lost = 0;
        ps.plat8_level   = (ctx->action_param == 8) ? 0 : 0xFF;
    }

    /* 设置速度 */
    if (ps.stage == STAGE_TURN) {
        *vx = 0.0f;
        *wz = (ps.turn_yaw_start > 0) ? PLAT_TURN_SPEED : -PLAT_TURN_SPEED;
    } else {
        *vx = (ps.stage == STAGE_EXIT) ? PLAT_EXIT_SPEED : PLAT_FWD_SPEED;
        *wz = 0.0f;
    }

    ps.enter_ms = osKernelSysTick();
    ctx->priv = &ps;
}

static cross_exec_result_t plat_exec_fn(cross_exec_ctx_t *ctx, fp32 *vx, fp32 *wz)
{
    (void)ctx;
    const ir_sensor_t *ir = get_ir_sensor_point();
    const gray_sensor_t *gs = get_gray_sensor_point();
    uint32_t now = osKernelSysTick();
    uint32_t elapsed = now - ps.enter_ms;

    switch (ps.stage) {

    /* ── 死推算前进 (找红外或找线) ── */
    case STAGE_DEAD_RECKON:
        *vx = PLAT_FWD_SPEED;
        *wz = 0.0f;

        /* 红外触发 → 舵机 */
        if (!ir->front) {
            ps.stage = STAGE_SERVO;
            ps.enter_ms = now;
            break;
        }

        /* 寻到线 → 退回 TRACKING 循线，等红外再触发 */
        if (is_any_line(gs) && gs->online) {
            ps.plat8_level++;  /* 平台8推进一次 */
            return CROSS_EXEC_RETRACK;
        }

        /* 超时 */
        if (elapsed > PLAT_DRIVE_TIMEOUT_MS) return CROSS_EXEC_ABORT;
        break;

    /* ── 舵机 ── */
    case STAGE_SERVO:
        *vx = 0.0f; *wz = 0.0f;
        if (elapsed == 0) servo_platform_activate();
        if (elapsed > PLAT_SERVO_HOLD_MS) {
            servo_platform_home();
            ps.enter_ms = now;

            if (ps.mode == 0) {
                ps.stage = STAGE_TURN;  /* 仅语音 → 跳过 vision */
            } else if (ps.mode == 2) {
                debug_console_vision_mode(1);
                vision_module_send_cmd(0x02);
                ps.stage = STAGE_VISION;
            } else {
                vision_module_reset();
                ps.stage = STAGE_VISION;
            }
        }
        break;

    /* ── 视觉 ── */
    case STAGE_VISION:
        *vx = 0.0f; *wz = 0.0f;
        if (ps.mode == 0) { ps.stage = STAGE_TURN; break; }

        if (ps.mode == 2) {
            if (vision_module_has_response()) {
                uint8_t d[4]; uint8_t n = vision_module_get_response(d);
                vision_result_store(0x02, d, n);
                debug_console_vision_mode(0);
                ps.stage = STAGE_TURN; ps.enter_ms = now;
            }
        } else {
            vision_module_poll();
            const vision_result_t *vis = get_vision_result_point();
            if (vis->fresh && vis->number <= 9) {
                ps.digit = vis->number;
                ps.stage = STAGE_TURN; ps.enter_ms = now;
            }
        }
        if (elapsed > PLAT_VISION_TIMEOUT_MS) { ps.stage = STAGE_TURN; ps.enter_ms = now; }
        break;

    /* ── 180° 掉头 ── */
    case STAGE_TURN:
        *vx = 0.0f;
        if (elapsed == 0) ps.turn_yaw_start = get_INS_angle_point()[INS_YAW_ADDRESS_OFFSET];
        *wz = PLAT_TURN_SPEED;
        {
            fp32 dy = get_INS_angle_point()[INS_YAW_ADDRESS_OFFSET] - ps.turn_yaw_start;
            if (dy < 0) dy = -dy;
            if (dy > PI) dy = (fp32)(2.0 * PI) - dy;
            if (dy >= (fp32)(PI * 0.85)) {
                *wz = 0.0f; *vx = 0.0f;
                ps.stage = STAGE_POST_TURN; ps.enter_ms = now;
            }
        }
        break;

    /* ── 掉头后死推算前进 ── */
    case STAGE_POST_TURN:
        *vx = PLAT_FWD_SPEED;
        *wz = 0.0f;
        if (elapsed * PLAT_FWD_SPEED * 1000.0f >= (fp32)PLAT_TURN_FWD_MM) {
            *vx = 0.0f; *wz = 0.0f;
            ps.stage = STAGE_EXIT; ps.enter_ms = now;
        }
        break;

    /* ── 退出: 前进寻线 ── */
    case STAGE_EXIT:
        *vx = PLAT_EXIT_SPEED;
        *wz = 0.0f;

        if (is_any_line(gs) && gs->online) {
            /* 平台8: 第二次寻线后退回 TRACKING */
            if (ps.plat8_level < 2) {
                ps.plat8_level++;
                ps.stage = STAGE_DEAD_RECKON;
                return CROSS_EXEC_RETRACK;
            }
            /* 普通平台/平台8最终: 完成 */
            if (ps.mode == 1)
                voice_module_play(VOICE_TRACK_NUMBER_BASE + ps.digit);
            return CROSS_EXEC_DONE;
        }

        /* 丢线后慢速3s超时 */
        ps.line_was_lost = 1;
        if (elapsed > PLAT_EXIT_TIMEOUT_MS) {
            if (ps.mode == 1)
                voice_module_play(VOICE_TRACK_NUMBER_BASE + ps.digit);
            return CROSS_EXEC_DONE;
        }
        break;
    }

    return CROSS_EXEC_RUNNING;
}

static void plat_exit_fn(cross_exec_ctx_t *ctx) { (void)ctx; }

const cross_executor_t platform_exec = {
    .name = "platform", .enter = plat_enter,
    .exec = plat_exec_fn, .exit = plat_exit_fn, .signal_navigator = 1,
};
