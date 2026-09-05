/**
 * vision_exec.c
 * 视觉执行器 — 红绿灯 / 二维码 / 数字识别。
 * USART6 分时复用：正常为 debug，视觉动作时切为 vision 模式。
 */
#include "vision_exec.h"
#include "nav_types.h"
#include "chassis_task.h"
#include "vision_module.h"
#include "debug_console.h"
#include "cmsis_os.h"

#define VIS_TL_MAX_RETRY     15

typedef enum { VS_REQ, VS_RESP, VS_DONE } vs_t;

typedef struct {
    vs_t     stage;
    uint32_t enter_ms;
    uint8_t  cmd;
    uint8_t  retries;
} vis_priv_t;

static vis_priv_t vst;

static void vis_enter(cross_exec_ctx_t *ctx, navigate_action_t a,
                      const gray_sensor_t *gs, fp32 *vx, fp32 *wz) {
    (void)gs; *vx = 0; *wz = 0;
    switch (a) {
    case TRAFFIC:   vst.cmd = 0x01; break;
    case QRSCAN:         vst.cmd = 0x02; break;
    case DIGIT: vst.cmd = 0x03; break;
    default: vst.cmd = 0; break;
    }
    vst.stage = VS_REQ; vst.enter_ms = osKernelSysTick(); vst.retries = 0;
    ctx->priv = &vst;
}

static cross_exec_result_t vis_exec(cross_exec_ctx_t *ctx, fp32 *vx, fp32 *wz) {
    (void)ctx; *vx = 0; *wz = 0;
    uint32_t now = osKernelSysTick();

    switch (vst.stage) {
    case VS_REQ:
        debug_console_vision_mode(1);
        vision_module_send_cmd(vst.cmd);
        vst.stage = VS_RESP; vst.enter_ms = now;
        break;

    case VS_RESP:
        if (vision_module_has_response()) {
            uint8_t d[4]; uint8_t n = vision_module_get_response(d);
            vision_result_store(vst.cmd, d, n);
            if (vst.cmd == 0x01 && n >= 1 && d[0] == 0) {
                if (++vst.retries >= VIS_TL_MAX_RETRY) vst.stage = VS_DONE;
                else { osDelay(200); vision_module_send_cmd(vst.cmd); vst.enter_ms = now; }
                break;
            }
            vst.stage = VS_DONE;
        } else if (now - vst.enter_ms > g_vision_timeout_ms) {
            return CROSS_EXEC_ABORT;
        }
        break;

    case VS_DONE: return CROSS_EXEC_DONE;
    }
    return CROSS_EXEC_RUNNING;
}

static void vis_exit(cross_exec_ctx_t *ctx) {
    (void)ctx;
    debug_console_vision_mode(0);
}

const cross_executor_t vision_exec = {
    .name = "vision", .enter = vis_enter,
    .exec = vis_exec, .exit = vis_exit, .signal_navigator = 1,
};
