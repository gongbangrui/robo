/**
 * move_exec.c
 * 指定距离移动 — param = cm (正=前进, 负=倒退). 时间×速度积分.
 */
#include "move_exec.h"
#include "nav_types.h"
#include "chassis_task.h"
#include "cmsis_os.h"

#define MV_SPEED 0.15f

typedef struct {
    uint32_t enter_ms;
    fp32     target_m;
    fp32     sign;
} mv_priv_t;

static mv_priv_t mv_st;

static void mv_enter(cross_exec_ctx_t *ctx, navigate_action_t a,
                     const gray_sensor_t *gs, fp32 *vx, fp32 *wz)
{
    (void)gs; (void)a;
    int8_t cm = (int8_t)ctx->action_param;
    mv_st.target_m = (fp32)(cm < 0 ? -cm : cm) * 0.01f;
    mv_st.sign     = (cm >= 0) ? 1.0f : -1.0f;
    mv_st.enter_ms = osKernelSysTick();

    *vx = MV_SPEED * mv_st.sign;
    *wz = 0.0f;
    ctx->priv = &mv_st;
}

static cross_exec_result_t mv_exec_fn(cross_exec_ctx_t *ctx, fp32 *vx, fp32 *wz)
{
    (void)ctx;
    *vx = MV_SPEED * mv_st.sign;
    *wz = 0.0f;

    fp32 elapsed = (fp32)(osKernelSysTick() - mv_st.enter_ms) * 0.001f;
    if (elapsed * MV_SPEED >= mv_st.target_m) {
        *vx = 0.0f;
        return CROSS_EXEC_DONE;
    }
    return CROSS_EXEC_RUNNING;
}

static void mv_exit(cross_exec_ctx_t *ctx) { (void)ctx; }

const cross_executor_t move_exec = {
    .name = "move", .enter = mv_enter,
    .exec = mv_exec_fn, .exit = mv_exit, .signal_navigator = 1,
};
