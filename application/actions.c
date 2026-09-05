/**
 * actions.c
 * 高层动作实现 — 阻塞式函数调用，每个动作自包含 while(sensor/clock) 逻辑。
 *
 * 无线动作 (move/turn/find_line/wait_ir) 期间暂停线跟踪任务底盘输出。
 * 有线动作 (cross) 通过信号量等线跟踪任务完成 CROSS 状态后返回。
 */
#include "actions.h"
#include "INS_task.h"
#include "chassis_task.h"
#include "cmsis_os.h"
#include "gray_sensor.h"
#include "ir_sensor.h"
#include "line_track_task.h"
#include "lt_utils.h"
#include "navigate_task.h"
#include "servo_ctrl.h"
#include "vision_module.h"
#include "voice_module.h"
#include "debug_console.h"

#ifndef PI
#define PI 3.14159265358979323846f
#endif

static SemaphoreHandle_t s_cross_done_sem = NULL;
static uint8_t g_actions_active = 0;
volatile uint8_t g_mission_active = 0;

static inline uint32_t now_ms(void) { return osKernelSysTick(); }
static inline uint8_t elapsed(uint32_t t0, uint32_t limit) {
  return (now_ms() - t0) > limit;
}

/* ---- 底盘控制权 (供 line_track_task 查询) ---- */

uint8_t actions_is_active(void) { return g_actions_active; }

void actions_signal_cross_done(void) {
  if (s_cross_done_sem)
    xSemaphoreGive(s_cross_done_sem);
}

void actions_force_stop(void) {
  g_mission_active = 0;
  navigate_stop();
  line_track_got_exit_event(); /* 清残留 exit_event */
  line_track_set_nav_action(STOP);
  actions_signal_cross_done(); /* 唤醒可能卡住的 cross() */
}

static void take_control(void) { g_actions_active = 1; }
static void release_control(void) { g_actions_active = 0; }

/* ---- 初始化 ---- */

void actions_init(void) {
  s_cross_done_sem = xSemaphoreCreateBinary();
  servo_ctrl_init();
  voice_module_init();
}

/* ================================================================
 * ① 有线动作 — 需要路口，阻塞
 * ================================================================ */

void cross(navigate_action_t action) {
  line_track_set_nav_action(action);
  xSemaphoreTake(s_cross_done_sem, 0); /* 清掉残留信号 */
  xSemaphoreTake(s_cross_done_sem, portMAX_DELAY);
  line_track_got_exit_event(); /* 取走 exit_event, 防 navigate_task
                                  用残留路径覆盖下一动作 */
}

/* ================================================================
 * ② 无线动作 — 无需路口，阻塞
 * ================================================================ */

void move(int16_t mm, float spd, uint32_t timeout_ms) {
  float dist = (float)(mm > 0 ? mm : -mm) * 0.001f;
  float dir = mm > 0 ? 1.0f : -1.0f;
  uint32_t t0 = now_ms();
  float traveled = 0.0f;

  take_control();
  while (traveled < dist && !elapsed(t0, timeout_ms)) {
    chassis_set_velocity(spd * dir, 0.0f);
    osDelay(5);
    traveled += spd * 0.005f;
  }
  chassis_set_velocity(0.0f, 0.0f);
  release_control();
}

void turn(int16_t deg) {
  float target = DEG2RAD((float)(deg > 0 ? deg : -deg));
  float wz = deg > 0 ? 2.0f : -2.0f;
  float yaw_prev = get_INS_angle_point()[INS_YAW_ADDRESS_OFFSET];
  float accumulated = 0.0f;

  take_control();
  for (;;) {
    float yaw_now = get_INS_angle_point()[INS_YAW_ADDRESS_OFFSET];
    float delta = yaw_now - yaw_prev;
    if (delta > PI)
      delta -= 2.0f * PI;
    else if (delta < -PI)
      delta += 2.0f * PI;
    accumulated += delta;
    yaw_prev = yaw_now;

    if (accumulated >= target || accumulated <= -target)
      break;

    chassis_set_velocity(0.0f, wz);
    osDelay(5);
  }
  chassis_set_velocity(0.0f, 0.0f);
  release_control();
}

int find_line(uint32_t timeout_ms) {
  uint32_t t0 = now_ms();

  take_control();
  while (!elapsed(t0, timeout_ms)) {
    gray_sensor_poll();
    const gray_sensor_t *gs = get_gray_sensor_point();
    if (is_any_line(gs) && gs->online) {
      release_control();
      return 0;
    }
    chassis_set_velocity(0.50f, 0.0f);
    osDelay(5);
  }
  chassis_set_velocity(0.0f, 0.0f);
  release_control();
  return -1;
}

void wait_ir(uint32_t timeout_ms) {
  uint32_t t0 = now_ms();

  take_control();
  while (!elapsed(t0, timeout_ms)) {
    if (!g_mission_active)
      break;
    ir_sensor_poll();
    if (!get_ir_sensor_point()->front)
      break;
    chassis_set_velocity(0.10f, 0.0f);
    osDelay(5);
  }
  chassis_set_velocity(0.0f, 0.0f);
  release_control();
}

int wait_ir_trigger(uint32_t timeout_ms, uint32_t stable_ms) {
  uint32_t t0 = now_ms();
  uint8_t saw_ir = 0;

  take_control();
  while (!elapsed(t0, timeout_ms)) {
    if (!g_mission_active) {
      release_control();
      return -1;
    }
    ir_sensor_poll();
    uint8_t front = get_ir_sensor_point()->front;
    if (!front) {
      saw_ir = 1;
    } else if (saw_ir) {
      uint32_t t1 = now_ms();
      while (!elapsed(t1, stable_ms) && !elapsed(t0, timeout_ms)) {
        if (!g_mission_active) {
          release_control();
          return -1;
        }
        osDelay(10);
      }
      release_control();
      if (elapsed(t0, timeout_ms))
        return -1;
      return 0;
    }
    osDelay(5);
  }
  release_control();
  return -1;
}

/* ---- 过桥 (对应旧 run_task: pitch 触发上桥 + gobri 侧红外纠偏) ---- */

#define BRIDGE_ENTER_PITCH_DEG 12.0f  /* |pitch| 超此值 = 已上坡 */
#define BRIDGE_LEVEL_PITCH_DEG 3.0f   /* |pitch| 回此值内 = 水平 */
#define BRIDGE_LEVEL_HOLD_MS 500      /* 水平持续确认 */
#define BRIDGE_CLIMB_TIMEOUT_MS 8000  /* 循线爬坡超时 */
#define BRIDGE_CROSS_TIMEOUT_MS 30000 /* 桥面超时 */
#define BRIDGE_SPEED 0.12f
#define BRIDGE_STEER 0.8f /* 侧红外纠偏 (rad/s), 同 bridge_exec */

int bridge(void) {
  const fp32 *ins = get_INS_angle_point();
  fp32 pitch0 = ins[INS_PITCH_ADDRESS_OFFSET];
  uint32_t t0 = now_ms();

  take_control();

  /* 阶段1: 循线前进 (无线则直行, 对齐旧 trackxian5 的 default 直行兜底),
   * 直到 pitch 抬起 (上坡) */
  for (;;) {
    if (elapsed(t0, BRIDGE_CLIMB_TIMEOUT_MS)) {
      chassis_set_velocity(0.0f, 0.0f);
      release_control();
      return -1;
    }
    fp32 dp = ins[INS_PITCH_ADDRESS_OFFSET] - pitch0;
    if (dp < 0)
      dp = -dp;
    if (dp > DEG2RAD(BRIDGE_ENTER_PITCH_DEG))
      break;

    gray_sensor_poll();
    const gray_sensor_t *gs = get_gray_sensor_point();
    fp32 wz = 0.0f;
    if (is_any_line(gs))
      wz = -LT_KP * gs->position;
    if (wz > 1.0f)
      wz = 1.0f;
    else if (wz < -1.0f)
      wz = -1.0f;
    chassis_set_velocity(BRIDGE_SPEED, wz);
    osDelay(5);
  }

  /* 阶段2: 桥面 — 侧红外纠偏, 直到 pitch 回平持续 500ms 且见线 */
  t0 = now_ms();
  uint32_t level_start = 0;
  for (;;) {
    if (elapsed(t0, BRIDGE_CROSS_TIMEOUT_MS))
      break;

    ir_sensor_poll();
    gray_sensor_poll();
    const ir_sensor_t *ir = get_ir_sensor_point();
    const gray_sensor_t *gs = get_gray_sensor_point();

    fp32 steer = 0.0f;
    if (!ir->left)
      steer += BRIDGE_STEER;
    if (!ir->right)
      steer -= BRIDGE_STEER;
    chassis_set_velocity(BRIDGE_SPEED, steer);

    fp32 dp = ins[INS_PITCH_ADDRESS_OFFSET] - pitch0;
    if (dp < 0)
      dp = -dp;
    if (dp < DEG2RAD(BRIDGE_LEVEL_PITCH_DEG)) {
      if (level_start == 0)
        level_start = now_ms();
      else if (now_ms() - level_start > BRIDGE_LEVEL_HOLD_MS && is_any_line(gs))
        break;
    } else {
      level_start = 0;
    }
    osDelay(5);
  }

  chassis_set_velocity(0.0f, 0.0f);
  release_control();
  return 0;
}

/* ================================================================
 * ③ 瞬时动作 — 立即返回
 * ================================================================ */

void speed(float v) {
  line_track_set_base_speed(v);
  line_track_set_slow_speed(v * 0.5f);
}

void voice(uint8_t track) { voice_module_play((voice_track_t)track); }

void servo(uint8_t ch, uint16_t us) { servo_set_raw(ch, us); }

/* ================================================================
 * ④ 视觉 — 阻塞等结果
 * ================================================================ */

/* 通用视觉请求: 切 vision 模式 + 发命令 + 等应答 + 取数据。
 * 不切模式的话应答会被 console 当命令字符吞掉 → 超时。返回数据长度; -1=超时 */
static int vis_cmd(uint8_t cmd, uint8_t data[4])
{
  debug_console_vision_mode(1);
  vision_module_send_cmd(cmd);
  uint32_t t0 = now_ms();
  while (!vision_module_has_response()) {
    if (elapsed(t0, g_vision_timeout_ms)) {
      debug_console_vision_mode(0);
      return -1;
    }
    osDelay(10);
  }
  uint8_t n = vision_module_get_response(data);
  debug_console_vision_mode(0);
  return (int)n;
}

int tl_scan(void) {
  uint8_t data[4] = {0};
  int n = vis_cmd(0x01, data);
  if (n < 1) return -1;
  vision_result_store(0x01, data, n);
  return (int)data[0]; /* 0=红, 1=绿 */
}

int qr_scan(uint8_t qr[3]) {
  servo(SERVO_CH_PLATFORM, SERVO_PLATFORM_ACTIVE_US);
  osDelay(800);
  servo(SERVO_CH_PLATFORM, SERVO_PLATFORM_HOME_US);

  uint8_t data[4] = {0};
  int n = vis_cmd(0x02, data);
  if (n < 1) return 0;
  vision_result_store(0x02, data, n);
  if (qr) {
    qr[0] = (n >= 1) ? data[0] : 0;
    qr[1] = (n >= 2) ? data[1] : 0;
    qr[2] = (n >= 3) ? data[2] : 0;
  }
  return 1;
}

int digit_read(void) {
  uint8_t data[4] = {0};
  if (vis_cmd(0x03, data) < 1) return -1;
  return (int)data[0];
}
