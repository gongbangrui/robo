/**
 * debug_console.c
 * Debug command console for testing and tuning.
 *
 * Command interface:
 *   help                - Show available commands
 *   state               - Show FSM state and navigation info
 *   gray                - Show gray sensor data (continuous)
 *   grayonce            - Show gray sensor data (once)
 *   imu                 - Show IMU data (continuous)
 *   imuonce             - Show IMU data (once)
 *   motor               - Show motor data (continuous)
 *   motoronce           - Show motor data (once)
 *   pid [kp ki kd]      - Show/set PID parameters
 *   speed [value]       - Show/set base speed
 *   slow [value]        - Show/set slow speed
 *   path [name]         - Switch test path
 * (basic/sharp/obtuse/uturn/fork/complex) stop                - Emergency stop
 *   start               - Start/resume
 *   reset               - Reset system
 *   save                - Save parameters to Flash
 *   load                - Load parameters from Flash
 *   clear               - Clear screen
 *   log [on/off]        - Enable/disable continuous logging
 */
#include "debug_console.h"
#include "CAN_receive.h"
#include "INS_task.h"
#include "calibrate_task.h"
#include "actions.h"
#include "bsp_buzzer.h"
#include "chassis_task.h"
#include "cmsis_os.h"
#include "gray_sensor.h"
#include "ir_sensor.h"
#include "line_track_task.h"
#include "lt_utils.h"
#include "navigate_task.h"
#include "paths.h"
#include "servo_ctrl.h"
#include "vision_module.h"

volatile uint8_t g_vision_mode = 0;

#include "uart_debug.h"
#include "voice_module.h"
#include "voltage_task.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ---- private variables ---- */

static char cmd_buf[DEBUG_CMD_BUF_SIZE];
static uint8_t cmd_idx = 0;
static uint8_t cmd_ready = 0;

static char line_buf[DEBUG_LINE_BUF_SIZE];

/* Continuous output flags */
static uint8_t gray_continuous = 0;
static uint8_t imu_continuous = 0;
static uint8_t motor_continuous = 0;
static uint8_t bitmask_continuous = 0;
static uint8_t accel_continuous = 0;
static uint8_t log_enabled = 0;

/* ---- private functions ---- */

static void print_help(void) {
  debug_console_puts("\r\n=== Help ======================================\r\n");
  debug_console_puts("[状态]  state  battery\r\n");
  debug_console_puts("[路线]  path [name]  path (list)\r\n");
  debug_console_puts("        run <name>   run (list)  — 执行预制路线\r\n");
  debug_console_puts("        msn <name>   msn (list)  — 执行任务函数\r\n");
  debug_console_puts("[动作]  cross <act>  — 单步阻塞执行 (l90/r90/fwd/stop...)\r\n");
  debug_console_puts("        climb — 爬坡越障   ir — 循线前红外触发停\r\n");
  debug_console_puts("        move <mm> <spd>   turn <deg>\r\n");
  debug_console_puts("        servox  — 舵机1推拉测试 (2000us→800ms→1500us)\r\n");
  debug_console_puts("        trigger  — 等红外触发 (遮挡→离开→消抖)\r\n");
  debug_console_puts("[视觉]  vision tl|qr|digit  vision results\r\n");
  debug_console_puts("        vt tl|qr|digit [期望值]  — 识别测试, 匹配蜂鸣\r\n");
  debug_console_puts("[调参]  pid [kp ki kd]  speed [v]  slow [v]\r\n");
  debug_console_puts("[控制]  stop  start  reset\r\n");
  debug_console_puts("[调试]  trace on/off  log on/off  clear\r\n");
  debug_console_puts("[传感器] gray/grayonce  imu/imuonce  motor/motoronce\r\n");
  debug_console_puts("         ir/ironce  bitmask/bitmaskonce\r\n");
  debug_console_puts("[测试]  buzzer  servo <ch> <us>\r\n");
  debug_console_puts("==============================================\r\n");
}


static void print_state(void) {
  const gray_sensor_t *gs = get_gray_sensor_point();

  debug_console_puts("\r\n=== System State ===\r\n");
  debug_console_printf("FSM State: %s\r\n", line_track_state_name());
  debug_console_printf("Position: %+.3f  Bitmask: 0x%04X  LineCount: %d\r\n",
                       gs->position, gs->bitmask, gs->line_count);
  debug_console_printf("Pending Action: %d  dvx: %+.3f\r\n",
                       (int)line_track_get_pending_action(),
                       line_track_get_pending_dvx());

  uint8_t step = navigate_get_current_step();
  uint8_t len = navigate_get_path_length();
  debug_console_printf("Navigation: step %d/%d\r\n", step, len);

  debug_console_printf("Gray Sensor: %s\r\n",
                       gs->online ? "ONLINE" : "OFFLINE");
  debug_console_printf("Log: %s  Trace: %s\r\n", log_enabled ? "ON" : "OFF",
                       line_track_get_trace() ? "ON" : "OFF");
  debug_console_printf("Continuous: gray=%d imu=%d motor=%d bitmask=%d\r\n",
                       gray_continuous, imu_continuous, motor_continuous,
                       bitmask_continuous);
  debug_console_puts("====================\r\n\r\n");
}

static void print_gray_data(void) {
  const gray_sensor_t *gs = get_gray_sensor_point();
  debug_console_printf("GRAY: pos=%+.3f bm=0x%04X lc=%2d out=%d %s\r\n",
                       gs->position, gs->bitmask, gs->line_count, gs->out_line,
                       gs->online ? "ON" : "OFF");
}

static void print_imu_data(void) {
  const fp32 *angle = get_INS_angle_point();
  const fp32 *gyro = get_gyro_data_point();
  const fp32 *accel = get_accel_data_point();
  const fp32 *mag = get_mag_data_point();

  debug_console_printf("IMU: y=%+.2f p=%+.2f r=%+.2f "
                       "gx=%+.3f gy=%+.3f gz=%+.3f "
                       "ax=%+.3f ay=%+.3f az=%+.3f "
                       "mx=%+.1f my=%+.1f mz=%+.1f\r\n",
                       RAD2DEG(angle[0]), RAD2DEG(angle[1]), RAD2DEG(angle[2]),
                       gyro[0], gyro[1], gyro[2],
                       accel[0], accel[1], accel[2], mag[0], mag[1], mag[2]);
}

static void print_motor_data(void) {
  const motor_measure_t *m0 = get_chassis_motor_measure_point(0);
  const motor_measure_t *m1 = get_chassis_motor_measure_point(1);
  const motor_measure_t *m2 = get_chassis_motor_measure_point(2);
  const motor_measure_t *m3 = get_chassis_motor_measure_point(3);

  debug_console_printf("MOTOR: 0:%+5d 1:%+5d 2:%+5d 3:%+5d (rpm)\r\n",
                       m0->speed_rpm, m1->speed_rpm, m2->speed_rpm,
                       m3->speed_rpm);
}

static void print_bitmask_data(void) {
  const gray_sensor_t *gs = get_gray_sensor_point();
  bitmask_analysis_t a;
  analyze_bitmask(gs->bitmask, &a);
  const char *itype =
      a.left_branch ? "LEFT" : (a.right_branch ? "RIGHT" : "CENTER");

  /* Print bitmask as 16-bit binary */
  debug_console_printf("BM: 0x%04X ", gs->bitmask);
  for (int i = 15; i >= 0; i--) {
    debug_console_putchar((gs->bitmask >> i) & 1 ? '1' : '0');
    if (i == 8)
      debug_console_putchar(' '); /* separator between high/low byte */
  }

  debug_console_printf("  cl=%d  type=%-5s  L=%d R=%d blob=%d  pos=%+.2f\r\n",
                       a.cluster_count, itype, a.left_branch, a.right_branch,
                       a.is_wide_blob, gs->position);

  /* Print cluster details */
  for (uint8_t i = 0; i < a.cluster_count && i < MAX_CLUSTERS; i++) {
    debug_console_printf(
        "  cl[%d]: ch%2d-%2d (w=%d)%s\r\n", i, a.cluster_start[i],
        a.cluster_start[i] + a.cluster_width[i] - 1, a.cluster_width[i],
        (a.center_cluster_idx == i) ? " [CENTER]" : "");
  }
}

static void print_accel_data(void) {
  const fp32 *accel = get_accel_data_point();
  debug_console_printf("ACCEL: x=%+.2f y=%+.2f z=%+.2f m/s²\r\n", accel[0],
                       accel[1], accel[2]);
}

static void process_cmd(const char *cmd) {
  /* Skip empty commands */
  if (cmd[0] == '\0')
    return;

  /* <cmd> help → print usage for that command */
  {
    const char *sp = strstr(cmd, " help");
    if (sp && sp[5] == '\0' && sp != cmd) {
      static const struct { const char *k, *v; } h[] = {
        {"path",   "path [名称]  — 列出或加载命名路线; 无参列出全部"},
        {"run",    "run [名称]    — 无参列出; 加名称阻塞执行预制路线"},
        {"msn",    "msn [名称]    — 无参列出; 加名称执行任务函数"},
        {"cross",  "cross [动作]  — 无参列出; 加名称阻塞执行单个路口动作"},
        {"move",   "move <mm> <spd>  — 测试死推算直行/后退"},
        {"turn",   "turn <deg>    — 测试原地转向"},
        {"servox", "servox        — 舵机推杆测试"},
        {"qr",     "qr            — 阻塞扫码测试"},
        {"digit",  "digit         — 阻塞数字识别测试"},
        {"trigger","trigger       — 等红外触发 (遮挡→离开→消抖1s)"},
        {"vision", "vision tl|qr|digit  — 测试视觉模块; results 查看历史"},
        {"pid",    "pid [kp ki kd]  — 查询或设置循线PID参数"},
        {"speed",  "speed [值]    — 查询或设置巡线速度 (m/s)"},
        {"slow",   "slow [值]     — 查询或设置路口慢速 (m/s)"},
        {"servo",  "servo <通道> <脉宽> — 舵机测试 (ch:0-2, us:500-2500)"},
        {"voice",  "voice <0-31>  — 语音模块播放指定音轨"},
        {"vt",     "vt tl|qr|digit [期望值] — 视觉识别测试, 匹配则蜂鸣"},
        {"vto",    "vto [ms]  — 查询/设置视觉应答超时(默认3000ms)"},
        {"climb",  "cross climb  — 爬坡越障: 循线→丢线→死推越顶"},
        {"ir",     "cross ir  — 循线前红外触发即停"},
        {"trace",  "trace on|off  — 开关FSM状态转换日志"},
        {"log",    "log on|off    — 开关传感器/电机连续日志"},
        {"cali",   "cali mag       — 磁力计校准(旋转车辆~10s, 完成后降调音效)"},
      };
      char tok[16]; uint8_t i = 0;
      while (*cmd != ' ' && *cmd && i < 15) tok[i++] = *cmd++;
      tok[i] = '\0';
      for (i = 0; i < sizeof(h)/sizeof(h[0]); i++)
        if (strcmp(tok, h[i].k) == 0) { debug_console_printf("%s\r\n", h[i].v); return; }
      debug_console_puts("未知命令，输入 help 查看帮助\r\n");
      return;
    }
  }

  /* Stop continuous outputs first */
  if (strcmp(cmd, "gray") != 0 && strcmp(cmd, "imu") != 0 &&
      strcmp(cmd, "motor") != 0 && strcmp(cmd, "bitmask") != 0 &&
      strcmp(cmd, "accel") != 0) {
    gray_continuous = 0;
    imu_continuous = 0;
    motor_continuous = 0;
    bitmask_continuous = 0;
    accel_continuous = 0;
  }

  /* Parse command */
  if (strcmp(cmd, "help") == 0) {
    print_help();
  } else if (strcmp(cmd, "state") == 0) {
    print_state();
  } else if (strcmp(cmd, "gray") == 0) {
    gray_continuous = 1;
    debug_console_puts(
        "Gray sensor continuous output (send any command to stop)\r\n");
  } else if (strcmp(cmd, "grayonce") == 0) {
    print_gray_data();
  } else if (strcmp(cmd, "imu") == 0) {
    imu_continuous = 1;
    debug_console_puts("IMU continuous output (send any command to stop)\r\n");
  } else if (strcmp(cmd, "imuonce") == 0) {
    print_imu_data();
  } else if (strcmp(cmd, "motor") == 0) {
    motor_continuous = 1;
    debug_console_puts(
        "Motor continuous output (send any command to stop)\r\n");
  } else if (strcmp(cmd, "motoronce") == 0) {
    print_motor_data();
  } else if (strcmp(cmd, "bitmask") == 0) {
    bitmask_continuous = 1;
    debug_console_puts(
        "Bitmask analysis continuous output (send any command to stop)\r\n");
  } else if (strcmp(cmd, "bitmaskonce") == 0) {
    print_bitmask_data();
  } else if (strncmp(cmd, "pid", 3) == 0) {
    /* Parse: pid [kp ki kd] */
    float kp, ki, kd;
    if (sscanf(cmd + 3, "%f %f %f", &kp, &ki, &kd) == 3) {
      /* Set new PID parameters */
      line_track_set_pid(kp, ki, kd);
      debug_console_printf("PID set: Kp=%.3f Ki=%.4f Kd=%.3f\r\n", kp, ki, kd);
    } else {
      /* Show current PID */
      fp32 cur_kp, cur_ki, cur_kd;
      line_track_get_pid(&cur_kp, &cur_ki, &cur_kd);
      debug_console_printf("Current PID: Kp=%.3f Ki=%.4f Kd=%.3f\r\n", cur_kp,
                           cur_ki, cur_kd);
      debug_console_puts("Usage: pid <kp> <ki> <kd>\r\n");
    }
  } else if (strncmp(cmd, "speed", 5) == 0) {
    /* Parse: speed [value] */
    float speed;
    if (sscanf(cmd + 5, "%f", &speed) == 1) {
      line_track_set_base_speed(speed);
      debug_console_printf("Base speed set: %.3f m/s\r\n", speed);
    } else {
      debug_console_printf("Current base speed: %.3f m/s\r\n",
                           line_track_get_base_speed());
      debug_console_puts("Usage: speed <value>\r\n");
    }
  } else if (strncmp(cmd, "slow", 4) == 0) {
    /* Parse: slow [value] */
    float speed;
    if (sscanf(cmd + 4, "%f", &speed) == 1) {
      line_track_set_slow_speed(speed);
      debug_console_printf("Slow speed set: %.3f m/s\r\n", speed);
    } else {
      debug_console_printf("Current slow speed: %.3f m/s\r\n",
                           line_track_get_slow_speed());
      debug_console_puts("Usage: slow <value>\r\n");
    }
  } else if (strcmp(cmd, "battery") == 0) {
    debug_console_printf("Battery: %.2f V  %d%%\r\n", battery_voltage,
                         get_battery_percentage());
  } else if (strcmp(cmd, "buzzer") == 0) {
    debug_console_puts("Buzzer test: init done melody\r\n");
    buzzer_init();
    buzzer_init_done();
  } else if (strncmp(cmd, "path", 4) == 0) {
    const char *name = cmd + 4;
    while (*name == ' ') name++;
    if (*name == '\0') {
      debug_console_puts("Available paths:\r\n");
      for (const path_entry_t *p = g_path_registry; p->name; p++) {
        debug_console_printf("  %s (%d steps)\r\n", p->name, p->length);
      }
      debug_console_puts("Usage: path <name>\r\n");
    } else {
      const path_entry_t *pe = path_find(name);
      if (pe) {
        navigate_load_path(pe->steps, pe->length);
        debug_console_printf("Path loaded: %s (%d steps)\r\n",
                             pe->name, pe->length);
      } else {
        debug_console_printf("Unknown path: %s\r\n", name);
      }
    }
  } else if (strncmp(cmd, "run", 3) == 0) {
    if (g_mission_active) { debug_console_puts("ERR: busy, 'stop' first\r\n"); return; }
    const char *name = cmd + 3;
    while (*name == ' ') name++;
    if (*name == '\0') {
      debug_console_puts("Named paths:\r\n");
      for (const path_entry_t *p = g_path_registry; p->name; p++)
        debug_console_printf("  %s (%d steps)\r\n", p->name, p->length);
      debug_console_puts("Usage: run <name>\r\n");
    } else {
      debug_console_printf("Running: %s...\r\n", name);
      run(name);
      debug_console_printf("Done: %s\r\n", name);
    }
  } else if (strncmp(cmd, "msn", 3) == 0) {
    if (g_mission_active) { debug_console_puts("ERR: busy, 'stop' first\r\n"); return; }
    /* clang-format off */
    extern void mission_comp(void);
    extern void mission_loop(void);
    extern void mission_simple(void);
    extern void go12(void);
    struct { const char *n; void (*f)(void); } map[] = {
      {"comp", mission_comp}, {"loop", mission_loop}, {"simple", mission_simple},
      {"go12", go12},
    };
    /* clang-format on */
    const char *name = cmd + 3;
    while (*name == ' ') name++;
    if (*name == '\0') {
      debug_console_puts("Missions: ");
      for (uint8_t i = 0; i < sizeof(map)/sizeof(map[0]); i++)
        debug_console_printf("%s ", map[i].n);
      debug_console_puts("\r\n");
    } else {
      for (uint8_t i = 0; i < sizeof(map)/sizeof(map[0]); i++)
        if (strcmp(name, map[i].n) == 0) {
          debug_console_printf("Mission: %s\r\n", name);
          map[i].f();
          return;
        }
      debug_console_printf("Unknown mission: %s\r\n", name);
    }
  } else if (strncmp(cmd, "cross", 5) == 0) {
    if (g_mission_active) { debug_console_puts("ERR: busy, 'stop' first\r\n"); return; }
    /* clang-format off */
    struct { const char *n; navigate_action_t a; } map[] = {
      {"fwd",  FORWARD}, {"back", BACKWARD}, {"stop", STOP},
      {"l10",LEFT_10},{"l20",LEFT_20},{"l30",LEFT_30},{"l40",LEFT_40},
      {"l50",LEFT_50},{"l60",LEFT_60},{"l70",LEFT_70},{"l80",LEFT_80},
      {"l90",LEFT_90},{"l100",LEFT_100},{"l110",LEFT_110},{"l120",LEFT_120},
      {"l130",LEFT_130},{"l140",LEFT_140},{"l150",LEFT_150},{"l160",LEFT_160},
      {"l170",LEFT_170},{"l180",LEFT_180},
      {"r10",RIGHT_10},{"r20",RIGHT_20},{"r30",RIGHT_30},{"r40",RIGHT_40},
      {"r50",RIGHT_50},{"r60",RIGHT_60},{"r70",RIGHT_70},{"r80",RIGHT_80},
      {"r90",RIGHT_90},{"r100",RIGHT_100},{"r110",RIGHT_110},{"r120",RIGHT_120},
      {"r130",RIGHT_130},{"r140",RIGHT_140},{"r150",RIGHT_150},{"r160",RIGHT_160},
      {"r170",RIGHT_170},{"r180",RIGHT_180},
      {"slow",SLOW},{"fast",FAST},{"auto",AUTO},
      {"plat",PLATFORM},{"bridge",BRIDGE},
      {"tl",TRAFFIC},{"qr",QRSCAN},{"digit",DIGIT},
      {"lend",LINE_END},{"climb",CLIMB},{"ir",IR},
    };
    /* clang-format on */
    const char *name = cmd + 5;
    while (*name == ' ') name++;
    if (*name == '\0') {
      debug_console_puts("Actions: ");
      for (uint8_t i = 0; i < sizeof(map)/sizeof(map[0]); i++)
        debug_console_printf("%s ", map[i].n);
      debug_console_puts("\r\n");
    } else {
      navigate_action_t act = NONE;
      for (uint8_t i = 0; i < sizeof(map)/sizeof(map[0]); i++)
        if (strcmp(name, map[i].n) == 0) { act = map[i].a; break; }
      if (act != NONE) {
        debug_console_printf("cross %s...\r\n", name);
        cross(act);
        line_track_set_nav_action(STOP);
        debug_console_puts("done\r\n");
      } else {
        debug_console_printf("Unknown: %s\r\n", name);
      }
    }
  } else if (strncmp(cmd, "move", 4) == 0) {
    if (g_mission_active) { debug_console_puts("ERR: busy, 'stop' first\r\n"); return; }
    int mm; float spd;
    if (sscanf(cmd + 4, "%d %f", &mm, &spd) == 2) {
      debug_console_printf("move %dmm %.2f m/s\r\n", mm, spd);
      move((int16_t)mm, spd, 5000);
    } else {
      debug_console_puts("Usage: move <mm> <spd>  (mm>0=前 mm<0=后)\r\n");
    }
  } else if (strncmp(cmd, "turn", 4) == 0) {
    if (g_mission_active) { debug_console_puts("ERR: busy, 'stop' first\r\n"); return; }
    int deg;
    if (sscanf(cmd + 4, "%d", &deg) == 1) {
      debug_console_printf("turn %d deg\r\n", deg);
      turn((int16_t)deg);
    } else {
      debug_console_puts("Usage: turn <deg>  (+ = right, - = left)\r\n");
    }
  } else if (strcmp(cmd, "servox") == 0) {
    if (g_mission_active) { debug_console_puts("ERR: busy, 'stop' first\r\n"); return; }
    servo(SERVO_CH_PLATFORM, SERVO_PLATFORM_ACTIVE_US);
    osDelay(800);
    servo(SERVO_CH_PLATFORM, SERVO_PLATFORM_HOME_US);
    debug_console_puts("servo done\r\n");
  } else if (strcmp(cmd, "qr") == 0) {
    if (g_mission_active) { debug_console_puts("ERR: busy, 'stop' first\r\n"); return; }
    debug_console_puts("qr scan...\r\n");
    uint8_t qr[3] = {0};
    if (qr_scan(qr))
      debug_console_printf("qr = [%c][%c][%c]\r\n", qr[0], qr[1], qr[2]);
    else
      debug_console_puts("qr timeout\r\n");
  } else if (strcmp(cmd, "digit") == 0) {
    if (g_mission_active) { debug_console_puts("ERR: busy, 'stop' first\r\n"); return; }
    debug_console_puts("digit read...\r\n");
    debug_console_printf("digit = %d\r\n", digit_read());
  } else if (strcmp(cmd, "trigger") == 0) {
    if (g_mission_active) { debug_console_puts("ERR: busy, 'stop' first\r\n"); return; }
    debug_console_puts("waiting for IR trigger...\r\n");
    int r = wait_ir_trigger(60000, 1000);
    debug_console_printf("trigger = %d\r\n", r);
  } else if (strncmp(cmd, "cali", 4) == 0) {
    if (g_mission_active) { debug_console_puts("ERR: busy, 'stop' first\r\n"); return; }
    if (strstr(cmd, "mag")) {
      debug_console_puts("Mag cali: 旋转车辆绕Z轴~10s, 扫描各方向\r\n");
      if (cali_mag(15000))
        debug_console_puts("Mag cali done ✓\r\n");
      else
        debug_console_puts("Mag cali TIMEOUT\r\n");
    } else {
      debug_console_puts("Usage: cali mag\r\n");
    }
  } else if (strcmp(cmd, "stop") == 0) {
    actions_force_stop();
  } else if (strcmp(cmd, "start") == 0) {
    /* Re-enable with default action */
    line_track_set_nav_action(FORWARD);
    debug_console_puts("Started\r\n");
  } else if (strcmp(cmd, "reset") == 0) {
    debug_console_puts("System reset...\r\n");
    osDelay(100);
    NVIC_SystemReset();
  } else if (strcmp(cmd, "save") == 0) {
    debug_console_puts("Save params to Flash (not implemented)\r\n");
  } else if (strcmp(cmd, "load") == 0) {
    debug_console_puts("Load params from Flash (not implemented)\r\n");
  } else if (strcmp(cmd, "clear") == 0) {
    /* ANSI escape: clear screen + move cursor home */
    debug_console_puts("\033[2J\033[H");
  } else if (strncmp(cmd, "log", 3) == 0) {
    const char *arg = cmd + 3;
    while (*arg == ' ')
      arg++;
    if (strcmp(arg, "on") == 0) {
      log_enabled = 1;
      debug_console_puts("Logging enabled\r\n");
    } else if (strcmp(arg, "off") == 0) {
      log_enabled = 0;
      debug_console_puts("Logging disabled\r\n");
    } else {
      debug_console_printf("Logging: %s\r\n", log_enabled ? "ON" : "OFF");
      debug_console_puts("Usage: log [on/off]\r\n");
    }
  } else if (strncmp(cmd, "trace", 5) == 0) {
    const char *arg = cmd + 5;
    while (*arg == ' ')
      arg++;
    if (strcmp(arg, "on") == 0) {
      line_track_set_trace(1);
    } else if (strcmp(arg, "off") == 0) {
      line_track_set_trace(0);
    } else {
      debug_console_printf("Trace: %s\r\n",
                           line_track_get_trace() ? "ON" : "OFF");
      debug_console_puts("Usage: trace [on/off]\r\n");
    }
  }
  /* ---- 红外传感器 ---- */
  else if (strcmp(cmd, "ironce") == 0) {
    const ir_sensor_t *ir = get_ir_sensor_point();
    debug_console_printf("IR: front=%d left=%d right=%d extra=%d\r\n",
                         ir->front, ir->left, ir->right, ir->extra);
  } else if (strcmp(cmd, "ir") == 0) {
    gray_continuous = 0;
    imu_continuous = 0;
    motor_continuous = 0;
    /* 红外传感器连续输出复用 tick 机制，在 loop 中处理。
     * 这里简化：用独立的 tick 变量触发。 */
    debug_console_puts("IR continuous output (send any command to stop)\r\n");
    /* 通过将 front/left/right 推送到 gray_continuous 循环中处理，
     * 采用简单方案：ironce 单次查看即可，连续查看用外部逻辑。 */
  }
  /* ---- 舵机控制 ---- */
  else if (strncmp(cmd, "servo", 5) == 0) {
    int ch, us;
    if (sscanf(cmd + 5, "%d %d", &ch, &us) == 2) {
      servo((uint8_t)ch, (uint16_t)us);
      debug_console_printf("Servo ch%d → %d us\r\n", ch, us);
    } else {
      debug_console_puts("Usage: servo <ch> <us>  (ch:0-2, us:500-2500)\r\n");
    }
  }
  /* ---- 语音模块 ---- */
  else if (strncmp(cmd, "voice", 5) == 0) {
    int track;
    if (sscanf(cmd + 5, "%d", &track) == 1) {
      voice_module_play((voice_track_t)track);
      debug_console_printf("Voice track %d\r\n", track);
    } else {
      debug_console_puts("Usage: voice <track>  (0-31)\r\n");
    }
  }
  /* ---- 视觉模块 (USART6 协议) ---- */
  else if (strncmp(cmd, "vision", 6) == 0) {
    const char *a = cmd + 6;
    while (*a == ' ') a++;
    if (strcmp(a, "tl") == 0) {
      debug_console_vision_mode(1);
      vision_module_send_cmd(0x01);
      debug_console_puts("Sent TL query, waiting...\r\n");
      uint32_t t0 = osKernelSysTick();
      while (!vision_module_has_response() && (osKernelSysTick() - t0) < 3000);
      debug_console_vision_mode(0);
      uint8_t d[4];
      if (vision_module_get_response(d))
        debug_console_printf("TL: %s  (0=红 1=绿)\r\n", d[0] ? "GREEN" : "RED");
      else
        debug_console_puts("TL timeout\r\n");
    } else if (strcmp(a, "qr") == 0) {
      debug_console_vision_mode(1);
      vision_module_send_cmd(0x02);
      debug_console_puts("Sent QR query, waiting...\r\n");
      uint32_t t0 = osKernelSysTick();
      while (!vision_module_has_response() && (osKernelSysTick() - t0) < 3000);
      debug_console_vision_mode(0);
      uint8_t d[4]; uint8_t n = vision_module_get_response(d);
      if (n >= 2)
        debug_console_printf("QR: [%d, %d]\r\n", d[0], d[1]);
      else
        debug_console_puts("QR timeout\r\n");
    } else if (strcmp(a, "digit") == 0) {
      debug_console_vision_mode(1);
      vision_module_send_cmd(0x03);
      debug_console_puts("Sent digit query, waiting...\r\n");
      uint32_t t0 = osKernelSysTick();
      while (!vision_module_has_response() && (osKernelSysTick() - t0) < 3000);
      debug_console_vision_mode(0);
      uint8_t d[4];
      if (vision_module_get_response(d))
        debug_console_printf("Digit: %d\r\n", d[0]);
      else
        debug_console_puts("Digit timeout\r\n");
    } else if (strcmp(a, "results") == 0) {
      debug_console_printf("Vision results (%d):\r\n", g_vision_result_count);
      for (uint8_t i = 0; i < g_vision_result_count; i++) {
        debug_console_printf("  [%d] cmd=%02X data=", i, g_vision_results[i].cmd);
        for (uint8_t j = 0; j < g_vision_results[i].len; j++)
          debug_console_printf("%d ", g_vision_results[i].data[j]);
        debug_console_puts("\r\n");
      }
    } else {
      debug_console_puts("Usage: vision tl|qr|digit|results\r\n");
    }
  }
  /* ---- 视觉测试: 发命令, 比对期望值, 匹配则蜂鸣 ---- */
  else if (strcmp(cmd, "vto") == 0) {
    debug_console_printf("VT timeout: %lu ms\r\n", (unsigned long)g_vision_timeout_ms);
    debug_console_puts("Usage: vto <ms>  -- set vision response timeout\r\n");
  } else if (strncmp(cmd, "vto ", 4) == 0) {
    unsigned long v = 0;
    if (sscanf(cmd + 4, "%lu", &v) == 1 && v >= 100 && v <= 30000) {
      g_vision_timeout_ms = (uint32_t)v;
      debug_console_printf("VT timeout set to %lu ms\r\n",
                           (unsigned long)g_vision_timeout_ms);
    } else {
      debug_console_puts("Usage: vto <ms>  (100-30000)\r\n");
    }
  }
  else if (strncmp(cmd, "vt", 2) == 0 && (cmd[2] == ' ' || cmd[2] == '\0')) {
    const char *a = cmd + 2;
    while (*a == ' ') a++;
    /* 定位子命令与期望值 */
    const char *sub = a;
    while (*sub && *sub != ' ') sub++;
    const char *args = (*sub == ' ') ? sub + 1 : NULL;

    uint8_t cmd_byte = 0xFF;
    if (strncmp(a, "tl", 2) == 0)          cmd_byte = 0x01;
    else if (strncmp(a, "qr", 2) == 0)     cmd_byte = 0x02;
    else if (strncmp(a, "digit", 5) == 0)  cmd_byte = 0x03;
    else {
      debug_console_puts("Usage: vt tl|qr|digit [expect...]\r\n");
      return;
    }

    debug_console_printf("VT: query 0x%02X...\r\n", cmd_byte);
    debug_console_vision_mode(1);
    vision_module_send_cmd(cmd_byte);
    uint32_t t0 = osKernelSysTick();
    while (!vision_module_has_response() &&
           (osKernelSysTick() - t0) < g_vision_timeout_ms)
      osDelay(5);
    debug_console_vision_mode(0);

    uint8_t d[4] = {0, 0, 0, 0};
    uint8_t n = vision_module_get_response(d);
    if (n == 0) {
      debug_console_puts("VT: timeout, no response\r\n");
      return;
    }

    /* 回显实际读数 */
    if (cmd_byte == 0x01)
      debug_console_printf("VT traffic-light: %s (0x%02X)\r\n",
                           d[0] ? "GREEN" : "RED", d[0]);
    else if (cmd_byte == 0x02)
      debug_console_printf("VT qr: qr[0]=0x%02X(%c) qr[1]=0x%02X(%c) qr[2]=0x%02X(%c)\r\n",
                           d[0], d[0] >= ' ' && d[0] <= '~' ? d[0] : '.',
                           d[1], d[1] >= ' ' && d[1] <= '~' ? d[1] : '.',
                           d[2], d[2] >= ' ' && d[2] <= '~' ? d[2] : '.');
    else
      debug_console_printf("VT digit: 0x%02X(%c)\r\n",
                           d[0], d[0] >= ' ' && d[0] <= '~' ? d[0] : '.');

    if (!args || !*args) {
      debug_console_puts("VT: no expected value, echo only\r\n");
      return;
    }

    /* 解析期望值 (QR 支持 1~3 个值, 其余只比首个) */
    int exp[3] = {-1, -1, -1};
    int used = 0;
    {
      const char *q = args;
      while (used < 3 && sscanf(q, "%d", &exp[used]) == 1) {
        used++;
        while (*q == ' ' || *q == '\t') q++;
        while (*q >= '0' && *q <= '9') q++;
      }
    }
    if (cmd_byte == 0x02) {
      /* QR 数据为 ASCII 字符: 期望值 n 转成 'n' 再比对 */
      if (used == 3) {
        if (d[0] == (uint8_t)(exp[0] + '0') && d[1] == (uint8_t)(exp[1] + '0') &&
            d[2] == (uint8_t)(exp[2] + '0')) {
          debug_console_printf("VT: [OK] qr[%d,%d,%d] all match\r\n",
                               d[0], d[1], d[2]);
          buzzer_on(10, 500);
          osDelay(150);
          buzzer_off();
        } else {
          debug_console_printf(
              "VT: [FAIL] got[%d,%d,%d](\"%c%c%c\") != exp[%d,%d,%d]\r\n",
              d[0], d[1], d[2], d[0], d[1], d[2], exp[0], exp[1], exp[2]);
        }
      } else {
        if (d[0] == (uint8_t)(exp[0] + '0')) {
          debug_console_printf("VT: [OK] qr[0]=%d(\"%c\") match %d\r\n", d[0],
                               d[0], exp[0]);
          buzzer_on(10, 500);
          osDelay(150);
          buzzer_off();
        } else {
          debug_console_printf("VT: [FAIL] qr[0]=%d(\"%c\") != exp %d\r\n", d[0],
                               d[0], exp[0]);
        }
      }
    } else {
      /* tl/digit 也可能返回 ASCII: 期望值 n 转 'n' 比对, 兼容数值 */
      uint8_t exp_ascii = (uint8_t)(exp[0] + '0');
      if (d[0] == exp_ascii || d[0] == (uint8_t)exp[0]) {
        debug_console_printf("VT: [OK] got 0x%02X(%c) == exp %d\r\n", d[0],
                             d[0] >= ' ' && d[0] <= '~' ? d[0] : '.', exp[0]);
        buzzer_on(10, 500);
        osDelay(150);
        buzzer_off();
      } else {
        debug_console_printf("VT: [FAIL] got 0x%02X(%c) != exp %d\r\n", d[0],
                             d[0] >= ' ' && d[0] <= '~' ? d[0] : '.', exp[0]);
      }
    }
  } else {
    debug_console_printf("unknown command: %s\r\n", cmd);
    debug_console_puts("Type 'help' for commands\r\n");
  }
}

/* ---- public API ---- */

void debug_console_putchar(uint8_t ch) {
  /* Send via USART6 */
  uart_putchar(ch);
}

void debug_console_puts(const char *str) {
  while (*str) {
    debug_console_putchar(*str++);
  }
}

void debug_console_printf(const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  vsnprintf(line_buf, DEBUG_LINE_BUF_SIZE, fmt, args);
  va_end(args);
  debug_console_puts(line_buf);
}

uint8_t debug_console_has_cmd(void) { return cmd_ready; }

const char *debug_console_get_cmd(void) { return cmd_buf; }

void debug_console_clear_cmd(void) {
  cmd_buf[0] = '\0';
  cmd_idx = 0;
  cmd_ready = 0;
}

/* ---- task entry ---- */

void debug_console_task(void const *pvParameters) {
  (void)pvParameters;

  uint32_t tick = 0;

  /* Start interrupt-based RX on USART6 */
  uart_debug_init();

  debug_console_puts("\r\n=== Smart Car Debug Console ===\r\n");
  debug_console_puts("Type 'help' for available commands\r\n\r\n");

  for (;;) {
    /* Drain all received characters from ring buffer */
    uint8_t ch;
    while ((ch = uart_getchar()) != 0) {
      if (ch == '\r' || ch == '\n') {
        debug_console_puts("\r\n");
        if (cmd_idx > 0) {
          cmd_buf[cmd_idx] = '\0';
          cmd_ready = 1;
        }
      } else if (ch == '\b' || ch == 127) {
        if (cmd_idx > 0) {
          cmd_idx--;
          debug_console_puts("\b \b");
        }
      } else if (ch >= 32 && ch < 127) {
        if (cmd_idx < DEBUG_CMD_BUF_SIZE - 1) {
          cmd_buf[cmd_idx++] = ch;
          debug_console_putchar(ch);
        }
      }
    }

    /* Process command if ready */
    if (cmd_ready) {
      process_cmd(cmd_buf);
      debug_console_clear_cmd();
    }

    /* Continuous output (every 100ms) */
    if (++tick >= 20) { /* 5ms * 20 = 100ms */
      tick = 0;

      if (gray_continuous) {
        print_gray_data();
      }
      if (imu_continuous) {
        print_imu_data();
      }
      if (motor_continuous) {
        print_motor_data();
      }
      if (bitmask_continuous) {
        print_bitmask_data();
      }
      if (accel_continuous) {
        print_accel_data();
      }
    }

    osDelay(5);
  }
}

void debug_console_vision_mode(uint8_t enable) { g_vision_mode = enable; }
