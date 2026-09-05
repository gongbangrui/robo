# Smart Car: Debug Infrastructure + Intersection Intelligence + Collision Detection

**Date:** 2026-05-16
**Status:** Implemented
**Scope:** Debug tooling, intersection handling improvements, collision detection, IMU calibration, code cleanup

---

## 1. Problem Statement

The smart car's line-following FSM and intersection navigation code was functionally complete but lacked observability. When running on real hardware, the gap between expected and actual behavior was difficult to diagnose because:

- State transitions happened silently — no way to see why the FSM changed state
- Cluster analysis (`analyze_bitmask()`) results were computed but never displayed
- Intersection crossing was a black box — couldn't see what the sensor detected mid-crossing
- Dead-reckon parameters were invisible during runtime
- No collision detection existed
- IMU calibration hooks were stubbed out (NULL)
- Dead code from the DJI template cluttered the codebase

## 2. Design Decisions

### 2.1 Debug-first approach

Every feature ships with its own debug observability. The debug console (`USART6`, 115200 baud) is the primary interface. Commands are added incrementally so each can be verified on hardware independently.

### 2.2 Collision detection via IMU acceleration

**Chosen approach:** Integrate into `line_track_task` control loop (5ms), check IMU net acceleration magnitude against threshold. Single-module, no new task.

**Alternatives considered:**
- Separate `collision_task`: rejected — adds inter-task latency for an emergency stop that must be immediate
- Gyro-based (angular velocity spike): rejected — car spinning doesn't necessarily mean collision; accel spike is more reliable

**Threshold:** ±15.0 m/s² net acceleration (total magnitude minus 9.8 gravity). This is ~1.5g, well above normal driving forces (~2-5 m/s² during acceleration/braking). Tunable via `accel` debug command.

### 2.3 IMU calibration

**Chosen approach:** Implement the existing `cali_hook_fun` framework hooks for accel and mag. Accel: 200-sample average while stationary. Mag: 500-sample min/max while rotating. Both persist to flash.

**Why not external tools:** The car has no WiFi/Bluetooth; serial debug console is the only interface. Calibration must be triggerable from the console.

### 2.4 Code cleanup

Removed dead BSP modules (fric, laser, servo, rc) and CAN2/gimbal code. These were inherited from the DJI standard robot template and never used by the smart car.

## 3. Architecture

### 3.1 Debug command flow

```
User (USART6 terminal)
  → debug_console_task (5ms polling)
    → process_cmd()
      → line_track_set_trace()     [trace on/off]
      → line_track_analyze_bitmask()  [bitmask/bitmaskonce]
      → line_track_get_deadreckon()   [deadreckon]
      → line_track_get_collision()    [collision]
      → line_track_get_net_accel()    [accel]
      → line_track_clear_collision()  [clear]
      → calibrate_trigger()           [calibrate accel/mag]
```

### 3.2 Collision detection data flow

```
INS_task (1ms) → get_accel_data_point() → accel[3]
  ↓
line_track_task (5ms) → check_collision()
  → sqrt(ax²+ay²+az²) - 9.8
  → |net| > 15.0? → collision_detected = 1
  → FSM → STOP, motors off
  → [COLLISION] trace output to debug console
```

### 3.3 FSM trace output format

```
[TRACE] %6lu ms  %-5s -> %-5s  reason=%-20s  pos=%+.2f bm=0x%04X lc=%d
```

Example:
```
[TRACE]    12 ms  TRACK -> APPR   reason=intersection_detected  pos=+0.05 bm=0x01E0 lc=4
[TRACE]   156 ms  APPR  -> ENTER  reason=all_black_enter        pos=-0.02 bm=0xFFFF lc=16
[TRACE]   156 ms    itype=T_LEFT exp_branch=1 dr_vx=0.20 dr_wz=1.50
```

### 3.4 Intersection crossing trace

During `INTERSECTION_CROSS`, a periodic trace fires every ~200ms:

```
[CROSS] %4lums  exp=%s  type=%-4s  cl=%d  pos=%+.2f  vx=%.2f wz=%.2f
```

### 3.5 Bitmask analysis output

```
BM: 0x01E0 0000000111100000  cl=2  type=T_LEFT  L=1 R=0 blob=0  pos=+0.05
  cl[0]: ch 5- 8 (w=4) [CENTER]
  cl[1]: ch 1- 3 (w=3)
```

## 4. Components Modified

### 4.1 `application/line_track_task.c/h`

| Addition | Purpose |
|----------|---------|
| `set_state(reason)` | Now takes trigger reason string, outputs trace when enabled |
| `trace_enabled`, `trace_start_ms` | Trace on/off state and time reference |
| `line_track_set_trace()` / `line_track_get_trace()` | Public trace control API |
| `check_collision()` | IMU accel-based collision detection, 500ms debounce |
| `collision_detected`, `collision_time_ms` | Collision state |
| `line_track_get_collision()` / `line_track_clear_collision()` | Public collision API |
| `line_track_get_net_accel()` | Debug: current net acceleration |
| `line_track_get_pending_action()` | Debug: current nav action |
| `line_track_get_deadreckon()` | Debug: current DR params + expected branch |
| `line_track_analyze_bitmask()` / `line_track_classify_type()` | Public bitmask analysis wrappers |
| `classify_intersection()` | Internal: determines intersection type + expected branch |
| `expected_branch` | Saved at APPROACH→ENTER transition for CROSS phase |
| `cross_trace_tick` | Timer for periodic CROSS-phase trace output |
| Branch-aware line reacquisition | CROSS state checks position side matching expected branch |

### 4.2 `application/debug_console.c/h`

| Command | Function |
|---------|----------|
| `trace [on/off]` | Enable/disable FSM transition trace |
| `bitmask` / `bitmaskonce` | Continuous/one-shot cluster analysis |
| `deadreckon` | Show DR params and expected branch |
| `collision` | Show collision state and net accel |
| `accel` | Continuous acceleration output |
| `clear` | Clear collision flag |
| `calibrate accel` | Trigger accel calibration |
| `calibrate mag` | Trigger mag calibration |

### 4.3 `application/CAN_receive.c/h`

Removed: `GIMBAL_CAN`, `hcan2`, gimbal CAN IDs, `CAN_cmd_gimbal()`, gimbal motor getters. Shrank `motor_chassis` from 7 to 4.

### 4.4 `application/calibrate_task.c/h`

Added `cali_accel_hook` (200-sample average) and `cali_mag_hook` (500-sample min/max). Wired into `cali_hook_fun` array at indices `CALI_ACC=3` and `CALI_MAG=4`. Added `calibrate_trigger(uint8_t id)` public API.

### 4.5 `bsp/boards/` (deleted)

Removed: `bsp_fric.c/h`, `bsp_laser.c/h`, `bsp_servo_pwm.c/h`, `bsp_rc.c/h` (8 files, not in Makefile, not referenced).

## 5. Tuning Parameters

| Parameter | Default | Location | Adjustable via |
|-----------|---------|----------|---------------|
| Collision threshold | 15.0 m/s² | `COLLISION_ACCEL_THRESHOLD` | Recompile |
| Collision debounce | 500 ms | `COLLISION_DEBOUNCE_MS` | Recompile |
| Cross trace period | 200 ms | Inline constant | Recompile |
| Accel cali samples | 200 | `ACCEL_CALI_SAMPLES` | Recompile |
| Mag cali samples | 500 | `MAG_CALI_SAMPLES` | Recompile |

## 6. Testing Checklist

- [ ] `grayonce` — verify sensor online, bitmask changes when line moves
- [ ] `trace on` + `path basic` + `start` — verify state transitions logged correctly
- [ ] `bitmask` — verify cluster count and type match physical intersection
- [ ] `deadreckon` — verify expected branch matches pending action
- [ ] `accel` — observe baseline during normal driving, note peak values
- [ ] Bump car while running — verify `collision` triggers and motors stop
- [ ] `clear` — verify car resumes after collision
- [ ] `calibrate accel` — level surface, verify offsets saved
- [ ] `calibrate mag` — rotate car, verify offsets saved
- [ ] Full path run with `trace on` — verify no unexpected state transitions
