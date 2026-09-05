# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Autonomous line-following smart car based on RoboMaster Dev Board Type C (STM32F407). Pure programmatic control — no remote control. The car follows lines using a 16-channel grayscale sensor (UART), navigates intersections via a state machine, and follows pre-defined paths through a data-driven path planner. IMU (BMI088 + IST8310) provides attitude for obstacle/collision detection.

**Toolchain:** GCC (arm-none-eabi-gcc) + Makefile, STM32CubeMX 5.2.1, STM32Cube FW_F4 V1.21.1, FreeRTOS 10.0.1 via CMSIS-RTOS v1.02.

## Pinout (DJI C-Type Board exposed connectors)

| Peripheral | Pins | Purpose |
|------------|------|---------|
| USART1 | PA9(TX) PB7(RX) | 16-channel gray sensor (DMA RX, idle-line) |
| USART6 | PG14(TX) | Debug console + printf (polling, 115200 8N1) |
| CAN1 | PD0(RX) PD1(TX) | M3508 chassis motors (1 Mbps) |
| I2C1 | PB8(SCL) PB9(SDA) | (unused) |
| I2C3 | PA8(SCL) PC9(SDA) | IST8310 magnetometer (IMU) |
| Voice | PB12(D0) PB13(D1) PB14(D2) PB15(D3) PF0(D4) | Voice module 5-bit parallel track select (low-active) |
| SPI1 | PB3(SCK) PB4(MISO) PA7(MOSI) | BMI088 IMU (accel + gyro) |
| USB_OTG_FS | PA11(DM) PA12(DP) | CDC virtual COM (optional debug) |
| RCC | PH0/PH1 | HSE 12 MHz → PLL → 168 MHz sysclk |

## Build System

**Option A — Make (primary):**

```bash
make -j$(nproc)        # build → build/smart_car.elf
make flash             # flash via OpenOCD (CMSIS-DAP)
make clean             # remove build/
make reset             # reset MCU via OpenOCD
make gdb               # start GDB session on :3333
```

**Option B — CMake:**

```bash
cmake -B build_cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
cmake --build build_cmake -j$(nproc)
```

A Clang/LLVM toolchain is also available via `cmake/starm-clang.cmake` (starm-clang with picolibc/newlib options).

### Important linker flags
Both build systems use `-u _printf_float` (Make) and `-u _printf_float -u _scanf_float` (CMake). These are **required** — the debug console's `sscanf("%f")` and `printf("%f")` need them, otherwise floating-point I/O silently produces garbage.

### Library compatibility
`AHRS.lib` and `arm_cortexM4lf_math.lib` in `components/algorithm/` are **Keil/ARMCC format** — they cannot be linked by GCC. For GCC builds, compile CMSIS-DSP from source or use the GCC prebuilt `libarm_cortexM4lf_math.a` from STM32CubeF4. Replace AHRS.lib with an open-source AHRS implementation or a GCC-compiled `libAHRS.a`.

### CubeMX changes from standard_robot.ioc

**Disable:** CAN2, CRC, USART3, USART6 DMA streams (RX and TX), DMA2_Stream7 (USART1_TX)
**Reconfigure:** USART1 → gray sensor (115200 8N1, DMA RX Circular on DMA2_Stream5), USART6 → TX-only debug console
**Release pins:** PB5, PB6, PC10, PC11, PG9, PI7, PG14

## Architecture: Three-Layer State Machine

### Layer 1 — Sensor (`components/devices/gray_sensor`)

16-channel UART grayscale sensor. Host-driven request-response protocol:
1. Host sends `0x57 + ID` (2 bytes)
2. Sensor responds with 41-byte frame: `0x75` + 37 data bytes + `0x26` tail
3. DMA idle-line reception on USART1 captures the response

`gray_sensor_poll()` blocks ~4 ms per call (TX + sensor response time). Call it from line_track_task at 5 ms period. Returns 1 when a fresh frame is decoded. Output via `get_gray_sensor_point()`: bitmask, normalized centroid position (-1.0 left … +1.0 right), black channel count, online flag.

### Layer 2 — Line Tracking (`application/line_track_task`)

Periodic state machine (5ms). 5 states:
- `LOST_LINE` → `TRACKING` → `INTERSECTION_APPROACH` → `INTERSECTION_CROSS` → `TRACKING`
- `STOP` reachable from any state via `NAV_ACTION_STOP` (immediate).

Intersection detection uses bitmask cluster analysis (geometry-based, not type-specific). APPROACH state debounces with `approach_confirm_cnt` (needs ≥6 consecutive frames or all-black trigger). On CROSS entry, dead-reckoning parameters are looked up from `DR_TABLE[]` and the gyro yaw reference is captured (`cross_yaw_start`).

**CROSS→TRACKING exit (3 paths):**
- **Turn action** (`yaw_threshold > 0`): gyro yaw accumulates until `|dyaw| ≥ yaw_threshold`
- **Forward action** (`yaw_threshold == 0`): sensor line re-acquisition (branch-aware — checks left/right/center based on `expected_branch`)
- **Timeout**: 10 s safety escape to TRACKING
- **Lost line**: 0 lines detected → LOST_LINE (cancels dead reckoning)

After exit, `just_crossed` flag blocks intersection re-detection until the line is centered (or 500ms timeout), then fires `exit_event` for `navigate_task` to advance the path.

### Layer 3 — Navigation (`application/navigate_task`)

Path is a data array — to change routes, change the array, not the code. Path steps are consumed at each intersection-exit event. `navigate_task` polls `line_track_got_exit_event()` at 20ms, advances the step index, and issues the next action via `line_track_set_nav_action()`. Path terminates with `NAV_ACTION_STOP`.

### Chassis (`application/chassis_task`)

**Skid-steer (differential drive)** with 4 M3508 motors — NOT mecanum. Rubber tires cannot crab sideways, so there is no lateral (vy) motion. Kinematics:

```
v_left  = vx - wz × WHEEL_BASE / 2    (motors 0,1 — CAN IDs 0x201, 0x202)
v_right = vx + wz × WHEEL_BASE / 2    (motors 2,3 — CAN IDs 0x203, 0x204)
```

Each motor has its own speed PID loop. `chassis_set_velocity(vx, wz)` is the public API — all other tasks command the chassis through this.

## Task List (priority order)

| Task | Priority | Stack | Period | Purpose |
|------|----------|-------|--------|---------|
| INS_task | Realtime | 1024 | 1ms | AHRS attitude estimation (BMI088+IST8310) |
| line_track_task | Normal | 512 | 5ms | Line-follow FSM + PID control (was High, lowered to avoid starving chassis) |
| chassis_task | AboveNormal | 512 | 2ms | Skid-steer chassis PID + CAN motor control |
| calibrate_task | Normal | 512 | once | IMU sensor calibration |
| detect_task | Normal | 256 | 10ms | Device heartbeat/watchdog |
| led_flow_task | Normal | 256 | — | RGB LED status indicator |
| navigate_task | Normal | 256 | 20ms | Path step planner |
| voltage_task | Normal | 128 | — | Battery voltage ADC |
| oled_task | Low | 256 | 100ms | Debug display (state, position, errors) |
| debug_console_task | Normal | 512 | 5ms | Interactive command console via USART6 |

## Data Flow

```
gray sensor USART1 → DMA → gray_sensor_on_rx_idle()
    → gray_sensor_t { position, bitmask, line_count }
        → line_track_task (state machine + PID)
            → chassis_set_velocity(vx, wz)
                → chassis_task (motor PID + skid-steer → CAN)
                    → M3508 motors

INS_task (BMI088+IST8310 → AHRS)
    → euler angles + gyro data
        → line_track_task (dead reckoning yaw tracking)
        → obstacle/collision detection (to be implemented)

navigate_task (path table)
    → polls line_track_got_exit_event()
        → line_track_set_nav_action(next_action)
            → consumed by line_track_task at next intersection

debug_console_task (USART6 RX interrupt + ring buffer)
    → parses commands → calls line_track_set_*() / navigate_load_path()
```

## Debug Console (Runtime Tuning)

`debug_console_task` provides an interactive command interface via USART6 (PG14, 115200 8N1) or USB CDC. This is the primary tuning interface — parameters can be changed **at runtime** without recompiling.

Connect with any serial terminal (PuTTY, screen, minicom) and type `help`:

| Command | Description |
|---------|-------------|
| `help` | Show all commands |
| `state` | FSM state, sensor position, navigation progress |
| `gray` / `grayonce` | Gray sensor data (continuous/once) |
| `imu` / `imuonce` | IMU angles, gyro, accel, mag (continuous/once) |
| `motor` / `motoronce` | Motor RPM for all 4 wheels (continuous/once) |
| `pid [kp ki kd]` | Show or set line-follow PID gains (Kp, Ki, Kd) |
| `speed [value]` | Show or set base tracking speed (m/s) |
| `slow [value]` | Show or set slow speed through intersections (m/s) |
| `path [name]` | List available paths or switch route (basic/sharp/obtuse/uturn/fork/complex) |
| `stop` | Emergency stop |
| `start` | Resume after stop |
| `reset` | System reset via NVIC |
| `save` / `load` | Save/load parameters to Flash (stub — not yet implemented) |
| `clear` | Clear terminal (ANSI escape) |
| `log [on/off]` | Toggle periodic sensor/motor logging |
| `trace [on/off]` | Toggle FSM state transition trace output |

**Runtime tuning API** (callable from console or code):
```c
line_track_set_pid(kp, ki, kd);        // Adjust line-follow PID
line_track_set_base_speed(speed);       // Adjust tracking speed
line_track_set_slow_speed(speed);       // Adjust intersection speed
line_track_set_trace(enable);           // Enable FSM transition logging
line_track_get_pid(&kp, &ki, &kd);     // Read current PID
line_track_get_base_speed();            // Read current base speed
```

## Navigation Actions (nav_types.h)

All actions available for path arrays and `DR_TABLE[]`:

| Action | Value | Description |
|--------|-------|-------------|
| `NAV_ACTION_FORWARD` | 0 | Go straight through intersection |
| `NAV_ACTION_TURN_LEFT` | 1 | Standard 90° left turn |
| `NAV_ACTION_TURN_RIGHT` | 2 | Standard 90° right turn |
| `NAV_ACTION_STOP` | 3 | Immediate stop (path terminator) |
| `NAV_ACTION_U_TURN` | 10 | 180° U-turn |
| `NAV_ACTION_AROUND_LEFT` | 11 | Wide left loop turn |
| `NAV_ACTION_AROUND_RIGHT` | 12 | Wide right loop turn |
| `NAV_ACTION_SHARP_LEFT` | 20 | Sharp-angle left |
| `NAV_ACTION_SHARP_RIGHT` | 21 | Sharp-angle right |
| `NAV_ACTION_OBTUSE_LEFT` | 30 | Obtuse-angle left |
| `NAV_ACTION_OBTUSE_RIGHT` | 31 | Obtuse-angle right |
| `NAV_ACTION_FORK_LEFT` | 40 | Take left branch at fork |
| `NAV_ACTION_FORK_RIGHT` | 41 | Take right branch at fork |
| `NAV_ACTION_FORK_CENTER` | 42 | Take center branch at fork |
| `NAV_ACTION_FORK_2ND_LEFT` | 43 | Take second-from-left branch |
| `NAV_ACTION_FORK_2ND_RIGHT` | 44 | Take second-from-right branch |
| `NAV_ACTION_SLOW_DOWN` | 50 | Reduce speed |
| `NAV_ACTION_SPEED_UP` | 51 | Increase speed |
| `NAV_ACTION_AVOID_LEFT` | 60 | Avoid obstacle to the left |
| `NAV_ACTION_AVOID_RIGHT` | 61 | Avoid obstacle to the right |
| `NAV_ACTION_MERGE_LEFT` | 62 | Merge from left |
| `NAV_ACTION_MERGE_RIGHT` | 63 | Merge from right |
| `NAV_ACTION_AUTO_SELECT` | 70 | FSM auto-picks direction from sensor |
| `NAV_ACTION_NONE` | 99 | No pending action (cleared after exit) |

Each action has corresponding dead-reckoning parameters in `DR_TABLE[]` (`line_track_task.c`): `{vx, wz, yaw_th}`. `VX_SLOW` sentinel means "use `current_slow_speed`." Actions with `yaw_th == 0` use sensor line re-acquisition instead of gyro-based exit.

## Adding a New Route

### At compile time — edit `Src/main.c` USER CODE PD section:

```c
static const navigate_action_t new_path[] = {
    NAV_ACTION_FORWARD,
    NAV_ACTION_TURN_LEFT,
    NAV_ACTION_FORK_RIGHT,
    NAV_ACTION_STOP,  // terminator
};
navigate_load_path(new_path, sizeof(new_path) / sizeof(new_path[0]));
```

### At runtime — use the debug console:

```
path basic      # 4-step basic route
path complex    # 10-step route with all intersection types
```

### Adding a new named path — edit `debug_console.c`:
1. Add a `static const navigate_action_t path_<name>[]` array
2. Add a `test_path_t` entry in the `test_paths[]` table

## Tuning Guide

| Parameter | Location | What it does |
|-----------|----------|--------------|
| `threshold` (80) | `gray_sensor_init()` | Black/white discrimination (0-255) |
| `GRAY_HEADER1/2` | `gray_sensor.h` | Frame sync bytes (match sensor protocol) |
| `LT_KP` (1.5), `LT_KI` (0.001), `LT_KD` (0.8), `LT_MAX_WZ` (4.0) | `line_track_task.h` | Line-follow PID gains |
| `LT_BASE_SPEED` (1.0) | `line_track_task.h` | Forward speed while tracking (m/s) |
| `LT_SLOW_SPEED` (0.2) | `line_track_task.h` | Speed through intersections (m/s) |
| `WHEEL_BASE` (0.35), `MOTOR_DISTANCE_TO_CENTER` | `chassis_task.h` | Wheel geometry — tune for your chassis |
| `M3508_MOTOR_RPM_TO_VECTOR` | `chassis_task.h` | RPM→m/s conversion (depends on wheel radius) |
| `CHASSIS_CONTROL_TIME_MS` (2) | `chassis_task.h` | Chassis control period (ms) |
| `M3505_MOTOR_SPEED_PID_*` | `chassis_task.h` | Per-motor speed PID gains |
| `configENABLE_FPU` (1) | `Inc/FreeRTOSConfig.h` | Must be 1 — saves FPU registers on context switch |
| `DR_TABLE[]` | `line_track_task.c` | Dead-reckoning params (vx, wz, yaw_th) per action — lookup by action enum index |

**Tuning workflow:** Connect serial terminal → try `pid`/`speed`/`slow` commands at runtime → once happy, copy values back to the `#define`s in `line_track_task.h`. The runtime `line_track_set_*()` values override the compile-time defaults until next reset.
