# AGENTS.md

语言: 中文（提问和回答均使用中文）

Bare-metal FreeRTOS firmware for an autonomous line-following smart car (STM32F407, RoboMaster Dev Board Type C). No tests, no CI, no unit test framework.

## Build & Flash

```bash
make -j$(nproc)        # build → build/smart_car.elf
make flash             # flash via OpenOCD (CMSIS-DAP)
make clean             # remove build/
make reset             # reset MCU
make gdb               # GDB session on :3333
```

CMake alternative:
```bash
cmake -B build_cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
cmake --build build_cmake -j$(nproc)
```

## Critical Gotchas

- **`-u _printf_float` / `-u _scanf_float`** are required in linker flags (both Make & CMake). Without them `printf("%f")` / `sscanf("%f")` silently produce garbage.
- **`AHRS.lib`** and **`arm_cortexM4lf_math.lib`** in `components/algorithm/` are **Keil/ARMCC format** — cannot link with GCC. Use CMSIS-DSP from source or a GCC-prebuilt `.a`.
- **`configENABLE_FPU 0`** in `Inc/FreeRTOSConfig.h` — the Tuning Guide says this should be `1` to save FPU registers on context switch. If FPU crashes occur, this is the first place to check.
- **CubeMX auto-generates** `Src/` and `Inc/` files. Edits outside `/* USER CODE BEGIN/END */` sections will be wiped on regeneration.

## Architecture

Three-layer pipeline running at different rates:

| Rate | Module | File |
|------|--------|------|
| 5ms | Line-follow FSM | `application/line_track_task.c` |
| 2ms | Chassis PID + CAN | `application/chassis_task.c` |
| 20ms | Path planner | `application/navigate_task.c` |
| 1ms | AHRS attitude | `application/INS_task.c` |
| 5ms | Debug console | `application/debug_console.c` |

Skid-steer (differential drive, **no** lateral/vy motion). Public API: `chassis_set_velocity(vx, wz)`.

Paths are data arrays of `navigate_step_t` (defined in `application/nav_types.h`), terminated by `NAV_ACTION_STOP`. Edit `Src/main.c` USER CODE to change routes at compile time, or use the debug console at runtime.

## Path System

所有命名路径定义在 **`application/paths.c`** 的 `g_path_registry[]` 中。添加新路径：

1. 在 `paths.c` 中定义 `static const navigate_step_t s_<name>[] = { ... };`
2. 注册: `ENTRY(name)` 加入 `g_path_registry[]`

**按名字引用路径**（`main.c` 或任意路径数组）:
```c
static const navigate_step_t my_route[] = {
    { .ref_name = "basic" },
    { .ref_name = "sharp" },
    { NAV_ACTION_STOP },
};
```
`navigate_step_t` 结构:
```c
typedef struct {
    navigate_action_t action;
    uint8_t           param;      // e.g. voice track for CLIMB_PLATFORM
    const char       *ref_name;   // non-NULL → 按名字执行已注册子路径
} navigate_step_t;
```

## Tuning (no recompile needed)

Connect via USART6 (PG14, 115200 8N1) or USB CDC. Commands: `pid`, `speed`, `slow`, `path`, `gray`, `imu`, `motor`, `trace`, `state`, `stop`, `start`. Full list via `help`.

Cross-reference against `#define`s in `line_track_task.h` when copying runtime-tuned values back to code.

## Dead Reckoning Parameters

`DR_TABLE[]` in `line_track_task.c` maps every action to `{vx, wz, yaw_th}`. `VX_SLOW` means "use current slow speed." Actions with `yaw_th == 0` use sensor line re-acquisition instead of gyro yaw to exit a crossing.
