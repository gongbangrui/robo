# BMI088 Accelerometer Regression Analysis

## Symptom

After commit `ec54174`, the BMI088 accelerometer data was always zero (ax=ay=az=0.000). Gyro and magnetometer were unaffected.

## Root Cause

Commit `ec54174` replaced the manual BMI088 accel init sequence with a call to `BMI088_init()`. The project's `BMI088_init()` added **self-test gating** not present in DJI's reference code:

```c
// Project (broken) — self-test gates init
uint8_t BMI088_init(void) {
    if (bmi088_accel_self_test() != BMI088_NO_ERROR)  // ← fails
        error |= BMI088_SELF_TEST_ACCEL_ERROR;        // ← accel init skipped
    else
        error |= bmi088_accel_init();                 // ← never reached
}

// DJI reference (working) — no self-test
uint8_t BMI088_init(void) {
    error |= bmi088_accel_init();   // always runs
    error |= bmi088_gyro_init();    // always runs
}
```

The self-test fails because `BMI088_LONG_DELAY_TIME` is 80ms, but this board's BMI088 chip needs >100ms after soft reset. The self-test's 80ms delay leaves the chip in a post-reset state where chip ID reads 0xFF, causing `BMI088_NO_SENSOR` return and skipping accel init entirely.

## Fix

Aligned `BMI088driver.c` with DJI's reference (`13.spi_bmi088/component/devices/BMI088driver.c`):

1. **Removed `bmi088_accel_self_test()` and `bmi088_gyro_self_test()`** — not in DJI reference, not needed for normal operation
2. **Restored DJI's `BMI088_init()`** — calls `bmi088_accel_init()` / `bmi088_gyro_init()` directly
3. **Replaced `BMI088Middleware.c` delay functions** — SysTick-based `BMI088_delay_us()` (DJI reference) instead of RTOS `osDelay`/`delay_us`
4. **Simplified `INS_task.c`** — removed 70+ lines of manual register writes and debug prints, replaced with single `BMI088_init()` call
5. **Cleaned up `BMI088driver.h`** — removed unused exports (`bmi088_accel_self_test`, `bmi088_gyro_self_test`, `BMI088_read_gyro_who_am_i`, etc.)

## Files Changed

| File | Change |
|------|--------|
| `components/devices/BMI088driver.c` | Replaced with DJI reference (no self-test) |
| `components/devices/BMI088driver.h` | Removed self-test and unused exports |
| `components/devices/BMI088Middleware.c` | SysTick-based delays (DJI reference) |
| `application/INS_task.c` | Simplified init to `BMI088_init()` |
