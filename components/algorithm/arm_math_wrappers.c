/**
 * arm_math_wrappers.c — lightweight CMSIS-DSP shim for GCC builds.
 *
 * Replaces arm_cortexM4lf_math.lib (Keil format) with standard libm.
 * The M4F FPU makes sinf/cosf fast enough for this application.
 */
#include <math.h>
#include "arm_math.h"

float32_t arm_sin_f32(float32_t x) { return sinf(x); }
float32_t arm_cos_f32(float32_t x) { return cosf(x); }
