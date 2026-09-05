/**
 * AHRS.c — Mahony AHRS filter.
 *
 * PI controller fuses gyro, accel, and mag via cross-product error feedback.
 * Reference: x-io Technologies / Mahony et al. (2008).
 *
 * Quaternion: q = [q0, q1, q2, q3] = [w, x, y, z]
 * Earth gravity: [0, 0, 1] (vertical up)
 */
#include "AHRS.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

/* ---- Mahony PI gains ---- */
#define TWO_KP 0.2f    /* 2 * proportional gain (Kp = 0.1) */
#define TWO_KI 0.05f   /* 2 * integral gain — estimates & cancels gyro bias drift */

#define USE_MAGNETOMETER 0 /* IST8310 数据不可靠(mz=481uT异常), 关闭修正, yaw 靠 gyro+TWO_KI */
#define MAG_WEIGHT 0.1f  /* reduce mag correction relative to accel */

/* ---- local state ---- */
static fp32 q0, q1, q2, q3;
static fp32 carrier_gravity;
static fp32 integralFBx, integralFBy, integralFBz;

/* ---- fast inverse sqrt ---- */
static fp32 inv_sqrt(fp32 x) {
  fp32 halfx = 0.5f * x;
  fp32 y = x;
  long i = *(long *)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(fp32 *)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

/* ---- init ---- */

void AHRS_init(fp32 quat[4], const fp32 accel[3], const fp32 mag[3]) {
  fp32 ax = accel[0], ay = accel[1], az = accel[2];
  fp32 roll, pitch;
  fp32 cosRoll, sinRoll, cosPitch, sinPitch;
  fp32 norm;

  pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
  roll = atan2f(ay, az);

  cosRoll = cosf(roll * 0.5f);
  sinRoll = sinf(roll * 0.5f);
  cosPitch = cosf(pitch * 0.5f);
  sinPitch = sinf(pitch * 0.5f);

  q0 = cosRoll * cosPitch;
  q1 = sinRoll * cosPitch;
  q2 = cosRoll * sinPitch;
  q3 = -sinRoll * sinPitch;

  norm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= norm; q1 *= norm; q2 *= norm; q3 *= norm;

#if USE_MAGNETOMETER
  {
    fp32 mx = mag[0], my = mag[1], mz = mag[2];
    norm = inv_sqrt(mx * mx + my * my + mz * mz);
    if (norm > 1e-12f) {
      mx *= norm; my *= norm; mz *= norm;

      fp32 _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2;
      fp32 q0q0 = q0 * q0, q1q1 = q1 * q1;

      fp32 hx = mx * q0q0 - _2q0 * my * q3 + _2q0 * mz * q2 + mx * q1q1 +
                _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2 * q2 - mx * q3 * q3;
      fp32 hy = _2q0 * mx * q3 + my * q0q0 - _2q0 * mz * q1 +
                _2q1 * mx * q2 - my * q1q1 + my * q2 * q2 +
                _2q2 * mz * q3 - my * q3 * q3;

      fp32 mag_yaw = atan2f(hy, hx);
      fp32 cy = cosf(mag_yaw * 0.5f);
      fp32 sy = sinf(mag_yaw * 0.5f);

      fp32 q0n = cy * q0 - sy * q3;
      fp32 q1n = cy * q1 - sy * q2;
      fp32 q2n = cy * q2 + sy * q1;
      fp32 q3n = cy * q3 + sy * q0;

      q0 = q0n; q1 = q1n; q2 = q2n; q3 = q3n;
      norm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
      q0 *= norm; q1 *= norm; q2 *= norm; q3 *= norm;
    }
  }
#else
  (void)mag;
#endif

  quat[0] = q0; quat[1] = q1; quat[2] = q2; quat[3] = q3;
  carrier_gravity = sqrtf(ax * ax + ay * ay + az * az);
  integralFBx = integralFBy = integralFBz = 0.0f;
}

/* ---- update ---- */

bool_t AHRS_update(fp32 quat[4], const fp32 timing_time,
                    const fp32 gyro[3], const fp32 accel[3], const fp32 mag[3]) {
  fp32 gx = gyro[0], gy = gyro[1], gz = gyro[2];
  fp32 ax = accel[0], ay = accel[1], az = accel[2];
  fp32 recipNorm;
  fp32 halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
  fp32 qa, qb, qc;
#if !USE_MAGNETOMETER
  (void)mag;
#endif

  /* Normalize accelerometer */
  recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
  if (recipNorm > 1e-12f) {
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

    /* Estimated gravity direction (body frame):
     *   halfv = last column of R(q)^T / 2 */
    fp32 halfvx = q1 * q3 - q0 * q2;
    fp32 halfvy = q0 * q1 + q2 * q3;
    fp32 halfvz = q0 * q0 - 0.5f + q3 * q3;

    /* Accel error = measured × estimated (cross product / 2) */
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);
  }

#if USE_MAGNETOMETER
  {
    fp32 mx = mag[0], my = mag[1], mz = mag[2];
    recipNorm = inv_sqrt(mx * mx + my * my + mz * mz);
    if (recipNorm > 1e-12f) {
      mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

      fp32 q0q0 = q0 * q0, q1q1 = q1 * q1;
      fp32 q2q2 = q2 * q2, q3q3 = q3 * q3;
      fp32 _2q0 = 2.0f * q0, _2q1 = 2.0f * q1;
      fp32 _2q2 = 2.0f * q2;

      /* Earth-frame horizontal field: h = R(q) * m */
      fp32 hx = mx * q0q0 - _2q0 * my * q3 + _2q0 * mz * q2 + mx * q1q1 +
                _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
      fp32 hy = _2q0 * mx * q3 + my * q0q0 - _2q0 * mz * q1 +
                _2q1 * mx * q2 - my * q1q1 + my * q2q2 +
                _2q2 * mz * q3 - my * q3q3;

      /* Reference directions (earth frame) */
      fp32 _2bx = sqrtf(hx * hx + hy * hy);
      fp32 _2bz = _2q0 * mx * q2 - _2q0 * my * q1 + mz * q0q0 +
                  _2q1 * mx * q3 - mz * q1q1 + _2q2 * my * q3 -
                  mz * q2q2 + mz * q3q3;

      /* Estimated mag direction (body frame):
       *   halfw = R(q)^T * [_2bx, 0, _2bz] */
      fp32 halfwx = _2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1 * q3 - q0 * q2);
      fp32 halfwy = _2bx * (q1 * q2 - q0 * q3) + _2bz * (q0 * q1 + q2 * q3);
      fp32 halfwz = _2bx * (q0 * q2 + q1 * q3) + _2bz * (0.5f - q1q1 - q2q2);

      /* Mag error = measured × estimated */
      halfex += MAG_WEIGHT * (my * halfwz - mz * halfwy);
      halfey += MAG_WEIGHT * (mz * halfwx - mx * halfwz);
      halfez += MAG_WEIGHT * (mx * halfwy - my * halfwx);
    }
  }
#endif

  /* PI controller: correct gyro with cross-product error */
  if (TWO_KI > 0.0f) {
    integralFBx += TWO_KI * halfex * timing_time;
    integralFBy += TWO_KI * halfey * timing_time;
    /* No absolute reference on z (yaw) without magnetometer:
     * integrating halfez here accumulates a fake yaw bias during tilt
     * correction transients and causes continuous yaw drift. Skip it. */
    integralFBz = 0.0f;
    gx += integralFBx;
    gy += integralFBy;
  } else {
    integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;
  }
  gx += TWO_KP * halfex;
  gy += TWO_KP * halfey;
  gz += TWO_KP * halfez;

  /* Integrate quaternion: q += 0.5 * q ⊗ ω * dt */
  gx *= (0.5f * timing_time);
  gy *= (0.5f * timing_time);
  gz *= (0.5f * timing_time);
  qa = q0; qb = q1; qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  /* Normalize */
  recipNorm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm; q1 *= recipNorm;
  q2 *= recipNorm; q3 *= recipNorm;

  quat[0] = q0; quat[1] = q1; quat[2] = q2; quat[3] = q3;
  carrier_gravity = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
  return 1;
}

/* ---- quaternion → Euler angles ---- */

void get_angle(const fp32 quat[4], fp32 *yaw, fp32 *pitch, fp32 *roll) {
  fp32 q0 = quat[0], q1 = quat[1], q2 = quat[2], q3 = quat[3];

  *yaw = -atan2f(2.0f * (q1 * q2 + q0 * q3),
                 q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
  *pitch = -asinf(2.0f * (q1 * q3 - q0 * q2));
  *roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                 q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
}

fp32 get_yaw(const fp32 quat[4])    { fp32 y, p, r; get_angle(quat, &y, &p, &r); return y; }
fp32 get_pitch(const fp32 quat[4])  { fp32 y, p, r; get_angle(quat, &y, &p, &r); return p; }
fp32 get_roll(const fp32 quat[4])   { fp32 y, p, r; get_angle(quat, &y, &p, &r); return r; }
fp32 get_carrier_gravity(void)      { return carrier_gravity; }
