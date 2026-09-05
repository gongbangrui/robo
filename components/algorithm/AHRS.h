#ifndef AHRS_H
#define AHRS_H

#include "AHRS_middleware.h"

/**
  * @brief 根据加速度的数据，磁力计的数据进行四元数初始化
  * @param [in]      需要初始化的四元数数组
  * @param [in]      需要初始化的四元数数组
  * @param [in]      需要初始化的四元数数组
  * @retval 返回空
  */
extern void AHRS_init(fp32 quat[4], const fp32 accel[3], const fp32 mag[3]);

/**
  * @brief 根据加速度的数据，磁力计的数据进行四元数初始化
  * @param [in]      需要初始化的四元数数组
  * @param [in]      需要初始化的四元数数组
  * @param [in]      需要初始化的四元数数组
  * @param [in]      需要初始化的四元数数组
  * @param [in]      需要初始化的四元数数组
  * @retval 1: 更新成功, 0:更新失败
  */
extern bool_t AHRS_update(fp32 quat[4], const fp32 timing_time, const fp32 gyro[3], const fp32 accel[3], const fp32 mag[3]);

/**
  * @brief 根据加速度的数据，磁力计的数据进行四元数初始化
  * @param [in]      需要初始化的四元数数组
  * @retval 返回空
  */
extern fp32 get_yaw(const fp32 quat[4]);

/**
  * @brief 根据加速度的数据，磁力计的数据进行四元数初始化
  * @param [in]      需要初始化的四元数数组
  * @retval 返回空
  */
extern fp32 get_pitch(const fp32 quat[4]);
/**
  * @brief 根据加速度的数据，磁力计的数据进行四元数初始化
  * @param [in]      需要初始化的四元数数组
  * @retval 返回空
  */
extern fp32 get_roll(const fp32 quat[4]);

/**
  * @brief 根据加速度的数据，磁力计的数据进行四元数初始化
  * @param [in]      需要初始化的四元数数组
  * @param [in]      需要初始化的四元数数组
  * @param [in]      需要初始化的四元数数组
  * @param [in]      需要初始化的四元数数组
  */
extern void get_angle(const fp32 quat[4], fp32 *yaw, fp32 *pitch, fp32 *roll);
/**
  * @brief 根据加速度的数据，磁力计的数据进行四元数初始化
  * @param [in]      需要初始化的四元数数组
  * @retval 返回空
  */
extern fp32 get_carrier_gravity(void);

#endif
