/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       voltage_task.c/h
  * @brief      24v power voltage ADC task, get voltage and calculate
  electricity
  *             percentage.24电源电压ADC任务,获取电压并且计算电量百分比.
  * @note       when power is not derectly link to delelopment, please change
  VOLTAGE_DROP
  *             当电源不直连开发板,请修改VOLTAGE_DROP
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "voltage_task.h"
#include "cmsis_os.h"
#include "main.h"

#include "bsp_adc.h"
#include "bsp_buzzer.h"
#include "user_lib.h"

/* 6S 电池电压阈值 */
#define LOW_BATTER_VOLTAGE 23.0f /* 6×3.5V 欠压报警 */
#define VOLTAGE_DROP 0.00f

static fp32 calc_battery_percentage(float voltage);

fp32 battery_voltage = 24.0f; /* 6S 安全默认值, 防启动误报 */
fp32 electricity_percentage;

/**
 * @brief          power ADC and calculate electricity percentage
 * @param[in]      pvParameters: NULL
 * @retval         none
 */
/**
 * @brief          电源采样和计算电源百分比
 * @param[in]      pvParameters: NULL
 * @retval         none
 */
void battery_voltage_task(void const *argument) {
  osDelay(1000);
  // use inner 1.2v to calbrate
  init_vrefint_reciprocal();
  while (1) {
    battery_voltage = get_battery_voltage() + VOLTAGE_DROP;
    electricity_percentage = calc_battery_percentage(battery_voltage);

    /* 低电压报警: 启动后 3 秒才启用，每 5 秒一次 */
    {
      static uint32_t last_alarm_ms = 0;
      uint32_t now = osKernelSysTick();
      if (now > 3000 && battery_voltage < LOW_BATTER_VOLTAGE &&
          now - last_alarm_ms > 5000) {
        buzzer_low_battery();
        last_alarm_ms = now;
      }
    }

    osDelay(100);
  }
}

static fp32 calc_battery_percentage(float voltage) {
  fp32 percentage;
  fp32 voltage_2 = voltage * voltage;
  fp32 voltage_3 = voltage_2 * voltage;

  if (voltage < 19.5f) {
    percentage = 0.0f;
  } else if (voltage < 21.9f) {
    percentage =
        0.005664f * voltage_3 - 0.3386f * voltage_2 + 6.765f * voltage - 45.17f;
  } else if (voltage < 25.5f) {
    percentage =
        0.02269f * voltage_3 - 1.654f * voltage_2 + 40.34f * voltage - 328.4f;
  } else {
    percentage = 1.0f;
  }
  if (percentage < 0.0f) {
    percentage = 0.0f;
  } else if (percentage > 1.0f) {
    percentage = 1.0f;
  }
  // another formulas
  // 另一套公式
  //    if(voltage < 19.5f)
  //    {
  //        percentage = 0.0f;
  //    }
  //    else if(voltage < 22.5f)
  //    {
  ////        percentage = 0.05776f * (voltage - 22.5f) * (voltage_2 - 39.0f *
  ///voltage + 383.4f) + 0.5f;
  //        percentage = 0.05021f * voltage_3 - 3.075f * voltage_2 + 62.77f *
  //        voltage - 427.02953125f;
  //    }
  //    else if(voltage < 25.5f)
  //    {
  ////        percentage = 0.01822f * (voltage - 22.5f) * (voltage_2 - 52.05f *
  ///voltage + 637.0f) + 0.5f;
  //        percentage = 0.0178f * voltage_3 - 1.292f * voltage_2 + 31.41f *
  //        voltage - 254.903125f;
  //    }
  //    else
  //    {
  //        percentage = 1.0f;
  //    }

  return percentage;
}

uint16_t get_battery_percentage(void) {
  return (uint16_t)(electricity_percentage * 100.0f);
}
