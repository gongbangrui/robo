/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define VOICE5_Pin GPIO_PIN_0
#define VOICE5_GPIO_Port GPIOF
#define RSTN_IST8310_Pin GPIO_PIN_6
#define RSTN_IST8310_GPIO_Port GPIOG
#define LED_R_Pin GPIO_PIN_12
#define LED_R_GPIO_Port GPIOH
#define DRDY_IST8310_Pin GPIO_PIN_3
#define DRDY_IST8310_GPIO_Port GPIOG
#define DRDY_IST8310_EXTI_IRQn EXTI3_IRQn
#define ADC_BAT_Pin GPIO_PIN_10
#define ADC_BAT_GPIO_Port GPIOF
#define LED_G_Pin GPIO_PIN_11
#define LED_G_GPIO_Port GPIOH
#define LED_B_Pin GPIO_PIN_10
#define LED_B_GPIO_Port GPIOH
#define HW0_Pin GPIO_PIN_0
#define HW0_GPIO_Port GPIOC
#define HW1_Pin GPIO_PIN_1
#define HW1_GPIO_Port GPIOC
#define HW2_Pin GPIO_PIN_2
#define HW2_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_14
#define BUZZER_GPIO_Port GPIOD
#define KEY_Pin GPIO_PIN_0
#define KEY_GPIO_Port GPIOA
#define CS1_ACCEL_Pin GPIO_PIN_4
#define CS1_ACCEL_GPIO_Port GPIOA
#define INT1_ACCEL_Pin GPIO_PIN_4
#define INT1_ACCEL_GPIO_Port GPIOC
#define INT1_ACCEL_EXTI_IRQn EXTI4_IRQn
#define IR_SENSOR3_Pin GPIO_PIN_13
#define IR_SENSOR3_GPIO_Port GPIOE
#define INT1_GYRO_Pin GPIO_PIN_5
#define INT1_GYRO_GPIO_Port GPIOC
#define INT1_GYRO_EXTI_IRQn EXTI9_5_IRQn
#define IR_SENSOR1_Pin GPIO_PIN_9
#define IR_SENSOR1_GPIO_Port GPIOE
#define IR_SENSOR2_Pin GPIO_PIN_11
#define IR_SENSOR2_GPIO_Port GPIOE
#define VOICE1_Pin GPIO_PIN_12
#define VOICE1_GPIO_Port GPIOB
#define VOICE2_Pin GPIO_PIN_13
#define VOICE2_GPIO_Port GPIOB
#define CS1_GYRO_Pin GPIO_PIN_0
#define CS1_GYRO_GPIO_Port GPIOB
#define VOICE3_Pin GPIO_PIN_14
#define VOICE3_GPIO_Port GPIOB
#define VOICE4_Pin GPIO_PIN_15
#define VOICE4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/*
 * ═════════════════════════════════════════════════════════════
 * 视觉 & 路由 API 速查
 * ═════════════════════════════════════════════════════════════
 *
 * ── 路由 (main.c 中写 if/else 选路) ──────────────────────────
 *   void run("name")        阻塞执行命名路径, 完成后返回
 *   void run_path(path, len) 同上, 直接传路径数组
 *
 * ── 红绿灯 ───────────────────────────────────────────────────
 *   vision_last_tl_green()    最近一次红绿灯是否绿 (1=绿 0=否)
 *   vision_last_tl_green_idx() 最近绿灯的索引 (0xFF=没有)
 *   vision_find_first_green()  全部结果中第一个绿灯索引 (0xFF=全红)
 *
 * ── 二维码 ───────────────────────────────────────────────────
 *   vision_qr_get_target(qi, n) 第 qi 个QR(0号=第一个)的第n个目标号
 *
 * ── 通用查询 ─────────────────────────────────────────────────
 *   vision_result_get_cmd(idx)  第idx条的命令码 (0x01=TL 0x02=QR)
 *   vision_result_get_byte(idx,n) 第idx条第n字节
 *   vision_result_clear()       清空全部结果
 *   g_vision_results[]         结果数组 {{cmd,data[4],len}}
 *   g_vision_result_count      当前结果条数
 *
 * ── 红外 ────────────────────────────────────────────────────
 *   get_ir_sensor_point() → .front .left .right .extra (0=触发)
 *
 * ── 用法模板 ─────────────────────────────────────────────────
 *   void my_route(void) {
 *       run("1_2");
 *       if (vision_last_tl_green()) run("to_4"); else run("to_3");
 *       uint8_t t = vision_qr_get_target(0, 0);
 *       if (t == 3) run("path_3"); else run("path_5");
 *   }
 * ═════════════════════════════════════════════════════════════
 */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
