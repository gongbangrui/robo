/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "rng.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "actions.h"
#include "bsp_buzzer.h"
#include "bsp_can.h"
#include "bsp_delay.h"
#include "bsp_usart.h"

#include "CAN_receive.h"
#include "gray_sensor.h"

#include "INS_task.h"
#include "calibrate_task.h"
#include "chassis_behaviour.h"
#include "chassis_task.h"
#include "detect_task.h"
#include "led_flow_task.h"
#include "line_track_task.h"
#include "navigate_task.h"
#include "paths.h"
#include "servo_ctrl.h"
#include "usb_cdc_task.h"
#include "vision_module.h"
#include "voltage_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 * ══════════════════════════════════════════════════════════════
 * 辅助：上下平台通用步骤
 * ══════════════════════════════════════════════════════════════
 *
 * 上平台 — 无线直行 → 红外触发 → 微调距离 → 舵机
 * 下平台 — 180° 掉头 → 前进寻线
 */

static void platform_enter(void) {
  move(300, 0.15f, 5000);  // 上坡 40cm, 不管灰度
  wait_ir(5000);           // 慢速至红外触发
  move(200, 0.10f, 5000);  // 前走 5cm
  move(-200, 0.10f, 5000); // 后退 3cm
  servo(SERVO_CH_PLATFORM, SERVO_PLATFORM_ACTIVE_US);
  osDelay(800);
  servo(SERVO_CH_PLATFORM, SERVO_PLATFORM_HOME_US);
}

static void platform_exit(void) {
  find_line(3000); // 前进寻线
}

/*
 * ══════════════════════════════════════════════════════════════
 * 示例1: 完整竞赛流程 (平台1 → 扫码 → 分支 → 数字识别)
 * ══════════════════════════════════════════════════════════════
 *
 * 触发: debug console 输入 msn comp
 *
 * 依赖路径 (见 paths.c):
 *   "1_2"       — 平台1→口→平台  p2
 *   "p2_men4"   — 平台2→口→3或4号门口
 *   "men4_men3"  — 4号门口→口→3号门口
 *   "men3_men2"  — 3号门口→口→2号门口
 */

static const char *platform_sum_route(int sum) {
  switch (sum) {
  case 1:
    return "men3_men1";
  case 2:
    return "men3_men2";
  case 3:
    return "men2_men3";
  default:
    return "men2_men1"; /* fallback */
  }
}

void mission_comp(void) {
  /* ── 上平台2, 扫码 ── */
  platform_exit();
  run("p1_p2");
  platform_enter();

  turn(170);
  platform_exit();
  run("p2_z1");
  turn(180);

  // /* ── 扫码: qr[0..2] 三字节全用 ──
  //  * qr_scan 阻塞等结果, 成功返回 1, 填充 3 个 ASCII 字节。
  //  * 路由判断直接取 qr[0]/qr[1]/qr[2]。 */
  // uint8_t qr[3] = {0};
  // if (qr_scan(qr)) {
  //   /* 示例: 三个字节都参与路由判断 */
  //   if (qr[0] == '1' && qr[1] == '2' && qr[2] == '3') {
  //     run("p2_men4");
  //   } else if (qr[0] == '1') {
  //     run("p2_men4");
  //     run("men4_men3");
  //   } else {
  //     run("p2_men4");
  //     run("men4_men4b");
  //   }
  // }
  //
  // /* ── 上平台3, 数字识别 ── */
  // platform_enter();
  // int d1 = digit_read();
  // voice(d1 + 10);
  // platform_exit();
  // run("men3_men2");
  //
  // /* ── 上平台4, 数字识别 ── */
  // platform_enter();
  // int d2 = digit_read();
  // voice(d2 + 10);
  // platform_exit();
  //
  // /* ── 按和路由 ── */
  // run(platform_sum_route(d1 + d2));
  // cross(STOP);
}

/*
 * ══════════════════════════════════════════════════════════════
 * 示例2: 路口循环 3 圈
 * ══════════════════════════════════════════════════════════════
 *
 * 触发: debug console 输入 msn loop
 */

void mission_loop(void) {
  speed(0.8f);
  for (int i = 0; i < 3; i++) {
    cross(FORWARD);
    cross(LEFT_90);
  }
  cross(STOP);
}

/*
 * ══════════════════════════════════════════════════════════════
 * 示例3: 简单路线（用 cross 替代旧路径表）
 * ══════════════════════════════════════════════════════════════
 *
 * 触发: debug console 输入 msn simple
 */

void mission_simple(void) {
  cross(FORWARD);
  cross(LEFT_90);
  cross(FORWARD);
  cross(RIGHT_90);
  cross(FORWARD);
  cross(STOP);
}

/*
 * ══════════════════════════════════════════════════════════════
 * go12 示例函数已移除 — 移植版见 application/run_task.c
 * (run_task 独占灰度+底盘, 由 freertos.c 创建)
 * ══════════════════════════════════════════════════════════════
 */

/*
 * ╔══════════════════════════════════════════════════════════════╗
 * ║  ★★  上电启动路线 — 改这个函数决定开机后行为  ★★          ║
 * ╚══════════════════════════════════════════════════════════════╝
 *
 * 上电 1s 后自动调用此函数. 用 actions API 随便写.
 *
 * 可用 API:
 *   有线: cross(FORWARD)  cross(LEFT_90)  cross(RIGHT_90) ...
 *   无线: move(mm, spd, ms)  turn(deg)  find_line(ms)  wait_ir(ms)
 *   瞬时: speed(v)  voice(track)  servo(ch, us)
 *   视觉: qr_scan()  digit_read()
 *   路线: run("basic")  run("1_2") ...
 *   平台: platform_enter()  platform_exit()
 */

void startup(void) {
  osDelay(1000); /* 等传感器稳定 */
  g_mission_active = 1;
  //
  // /* ══════════════ 改下面这行 ══════════════ */
  // /* 等红外触发 (遮挡→离开→消抖1s), 不触发不启动 */
  if (wait_ir_trigger(60000, 1000) == 0) {
    mission_comp(); /* ← 当前: 竞赛完整流程  */
  }
  // /* ═════════════════════════════════════════ */

  g_mission_active = 0;
  vTaskDelete(NULL); /* 执行完删除自己 */
}

/* ---- 兼容旧版: debug console path 命令用的出厂路径表 ---- */
static const navigate_step_t demo_path[] = {{STOP}};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_RNG_Init();
  MX_I2C3_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM10_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* BSP init */
  buzzer_init();
  can_filter_init();

  /* Gray sensor — default ID 0x01 */
  gray_sensor_init(0x01);

  /* Load navigation path. */
  navigate_load_path(demo_path, sizeof(demo_path) / sizeof(demo_path[0]));

  /* ★ 上电启动任务 — startup() 在这里创建, 调度器启动后自动执行。
   * 已禁用: run_task (application/run_task.c) 独占灰度+底盘, 由 freertos.c 创建。 */
#if 0
  xTaskCreate((TaskFunction_t)startup, "startup", 1024, NULL, osPriorityNormal,
              NULL);
#endif

  MX_USB_DEVICE_Init();
  HAL_Delay(500);

  /* Init calibration subsystem (load flash data, set up hooks). */
  cali_param_init();

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* Override __weak test_task — USB already init in main(), re-init resets the
 * peripheral. */
void test_task(void const *argument) {
  (void)argument;
  vTaskDelete(NULL);
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
