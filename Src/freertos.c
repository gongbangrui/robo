/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "actions.h"
#include "calibrate_task.h"
#include "chassis_task.h"
#include "detect_task.h"
#include "INS_task.h"
#include "led_flow_task.h"
#include "voltage_task.h"
#include "line_track_task.h"
#include "navigate_task.h"
#include "usb_cdc_task.h"
#include "debug_console.h"
#include "run_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId calibrate_task_handle;
osThreadId chassis_task_handle;
osThreadId detect_task_handle;
osThreadId ins_task_handle;
osThreadId led_flow_task_handle;
osThreadId voltage_task_handle;
osThreadId line_track_task_handle;
osThreadId navigate_task_handle;
osThreadId run_task_handle;
osThreadId usb_cdc_task_handle;
osThreadId debug_console_task_handle;
/* USER CODE END Variables */
osThreadId testHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void test_task(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  navigate_init_semaphore();
  actions_init();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of test */
  osThreadDef(test, test_task, osPriorityNormal, 0, 128);
  testHandle = osThreadCreate(osThread(test), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USB CDC: printf/scanf via ring buffers + semaphores */
  osThreadDef(usb_cdc, usb_cdc_task, osPriorityNormal, 0, 256);
  usb_cdc_task_handle = osThreadCreate(osThread(usb_cdc), NULL);

  /* calibrate: IMU calibration, runs once then deletes itself */
  osThreadDef(calibrate, calibrate_task, osPriorityNormal, 0, 512);
  calibrate_task_handle = osThreadCreate(osThread(calibrate), NULL);

  /* INS: AHRS attitude estimation, 1ms period */
  osThreadDef(ins, INS_task, osPriorityRealtime, 0, 1024);
  ins_task_handle = osThreadCreate(osThread(ins), NULL);

  /* chassis: mecanum motion control, 2ms period */
  osThreadDef(chassis, chassis_task, osPriorityAboveNormal, 0, 512);
  chassis_task_handle = osThreadCreate(osThread(chassis), NULL);

  /* detect: device watchdog/heartbeat */
  osThreadDef(detect, detect_task, osPriorityNormal, 0, 256);
  detect_task_handle = osThreadCreate(osThread(detect), NULL);

  /* LED flow: RGB status indicator */
  osThreadDef(led_flow, led_RGB_flow_task, osPriorityNormal, 0, 256);
  led_flow_task_handle = osThreadCreate(osThread(led_flow), NULL);

  /* voltage: battery ADC */
  osThreadDef(voltage, battery_voltage_task, osPriorityNormal, 0, 128);
  voltage_task_handle = osThreadCreate(osThread(voltage), NULL);

  /* run_task: 技能赛2025 主控任务 (独占灰度+底盘)。
   * line_track/navigate 已禁用 — 灰度单主查询, 与 run_task 冲突。 */
  osThreadDef(run, run_task, osPriorityNormal, 0, 1024);
  run_task_handle = osThreadCreate(osThread(run), NULL);

  /* line_track: line-follow FSM + PID, 5ms period.
   * Priority = Normal (not High) to avoid starving chassis_task (AboveNormal, 2ms).
   * The sensor poll loop yields via osDelay(1), so computation bursts are short. */
#if 0
  osThreadDef(line_track, line_track_task, osPriorityNormal, 0, 512);
  line_track_task_handle = osThreadCreate(osThread(line_track), NULL);

  /* navigate: path step planner, 20ms polling */
  osThreadDef(navigate, navigate_task, osPriorityNormal, 0, 256);
  navigate_task_handle = osThreadCreate(osThread(navigate), NULL);
#endif

  /* debug_console: command interface for testing */
  osThreadDef(debug_console, debug_console_task, osPriorityNormal, 0, 512);
  debug_console_task_handle = osThreadCreate(osThread(debug_console), NULL);

  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_test_task */
/**
  * @brief  Function implementing the test thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_test_task */
__weak void test_task(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN test_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END test_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
