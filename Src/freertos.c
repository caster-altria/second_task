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

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId LEDTaskHandle;
osThreadId BUZZERTaskHandle;
osThreadId LASERTaskHandle;
osThreadId UARTTaskHandle;
osThreadId IMUTaskHandle;
osThreadId PLOTTERTaskHandle;
osThreadId CANTaskHandle;
osThreadId CHASSISTaskHandle;
osThreadId GIMBALTaskHandle;
osThreadId GIMBALFOLDTaskHandle;
osThreadId TRAJECTORYTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
extern void led_task(void const * argument);
extern void buzzer_task(void const * argument);
extern void laser_task(void const * argument);
extern void uart_task(void const * argument);
extern void imu_task(void const * argument);
extern void plotter_task(void const * argument);
extern void can_task(void const * argument);
extern void chassis_task(void const * argument);
extern void gimbal_task(void const * argument);
extern void gimbal_fold_task(void const * argument);
extern void trajectory_task(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

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
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of LEDTask */
  osThreadDef(LEDTask, led_task, osPriorityIdle, 0, 128);
  LEDTaskHandle = osThreadCreate(osThread(LEDTask), NULL);

  /* definition and creation of BUZZERTask */
  osThreadDef(BUZZERTask, buzzer_task, osPriorityIdle, 0, 128);
  BUZZERTaskHandle = osThreadCreate(osThread(BUZZERTask), NULL);

  /* definition and creation of LASERTask */
  osThreadDef(LASERTask, laser_task, osPriorityBelowNormal, 0, 128);
  LASERTaskHandle = osThreadCreate(osThread(LASERTask), NULL);

  /* definition and creation of UARTTask */
  osThreadDef(UARTTask, uart_task, osPriorityNormal, 0, 256);
  UARTTaskHandle = osThreadCreate(osThread(UARTTask), NULL);

  /* definition and creation of IMUTask */
  osThreadDef(IMUTask, imu_task, osPriorityBelowNormal, 0, 256);
  IMUTaskHandle = osThreadCreate(osThread(IMUTask), NULL);

  /* definition and creation of PLOTTERTask */
  osThreadDef(PLOTTERTask, plotter_task, osPriorityLow, 0, 128);
  PLOTTERTaskHandle = osThreadCreate(osThread(PLOTTERTask), NULL);

  /* definition and creation of CANTask */
  osThreadDef(CANTask, can_task, osPriorityAboveNormal, 0, 256);
  CANTaskHandle = osThreadCreate(osThread(CANTask), NULL);

  /* definition and creation of CHASSISTask */
  osThreadDef(CHASSISTask, chassis_task, osPriorityNormal, 0, 256);
  CHASSISTaskHandle = osThreadCreate(osThread(CHASSISTask), NULL);

  /* definition and creation of GIMBALTask */
  osThreadDef(GIMBALTask, gimbal_task, osPriorityNormal, 0, 256);
  GIMBALTaskHandle = osThreadCreate(osThread(GIMBALTask), NULL);

  /* definition and creation of GIMBALFOLDTask */
  osThreadDef(GIMBALFOLDTask, gimbal_fold_task, osPriorityNormal, 0, 256);
  GIMBALFOLDTaskHandle = osThreadCreate(osThread(GIMBALFOLDTask), NULL);

  /* definition and creation of TRAJECTORYTask */
  osThreadDef(TRAJECTORYTask, trajectory_task, osPriorityNormal, 0, 256);
  TRAJECTORYTaskHandle = osThreadCreate(osThread(TRAJECTORYTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
