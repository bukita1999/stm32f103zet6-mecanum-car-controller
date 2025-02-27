/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MotorControlTas */
osThreadId_t MotorControlTasHandle;
const osThreadAttr_t MotorControlTas_attributes = {
  .name = "MotorControlTas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ServoControlTas */
osThreadId_t ServoControlTasHandle;
const osThreadAttr_t ServoControlTas_attributes = {
  .name = "ServoControlTas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CommunicationTa */
osThreadId_t CommunicationTaHandle;
const osThreadAttr_t CommunicationTa_attributes = {
  .name = "CommunicationTa",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MonitorTask */
osThreadId_t MonitorTaskHandle;
const osThreadAttr_t MonitorTask_attributes = {
  .name = "MonitorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for commandQueue */
osMessageQueueId_t commandQueueHandle;
const osMessageQueueAttr_t commandQueue_attributes = {
  .name = "commandQueue"
};
/* Definitions for motorDataMutex */
osMutexId_t motorDataMutexHandle;
const osMutexAttr_t motorDataMutex_attributes = {
  .name = "motorDataMutex"
};
/* Definitions for servoDataMutex */
osMutexId_t servoDataMutexHandle;
const osMutexAttr_t servoDataMutex_attributes = {
  .name = "servoDataMutex"
};
/* Definitions for i2cMutex */
osMutexId_t i2cMutexHandle;
const osMutexAttr_t i2cMutex_attributes = {
  .name = "i2cMutex"
};
/* Definitions for statusSemaphore */
osSemaphoreId_t statusSemaphoreHandle;
const osSemaphoreAttr_t statusSemaphore_attributes = {
  .name = "statusSemaphore"
};
/* Definitions for systemEventGroup */
osEventFlagsId_t systemEventGroupHandle;
const osEventFlagsAttr_t systemEventGroup_attributes = {
  .name = "systemEventGroup"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartMotorControlTask(void *argument);
void StartServoControlTask(void *argument);
void StartCommunicationTask(void *argument);
void StartMonitorTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of motorDataMutex */
  motorDataMutexHandle = osMutexNew(&motorDataMutex_attributes);

  /* creation of servoDataMutex */
  servoDataMutexHandle = osMutexNew(&servoDataMutex_attributes);

  /* creation of i2cMutex */
  i2cMutexHandle = osMutexNew(&i2cMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of statusSemaphore */
  statusSemaphoreHandle = osSemaphoreNew(1, 1, &statusSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of commandQueue */
  commandQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &commandQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of MotorControlTas */
  MotorControlTasHandle = osThreadNew(StartMotorControlTask, NULL, &MotorControlTas_attributes);

  /* creation of ServoControlTas */
  ServoControlTasHandle = osThreadNew(StartServoControlTask, NULL, &ServoControlTas_attributes);

  /* creation of CommunicationTa */
  CommunicationTaHandle = osThreadNew(StartCommunicationTask, NULL, &CommunicationTa_attributes);

  /* creation of MonitorTask */
  MonitorTaskHandle = osThreadNew(StartMonitorTask, NULL, &MonitorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of systemEventGroup */
  systemEventGroupHandle = osEventFlagsNew(&systemEventGroup_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartMotorControlTask */
/**
* @brief Function implementing the MotorControlTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorControlTask */
void StartMotorControlTask(void *argument)
{
  /* USER CODE BEGIN StartMotorControlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartMotorControlTask */
}

/* USER CODE BEGIN Header_StartServoControlTask */
/**
* @brief Function implementing the ServoControlTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartServoControlTask */
void StartServoControlTask(void *argument)
{
  /* USER CODE BEGIN StartServoControlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartServoControlTask */
}

/* USER CODE BEGIN Header_StartCommunicationTask */
/**
* @brief Function implementing the CommunicationTa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommunicationTask */
void StartCommunicationTask(void *argument)
{
  /* USER CODE BEGIN StartCommunicationTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartCommunicationTask */
}

/* USER CODE BEGIN Header_StartMonitorTask */
/**
* @brief Function implementing the MonitorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMonitorTask */
void StartMonitorTask(void *argument)
{
  /* USER CODE BEGIN StartMonitorTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartMonitorTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

