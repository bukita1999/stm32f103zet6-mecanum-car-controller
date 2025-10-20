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
#include "motor_control.h"
#include "communication.h"
#include "monitor.h"
#include "usb_handler.h"
#include "usb_device.h"
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
/* 声明系统状态全局变量 */
SystemState_t systemState;

/* 任务上下文变量 */
TickType_t motorControlLastWake;
uint32_t motorControlPeriod_ms;


void MotorControlTask_Init(void *argument);
void MotorControlTask_Loop(void);
void CommunicationTask_Init(void *argument);
void CommunicationTask_Loop(void);
void MonitorTask_Init(void *argument);
void MonitorTask_Loop(void);
void UsbTask_Init(void *argument);
void UsbTask_Loop(void);
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
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CommunicationTa */
osThreadId_t CommunicationTaHandle;
const osThreadAttr_t CommunicationTa_attributes = {
  .name = "CommunicationTa",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MonitorTask */

osThreadId_t MonitorTaskHandle;
const osThreadAttr_t MonitorTask_attributes = {
  .name = "MonitorTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UsbTxRxTask */
osThreadId_t UsbTxRxTaskHandle;
const osThreadAttr_t UsbTxRxTask_attributes = {
  .name = "UsbTxRxTask",
  .stack_size = 384 * 4,
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
void StartCommunicationTask(void *argument);
void StartMonitorTask(void *argument);
void StartUsbTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

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

  /* creation of CommunicationTa */
  CommunicationTaHandle = osThreadNew(StartCommunicationTask, NULL, &CommunicationTa_attributes);

  /* creation of MonitorTask */
  MonitorTaskHandle = osThreadNew(StartMonitorTask, NULL, &MonitorTask_attributes);

  /* creation of UsbTxRxTask */
  UsbTxRxTaskHandle = osThreadNew(StartUsbTask, NULL, &UsbTxRxTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  USBHandler_Init();
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
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* USB设备已在上面初始化，这里不需要重复初始化 */
  /* Infinite loop */
  for (;;)
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
  /* 初始化 */
  MotorControlTask_Init(argument);

  /* 主循环 */
  for (;;)
  {
    /* 执行主循环逻辑 */
    MotorControlTask_Loop();

    /* 等周期延时（消除周期漂移） */
    vTaskDelayUntil(&motorControlLastWake, pdMS_TO_TICKS(motorControlPeriod_ms));
  }
  /* USER CODE END StartMotorControlTask */
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
  /* 初始化 */
  CommunicationTask_Init(argument);

  /* 主循环 */
  for (;;)
  {
    /* 执行主循环逻辑 */
    CommunicationTask_Loop();
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
  /* 初始化 */
  MonitorTask_Init(argument);

  /* 主循环 */
  for (;;)
  {
    /* 执行主循环逻辑 */
    MonitorTask_Loop();
  }
  /* USER CODE END StartMonitorTask */
}

/* USER CODE BEGIN Header_StartUsbTask */
/**
  * @brief Function implementing the UsbTxRxTask thread.
  * @param argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartUsbTask */
void StartUsbTask(void *argument)
{
  /* USER CODE BEGIN StartUsbTask */
  /* 初始化 */
  UsbTask_Init(argument);

  /* 主循环 */
  for (;;)
  {
    /* 执行主循环逻辑 */
    UsbTask_Loop();
  }
  /* USER CODE END StartUsbTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

