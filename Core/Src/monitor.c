/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : monitor.c
  * @brief          : Monitor module implementation
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
#include "monitor.h"
#include "communication.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief 监控任务函数
 * @param argument: 未使用
 * @retval None
 */
void StartMonitorTask(void *argument)
{
  /* 等待系统初始化完成 */
  osDelay(500);
  
  uint32_t lastReportTime = 0;
  
  /* 无限循环 */
  for (;;)
  {
    uint32_t currentTime = osKernelGetTickCount();
    
    /* 每5秒报告一次电机状态 */
    if ((currentTime - lastReportTime) > 5000)
    {
      lastReportTime = currentTime;
      
      /* 获取电机数据 */
      if (osMutexAcquire(motorDataMutexHandle, 100) == osOK)
      {
        /* 报告所有电机状态 */
        for (uint8_t i = 0; i < 4; i++)
        {
          snprintf(uartTxBuffer, sizeof(uartTxBuffer),
                  "Motor%d: Target:%d Current:%d RPM, PWM:%d%%, Error:%.2f\r\n",
                  i + 1,
                  systemState.motors[i].targetSpeed,
                  systemState.motors[i].currentSpeed,
                  systemState.motors[i].pwmPercent,
                  systemState.motors[i].pidController.error);
          
          /* 释放互斥量发送期间，避免长时间占用 */
          osMutexRelease(motorDataMutexHandle);
          
          /* 发送数据 */
          HAL_UART_Transmit(&huart1, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), 100);
          
          /* 短暂延时，避免UART缓冲区溢出 */
          osDelay(20);
          
          /* 重新获取互斥量继续处理 */
          if (i < 3) { // 只在不是最后一个电机时重新获取
            osMutexAcquire(motorDataMutexHandle, 100);
          }
        }
        
        /* 如果最后一次没有释放，这里释放 */
        // osMutexRelease(motorDataMutexHandle); // 已在循环中释放
      }
      
      /* 可以添加其他系统参数监控，如温度、电压等 */
    }
    
    /* 每100ms发送一次Telemetry */
    static uint32_t lastTelemetryTime = 0;
    uint32_t now = osKernelGetTickCount();
    if (now - lastTelemetryTime >= 100) {
      lastTelemetryTime = now;
      USB_SendTelemetry();
    }
    
    /* 监控任务周期 */
    osDelay(100);
  }
}
