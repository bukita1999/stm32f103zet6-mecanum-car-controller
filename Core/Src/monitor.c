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
#include <stdlib.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief 监控任务实现函数（由freertos.c中的StartMonitorTask调用）
 * @param argument: 未使用
 * @retval None
 */
/* 监控任务的上下文变量 */
static uint32_t monitorLastReportTime = 0;

/**
 * @brief 监控任务初始化函数
 * @param argument: 未使用
 * @retval None
 */
void MonitorTask_Init(void *argument)
{
  /* 等待系统初始化完成 */
  osDelay(500);

  monitorLastReportTime = 0;
}

/**
 * @brief 监控任务主循环函数
 * @retval None
 */
void MonitorTask_Loop(void)
{
  uint32_t currentTime = osKernelGetTickCount();

  /* 每半秒报告一次电机状态 */
  if ((currentTime - monitorLastReportTime) > 500)
  {
    monitorLastReportTime = currentTime;

    /* 获取电机数据 */
    if (osMutexAcquire(motorDataMutexHandle, 100) == osOK)
    {
      /* 报告所有电机状态 */
      for (uint8_t i = 0; i < 4; i++)
      {
        /* 将浮点错误值转换为整数显示（乘以100保留2位小数精度） */
        int16_t errorInt = (int16_t)(systemState.motors[i].pidController.error * 100);
        snprintf(uartTxBuffer, sizeof(uartTxBuffer),
                "Motor%d: Target:%d Current:%d RPM, PWM:%d%%, Error:%d.%02d\r\n",
                i + 1,
                systemState.motors[i].targetSpeed,
                systemState.motors[i].currentSpeed,
                systemState.motors[i].pwmPercent,
                errorInt / 100, abs(errorInt % 100));

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

    /* 报告系统状态 */
    char statusBuffer[64];
    snprintf(statusBuffer, sizeof(statusBuffer),
             "System: Init=%d, PCA9685=%d, MotorErr=%d\r\n",
             systemState.systemFlags.initialized,
             systemState.systemFlags.pca9685Error,
             systemState.systemFlags.motorError);
    HAL_UART_Transmit(&huart1, (uint8_t *)statusBuffer, strlen(statusBuffer), 100);

    /* 可以添加其他系统参数监控，如温度、电压等 */
  }

  /* USB CDC已经通过定时任务发送Hello World消息 */

  /* 监控任务周期 */
  osDelay(100);
}

/**
 * @brief 监控任务实现函数（由freertos.c中的StartMonitorTask调用）
 * @param argument: 未使用
 * @retval None
 */
void MonitorTask_Implementation(void *argument)
{
  /* 此函数现在由 freertos.c 中的 StartMonitorTask 调用 */
  /* 不再包含循环逻辑，循环在 freertos.c 中处理 */
}
