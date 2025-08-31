/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : communication.c
  * @brief          : Communication module implementation
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
#include "communication.h"
#include "usart.h"
#include "pid_controller.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* 用于UART发送的缓冲区 */
char uartTxBuffer[128];

/* 用于UART接收的缓冲区 */
uint8_t rxBuffer[64];
uint8_t rxIndex = 0;

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief 串口通信任务函数
 * @param argument: 未使用
 * @retval None
 */
void StartCommunicationTask(void *argument)
{
  osDelay(100);

  /* 初始化接收 */
  rxIndex = 0;
  memset(rxBuffer, 0, sizeof(rxBuffer));
  HAL_UART_Receive_IT(&huart1, &rxBuffer[rxIndex], 1);

  /* 提示信息 */
  HAL_UART_Transmit(&huart1, (uint8_t *)"Communication task started\r\n", 28, 100);

  /* 任务主循环 */
  for (;;)
  {
    /* 等待电机数据更新事件或通信事件 */
    uint32_t eventFlag = osEventFlagsWait(systemEventGroupHandle,
                                          EVENT_MOTOR_UPDATE | EVENT_COMMUNICATION,
                                          osFlagsWaitAny,
                                          100);

    /* 如果电机更新事件触发 */
    if ((eventFlag & EVENT_MOTOR_UPDATE) == EVENT_MOTOR_UPDATE)
    {
      /* 请求电机数据互斥量 */
      if (osMutexAcquire(motorDataMutexHandle, 10) == osOK)
      {
        /* 格式化电机速度数据 - 这里注释掉，避免过多输出 */
        // int len = sprintf(uartTxBuffer, "SPD,%d,%d,%d,%d\r\n",
        //                   systemState.motors[0].currentSpeed,
        //                   systemState.motors[1].currentSpeed,
        //                   systemState.motors[2].currentSpeed,
        //                   systemState.motors[3].currentSpeed);

        /* 释放电机数据互斥量 */
        osMutexRelease(motorDataMutexHandle);

        /* 通过UART发送数据 - 注释掉 */
        // HAL_UART_Transmit(&huart1, (uint8_t *)uartTxBuffer, len, 100);
      }
    }

    /* 处理接收到的命令 */
    if ((eventFlag & EVENT_COMMUNICATION) == EVENT_COMMUNICATION)
    {
      /* 保证字符串结束 */
      rxBuffer[rxIndex] = '\0';
      
      /* 只有接收到完整命令(#结尾)才处理 */
      if (rxIndex >= 1 && rxBuffer[rxIndex-1] == '#')
      {
        /* 清除结束符，便于后续处理 */
        rxBuffer[rxIndex-1] = '\0';
        
        /* 处理电机速度命令 */
        if (rxBuffer[0] == '$' && rxBuffer[1] == 'S' && rxBuffer[2] == 'P' && 
            rxBuffer[3] == 'D' && rxBuffer[4] == ',')
        {
          /* 输出接收到的命令用于调试 */
          int len = sprintf(uartTxBuffer, "Received: %s#\r\n", rxBuffer);
          HAL_UART_Transmit(&huart1, (uint8_t *)uartTxBuffer, len, 100);
          
          /* 解析命令参数 */
          char *token = strtok((char *)&rxBuffer[5], ",");
          int motorId = 0;
          
          while (token != NULL && motorId < 4)
          {
            int16_t speed = atoi(token);
            
            /* 设置电机速度 */
            if (osMutexAcquire(motorDataMutexHandle, 10) == osOK)
            {
              SetMotorSpeed(&systemState.motors[motorId], speed);
              osMutexRelease(motorDataMutexHandle);
              
              /* 确认消息 */
              len = sprintf(uartTxBuffer, "ACK,M%d,%d\r\n", motorId, speed);
              HAL_UART_Transmit(&huart1, (uint8_t *)uartTxBuffer, len, 100);
            }
            
            token = strtok(NULL, ",");
            motorId++;
          }
        }
        /* 处理PID参数设置命令 */
        else if (rxBuffer[0] == '$' && rxBuffer[1] == 'P' && rxBuffer[2] == 'I' && 
                 rxBuffer[3] == 'D' && rxBuffer[4] == ',')
        {
          /* 输出接收到的命令用于调试 */
          int len = sprintf(uartTxBuffer, "Received PID: %s#\r\n", rxBuffer);
          HAL_UART_Transmit(&huart1, (uint8_t *)uartTxBuffer, len, 100);
          
          /* 解析PID参数 */
          char *motorIdStr = strtok((char *)&rxBuffer[5], ",");
          char *kpStr = strtok(NULL, ",");
          char *kiStr = strtok(NULL, ",");
          char *kdStr = strtok(NULL, ",");
          
          if (motorIdStr && kpStr && kiStr && kdStr)
          {
            int motorId = atoi(motorIdStr);
            float kp = atof(kpStr);
            float ki = atof(kiStr);
            float kd = atof(kdStr);
            
            /* 检查电机ID有效性 */
            if (motorId >= 0 && motorId < 4)
            {
              if (osMutexAcquire(motorDataMutexHandle, 10) == osOK)
              {
                /* 更新PID参数 */
                systemState.motors[motorId].pidController.Kp = kp;
                systemState.motors[motorId].pidController.Ki = ki;
                systemState.motors[motorId].pidController.Kd = kd;
                
                /* 重置PID状态以避免突变 */
                systemState.motors[motorId].pidController.errorSum = 0;
                systemState.motors[motorId].pidController.lastError = 0;
                
                osMutexRelease(motorDataMutexHandle);
                
                /* 确认消息 */
                len = sprintf(uartTxBuffer, "ACK,PID,M%d,%.3f,%.3f,%.3f\r\n", 
                             motorId, kp, ki, kd);
                HAL_UART_Transmit(&huart1, (uint8_t *)uartTxBuffer, len, 100);
              }
            }
            else
            {
              /* 错误：无效的电机ID */
              len = sprintf(uartTxBuffer, "ERR,INVALID_MOTOR_ID,%d\r\n", motorId);
              HAL_UART_Transmit(&huart1, (uint8_t *)uartTxBuffer, len, 100);
            }
          }
          else
          {
            /* 错误：参数不完整 */
            len = sprintf(uartTxBuffer, "ERR,INCOMPLETE_PID_PARAMS\r\n");
            HAL_UART_Transmit(&huart1, (uint8_t *)uartTxBuffer, len, 100);
          }
        }
      }
      
      /* 命令处理完成，清除缓冲区并重启接收 */
      memset(rxBuffer, 0, sizeof(rxBuffer));
      rxIndex = 0;
      HAL_UART_Receive_IT(&huart1, &rxBuffer[rxIndex], 1);
    }
    
    osDelay(10);
  }
}

/**
 * @brief UART接收完成回调
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    /* 更新索引 */
    rxIndex++;
    
    /* 检查是否收到结束符'#'或缓冲区将满 */
    if (rxBuffer[rxIndex-1] == '#' || rxIndex >= sizeof(rxBuffer) - 2)
    {
      /* 设置事件标志，通知任务处理命令 */
      osEventFlagsSet(systemEventGroupHandle, EVENT_COMMUNICATION);
    }
    else
    {
      /* 继续接收下一个字符 */
      HAL_UART_Receive_IT(&huart1, &rxBuffer[rxIndex], 1);
    }
  }
}
