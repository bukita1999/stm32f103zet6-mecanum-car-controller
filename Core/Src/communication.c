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
    /* 优先处理通信事件，提高响应速度 */
    uint32_t eventFlag = osEventFlagsWait(systemEventGroupHandle,
                                          EVENT_COMMUNICATION,
                                          osFlagsWaitAny,
                                          10); // 减少等待时间

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
          HAL_UART_Transmit(&huart1, (uint8_t *)uartTxBuffer, len, 20);
          
          /* 创建一个工作字符串副本用于解析，避免破坏原始数据 */
          char workBuffer[64];
          strcpy(workBuffer, (char *)&rxBuffer[5]);
          
          /* 先解析所有参数，存储到临时数组 */
          int16_t speeds[4] = {0};
          int validMotors = 0;
          char *token = strtok(workBuffer, ",");
          
          /* 解析所有速度参数 */
          while (token != NULL && validMotors < 4)
          {
            speeds[validMotors] = atoi(token);
            
            /* 输出解析过程调试信息 - 注释掉以提高响应速度 */
            // len = sprintf(uartTxBuffer, "Parsing M%d: %s -> %d\r\n", validMotors, token, speeds[validMotors]);
            // HAL_UART_Transmit(&huart1, (uint8_t *)uartTxBuffer, len, 20);
            
            token = strtok(NULL, ",");
            validMotors++;
          }
          
          /* 输出当前尝试设置的速度，用于调试 */
          len = sprintf(uartTxBuffer, "Setting: M0=%d,M1=%d,M2=%d,M3=%d\r\n", 
                       (validMotors>0)?speeds[0]:0, (validMotors>1)?speeds[1]:0, 
                       (validMotors>2)?speeds[2]:0, (validMotors>3)?speeds[3]:0);
          HAL_UART_Transmit(&huart1, (uint8_t *)uartTxBuffer, len, 20);
          
          /* 尝试获取互斥量，增加超时时间 */
          osStatus_t mutexStatus = osMutexAcquire(motorDataMutexHandle, 200);
          if (mutexStatus == osOK)
          {
            /* 先设置所有电机，减少互斥量占用时间 */
            for (int motorId = 0; motorId < validMotors; motorId++)
            {
              SetMotorSpeed(&systemState.motors[motorId], speeds[motorId]);
            }
            osMutexRelease(motorDataMutexHandle);
            
            /* 释放互斥量后再发送确认消息，避免长时间占用 */
            for (int motorId = 0; motorId < validMotors; motorId++)
            {
              len = sprintf(uartTxBuffer, "ACK,M%d,%d\r\n", motorId, speeds[motorId]);
              HAL_UART_Transmit(&huart1, (uint8_t *)uartTxBuffer, len, 20);
            }
          }
          else
          {
            /* 互斥量获取失败，输出详细错误信息 */
            len = sprintf(uartTxBuffer, "ERR,MUTEX_FAIL_ALL,STATUS=%d\r\n", (int)mutexStatus);
            HAL_UART_Transmit(&huart1, (uint8_t *)uartTxBuffer, len, 20);
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
                
                /* 确认消息 - 将浮点数转换为整数输出 */
                int16_t kpInt = (int16_t)(kp * 1000);
                int16_t kiInt = (int16_t)(ki * 1000);
                int16_t kdInt = (int16_t)(kd * 1000);
                len = sprintf(uartTxBuffer, "ACK,PID,M%d,%d.%03d,%d.%03d,%d.%03d\r\n", 
                             motorId, 
                             kpInt / 1000, abs(kpInt % 1000),
                             kiInt / 1000, abs(kiInt % 1000),
                             kdInt / 1000, abs(kdInt % 1000));
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
    else
    {
      /* 没有通信事件时，短暂延时避免CPU占用过高 */
      osDelay(5);
    }
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
