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

/* 用于存储完整命令的缓冲区 */
uint8_t commandBuffer[64];
uint8_t commandLength = 0;
volatile uint8_t commandReady = 0;

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief 串口通信任务初始化函数
 * @param argument: 未使用
 * @retval None
 */
void CommunicationTask_Init(void *argument)
{
  osDelay(100);

  /* 初始化接收 */
  rxIndex = 0;
  memset(rxBuffer, 0, sizeof(rxBuffer));
  
  /* 启动UART中断接收 */
  HAL_StatusTypeDef status = HAL_UART_Receive_IT(&huart1, &rxBuffer[rxIndex], 1);
  
  /* 提示信息 */
  char initMsg[64];
  int len = sprintf(initMsg, "Communication task started, UART status: %d\r\n", status);
  HAL_UART_Transmit(&huart1, (uint8_t *)initMsg, len, 100);
}

/**
 * @brief 串口通信任务主循环函数
 * @retval None
 */
void CommunicationTask_Loop(void)
{
  /* 用于UART发送的长度变量 */
  int len = 0;

  /* 优先处理通信事件，提高响应速度 */
  uint32_t eventFlag = osEventFlagsWait(systemEventGroupHandle,
                                        EVENT_COMMUNICATION,
                                        osFlagsWaitAny,
                                        10); // 减少等待时间

  /* 处理接收到的命令 */
  if ((eventFlag & EVENT_COMMUNICATION) == EVENT_COMMUNICATION && commandReady)
  {
    /* 使用命令缓冲区而不是接收缓冲区 */
    commandBuffer[commandLength] = '\0';

    /* 只有接收到完整命令(#结尾)才处理 */
    if (commandLength >= 1 && commandBuffer[commandLength-1] == '#')
    {
      /* 清除结束符，便于后续处理 */
      commandBuffer[commandLength-1] = '\0';

      /* 输出命令处理开始的调试信息 */
      len = sprintf(uartTxBuffer, "PROCESS_CMD: Len=%d, Cmd=[%.*s]\r\n",
                   commandLength, commandLength-1, commandBuffer);
      HAL_UART_Transmit(&huart1, (uint8_t *)uartTxBuffer, len, 20);

      /* 详细检查前5个字符 */
      len = sprintf(uartTxBuffer, "CHECK_CHARS: [%c][%c][%c][%c][%c] (ASCII: %d,%d,%d,%d,%d)\r\n",
                   commandBuffer[0], commandBuffer[1], commandBuffer[2], commandBuffer[3], commandBuffer[4],
                   (int)commandBuffer[0], (int)commandBuffer[1], (int)commandBuffer[2],
                   (int)commandBuffer[3], (int)commandBuffer[4]);
      HAL_UART_Transmit(&huart1, (uint8_t *)uartTxBuffer, len, 20);

      /* 检查命令长度是否足够 */
      if (commandLength < 6)
      {
        len = sprintf(uartTxBuffer, "ERR,COMMAND_TOO_SHORT,LEN=%d\r\n", commandLength);
        HAL_UART_Transmit(&huart1, (uint8_t *)uartTxBuffer, len, 20);
      }
      /* 处理电机速度命令 */
      else if (commandBuffer[0] == '$' && commandBuffer[1] == 'S' && commandBuffer[2] == 'P' &&
               commandBuffer[3] == 'D' && commandBuffer[4] == ',')
      {
        /* 输出接收到的命令用于调试 */
        len = sprintf(uartTxBuffer, "SPD_COMMAND_MATCHED! Processing...\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t *)uartTxBuffer, len, 20);

        /* 创建一个工作字符串副本用于解析，避免破坏原始数据 */
        char workBuffer[64];
        strcpy(workBuffer, (char *)&commandBuffer[5]);

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
      else if (commandBuffer[0] == '$' && commandBuffer[1] == 'P' && commandBuffer[2] == 'I' &&
               commandBuffer[3] == 'D' && commandBuffer[4] == ',')
      {
        /* 输出接收到的命令用于调试 */
        len = sprintf(uartTxBuffer, "Received PID[%d]: %s\r\n", commandLength, commandBuffer);
        HAL_UART_Transmit(&huart1, (uint8_t *)uartTxBuffer, len, 100);

        /* 解析PID参数 */
        char *motorIdStr = strtok((char *)&commandBuffer[5], ",");
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
      else
      {
        /* 未知命令格式 */
        len = sprintf(uartTxBuffer, "ERR,UNKNOWN_COMMAND\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t *)uartTxBuffer, len, 20);
      }
    }

    /* 命令处理完成，清除命令状态 */
    commandReady = 0;
    commandLength = 0;
    memset(commandBuffer, 0, sizeof(commandBuffer));

    /* 输出命令处理完成的确认信息 */
    len = sprintf(uartTxBuffer, "CMD_CLEANUP: Buffer cleared, ready for next command\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t *)uartTxBuffer, len, 20);
  }
  else
  {
    /* 没有通信事件时，确保UART接收始终处于激活状态 */
    if (huart1.RxState != HAL_UART_STATE_BUSY_RX)
    {
      /* UART接收未激活，重新启动接收 */
      HAL_StatusTypeDef status = HAL_UART_Receive_IT(&huart1, &rxBuffer[rxIndex], 1);

      /* 输出UART状态用于调试 */
      if (status != HAL_OK)
      {
        len = sprintf(uartTxBuffer, "UART Restart Error: %d, State: %d\r\n",
                     (int)status, (int)huart1.RxState);
        HAL_UART_Transmit(&huart1, (uint8_t *)uartTxBuffer, len, 20);
      }
    }
    /* 短暂延时避免CPU占用过高，但要足够短以保证响应性 */
    osDelay(1);
  }
}

/**
 * @brief 串口通信任务实现函数（由freertos.c中的StartCommunicationTask调用）
 * @param argument: 未使用
 * @retval None
 */
void CommunicationTask_Implementation(void *argument)
{
  /* 此函数现在由 freertos.c 中的 StartCommunicationTask 调用 */
  /* 不再包含循环逻辑，循环在 freertos.c 中处理 */
}

/**
 * @brief UART接收完成回调
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    /* 忽略回车符和换行符 */
    if (rxBuffer[rxIndex] == '\r' || rxBuffer[rxIndex] == '\n')
    {
      /* 忽略这个字符，直接继续接收下一个字符 */
      HAL_UART_Receive_IT(&huart1, &rxBuffer[rxIndex], 1);
      return;
    }

    /* 更新索引 */
    rxIndex++;

    /* 检查是否收到结束符'#'或缓冲区将满 */
    if (rxBuffer[rxIndex-1] == '#' || rxIndex >= sizeof(rxBuffer) - 2)
    {
      /* 接收到完整命令 - 立即处理并清空缓冲区 */

      /* 复制完整命令到命令缓冲区 */
      if (rxIndex <= sizeof(commandBuffer))
      {
        memcpy(commandBuffer, rxBuffer, rxIndex);
        commandLength = rxIndex;
        commandReady = 1;

        /* 设置事件标志，通知任务处理命令 */
        osEventFlagsSet(systemEventGroupHandle, EVENT_COMMUNICATION);

        /* 调试信息：显示接收到的完整命令 */
        int debugLen = sprintf(uartTxBuffer, "RECV_CMD: [%.*s]\r\n", rxIndex-1, rxBuffer);
        HAL_UART_Transmit(&huart1, (uint8_t *)uartTxBuffer, debugLen, 20);
      }

      /* 立即清空接收缓冲区，确保下一次接收干净 */
      memset(rxBuffer, 0, sizeof(rxBuffer));
      rxIndex = 0;

      /* 重新启动UART接收，为下一次命令做准备 */
      HAL_StatusTypeDef status = HAL_UART_Receive_IT(&huart1, &rxBuffer[rxIndex], 1);

      /* 如果UART重启失败，输出错误信息 */
      if (status != HAL_OK)
      {
        int errorLen = sprintf(uartTxBuffer, "UART_RESTART_FAIL: %d\r\n", (int)status);
        HAL_UART_Transmit(&huart1, (uint8_t *)uartTxBuffer, errorLen, 20);
      }
    }
    else
    {
      /* 继续接收下一个字符 */
      HAL_UART_Receive_IT(&huart1, &rxBuffer[rxIndex], 1);
    }
  }
}
