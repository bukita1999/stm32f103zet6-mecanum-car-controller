/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : communication.h
  * @brief          : Header for communication module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "robot_types.h"
#include "motor_control.h"
#include "cmsis_os.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern char uartTxBuffer[128];
extern uint8_t rxBuffer[64];
extern uint8_t rxIndex;

/* Exported functions prototypes ---------------------------------------------*/
void StartCommunicationTask(void *argument);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* __COMMUNICATION_H */
