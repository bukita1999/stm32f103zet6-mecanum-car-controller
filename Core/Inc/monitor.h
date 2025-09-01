/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : monitor.h
  * @brief          : Header for monitor module
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
#ifndef __MONITOR_H
#define __MONITOR_H

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

/* Exported functions prototypes ---------------------------------------------*/
void MonitorTask_Init(void *argument);
void MonitorTask_Loop(void);
void MonitorTask_Implementation(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* __MONITOR_H */
