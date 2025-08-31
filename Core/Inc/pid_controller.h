/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : pid_controller.h
  * @brief          : Header for PID controller module
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
#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "robot_types.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/
void PIDInit(PIDController_t *pid, float kp, float ki, float kd, float min, float max);
float PIDCompute(PIDController_t *pid, float dt);

#ifdef __cplusplus
}
#endif

#endif /* __PID_CONTROLLER_H */
