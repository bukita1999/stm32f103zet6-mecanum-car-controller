/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : motor_control.h
  * @brief          : Header for motor control module
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
#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "robot_types.h"
#include "pid_controller.h"
#include "cmsis_os.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern SystemState_t systemState;
extern osMutexId_t motorDataMutexHandle;
extern osMutexId_t i2cMutexHandle;
extern osEventFlagsId_t systemEventGroupHandle;

/* Exported functions prototypes ---------------------------------------------*/
void MotorSystemInit(void);
void MotorInit(Motor_t *motor, uint8_t id, TIM_HandleTypeDef *encoderTimer, uint8_t pwmChannel,
               GPIO_TypeDef *dir0Port, uint16_t dir0Pin,
               GPIO_TypeDef *dir1Port, uint16_t dir1Pin);
int16_t CalculateMotorSpeed(Motor_t *motor, uint32_t deltaTime);
void SetMotorSpeed(Motor_t *motor, int16_t speed);
void SetMotorPWMPercentage(Motor_t *motor, int16_t pwmPercent);
float GetMotorFilteredSpeed(uint8_t motorIndex);
void MotorControlTask_Init(void *argument);
void MotorControlTask_Loop(void);
void MotorControlTask_Implementation(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H */
