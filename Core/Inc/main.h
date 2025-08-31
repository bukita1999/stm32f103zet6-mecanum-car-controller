/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pca9685.h"
#include "robot_types.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* 编码器参数定义 */
#define ENCODER_BASE_PPR       11      /* 编码器基函数冲数 */
#define ENCODER_POLE_PAIRS     11      /* 磁环极对数 */
#define MOTOR_GEAR_RATIO       131      /* 电机减速比（示例值，请替换为实际值） */
#define ENCODER_COUNTS_PER_REV (ENCODER_BASE_PPR * ENCODER_POLE_PAIRS * 4 * MOTOR_GEAR_RATIO) /* 每转总脉冲数(含象限倍频) */

/* 系统相关定义 */
#define MOTOR_CONTROL_PERIOD   10      /* 电机控制周期 (ms) */
#define SYSTEM_TICK_FREQ       1000    /* 系统时钟频率 (Hz) */

/* 事件标志定义 */
#define EVENT_MOTOR_UPDATE     (1 << 0) /* 电机数据更新事件 */
#define EVENT_ERROR            (1 << 1) /* 错误事件 */
#define EVENT_COMMUNICATION    (1 << 2) /* 通信事件 */

/* 全局变量声明 */
extern SystemState_t systemState;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

/* 模块头文件包含 */
#include "motor_control.h"
#include "communication.h" 
#include "monitor.h"
#include "usb_handler.h"
#include "pid_controller.h"

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_DS1_Pin GPIO_PIN_5
#define LED_DS1_GPIO_Port GPIOE
#define MOTOR1_DIR1_Pin GPIO_PIN_0
#define MOTOR1_DIR1_GPIO_Port GPIOF
#define MOTOR2_DIR1_Pin GPIO_PIN_1
#define MOTOR2_DIR1_GPIO_Port GPIOF
#define MOTOR3_DIR1_Pin GPIO_PIN_2
#define MOTOR3_DIR1_GPIO_Port GPIOF
#define MOTOR4_DIR1_Pin GPIO_PIN_3
#define MOTOR4_DIR1_GPIO_Port GPIOF
#define MOTOR1_DIR0_Pin GPIO_PIN_0
#define MOTOR1_DIR0_GPIO_Port GPIOC
#define MOTOR2_DIR0_Pin GPIO_PIN_1
#define MOTOR2_DIR0_GPIO_Port GPIOC
#define MOTOR3_DIR0_Pin GPIO_PIN_2
#define MOTOR3_DIR0_GPIO_Port GPIOC
#define MOTOR4_DIR0_Pin GPIO_PIN_3
#define MOTOR4_DIR0_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
