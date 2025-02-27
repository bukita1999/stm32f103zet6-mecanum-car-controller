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

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* PID控制器结构体 */
typedef struct {
    float Kp;                  /* 比例系数 */
    float Ki;                  /* 积分系数 */
    float Kd;                  /* 微分系数 */
    float targetValue;         /* 目标值 */
    float currentValue;        /* 当前值 */
    float error;               /* 当前误差 */
    float lastError;           /* 上一次误差 */
    float errorSum;            /* 误差累积（积分项） */
    float output;              /* PID输出 */
    float outputMax;           /* 输出上限 */
    float outputMin;           /* 输出下限 */
} PIDController_t;

/* 电机状态枚举 */
typedef enum {
    MOTOR_STOP = 0,            /* 电机停止 */
    MOTOR_FORWARD = 1,         /* 电机正向运行 */
    MOTOR_BACKWARD = 2,        /* 电机反向运行 */
    MOTOR_ERROR = 3            /* 电机错误状态 */
} MotorState_t;

/* 电机结构体 */
typedef struct {
    uint8_t id;                /* 电机ID (0-3) */
    MotorState_t state;        /* 电机状态 */
    int16_t targetSpeed;       /* 目标速度 (RPM) */
    int16_t currentSpeed;      /* 当前速度 (RPM) */
    int32_t encoderCount;      /* 编码器计数 */
    int32_t lastEncoderCount;  /* 上次编码器计数 */
    uint32_t lastUpdateTime;   /* 上次更新时间 (ms) */
    TIM_TypeDef* encoderTimer; /* 编码器定时器 */
    uint8_t pwmChannel;        /* PWM通道 */
    PIDController_t pidController; /* 速度PID控制器 */
    uint8_t errorCounter;      /* 错误计数器 */
    struct {
        uint8_t encoderError : 1;  /* 编码器错误标志 */
        uint8_t overCurrent : 1;   /* 过流标志 */
        uint8_t stalled : 1;       /* 堵转标志 */
        uint8_t reserved : 5;      /* 保留位 */
    } flags;                   /* 使用位域优化内存 */
} Motor_t;

/* 系统状态结构体 */
typedef struct {
    Motor_t motors[4];         /* 四个电机 */
    struct {
        uint8_t initialized : 1;   /* 系统初始化标志 */
        uint8_t motorError : 1;    /* 电机错误标志 */
        uint8_t servoError : 1;    /* 舵机错误标志 */
        uint8_t communicationError : 1; /* 通信错误标志 */
        uint8_t lowBattery : 1;    /* 低电量标志 */
        uint8_t emergencyStop : 1; /* 紧急停止标志 */
        uint8_t reserved : 2;      /* 保留位 */
    } systemFlags;             /* 系统状态标志，使用位域优化内存 */
} SystemState_t;

/* 编码器参数定义 */
#define ENCODER_BASE_PPR       11      /* 编码器基函数冲数 */
#define ENCODER_POLE_PAIRS     11      /* 磁环极对数 */
#define MOTOR_GEAR_RATIO       34      /* 电机减速比（示例值，请替换为实际值） */
#define ENCODER_COUNTS_PER_REV (ENCODER_BASE_PPR * ENCODER_POLE_PAIRS * 4 * MOTOR_GEAR_RATIO) /* 每转总脉冲数(含象限倍频) */

/* 系统相关定义 */
#define MOTOR_CONTROL_PERIOD   10      /* 电机控制周期 (ms) */
#define SYSTEM_TICK_FREQ       1000    /* 系统时钟频率 (Hz) */

/* 事件标志定义 */
#define EVENT_MOTOR_UPDATE     (1 << 0) /* 电机数据更新事件 */
#define EVENT_SERVO_UPDATE     (1 << 1) /* 舵机数据更新事件 */
#define EVENT_ERROR            (1 << 2) /* 错误事件 */
#define EVENT_COMMUNICATION    (1 << 3) /* 通信事件 */

/* 全局变量声明 */
extern SystemState_t systemState;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

/* 函数声明 */
void MotorSystemInit(void);
void MotorInit(Motor_t* motor, uint8_t id, TIM_TypeDef* encoderTimer, uint8_t pwmChannel);
void PIDInit(PIDController_t* pid, float kp, float ki, float kd, float min, float max);
float PIDCompute(PIDController_t* pid);
int16_t CalculateMotorSpeed(Motor_t* motor, uint32_t deltaTime);
void SetMotorSpeed(Motor_t* motor, int16_t speed);
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

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
