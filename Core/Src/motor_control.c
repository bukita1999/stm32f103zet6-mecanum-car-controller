/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : motor_control.c
  * @brief          : Motor control implementation
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
#include "motor_control.h"
#include "pca9685.h"
#include "i2c.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define STARTUP_PWM 15

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* 速度小低通的状态（每路电机一个），初值 0 */
static float g_motorSpeedFilt[4] = {0};

/**
 * @brief 获取电机滤波后的速度
 * @param motorIndex: 电机索引 (0-3)
 * @return float: 滤波后的速度值
 */
float GetMotorFilteredSpeed(uint8_t motorIndex)
{
    if (motorIndex < 4) {
        return g_motorSpeedFilt[motorIndex];
    }
    return 0.0f;
}

/* I2C 输出节流（跟踪上次写入的原始 0~4095 PWM 值） */
static uint16_t g_lastPwmRaw[4] = {0};

/* PWM调试输出节流已移除（避免输出过于频繁） */

/* 用于UART发送的缓冲区 */
extern char uartTxBuffer[128];
extern UART_HandleTypeDef huart1;

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief 初始化电机
 * @param motor: 电机结构体指针
 * @param id: 电机ID
 * @param encoderTimer: 编码器定时器句柄
 * @param pwmChannel: PCA9685 PWM通道
 * @param dir0Port: 方向0控制GPIO端口 (IN1)
 * @param dir0Pin: 方向0控制GPIO引脚 (IN1)
 * @param dir1Port: 方向1控制GPIO端口 (IN2)
 * @param dir1Pin: 方向1控制GPIO引脚 (IN2)
 */
void MotorInit(Motor_t *motor, uint8_t id, TIM_HandleTypeDef *encoderTimer, uint8_t pwmChannel,
               GPIO_TypeDef *dir0Port, uint16_t dir0Pin, 
               GPIO_TypeDef *dir1Port, uint16_t dir1Pin)
{
  /* 初始化基本参数 */
  motor->id = id;
  motor->state = MOTOR_STOP;
  motor->direction = MOTOR_DIR_FORWARD;
  motor->targetSpeed = 0;
  motor->currentSpeed = 0;
  motor->pwmPercent = 0;
  motor->encoderCount = 0;
  motor->lastEncoderCount = 0;
  motor->lastUpdateTime = 0;
  motor->encoderTimer = encoderTimer;
  motor->pwmChannel = pwmChannel;
  
  /* 更新方向控制引脚 */
  motor->dir0Port = dir0Port;
  motor->dir0Pin = dir0Pin;
  motor->dir1Port = dir1Port;
  motor->dir1Pin = dir1Pin;
  
  motor->errorCounter = 0;

  /* 清除错误标志 */
  motor->flags.encoderError = 0;
  motor->flags.overCurrent = 0;
  motor->flags.stalled = 0;

  /* 初始化PID控制器 */
  PIDInit(&motor->pidController, MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD,
          MOTOR_PID_MIN, MOTOR_PID_MAX);

  /* 设置方向引脚为输出 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  /* 配置方向0引脚 */
  GPIO_InitStruct.Pin = dir0Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(dir0Port, &GPIO_InitStruct);
  
  /* 配置方向1引脚 */
  GPIO_InitStruct.Pin = dir1Pin;
  HAL_GPIO_Init(dir1Port, &GPIO_InitStruct);

  /* 初始状态：两个方向引脚都设为低，电机停止 */
  HAL_GPIO_WritePin(dir0Port, dir0Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(dir1Port, dir1Pin, GPIO_PIN_RESET);
}

/**
 * @brief 计算电机速度
 * @param motor: 电机结构体指针
 * @param deltaTime: 时间差(ms)
 * @return int16_t: 计算得到的速度(RPM, 电机轴转速-减速前)
 */
int16_t CalculateMotorSpeed(Motor_t *motor, uint32_t deltaTime)
{
  /* 读取当前编码器值 */
  int32_t currentCount = (int32_t)__HAL_TIM_GET_COUNTER(motor->encoderTimer);
  
  /* 计算编码器变化量 */
  int32_t deltaCount = currentCount - motor->lastEncoderCount;

  /* 处理编码器溢出 - STM32 TIM_Encoder模式使用16位计数器 */
  if (deltaCount > 32767) {
    /* 向下溢出：计数器从0跳到65535 */
    deltaCount -= 65536;
  } else if (deltaCount < -32768) {
    /* 向上溢出：计数器从65535跳到0 */
    deltaCount += 65536;
  }
  
  /* 计算速度（RPM） */
  int16_t speed = 0;
  if (deltaTime > 0) {
    speed = enc_delta_to_rpm(deltaCount, deltaTime);
  }
  
  /* 更新电机状态 */
  motor->lastEncoderCount = currentCount;
  motor->encoderCount += deltaCount;
  motor->currentSpeed = speed;
  
  return speed;
}

/**
 * @brief 设置电机速度
 * @param motor: 电机结构体指针
 * @param speed: 目标速度(RPM，带符号，电机轴转速-减速前)
 */
void SetMotorSpeed(Motor_t *motor, int16_t speed)
{
  /* 限制用户输入的速度范围（基于绝对值） */
  int16_t abs_speed = abs(speed);

  if (abs_speed > 0 && abs_speed < MOTOR_USER_MIN_RPM) {
    /* 小于最小值时，根据原方向设为最小值 */
    speed = (speed > 0) ? MOTOR_USER_MIN_RPM : -MOTOR_USER_MIN_RPM;
  } else if (abs_speed > MOTOR_USER_MAX_RPM) {
    /* 大于最大值时，根据原方向设为最大值 */
    speed = (speed > 0) ? MOTOR_USER_MAX_RPM : -MOTOR_USER_MAX_RPM;
  }

  int16_t prev = motor->targetSpeed;
  motor->targetSpeed = speed;
  motor->pidController.targetValue = (float)speed;

  if ((prev > 0 && speed <= 0) || (prev < 0 && speed >= 0)) {
    motor->pidController.errorSum = 0.0f;
    motor->pidController.lastError = 0.0f;
  }

  /* 固定起步 PWM，避免值过大导致一下子顶满 */
  int16_t initialPwm = (speed==0) ? 0 : ((speed>0)? +STARTUP_PWM : -STARTUP_PWM);
  SetMotorPWMPercentage(motor, initialPwm);

  /* 状态标记保持 */
  if (speed > 0)      motor->state = MOTOR_FORWARD;
  else if (speed < 0) motor->state = MOTOR_BACKWARD;
  else                motor->state = MOTOR_STOP;
}

/**
 * @brief 设置电机PWM百分比
 * @param motor: 电机结构体指针
 * @param pwmPercent: PWM百分比(-100到100)
 */
void SetMotorPWMPercentage(Motor_t *motor, int16_t pwmPercent)
{
  /* 限制PWM百分比范围 */
  if (pwmPercent > 100) pwmPercent = 100;
  if (pwmPercent < -100) pwmPercent = -100;
  
  motor->pwmPercent = (uint8_t)abs(pwmPercent);

  /* 根据PWM百分比符号设置方向 */
  if (pwmPercent > 0) {
    motor->direction = MOTOR_DIR_FORWARD;
    motor->state = MOTOR_FORWARD;
    /* 正向：DIR0=HIGH, DIR1=LOW */
    HAL_GPIO_WritePin(motor->dir0Port, motor->dir0Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor->dir1Port, motor->dir1Pin, GPIO_PIN_RESET);
  } else if (pwmPercent < 0) {
    motor->direction = MOTOR_DIR_BACKWARD;
    motor->state = MOTOR_BACKWARD;
    /* 反向：DIR0=LOW, DIR1=HIGH */
    HAL_GPIO_WritePin(motor->dir0Port, motor->dir0Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->dir1Port, motor->dir1Pin, GPIO_PIN_SET);
  } else {
    motor->state = MOTOR_STOP;
    /* 停止：DIR0=LOW, DIR1=LOW */
    HAL_GPIO_WritePin(motor->dir0Port, motor->dir0Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->dir1Port, motor->dir1Pin, GPIO_PIN_RESET);
  }

  /* 转换为PCA9685的PWM值 (0-4095) */
  /* 使用32位临时变量避免整数溢出：uint8_t(255) * 4095 = 1,044,225 > uint16_t最大值 */
  uint32_t temp = (uint32_t)motor->pwmPercent * 4095UL;
  uint16_t pwmValue = (uint16_t)(temp / 100U);
  
  /* I2C节流：只在值变化时才写入 */
  if (g_lastPwmRaw[motor->id] != pwmValue) {
    if (osMutexAcquire(i2cMutexHandle, 10) == osOK) {
      HAL_StatusTypeDef pwmStatus = PCA9685_SetPWM(&hi2c1, motor->pwmChannel, 0, pwmValue);
      if (pwmStatus == HAL_OK) {
        g_lastPwmRaw[motor->id] = pwmValue;

        /* PWM调试输出已移除 - 避免I2C通信时的USART干扰 */
      } else {
        systemState.systemFlags.pca9685Error = 1;

        /* PWM错误输出已移除 - 避免I2C通信时的USART干扰 */
      }
      osMutexRelease(i2cMutexHandle);
    } else {
      /* I2C忙时输出已移除 - 避免通信冲突 */
    }
  }
}

/**
 * @brief 系统初始化
 */
void MotorSystemInit(void)
{
  /* 初始化系统状态 */
  memset(&systemState, 0, sizeof(SystemState_t));

  /* 初始化PCA9685 */
  if (osMutexAcquire(i2cMutexHandle, 100) == osOK)
  {
    /* I2C连接检查输出已移除 - 避免通信冲突 */

    HAL_StatusTypeDef deviceCheck = HAL_I2C_IsDeviceReady(&hi2c1, PCA9685_I2C_ADDR << 1, 3, 100);
    if (deviceCheck == HAL_OK) {
      /* 尝试读取设备ID */
      uint8_t mode1_reg;
      PCA9685_ReadRegister(&hi2c1, PCA9685_MODE1, &mode1_reg); /* 读取但不检查返回值 */
      /* PCA9685检测成功输出已移除 */

      if (PCA9685_Init(&hi2c1) != HAL_OK)
      {
        systemState.systemFlags.pca9685Error = 1;
        /* PCA9685初始化失败输出已移除 */
      } else {
        systemState.systemFlags.pca9685Error = 0;
        /* PCA9685初始化成功输出已移除 */
      }
    } else {
      systemState.systemFlags.pca9685Error = 1;
      /* PCA9685设备未检测到输出已移除 */
    }

    /* I2C连接检查结束输出已移除 */

    osMutexRelease(i2cMutexHandle);
  } else {
    /* I2C互斥量获取失败输出已移除 */
  }

  /* 初始化四个电机 - 使用两个方向引脚 */
  MotorInit(&systemState.motors[0], 0, &htim2, 0, 
            GPIOC, MOTOR1_DIR0_Pin, GPIOF, MOTOR1_DIR1_Pin);
  MotorInit(&systemState.motors[1], 1, &htim3, 1, 
            GPIOC, MOTOR2_DIR0_Pin, GPIOF, MOTOR2_DIR1_Pin);
  MotorInit(&systemState.motors[2], 2, &htim4, 2, 
            GPIOC, MOTOR3_DIR0_Pin, GPIOF, MOTOR3_DIR1_Pin);
  MotorInit(&systemState.motors[3], 3, &htim5, 3, 
            GPIOC, MOTOR4_DIR0_Pin, GPIOF, MOTOR4_DIR1_Pin);

  /* 启动编码器计数 */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

  /* 设置系统初始化标志 */
  systemState.systemFlags.initialized = 1;
}

/* 电机控制任务的上下文变量（现在在freertos.c中定义） */
extern TickType_t motorControlLastWake;
extern uint32_t motorControlPeriod_ms;
static float motorControlDt_s;

/**
 * @brief 电机控制任务初始化函数
 * @param argument: 未使用
 * @retval None
 */
void MotorControlTask_Init(void *argument)
{
  /* 系统初始化 */
  MotorSystemInit();

  /* 严格等周期调度准备 */
  motorControlLastWake = xTaskGetTickCount();
  motorControlPeriod_ms = MOTOR_CONTROL_PERIOD;      // 例如 10
  motorControlDt_s = motorControlPeriod_ms / 1000.0f;

  /* 初始化所有电机的目标速度和PID参数 */
  for (uint8_t i = 0; i < 4; i++)
  {
    if (osMutexAcquire(motorDataMutexHandle, 100) == osOK)
    {
      /* 初始化PID控制器参数 - 根据实际电机特性调整 */
      PIDInit(&systemState.motors[i].pidController,
              MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD,
              -100, 100); // 输出范围为-100到100，对应PWM百分比

      /* 设置初始目标速度为0（带符号） */
      SetMotorSpeed(&systemState.motors[i], 0);
      osMutexRelease(motorDataMutexHandle);
    }
  }

  /* 电机控制任务调试输出已移除 - 避免I2C通信干扰 */

  /* PWM测试输出已移除 - 避免I2C通信干扰 */
}

/**
 * @brief 电机控制任务主循环函数
 * @retval None
 */
void MotorControlTask_Loop(void)
{
  /* 一次性获取互斥量处理所有电机，减少竞争 */
  if (osMutexAcquire(motorDataMutexHandle, 5) == osOK)
  {
    /* 处理每个电机 */
    for (uint8_t i = 0; i < 4; i++)
    {
      /* 1) 速度计算（deltaTime 使用固定周期） */
      CalculateMotorSpeed(&systemState.motors[i], motorControlPeriod_ms);

      /* 2) 速度小低通（抑制低速量化噪声） */
      float tau = 0.10f;                             // 100ms 时间常数，先保守
      float alpha = motorControlDt_s / (tau + motorControlDt_s);
      g_motorSpeedFilt[i] += alpha * ((float)systemState.motors[i].currentSpeed - g_motorSpeedFilt[i]);

      /* 3) 带符号 PID：测量/目标均用带符号速度 */
      systemState.motors[i].pidController.currentValue = g_motorSpeedFilt[i];
      /* targetValue 由 SetMotorSpeed() 设定为带符号，不需要 abs() */

      /* 4) 计算控制量（已含抗积分饱和与限幅） */
      float pidOut = PIDCompute(&systemState.motors[i].pidController, motorControlDt_s);

      /* 5) 直接用控制量决定方向与占空比（不再额外翻转号） */
      /* 使用四舍五入避免精度损失 */
      SetMotorPWMPercentage(&systemState.motors[i], (int16_t)(pidOut + (pidOut >= 0 ? 0.5f : -0.5f)));
    }

    /* 释放电机数据互斥量 */
    osMutexRelease(motorDataMutexHandle);
  }
  else
  {
    /* 互斥量获取失败处理 - 偶尔失败是可以接受的 */
    systemState.systemFlags.motorError = 1;
  }

  /* 触发电机数据更新事件 */
  osEventFlagsSet(systemEventGroupHandle, EVENT_MOTOR_UPDATE);
}

/**
 * @brief 电机控制任务实现函数（由freertos.c中的StartMotorControlTask调用）
 * @param argument: 未使用
 * @retval None
 */
void MotorControlTask_Implementation(void *argument)
{
  /* 此函数现在由 freertos.c 中的 StartMotorControlTask 调用 */
  /* 不再包含循环逻辑，循环在 freertos.c 中处理 */
}