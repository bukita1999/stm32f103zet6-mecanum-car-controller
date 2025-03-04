/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "pca9685.h"
#include "i2c.h"
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* 声明系统状态全局变量 */
SystemState_t systemState;

/* 用于UART发送的缓冲区 */
char uartTxBuffer[128];

/* 用于UART接收的缓冲区 */
uint8_t rxBuffer[64];
uint8_t rxIndex = 0;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MotorControlTas */
osThreadId_t MotorControlTasHandle;
const osThreadAttr_t MotorControlTas_attributes = {
  .name = "MotorControlTas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ServoControlTas */
osThreadId_t ServoControlTasHandle;
const osThreadAttr_t ServoControlTas_attributes = {
  .name = "ServoControlTas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CommunicationTa */
osThreadId_t CommunicationTaHandle;
const osThreadAttr_t CommunicationTa_attributes = {
  .name = "CommunicationTa",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MonitorTask */
osThreadId_t MonitorTaskHandle;
const osThreadAttr_t MonitorTask_attributes = {
  .name = "MonitorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for commandQueue */
osMessageQueueId_t commandQueueHandle;
const osMessageQueueAttr_t commandQueue_attributes = {
  .name = "commandQueue"
};
/* Definitions for motorDataMutex */
osMutexId_t motorDataMutexHandle;
const osMutexAttr_t motorDataMutex_attributes = {
  .name = "motorDataMutex"
};
/* Definitions for servoDataMutex */
osMutexId_t servoDataMutexHandle;
const osMutexAttr_t servoDataMutex_attributes = {
  .name = "servoDataMutex"
};
/* Definitions for i2cMutex */
osMutexId_t i2cMutexHandle;
const osMutexAttr_t i2cMutex_attributes = {
  .name = "i2cMutex"
};
/* Definitions for statusSemaphore */
osSemaphoreId_t statusSemaphoreHandle;
const osSemaphoreAttr_t statusSemaphore_attributes = {
  .name = "statusSemaphore"
};
/* Definitions for systemEventGroup */
osEventFlagsId_t systemEventGroupHandle;
const osEventFlagsAttr_t systemEventGroup_attributes = {
  .name = "systemEventGroup"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartMotorControlTask(void *argument);
void StartServoControlTask(void *argument);
void StartCommunicationTask(void *argument);
void StartMonitorTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of motorDataMutex */
  motorDataMutexHandle = osMutexNew(&motorDataMutex_attributes);

  /* creation of servoDataMutex */
  servoDataMutexHandle = osMutexNew(&servoDataMutex_attributes);

  /* creation of i2cMutex */
  i2cMutexHandle = osMutexNew(&i2cMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of statusSemaphore */
  statusSemaphoreHandle = osSemaphoreNew(1, 1, &statusSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of commandQueue */
  commandQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &commandQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of MotorControlTas */
  MotorControlTasHandle = osThreadNew(StartMotorControlTask, NULL, &MotorControlTas_attributes);

  /* creation of ServoControlTas */
  ServoControlTasHandle = osThreadNew(StartServoControlTask, NULL, &ServoControlTas_attributes);

  /* creation of CommunicationTa */
  CommunicationTaHandle = osThreadNew(StartCommunicationTask, NULL, &CommunicationTa_attributes);

  /* creation of MonitorTask */
  MonitorTaskHandle = osThreadNew(StartMonitorTask, NULL, &MonitorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of systemEventGroup */
  systemEventGroupHandle = osEventFlagsNew(&systemEventGroup_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartMotorControlTask */
/**
 * @brief 电机控制任务函数
 * @param argument: 未使用
 * @retval None
 */
/* USER CODE END Header_StartMotorControlTask */
void StartMotorControlTask(void *argument)
{
  /* USER CODE BEGIN StartMotorControlTask */
  /* 系统初始化 */
  MotorSystemInit();

  /* 任务时间戳变量 */
  uint32_t lastWakeTime;
  uint32_t currentTime;
  static uint32_t lastReportTime = 0;

  /* 初始化任务时间戳 */
  lastWakeTime = osKernelGetTickCount();

  /* 初始化所有电机的目标速度和PID参数 */
  for (uint8_t i = 0; i < 4; i++)
  {
    if (osMutexAcquire(motorDataMutexHandle, 100) == osOK)
    {
      /* 初始化PID控制器参数 - 根据实际电机特性调整 */
      systemState.motors[i].pidController.Kp = MOTOR_PID_KP;
      systemState.motors[i].pidController.Ki = MOTOR_PID_KI;
      systemState.motors[i].pidController.Kd = MOTOR_PID_KD;

      /* 设置初始目标速度为0 */
      SetMotorSpeed(&systemState.motors[i], 0);
      osMutexRelease(motorDataMutexHandle);
    }
  }

  /* 输出调试信息 */
  snprintf(uartTxBuffer, sizeof(uartTxBuffer), "Motor control task started, PID velocity control enabled for all motors\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), 100);

  /* 无限循环 */
  for (;;)
  {
    /* 获取当前时间 */
    currentTime = osKernelGetTickCount();
    uint32_t deltaTime = currentTime - lastWakeTime;

    /* 处理每个电机 */
    for (uint8_t i = 0; i < 4; i++)
    {
      /* 请求电机数据互斥量 */
      if (osMutexAcquire(motorDataMutexHandle, 10) == osOK)
      {
        /* 计算当前电机速度 */
        CalculateMotorSpeed(&systemState.motors[i], deltaTime);

        /* 添加PID控制逻辑 */
        if (systemState.motors[i].state != MOTOR_STOP)
        {
          /* 更新PID控制器的当前值 */
          systemState.motors[i].pidController.currentValue = (float)abs(systemState.motors[i].currentSpeed);

          /* 计算PID输出 */
          float pidOutput = PIDCompute(&systemState.motors[i].pidController);

          /* 应用PID输出到PWM百分比 */
          int16_t newPwmPercent = (int16_t)pidOutput;

          /* 保持方向一致 */
          if (systemState.motors[i].direction == MOTOR_DIR_BACKWARD)
          {
            newPwmPercent = -newPwmPercent;
          }

          /* 设置新的PWM值 */
          SetMotorPWMPercentage(&systemState.motors[i], newPwmPercent);
        }
        else
        {
          /* 如果电机停止，确保PWM为0 */
          SetMotorPWMPercentage(&systemState.motors[i], 0);
        }

        /* 释放电机数据互斥量 */
        osMutexRelease(motorDataMutexHandle);
      }
      else
      {
        /* 互斥量获取失败处理 */
        systemState.systemFlags.motorError = 1;
      }
    }

    /* 每5秒输出一次所有电机的状态 */
    if ((currentTime - lastReportTime) > 5000)
    {
      lastReportTime = currentTime;

      for (uint8_t i = 0; i < 4; i++)
      {
        if (osMutexAcquire(motorDataMutexHandle, 10) == osOK)
        {
          snprintf(uartTxBuffer, sizeof(uartTxBuffer),
                   "Motor%d: Target:%d Current:%d RPM, PWM:%d%%, Error:%.2f\r\n",
                   i + 1, 
                   systemState.motors[i].targetSpeed,
                   systemState.motors[i].currentSpeed,
                   systemState.motors[i].pwmPercent,
                   systemState.motors[i].pidController.error);
          
          osMutexRelease(motorDataMutexHandle);
          HAL_UART_Transmit(&huart1, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), 100);
        }
      }
    }

    /* 触发电机数据更新事件 */
    osEventFlagsSet(systemEventGroupHandle, EVENT_MOTOR_UPDATE);

    /* 更新时间戳 */
    lastWakeTime = currentTime;

    /* 等待下一个周期(10ms) */
    osDelay(MOTOR_CONTROL_PERIOD);
  }
  /* USER CODE END StartMotorControlTask */
}

/* USER CODE BEGIN Header_StartServoControlTask */
/**
 * @brief 舵机控制任务函数
 * @param argument: 未使用
 * @retval None
 */
/* USER CODE END Header_StartServoControlTask */
void StartServoControlTask(void *argument)
{
  /* USER CODE BEGIN StartServoControlTask */
  /* 等待系统初始化完成 */
  osDelay(200);

  /* 舵机控制周期 */
  const uint32_t SERVO_CONTROL_PERIOD = 20; /* 20ms */
  uint16_t command;
  uint8_t servoId, angle;

  /* 设置第一个舵机角度为90度 */
  /* 对于90度位置，PCA9685将产生7.5%占空比PWM信号，约307计数值 */
  /* 但根据我们的SERVO_MIN_PULSE和SERVO_MAX_PULSE设置，实际值约为375 */
  if (osMutexAcquire(servoDataMutexHandle, 100) == osOK)
  {
    SetServoAngle(&systemState.servos[0], 90);
    osMutexRelease(servoDataMutexHandle);

    /* 输出调试信息 */
    snprintf(uartTxBuffer, sizeof(uartTxBuffer),
             "Servo1 set to 90 degrees, PWM pulse width: ~1.5ms (7.5%% duty cycle)\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), 100);
  }

  /* 无限循环 */
  for (;;)
  {
    /* 接收命令并执行舵机控制 - 实际应用中这里会处理接收到的命令 */
    if (osMessageQueueGet(commandQueueHandle, &command, NULL, 0) == osOK)
    {
      /* 解析命令 - 示例格式：高8位为舵机ID，低8位为角度值 */
      servoId = (command >> 8) & 0xFF;
      angle = command & 0xFF;

      if (servoId < 8)
      {
        /* 使用简化接口设置舵机角度，确保线程安全 */
        if (osMutexAcquire(servoDataMutexHandle, 10) == osOK)
        {
          SetServoAngle(&systemState.servos[servoId], (float)angle);
          osMutexRelease(servoDataMutexHandle);
        }
        else
        {
          systemState.systemFlags.servoError = 1;
        }
      }
    }

    /* 等待下一个周期 */
    osDelay(SERVO_CONTROL_PERIOD);
  }
  /* USER CODE END StartServoControlTask */
}

/* USER CODE BEGIN Header_StartCommunicationTask */
/**
 * @brief Function implementing the CommunicationTa thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartCommunicationTask */
void StartCommunicationTask(void *argument)
{
  /* USER CODE BEGIN StartCommunicationTask */
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
        /* 格式化电机速度数据 */
        int len = sprintf(uartTxBuffer, "SPD,%d,%d,%d,%d\r\n",
                          systemState.motors[0].currentSpeed,
                          systemState.motors[1].currentSpeed,
                          systemState.motors[2].currentSpeed,
                          systemState.motors[3].currentSpeed);

        /* 释放电机数据互斥量 */
        osMutexRelease(motorDataMutexHandle);

        /* 通过UART发送数据 */
        //HAL_UART_Transmit(&huart1, (uint8_t *)uartTxBuffer, len, 100);
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
          int len = sprintf(uartTxBuffer, "Received: %s#\r\n", rxBuffer);
          HAL_UART_Transmit(&huart1, (uint8_t *)uartTxBuffer, len, 100);
          
          /* 解析PID参数设置命令: $PID,motorId,Kp,Ki,Kd */
          char *token = strtok((char *)&rxBuffer[5], ",");
          
          if (token != NULL)
          {
            int motorId = atoi(token);
            token = strtok(NULL, ",");
            
            if (token != NULL && motorId >= 0 && motorId < 4)
            {
              float kp = atof(token);
              token = strtok(NULL, ",");
              
              if (token != NULL)
              {
                float ki = atof(token);
                token = strtok(NULL, ",");
                
                if (token != NULL)
                {
                  float kd = atof(token);
                  
                  /* 更新PID参数 */
                  if (osMutexAcquire(motorDataMutexHandle, 10) == osOK)
                  {
                    systemState.motors[motorId].pidController.Kp = kp;
                    systemState.motors[motorId].pidController.Ki = ki;
                    systemState.motors[motorId].pidController.Kd = kd;
                    osMutexRelease(motorDataMutexHandle);
                    
                    /* 确认命令接收 */
                    len = sprintf(uartTxBuffer, "PID,M%d,%.2f,%.2f,%.2f\r\n",
                                        motorId, kp, ki, kd);
                    HAL_UART_Transmit(&huart1, (uint8_t *)uartTxBuffer, len, 100);
                  }
                }
              }
            }
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
  /* USER CODE END StartCommunicationTask */
}

/* USER CODE BEGIN Header_StartMonitorTask */
/**
 * @brief Function implementing the MonitorTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartMonitorTask */
void StartMonitorTask(void *argument)
{
  /* USER CODE BEGIN StartMonitorTask */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartMonitorTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
 * @brief 初始化PID控制器
 * @param pid: PID控制器结构体指针
 * @param kp: 比例系数
 * @param ki: 积分系数
 * @param kd: 微分系数
 * @param min: 输出下限
 * @param max: 输出上限
 */
void PIDInit(PIDController_t *pid, float kp, float ki, float kd, float min, float max)
{
  /* 设置PID参数 */
  pid->Kp = kp;
  pid->Ki = ki;
  pid->Kd = kd;

  /* 设置输出限制 */
  pid->outputMin = min;
  pid->outputMax = max;

  /* 初始化PID状态 */
  pid->targetValue = 0;
  pid->currentValue = 0;
  pid->error = 0;
  pid->lastError = 0;
  pid->errorSum = 0;
  pid->output = 0;
}

/**
 * @brief 计算PID输出
 * @param pid: PID控制器结构体指针
 * @return float: PID计算结果
 */
float PIDCompute(PIDController_t *pid)
{
  /* 计算误差 */
  pid->error = pid->targetValue - pid->currentValue;

  /* 计算积分项 */
  pid->errorSum += pid->error;

  /* 防止积分饱和 */
  if (pid->errorSum > pid->outputMax)
  {
    pid->errorSum = pid->outputMax;
  }
  else if (pid->errorSum < pid->outputMin)
  {
    pid->errorSum = pid->outputMin;
  }

  /* 计算微分项 */
  float errorDiff = pid->error - pid->lastError;

  /* 计算PID输出 */
  pid->output = pid->Kp * pid->error + pid->Ki * pid->errorSum + pid->Kd * errorDiff;

  /* 输出限幅 */
  if (pid->output > pid->outputMax)
  {
    pid->output = pid->outputMax;
  }
  else if (pid->output < pid->outputMin)
  {
    pid->output = pid->outputMin;
  }

  /* 保存当前误差 */
  pid->lastError = pid->error;

  return pid->output;
}

/**
 * @brief 初始化电机
 * @param motor: 电机结构体指针
 * @param id: 电机ID
 * @param encoderTimer: 编码器定时器句柄
 * @param pwmChannel: PCA9685 PWM通道
 * @param dirPort: 方向控制GPIO端口
 * @param dirPin: 方向控制GPIO引脚
 */
void MotorInit(Motor_t *motor, uint8_t id, TIM_HandleTypeDef *encoderTimer, uint8_t pwmChannel,
               GPIO_TypeDef *dirPort, uint16_t dirPin)
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
  motor->dirPort = dirPort;
  motor->dirPin = dirPin;
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
  GPIO_InitStruct.Pin = dirPin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(dirPort, &GPIO_InitStruct);

  /* 初始方向设为正向 */
  HAL_GPIO_WritePin(dirPort, dirPin, GPIO_PIN_RESET); /* 根据您的硬件可能需要调整 */
}

/**
 * @brief 计算电机速度
 * @param motor: 电机结构体指针
 * @param deltaTime: 时间差(ms)
 * @return int16_t: 计算得到的速度(RPM)
 */
int16_t CalculateMotorSpeed(Motor_t *motor, uint32_t deltaTime)
{
  /* 读取当前编码器计数 */
  int32_t currentCount = __HAL_TIM_GET_COUNTER(motor->encoderTimer);

  /* 计算编码器计数变化量 */
  int32_t encoderDelta = currentCount - motor->lastEncoderCount;

  /* 处理计数器溢出情况 */
  if (encoderDelta > 32768)
  { /* 正向溢出 */
    encoderDelta = encoderDelta - 65536;
  }
  else if (encoderDelta < -32768)
  { /* 负向溢出 */
    encoderDelta = encoderDelta + 65536;
  }

  /* 更新编码器计数 */
  motor->lastEncoderCount = currentCount;
  motor->encoderCount += encoderDelta;

  /* 计算速度(转/分) */
  /* 公式: speed = (脉冲数 / 每转脉冲数) * (60秒/分 / 时间秒) */
  int16_t speed = (int16_t)((float)encoderDelta * 60.0f * 1000.0f /
                            ((float)ENCODER_COUNTS_PER_REV * (float)deltaTime));

  /* 更新电机速度 */
  motor->currentSpeed = speed;

  /* 检测堵转情况 */
  if (abs(motor->targetSpeed) > 20 && abs(motor->currentSpeed) < 5)
  {
    motor->errorCounter++;
    if (motor->errorCounter > 50)
    { /* 连续50次检测到可能堵转 */
      motor->flags.stalled = 1;
    }
  }
  else
  {
    /* 恢复正常时重置计数器 */
    motor->errorCounter = 0;
  }

  return speed;
}

/**
 * @brief 设置电机速度(简化接口)
 * @param motor: 电机结构体指针
 * @param speed: 速度(-100到100，负值表示反向)
 */
void SetMotorPWMPercentage(Motor_t *motor, int16_t pwmPercent)
{
  /* 限制速度范围 */
  if (pwmPercent > 100)
    pwmPercent = 100;
  if (pwmPercent < -100)
    pwmPercent = -100;

  /* 设置方向 */
  if (pwmPercent >= 0)
  {
    motor->direction = MOTOR_DIR_FORWARD;
    HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, GPIO_PIN_RESET); /* 正向 */
    motor->state = (pwmPercent > 0) ? MOTOR_FORWARD : MOTOR_STOP;

    /* 设置PWM百分比 */
    motor->pwmPercent = abs(pwmPercent);

    /* 计算PWM值(0-4095) */
    uint16_t pwmValue = (motor->pwmPercent * 4095) / 100;

    /* 通过I2C设置PWM */
    if (osMutexAcquire(i2cMutexHandle, 10) == osOK)
    {
      SetMotorPWM(&hi2c1, motor->pwmChannel, pwmValue);
      osMutexRelease(i2cMutexHandle);
    }
  }
  else
  {
    motor->direction = MOTOR_DIR_BACKWARD;
    HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, GPIO_PIN_SET); /* 反向 */
    motor->state = MOTOR_BACKWARD;

    /* 设置PWM百分比 */
    motor->pwmPercent = abs(pwmPercent);

    /* 计算PWM值(0-4095) */
    uint16_t pwmValue = ((100 - motor->pwmPercent) * 4095) / 100;

    /* 通过I2C设置PWM */
    if (osMutexAcquire(i2cMutexHandle, 10) == osOK)
    {
      SetMotorPWM(&hi2c1, motor->pwmChannel, pwmValue);
      osMutexRelease(i2cMutexHandle);
    }
  }
}

/**
 * @brief 设置电机目标速度
 * @param motor: 电机结构体指针
 * @param speed: 目标速度(RPM，负值表示反向)
 */
void SetMotorSpeed(Motor_t *motor, int16_t speed)
{
  /* 设置目标速度(用于PID控制) */
  motor->targetSpeed = abs(speed);
  motor->pidController.targetValue = (float)abs(speed);

  /* 设置方向标志 */
  motor->direction = (speed >= 0) ? MOTOR_DIR_FORWARD : MOTOR_DIR_BACKWARD;

  /* 更新电机状态 */
  if (speed > 0)
    motor->state = MOTOR_FORWARD;
  else if (speed < 0)
    motor->state = MOTOR_BACKWARD;
  else
    motor->state = MOTOR_STOP;

  /* 初始PWM设置 - 后续由PID控制器调整 */
  /* 此处可以设置一个估计的初始PWM值，或者保持当前值让PID控制器逐渐调整 */
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
    if (PCA9685_Init(&hi2c1) != HAL_OK)
    {
      systemState.systemFlags.pca9685Error = 1;
    }
    osMutexRelease(i2cMutexHandle);
  }

  /* 初始化四个电机 - 使用简化接口 */
  /* 注意：这里的GPIO端口和引脚需要根据实际硬件连接进行调整 */
  MotorInit(&systemState.motors[0], 0, &htim2, 0, GPIOC, GPIO_PIN_0);
  MotorInit(&systemState.motors[1], 1, &htim3, 1, GPIOC, GPIO_PIN_1);
  MotorInit(&systemState.motors[2], 2, &htim4, 2, GPIOC, GPIO_PIN_2);
  MotorInit(&systemState.motors[3], 3, &htim5, 3, GPIOC, GPIO_PIN_3);

  /* 初始化八个舵机 - 简化接口 */
  ServoInit(&systemState.servos[0], 0, 4);  /* 通道4 */
  ServoInit(&systemState.servos[1], 1, 5);  /* 通道5 */
  ServoInit(&systemState.servos[2], 2, 6);  /* 通道6 */
  ServoInit(&systemState.servos[3], 3, 7);  /* 通道7 */
  ServoInit(&systemState.servos[4], 4, 8);  /* 通道8 */
  ServoInit(&systemState.servos[5], 5, 9);  /* 通道9 */
  ServoInit(&systemState.servos[6], 6, 10); /* 通道10 */
  ServoInit(&systemState.servos[7], 7, 11); /* 通道11 */

  /* 启动编码器计数 */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

  /* 设置系统初始化标志 */
  systemState.systemFlags.initialized = 1;
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

/* USER CODE END Application */
