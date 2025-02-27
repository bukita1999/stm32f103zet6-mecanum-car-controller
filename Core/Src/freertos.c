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
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for MotorControlTas */
osThreadId_t MotorControlTasHandle;
const osThreadAttr_t MotorControlTas_attributes = {
    .name = "MotorControlTas",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for ServoControlTas */
osThreadId_t ServoControlTasHandle;
const osThreadAttr_t ServoControlTas_attributes = {
    .name = "ServoControlTas",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for CommunicationTa */
osThreadId_t CommunicationTaHandle;
const osThreadAttr_t CommunicationTa_attributes = {
    .name = "CommunicationTa",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for MonitorTask */
osThreadId_t MonitorTaskHandle;
const osThreadAttr_t MonitorTask_attributes = {
    .name = "MonitorTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for commandQueue */
osMessageQueueId_t commandQueueHandle;
const osMessageQueueAttr_t commandQueue_attributes = {
    .name = "commandQueue"};
/* Definitions for motorDataMutex */
osMutexId_t motorDataMutexHandle;
const osMutexAttr_t motorDataMutex_attributes = {
    .name = "motorDataMutex"};
/* Definitions for servoDataMutex */
osMutexId_t servoDataMutexHandle;
const osMutexAttr_t servoDataMutex_attributes = {
    .name = "servoDataMutex"};
/* Definitions for i2cMutex */
osMutexId_t i2cMutexHandle;
const osMutexAttr_t i2cMutex_attributes = {
    .name = "i2cMutex"};
/* Definitions for statusSemaphore */
osSemaphoreId_t statusSemaphoreHandle;
const osSemaphoreAttr_t statusSemaphore_attributes = {
    .name = "statusSemaphore"};
/* Definitions for systemEventGroup */
osEventFlagsId_t systemEventGroupHandle;
const osEventFlagsAttr_t systemEventGroup_attributes = {
    .name = "systemEventGroup"};

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
void MX_FREERTOS_Init(void)
{
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
  commandQueueHandle = osMessageQueueNew(16, sizeof(uint16_t), &commandQueue_attributes);

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
 * @brief Function implementing the MotorControlTas thread.
 * @param argument: Not used
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

  /* 初始化任务时间戳 */
  lastWakeTime = osKernelGetTickCount();

  /* Infinite loop */
  for (;;)
  {
    /* 获取当前时间 */
    currentTime = osKernelGetTickCount();

    /* 计算时间差 */
    uint32_t deltaTime = currentTime - lastWakeTime;

    /* 处理每个电机 */
    for (int i = 0; i < 4; i++)
    {
      /* 请求电机数据互斥量 */
      if (osMutexAcquire(motorDataMutexHandle, 10) == osOK)
      {
        /* 计算当前电机速度 */
        CalculateMotorSpeed(&systemState.motors[i], deltaTime);

        /* 计算PID输出 */
        systemState.motors[i].pidController.currentValue = (float)systemState.motors[i].currentSpeed;
        float pidOutput = PIDCompute(&systemState.motors[i].pidController);

        /* 更新电机输出 */
        /* 在实际系统中这里会根据PID输出设置PWM占空比 */
        
        /* 此处省略具体PWM设置代码 */

        /* 释放电机数据互斥量 */
        osMutexRelease(motorDataMutexHandle);
      }
      else
      {
        /* 设置互斥量获取失败标志 */
        systemState.systemFlags.motorError = 1;
        /* 在实际系统中可能需要添加错误处理 */
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
 * @brief Function implementing the ServoControlTas thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartServoControlTask */
void StartServoControlTask(void *argument)
{
  /* USER CODE BEGIN StartServoControlTask */
  
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
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

  /* Infinite loop */
  for (;;)
  {
    /* 等待电机数据更新事件 */
    uint32_t eventFlag = osEventFlagsWait(systemEventGroupHandle, 
                                         EVENT_MOTOR_UPDATE, 
                                         osFlagsWaitAny, 
                                         100);
    
    /* 如果事件触发 */
    if ((eventFlag & EVENT_MOTOR_UPDATE) == EVENT_MOTOR_UPDATE) {
      /* 请求电机数据互斥量 */
      if (osMutexAcquire(motorDataMutexHandle, 10) == osOK) {
        /* 格式化电机速度数据 */
        int len = sprintf(uartTxBuffer, "SPD,%d,%d,%d,%d\r\n", 
                systemState.motors[0].currentSpeed,
                systemState.motors[1].currentSpeed,
                systemState.motors[2].currentSpeed,
                systemState.motors[3].currentSpeed);
        
        /* 释放电机数据互斥量 */
        osMutexRelease(motorDataMutexHandle);
        
        /* 通过UART发送数据 */
        HAL_UART_Transmit(&huart1, (uint8_t*)uartTxBuffer, len, 100);
      }
    }
    osDelay(500);
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
 * @param encoderTimer: 编码器定时器
 * @param pwmChannel: PWM通道
 */
void MotorInit(Motor_t *motor, uint8_t id, TIM_TypeDef *encoderTimer, uint8_t pwmChannel)
{
  /* 初始化基本参数 */
  motor->id = id;
  motor->state = MOTOR_STOP;
  motor->targetSpeed = 0;
  motor->currentSpeed = 0;
  motor->encoderCount = 0;
  motor->lastEncoderCount = 0;
  motor->lastUpdateTime = 0;
  motor->encoderTimer = encoderTimer;
  motor->pwmChannel = pwmChannel;
  motor->errorCounter = 0;

  /* 清除错误标志 */
  motor->flags.encoderError = 0;
  motor->flags.overCurrent = 0;
  motor->flags.stalled = 0;

  /* 初始化PID控制器 */
  PIDInit(&motor->pidController, 1.0f, 0.1f, 0.01f, -1000.0f, 1000.0f);
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
  int32_t currentCount = motor->encoderTimer->CNT;

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
 * @brief 设置电机速度
 * @param motor: 电机结构体指针
 * @param speed: 目标速度
 */
void SetMotorSpeed(Motor_t *motor, int16_t speed)
{
  /* 更新目标速度 */
  motor->targetSpeed = speed;
  motor->pidController.targetValue = (float)speed;

  /* 设置电机状态 */
  if (speed > 0)
  {
    motor->state = MOTOR_FORWARD;
  }
  else if (speed < 0)
  {
    motor->state = MOTOR_BACKWARD;
  }
  else
  {
    motor->state = MOTOR_STOP;
  }

  /* 在实际系统中这里会处理PWM输出的设置 */
  /* 此处仅作示意 */
}

/**
 * @brief 系统初始化
 */
void MotorSystemInit(void)
{
  /* 初始化系统状态 */
  memset(&systemState, 0, sizeof(SystemState_t));

  /* 初始化四个电机 */
  MotorInit(&systemState.motors[0], 0, TIM2, 1);
  MotorInit(&systemState.motors[1], 1, TIM3, 2);
  MotorInit(&systemState.motors[2], 2, TIM4, 3);
  MotorInit(&systemState.motors[3], 3, TIM5, 4);

  /* 设置系统初始化标志 */
  systemState.systemFlags.initialized = 1;
}

/* USER CODE END Application */
