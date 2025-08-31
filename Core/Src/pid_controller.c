/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : pid_controller.c
  * @brief          : PID controller implementation
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
#include "pid_controller.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

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
 * @param dt: 时间步长(秒)
 * @return float: PID计算结果
 */
float PIDCompute(PIDController_t *pid, float dt)
{
  /* 误差与基本项 */
  float e = pid->targetValue - pid->currentValue;

  /* 先"候选"积分，再基于饱和判定是否采纳 */
  float I_candidate = pid->errorSum + e * dt;

  /* 误差微分（保留你的做法；若需 DoM 可后续再改） */
  float d = 0.0f;
  if (dt > 0.0f) d = (e - pid->lastError) / dt;

  /* 未饱和输出 */
  float u_unsat = pid->Kp * e + pid->Ki * I_candidate + pid->Kd * d;

  /* 限幅 */
  float u = u_unsat;
  if (u > pid->outputMax) u = pid->outputMax;
  else if (u < pid->outputMin) u = pid->outputMin;

  /* 条件积分（抗饱和）：未饱和，或积分有助于"脱离饱和"时才采纳 */
  if ((u == u_unsat) || (u * e > 0.0f))
  {
    pid->errorSum = I_candidate;
  }
  /* else: 保持原有 pid->errorSum，不再"越积越多" */

  pid->error = e;
  pid->lastError = e;
  pid->output = u;
  return u;
}
