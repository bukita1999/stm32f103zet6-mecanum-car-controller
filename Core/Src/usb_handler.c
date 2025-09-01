/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usb_handler.c
  * @brief          : USB handler implementation
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
#include "usb_handler.h"
#include "usb_comm.h"
#include "usbd_cdc_if.h"
#include "cmsis_os.h"
#include <stdlib.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief USB任务实现函数（由freertos.c中的StartUsbTask调用）
 * @param argument: 未使用
 * @retval None
 */
/* USB任务的上下文变量 */
/* 文本模式配置 */
#define TEXT_BUFFER_SIZE    512     /* 文本数据缓冲区大小 (足够存放4个电机+系统状态的文本) */

static uint8_t textBuffer[TEXT_BUFFER_SIZE];         /* 文本数据缓冲区 */
#if 0  /* 文本模式下不再需要帧缓冲区 */
static uint8_t frameBuffer[FRAME_BUFFER_SIZE];      /* 完整帧缓冲区 */
#endif
/* 文本模式下不再需要批次ID计数器 */
#if 0
static uint16_t batchId = 0;                        /* 批次ID计数器 */
#endif
static uint32_t baseTimestamp = 0;                  /* 基准时间戳 */

/**
 * @brief 生成模拟的批量数据 (已废弃，现在使用文本模式)
 * @param header: 批量数据包头
 * @param data: 批量数据数组
 * @param count: 数据组数
 *
 * 注意: 此函数在文本模式下不再使用，仅保留以防将来需要
 */
#if 0  /* 注释掉不再使用的函数 */
static void GenerateBatchData(BatchHeader_t *header, BatchData_t *data, uint16_t count)
{
    /* 设置包头 */
    header->batch_id = batchId++;
    header->data_count = count;
    header->start_time = HAL_GetTick();

    /* 生成模拟数据 */
    for (uint16_t i = 0; i < count; i++) {
        data[i].timestamp = header->start_time + i * 10;  /* 每10ms一组数据 */

        /* 模拟四个电机的速度数据 */
        for (int motor = 0; motor < 4; motor++) {
            /* 速度范围: -1310 ~ +1310 RPM */
            data[i].speed[motor] = (int16_t)(rand() % 2621 - 1310);

            /* PWM百分比: 0-100 */
            data[i].pwm[motor] = (uint16_t)(rand() % 101);

            /* PID误差: -50.0 ~ +50.0 */
            data[i].error[motor] = (float)(rand() % 10001 - 5000) / 100.0f;
        }
    }
}
#endif

/**
 * @brief 打包并发送批量数据（文本格式，与UART监控输出格式一致）
 * @return 发送结果
 */
static uint8_t SendBatchData(void)
{
    uint16_t totalLen = 0;
    char *buffer = (char *)textBuffer;  // 使用文本缓冲区

    /* 获取互斥量保护电机数据 */
    if (osMutexAcquire(motorDataMutexHandle, 100) != osOK) {
        return USBD_FAIL;
    }

    /* 构建电机状态文本数据（与UART监控输出格式完全一致） */
    for (uint8_t i = 0; i < 4; i++) {
        /* 将浮点错误值转换为整数显示（乘以100保留2位小数精度） */
        int16_t errorInt = (int16_t)(systemState.motors[i].pidController.error * 100);

        /* 构建单行电机状态信息 */
        int len = snprintf(buffer + totalLen, TEXT_BUFFER_SIZE - totalLen,
                "Motor%d: Target:%d Current:%d RPM, PWM:%d%%, Error:%d.%02d\r\n",
                i + 1,
                systemState.motors[i].targetSpeed,
                systemState.motors[i].currentSpeed,
                systemState.motors[i].pwmPercent,
                errorInt / 100, abs(errorInt % 100));

        if (len > 0 && totalLen + len < TEXT_BUFFER_SIZE) {
            totalLen += len;
        } else {
            /* 缓冲区空间不足 */
            osMutexRelease(motorDataMutexHandle);
            return USBD_FAIL;
        }
    }

    /* 添加系统状态信息 */
    int len = snprintf(buffer + totalLen, TEXT_BUFFER_SIZE - totalLen,
            "System: Init=%d, PCA9685=%d, MotorErr=%d\r\n",
            systemState.systemFlags.initialized,
            systemState.systemFlags.pca9685Error,
            systemState.systemFlags.motorError);

    if (len > 0 && totalLen + len < TEXT_BUFFER_SIZE) {
        totalLen += len;
    }

    /* 释放互斥量 */
    osMutexRelease(motorDataMutexHandle);

    /* 发送文本数据 */
    return CDC_Transmit_Buffer((uint8_t *)buffer, totalLen);
}

/**
 * @brief USB任务初始化函数
 * @param argument: 未使用
 * @retval None
 */
void UsbTask_Init(void *argument)
{
    /* 初始化随机数种子 */
    srand(HAL_GetTick());

    /* 初始化基准时间戳 */
    baseTimestamp = HAL_GetTick();
}

/**
 * @brief USB任务主循环函数
 * @retval None
 */
void UsbTask_Loop(void)
{
    uint8_t result = SendBatchData();

    if (result == USBD_OK) {
        /* 发送成功，每100ms发送一批数据 */
        osDelay(100);
    } else {
        /* 发送失败，等待较短时间后重试 */
        osDelay(50);
    }
}

/**
 * @brief USB任务实现函数（由freertos.c中的StartUsbTask调用）
 * @param argument: 未使用
 * @retval None
 */
void UsbTask_Implementation(void *argument)
{
  /* 此函数现在由 freertos.c 中的 StartUsbTask 调用 */
  /* 不再包含循环逻辑，循环在 freertos.c 中处理 */
}

/**
 * @brief 初始化USB处理器
 */
void USBHandler_Init(void)
{
    /* USB处理器初始化 */
    /* 这里可以添加USB相关的初始化逻辑，比如：
     * - 初始化USB状态标志
     * - 准备数据缓冲区
     * - 设置USB回调函数
     */

    /* 目前不需要额外的初始化，USB设备已在MX_USB_DEVICE_Init()中初始化 */
}

/**
 * @brief USB CDC 接收数据回调函数
 * @param Buf: 接收到的数据缓冲区
 * @param Len: 数据长度
 * @retval None
 */
void USB_CDC_RxHandler(uint8_t* Buf, uint32_t Len)
{
  // 简单的回环测试 - 收到什么就发送什么
  CDC_Transmit_FS(Buf, Len);
}