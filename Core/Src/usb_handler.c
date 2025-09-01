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
#define BATCH_SIZE          100     /* 每批发送100组数据 */
#define BATCH_DATA_SIZE     (sizeof(BatchHeader_t) + BATCH_SIZE * sizeof(BatchData_t))
#define TLV_BUFFER_SIZE     (BATCH_DATA_SIZE + 10)  /* TLV数据缓冲区 */
#define FRAME_BUFFER_SIZE   (TLV_BUFFER_SIZE + CRC32_SIZE + COBS_OVERHEAD + 10)

static uint8_t tlvBuffer[TLV_BUFFER_SIZE];           /* TLV数据缓冲区 */
static uint8_t frameBuffer[FRAME_BUFFER_SIZE];      /* 完整帧缓冲区 */
static uint16_t batchId = 0;                        /* 批次ID计数器 */
static uint32_t baseTimestamp = 0;                  /* 基准时间戳 */

/**
 * @brief 生成模拟的批量数据
 * @param header: 批量数据包头
 * @param data: 批量数据数组
 * @param count: 数据组数
 */
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

/**
 * @brief 打包并发送批量数据
 * @return 发送结果
 */
static uint8_t SendBatchData(void)
{
    /* 生成批量数据 */
    BatchHeader_t *header = (BatchHeader_t *)tlvBuffer;
    BatchData_t *data = (BatchData_t *)(tlvBuffer + sizeof(BatchHeader_t));

    GenerateBatchData(header, data, BATCH_SIZE);

    /* 计算TLV数据总长度 */
    uint16_t tlvDataLen = sizeof(BatchHeader_t) + BATCH_SIZE * sizeof(BatchData_t);

    /* 构建TLV数据 */
    uint8_t *p = tlvBuffer;
    p = tlv_put(p, TLV_BATCH_DATA, tlvBuffer, tlvDataLen);

    uint16_t payloadLen = p - tlvBuffer;

    /* 构建完整帧 */
    size_t frameLen = usb_build_frame(frameBuffer, FRAME_BUFFER_SIZE, tlvBuffer, payloadLen);

    if (frameLen == 0) {
        return USBD_FAIL;  /* 构建帧失败 */
    }

    /* 发送帧数据 */
    return CDC_Transmit_Buffer(frameBuffer, (uint16_t)frameLen);
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
        /* 发送成功，每2秒发送一批数据 */
        osDelay(2000);
    } else {
        /* 发送失败，等待较短时间后重试 */
        osDelay(100);
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