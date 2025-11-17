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
#include <stddef.h>
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
/* USB Telemetry frame configuration */
#define USB_FRAME_VERSION        0x01
#define USB_MOTOR_COUNT          4
#define USB_FRAME_SYNC           0xAA55
#define USB_FRAME_TRAIL          0x55AA
#define USB_FRAME_BUFFER_SIZE    64

typedef struct __attribute__((packed)) {
    uint16_t sync;
    uint8_t  version;
    uint8_t  reserved;
    uint16_t frame_length;
    uint32_t timestamp_ms;
} UsbFrameHeader_t;

typedef struct __attribute__((packed)) {
    uint8_t  motor_id;
    int16_t  target_rpm;
    int16_t  current_rpm;
    uint16_t pwm_percent;
} UsbMotorRecord_t;

typedef struct __attribute__((packed)) {
    UsbFrameHeader_t header;
    UsbMotorRecord_t motors[USB_MOTOR_COUNT];
    uint32_t crc32;
    uint16_t trail;
} UsbTelemetryFrame_t;

static uint8_t usbFrameBuffer[USB_FRAME_BUFFER_SIZE];

_Static_assert(sizeof(UsbTelemetryFrame_t) <= USB_FRAME_BUFFER_SIZE, "USB frame buffer too small");

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
        data[i].timestamp = header->start_time + i * 20;  /* 每20ms一组数据 */

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
 * @brief 打包并发送 USB 二进制遥测帧
 * @return 发送结果
 */
static uint8_t SendBatchData(void)
{
    UsbTelemetryFrame_t *frame = (UsbTelemetryFrame_t *)usbFrameBuffer;

    frame->header.sync = USB_FRAME_SYNC;
    frame->header.version = USB_FRAME_VERSION;
    frame->header.reserved = 0;
    frame->header.frame_length = sizeof(UsbTelemetryFrame_t);
    frame->header.timestamp_ms = HAL_GetTick();

    /* 获取互斥量保护电机数据 */
    if (osMutexAcquire(motorDataMutexHandle, 100) != osOK) {
        return USBD_FAIL;
    }

    for (uint8_t i = 0; i < USB_MOTOR_COUNT; i++) {
        frame->motors[i].motor_id = i + 1;
        frame->motors[i].target_rpm = systemState.motors[i].targetSpeed;
        frame->motors[i].current_rpm = systemState.motors[i].currentSpeed;
        frame->motors[i].pwm_percent = systemState.motors[i].pwmPercent;
    }

    /* 释放互斥量 */
    osMutexRelease(motorDataMutexHandle);

    frame->crc32 = crc32_zlib((uint8_t *)frame, offsetof(UsbTelemetryFrame_t, crc32));
    frame->trail = USB_FRAME_TRAIL;

    /* 发送二进制帧 */
    return CDC_Transmit_Buffer((uint8_t *)frame, frame->header.frame_length);
}

/**
 * @brief USB任务初始化函数
 * @param argument: 未使用
 * @retval None
 */
void UsbTask_Init(void *argument)
{
    (void)argument;
}

/**
 * @brief USB任务主循环函数
 * @retval None
 */
void UsbTask_Loop(void)
{
    uint8_t result = SendBatchData();

    if (result == USBD_OK) {
        /* 发送成功，每20ms发送一批数据 */
        osDelay(20);
    } else {
        /* 发送失败，等待较短时间后重试 */
        osDelay(5);
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
