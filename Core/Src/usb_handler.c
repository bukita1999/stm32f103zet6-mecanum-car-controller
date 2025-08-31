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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
StreamBufferHandle_t usbRxStreamHandle;
static uint8_t usbRxStreamStorage[USB_RX_STREAM_SIZE];
static StaticStreamBuffer_t usbRxStreamCtrl;

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief USB接收任务函数
 * @param argument: 未使用
 * @retval None
 */
void StartTask05(void *argument)
{
  uint8_t in[128];                 // 每次从流里拉一批
  static uint8_t seg[USB_ENC_MAX]; // 暂存一帧的 COBS 片段
  size_t seg_len = 0;

  for(;;){
    size_t n = xStreamBufferReceive(usbRxStreamHandle, in, sizeof(in), portMAX_DELAY);
    for (size_t i=0; i<n; i++){
      uint8_t b = in[i];
      if (b == 0x00){
        // ----- 到达帧尾：解码 + 校验 + 交业务 -----
        uint8_t raw[USB_FRAME_MAX+4];
        size_t  raw_len = cobs_decode(raw, sizeof(raw), seg, seg_len);
        seg_len = 0;                              // 重置片段
        if (raw_len >= 4){
          uint32_t crc_got = *(uint32_t*)&raw[raw_len-4];
          uint32_t crc_cal = crc32_zlib(raw, raw_len-4);
          if (crc_got == crc_cal){
            USB_HandleFrame(raw, (uint16_t)(raw_len - 4)); // payload
          }
          // CRC 错误直接丢弃，避免污染上层
        }
      }else{
        // 累积片段；过长就丢弃这帧，避免溢出
        if (seg_len < sizeof(seg)) seg[seg_len++] = b;
        else seg_len = 0;
      }
    }
  }
}

/**
 * @brief 初始化USB流缓冲区
 */
void USBHandler_Init(void)
{
  usbRxStreamHandle = xStreamBufferCreateStatic(
      USB_RX_STREAM_SIZE, 1,            // 触发级别=1字节
      usbRxStreamStorage, &usbRxStreamCtrl);
}
