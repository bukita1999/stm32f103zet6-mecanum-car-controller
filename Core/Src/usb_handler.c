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
#include "usbd_cdc_if.h"
#include "cmsis_os.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief USB Hello World 定时发送任务
 * @param argument: 未使用
 * @retval None
 */
void StartTask05(void *argument)
{
  uint8_t TxBuffer[] = "Hello World! From STM32 USB CDC Device To Virtual COM Port\r\n";
  uint8_t TxBufferLen = sizeof(TxBuffer) - 1; // 减去字符串结尾的\0
  
  for(;;){
    CDC_Transmit_FS(TxBuffer, TxBufferLen);
    osDelay(1000); // 每1秒发送一次
  }
}

/**
 * @brief 初始化USB处理器（现在只是一个空函数）
 */
void USBHandler_Init(void)
{
  // 不需要任何初始化，保留函数以避免编译错误
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