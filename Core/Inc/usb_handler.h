/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usb_handler.h
  * @brief          : Header for USB handler module
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
#ifndef __USB_HANDLER_H
#define __USB_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void StartTask05(void *argument);
void USBHandler_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __USB_HANDLER_H */
