/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_COMM_H
#define __USB_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/* TLV数据类型定义 */
typedef enum {
    TLV_TELEMETRY = 0x10,     /* 遥测数据 */
    TLV_SPEED_UNIT = 0x11,    /* 速度单位声明 */
    TLV_BATCH_DATA = 0x20,    /* 批量数据包 */
} TLV_Type_t;

/* TLV数据结构 */
typedef struct {
    uint8_t  type;      /* 数据类型 */
    uint16_t length;    /* 数据长度 */
    uint8_t  value[];   /* 数据内容 */
} TLV_t;

/* 批量数据结构 - 模拟遥测数据 */
typedef struct __attribute__((packed)) {
    uint32_t timestamp;    /* 时间戳 */
    int16_t speed[4];     /* 四个电机速度 */
    uint16_t pwm[4];      /* 四个电机PWM值 */
    float error[4];       /* 四个电机误差 */
} BatchData_t;

/* 批量数据包头 */
typedef struct __attribute__((packed)) {
    uint16_t batch_id;     /* 批次ID */
    uint16_t data_count;   /* 数据组数 */
    uint32_t start_time;   /* 开始时间戳 */
} BatchHeader_t;

/* Exported constants --------------------------------------------------------*/
#define MAX_FRAME_SIZE    256     /* 最大帧长度 */
#define COBS_OVERHEAD     1       /* COBS编码开销 */
#define CRC32_SIZE        4       /* CRC32校验码大小 */
#define MAX_PAYLOAD_SIZE  (MAX_FRAME_SIZE - COBS_OVERHEAD - CRC32_SIZE)

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
size_t cobs_encode(uint8_t *out, size_t out_max, const uint8_t *in, size_t len);
size_t cobs_decode(uint8_t *out, size_t out_max, const uint8_t *in, size_t len);
uint32_t crc32_zlib(const uint8_t *data, size_t len);
uint8_t* tlv_put(uint8_t *p, uint8_t type, const void *data, uint16_t len);
size_t usb_build_frame(uint8_t *buffer, size_t buffer_size, const uint8_t *payload, size_t payload_len);
uint8_t CDC_Transmit_Buffer(const uint8_t* Buf, uint16_t Len);

#ifdef __cplusplus
}
#endif

#endif /* __USB_COMM_H */
