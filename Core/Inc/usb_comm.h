#pragma once
#include <stdint.h>
#include <stddef.h>
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---- 协议与缓冲参数（可按需调整）----
#define USB_FRAME_MAX   256   // 一帧TLV原始总长度上限（不含CRC）
#define USB_ENC_MAX     (USB_FRAME_MAX + USB_FRAME_MAX/254 + 2) // COBS开销

// 上报结构（小端；__packed 避免填充）
typedef struct __attribute__((packed)) { int16_t tgt, spd; uint16_t pwm; float err; } MotorTelem_t;
typedef struct __attribute__((packed)) { MotorTelem_t m[4]; } Telemetry_t;

#define TLV_TELEMETRY        0x10
#define TLV_SPEED_UNIT       0x11  /* payload: {u8 code; char name[8]} */
#define SPEED_UNIT_CODE_ENC_CPS  1

typedef struct __attribute__((packed)){
  uint8_t code;      /* 1 = enc_cps */
  char    name[8];   /* "enc_cps"   */
} UsbSpeedUnit_t;

// --- TLV 辅助 ---
uint8_t* tlv_put(uint8_t *p, uint8_t type, const void *data, uint16_t len);

// --- 业务接口（任务上下文里调用）---
void USB_SendTelemetry(void);                        // 上报一次四电机状态
void USB_SendRawTlvFrame(const uint8_t *raw, uint16_t raw_len); // 发送任意TLV集合
void USB_HandleFrame(const uint8_t *payload, uint16_t len);     // 解析/执行业务

// --- COBS / CRC32 ---
size_t   cobs_encode(uint8_t *out, size_t out_max, const uint8_t *in, size_t len);
size_t   cobs_decode(uint8_t *out, size_t out_max, const uint8_t *in, size_t len);
uint32_t crc32_zlib(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif