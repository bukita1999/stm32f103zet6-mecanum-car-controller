#include "usb_comm.h"
#include <string.h>
#include "usbd_cdc_if.h"
#include "main.h"

// === COBS ===
size_t cobs_encode(uint8_t *out, size_t out_max, const uint8_t *in, size_t len){
  size_t rd=0, wr=1, code_idx=0; uint8_t code=1;
  if (!out_max) return 0;
  while (rd<len){
    if (in[rd]==0){
      out[code_idx]=code; code=1; code_idx=wr++;
      if (wr>out_max) return 0;
      rd++;
    }else{
      if (wr>=out_max) return 0;
      out[wr++]=in[rd++]; code++;
      if (code==0xFF){
        out[code_idx]=code; code=1; code_idx=wr++;
        if (wr>out_max) return 0;
      }
    }
  }
  out[code_idx]=code; return wr;
}
size_t cobs_decode(uint8_t *out, size_t out_max, const uint8_t *in, size_t len){
  size_t rd=0, wr=0;
  while (rd<len){
    uint8_t code=in[rd++];
    if (code==0 || rd+code-1>len) return 0;
    for (uint8_t i=1; i<code; i++){
      if (wr>=out_max) return 0;
      out[wr++]=in[rd++];
    }
    if (code!=0xFF && rd<len){
      if (wr>=out_max) return 0;
      out[wr++]=0;
    }
  }
  return wr;
}

// === CRC32 (zlib/IEEE) ===
uint32_t crc32_zlib(const uint8_t *data, size_t len){
  uint32_t crc=0xFFFFFFFFu;
  for (size_t i=0;i<len;i++){
    crc^=data[i];
    for (int j=0;j<8;j++) crc = (crc>>1) ^ (0xEDB88320u & (-(int)(crc & 1)));
  }
  return ~crc;
}

// === TLV pack ===
uint8_t* tlv_put(uint8_t *p, uint8_t type, const void *data, uint16_t len){
  *p++ = type;
  *(uint16_t*)p = len;
  p += 2;
  memcpy(p, data, len);
  return p + len;
}

// ==== 外部对象（你的工程已有）====
extern SystemState_t systemState;
extern osMutexId_t   motorDataMutexHandle;

// === 发送（raw 是一个或多个TLV拼起来）===
void USB_SendRawTlvFrame(const uint8_t *raw, uint16_t raw_len){
  uint8_t tmp[USB_FRAME_MAX+4], enc[USB_ENC_MAX];
  if (raw_len > USB_FRAME_MAX) return;
  memcpy(tmp, raw, raw_len);
  uint32_t crc = crc32_zlib(raw, raw_len);
  memcpy(tmp + raw_len, &crc, 4);
  size_t n = cobs_encode(enc, sizeof(enc), tmp, raw_len + 4);
  if (!n) return;
  enc[n++] = 0x00;                    // 帧尾
  (void)CDC_Transmit_FS(enc, n);      // 忙则丢帧：Telemetry 可容忍
}

// === 打包并发送四电机 Telemetry ===
void USB_SendTelemetry(void){
  Telemetry_t t; uint8_t raw[128], *p = raw;

  /* === 新增：单位声明 TLV === */
  UsbSpeedUnit_t unit = { .code = SPEED_UNIT_CODE_ENC_CPS };
  memcpy(unit.name, SPEED_UNIT_STR, sizeof("enc_cps"));  // 包含终止符也没问题
  p = tlv_put(p, TLV_SPEED_UNIT, &unit, sizeof(unit));

  if (osMutexAcquire(motorDataMutexHandle, 10) == osOK){
    for (int i=0;i<4;i++){
      t.m[i].tgt = systemState.motors[i].targetSpeed;  // CPS
      t.m[i].spd = systemState.motors[i].currentSpeed; // CPS
      t.m[i].pwm = systemState.motors[i].pwmPercent;
      t.m[i].err = systemState.motors[i].pidController.error;
    }
    osMutexRelease(motorDataMutexHandle);
  }else{ memset(&t, 0, sizeof(t)); }
  p = tlv_put(p, TLV_TELEMETRY, &t, sizeof(t));
  USB_SendRawTlvFrame(raw, (uint16_t)(p - raw));
}

// === 解析并执行业务 ===
void USB_HandleFrame(const uint8_t *payload, uint16_t len){
  const uint8_t *p = payload, *end = payload + len;
  while (p + 3 <= end){
    uint8_t  type = *p++; uint16_t L = *(uint16_t*)p; p += 2;
    if (p + L > end) break;                 // 长度防御

    switch(type){
      case 0x01: { // SetSpeeds: int16 v[4]
        if (L >= 8){
          int16_t v[4]; memcpy(v, p, 8);
          if (osMutexAcquire(motorDataMutexHandle, 10) == osOK){
            for (int i=0;i<4;i++) SetMotorSpeed(&systemState.motors[i], v[i]);
            osMutexRelease(motorDataMutexHandle);
          }
        }
      } break;

      case 0x02: { // SetPID: {u8 id; float kp,ki,kd}
        if (L >= 1 + 3*sizeof(float)){
          struct __attribute__((packed)){ uint8_t id; float kp,ki,kd; } s;
          memcpy(&s, p, sizeof(s));
          if (s.id < 4 && osMutexAcquire(motorDataMutexHandle, 10) == osOK){
            PIDController_t *pid = &systemState.motors[s.id].pidController;
            pid->Kp = s.kp; pid->Ki = s.ki; pid->Kd = s.kd;
            osMutexRelease(motorDataMutexHandle);
          }
        }
      } break;

      // 其他type留作扩展
      default: break;
    }
    p += L;
  }
}