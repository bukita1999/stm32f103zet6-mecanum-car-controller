/* Includes ------------------------------------------------------------------*/
#include "usb_comm.h"
#include "usbd_cdc_if.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define COBS_MAX_FRAME_SIZE  MAX_FRAME_SIZE
#define POLY_CRC32           0xEDB88320  /* zlib CRC32 polynomial */

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint32_t crc32_table[256];

/* Private function prototypes -----------------------------------------------*/
static void crc32_init_table(void);
static uint32_t crc32_compute(const uint8_t *data, size_t len);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 初始化CRC32查找表
 */
static void crc32_init_table(void)
{
    static uint8_t initialized = 0;

    if (initialized) return;

    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int j = 0; j < 8; j++) {
            if (c & 1) {
                c = POLY_CRC32 ^ (c >> 1);
            } else {
                c >>= 1;
            }
        }
        crc32_table[i] = c;
    }

    initialized = 1;
}

/**
 * @brief 计算CRC32校验码
 * @param data: 数据缓冲区
 * @param len: 数据长度
 * @return CRC32校验码
 */
static uint32_t crc32_compute(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFF;

    crc32_init_table();

    for (size_t i = 0; i < len; i++) {
        crc = crc32_table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    }

    return crc ^ 0xFFFFFFFF;
}

/* Exported functions ------------------------------------------------------- */

/**
 * @brief COBS编码函数
 * @param out: 输出缓冲区
 * @param out_max: 输出缓冲区最大长度
 * @param in: 输入数据
 * @param len: 输入数据长度
 * @return 编码后的长度，失败返回0
 */
size_t cobs_encode(uint8_t *out, size_t out_max, const uint8_t *in, size_t len)
{
    if (!out || !in || len == 0 || out_max < len + 2) {
        return 0;
    }

    size_t out_idx = 1;  /* 跳过第一个字节，用来记录下一个0的位置 */
    size_t code_idx = 0; /* 当前code的位置 */
    uint8_t code = 1;    /* 当前code值 */

    for (size_t i = 0; i < len; i++) {
        if (in[i] == 0) {
            out[code_idx] = code;  /* 写入code */
            code_idx = out_idx;    /* 更新code位置 */
            code = 1;             /* 重置code */
            out_idx++;            /* 移动到下一个位置 */
        } else {
            out[out_idx++] = in[i];
            code++;

            if (code == 0xFF) {  /* code达到最大值 */
                out[code_idx] = code;
                code_idx = out_idx;
                code = 1;
                out_idx++;
            }
        }

        if (out_idx >= out_max) {
            return 0;  /* 缓冲区溢出 */
        }
    }

    out[code_idx] = code;  /* 写入最后的code */
    out[out_idx++] = 0;    /* 添加帧尾 */

    return out_idx;
}

/**
 * @brief COBS解码函数
 * @param out: 输出缓冲区
 * @param out_max: 输出缓冲区最大长度
 * @param in: 输入数据
 * @param len: 输入数据长度
 * @return 解码后的长度，失败返回0
 */
size_t cobs_decode(uint8_t *out, size_t out_max, const uint8_t *in, size_t len)
{
    if (!out || !in || len == 0 || out_max == 0) {
        return 0;
    }

    size_t out_idx = 0;

    for (size_t i = 0; i < len;) {
        uint8_t code = in[i++];
        uint8_t j;

        for (j = 1; j < code && i < len; j++) {
            if (out_idx >= out_max) {
                return 0;  /* 缓冲区溢出 */
            }
            out[out_idx++] = in[i++];
        }

        if (j < code) {
            return 0;  /* 输入数据不完整 */
        }

        if (code < 0xFF && i < len) {
            if (out_idx >= out_max) {
                return 0;  /* 缓冲区溢出 */
            }
            out[out_idx++] = 0;
        }
    }

    return out_idx;
}

/**
 * @brief 计算zlib CRC32校验码
 * @param data: 数据缓冲区
 * @param len: 数据长度
 * @return CRC32校验码
 */
uint32_t crc32_zlib(const uint8_t *data, size_t len)
{
    return crc32_compute(data, len);
}

/**
 * @brief TLV数据打包函数
 * @param p: 当前缓冲区位置指针
 * @param type: 数据类型
 * @param data: 数据内容
 * @param len: 数据长度
 * @return 下一个可用位置指针
 */
uint8_t* tlv_put(uint8_t *p, uint8_t type, const void *data, uint16_t len)
{
    if (!p || !data) {
        return p;
    }

    /* 写入Type */
    *p++ = type;

    /* 写入Length (小端序) */
    *p++ = (uint8_t)(len & 0xFF);
    *p++ = (uint8_t)((len >> 8) & 0xFF);

    /* 写入Value */
    memcpy(p, data, len);
    p += len;

    return p;
}

/**
 * @brief 构建完整的USB帧（TLV数据 + CRC32 + COBS编码）
 * @param buffer: 输出缓冲区
 * @param buffer_size: 缓冲区大小
 * @param payload: TLV负载数据
 * @param payload_len: 负载长度
 * @return 完整帧长度，失败返回0
 */
size_t usb_build_frame(uint8_t *buffer, size_t buffer_size, const uint8_t *payload, size_t payload_len)
{
    if (!buffer || !payload || payload_len == 0 || buffer_size < payload_len + CRC32_SIZE + 2) {
        return 0;
    }

    uint8_t *temp_buffer = buffer + buffer_size - payload_len - CRC32_SIZE;

    /* 复制TLV数据 */
    memcpy(temp_buffer, payload, payload_len);

    /* 计算并添加CRC32校验码 */
    uint32_t crc = crc32_zlib(temp_buffer, payload_len);
    memcpy(temp_buffer + payload_len, &crc, CRC32_SIZE);

    /* COBS编码 */
    size_t encoded_len = cobs_encode(buffer, buffer_size, temp_buffer, payload_len + CRC32_SIZE);

    return encoded_len;
}

/**
 * @brief 发送缓冲区数据到USB CDC（带重试机制）
 * @param Buf: 数据缓冲区
 * @param Len: 数据长度
 * @return 发送结果 (USBD_OK/USBD_BUSY/USBD_FAIL)
 */
uint8_t CDC_Transmit_Buffer(const uint8_t* Buf, uint16_t Len)
{
    uint8_t result;
    uint8_t retry_count = 0;
    const uint8_t max_retries = 3;

    do {
        result = CDC_Transmit_FS((uint8_t*)Buf, Len);
        if (result == USBD_OK) {
            break;
        } else if (result == USBD_BUSY) {
            /* 等待一小段时间后重试 */
            HAL_Delay(1);
            retry_count++;
        } else {
            /* 发送失败 */
            break;
        }
    } while (retry_count < max_retries);

    return result;
}
