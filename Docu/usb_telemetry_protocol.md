# USB Telemetry Binary Protocol

本文档定义了通过 USB CDC 发送的二进制遥测帧格式，取代此前的纯文本输出。协议设计目标是：

- 帧结构固定且易于解析，可快速定位同步错误
- 兼顾扩展性（版本号、预留字段）
- 通过 CRC32 校验保证数据完整性
- 每帧包含四个电机的实时状态

## 帧整体布局

```
+------------+-------------+--------------------+-------------+---------------+
| Frame Head | Frame Body  | CRC32 (little-end) | Frame Trail |
+------------+-------------+--------------------+-------------+---------------+
```

| 区域        | 说明                                                 |
| ----------- | ---------------------------------------------------- |
| Frame Head  | 固定 8 字节，包括同步字、版本号、帧长度、时间戳等     |
| Frame Body  | N 个电机记录，每条记录长度固定                       |
| CRC32       | 4 字节，覆盖 Frame Head 与 Frame Body                |
| Frame Trail | 固定 2 字节，帮助解析端确认帧结束                     |

所有整型字段均使用小端序（Least Significant Byte first）。

## Frame Head

| 偏移 | 字段              | 类型      | 说明                                                                 |
| ---- | ----------------- | --------- | -------------------------------------------------------------------- |
| 0    | `sync`            | `uint16`  | 帧同步字，固定 `0xAA55`                                             |
| 2    | `version`         | `uint8`   | 协议版本，目前为 `0x01`                                             |
| 3    | `reserved`        | `uint8`   | 预留，发送端填 0                                                   |
| 4    | `frame_length`    | `uint16`  | 帧总长度（含头、体、CRC、尾）                                       |
| 6    | `timestamp_ms`    | `uint32`  | 从设备上电到当前的毫秒计时 (`HAL_GetTick()`)                        |

## Frame Body

`motor_count` 固定为 4，每条 Motor 记录 7 字节，总长度 28 字节。若未来扩展，可在版本升级时允许动态改变条目数量。

| 偏移 | 字段              | 类型      | 说明                                                                 |
| ---- | ----------------- | --------- | -------------------------------------------------------------------- |
| 0    | `motor_id`        | `uint8`   | 电机序号，范围 1-4                                                   |
| 1    | `target_rpm`      | `int16`   | 目标转速（RPM）                                                      |
| 3    | `current_rpm`     | `int16`   | 实际转速（RPM）                                                      |
| 5    | `pwm_percent`     | `uint16`  | 当前 PWM 占空比（0-100，放大 1 倍，单位 %）                          |

## CRC 与帧尾

- `CRC32`：4 字节，使用 `crc32_zlib()` 计算，输入数据为 Frame Head + Frame Body。
- `Frame Trail`：`uint16`，固定 `0x55AA`，帮助解析端确认帧结束。

## C 结构体参考

```c
typedef struct __attribute__((packed)) {
    uint16_t sync;           // 0xAA55
    uint8_t  version;        // 0x01
    uint8_t  reserved;       // = 0
    uint16_t frame_length;   // sizeof(full frame)
    uint32_t timestamp_ms;   // HAL_GetTick()
} UsbFrameHeader_t;

typedef struct __attribute__((packed)) {
    uint8_t  motor_id;       // 1..4
    int16_t  target_rpm;
    int16_t  current_rpm;
    uint16_t pwm_percent;    // 0..100
} UsbMotorRecord_t;

typedef struct __attribute__((packed)) {
    UsbFrameHeader_t header;
    UsbMotorRecord_t motors[4];
    uint32_t crc32;          // over header+motors
    uint16_t trail;          // 0x55AA
} UsbTelemetryFrame_t;
```

## 示例

假设时间戳 123456 ms，Motor1 目标 100 RPM、实际 95 RPM、占空比 42%，其余电机数据类推。帧内容如下：

```
AA 55 01 00 2A 00 40 E2 01 00
01 64 00 5F 00 2A 00
02 64 00 60 00 28 00
03 64 00 61 00 29 00
04 64 00 62 00 2B 00
xx xx xx xx
55 AA
```

- `2A 00` 为帧总长度 42 字节
- `40 E2 01 00` 为 123456 ms
- `xx xx xx xx` 为 CRC32（小端）

接收端在校验 CRC 成功后，即可解析任意电机记录。
