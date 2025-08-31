# STM32小车USB通讯测试程序

这是一个基于异步Python框架的STM32小车USB通讯测试程序，用于测试与小车的通讯功能。

## 功能特性

- 🔄 **异步通讯**: 基于 asyncio 和 pyserial-asyncio 的异步USB通讯
- 📊 **数据采集**: 自动采集遥测数据并保存为CSV格式
- 🎮 **电机控制**: 支持设置电机速度和PID参数
- 📈 **数据分析**: 自动生成数据摘要和对比报告
- 🛡️ **协议支持**: 完整实现TLV协议、COBS编码和CRC32校验

## 安装依赖

```bash
pip install -r requirements.txt
```

## 文件结构

```
python/
├── requirements.txt           # 依赖包列表
├── tlv_protocol.py           # TLV协议实现
├── cobs_crc.py              # COBS编码和CRC32校验
├── usb_communication.py     # USB异步通讯类
├── csv_exporter.py          # CSV数据导出器
├── test_stm32_communication.py  # 主测试程序
└── README.md                # 说明文档
```

## 使用方法

### 1. 快速开始

```bash
cd python
python test_stm32_communication.py
```

程序会提示输入串口号，默认为 COM3。

### 2. 测试内容

程序包含两个主要测试：

#### 测试1: 基线遥测数据采集
- 连接STM32设备后，采集5帧连续的遥测数据
- 数据包含四个电机的目标速度、当前速度、PWM值和PID误差
- 自动保存为 `baseline_telemetry_YYYYMMDD_HHMMSS.csv`

#### 测试2: 电机速度控制测试
- 设置四个电机的目标速度为: [239, 342, -321, -395] RPM
- 采集10帧遥测数据观察速度跟踪效果
- 自动保存为 `speed_control_test_YYYYMMDD_HHMMSS.csv`

### 3. 输出文件

测试完成后，在 `test_results/` 目录下会生成：

- `baseline_telemetry_*.csv` - 基线遥测数据
- `baseline_telemetry_*_summary.txt` - 基线数据摘要
- `speed_control_test_*.csv` - 速度控制测试数据  
- `speed_control_test_*_summary.txt` - 控制测试摘要
- `comparison_report_*.txt` - 对比分析报告

## CSV数据格式

每个CSV文件包含以下列：

| 列名 | 描述 |
|------|------|
| frame_index | 帧索引 |
| frame_timestamp | 帧时间戳 |
| motor_id | 电机ID (0-3) |
| motor_timestamp | 电机数据时间戳 |
| target_speed_rpm | 目标速度 (RPM) |
| current_speed_rpm | 当前速度 (RPM) |
| pwm_percent | PWM百分比 |
| pid_error | PID误差 |
| speed_unit | 速度单位 |

## 高级用法

### 自定义测试

```python
import asyncio
from usb_communication import USBCommunication, USBConfig
from csv_exporter import CSVExporter

async def custom_test():
    # 创建通讯对象
    config = USBConfig(port='COM3')
    comm = USBCommunication(config)
    
    # 连接设备
    await comm.connect()
    
    # 设置电机速度
    await comm.set_motor_speeds([100, 200, -150, -250])
    
    # 等待遥测数据
    frames = await comm.wait_for_telemetry(count=5)
    
    # 导出数据
    exporter = CSVExporter()
    exporter.export_telemetry_frames(frames, "my_test.csv", "自定义测试")
    
    await comm.disconnect()

# 运行测试
asyncio.run(custom_test())
```

### 设置PID参数

```python
# 设置电机0的PID参数
await comm.set_motor_pid(motor_id=0, kp=0.3, ki=0.05, kd=0.01)
```

### 实时数据监控

```python
def telemetry_handler(frame):
    print(f"收到遥测数据: {len(frame.motors)} 个电机")
    for i, motor in enumerate(frame.motors):
        print(f"电机{i}: {motor.current_speed} RPM")

comm.set_telemetry_callback(telemetry_handler)
```

## 故障排除

### 常见问题

1. **连接失败**
   - 检查串口号是否正确
   - 确认STM32设备已连接并正常工作
   - 检查串口是否被其他程序占用

2. **数据接收超时**
   - 确认STM32程序正在运行
   - 检查波特率设置 (默认115200)
   - 增加超时时间

3. **CRC校验失败**
   - 检查STM32和Python的CRC算法是否一致
   - 可能存在数据传输错误

### 调试模式

设置日志级别为DEBUG可以看到详细的通讯信息：

```python
import logging
logging.basicConfig(level=logging.DEBUG)
```

## 协议说明

程序完整实现了与STM32小车的通讯协议：

- **帧格式**: COBS编码 + CRC32校验 + 0x00帧尾
- **数据格式**: TLV (Type-Length-Value) 协议
- **支持的TLV类型**:
  - 0x10: 遥测数据
  - 0x11: 速度单位声明
  - 0x01: 设置电机速度
  - 0x02: 设置PID参数

详细的协议说明请参考 `../Docu/USB_COMMUNICATION_ANALYSIS.md`。

## 版本信息

- 版本: v1.0
- Python要求: 3.7+
- 测试平台: Windows 10/11

---

如有问题，请检查串口连接和设备状态。
