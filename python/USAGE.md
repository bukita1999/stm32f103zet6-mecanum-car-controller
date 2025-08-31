# STM32小车USB通讯测试程序 - 使用说明

## 📋 项目概述

这是一个基于Python异步框架开发的STM32小车USB通讯测试工具，实现了完整的TLV协议通讯、COBS编码、CRC32校验等功能。可以与STM32小车进行实时通讯，控制电机运行并采集遥测数据。

## 🚀 快速开始

### 1. 环境准备

确保您的系统已安装以下软件：
- Python 3.11+ 
- uv 包管理器
- STM32小车硬件设备

### 2. 安装项目

```bash
# 克隆或下载项目到本地
cd python

# 使用uv创建虚拟环境并安装依赖
uv sync

# 激活虚拟环境 (Windows)
.venv\Scripts\activate

# 激活虚拟环境 (Linux/Mac)
source .venv/bin/activate
```

### 3. 连接硬件

1. 通过USB线连接STM32小车到电脑
2. 确认设备管理器中出现COM端口（如COM3）
3. 确保STM32程序正常运行

### 4. 运行测试

```bash
# 方式1: 使用uv直接运行
uv run python test_stm32_communication.py

# 方式2: 激活环境后运行
python test_stm32_communication.py

# 方式3: 使用项目脚本（如果配置了）
uv run stm32-test
```

## 🔧 详细配置

### 串口配置

程序启动时会提示输入串口号，默认为COM3。您也可以修改代码中的默认配置：

```python
# 在 test_stm32_communication.py 中修改
config = USBConfig(
    port='COM3',        # 串口号
    baudrate=115200,    # 波特率
    timeout=1.0         # 超时时间
)
```

### 测试参数调整

可以在代码中调整测试参数：

```python
# 修改电机速度设置
target_speeds = [239, 342, -321, -395]  # 四个电机的目标速度(RPM)

# 修改数据采集数量
baseline_frames = await comm.wait_for_telemetry(count=5)    # 基线数据帧数
control_frames = await comm.wait_for_telemetry(count=10)   # 控制测试帧数
```

## 📊 测试项目详解

### 测试1: 基线遥测数据采集

**目标**: 采集设备在默认状态下的运行数据

**流程**:
1. 连接USB设备
2. 等待系统稳定（2秒）
3. 连续采集5帧遥测数据
4. 保存为 `baseline_telemetry_YYYYMMDD_HHMMSS.csv`

**数据内容**: 四个电机的目标速度、当前速度、PWM值、PID误差

### 测试2: 电机速度控制测试

**目标**: 验证电机速度控制功能和响应特性

**流程**:
1. 发送速度设置命令：[239, 342, -321, -395] RPM
2. 等待系统响应（3秒）
3. 连续采集10帧遥测数据
4. 保存为 `speed_control_test_YYYYMMDD_HHMMSS.csv`

**分析内容**: 速度跟踪精度、系统响应时间、稳态误差等

## 📈 数据分析

### CSV文件结构

每个CSV文件包含以下列：

| 列名 | 类型 | 描述 | 单位 |
|------|------|------|------|
| frame_index | int | 数据帧序号 | - |
| frame_timestamp | datetime | 帧接收时间戳 | ISO格式 |
| motor_id | int | 电机编号 | 0-3 |
| motor_timestamp | datetime | 电机数据时间戳 | ISO格式 |
| target_speed_rpm | int | 目标转速 | RPM |
| current_speed_rpm | int | 当前转速 | RPM |
| pwm_percent | int | PWM占空比 | % |
| pid_error | float | PID控制误差 | - |
| speed_unit | str | 速度单位 | rpm |

### 自动生成的报告

程序会自动生成以下分析报告：

1. **数据摘要报告** (`*_summary.txt`)
   - 数据统计信息
   - 各电机性能指标
   - 运行时间分析

2. **对比分析报告** (`comparison_report_*.txt`)
   - 基线与测试数据对比
   - 速度跟踪精度分析
   - 性能变化评估

## 🔬 高级功能

### 自定义测试脚本

```python
import asyncio
from usb_communication import USBCommunication, USBConfig
from csv_exporter import CSVExporter

async def custom_motor_test():
    """自定义电机测试"""
    # 创建通讯实例
    config = USBConfig(port='COM3')
    comm = USBCommunication(config)
    
    try:
        # 连接设备
        await comm.connect()
        
        # 设置特定速度
        await comm.set_motor_speeds([500, -300, 0, 200])
        
        # 等待并采集数据
        frames = await comm.wait_for_telemetry(count=20, timeout=30)
        
        # 导出数据
        exporter = CSVExporter()
        exporter.export_telemetry_frames(
            frames, 
            "custom_test.csv",
            "自定义电机速度测试"
        )
        
    finally:
        await comm.disconnect()

# 运行自定义测试
asyncio.run(custom_motor_test())
```

### PID参数调整

```python
# 设置电机0的PID参数
await comm.set_motor_pid(
    motor_id=0,
    kp=0.3,    # 比例系数
    ki=0.05,   # 积分系数  
    kd=0.01    # 微分系数
)
```

### 实时数据监控

```python
def telemetry_callback(frame):
    """实时数据处理回调"""
    print(f"时间: {frame.timestamp}")
    for i, motor in enumerate(frame.motors):
        print(f"电机{i}: 目标={motor.target_speed}, "
              f"当前={motor.current_speed}, "
              f"PWM={motor.pwm_percent}%")

# 设置回调函数
comm.set_telemetry_callback(telemetry_callback)
```

## 🐛 故障排除

### 常见问题及解决方案

#### 1. 连接失败

**症状**: 程序报告"USB连接失败"

**可能原因**:
- 串口号错误
- 设备未连接或驱动问题
- 串口被其他程序占用

**解决方案**:
```bash
# 检查可用串口
python -c "import serial.tools.list_ports; print([port.device for port in serial.tools.list_ports.comports()])"

# 确认设备管理器中的COM端口号
# 关闭其他可能占用串口的程序（如串口调试工具）
```

#### 2. 数据接收超时

**症状**: 程序等待遥测数据超时

**可能原因**:
- STM32程序未运行或异常
- 通讯协议不匹配
- 波特率设置错误

**解决方案**:
```python
# 增加超时时间
frames = await comm.wait_for_telemetry(count=5, timeout=30.0)

# 检查波特率设置
config = USBConfig(port='COM3', baudrate=115200)
```

#### 3. CRC校验失败

**症状**: 控制台显示"CRC32校验失败"

**可能原因**:
- 数据传输错误
- 协议版本不匹配
- 电磁干扰

**解决方案**:
- 检查USB连接稳定性
- 更换USB线或端口
- 确认STM32和Python的CRC算法一致

#### 4. 电机控制无响应

**症状**: 发送速度命令后电机无反应

**可能原因**:
- 命令格式错误
- 电机硬件故障
- 安全限制触发

**解决方案**:
```python
# 检查速度范围（通常±1310 RPM）
await comm.set_motor_speeds([100, 200, -150, -250])

# 先发送较小的速度值进行测试
await comm.set_motor_speeds([50, 50, 50, 50])
```

### 调试模式

启用详细日志输出：

```python
import logging

# 设置日志级别
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
```

### 性能优化

```python
# 调整缓冲区大小
config = USBConfig(
    port='COM3',
    baudrate=115200,
    timeout=0.5,        # 减少超时时间
    read_timeout=0.05   # 减少读取超时
)
```

## 📝 开发指南

### 添加新功能

1. **新增TLV命令类型**:
```python
# 在 tlv_protocol.py 中添加
TLV_NEW_COMMAND = 0x03

@staticmethod
def pack_new_command(param1, param2):
    data = struct.pack('<ff', param1, param2)
    return TLVProtocol.pack_tlv(TLV_NEW_COMMAND, data)
```

2. **扩展数据分析**:
```python
# 在 csv_exporter.py 中添加新的分析函数
@staticmethod
def analyze_motor_efficiency(frames):
    # 自定义分析逻辑
    pass
```

### 代码风格

项目使用以下工具确保代码质量：

```bash
# 代码格式化
uv run black .

# 代码检查
uv run ruff check .

# 类型检查（可选）
uv run mypy .
```

### 测试

```bash
# 运行单元测试
uv run pytest

# 运行异步测试
uv run pytest -v tests/
```

## 📚 API参考

### USBCommunication 类

主要的USB通讯接口类。

#### 方法

- `connect() -> bool`: 连接USB设备
- `disconnect()`: 断开连接
- `set_motor_speeds(speeds: List[int])`: 设置电机速度
- `set_motor_pid(motor_id: int, kp: float, ki: float, kd: float)`: 设置PID参数
- `wait_for_telemetry(count: int, timeout: float) -> List[TelemetryFrame]`: 等待遥测数据
- `set_telemetry_callback(callback: Callable)`: 设置数据回调

### TLVProtocol 类

TLV协议处理类。

#### 方法

- `pack_set_speeds(speeds: List[int]) -> bytes`: 打包速度设置命令
- `pack_set_pid(motor_id: int, kp: float, ki: float, kd: float) -> bytes`: 打包PID设置命令
- `unpack_telemetry(data: bytes) -> TelemetryFrame`: 解包遥测数据
- `parse_frame(tlv_data: bytes) -> Dict[str, Any]`: 解析TLV帧

### CSVExporter 类

数据导出和分析类。

#### 方法

- `export_telemetry_frames(frames: List[TelemetryFrame], filename: str, description: str)`: 导出遥测数据
- `create_comparison_report(baseline_file: str, test_file: str, output_file: str)`: 创建对比报告

## 📞 技术支持

如果遇到问题或需要技术支持，请：

1. 查阅本使用说明文档
2. 检查故障排除部分
3. 启用调试模式获取详细日志
4. 查看项目的README.md文件

## 📄 许可证

本项目采用MIT许可证，详见LICENSE文件。

---

**版本**: v1.0  
**更新时间**: 2025年1月  
**维护者**: STM32 Car Project Team
