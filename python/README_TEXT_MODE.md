# STM32 USB文本模式数据传输

## 📋 概述

这是STM32 USB数据传输的文本模式实现，采用与UART监控输出相同的格式进行USB CDC传输，简化了数据处理流程。

## 🔄 传输格式

### STM32发送格式
```
Motor1: Target:1000 Current:950 RPM, PWM:75%, Error:5.00
Motor2: Target:-500 Current:-480 RPM, PWM:35%, Error:-2.00
Motor3: Target:800 Current:820 RPM, PWM:65%, Error:-2.00
Motor4: Target:0 Current:5 RPM, PWM:0%, Error:-0.50
System: Init=1, PCA9685=0, MotorErr=0
```

### 数据字段说明

| 字段 | 说明 | 数据类型 | 范围 |
|------|------|----------|------|
| **Target** | 目标速度 | int16_t | -1310 ~ +1310 RPM |
| **Current** | 当前速度 | int16_t | -1310 ~ +1310 RPM |
| **PWM** | PWM百分比 | uint8_t | 0 ~ 100 % |
| **Error** | PID误差 | float | -50.0 ~ +50.0 |
| **Init** | 系统初始化状态 | bool | 0/1 |
| **PCA9685** | PCA9685错误状态 | bool | 0/1 |
| **MotorErr** | 电机错误状态 | bool | 0/1 |

## 🐍 Python接收处理

### 解析流程
1. **接收文本数据** - 直接从USB CDC读取文本
2. **行分割解析** - 按行分割并识别不同类型的数据
3. **正则表达式匹配** - 使用正则表达式解析电机状态行
4. **数据合并** - 将4个电机的数据合并为一条CSV记录
5. **CSV保存** - 保存到带时间戳的CSV文件

### CSV输出格式
```csv
batch_id,data_index,timestamp,motor0_target_speed,motor0_current_speed,motor0_pwm,motor0_error,motor1_target_speed,motor1_current_speed,motor1_pwm,motor1_error,motor2_target_speed,motor2_current_speed,motor2_pwm,motor2_error,motor3_target_speed,motor3_current_speed,motor3_pwm,motor3_error,receive_time
0,0,1640995200000,1000,950,75,5.0,-500,-480,35,-2.0,800,820,65,-2.0,0,5,0,-0.5,1640995200.123
```

## 🔧 技术特点

### 优势
- **简单直观**: 直接使用文本格式，无需复杂的编解码
- **易于调试**: 数据格式与UART输出完全一致，便于调试
- **兼容性好**: 可以直接在串口调试工具中查看数据
- **错误率低**: 文本格式不易受传输干扰影响

### 传输参数
- **发送频率**: 每100ms一次
- **数据量**: 约200-300字节/次
- **格式**: UTF-8文本
- **分隔符**: \r\n (回车换行)

## 🚀 使用方法

### 1. STM32端编译
```bash
# 重新编译并下载到STM32
# USB发送代码已修改为文本模式
```

### 2. Python端运行
```bash
cd python
python batch_data_receiver.py
```

### 3. 验证数据
- 查看控制台输出，确认接收到文本格式数据
- 检查生成的CSV文件，验证数据解析正确
- 对比UART输出和USB输出的一致性

## 🔍 调试和验证

### 串口调试工具验证
使用串口调试工具连接COM4，可以直接看到STM32发送的文本数据：
```
Motor1: Target:1000 Current:950 RPM, PWM:75%, Error:5.00
Motor2: Target:-500 Current:-480 RPM, PWM:35%, Error:-2.00
Motor3: Target:800 Current:820 RPM, PWM:65%, Error:-2.00
Motor4: Target:0 Current:5 RPM, PWM:0%, Error:-0.5
System: Init=1, PCA9685=0, MotorErr=0
```

### Python调试输出
```
接收到数据帧: 256字符
原始数据:
Motor1: Target:1000 Current:950 RPM, PWM:75%, Error:5.00
Motor2: Target:-500 Current:-480 RPM, PWM:35%, Error:-2.00
...

✓ 成功处理一批数据
```

## ⚡ 性能对比

| 模式 | 复杂度 | 数据大小 | 解析速度 | 错误率 | 调试难度 |
|------|--------|----------|----------|--------|----------|
| **原二进制模式** | 高 | ~3620字节 | 中等 | 低 | 高 |
| **新文本模式** | 低 | ~250字节 | 快 | 极低 | 低 |

## 🛠️ 故障排除

### 常见问题
1. **数据不完整**: 检查USB连接是否稳定
2. **解析失败**: 确认STM32发送的格式是否正确
3. **CSV字段错位**: 检查正则表达式匹配是否准确

### 调试建议
1. 先用串口调试工具验证STM32输出
2. 运行Python程序查看接收到的原始数据
3. 检查CSV文件的数据完整性

## 📝 版本信息

- **修改时间**: 2025年1月
- **修改内容**: 从二进制TLV模式改为文本模式
- **兼容性**: 保持与UART监控输出格式一致

---

*文本模式简化了数据处理流程，提高了系统的可靠性和可调试性。*
