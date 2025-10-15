# 自定义轮速控制功能

## 功能概述

新增的自定义轮速控制功能允许远程设定四个轮子的精确速度，实现复杂的机器人运动控制。

## 网络协议

### 新增消息类型

#### `custom_speed` 消息
```json
{
  "type": "custom_speed",
  "speeds": [motor0_speed, motor1_speed, motor2_speed, motor3_speed],
  "duration": 持续时间秒数,
  "timestamp": 时间戳
}
```

- `speeds`: 必须是包含4个整数的数组，每个电机的速度（RPM）
- `duration`: 可选，运动持续时间（秒），0表示持续运动直到新命令
- 速度范围: -3000 ~ 3000 RPM

## 服务端功能

### python-rasp/robot_control_server.py

- **消息处理**: 新增 `custom_speed` 消息类型处理
- **参数验证**: 自动验证速度参数的合法性
- **串口命令生成**: 自动生成 `$SPD,speed0,speed1,speed2,speed3#` 格式命令
- **定时停止**: 支持设定持续时间，自动停止功能
- **状态广播**: 实时广播当前运动状态给所有客户端

### 运动控制示例

```python
# 前进
speeds = [2000, -2000, -2000, 2000]

# 后退
speeds = [-2000, 2000, 2000, -2000]

# 左转
speeds = [1500, -1500, -2500, 2500]

# 右转
speeds = [-1500, 1500, 2500, -2500]

# 原地顺时针旋转
speeds = [1000, 1000, 1000, 1000]

# 停止
speeds = [0, 0, 0, 0]
```

## 客户端功能

### python/network_control/robot_control_client.py

- **GUI界面**: 新增自定义轮速控制面板
- **API接口**: 新增 `send_custom_speed(speeds, duration=0)` 方法
- **参数验证**: 客户端本地验证参数有效性
- **错误处理**: 完善的错误提示和异常处理

### GUI 使用方法

1. 启动客户端: `python robot_control_client.py`
2. 在"Custom Speed Control"区域输入四个电机的速度
3. 可选输入持续时间（秒）
4. 点击"Send Custom Speed"发送命令
5. 点击"Stop"立即停止运动

### 编程接口

```python
from robot_control_client import RobotControlClient

client = RobotControlClient()
client.connect_to_server()

# 发送持续运动命令
client.send_custom_speed([2000, -2000, -2000, 2000])

# 发送2秒后自动停止的命令
client.send_custom_speed([1000, 1000, 1000, 1000], duration=2)

# 停止运动
client.send_custom_speed([0, 0, 0, 0])
```

## 测试脚本

运行测试脚本验证功能：

```bash
cd python/network_control
python test_custom_speed.py
```

## 麦轮车运动原理

四个轮子的速度对应不同的运动：

- **前进**: [2000, -2000, -2000, 2000] - 前后轮反向旋转
- **后退**: [-2000, 2000, 2000, -2000] - 与前进相反
- **左转**: [1500, -1500, -2500, 2500] - 左前右后速度不同
- **右转**: [-1500, 1500, 2500, -2500] - 与左转相反
- **平移**: 根据需要调整四个轮子的速度组合

## 注意事项

1. **速度范围**: 每个轮子的速度必须在 -3000 ~ 3000 RPM 范围内
2. **数据类型**: speeds 必须是包含4个数字的列表
3. **持续时间**: duration=0 表示持续运动，>0 表示定时自动停止
4. **线程安全**: 服务端使用异步处理，保证多客户端并发安全
5. **错误处理**: 完整的参数验证和错误日志记录

## 故障排除

- **连接失败**: 检查树莓派服务器是否运行在正确端口
- **参数错误**: 确认速度值在有效范围内
- **无响应**: 检查串口连接和STM32设备状态
- **自动停止不工作**: 确认duration参数设置正确
