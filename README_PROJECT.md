# STM32 Robot Control System

这是一个基于STM32单片机的机器人控制系统，包含上位机Python程序和Docker容器化部署方案。

## 项目结构

```
stm32-small-surface/
├── Core/                          # STM32 HAL库核心代码
│   ├── Inc/                       # 头文件
│   │   ├── communication.h        # 通信模块
│   │   ├── usb_handler.h          # USB处理器
│   │   ├── usart.h               # UART通信
│   │   └── ...
│   └── Src/                       # 源文件
│       ├── communication.c       # 通信实现
│       ├── usb_handler.c         # USB处理实现
│       └── ...
├── python/                        # 上位机Python程序
│   ├── batch_data_receiver.py     # 批量数据接收器
│   ├── keyboard_robot_control.py  # 键盘控制程序
│   ├── csv_analyzer.py           # 数据分析程序
│   ├── robot_config.yaml         # 机器人配置文件
│   ├── config_loader.py          # 配置加载器
│   ├── requirements.txt          # Python依赖
│   ├── start.sh                  # 启动脚本
│   └── README_DOCKER.md          # Docker使用指南
├── Dockerfile                     # Docker构建文件
├── docker-compose.yml            # Docker Compose配置
├── run_docker.sh                 # Docker运行脚本
├── .dockerignore                 # Docker忽略文件
└── README_PROJECT.md             # 项目说明文档
```

## STM32固件功能

### 通信模块
- **USB-CDC通信**: 通过USB虚拟串口与上位机通信
- **UART通信**: 支持串口命令控制
- **协议支持**: COBS编码、CRC32校验、TLV数据格式

### 控制功能
- **电机控制**: 支持4个电机独立控制
- **PID控制器**: 速度闭环控制
- **运动监控**: 实时监控电机状态和系统状态

### 命令协议
- `$SPD,motor0,motor1,motor2,motor3#` - 速度控制
- `$PID,motorId,Kp,Ki,Kd#` - PID参数设置

## Python上位机程序

### 核心程序
1. **batch_data_receiver.py** - 数据接收和存储
2. **keyboard_robot_control.py** - 键盘控制界面
3. **csv_analyzer.py** - 数据分析和可视化

### 配置管理
- **robot_config.yaml** - YAML配置文件定义运动参数
- **config_loader.py** - 配置加载和验证

### 运动控制
通过YAML配置文件定义：
- 前进、后退、左转、右转、停止命令
- 电机速度参数
- 串口通信设置
- PID控制参数

## Docker容器化部署

### 快速开始
```bash
# 构建并启动
docker-compose up --build

# 使用运行脚本
./run_docker.sh build
./run_docker.sh keyboard
```

### 串口映射
- **CH340串口**: `/dev/ttyUSB0`
- **虚拟串口**: `/dev/ttyACM0`
- 支持自动设备检测和映射

### 程序运行
```bash
# 键盘控制
./run_docker.sh keyboard

# 数据接收
./run_docker.sh receiver

# 数据分析
./run_docker.sh analyzer
```

## 使用方法

### 1. 硬件连接
1. 将STM32开发板通过USB连接到电脑
2. 确保CH340驱动已安装（如果使用CH340串口）

### 2. 固件烧录
1. 使用STM32CubeIDE编译项目
2. 通过ST-Link或USB烧录固件

### 3. 上位机运行
```bash
# 方式1: 直接运行Python程序
cd python
python3 keyboard_robot_control.py

# 方式2: 使用Docker
./run_docker.sh keyboard
```

### 4. 控制说明
- **W** - 前进
- **S** - 后退
- **A** - 左转
- **D** - 右转
- **空格** - 停止
- **Q** - 退出

## 配置自定义

### 修改运动参数
编辑 `python/robot_config.yaml`：

```yaml
movement_commands:
  forward:
    speeds: [2000, -2000, -2000, 2000]  # 自定义速度
    command: "$SPD,2000,-2000,-2000,2000#"
```

### 串口配置
```yaml
serial:
  port: "COM10"        # Windows
  # port: "/dev/ttyUSB0"  # Linux
  baudrate: 115200
```

## 技术特点

### STM32固件
- **实时性**: FreeRTOS多任务调度
- **通信**: 双通信接口（USB+UART）
- **控制**: 四轮独立PID控制
- **监控**: 实时状态监控和数据上报

### Python上位机
- **模块化**: 配置文件驱动的架构
- **跨平台**: 支持Windows、Linux、macOS
- **可视化**: matplotlib数据分析
- **容器化**: Docker一键部署

### Docker支持
- **多架构**: Python 3.13环境
- **设备映射**: 自动串口设备检测
- **权限管理**: 非root用户运行
- **数据持久化**: CSV文件自动保存

## 开发和调试

### 串口调试
```bash
# 监听串口数据
./run_docker.sh receiver --port /dev/ttyUSB0

# 发送测试命令
echo '$SPD,1000,0,0,0#' > /dev/ttyUSB0
```

### 数据分析
```bash
# 分析CSV数据
./run_docker.sh analyzer

# 自定义分析参数
python3 csv_analyzer.py --input data/robot_data.csv
```

### 配置文件验证
```bash
# 验证配置
./run_docker.sh config
```

## 故障排除

### 常见问题
1. **串口权限**: 确保用户有串口访问权限
2. **设备映射**: 检查Docker设备映射配置
3. **显示问题**: 配置X11显示服务器（Linux）
4. **依赖问题**: 确保所有Python依赖已安装

### 日志查看
```bash
# Docker日志
docker-compose logs -f

# 应用程序日志
./run_docker.sh logs
```

## 扩展开发

### 添加新功能
1. 在 `robot_config.yaml` 中定义新命令
2. 更新 `config_loader.py` 支持新配置
3. 修改相应Python程序实现功能

### 自定义控制算法
1. 修改STM32的PID控制器参数
2. 实现新的运动控制算法
3. 添加传感器数据处理

## 许可证

本项目采用MIT许可证。

## 贡献

欢迎提交Issue和Pull Request来改进这个项目。

## 联系方式

如有问题，请通过GitHub Issues联系。

