# STM32 Robot Control - Docker 使用指南

本指南介绍如何使用Docker和docker-compose来运行STM32机器人控制系统。

## 前置要求

1. **Docker和Docker Compose**: 确保已安装Docker和Docker Compose
2. **串口设备**: 确保CH340串口设备已连接到主机
3. **X11显示服务器**: 如果需要运行GUI程序（matplotlib），需要X11支持

## 快速开始

### 1. 构建和启动容器

```bash
# 构建镜像并启动服务
docker-compose up --build

# 或者后台运行
docker-compose up -d --build
```

### 2. 查看运行中的容器

```bash
docker-compose ps
```

### 3. 查看容器日志

```bash
docker-compose logs -f stm32-robot-control
```

## 使用方法

### 键盘控制程序

```bash
# 运行键盘控制程序
docker-compose run --rm stm32-robot-control ./start.sh keyboard

# 或者直接运行
docker-compose run --rm stm32-robot-control python3 keyboard_robot_control.py
```

### 数据接收程序

```bash
# 运行数据接收程序
docker-compose run --rm stm32-robot-control ./start.sh receiver

# 指定串口端口
docker-compose run --rm stm32-robot-control python3 batch_data_receiver.py --port /dev/ttyUSB0
```

### 数据分析程序

```bash
# 运行数据分析程序
docker-compose run --rm stm32-robot-control ./start.sh analyzer
```

### 配置文件测试

```bash
# 测试配置文件加载
docker-compose run --rm stm32-robot-control ./start.sh config
```

## 串口设备配置

### Linux主机

在Linux系统上，串口设备通常映射为：
- `/dev/ttyUSB0` - CH340 USB转串口
- `/dev/ttyACM0` - STM32虚拟串口

### Windows主机

在Windows上，需要使用特殊的设备映射语法：

```yaml
devices:
  - "//./COM3:/dev/ttyUSB0"  # 将Windows的COM3映射到容器的/dev/ttyUSB0
```

### macOS主机

在macOS上，需要允许Docker访问串口设备：

```bash
# 允许Docker访问所有USB设备
docker run --privileged -v /dev:/dev ...
```

## 环境变量

可以通过环境变量自定义配置：

```bash
# 设置显示环境变量（用于GUI程序）
export DISPLAY=:0

# 运行容器
docker-compose run --rm stm32-robot-control ./start.sh analyzer
```

## 数据持久化

- CSV文件保存在 `./python/data/` 目录中
- 配置文件可以通过挂载进行热更新
- 日志文件会保存在容器内部

## 故障排除

### 串口权限问题

```bash
# 检查串口设备权限
ls -la /dev/ttyUSB*

# 添加用户到dialout组
sudo usermod -a -G dialout $USER

# 重新登录或重启Docker服务
```

### X11显示问题

```bash
# 允许Docker连接到X11
xhost +local:docker

# 或者使用xauth
touch ~/.Xauthority
xauth add $(xauth list | grep $(hostname) | head -1)
```

### 设备映射问题

```bash
# 检查Docker可以访问的设备
docker run --rm --device /dev/ttyUSB0 ubuntu ls /dev/ttyUSB*

# 查看设备详细信息
udevadm info -a -n /dev/ttyUSB0
```

## 监控模式

启用串口监控服务：

```bash
# 启动监控服务
docker-compose --profile monitor up -d

# 查看监控日志
docker-compose logs -f serial-monitor
```

## 自定义配置

### 修改串口映射

编辑 `docker-compose.yml` 中的 `devices` 部分：

```yaml
devices:
  - "/dev/ttyUSB0:/dev/ttyUSB0"  # CH340串口
  - "/dev/ttyACM0:/dev/ttyACM0"  # 虚拟串口
```

### 修改配置文件

可以通过挂载配置文件进行热更新：

```yaml
volumes:
  - ./python/robot_config.yaml:/app/robot_config.yaml:ro
```

## 停止和清理

```bash
# 停止服务
docker-compose down

# 停止服务并删除卷
docker-compose down -v

# 清理所有相关镜像
docker-compose down --rmi all
```

## 高级用法

### 多容器部署

可以同时运行多个服务：

```bash
# 启动键盘控制和数据接收
docker-compose up keyboard-service receiver-service
```

### 自定义Dockerfile

如果需要自定义构建，可以修改 `Dockerfile`：

```dockerfile
FROM python:3.13-slim

# 添加自定义依赖
RUN apt-get update && apt-get install -y \
    your-custom-package \
    && rm -rf /var/lib/apt/lists/*
```

## 开发模式

在开发模式下挂载源代码：

```yaml
volumes:
  - ./python:/app:ro
  - ./python/data:/app/data
```

这样可以实时修改代码而无需重新构建镜像。

## 支持

如果遇到问题，请检查：

1. Docker和Docker Compose版本
2. 串口设备权限
3. X11显示配置
4. 设备映射配置

可以通过以下命令获取更多信息：

```bash
docker-compose config  # 验证配置
docker-compose logs    # 查看日志
docker stats          # 查看资源使用
```

