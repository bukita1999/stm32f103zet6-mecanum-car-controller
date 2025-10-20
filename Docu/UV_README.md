# STM32 Robot Control - UV 构建指南

本指南介绍如何使用 [uv](https://github.com/astral-sh/uv) 来构建和运行STM32机器人控制系统。

## 什么是 uv？

uv 是 Python 包管理器，由 Astral 开发，旨在替代 pip 和 virtualenv。它的特点包括：

- 🚀 **快速**: 比 pip 快 10-100 倍
- 📦 **统一**: 单二进制文件，无需 Python 预装
- 🔒 **安全**: 内置依赖解析和锁定
- 🛠️ **多功能**: 支持项目管理、脚本运行等

## 前置要求

1. **安装 uv**:
   ```bash
   # 自动安装
   curl -LsSf https://astral.sh/uv/install.sh | sh

   # 或者手动下载
   # 访问 https://github.com/astral-sh/uv/releases
   ```

2. **Python 版本**: 3.8 或更高版本

3. **系统依赖** (Linux):
   ```bash
   # Ubuntu/Debian
   sudo apt-get update
   sudo apt-get install python3-tk

   # CentOS/RHEL
   sudo yum install tkinter

   # 串口权限 (树莓派)
   sudo usermod -a -G dialout $USER
   ```

## 快速开始

### 1. 克隆项目

```bash
cd /path/to/project
cd python  # 进入Python项目目录
```

### 2. 安装依赖

```bash
# 使用 pyproject.toml (推荐)
uv sync

# 或者使用 requirements.txt
uv pip install -r requirements.txt
```

### 3. 运行程序

```bash
# 使用脚本入口点
uv run robot-keyboard    # 键盘控制
uv run robot-receiver    # 数据接收
uv run robot-analyzer    # 数据分析
uv run robot-config      # 配置测试

# 网络控制
uv run robot-client      # 客户端 (x86电脑)
uv run robot-server      # 服务端 (树莓派)
```

## 项目结构

```
python/
├── pyproject.toml          # 项目配置
├── requirements.txt        # 依赖文件
├── __init__.py            # 包初始化
├── keyboard_robot_control.py    # 键盘控制程序
├── batch_data_receiver.py       # 数据接收程序
├── csv_analyzer.py             # 数据分析程序
├── config_loader.py           # 配置加载器
├── start.sh                   # 主启动脚本
├── UV_README.md              # 本文件
├── network_control/          # 网络控制模块
│   ├── __init__.py
│   ├── robot_control_client.py    # 网络客户端
│   ├── robot_control_server.py   # 网络服务端
│   ├── network_config.yaml       # 网络配置
│   ├── start_client.sh          # 客户端启动脚本
│   └── start_server.sh         # 服务端启动脚本
├── data/                      # 数据目录
└── logs/                      # 日志目录
```

## 详细使用指南

### 基本程序运行

#### 键盘控制程序
```bash
# 直接运行
uv run python keyboard_robot_control.py

# 或使用入口点
uv run robot-keyboard

# 指定串口
uv run robot-keyboard --port /dev/ttyUSB0
```

#### 数据接收程序
```bash
# 运行数据接收
uv run robot-receiver

# 指定串口和波特率
uv run robot-receiver --port /dev/ttyUSB0 --baudrate 115200
```

#### 数据分析程序
```bash
# 运行数据分析
uv run robot-analyzer

# 分析特定文件
uv run robot-analyzer --file data/my_data.csv
```

### 网络控制系统

#### 服务端 (树莓派)
```bash
# 使用启动脚本 (推荐)
cd network_control
./start_server.sh

# 或直接运行
uv run robot-server

# 指定端口
uv run robot-server --port 9999
```

#### 客户端 (x86电脑)
```bash
# 使用启动脚本 (推荐)
cd network_control
./start_client.sh

# 或直接运行
uv run robot-client

# 指定服务器
./start_client.sh --server-ip 192.168.1.100 --server-port 8888
```

## 开发模式

### 安装开发依赖

```bash
# 安装所有依赖包括开发工具
uv sync --extra dev

# 或分别安装
uv pip install pytest black isort flake8 mypy
```

### 代码格式化和检查

```bash
# 格式化代码
uv run black .

# 排序导入
uv run isort .

# 类型检查
uv run mypy .

# 代码检查
uv run flake8 .
```

### 运行测试

```bash
# 运行所有测试
uv run pytest

# 运行特定测试
uv run pytest tests/test_robot_control.py

# 带覆盖率
uv run pytest --cov=robot_control --cov-report=html
```

## 配置管理

### 项目配置

`pyproject.toml` 包含了项目的完整配置：

- **项目信息**: 名称、版本、描述
- **依赖管理**: 运行时依赖、开发依赖、可选依赖
- **构建配置**: 构建后端、包配置
- **工具配置**: Black、isort、mypy、pytest 等

### 机器人配置

- `robot_config.yaml`: 机器人运动参数配置
- `network_control/network_config.yaml`: 网络连接配置

## 故障排除

### 常见问题

#### 1. uv 命令未找到
```bash
# 检查是否在 PATH 中
which uv

# 重新安装
curl -LsSf https://astral.sh/uv/install.sh | sh

# 或手动添加 PATH
export PATH="$HOME/.cargo/bin:$PATH"
```

#### 2. 串口权限问题 (Linux)
```bash
# 检查串口设备
ls -la /dev/ttyUSB*

# 添加到 dialout 组
sudo usermod -a -G dialout $USER

# 重新登录或
newgrp dialout
```

#### 3. tkinter 未找到
```bash
# Ubuntu/Debian
sudo apt-get install python3-tk

# CentOS/RHEL
sudo yum install tkinter

# 检查安装
uv run python -c "import tkinter; print('tkinter OK')"
```

#### 4. 网络连接问题
```bash
# 检查防火墙
sudo ufw status

# 检查端口占用
netstat -tlnp | grep 8888

# 测试连接
telnet 192.168.1.100 8888
```

### 日志调试

启用详细日志：

```bash
# 服务端
LOG_LEVEL=DEBUG uv run robot-server

# 客户端
LOG_LEVEL=DEBUG uv run robot-client
```

日志文件位置：
- 服务端: `robot_server.log`
- 客户端: `robot_client.log`

## 性能优化

### uv 性能特性

1. **并行下载**: uv 使用并行下载加速包安装
2. **智能缓存**: 本地缓存避免重复下载
3. **增量更新**: 只更新变更的依赖

### 缓存管理

```bash
# 查看缓存位置
uv cache dir

# 清理缓存
uv cache clean

# 显示缓存信息
uv cache info
```

## 部署到树莓派

### 完整部署流程

1. **在树莓派上安装 uv**:
   ```bash
   curl -LsSf https://astral.sh/uv/install.sh | sh
   ```

2. **克隆项目**:
   ```bash
   git clone <repository>
   cd stm32-robot-control/python
   ```

3. **安装依赖**:
   ```bash
   uv sync
   ```

4. **配置串口权限**:
   ```bash
   sudo usermod -a -G dialout $USER
   # 重新登录
   ```

5. **启动服务端**:
   ```bash
   cd network_control
   ./start_server.sh
   ```

### 树莓派特定配置

在 `network_control/network_config.yaml` 中：

```yaml
server:
  host: "0.0.0.0"  # 监听所有接口
  port: 8888

# 根据树莓派的串口设备调整
serial:
  port: "/dev/ttyUSB0"  # 或 /dev/ttyACM0
```

## 高级用法

### 自定义构建

```bash
# 构建 wheel 包
uv build

# 安装到系统
uv pip install --system .

# 创建可执行文件
uv run pyinstaller robot_control_server.py --onefile
```

### 环境管理

```bash
# 创建特定Python版本的环境
uv venv --python 3.11

# 激活环境
source .venv/bin/activate

# 在环境中运行
uv run --python .venv/bin/python robot-server
```

### 依赖分析

```bash
# 显示依赖树
uv tree

# 检查安全漏洞
uv pip audit

# 导出 requirements.txt
uv export --format requirements-txt > requirements.txt
```

## 支持和贡献

### 获取帮助

1. 查看项目文档: `README_DOCKER.md`
2. 检查日志文件
3. 运行诊断命令:
   ```bash
   uv run python -c "import sys; print(sys.version)"
   uv run python -c "import serial; print('Serial OK')"
   ```

### 报告问题

提交 Issue 时请包含:
- uv 版本: `uv --version`
- Python 版本: `uv run python --version`
- 操作系统信息
- 完整的错误日志
- 复现步骤

## 更新日志

### v1.0.0 (2025-01-10)
- ✨ 初始发布
- 🚀 集成 uv 包管理
- 🌐 添加网络控制功能
- 📦 创建 pyproject.toml 配置
- 🛠️ 更新所有启动脚本使用 uv
