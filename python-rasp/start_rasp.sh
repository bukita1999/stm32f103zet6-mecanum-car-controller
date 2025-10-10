#!/bin/bash

# STM32 Robot Control 启动脚本

set -e

echo "======================================="
echo "   STM32 Robot Control System"
echo "======================================="

# 检查uv
if ! command -v uv &> /dev/null; then
    echo "错误: 未找到uv，请先安装uv"
    echo "安装方法: curl -LsSf https://astral.sh/uv/install.sh | sh"
    exit 1
fi

# 检查并安装依赖
echo "检查项目依赖..."
if [ -f "pyproject.toml" ]; then
    uv sync --quiet
else
    uv pip install -r requirements.txt --quiet
fi
echo "✓ 依赖检查完成"

# 检查参数
if [ $# -eq 0 ]; then
    echo "使用方法: $0 [program] [options]"
    echo ""
    echo "可用的程序:"
    echo "  keyboard    - 键盘控制程序"
    echo "  receiver    - 数据接收程序"
    echo "  analyzer    - CSV数据分析程序"
    echo "  config      - 配置文件测试"
    echo ""
    echo "示例:"
    echo "  $0 keyboard"
    echo "  $0 receiver --port COM11"
    echo "  $0 analyzer"
    echo "  $0 config"
    exit 1
fi

PROGRAM=$1
shift

case $PROGRAM in
    "keyboard")
        echo "启动键盘控制程序..."
        if [ -f "pyproject.toml" ]; then
            uv run robot-keyboard "$@"
        else
            uv run python keyboard_robot_control.py "$@"
        fi
        ;;
    "receiver")
        echo "启动数据接收程序..."
        if [ -f "pyproject.toml" ]; then
            uv run robot-receiver "$@"
        else
            uv run python batch_data_receiver.py "$@"
        fi
        ;;
    "analyzer")
        echo "启动数据分析程序..."
        if [ -f "pyproject.toml" ]; then
            uv run robot-analyzer "$@"
        else
            uv run python csv_analyzer.py "$@"
        fi
        ;;
    "config")
        echo "测试配置文件..."
        if [ -f "pyproject.toml" ]; then
            uv run robot-config "$@"
        else
            uv run python config_loader.py "$@"
        fi
        ;;
    *)
        echo "错误: 未知的程序 '$PROGRAM'"
        echo "可用的程序: keyboard, receiver, analyzer, config"
        exit 1
        ;;
esac

