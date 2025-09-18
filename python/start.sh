#!/bin/bash

# STM32 Robot Control 启动脚本

set -e

echo "======================================="
echo "   STM32 Robot Control System"
echo "======================================="

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
        python3 keyboard_robot_control.py "$@"
        ;;
    "receiver")
        echo "启动数据接收程序..."
        python3 batch_data_receiver.py "$@"
        ;;
    "analyzer")
        echo "启动数据分析程序..."
        python3 csv_analyzer.py "$@"
        ;;
    "config")
        echo "测试配置文件..."
        python3 config_loader.py "$@"
        ;;
    *)
        echo "错误: 未知的程序 '$PROGRAM'"
        echo "可用的程序: keyboard, receiver, analyzer, config"
        exit 1
        ;;
esac

