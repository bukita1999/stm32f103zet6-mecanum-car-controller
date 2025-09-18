#!/bin/bash

# STM32 Robot Control Docker 运行脚本

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 显示帮助信息
show_help() {
    echo -e "${BLUE}STM32 Robot Control Docker 运行脚本${NC}"
    echo "======================================="
    echo ""
    echo "用法: $0 [command] [options]"
    echo ""
    echo "Commands:"
    echo "  build      - 构建Docker镜像"
    echo "  keyboard   - 运行键盘控制程序"
    echo "  receiver   - 运行数据接收程序"
    echo "  analyzer   - 运行数据分析程序"
    echo "  config     - 测试配置文件"
    echo "  shell      - 进入容器shell"
    echo "  logs       - 查看容器日志"
    echo "  stop       - 停止所有服务"
    echo "  clean      - 清理Docker资源"
    echo "  help       - 显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  $0 build"
    echo "  $0 keyboard"
    echo "  $0 receiver --port /dev/ttyUSB0"
    echo "  $0 shell"
}

# 检查Docker和docker-compose是否安装
check_dependencies() {
    if ! command -v docker &> /dev/null; then
        echo -e "${RED}错误: Docker 未安装${NC}"
        exit 1
    fi

    if ! command -v docker-compose &> /dev/null; then
        echo -e "${RED}错误: docker-compose 未安装${NC}"
        exit 1
    fi
}

# 构建镜像
build_image() {
    echo -e "${BLUE}构建Docker镜像...${NC}"
    docker-compose build --no-cache
    echo -e "${GREEN}镜像构建完成${NC}"
}

# 运行程序
run_program() {
    local program=$1
    shift

    echo -e "${BLUE}运行 $program 程序...${NC}"

    case $program in
        "keyboard")
            docker-compose run --rm stm32-robot-control ./start.sh keyboard "$@"
            ;;
        "receiver")
            docker-compose run --rm stm32-robot-control ./start.sh receiver "$@"
            ;;
        "analyzer")
            # 设置显示环境变量用于GUI程序
            export DISPLAY=${DISPLAY:-:0}
            docker-compose run --rm stm32-robot-control ./start.sh analyzer "$@"
            ;;
        "config")
            docker-compose run --rm stm32-robot-control ./start.sh config "$@"
            ;;
        "shell")
            docker-compose run --rm stm32-robot-control bash
            ;;
        *)
            echo -e "${RED}错误: 未知的程序 '$program'${NC}"
            show_help
            exit 1
            ;;
    esac
}

# 查看日志
show_logs() {
    echo -e "${BLUE}查看容器日志...${NC}"
    docker-compose logs -f stm32-robot-control
}

# 停止服务
stop_services() {
    echo -e "${BLUE}停止所有服务...${NC}"
    docker-compose down
    echo -e "${GREEN}服务已停止${NC}"
}

# 清理资源
clean_resources() {
    echo -e "${YELLOW}清理Docker资源...${NC}"

    read -p "这将删除所有相关的容器、镜像和卷，确定要继续吗? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        docker-compose down -v --rmi all
        docker system prune -f
        echo -e "${GREEN}清理完成${NC}"
    else
        echo "已取消清理操作"
    fi
}

# 主函数
main() {
    check_dependencies

    if [ $# -eq 0 ]; then
        show_help
        exit 0
    fi

    local command=$1
    shift

    case $command in
        "build")
            build_image
            ;;
        "keyboard"|"receiver"|"analyzer"|"config"|"shell")
            run_program "$command" "$@"
            ;;
        "logs")
            show_logs
            ;;
        "stop")
            stop_services
            ;;
        "clean")
            clean_resources
            ;;
        "help"|"-h"|"--help")
            show_help
            ;;
        *)
            echo -e "${RED}错误: 未知的命令 '$command'${NC}"
            echo ""
            show_help
            exit 1
            ;;
    esac
}

# 检查是否以root权限运行（某些串口操作需要）
if [ "$EUID" -eq 0 ]; then
    echo -e "${YELLOW}警告: 以root权限运行，某些串口设备可能需要特殊配置${NC}"
fi

# 运行主函数
main "$@"

