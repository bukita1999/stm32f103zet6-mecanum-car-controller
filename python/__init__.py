"""
STM32 Robot Control System

这是一个用于控制STM32机器人的Python软件包，
支持串口通信和网络远程控制功能。

主要功能：
- 串口通信和数据接收
- CSV数据分析和可视化
- 键盘控制机器人运动
- 网络远程控制 (客户端-服务端架构)
"""

__version__ = "1.0.0"
__author__ = "AI Assistant"
__email__ = "ai@example.com"

# 导入主要模块
from . import config_loader
from . import batch_data_receiver
from . import csv_analyzer
from .network_control import robot_control_client, robot_control_server

__all__ = [
    "config_loader",
    "batch_data_receiver",
    "csv_analyzer",
    "robot_control_client",
    "robot_control_server",
]
