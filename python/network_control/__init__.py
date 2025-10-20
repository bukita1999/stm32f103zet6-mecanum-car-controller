"""
网络控制模块

提供客户端和服务端的网络控制功能：
- robot_control_client: 在x86电脑上运行的键盘控制客户端
- robot_control_server: 在树莓派上运行的机器人控制服务端
"""

from .robot_control_client import RobotControlClient, main as client_main
from .robot_control_server import RobotControlServer, main as server_main

__all__ = [
    "RobotControlClient",
    "RobotControlServer",
    "client_main",
    "server_main",
]
