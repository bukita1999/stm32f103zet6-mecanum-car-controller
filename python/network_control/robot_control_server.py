#!/usr/bin/env python3
"""
机器人网络控制服务端
在树莓派上运行，接收网络指令并通过串口控制机器人运动

使用异步I/O处理网络通信和串口控制
支持多客户端连接和命令队列处理

作者: AI Assistant
日期: 2025年1月
"""

import asyncio
import socket
import json
import time
import sys
import os
import logging
from datetime import datetime
import serial
import serial_asyncio
from config_loader import RobotConfigLoader
import yaml


class SerialController:
    """异步串口控制器"""

    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, timeout=1.0):
        """
        初始化串口控制器

        Args:
            port: 串口端口
            baudrate: 波特率
            timeout: 超时时间
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.writer = None
        self.is_connected = False

    async def connect(self):
        """连接到串口设备"""
        try:
            # 创建异步串口连接
            self.reader, self.writer = await serial_asyncio.open_serial_connection(
                url=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            self.is_connected = True
            logging.info(f"✓ 成功连接到串口: {self.port} @ {self.baudrate}bps")
            return True
        except Exception as e:
            logging.error(f"✗ 串口连接失败: {e}")
            return False

    async def disconnect(self):
        """断开串口连接"""
        if self.writer:
            self.writer.close()
            await self.writer.wait_closed()
        self.writer = None
        self.reader = None
        self.is_connected = False
        logging.info("串口连接已关闭")

    async def send_command(self, command):
        """发送命令到串口"""
        if not self.is_connected or not self.writer:
            # 串口未连接时，打印接收到的数据
            logging.info(f"📄 [模拟模式] 接收到的命令数据: {command.strip()}")
            print(f"📄 [模拟模式] 接收到的命令数据: {command.strip()}")
            return True

        try:
            # 发送命令
            self.writer.write(command.encode('utf-8'))
            await self.writer.drain()

            logging.debug(f"→ 发送串口命令: {command.strip()}")
            return True

        except Exception as e:
            logging.error(f"✗ 发送串口命令失败: {e}")
            self.is_connected = False
            return False


class RobotControlServer:
    """机器人网络控制服务端"""

    def __init__(self, network_config_file='network_control/network_config.yaml', robot_config_file='./robot_config.yaml'):
        """
        初始化服务端

        Args:
            network_config_file: 网络配置文件路径
            robot_config_file: 机器人配置文件路径
        """
        # 从网络配置文件读取服务器配置
        self.network_config_file = network_config_file
        self.robot_config_file = robot_config_file

        # 加载网络配置
        self.load_network_config()

        # 初始化其他属性
        self.server = None
        self.is_running = False

        # 客户端连接
        self.clients = set()
        self.command_queue = asyncio.Queue()

        # 串口控制器
        self.serial_controller = None

        # 配置加载器
        self.config_loader = None

        # 运动命令映射
        self.commands = {}

        # 当前运动状态
        self.current_command = 'stop'
        self.last_command_time = 0

        # 设置日志
        self.setup_logging()

        # 客户端连接
        self.clients = set()
        self.command_queue = asyncio.Queue()

        # 串口控制器
        self.serial_controller = None

        # 配置加载器
        self.config_loader = None

        # 运动命令映射
        self.commands = {}

        # 当前运动状态
        self.current_command = 'stop'
        self.last_command_time = 0

        # 设置日志
        self.setup_logging()

    def setup_logging(self):
        """设置日志配置"""
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.StreamHandler(),
                logging.FileHandler('robot_server.log')
            ]
        )

    def load_network_config(self):
        """加载网络配置文件"""
        try:
            with open(self.network_config_file, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)

            server_config = config.get('server', {})
            self.host = server_config.get('host', '0.0.0.0')
            self.port = server_config.get('port', 8888)

            logging.info(f"✓ 网络配置文件加载成功 [{self.network_config_file}]: host={self.host}, port={self.port}")

        except FileNotFoundError as e:
            logging.warning(f"⚠️ 网络配置文件不存在 [{self.network_config_file}]，使用默认配置: {e}")
            # 使用默认配置
            self.host = '0.0.0.0'
            self.port = 8888
        except yaml.YAMLError as e:
            logging.warning(f"⚠️ 网络配置文件格式错误 [{self.network_config_file}]，使用默认配置: {e}")
            # 使用默认配置
            self.host = '0.0.0.0'
            self.port = 8888
        except Exception as e:
            logging.warning(f"⚠️ 加载网络配置文件时发生未知错误 [{self.network_config_file}]，使用默认配置: {e}")
            # 使用默认配置
            self.host = '0.0.0.0'
            self.port = 8888

    async def load_config(self):
        """加载机器人配置文件"""
        try:
            self.config_loader = RobotConfigLoader(self.robot_config_file)

            # 获取串口配置
            serial_config = self.config_loader.get_serial_config()
            port = serial_config.get('port', '/dev/ttyUSB0')
            baudrate = serial_config.get('baudrate', 115200)
            timeout = serial_config.get('timeout', 1.0)

            # 创建串口控制器
            self.serial_controller = SerialController(port, baudrate, timeout)

            # 加载运动命令
            movement_commands = self.config_loader.get_movement_commands()
            for cmd_name, cmd_info in movement_commands.items():
                self.commands[cmd_name] = cmd_info.get('command', '')

            logging.info(f"✓ 机器人配置文件加载成功: {self.robot_config_file}")
            return True

        except Exception as e:
            logging.error(f"✗ 机器人配置文件加载失败 [{self.robot_config_file}]: {e}")
            return False

    async def handle_client(self, reader, writer):
        """处理客户端连接"""
        addr = writer.get_extra_info('peername')
        logging.info(f"客户端连接: {addr}")

        # 添加到客户端集合
        self.clients.add(writer)

        try:
            while self.is_running:
                # 读取数据
                data = await reader.read(1024)
                if not data:
                    break

                # 解析消息
                try:
                    message = json.loads(data.decode('utf-8'))
                    await self.process_message(message, writer)
                except json.JSONDecodeError as e:
                    logging.warning(f"无效的JSON消息: {e}")

        except Exception as e:
            logging.error(f"客户端处理错误: {e}")
        finally:
            # 移除客户端
            self.clients.discard(writer)
            writer.close()
            await writer.wait_closed()
            logging.info(f"客户端断开: {addr}")

    async def process_message(self, message, writer):
        """处理接收到的消息"""
        try:
            msg_type = message.get('type', '')
            timestamp = message.get('timestamp', time.time())

            if msg_type == 'control':
                command = message.get('command', '')
                if command in self.commands:
                    # 添加到命令队列
                    await self.command_queue.put({
                        'command': command,
                        'timestamp': timestamp,
                        'client': writer
                    })
                    logging.info(f"接收控制命令: {command}")
                else:
                    logging.warning(f"未知的控制命令: {command}")

            elif msg_type == 'ping':
                # 响应ping消息
                response = {
                    'type': 'pong',
                    'timestamp': time.time(),
                    'server_time': time.time()
                }
                writer.write(json.dumps(response).encode('utf-8'))
                await writer.drain()

            else:
                logging.warning(f"未知的消息类型: {msg_type}")

        except Exception as e:
            logging.error(f"消息处理错误: {e}")

    async def command_processor(self):
        """命令处理器 - 处理命令队列"""
        logging.info("命令处理器启动")

        while self.is_running:
            try:
                # 获取命令（带超时）
                command_data = await asyncio.wait_for(
                    self.command_queue.get(),
                    timeout=1.0
                )

                command = command_data['command']
                timestamp = command_data['timestamp']

                # 检查是否是重复命令
                if command == self.current_command:
                    continue

                # 获取串口命令
                if command in self.commands:
                    serial_command = self.commands[command]

                    # 发送到串口
                    if await self.serial_controller.send_command(serial_command):
                        self.current_command = command
                        self.last_command_time = time.time()

                        # 广播状态更新给所有客户端
                        await self.broadcast_status()

                        logging.info(f"✓ 执行命令: {command} -> {serial_command.strip()}")
                    else:
                        logging.error(f"✗ 执行命令失败: {command}")

                self.command_queue.task_done()

            except asyncio.TimeoutError:
                continue
            except Exception as e:
                logging.error(f"命令处理错误: {e}")

        logging.info("命令处理器停止")

    async def broadcast_status(self):
        """广播当前状态给所有客户端"""
        if not self.clients:
            return

        status_message = {
            'type': 'status',
            'current_command': self.current_command,
            'last_command_time': self.last_command_time,
            'timestamp': time.time()
        }

        message_data = json.dumps(status_message).encode('utf-8')

        # 发送给所有客户端
        disconnected_clients = set()
        for client in self.clients:
            try:
                client.write(message_data)
                await client.drain()
            except Exception as e:
                logging.warning(f"广播状态失败: {e}")
                disconnected_clients.add(client)

        # 移除断开的客户端
        for client in disconnected_clients:
            self.clients.discard(client)

    async def heartbeat(self):
        """心跳检查"""
        while self.is_running:
            await asyncio.sleep(30)  # 每30秒检查一次

            # 检查串口连接（仅在非模拟模式下尝试重连）
            if not self.serial_controller.is_connected:
                logging.warning("串口连接丢失，尝试重连...")
                if await self.serial_controller.connect():
                    logging.info("✓ 串口重连成功")
                else:
                    logging.info("📄 继续以模拟模式运行")

            # 发送心跳到所有客户端
            await self.broadcast_status()

    async def start_server(self):
        """启动服务器"""
        logging.info("机器人网络控制服务端启动中...")
        logging.info(f"监听地址: {self.host}:{self.port}")

        # 加载配置
        if not await self.load_config():
            return False

        # 连接串口
        if not await self.serial_controller.connect():
            logging.warning("⚠️ 无法连接到串口设备，将以模拟模式运行")
            logging.info("📄 模拟模式：将打印接收到的所有命令数据")
            # 不返回False，继续启动服务器

        # 启动服务器
        self.server = await asyncio.start_server(
            self.handle_client, self.host, self.port
        )

        self.is_running = True
        logging.info("✓ 服务器启动成功")

        # 启动任务
        tasks = [
            self.command_processor(),
            self.heartbeat()
        ]

        try:
            # 并发运行所有任务
            await asyncio.gather(*tasks, return_exceptions=True)
        except KeyboardInterrupt:
            logging.info("接收到中断信号")
        except Exception as e:
            logging.error(f"服务器运行错误: {e}")
        finally:
            await self.stop_server()

        return True

    async def stop_server(self):
        """停止服务器"""
        logging.info("正在停止服务器...")
        self.is_running = False

        # 关闭所有客户端连接
        for client in self.clients:
            try:
                client.close()
                await client.wait_closed()
            except:
                pass
        self.clients.clear()

        # 关闭服务器
        if self.server:
            self.server.close()
            await self.server.wait_closed()

        # 断开串口
        if self.serial_controller:
            await self.serial_controller.disconnect()

        logging.info("服务器已停止")

    def run(self):
        """运行服务器"""
        try:
            asyncio.run(self.start_server())
        except KeyboardInterrupt:
            logging.info("用户中断")
        except Exception as e:
            logging.error(f"服务器启动失败: {e}")
        finally:
            logging.info("服务器程序退出")


def main():
    """主函数"""
    print("机器人网络控制服务端")
    print("=" * 40)
    print("按Ctrl+C退出程序")
    print()

    # 创建服务器实例
    server = RobotControlServer()

    try:
        # 运行服务器
        server.run()

    except Exception as e:
        print(f"程序运行错误: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
