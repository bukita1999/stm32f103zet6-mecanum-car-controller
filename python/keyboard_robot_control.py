#!/usr/bin/env python3
"""
机器人键盘控制程序
通过键盘控制机器人运动，使用COM10串口通信

控制键：
W - 前进
S - 后退
A - 左转
D - 右转
空格 - 停止
Q - 退出程序

作者: AI Assistant
日期: 2025年1月
"""

import serial
import time
import sys
import os
import msvcrt  # Windows键盘输入库
from config_loader import RobotConfigLoader


class RobotController:
    """机器人控制器"""

    def __init__(self, config_file='robot_config.yaml'):
        """
        初始化机器人控制器

        Args:
            config_file: 配置文件路径
        """
        # 加载配置文件
        self.config_loader = RobotConfigLoader(config_file)

        # 从配置文件获取串口设置
        serial_config = self.config_loader.get_serial_config()
        self.port = serial_config.get('port', 'COM10')
        self.baudrate = serial_config.get('baudrate', 115200)
        self.timeout = serial_config.get('timeout', 1.0)

        self.serial = None
        self.is_running = False

        # 从配置文件加载运动命令
        self.commands = {}
        movement_commands = self.config_loader.get_movement_commands()
        for cmd_name, cmd_info in movement_commands.items():
            self.commands[cmd_name] = cmd_info.get('command', '')

        # 当前运动状态
        self.current_command = 'stop'

    def connect_serial(self):
        """连接到串口设备"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            print(f"✓ 成功连接到串口: {self.port} @ {self.baudrate}bps")
            return True
        except serial.SerialException as e:
            print(f"✗ 串口连接失败: {e}")
            return False

    def send_command(self, command_key):
        """发送运动命令"""
        if not self.serial or not self.serial.is_open:
            print("✗ 串口未连接")
            return False

        if command_key not in self.commands:
            print(f"✗ 无效的命令: {command_key}")
            return False

        command = self.commands[command_key]

        try:
            # 发送命令
            self.serial.write(command.encode('utf-8'))
            self.serial.flush()

            # 更新当前状态
            self.current_command = command_key

            print(f"→ 发送命令: {command_key} ({command})")
            return True

        except Exception as e:
            print(f"✗ 发送命令失败: {e}")
            return False

    def stop_robot(self):
        """停止机器人"""
        self.send_command('stop')


    def print_control_info(self):
        """打印控制信息"""
        print("\n" + "="*60)
        print("           机器人键盘控制程序")
        print("="*60)
        print("控制键说明:")
        print("  W - 前进      S - 后退")
        print("  A - 左转      D - 右转")
        print("  空格 - 停止   Q - 退出程序")
        print("="*60)
        print("状态: 准备就绪，请按任意控制键开始...")
        print("注意: 每个按键后机器人会持续运动，直到下次按键")
        print("="*60)

    def start_keyboard_control(self):
        """开始键盘控制"""
        self.print_control_info()

        # 连接串口
        if not self.connect_serial():
            return

        self.is_running = True
        print("\n等待键盘输入... (按Q退出)")

        try:
            while self.is_running:
                # 检查是否有键盘输入
                if msvcrt.kbhit():
                    key = msvcrt.getch()
                    key_char = key.decode('utf-8', errors='ignore').lower()

                    if key_char == 'q':
                        print("\n正在退出程序...")
                        self.is_running = False
                        break
                    elif key_char == 'w':
                        self.send_command('forward')
                        print("前进中...")
                    elif key_char == 's':
                        self.send_command('backward')
                        print("后退中...")
                    elif key_char == 'a':
                        self.send_command('left')
                        print("左转中...")
                    elif key_char == 'd':
                        self.send_command('right')
                        print("右转中...")
                    elif key_char == ' ':  # 空格键
                        self.send_command('stop')
                        print("已停止")

                time.sleep(0.01)  # 小延时避免CPU占用过高

        except KeyboardInterrupt:
            print("\n用户中断")
        finally:
            # 停止机器人
            self.stop_robot()
            # 关闭串口
            if self.serial and self.serial.is_open:
                self.serial.close()
                print("串口连接已关闭")

    def close(self):
        """关闭控制器"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("串口连接已关闭")


def main():
    """主函数"""
    print("机器人键盘控制程序启动中...")
    print("使用YAML配置文件: robot_config.yaml")

    # 创建控制器实例（使用默认配置文件）
    controller = RobotController()

    try:
        # 开始键盘控制
        controller.start_keyboard_control()

    except Exception as e:
        print(f"程序运行错误: {e}")
    finally:
        controller.close()
        print("程序已退出")


if __name__ == "__main__":
    main()
