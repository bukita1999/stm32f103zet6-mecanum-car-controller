#!/usr/bin/env python3
"""
机器人配置文件加载器
从YAML文件中加载机器人运动控制配置

作者: AI Assistant
日期: 2025年1月
"""

import yaml
import os
from typing import Dict, List, Any


class RobotConfigLoader:
    """机器人配置文件加载器"""

    def __init__(self, config_file: str = "robot_config.yaml"):
        """
        初始化配置加载器

        Args:
            config_file: 配置文件路径
        """
        self.config_file = config_file
        self.config = {}
        self.load_config()

    def load_config(self) -> bool:
        """
        加载配置文件

        Returns:
            加载是否成功
        """
        try:
            if not os.path.exists(self.config_file):
                print(f"配置文件不存在: {self.config_file}")
                return False

            with open(self.config_file, 'r', encoding='utf-8') as file:
                self.config = yaml.safe_load(file)

            print(f"✓ 成功加载配置文件: {self.config_file}")
            return True

        except yaml.YAMLError as e:
            print(f"✗ YAML配置文件解析错误: {e}")
            return False
        except Exception as e:
            print(f"✗ 加载配置文件失败: {e}")
            return False

    def get_serial_config(self) -> Dict[str, Any]:
        """
        获取串口配置

        Returns:
            串口配置字典
        """
        return self.config.get('serial', {})

    def get_movement_commands(self) -> Dict[str, Dict[str, Any]]:
        """
        获取运动命令配置

        Returns:
            运动命令配置字典
        """
        return self.config.get('movement_commands', {})

    def get_motor_config(self) -> Dict[str, Dict[str, Any]]:
        """
        获取电机配置

        Returns:
            电机配置字典
        """
        return self.config.get('motors', {})

    def get_movement_params(self) -> Dict[str, Any]:
        """
        获取运动参数

        Returns:
            运动参数字典
        """
        return self.config.get('movement_params', {})

    def get_pid_defaults(self) -> Dict[str, float]:
        """
        获取PID默认参数

        Returns:
            PID参数字典
        """
        return self.config.get('pid_defaults', {})

    def get_command_by_name(self, command_name: str) -> str:
        """
        根据命令名称获取命令字符串

        Args:
            command_name: 命令名称 (forward, backward, left, right, stop)

        Returns:
            命令字符串
        """
        commands = self.get_movement_commands()
        if command_name in commands:
            return commands[command_name].get('command', '')
        return ''

    def get_command_speeds(self, command_name: str) -> List[int]:
        """
        根据命令名称获取速度列表

        Args:
            command_name: 命令名称 (forward, backward, left, right, stop)

        Returns:
            速度列表 [motor0, motor1, motor2, motor3]
        """
        commands = self.get_movement_commands()
        if command_name in commands:
            return commands[command_name].get('speeds', [0, 0, 0, 0])
        return [0, 0, 0, 0]

    def get_command_description(self, command_name: str) -> str:
        """
        根据命令名称获取命令描述

        Args:
            command_name: 命令名称

        Returns:
            命令描述
        """
        commands = self.get_movement_commands()
        if command_name in commands:
            return commands[command_name].get('description', command_name)
        return command_name

    def validate_config(self) -> bool:
        """
        验证配置文件是否有效

        Returns:
            配置是否有效
        """
        try:
            # 检查必需的配置项
            required_sections = ['serial', 'movement_commands']
            for section in required_sections:
                if section not in self.config:
                    print(f"✗ 配置缺少必需的章节: {section}")
                    return False

            # 检查运动命令
            commands = self.get_movement_commands()
            required_commands = ['forward', 'backward', 'left', 'right', 'stop']
            for cmd in required_commands:
                if cmd not in commands:
                    print(f"✗ 缺少必需的运动命令: {cmd}")
                    return False

                # 检查每个命令是否有必需的字段
                if 'command' not in commands[cmd]:
                    print(f"✗ 命令 {cmd} 缺少 'command' 字段")
                    return False

                if 'speeds' not in commands[cmd]:
                    print(f"✗ 命令 {cmd} 缺少 'speeds' 字段")
                    return False

                # 检查速度列表长度
                speeds = commands[cmd]['speeds']
                if not isinstance(speeds, list) or len(speeds) != 4:
                    print(f"✗ 命令 {cmd} 的 speeds 必须是包含4个元素的列表")
                    return False

            print("✓ 配置文件验证通过")
            return True

        except Exception as e:
            print(f"✗ 配置文件验证失败: {e}")
            return False


def main():
    """测试配置加载器"""
    print("测试机器人配置文件加载器")
    print("=" * 40)

    loader = RobotConfigLoader()

    if not loader.validate_config():
        return

    # 显示配置信息
    print("\n串口配置:")
    serial_config = loader.get_serial_config()
    for key, value in serial_config.items():
        print(f"  {key}: {value}")

    print("\n运动命令:")
    commands = loader.get_movement_commands()
    for name, cmd_info in commands.items():
        print(f"  {name}: {cmd_info.get('description', '')}")
        print(f"    命令: {cmd_info.get('command', '')}")
        print(f"    速度: {cmd_info.get('speeds', [])}")

    print("\n电机配置:")
    motors = loader.get_motor_config()
    for motor_id, motor_info in motors.items():
        print(f"  {motor_id}: {motor_info.get('name', '')}")


if __name__ == "__main__":
    main()

