#!/usr/bin/env python3
"""
STM32批量数据接收器
接收STM32通过USB CDC发送的批量二进制数据并保存到CSV文件

作者: AI Assistant
日期: 2025年1月
"""

import serial
import struct
import time
import csv
import os
from datetime import datetime
import zlib  # 用于CRC32校验


class STM32BatchDataReceiver:
    """STM32批量数据接收器"""

    def __init__(self, port='COM4', baudrate=115200, timeout=0.1):
        """
        初始化接收器

        Args:
            port: 串口端口号
            baudrate: 波特率
            timeout: 读取超时时间 (调整为0.1s以适应100ms发送间隔)
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.csv_writer = None
        self.csv_file = None

        # 数据结构定义（文本模式，不再需要批量大小和帧同步）
        # 直接接收STM32发送的文本格式数据

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
            print(f"成功连接到串口: {self.port} @ {self.baudrate}bps")
            return True
        except serial.SerialException as e:
            print(f"串口连接失败: {e}")
            return False

    # COBS解码和CRC32校验方法已移除，现在使用文本模式直接解析

    # parse_batch_data方法已移除，现在使用文本解析方法

    def create_csv_file(self):
        """创建CSV文件（适配文本模式）"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"stm32_text_data_{timestamp}.csv"

        self.csv_file = open(filename, 'w', newline='', encoding='utf-8')
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=[
            'batch_id', 'data_index', 'timestamp',
            'motor0_target_speed', 'motor0_current_speed', 'motor0_pwm', 'motor0_error',
            'motor1_target_speed', 'motor1_current_speed', 'motor1_pwm', 'motor1_error',
            'motor2_target_speed', 'motor2_current_speed', 'motor2_pwm', 'motor2_error',
            'motor3_target_speed', 'motor3_current_speed', 'motor3_pwm', 'motor3_error',
            'receive_time'
        ])

        self.csv_writer.writeheader()
        print(f"创建CSV文件: {filename}")
        print("注意: 现在使用文本模式，数据来自STM32 UART监控输出格式")

    def save_to_csv(self, data_records):
        """将数据保存到CSV文件"""
        if not self.csv_writer:
            self.create_csv_file()

        for record in data_records:
            self.csv_writer.writerow(record)

        print(f"保存了 {len(data_records)} 条数据记录到CSV")

    def process_frame(self, frame_data):
        """
        处理接收到的数据帧（文本格式，与UART监控输出格式一致）

        Args:
            frame_data: 接收到的帧数据

        Returns:
            处理是否成功
        """
        try:
            # 将字节数据转换为字符串
            text_data = frame_data.decode('utf-8', errors='ignore')

            print(f"接收到数据帧: {len(text_data)}字符")
            print(f"原始数据:\n{text_data}")

            # 按行分割数据
            lines = text_data.strip().split('\n')

            parsed_records = []
            motor_data = {}  # 临时存储电机数据
            system_info = None

            for line in lines:
                line = line.strip()
                if not line:
                    continue

                # 解析电机状态行
                if line.startswith('Motor'):
                    motor_record = self.parse_motor_line(line)
                    if motor_record:
                        motor_id = motor_record['data_index']
                        motor_data[motor_id] = motor_record

                # 解析系统状态行
                elif line.startswith('System:'):
                    system_info = self.parse_system_line(line)
                    if system_info:
                        print(f"系统状态: {system_info}")

            # 如果收集到了电机数据，合并为一条完整记录
            if motor_data:
                # 创建一条包含所有电机数据的记录
                complete_record = {
                    'batch_id': 0,
                    'data_index': 0,  # 批量记录
                    'timestamp': int(time.time() * 1000),
                    'receive_time': time.time()
                }

                # 为所有电机设置数据
                for i in range(4):
                    if i in motor_data:
                        # 从对应的电机记录中获取数据
                        motor_record = motor_data[i]
                        complete_record[f'motor{i}_target_speed'] = motor_record[f'motor{i}_target_speed']
                        complete_record[f'motor{i}_current_speed'] = motor_record[f'motor{i}_current_speed']
                        complete_record[f'motor{i}_pwm'] = motor_record[f'motor{i}_pwm']
                        complete_record[f'motor{i}_error'] = motor_record[f'motor{i}_error']
                    else:
                        # 如果某个电机没有数据，设为None
                        complete_record[f'motor{i}_target_speed'] = None
                        complete_record[f'motor{i}_current_speed'] = None
                        complete_record[f'motor{i}_pwm'] = None
                        complete_record[f'motor{i}_error'] = None

                parsed_records.append(complete_record)

            # 保存解析后的数据
            if parsed_records:
                self.save_to_csv(parsed_records)
                print(f"成功解析并保存了 {len(parsed_records)} 条完整记录")
                return True

            return False

        except Exception as e:
            print(f"帧处理错误: {e}")
            return False

    def parse_motor_line(self, line):
        """
        解析电机状态行
        格式: "Motor1: Target:1000 Current:950 RPM, PWM:75%, Error:5.00"

        Args:
            line: 电机状态行字符串

        Returns:
            解析后的数据字典
        """
        try:
            # 使用正则表达式解析
            import re

            # 匹配模式: Motor1: Target:1000 Current:950 RPM, PWM:75%, Error:5.00
            pattern = r'Motor(\d+): Target:(-?\d+) Current:(-?\d+) RPM, PWM:(\d+)%, Error:(-?\d+)\.(\d+)'
            match = re.match(pattern, line)

            if match:
                motor_id = int(match.group(1)) - 1  # 转换为0-based索引
                target_speed = int(match.group(2))
                current_speed = int(match.group(3))
                pwm_percent = int(match.group(4))
                error_integer = int(match.group(5))
                error_decimal = int(match.group(6))

                # 重建误差值为浮点数
                error = error_integer + error_decimal / 100.0
                if error_integer < 0:
                    error = error_integer - error_decimal / 100.0

                # 构建记录（文本模式，单电机数据）
                record = {
                    'batch_id': 0,  # 文本模式下固定为0
                    'data_index': motor_id,
                    'timestamp': int(time.time() * 1000),  # 毫秒级时间戳
                    'receive_time': time.time()
                }

                # 只为当前电机设置数据，其他电机字段设为空或0
                for i in range(4):
                    if i == motor_id:
                        record[f'motor{i}_target_speed'] = target_speed
                        record[f'motor{i}_current_speed'] = current_speed
                        record[f'motor{i}_pwm'] = pwm_percent
                        record[f'motor{i}_error'] = error
                    else:
                        # 其他电机没有数据，设为None或0
                        record[f'motor{i}_target_speed'] = None
                        record[f'motor{i}_current_speed'] = None
                        record[f'motor{i}_pwm'] = None
                        record[f'motor{i}_error'] = None

                return record
            else:
                print(f"无法解析电机行: {line}")
                return None

        except Exception as e:
            print(f"解析电机行错误: {e}")
            return None

    def parse_system_line(self, line):
        """
        解析系统状态行
        格式: "System: Init=1, PCA9685=0, MotorErr=0"

        Args:
            line: 系统状态行字符串

        Returns:
            系统状态信息字符串
        """
        try:
            # 简单的字符串提取
            if 'Init=' in line and 'PCA9685=' in line and 'MotorErr=' in line:
                return line.replace('System: ', '')
            return None
        except Exception as e:
            print(f"解析系统行错误: {e}")
            return None

    def receive_data_loop(self):
        """主数据接收循环（文本模式）"""
        if not self.serial:
            print("串口未连接")
            return

        print("开始接收数据... (按Ctrl+C退出)")
        print("注意: 现在使用文本模式，直接接收STM32发送的电机状态信息")

        try:
            while True:
                # 读取可用数据
                if self.serial.in_waiting > 0:
                    data = self.serial.read(self.serial.in_waiting)

                    # 处理接收到的数据
                    if self.process_frame(data):
                        print("✓ 成功处理一批数据")
                    else:
                        print("✗ 数据处理失败")

                # 调整延时以适应100ms发送间隔
                time.sleep(0.01)  # 10ms延时

        except KeyboardInterrupt:
            print("\n用户中断接收")
        except Exception as e:
            print(f"接收循环错误: {e}")
        finally:
            print("\n接收程序结束")

    def close(self):
        """关闭连接和文件"""
        if self.serial:
            self.serial.close()
            print("串口连接已关闭")

        if self.csv_file:
            self.csv_file.close()
            print("CSV文件已关闭")


def main():
    """主函数"""
    print("STM32批量数据接收器")
    print("=" * 40)

    # 创建接收器实例
    receiver = STM32BatchDataReceiver(port='COM11', baudrate=115200)

    try:
        # 连接串口
        if not receiver.connect_serial():
            return

        # 启动数据接收循环
        receiver.receive_data_loop()

    except Exception as e:
        print(f"程序运行错误: {e}")
    finally:
        receiver.close()


if __name__ == "__main__":
    main()
