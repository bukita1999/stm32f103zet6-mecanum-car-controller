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

        # 数据结构定义（与STM32端保持一致）
        self.BATCH_SIZE = 10
        self.FRAME_END = b'\x00'  # COBS帧结束符

        # TLV类型定义
        self.TLV_BATCH_DATA = 0x20

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

    def cobs_decode(self, encoded_data):
        """
        COBS解码函数

        Args:
            encoded_data: COBS编码的数据

        Returns:
            解码后的数据，失败返回None
        """
        if not encoded_data:
            return None

        output = []
        i = 0

        while i < len(encoded_data):
            code = encoded_data[i]
            i += 1

            for j in range(1, code):
                if i >= len(encoded_data):
                    return None  # 数据不完整
                output.append(encoded_data[i])
                i += 1

            if code < 0xFF and i < len(encoded_data):
                output.append(0)

        return bytes(output)

    def crc32_verify(self, data, expected_crc):
        """
        验证CRC32校验码

        Args:
            data: 数据内容
            expected_crc: 期望的CRC32值

        Returns:
            校验是否通过
        """
        calculated_crc = zlib.crc32(data) & 0xFFFFFFFF
        return calculated_crc == expected_crc

    def parse_batch_data(self, data):
        """
        解析批量数据

        Args:
            data: 批量数据内容

        Returns:
            解析后的数据字典列表
        """
        try:
            # 解析批量数据包头
            header_format = '<HHHHI'  # batch_id, data_count, reserved, start_time
            header_size = struct.calcsize(header_format)

            if len(data) < header_size:
                print("数据长度不足，无法解析包头")
                return None

            # 解析包头（需要根据实际结构体调整）
            batch_id = struct.unpack('<H', data[0:2])[0]
            data_count = struct.unpack('<H', data[2:4])[0]
            start_time = struct.unpack('<I', data[4:8])[0]

            print(f"解析到批量数据: 批次ID={batch_id}, 数据组数={data_count}, 开始时间={start_time}")

            # 解析数据内容
            parsed_data = []
            data_offset = 8  # 包头大小

            for i in range(min(data_count, self.BATCH_SIZE)):
                if data_offset + 36 > len(data):  # 每组数据36字节
                    print(f"数据{i}长度不足")
                    break

                # 解析单组数据：timestamp(4) + speed[4](8) + pwm[4](8) + error[4](16) = 36字节
                timestamp = struct.unpack('<I', data[data_offset:data_offset+4])[0]

                speeds = []
                for j in range(4):
                    speed = struct.unpack('<h', data[data_offset+4+j*2:data_offset+6+j*2])[0]
                    speeds.append(speed)

                pwms = []
                for j in range(4):
                    pwm = struct.unpack('<H', data[data_offset+12+j*2:data_offset+14+j*2])[0]
                    pwms.append(pwm)

                errors = []
                for j in range(4):
                    error = struct.unpack('<f', data[data_offset+20+j*4:data_offset+24+j*4])[0]
                    errors.append(error)

                data_record = {
                    'batch_id': batch_id,
                    'data_index': i,
                    'timestamp': timestamp,
                    'motor0_speed': speeds[0],
                    'motor1_speed': speeds[1],
                    'motor2_speed': speeds[2],
                    'motor3_speed': speeds[3],
                    'motor0_pwm': pwms[0],
                    'motor1_pwm': pwms[1],
                    'motor2_pwm': pwms[2],
                    'motor3_pwm': pwms[3],
                    'motor0_error': errors[0],
                    'motor1_error': errors[1],
                    'motor2_error': errors[2],
                    'motor3_error': errors[3],
                    'receive_time': time.time()
                }

                parsed_data.append(data_record)
                data_offset += 36  # 每组数据36字节

            return parsed_data

        except struct.error as e:
            print(f"数据解析错误: {e}")
            return None

    def create_csv_file(self):
        """创建CSV文件"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"stm32_batch_data_{timestamp}.csv"

        self.csv_file = open(filename, 'w', newline='', encoding='utf-8')
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=[
            'batch_id', 'data_index', 'timestamp',
            'motor0_speed', 'motor1_speed', 'motor2_speed', 'motor3_speed',
            'motor0_pwm', 'motor1_pwm', 'motor2_pwm', 'motor3_pwm',
            'motor0_error', 'motor1_error', 'motor2_error', 'motor3_error',
            'receive_time'
        ])

        self.csv_writer.writeheader()
        print(f"创建CSV文件: {filename}")

    def save_to_csv(self, data_records):
        """将数据保存到CSV文件"""
        if not self.csv_writer:
            self.create_csv_file()

        for record in data_records:
            self.csv_writer.writerow(record)

        print(f"保存了 {len(data_records)} 条数据记录到CSV")

    def process_frame(self, frame_data):
        """
        处理接收到的数据帧

        Args:
            frame_data: 接收到的帧数据（不含帧尾0x00）

        Returns:
            处理是否成功
        """
        try:
            # COBS解码
            decoded_data = self.cobs_decode(frame_data)
            if decoded_data is None:
                print("COBS解码失败")
                return False

            # 分离数据和CRC32校验码
            if len(decoded_data) < 5:  # 最少需要TLV头(3字节) + CRC32(4字节)
                print("数据长度太短")
                return False

            payload = decoded_data[:-4]  # 去除最后的CRC32
            crc_bytes = decoded_data[-4:]

            # 解析CRC32
            expected_crc = struct.unpack('<I', crc_bytes)[0]

            # 验证CRC32
            if not self.crc32_verify(payload, expected_crc):
                print("CRC32校验失败")
                return False

            print(f"接收到有效帧: 长度={len(decoded_data)}字节, CRC32=0x{expected_crc:08X}")

            # 解析TLV数据
            if len(payload) < 3:
                print("TLV数据长度不足")
                return False

            tlv_type = payload[0]
            tlv_length = struct.unpack('<H', payload[1:3])[0]
            tlv_value = payload[3:]

            if tlv_type != self.TLV_BATCH_DATA:
                print(f"不支持的TLV类型: 0x{tlv_type:02X}")
                return False

            if len(tlv_value) != tlv_length:
                print(f"TLV长度不匹配: 期望{tlv_length}, 实际{len(tlv_value)}")
                return False

            # 解析批量数据
            parsed_data = self.parse_batch_data(tlv_value)
            if parsed_data:
                self.save_to_csv(parsed_data)
                return True

            return False

        except Exception as e:
            print(f"帧处理错误: {e}")
            return False

    def receive_data_loop(self):
        """主数据接收循环"""
        if not self.serial:
            print("串口未连接")
            return

        print("开始接收数据... (按Ctrl+C退出)")

        buffer = b''
        frame_count = 0
        success_count = 0

        try:
            while True:
                # 读取数据
                if self.serial.in_waiting > 0:
                    data = self.serial.read(self.serial.in_waiting)
                    buffer += data

                # 查找帧边界
                frame_end = buffer.find(self.FRAME_END)
                while frame_end != -1:
                    frame_data = buffer[:frame_end]  # 提取帧数据
                    buffer = buffer[frame_end + 1:]  # 移除已处理的数据

                    frame_count += 1

                    # 处理帧数据
                    if self.process_frame(frame_data):
                        success_count += 1

                    # 查找下一个帧
                    frame_end = buffer.find(self.FRAME_END)

                # 调整延时以适应100ms发送间隔
                time.sleep(0.005)  # 5ms延时，减少CPU占用同时保持响应性

        except KeyboardInterrupt:
            print("\n用户中断接收")
        except Exception as e:
            print(f"接收循环错误: {e}")
        finally:
            print(f"\n接收统计: 总帧数={frame_count}, 成功处理={success_count}")

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
