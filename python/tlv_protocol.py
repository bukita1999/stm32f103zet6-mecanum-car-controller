"""
TLV协议实现模块
实现与STM32小车通讯的TLV协议编解码功能
"""

import struct
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass
from datetime import datetime

# TLV类型定义
TLV_TELEMETRY = 0x10
TLV_SPEED_UNIT = 0x11
TLV_SET_SPEEDS = 0x01
TLV_SET_PID = 0x02

# 速度单位定义
SPEED_UNIT_CODE_ENC_RPM = 1

@dataclass
class MotorTelemetry:
    """单个电机遥测数据"""
    target_speed: int      # 目标速度 (RPM)
    current_speed: int     # 当前速度 (RPM)
    pwm_percent: int       # PWM百分比
    pid_error: float       # PID误差
    timestamp: datetime = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = datetime.now()

@dataclass
class TelemetryFrame:
    """完整的遥测数据帧（四个电机）"""
    motors: List[MotorTelemetry]
    speed_unit: str = "rpm"
    timestamp: datetime = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = datetime.now()

@dataclass
class SpeedUnit:
    """速度单位声明"""
    code: int
    name: str

class TLVProtocol:
    """TLV协议处理类"""
    
    @staticmethod
    def pack_tlv(tlv_type: int, data: bytes) -> bytes:
        """打包TLV数据"""
        length = len(data)
        return struct.pack('<BH', tlv_type, length) + data
    
    @staticmethod
    def unpack_tlv(data: bytes) -> List[Tuple[int, bytes]]:
        """解包TLV数据"""
        tlv_list = []
        offset = 0
        
        while offset + 3 <= len(data):
            tlv_type, length = struct.unpack('<BH', data[offset:offset+3])
            offset += 3
            
            if offset + length > len(data):
                break  # 数据不完整
                
            payload = data[offset:offset+length]
            tlv_list.append((tlv_type, payload))
            offset += length
            
        return tlv_list
    
    @staticmethod
    def pack_set_speeds(speeds: List[int]) -> bytes:
        """打包设置电机速度命令"""
        if len(speeds) != 4:
            raise ValueError("必须提供4个电机的速度值")
        
        # 打包为4个int16
        data = struct.pack('<4h', *speeds)
        return TLVProtocol.pack_tlv(TLV_SET_SPEEDS, data)
    
    @staticmethod
    def pack_set_pid(motor_id: int, kp: float, ki: float, kd: float) -> bytes:
        """打包设置PID参数命令"""
        if not (0 <= motor_id < 4):
            raise ValueError("电机ID必须在0-3范围内")
        
        # 打包为 uint8 + 3个float
        data = struct.pack('<Bfff', motor_id, kp, ki, kd)
        return TLVProtocol.pack_tlv(TLV_SET_PID, data)
    
    @staticmethod
    def unpack_telemetry(data: bytes) -> TelemetryFrame:
        """解包遥测数据"""
        if len(data) < 40:  # 4个电机 × 10字节
            raise ValueError("遥测数据长度不足")
        
        motors = []
        for i in range(4):
            offset = i * 10
            motor_data = struct.unpack('<hhHf', data[offset:offset+10])
            motor = MotorTelemetry(
                target_speed=motor_data[0],
                current_speed=motor_data[1],
                pwm_percent=motor_data[2],
                pid_error=motor_data[3]
            )
            motors.append(motor)
        
        return TelemetryFrame(motors=motors)
    
    @staticmethod
    def unpack_speed_unit(data: bytes) -> SpeedUnit:
        """解包速度单位声明"""
        if len(data) < 9:
            raise ValueError("速度单位数据长度不足")
        
        code = struct.unpack('<B', data[0:1])[0]
        name = data[1:9].rstrip(b'\x00').decode('ascii', errors='ignore')
        
        return SpeedUnit(code=code, name=name)
    
    @staticmethod
    def parse_frame(tlv_data: bytes) -> Dict[str, Any]:
        """解析完整的TLV帧"""
        result = {}
        tlv_list = TLVProtocol.unpack_tlv(tlv_data)
        
        for tlv_type, payload in tlv_list:
            if tlv_type == TLV_TELEMETRY:
                result['telemetry'] = TLVProtocol.unpack_telemetry(payload)
            elif tlv_type == TLV_SPEED_UNIT:
                result['speed_unit'] = TLVProtocol.unpack_speed_unit(payload)
            else:
                result[f'unknown_{tlv_type:02x}'] = payload
        
        return result
