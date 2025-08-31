"""
USB通讯模块
实现与STM32小车的异步USB通讯功能
"""

import asyncio
import logging
import serial_asyncio
from typing import Optional, Callable, List
from dataclasses import dataclass
from datetime import datetime

from tlv_protocol import TLVProtocol, TelemetryFrame
from cobs_crc import FrameProcessor

@dataclass
class USBConfig:
    """USB通讯配置"""
    port: str = 'COM4'  # 默认串口，需要根据实际情况修改
    baudrate: int = 115200
    timeout: float = 1.0
    read_timeout: float = 0.1

class USBCommunication:
    """USB异步通讯类"""
    
    def __init__(self, config: USBConfig):
        self.config = config
        self.reader: Optional[asyncio.StreamReader] = None
        self.writer: Optional[asyncio.StreamWriter] = None
        self.frame_processor = FrameProcessor()
        self.tlv_protocol = TLVProtocol()
        self.is_connected = False
        self.receive_buffer = bytearray()
        
        # 设置日志
        self.logger = logging.getLogger(__name__)
        logging.basicConfig(level=logging.INFO)
        
        # 回调函数
        self.telemetry_callback: Optional[Callable[[TelemetryFrame], None]] = None
    
    async def connect(self) -> bool:
        """连接USB设备"""
        try:
            self.logger.info(f"正在连接到 {self.config.port}...")
            
            self.reader, self.writer = await serial_asyncio.open_serial_connection(
                url=self.config.port,
                baudrate=self.config.baudrate
            )
            
            self.is_connected = True
            self.logger.info(f"成功连接到 {self.config.port}")
            
            # 启动接收任务
            asyncio.create_task(self._receive_task())
            
            return True
            
        except Exception as e:
            self.logger.error(f"连接失败: {e}")
            return False
    
    async def disconnect(self):
        """断开连接"""
        if self.writer:
            self.writer.close()
            await self.writer.wait_closed()
        
        self.is_connected = False
        self.logger.info("已断开连接")
    
    async def _receive_task(self):
        """异步接收任务"""
        self.logger.info("开始接收数据...")
        
        while self.is_connected:
            try:
                # 读取数据
                data = await asyncio.wait_for(
                    self.reader.read(1024), 
                    timeout=self.config.read_timeout
                )
                
                if not data:
                    continue
                
                # 添加到缓冲区
                self.receive_buffer.extend(data)
                
                # 处理完整的帧
                await self._process_received_data()
                
            except asyncio.TimeoutError:
                continue
            except Exception as e:
                self.logger.error(f"接收数据错误: {e}")
                await asyncio.sleep(0.1)
    
    async def _process_received_data(self):
        """处理接收到的数据"""
        while True:
            # 查找帧尾标志
            frame_end = self.receive_buffer.find(b'\x00')
            if frame_end == -1:
                break  # 没有完整的帧
            
            # 提取完整帧（包含帧尾）
            frame_data = bytes(self.receive_buffer[:frame_end + 1])
            self.receive_buffer = self.receive_buffer[frame_end + 1:]
            
            try:
                # 解包帧
                payload = self.frame_processor.unpack_frame(frame_data)
                
                # 解析TLV数据
                parsed = self.tlv_protocol.parse_frame(payload)
                
                # 处理遥测数据
                if 'telemetry' in parsed:
                    telemetry = parsed['telemetry']
                    self.logger.debug(f"收到遥测数据: {len(telemetry.motors)} 个电机")
                    
                    if self.telemetry_callback:
                        self.telemetry_callback(telemetry)
                
            except Exception as e:
                self.logger.warning(f"帧解析错误: {e}")
                continue
    
    async def send_frame(self, payload: bytes):
        """发送数据帧"""
        if not self.is_connected or not self.writer:
            raise ConnectionError("未连接到设备")
        
        try:
            frame = self.frame_processor.pack_frame(payload)
            self.writer.write(frame)
            await self.writer.drain()
            self.logger.debug(f"发送了 {len(frame)} 字节数据")
            
        except Exception as e:
            self.logger.error(f"发送数据失败: {e}")
            raise
    
    async def set_motor_speeds(self, speeds: List[int]):
        """设置电机速度"""
        if len(speeds) != 4:
            raise ValueError("必须提供4个电机的速度值")
        
        self.logger.info(f"设置电机速度: {speeds}")
        
        tlv_data = self.tlv_protocol.pack_set_speeds(speeds)
        await self.send_frame(tlv_data)
    
    async def set_motor_pid(self, motor_id: int, kp: float, ki: float, kd: float):
        """设置电机PID参数"""
        self.logger.info(f"设置电机{motor_id} PID参数: Kp={kp}, Ki={ki}, Kd={kd}")
        
        tlv_data = self.tlv_protocol.pack_set_pid(motor_id, kp, ki, kd)
        await self.send_frame(tlv_data)
    
    def set_telemetry_callback(self, callback: Callable[[TelemetryFrame], None]):
        """设置遥测数据回调函数"""
        self.telemetry_callback = callback
    
    async def wait_for_telemetry(self, count: int = 1, timeout: float = 10.0) -> List[TelemetryFrame]:
        """等待指定数量的遥测数据"""
        telemetry_list = []
        
        def collect_telemetry(frame: TelemetryFrame):
            telemetry_list.append(frame)
        
        # 临时设置回调
        original_callback = self.telemetry_callback
        self.set_telemetry_callback(collect_telemetry)
        
        try:
            # 等待收集到足够的数据
            start_time = asyncio.get_event_loop().time()
            while len(telemetry_list) < count:
                if asyncio.get_event_loop().time() - start_time > timeout:
                    raise TimeoutError(f"等待遥测数据超时，只收到 {len(telemetry_list)}/{count} 帧")
                await asyncio.sleep(0.01)
            
            return telemetry_list[:count]
            
        finally:
            # 恢复原来的回调
            self.telemetry_callback = original_callback
