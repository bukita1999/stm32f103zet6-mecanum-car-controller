"""
COBS编码和CRC32校验模块
实现与STM32小车通讯的帧格式处理
"""

import crcmod

class COBSCodec:
    """COBS编码解码器"""
    
    @staticmethod
    def encode(data: bytes) -> bytes:
        """COBS编码"""
        if not data:
            return b'\x01'
        
        result = bytearray()
        code_idx = 0
        code = 1
        result.append(0)  # 占位符，稍后填充
        
        for byte in data:
            if byte == 0:
                result[code_idx] = code
                code = 1
                code_idx = len(result)
                result.append(0)  # 新的占位符
            else:
                result.append(byte)
                code += 1
                if code == 0xFF:
                    result[code_idx] = code
                    code = 1
                    code_idx = len(result)
                    result.append(0)  # 新的占位符
        
        result[code_idx] = code
        return bytes(result)
    
    @staticmethod
    def decode(data: bytes) -> bytes:
        """COBS解码"""
        if not data:
            return b''
        
        result = bytearray()
        i = 0
        
        while i < len(data):
            code = data[i]
            if code == 0:
                return b''  # 无效的COBS数据
            
            i += 1
            for j in range(1, code):
                if i >= len(data):
                    return b''  # 数据不完整
                result.append(data[i])
                i += 1
            
            if code != 0xFF and i < len(data):
                result.append(0)
        
        return bytes(result)

class CRC32:
    """CRC32校验器（zlib/IEEE标准）"""
    
    def __init__(self):
        # 创建zlib标准的CRC32计算器
        self.crc_func = crcmod.mkCrcFun(0x104C11DB7, initCrc=0xFFFFFFFF, xorOut=0xFFFFFFFF)
    
    def calculate(self, data: bytes) -> int:
        """计算CRC32校验值"""
        return self.crc_func(data)
    
    def verify(self, data: bytes, expected_crc: int) -> bool:
        """验证CRC32校验值"""
        return self.calculate(data) == expected_crc

class FrameProcessor:
    """帧处理器"""
    
    def __init__(self):
        self.cobs = COBSCodec()
        self.crc32 = CRC32()
    
    def pack_frame(self, payload: bytes) -> bytes:
        """打包帧：添加CRC32，COBS编码，添加帧尾"""
        # 计算CRC32
        crc = self.crc32.calculate(payload)
        crc_bytes = crc.to_bytes(4, byteorder='little')
        
        # 组合payload和CRC
        raw_frame = payload + crc_bytes
        
        # COBS编码
        encoded = self.cobs.encode(raw_frame)
        
        # 添加帧尾
        return encoded + b'\x00'
    
    def unpack_frame(self, frame_data: bytes) -> bytes:
        """解包帧：COBS解码，验证CRC32，返回payload"""
        if not frame_data.endswith(b'\x00'):
            raise ValueError("帧格式错误：没有找到帧尾")
        
        # 移除帧尾
        encoded_data = frame_data[:-1]
        
        # COBS解码
        raw_frame = self.cobs.decode(encoded_data)
        
        if len(raw_frame) < 4:
            raise ValueError("帧长度不足：至少需要4字节CRC")
        
        # 分离payload和CRC
        payload = raw_frame[:-4]
        received_crc = int.from_bytes(raw_frame[-4:], byteorder='little')
        
        # 验证CRC32
        if not self.crc32.verify(payload, received_crc):
            raise ValueError("CRC32校验失败")
        
        return payload
