#!/usr/bin/env python3
"""
简单的串口连接测试脚本
用于诊断COM4连接问题
"""

import serial
import time
import sys

def test_port_connection(port='COM4', baudrate=115200):
    """测试串口连接"""
    print(f"🔍 测试串口连接: {port}")
    print("=" * 40)
    
    try:
        # 尝试打开串口
        print(f"⏳ 正在尝试打开 {port}...")
        
        # Windows平台需要独占访问
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=1
        )
        
        print(f"✅ 成功打开 {port}")
        print(f"   波特率: {ser.baudrate}")
        print(f"   超时时间: {ser.timeout}")
        print(f"   是否打开: {ser.is_open}")
        
        # 尝试读取数据
        print("⏳ 等待数据...")
        time.sleep(2)
        
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            print(f"📨 收到 {len(data)} 字节数据: {data.hex()}")
        else:
            print("📭 没有收到数据")
        
        # 关闭连接
        ser.close()
        print("✅ 连接测试完成")
        return True
        
    except serial.SerialException as e:
        print(f"❌ 串口异常: {e}")
        
        # 详细错误分析
        error_str = str(e).lower()
        if "access is denied" in error_str or "permission" in error_str:
            print("🔧 可能的解决方案:")
            print("   1. 关闭其他占用串口的程序")
            print("   2. 重新拔插USB线")
            print("   3. 重启设备或电脑")
            
        elif "cannot configure port" in error_str:
            print("🔧 可能的解决方案:")
            print("   1. 检查设备管理器中的设备状态")
            print("   2. 更新或重新安装驱动程序")
            print("   3. 尝试不同的USB端口")
            
        return False
        
    except Exception as e:
        print(f"💥 未知错误: {e}")
        return False

if __name__ == "__main__":
    # 测试默认端口
    success = test_port_connection()
    
    if not success:
        print("\n🛠️  故障排除建议:")
        print("1. 检查Windows设备管理器")
        print("2. 确认STM32设备正常工作")
        print("3. 尝试重启测试程序")
        sys.exit(1)
