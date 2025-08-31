#!/usr/bin/env python3
"""
串口诊断工具 - 检测可用串口和连接状态
"""

import serial
import serial.tools.list_ports
import time
import sys

def list_available_ports():
    """列出所有可用串口"""
    print("🔍 扫描可用串口...")
    ports = serial.tools.list_ports.comports()
    
    if not ports:
        print("❌ 没有找到任何串口设备")
        return []
    
    print(f"✅ 找到 {len(ports)} 个串口设备:")
    for i, port in enumerate(ports):
        print(f"  {i+1}. {port.device}")
        print(f"     描述: {port.description}")
        print(f"     硬件ID: {port.hwid}")
        if hasattr(port, 'manufacturer') and port.manufacturer:
            print(f"     制造商: {port.manufacturer}")
        print()
    
    return [port.device for port in ports]

def test_port_basic_access(port_name):
    """测试串口基本访问"""
    print(f"🔧 测试串口 {port_name} 基本访问...")
    
    try:
        # 尝试打开串口（不设置DTR）
        ser = serial.Serial()
        ser.port = port_name
        ser.baudrate = 115200
        ser.timeout = 1
        ser.dtr = False  # 不设置DTR
        ser.rts = False  # 不设置RTS
        
        ser.open()
        print(f"✅ 串口 {port_name} 打开成功")
        
        # 检查串口状态
        print(f"   波特率: {ser.baudrate}")
        print(f"   DTR状态: {ser.dtr}")
        print(f"   RTS状态: {ser.rts}")
        print(f"   CTS状态: {ser.cts}")
        print(f"   DSR状态: {ser.dsr}")
        
        ser.close()
        return True
        
    except PermissionError as e:
        print(f"❌ 权限错误: {e}")
        return False
    except serial.SerialException as e:
        print(f"❌ 串口错误: {e}")
        return False
    except Exception as e:
        print(f"❌ 未知错误: {e}")
        return False

def test_port_with_dtr(port_name):
    """测试串口DTR控制"""
    print(f"🎯 测试串口 {port_name} DTR控制...")
    
    try:
        ser = serial.Serial()
        ser.port = port_name
        ser.baudrate = 115200
        ser.timeout = 1
        
        ser.open()
        
        # 测试DTR控制
        print("   设置DTR=False...")
        ser.dtr = False
        time.sleep(0.5)
        
        print("   设置DTR=True...")
        ser.dtr = True
        time.sleep(0.5)
        
        print("   设置RTS=True...")
        ser.rts = True
        time.sleep(0.5)
        
        print("✅ DTR/RTS控制测试完成")
        print("   💡 如果STM32板上PE5 LED有闪烁，说明USB CDC工作正常")
        
        ser.close()
        return True
        
    except Exception as e:
        print(f"❌ DTR测试失败: {e}")
        return False

def main():
    print("🔬 STM32 USB CDC 串口诊断工具")
    print("=" * 50)
    
    # 列出可用串口
    available_ports = list_available_ports()
    
    if not available_ports:
        print("请检查:")
        print("1. STM32设备是否已连接")
        print("2. USB线缆是否正常")
        print("3. 设备驱动是否正确安装")
        return
    
    # 让用户选择端口
    print("请选择要测试的串口:")
    for i, port in enumerate(available_ports):
        print(f"  {i+1}. {port}")
    
    try:
        choice = input("\n输入端口编号 (回车选择第一个): ").strip()
        if not choice:
            port_to_test = available_ports[0]
        else:
            port_to_test = available_ports[int(choice) - 1]
    except (ValueError, IndexError):
        print("❌ 无效选择")
        return
    
    print(f"\n🧪 开始测试串口: {port_to_test}")
    print("-" * 30)
    
    # 基本访问测试
    if not test_port_basic_access(port_to_test):
        print("\n❌ 基本访问测试失败")
        print("可能的解决方案:")
        print("1. 检查是否有其他程序占用该串口")
        print("2. 尝试重新插拔USB线缆")
        print("3. 重启STM32设备")
        return
    
    print()
    
    # DTR控制测试
    if test_port_with_dtr(port_to_test):
        print("\n✅ 所有测试通过!")
        print("\n💡 建议:")
        print("1. 观察STM32板上PE5 LED是否在DTR测试时闪烁")
        print("2. 如果LED闪烁，说明USB CDC驱动工作正常")
        print("3. 如果LED不闪烁，可能需要检查STM32代码")
        
        # 询问是否运行通讯测试
        run_comm_test = input("\n是否运行完整通讯测试? (y/N): ").strip().lower()
        if run_comm_test == 'y':
            print(f"\n🚀 启动通讯测试...")
            print(f"python test_stm32_communication.py")
            print(f"提示：在测试程序中输入端口号: {port_to_test}")

if __name__ == "__main__":
    main()
