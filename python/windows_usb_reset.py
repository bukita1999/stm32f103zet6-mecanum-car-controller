#!/usr/bin/env python3
"""
Windows USB设备重置工具
专门针对STM32 USB CDC设备的连接问题
"""

import subprocess
import time
import serial.tools.list_ports

def find_stm32_device():
    """查找STM32设备"""
    print("🔍 查找STM32设备...")
    
    ports = serial.tools.list_ports.comports()
    stm32_ports = []
    
    for port in ports:
        # 检查VID:PID是否匹配STM32 (0483:5740)
        if hasattr(port, 'vid') and hasattr(port, 'pid'):
            if port.vid == 0x0483 and port.pid == 0x5740:
                stm32_ports.append(port)
                print(f"✅ 找到STM32设备: {port.device}")
                print(f"   描述: {port.description}")
                print(f"   VID:PID = {port.vid:04X}:{port.pid:04X}")
                if hasattr(port, 'location'):
                    print(f"   USB位置: {port.location}")
    
    return stm32_ports

def disable_enable_device(device_instance_id):
    """禁用并重新启用USB设备"""
    try:
        print(f"🔧 正在禁用设备...")
        # 禁用设备
        cmd_disable = f'pnputil /disable-device "{device_instance_id}"'
        result = subprocess.run(cmd_disable, shell=True, capture_output=True, text=True)
        
        if result.returncode == 0:
            print("✅ 设备已禁用")
            time.sleep(3)  # 等待3秒
            
            print("🔧 正在启用设备...")
            # 启用设备
            cmd_enable = f'pnputil /enable-device "{device_instance_id}"'
            result = subprocess.run(cmd_enable, shell=True, capture_output=True, text=True)
            
            if result.returncode == 0:
                print("✅ 设备已重新启用")
                return True
            else:
                print(f"❌ 启用设备失败: {result.stderr}")
        else:
            print(f"❌ 禁用设备失败: {result.stderr}")
            
    except Exception as e:
        print(f"❌ 设备重置失败: {e}")
    
    return False

def get_device_instance_id(vid, pid):
    """获取设备实例ID"""
    try:
        # 使用PowerShell获取设备实例ID
        cmd = f"""
        Get-PnpDevice | Where-Object {{
            $_.InstanceId -like "*VID_{vid:04X}&PID_{pid:04X}*"
        }} | Select-Object -ExpandProperty InstanceId
        """
        
        result = subprocess.run(
            ["powershell", "-Command", cmd], 
            capture_output=True, 
            text=True
        )
        
        if result.returncode == 0 and result.stdout.strip():
            instance_id = result.stdout.strip()
            print(f"📋 设备实例ID: {instance_id}")
            return instance_id
            
    except Exception as e:
        print(f"❌ 获取设备ID失败: {e}")
    
    return None

def manual_reset_instructions():
    """显示手动重置说明"""
    print("\n🛠️  手动设备管理器重置方法:")
    print("1. 按 Win + X，选择'设备管理器'")
    print("2. 展开'端口(COM和LPT)'")
    print("3. 找到'STMicroelectronics Virtual COM Port (COM4)'")
    print("4. 右键点击 → 禁用设备 → 等待3秒 → 启用设备")
    print("5. 或者右键点击 → 卸载设备 → 重新插拔USB线")

def test_connection_after_reset():
    """重置后测试连接"""
    print("\n⏳ 等待设备稳定...")
    time.sleep(5)
    
    print("🧪 测试连接...")
    try:
        import sys
        sys.path.append('.')
        from test_port_connection import test_port_connection
        
        # 查找新的端口
        stm32_devices = find_stm32_device()
        if stm32_devices:
            port = stm32_devices[0].device
            print(f"📍 测试端口: {port}")
            return test_port_connection(port)
        else:
            print("❌ 未找到STM32设备")
            return False
            
    except ImportError:
        print("⚠️  无法导入测试模块，请手动运行 test_port_connection.py")
        return None

def main():
    print("🔄 STM32 USB设备重置工具")
    print("=" * 50)
    
    # 查找STM32设备
    stm32_devices = find_stm32_device()
    
    if not stm32_devices:
        print("❌ 未找到STM32设备 (VID:0483, PID:5740)")
        print("\n🔧 可能的原因:")
        print("1. 设备未连接")
        print("2. 驱动程序未正确安装")
        print("3. 设备处于DFU模式或其他状态")
        return
    
    device = stm32_devices[0]
    print(f"\n🎯 目标设备: {device.device}")
    
    # 尝试自动重置
    print("\n方法1: 自动设备重置")
    if hasattr(device, 'vid') and hasattr(device, 'pid'):
        instance_id = get_device_instance_id(device.vid, device.pid)
        
        if instance_id:
            print("⚠️  需要管理员权限执行设备重置")
            user_choice = input("是否尝试自动重置? (需要管理员权限) [y/N]: ").strip().lower()
            
            if user_choice == 'y':
                if disable_enable_device(instance_id):
                    success = test_connection_after_reset()
                    if success:
                        print("\n🎉 设备重置成功！连接正常!")
                        return
                    elif success is False:
                        print("\n⚠️  设备重置完成，但连接仍有问题")
                else:
                    print("\n⚠️  自动重置失败")
    
    # 显示手动方法
    print("\n方法2: 手动设备管理器重置")
    manual_reset_instructions()
    
    print("\n方法3: USB物理重置")
    print("1. 断开STM32的USB连接")
    print("2. 等待10秒")
    print("3. 重新连接USB线")
    
    # 等待用户手动操作
    input("\n按回车键继续测试连接...")
    test_connection_after_reset()

if __name__ == "__main__":
    main()
