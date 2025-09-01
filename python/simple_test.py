#!/usr/bin/env python3
"""
简化的STM32批量数据接收器测试脚本
用于快速测试数据接收功能

使用方法:
python simple_test.py
"""

import serial
import time
import csv
from datetime import datetime


def test_serial_connection():
    """测试串口连接"""
    print("🔌 测试串口连接...")
    try:
        ser = serial.Serial('COM11', 115200, timeout=1)
        ser.close()
        print("✅ 串口连接成功")
        return True
    except Exception as e:
        print(f"❌ 串口连接失败: {e}")
        return False


def test_csv_writing():
    """测试CSV写入"""
    print("📝 测试CSV写入...")
    try:
        filename = f"test_{int(time.time())}.csv"
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'motor0_speed', 'motor1_speed'])
            writer.writerow([time.time(), 1000, 950])
        print(f"✅ CSV写入成功: {filename}")
        return True
    except Exception as e:
        print(f"❌ CSV写入失败: {e}")
        return False


def run_quick_test():
    """运行快速测试"""
    print("🚀 STM32批量数据接收器 - 快速测试")
    print("=" * 40)

    tests_passed = 0
    total_tests = 2

    if test_serial_connection():
        tests_passed += 1

    if test_csv_writing():
        tests_passed += 1

    print(f"\n📊 测试结果: {tests_passed}/{total_tests} 通过")

    if tests_passed == total_tests:
        print("🎉 基础测试通过！可以运行主程序了")
        print("\n运行主程序:")
        print("python batch_data_receiver.py")
    else:
        print("⚠️  有测试失败，请检查配置")

    return tests_passed == total_tests


def main():
    """主函数"""
    import sys

    if len(sys.argv) > 1:
        if sys.argv[1] == "--serial":
            test_serial_connection()
        elif sys.argv[1] == "--csv":
            test_csv_writing()
        else:
            print("使用方法:")
            print("python simple_test.py          # 运行完整测试")
            print("python simple_test.py --serial # 只测试串口")
            print("python simple_test.py --csv    # 只测试CSV")
    else:
        run_quick_test()


if __name__ == "__main__":
    main()
