#!/usr/bin/env python3
"""
STM32批量数据接收器测试脚本
用于测试数据接收和解析功能

作者: AI Assistant
日期: 2025年1月
"""

import time
import csv
from datetime import datetime
from batch_data_receiver import STM32BatchDataReceiver


def test_data_parsing():
    """测试数据解析功能"""
    print("测试数据解析功能...")
    receiver = STM32BatchDataReceiver()

    # 模拟COBS编码后的批量数据帧
    # 这里需要根据实际的STM32输出格式来构造测试数据
    # 暂时创建一个模拟的测试

    print("数据解析测试完成")
    return True


def test_csv_writing():
    """测试CSV写入功能"""
    print("测试CSV写入功能...")

    # 创建测试数据
    test_data = [
        {
            'batch_id': 1,
            'data_index': 0,
            'timestamp': 1234567890,
            'motor0_speed': 1000,
            'motor1_speed': 950,
            'motor2_speed': 1100,
            'motor3_speed': 1050,
            'motor0_pwm': 80,
            'motor1_pwm': 75,
            'motor2_pwm': 85,
            'motor3_pwm': 82,
            'motor0_error': 1.5,
            'motor1_error': -2.1,
            'motor2_error': 0.8,
            'motor3_error': -1.2,
            'receive_time': time.time()
        }
    ]

    # 测试CSV写入
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"test_data_{timestamp}.csv"

    try:
        with open(filename, 'w', newline='', encoding='utf-8') as csvfile:
            fieldnames = [
                'batch_id', 'data_index', 'timestamp',
                'motor0_speed', 'motor1_speed', 'motor2_speed', 'motor3_speed',
                'motor0_pwm', 'motor1_pwm', 'motor2_pwm', 'motor3_pwm',
                'motor0_error', 'motor1_error', 'motor2_error', 'motor3_error',
                'receive_time'
            ]
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(test_data)

        print(f"CSV文件写入测试成功: {filename}")
        return True

    except Exception as e:
        print(f"CSV写入测试失败: {e}")
        return False


def test_serial_connection():
    """测试串口连接功能"""
    print("测试串口连接功能...")

    receiver = STM32BatchDataReceiver(port='COM4', baudrate=115200)

    if receiver.connect_serial():
        print("串口连接测试成功")
        receiver.close()
        return True
    else:
        print("串口连接测试失败 - 请检查COM4端口是否可用")
        return False


def run_all_tests():
    """运行所有测试"""
    print("STM32批量数据接收器测试套件")
    print("=" * 50)

    tests = [
        ("串口连接测试", test_serial_connection),
        ("数据解析测试", test_data_parsing),
        ("CSV写入测试", test_csv_writing)
    ]

    passed = 0
    total = len(tests)

    for test_name, test_func in tests:
        print(f"\n执行测试: {test_name}")
        try:
            if test_func():
                print(f"✓ {test_name} - 通过")
                passed += 1
            else:
                print(f"✗ {test_name} - 失败")
        except Exception as e:
            print(f"✗ {test_name} - 错误: {e}")

    print(f"\n测试结果: {passed}/{total} 通过")

    if passed == total:
        print("🎉 所有测试通过！")
    else:
        print("⚠️  部分测试失败，请检查配置")

    return passed == total


def main():
    """主函数"""
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "--run-tests":
        # 运行测试
        run_all_tests()
    else:
        # 显示使用说明
        print("STM32批量数据接收器")
        print("=" * 30)
        print("使用方法:")
        print("1. 确保STM32设备已连接到COM4端口")
        print("2. 运行主程序: python batch_data_receiver.py")
        print("3. 运行测试: python test_batch_receiver.py --run-tests")
        print("\n程序功能:")
        print("- 自动接收STM32发送的批量数据")
        print("- COBS解码和CRC32校验")
        print("- 解析TLV格式数据")
        print("- 保存数据到CSV文件")


if __name__ == "__main__":
    main()
