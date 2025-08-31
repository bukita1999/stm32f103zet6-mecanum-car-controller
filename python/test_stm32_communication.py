
"""
STM32小车USB通讯测试程序
实现异步测试功能，包括遥测数据采集和电机控制测试
"""

import asyncio
import logging
from datetime import datetime
from pathlib import Path
from typing import List

from usb_communication import USBCommunication, USBConfig
from tlv_protocol import TelemetryFrame
from csv_exporter import CSVExporter

class STM32TestSuite:
    """STM32通讯测试套件"""
    
    def __init__(self, port: str = 'COM3'):
        self.config = USBConfig(port=port)
        self.comm = USBCommunication(self.config)
        self.exporter = CSVExporter()
        
        # 设置日志
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s'
        )
        self.logger = logging.getLogger(__name__)
        
        # 创建输出目录
        self.output_dir = Path("test_results")
        self.output_dir.mkdir(exist_ok=True)
    
    async def setup(self) -> bool:
        """初始化连接"""
        self.logger.info("开始初始化USB连接...")
        
        if await self.comm.connect():
            self.logger.info("✅ USB连接成功")
            # 等待系统稳定
            await asyncio.sleep(2.0)
            return True
        else:
            self.logger.error("❌ USB连接失败")
            return False
    
    async def cleanup(self):
        """清理资源"""
        await self.comm.disconnect()
        self.logger.info("测试清理完成")
    
    async def test_1_baseline_telemetry(self) -> List[TelemetryFrame]:
        """测试1: 采集基线遥测数据（连续5帧）"""
        self.logger.info("📊 开始测试1: 采集基线遥测数据")
        
        try:
            # 等待5帧遥测数据，超时时间15秒
            telemetry_frames = await self.comm.wait_for_telemetry(count=5, timeout=15.0)
            
            # 导出到CSV
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = self.output_dir / f"baseline_telemetry_{timestamp}.csv"
            
            self.exporter.export_telemetry_frames(
                telemetry_frames, 
                str(filename),
                "基线遥测数据 - 电机在默认状态下的5帧数据"
            )
            
            # 打印摘要信息
            self._print_telemetry_summary(telemetry_frames, "基线数据")
            
            self.logger.info("✅ 测试1完成")
            return telemetry_frames
            
        except Exception as e:
            self.logger.error(f"❌ 测试1失败: {e}")
            raise
    
    async def test_2_speed_control(self) -> List[TelemetryFrame]:
        """测试2: 电机速度控制测试"""
        self.logger.info("🎮 开始测试2: 电机速度控制测试")
        
        try:
            # 设置电机速度: [239, 342, -321, -395]
            target_speeds = [239, 342, -321, -395]
            self.logger.info(f"设置目标速度: {target_speeds}")
            
            await self.comm.set_motor_speeds(target_speeds)
            
            # 等待系统响应
            await asyncio.sleep(3.0)
            
            # 采集遥测数据
            telemetry_frames = await self.comm.wait_for_telemetry(count=10, timeout=20.0)
            
            # 导出到CSV
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = self.output_dir / f"speed_control_test_{timestamp}.csv"
            
            self.exporter.export_telemetry_frames(
                telemetry_frames,
                str(filename),
                f"速度控制测试 - 目标速度: {target_speeds}"
            )
            
            # 打印摘要信息
            self._print_telemetry_summary(telemetry_frames, "速度控制测试")
            self._print_speed_tracking_analysis(telemetry_frames, target_speeds)
            
            self.logger.info("✅ 测试2完成")
            return telemetry_frames
            
        except Exception as e:
            self.logger.error(f"❌ 测试2失败: {e}")
            raise
    
    def _print_telemetry_summary(self, frames: List[TelemetryFrame], test_name: str):
        """打印遥测数据摘要"""
        if not frames:
            return
        
        print(f"\n📈 {test_name} - 数据摘要:")
        print("=" * 60)
        print(f"数据帧数: {len(frames)}")
        print(f"时间跨度: {frames[0].timestamp.strftime('%H:%M:%S')} 到 {frames[-1].timestamp.strftime('%H:%M:%S')}")
        
        # 统计各电机数据
        for motor_id in range(4):
            motor_data = []
            for frame in frames:
                if motor_id < len(frame.motors):
                    motor_data.append(frame.motors[motor_id])
            
            if motor_data:
                target_speeds = [m.target_speed for m in motor_data]
                current_speeds = [m.current_speed for m in motor_data]
                pwm_values = [m.pwm_percent for m in motor_data]
                
                print(f"\n电机 {motor_id}:")
                print(f"  目标速度: {target_speeds[0]} → {target_speeds[-1]} RPM")
                print(f"  当前速度: {min(current_speeds)} ~ {max(current_speeds)} RPM (平均: {sum(current_speeds)/len(current_speeds):.1f})")
                print(f"  PWM范围: {min(pwm_values)} ~ {max(pwm_values)}% (平均: {sum(pwm_values)/len(pwm_values):.1f})")
    
    def _print_speed_tracking_analysis(self, frames: List[TelemetryFrame], target_speeds: List[int]):
        """打印速度跟踪分析"""
        print(f"\n🎯 速度跟踪性能分析:")
        print("-" * 40)
        
        for motor_id in range(4):
            motor_data = []
            for frame in frames:
                if motor_id < len(frame.motors):
                    motor_data.append(frame.motors[motor_id])
            
            if motor_data and motor_id < len(target_speeds):
                target = target_speeds[motor_id]
                actual_speeds = [m.current_speed for m in motor_data]
                final_speed = actual_speeds[-1] if actual_speeds else 0
                
                # 计算跟踪误差
                tracking_error = abs(final_speed - target)
                tracking_accuracy = max(0, 100 - (tracking_error / max(abs(target), 1)) * 100)
                
                print(f"电机 {motor_id}: 目标={target:+4d}, 实际={final_speed:+4d}, "
                      f"误差={tracking_error:3.0f}, 精度={tracking_accuracy:5.1f}%")
    
    async def run_all_tests(self):
        """运行所有测试"""
        self.logger.info("🚀 开始运行STM32通讯测试套件")
        
        try:
            # 初始化
            if not await self.setup():
                return False
            
            # 运行测试1: 基线遥测数据
            baseline_frames = await self.test_1_baseline_telemetry()
            
            # 等待间隔
            await asyncio.sleep(2.0)
            
            # 运行测试2: 速度控制测试
            control_frames = await self.test_2_speed_control()
            
            # 生成对比报告
            self._generate_comparison_report()
            
            self.logger.info("🎉 所有测试完成!")
            return True
            
        except Exception as e:
            self.logger.error(f"💥 测试过程中发生错误: {e}")
            return False
            
        finally:
            await self.cleanup()
    
    def _generate_comparison_report(self):
        """生成对比报告"""
        try:
            # 查找最新的CSV文件
            baseline_files = list(self.output_dir.glob("baseline_telemetry_*.csv"))
            control_files = list(self.output_dir.glob("speed_control_test_*.csv"))
            
            if baseline_files and control_files:
                baseline_file = max(baseline_files, key=lambda f: f.stat().st_mtime)
                control_file = max(control_files, key=lambda f: f.stat().st_mtime)
                
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                report_file = self.output_dir / f"comparison_report_{timestamp}.txt"
                
                self.exporter.create_comparison_report(
                    str(baseline_file),
                    str(control_file),
                    str(report_file)
                )
                
        except Exception as e:
            self.logger.warning(f"生成对比报告失败: {e}")

async def main():
    """主函数"""
    print("🤖 STM32小车USB通讯测试程序")
    print("=" * 50)
    
    # 获取串口号
    port = input("请输入串口号 (默认: COM3): ").strip()
    if not port:
        port = "COM3"
    
    # 创建测试套件
    test_suite = STM32TestSuite(port=port)
    
    try:
        # 运行测试
        success = await test_suite.run_all_tests()
        
        if success:
            print("\n✅ 测试完成! 请查看 test_results 目录中的CSV文件和报告。")
        else:
            print("\n❌ 测试失败，请检查连接和设备状态。")
            
    except KeyboardInterrupt:
        print("\n⏹️  测试被用户中断")
        await test_suite.cleanup()
    except Exception as e:
        print(f"\n💥 程序异常: {e}")
        await test_suite.cleanup()

if __name__ == "__main__":
    # 运行异步主函数
    asyncio.run(main())
