"""
CSV数据导出模块
将遥测数据导出到CSV文件
"""

import pandas as pd
from typing import List
from datetime import datetime
from pathlib import Path

from tlv_protocol import TelemetryFrame, MotorTelemetry

class CSVExporter:
    """CSV数据导出器"""
    
    @staticmethod
    def export_telemetry_frames(frames: List[TelemetryFrame], filename: str, description: str = ""):
        """导出遥测数据帧到CSV文件"""
        # 准备数据列表
        data_rows = []
        
        for frame_idx, frame in enumerate(frames):
            for motor_idx, motor in enumerate(frame.motors):
                row = {
                    'frame_index': frame_idx,
                    'frame_timestamp': frame.timestamp.isoformat(),
                    'motor_id': motor_idx,
                    'motor_timestamp': motor.timestamp.isoformat(),
                    'target_speed_rpm': motor.target_speed,
                    'current_speed_rpm': motor.current_speed,
                    'pwm_percent': motor.pwm_percent,
                    'pid_error': motor.pid_error,
                    'speed_unit': frame.speed_unit
                }
                data_rows.append(row)
        
        # 创建DataFrame
        df = pd.DataFrame(data_rows)
        
        # 确保输出目录存在
        output_path = Path(filename)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        # 导出到CSV
        df.to_csv(filename, index=False, encoding='utf-8-sig')
        
        # 创建摘要信息
        summary = CSVExporter._create_summary(frames, description)
        
        # 导出摘要到同名的_summary.txt文件
        summary_filename = filename.replace('.csv', '_summary.txt')
        with open(summary_filename, 'w', encoding='utf-8') as f:
            f.write(summary)
        
        print(f"✅ 已导出 {len(frames)} 帧遥测数据到: {filename}")
        print(f"📊 摘要信息已保存到: {summary_filename}")
        
        return df
    
    @staticmethod
    def _create_summary(frames: List[TelemetryFrame], description: str) -> str:
        """创建数据摘要"""
        if not frames:
            return "没有数据"
        
        summary_lines = [
            f"数据摘要报告",
            f"=" * 50,
            f"描述: {description}",
            f"导出时间: {datetime.now().isoformat()}",
            f"数据帧数量: {len(frames)}",
            f"时间范围: {frames[0].timestamp.isoformat()} 到 {frames[-1].timestamp.isoformat()}",
            f"",
            f"各电机数据统计:",
            f"-" * 30
        ]
        
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
                pid_errors = [m.pid_error for m in motor_data]
                
                summary_lines.extend([
                    f"",
                    f"电机 {motor_id}:",
                    f"  目标速度: 最小={min(target_speeds)}, 最大={max(target_speeds)}, 平均={sum(target_speeds)/len(target_speeds):.1f}",
                    f"  当前速度: 最小={min(current_speeds)}, 最大={max(current_speeds)}, 平均={sum(current_speeds)/len(current_speeds):.1f}",
                    f"  PWM百分比: 最小={min(pwm_values)}, 最大={max(pwm_values)}, 平均={sum(pwm_values)/len(pwm_values):.1f}",
                    f"  PID误差: 最小={min(pid_errors):.3f}, 最大={max(pid_errors):.3f}, 平均={sum(pid_errors)/len(pid_errors):.3f}"
                ])
        
        return "\n".join(summary_lines)
    
    @staticmethod
    def create_comparison_report(baseline_file: str, test_file: str, output_file: str):
        """创建对比报告"""
        try:
            baseline_df = pd.read_csv(baseline_file)
            test_df = pd.read_csv(test_file)
            
            report_lines = [
                f"对比分析报告",
                f"=" * 50,
                f"基准数据: {baseline_file}",
                f"测试数据: {test_file}",
                f"生成时间: {datetime.now().isoformat()}",
                f"",
                f"数据量对比:",
                f"  基准数据帧数: {baseline_df['frame_index'].nunique()}",
                f"  测试数据帧数: {test_df['frame_index'].nunique()}",
                f"",
                f"各电机速度对比:",
                f"-" * 30
            ]
            
            for motor_id in range(4):
                baseline_motor = baseline_df[baseline_df['motor_id'] == motor_id]
                test_motor = test_df[test_df['motor_id'] == motor_id]
                
                if not baseline_motor.empty and not test_motor.empty:
                    baseline_avg = baseline_motor['current_speed_rpm'].mean()
                    test_avg = test_motor['current_speed_rpm'].mean()
                    target_avg = test_motor['target_speed_rpm'].mean()
                    
                    report_lines.extend([
                        f"",
                        f"电机 {motor_id}:",
                        f"  基准平均速度: {baseline_avg:.1f} RPM",
                        f"  测试目标速度: {target_avg:.1f} RPM",
                        f"  测试平均速度: {test_avg:.1f} RPM",
                        f"  速度变化: {test_avg - baseline_avg:+.1f} RPM",
                        f"  目标跟踪误差: {abs(test_avg - target_avg):.1f} RPM"
                    ])
            
            # 保存报告
            with open(output_file, 'w', encoding='utf-8') as f:
                f.write("\n".join(report_lines))
            
            print(f"📈 对比报告已保存到: {output_file}")
            
        except Exception as e:
            print(f"❌ 生成对比报告失败: {e}")
