import tkinter as tk
from tkinter import filedialog
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


class MotorAnalyzer:
    def __init__(self):
        self.df = None
        self.filepath = None
        self.selected_points = []
        self.time_range = None

    def load_csv(self):
        # 打开文件选择对话框
        filepath = filedialog.askopenfilename(
            title="选择CSV文件",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        if not filepath:
            return False

        self.filepath = filepath
        # 读取CSV
        self.df = pd.read_csv(filepath)
        return True

    def on_click(self, event):
        """处理鼠标点击事件"""
        if event.button == 1 and event.xdata is not None:  # 左键点击
            self.selected_points.append(event.xdata)

            # 在点击位置绘制垂直线
            plt.axvline(x=event.xdata, color='red', linestyle='--', alpha=0.7)

            if len(self.selected_points) == 1:
                plt.title('电机0实际速度 - 请点击选择第二个时间点')
            elif len(self.selected_points) == 2:
                # 两个点都选择了
                self.time_range = sorted(self.selected_points)
                plt.title('.1f')
                plt.draw()

                # 关闭当前图表，显示四个电机图表
                plt.close()
                self.plot_four_motors()

    def show_motor0_speed_plot(self):
        """显示电机0的速度图用于选择时间范围"""
        if self.df is None:
            return

        time = self.df['timestamp']
        current_speed = self.df['motor0_current_speed'].abs()

        fig, ax = plt.subplots(figsize=(12, 6))
        ax.plot(time, current_speed, label='Motor0 Current Speed', color='blue')
        ax.set_title('电机0实际速度 - 请点击选择第一个时间点')
        ax.set_xlabel('Timestamp')
        ax.set_ylabel('Speed (abs)')
        ax.legend()
        ax.grid(True)

        # 连接鼠标点击事件
        fig.canvas.mpl_connect('button_press_event', self.on_click)

        plt.show()

    def plot_four_motors(self):
        """基于选择的时间范围绘制四个电机的图表"""
        if self.df is None or self.time_range is None:
            return

        # 筛选数据在选择的时间范围内
        mask = (self.df['timestamp'] >= self.time_range[0]) & (self.df['timestamp'] <= self.time_range[1])
        filtered_df = self.df[mask]

        if len(filtered_df) == 0:
            print("所选时间范围内没有数据")
            return

        time = filtered_df['timestamp']
        motor_ids = [0, 1, 2, 3]

        # 绘图
        plt.figure(figsize=(15, 10))
        for i, mid in enumerate(motor_ids):
            plt.subplot(2, 2, i+1)
            target = filtered_df[f'motor{mid}_target_speed'].abs()
            current = filtered_df[f'motor{mid}_current_speed'].abs()
            plt.plot(time, target, label=f'Motor{mid} Target Speed', linestyle='--')
            plt.plot(time, current, label=f'Motor{mid} Current Speed')
            plt.title('.1f')
            plt.xlabel('Timestamp')
            plt.ylabel('Speed (abs)')
            plt.legend()
            plt.grid(True)

        plt.tight_layout()
        plt.show()

    def start_analysis(self):
        """开始分析流程"""
        if not self.load_csv():
            return

        # 重置选择状态
        self.selected_points = []
        self.time_range = None

        # 显示电机0的速度图用于选择时间范围
        self.show_motor0_speed_plot()


def load_and_analyze():
    analyzer = MotorAnalyzer()
    analyzer.start_analysis()


# 主窗口
root = tk.Tk()
root.title("电机数据可视化 - 时间范围选择")

btn = tk.Button(root, text="选择CSV文件并开始分析", command=load_and_analyze, width=30, height=2)
btn.pack(pady=20)

# 添加说明标签
instruction_text = """
使用说明：
1. 点击按钮选择CSV文件
2. 在电机0速度图上点击两个点来选择时间范围
3. 系统将基于选择的时间范围生成四个电机的详细图表
"""
label = tk.Label(root, text=instruction_text, justify=tk.LEFT)
label.pack(pady=10)

root.mainloop()
