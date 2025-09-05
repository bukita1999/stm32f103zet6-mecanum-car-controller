import tkinter as tk
from tkinter import filedialog
import pandas as pd
import matplotlib.pyplot as plt


def load_and_plot():
    # 打开文件选择对话框
    filepath = filedialog.askopenfilename(
        title="选择CSV文件",
        filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
    )
    if not filepath:
        return

    # 读取CSV
    df = pd.read_csv(filepath)

    # 时间戳
    time = df['timestamp']
    motor_ids = [0, 1, 2, 3]

    # 绘图
    plt.figure(figsize=(15, 10))
    for i, mid in enumerate(motor_ids):
        plt.subplot(2, 2, i+1)
        target = df[f'motor{mid}_target_speed'].abs()
        current = df[f'motor{mid}_current_speed'].abs()
        plt.plot(time, target, label=f'Motor{mid} Target Speed', linestyle='--')
        plt.plot(time, current, label=f'Motor{mid} Current Speed')
        plt.title(f'Motor {mid} Speed Tracking')
        plt.xlabel('Timestamp')
        plt.ylabel('Speed (abs)')
        plt.legend()
        plt.grid(True)

    plt.tight_layout()
    plt.show()


# 主窗口
root = tk.Tk()
root.title("电机数据可视化")

btn = tk.Button(root, text="选择CSV文件并绘图", command=load_and_plot, width=30, height=2)
btn.pack(pady=20)

root.mainloop()
