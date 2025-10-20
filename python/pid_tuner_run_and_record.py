#!/usr/bin/env python3
"""
PID tuning run-and-record script

Workflow:
- Wait for SPACE to start a forward run
- Run ~5s, then send stop; record text telemetry to CSV
- Plot Target vs Current speeds for 4 motors; save and show image

Notes:
- Uses robot_config.yaml for motion serial + commands (same as keyboard_robot_control.py)
- Receiver port defaults to 'COM11' (override with --recv-port or RECV_PORT)
- Reuses STM32BatchDataReceiver (text mode parsing of Motor lines)
"""

import os
import time
import glob
import argparse
import msvcrt  # Windows keyboard input
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime

from batch_data_receiver import STM32BatchDataReceiver
from keyboard_robot_control import RobotController
from config_loader import RobotConfigLoader


def find_latest_csv(prefix: str = "stm32_text_data_", suffix: str = ".csv") -> str | None:
    pattern = f"{prefix}*{suffix}"
    files = glob.glob(pattern)
    if not files:
        return None
    files.sort(key=lambda p: os.path.getmtime(p), reverse=True)
    return files[0]


def wait_for_space_to_start() -> bool:
    print("\nReady: press SPACE to start, ESC to cancel...")
    while True:
        if msvcrt.kbhit():
            ch = msvcrt.getch()
            if not ch:
                continue
            if ch == b' ':
                print("Starting run...")
                return True
            # ESC
            if ch == b'\x1b':
                print("Cancelled.")
                return False
        time.sleep(0.02)


def collect_for_duration(receiver: STM32BatchDataReceiver, duration_s: float = 5.0):
    start = time.time()
    if receiver.csv_writer is None:
        receiver.create_csv_file()
    print(f"Collecting for ~{duration_s:.1f}s...")
    while (time.time() - start) < duration_s:
        try:
            if receiver.serial and receiver.serial.in_waiting > 0:
                data = receiver.serial.read(receiver.serial.in_waiting)
                receiver.process_frame(data)
        except Exception as e:
            print(f"Collect error: {e}")
            break
        time.sleep(0.01)


def plot_target_vs_current(csv_path: str, save_png: bool = True) -> str | None:
    if not os.path.exists(csv_path):
        print(f"CSV not found: {csv_path}")
        return None
    df = pd.read_csv(csv_path)
    if df.empty:
        print("CSV has no data")
        return None

    time_col = 'timestamp'
    if time_col not in df.columns:
        print("CSV missing 'timestamp' column")
        return None

    fig, axes = plt.subplots(2, 2, figsize=(14, 9), sharex=True)
    axes = axes.flatten()
    for i in range(4):
        ax = axes[i]
        tgt = df.get(f'motor{i}_target_speed')
        cur = df.get(f'motor{i}_current_speed')
        if tgt is None or cur is None:
            ax.set_title(f'Motor{i} (no data)')
            ax.grid(True)
            continue
        ax.plot(df[time_col], tgt.abs(), label=f'M{i} Target', linestyle='--')
        ax.plot(df[time_col], cur.abs(), label=f'M{i} Current')
        ax.set_title(f'Motor{i} Target vs Current')
        ax.set_ylabel('Speed (abs)')
        ax.grid(True)
        ax.legend()
    axes[2].set_xlabel('Timestamp (ms)')
    axes[3].set_xlabel('Timestamp (ms)')
    fig.suptitle('PID run - Target vs Current (4 motors)')
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])

    out_path = None
    if save_png:
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        out_path = f'pid_tune_plot_{ts}.png'
        fig.savefig(out_path, dpi=150)
        print(f"Saved plot: {out_path}")
    plt.show()
    return out_path


def main():
    parser = argparse.ArgumentParser(description='PID run and record')
    parser.add_argument('--recv-port', default=None,
                        help='Receive COM port (override YAML)')
    parser.add_argument('--recv-baud', type=int, default=None, help='Receive baudrate (override YAML)')
    parser.add_argument('--duration', type=float, default=5.0, help='Run duration seconds (default 5.0)')
    parser.add_argument('--config', default='robot_config.yaml', help='Robot control config path')
    args = parser.parse_args()

    print("PID tuner run starting...")
    print(f"Duration: {args.duration:.1f}s")
    print(f"Config: {args.config}")

    # Load YAML for both control and receive ports
    cfg_loader = RobotConfigLoader(args.config)
    recv_cfg = cfg_loader.get_serial_receive_config() or {}

    recv_port = args.recv_port if args.recv_port else recv_cfg.get('port', os.environ.get('RECV_PORT', 'COM11'))
    recv_baud = args.recv_baud if args.recv_baud else int(recv_cfg.get('baudrate', 115200))
    recv_timeout = float(recv_cfg.get('timeout', 0.02))
    print(f"Recv: {recv_port} @ {recv_baud}")

    controller = RobotController(config_file=args.config)
    receiver = STM32BatchDataReceiver(port=recv_port, baudrate=recv_baud, timeout=recv_timeout)

    try:
        if not receiver.connect_serial():
            print("Failed to open receive port.")
            return

        if not wait_for_space_to_start():
            return

        if not controller.connect_serial():
            print("Failed to open control port.")
            return

        receiver.create_csv_file()
        controller.send_command('forward')
        collect_for_duration(receiver, duration_s=args.duration)
        controller.send_command('stop')

        time.sleep(0.2)
        if receiver.serial and receiver.serial.in_waiting > 0:
            data = receiver.serial.read(receiver.serial.in_waiting)
            receiver.process_frame(data)

    except KeyboardInterrupt:
        print("Interrupted, stopping...")
        try:
            controller.send_command('stop')
        except Exception:
            pass
    finally:
        controller.close()
        receiver.close()

    csv_path = find_latest_csv()
    if not csv_path:
        print("No CSV found; possibly no data was received.")
        return
    print(f"Plotting from: {csv_path}")
    plot_target_vs_current(csv_path, save_png=True)


if __name__ == '__main__':
    main()
