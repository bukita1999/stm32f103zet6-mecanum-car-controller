#!/usr/bin/env python3
"""
Extended PID tuning run-and-record script.

Differences vs pid_tuner_run_and_record.py:
- Default duration comes from robot_config.yaml (scripts.run_and_record_full.duration_seconds, default 10s).
- Allows choosing the motion (forward by default, 1=left, 2=right) before starting.
- Captures every line of telemetry coming from the STM32, not just wheel speeds.
- Writes the raw and parsed telemetry into python/output/<timestamp>.csv.

Each CSV row keeps the original text plus any parsed fields so post-processing
scripts can decide how to use the data.
"""

import argparse
import csv
import os
import re
import time
from datetime import datetime
from pathlib import Path
from typing import IO

import msvcrt  # Windows keyboard input

from batch_data_receiver import STM32BatchDataReceiver
from keyboard_robot_control import RobotController
from config_loader import RobotConfigLoader


FIELDNAMES = [
    'sequence',
    'timestamp_ms',
    'receive_time_s',
    'line_type',
    'raw_text',
    'motor_id',
    'motor_target_speed',
    'motor_current_speed',
    'motor_pwm_percent',
    'motor_error',
    'system_init',
    'system_pca9685',
    'system_motor_error',
]

MOTOR_PATTERN = re.compile(
    r'^Motor(\d+):\s*Target:(-?\d+)\s+Current:(-?\d+)\s+RPM,\s*PWM:(\d+)%,'  # type: ignore[unicode]
    r'\s*Error:(-?\d+)\.(\d+)\s*$'
)
SYSTEM_PATTERN = re.compile(
    r'^System:\s*Init=(\d+),\s*PCA9685=(\d+),\s*MotorErr=(\d+)\s*$'
)


def wait_for_space_to_start() -> bool:
    """Prompt the user to press SPACE to start or ESC to cancel."""
    print("\nReady: press SPACE to start, ESC to cancel...")
    while True:
        if msvcrt.kbhit():
            ch = msvcrt.getch()
            if not ch:
                continue
            if ch == b' ':
                print("Starting run...")
                return True
            if ch == b'\x1b':  # ESC
                print("Cancelled.")
                return False
        time.sleep(0.02)


def prompt_motion_choice(cfg_loader: RobotConfigLoader) -> str:
    """
    Ask the user which motion command to execute before the run starts.

    Returns:
        Selected command key from movement_commands (forward by default).
    """
    movement_commands = cfg_loader.get_movement_commands()
    options = {
        '1': 'left',
        '2': 'right',
    }
    descriptions = {
        key: cfg_loader.get_command_description(cmd_key)
        for key, cmd_key in options.items()
    }

    print("\nSelect motion command before starting:")
    print("  Press ENTER for forward (default).")
    for key, cmd_key in options.items():
        desc = descriptions.get(key, cmd_key)
        speeds = movement_commands.get(cmd_key, {}).get('speeds', [])
        print(f"  Press {key} for {cmd_key} ({desc}), speeds={speeds}")

    while True:
        choice = input("Motion choice [ENTER=forward, 1=left, 2=right]: ").strip()
        if choice == '':
            print("Selected command: forward (default)")
            return 'forward'
        if choice in options:
            cmd_key = options[choice]
            if cmd_key in movement_commands:
                desc = descriptions.get(choice, cmd_key)
                speeds = movement_commands.get(cmd_key, {}).get('speeds', [])
                print(f"Selected command: {cmd_key} ({desc}), speeds={speeds}")
                return cmd_key
            print(f"Command '{cmd_key}' not defined in YAML; defaulting to forward.")
            return 'forward'
        print("Invalid selection. Please press ENTER, 1, or 2.")


def create_output_writer(output_dir: Path) -> tuple[Path, csv.DictWriter, IO[str]]:
    """Create the CSV writer under the output directory."""
    output_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = output_dir / f'stm32_full_text_data_{timestamp}.csv'
    csv_file = csv_path.open('w', newline='', encoding='utf-8')
    writer = csv.DictWriter(csv_file, fieldnames=FIELDNAMES)
    writer.writeheader()
    return csv_path, writer, csv_file


def parse_line(line: str, sequence: int, timestamp_s: float) -> dict:
    """
    Parse a single telemetry line into a record containing both raw and parsed data.

    Unrecognised lines still get stored with type 'raw' and only the raw_text field.
    """
    record = {key: None for key in FIELDNAMES}
    record['sequence'] = sequence
    record['timestamp_ms'] = int(timestamp_s * 1000)
    record['receive_time_s'] = timestamp_s
    record['line_type'] = 'raw'
    record['raw_text'] = line

    motor_match = MOTOR_PATTERN.match(line)
    if motor_match:
        motor_id = int(motor_match.group(1)) - 1
        target_speed = int(motor_match.group(2))
        current_speed = int(motor_match.group(3))
        pwm_percent = int(motor_match.group(4))
        error_integer = int(motor_match.group(5))
        error_decimal = int(motor_match.group(6))
        error_value = error_integer + error_decimal / 100.0
        if error_integer < 0:
            error_value = error_integer - error_decimal / 100.0

        record['line_type'] = 'motor'
        record['motor_id'] = motor_id
        record['motor_target_speed'] = target_speed
        record['motor_current_speed'] = current_speed
        record['motor_pwm_percent'] = pwm_percent
        record['motor_error'] = error_value
        return record

    system_match = SYSTEM_PATTERN.match(line)
    if system_match:
        record['line_type'] = 'system'
        record['system_init'] = int(system_match.group(1))
        record['system_pca9685'] = int(system_match.group(2))
        record['system_motor_error'] = int(system_match.group(3))
        return record

    return record


def collect_full_telemetry(receiver: STM32BatchDataReceiver,
                           writer: csv.DictWriter,
                           duration_s: float) -> int:
    """
    Collect telemetry lines for the requested duration and store them via writer.

    Returns:
        Total number of lines written.
    """
    serial_port = receiver.serial
    if serial_port is None:
        print("Receiver serial port is not connected.")
        return 0

    start_time = time.time()
    line_buffer = ''
    sequence = 0
    lines_written = 0

    while (time.time() - start_time) < duration_s:
        try:
            if serial_port.in_waiting > 0:
                data = serial_port.read(serial_port.in_waiting)
                if not data:
                    time.sleep(0.01)
                    continue
                line_buffer += data.decode('utf-8', errors='ignore')

                while '\n' in line_buffer:
                    line, line_buffer = line_buffer.split('\n', 1)
                    line = line.strip('\r').strip()
                    if not line:
                        continue
                    sequence += 1
                    timestamp_s = time.time()
                    record = parse_line(line, sequence, timestamp_s)
                    writer.writerow(record)
                    lines_written += 1
            time.sleep(0.01)
        except Exception as exc:  # pylint: disable=broad-except
            print(f"Telemetry collection error: {exc}")
            break

    # Flush any trailing partial line after duration expires.
    line = line_buffer.strip('\r').strip()
    if line:
        sequence += 1
        timestamp_s = time.time()
        record = parse_line(line, sequence, timestamp_s)
        writer.writerow(record)
        lines_written += 1

    return lines_written


def main():
    parser = argparse.ArgumentParser(
        description='Run PID command and capture full telemetry to CSV (duration from config unless overridden).')
    parser.add_argument('--recv-port', default=None,
                        help='Receive COM port (override YAML)')
    parser.add_argument('--recv-baud', type=int, default=None,
                        help='Receive baudrate (override YAML)')
    parser.add_argument('--duration', type=float, default=None,
                        help='Run duration seconds (override config)')
    parser.add_argument('--config', default='robot_config.yaml',
                        help='Robot control config path')
    parser.add_argument('--output-dir', default='output',
                        help='Directory (relative or absolute) for CSV output')
    args = parser.parse_args()

    print("Extended PID run starting...")
    print(f"Config: {args.config}")
    print(f"Output directory: {args.output_dir}")

    cfg_loader = RobotConfigLoader(args.config)
    recv_cfg = cfg_loader.get_serial_receive_config() or {}

    recv_port = args.recv_port if args.recv_port else recv_cfg.get('port', os.environ.get('RECV_PORT', 'COM11'))
    recv_baud = args.recv_baud if args.recv_baud else int(recv_cfg.get('baudrate', 115200))
    recv_timeout = float(recv_cfg.get('timeout', 0.02))
    print(f"Recv: {recv_port} @ {recv_baud}")

    scripts_cfg = cfg_loader.config.get('scripts', {}) if isinstance(cfg_loader.config, dict) else {}
    run_cfg = scripts_cfg.get('run_and_record_full', {}) if isinstance(scripts_cfg, dict) else {}
    config_duration = run_cfg.get('duration_seconds') if isinstance(run_cfg, dict) else None
    try:
        config_duration = float(config_duration)
    except (TypeError, ValueError):
        config_duration = None
    if config_duration is None:
        config_duration = 10.0

    run_duration = args.duration if args.duration is not None else config_duration
    if args.duration is not None:
        print(f"Duration: {run_duration:.1f}s (override)")
    else:
        print(f"Duration: {run_duration:.1f}s (from config)")

    controller = RobotController(config_file=args.config)
    receiver = STM32BatchDataReceiver(port=recv_port, baudrate=recv_baud, timeout=recv_timeout)

    selected_command = prompt_motion_choice(cfg_loader)

    csv_path = None
    csv_file = None

    try:
        if not receiver.connect_serial():
            print("Failed to open receive port.")
            return

        if not wait_for_space_to_start():
            return

        if not controller.connect_serial():
            print("Failed to open control port.")
            return

        csv_path, writer, csv_file = create_output_writer(Path(args.output_dir))
        controller.send_command(selected_command)
        collected_lines = collect_full_telemetry(receiver, writer, run_duration)
        controller.send_command('stop')
        print(f"Collected {collected_lines} telemetry lines.")

        # Drain remaining bytes briefly to capture late data.
        time.sleep(0.2)
        collected_lines += collect_full_telemetry(receiver, writer, 0.2)
        print(f"Total lines captured (including drain): {collected_lines}")

    except KeyboardInterrupt:
        print("Interrupted, stopping...")
        try:
            controller.send_command('stop')
        except Exception:
            pass
    finally:
        controller.close()
        receiver.close()
        if csv_file is not None:
            csv_file.close()

    if csv_path:
        print(f"Telemetry saved to: {csv_path}")


if __name__ == '__main__':
    main()
