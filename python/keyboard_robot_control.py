#!/usr/bin/env python3
"""Keyboard-based robot controller for the STM32 platform."""

import argparse
import time

import msvcrt  # Windows keyboard input
import serial

from config_loader import RobotConfigLoader


class RobotController:
    """Keyboard-driven robot motion controller."""

    def __init__(self, config_file: str = "robot_config.yaml") -> None:
        """Load configuration and prepare serial settings."""
        self.config_loader = RobotConfigLoader(config_file)

        serial_cfg = self.config_loader.get_serial_control_config()
        self.port = serial_cfg.get("port", "COM10")
        self.baudrate = serial_cfg.get("baudrate", 115200)
        self.timeout = serial_cfg.get("timeout", 1.0)

        self.serial: serial.Serial | None = None
        self.is_running = False

        # Cached movement commands (for hints / validation)
        self.commands = {
            name: info.get("speeds", [0, 0, 0, 0])
            for name, info in self.config_loader.get_movement_commands().items()
        }

        self.current_command = "stop"

    def connect_serial(self) -> bool:
        """Open the configured control serial port."""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
            print(f"Connected to {self.port} @ {self.baudrate}bps")
            return True
        except serial.SerialException as exc:
            print(f"Failed to open serial port: {exc}")
            return False

    def send_command(self, command_key: str) -> bool:
        """Send the motion command specified in the YAML (built from speeds)."""
        if not self.serial or not self.serial.is_open:
            print("Serial port is not connected")
            return False

        commands_map = self.config_loader.get_movement_commands()
        if command_key not in commands_map:
            print(f"Invalid command: {command_key}")
            return False

        speeds = self.config_loader.get_command_speeds(command_key)
        command = self.config_loader.build_command_from_speeds(speeds)

        try:
            self.serial.write(command.encode("utf-8"))
            self.serial.flush()
            self.current_command = command_key
            print(f"Sent {command_key}: speeds={speeds} -> {command}")
            return True
        except Exception as exc:  # noqa: broad-except (serial errors vary)
            print(f"Failed to send command: {exc}")
            return False

    def stop_robot(self) -> None:
        """Send stop command."""
        self.send_command("stop")

    @staticmethod
    def print_control_info() -> None:
        """Display keyboard mapping."""
        print("\n" + "=" * 60)
        print("           Robot Keyboard Controller")
        print("=" * 60)
        print("Key bindings:")
        print("  W - Forward      S - Backward")
        print("  A - Turn Left    D - Turn Right")
        print("  Space - Stop     Q - Quit")
        print("=" * 60)
        print("Status: Ready. Press a control key to start...")
        print("Note: Motion continues until another command or stop is sent")
        print("=" * 60)

    def start_keyboard_control(self) -> None:
        """Run the main keyboard loop."""
        self.print_control_info()

        if not self.connect_serial():
            return

        self.is_running = True
        print("\nWaiting for keyboard input... (press Q to exit)")

        try:
            while self.is_running:
                if msvcrt.kbhit():
                    key = msvcrt.getch()
                    key_char = key.decode("utf-8", errors="ignore").lower()

                    if key_char == "q":
                        print("\nExiting controller...")
                        self.is_running = False
                        break
                    if key_char == "w":
                        self.send_command("forward")
                        print("Moving forward...")
                    elif key_char == "s":
                        self.send_command("backward")
                        print("Moving backward...")
                    elif key_char == "a":
                        self.send_command("left")
                        print("Turning left...")
                    elif key_char == "d":
                        self.send_command("right")
                        print("Turning right...")
                    elif key_char == " ":
                        self.send_command("stop")
                        print("Stopped.")

                time.sleep(0.01)

        except KeyboardInterrupt:
            print("\nKeyboard interrupt received")
        finally:
            self.stop_robot()
            if self.serial and self.serial.is_open:
                self.serial.close()
                print("Serial port closed")

    def close(self) -> None:
        """Close the serial port if open."""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Serial port closed")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Keyboard robot controller")
    parser.add_argument(
        "--config",
        default="robot_config.yaml",
        help="Path to robot configuration YAML",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    print("Robot keyboard control starting...")
    print(f"Using YAML config: {args.config}")

    controller = RobotController(config_file=args.config)

    try:
        controller.start_keyboard_control()
    except Exception as exc:  # noqa: broad-except
        print(f"Runtime error: {exc}")
    finally:
        controller.close()
        print("Program exited")


if __name__ == "__main__":  # pragma: no cover
    main()
