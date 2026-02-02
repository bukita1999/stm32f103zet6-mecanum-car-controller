"""Predefined sequence mode."""
from __future__ import annotations

import csv
import logging
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence

from ..serial_client import SerialCommandClient
from .base import ControlMode

logger = logging.getLogger(__name__)


@dataclass
class SequenceCommand:
    timestamp_ms: int
    speeds: List[int]


def _parse_timestamp_ms(raw: str) -> int:
    """Parse timestamp as integer milliseconds."""
    return int(raw)


def load_sequence_commands(csv_path: Path) -> List["SequenceCommand"]:
    """Parse commands from a CSV file."""
    commands: List[SequenceCommand] = []
    with csv_path.open("r", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        required = ["time_ms", "m0", "m1", "m2", "m3"]
        if reader.fieldnames is None:
            raise ValueError("CSV must contain a header row")
        missing = [field for field in required if field not in reader.fieldnames]
        if missing:
            raise ValueError(f"CSV is missing fields: {missing}")
        for row in reader:
            if (row.get("time_ms") or "").strip() == "time_ms":
                logger.debug("Skipping duplicate header row: %s", row)
                continue
            try:
                timestamp_ms = _parse_timestamp_ms(row["time_ms"])
                speeds = [int(row[f"m{i}"]) for i in range(4)]
            except (TypeError, ValueError) as exc:
                logger.warning("Skipping invalid row %s: %s", row, exc)
                continue
            commands.append(SequenceCommand(timestamp_ms=timestamp_ms, speeds=speeds))
    commands.sort(key=lambda cmd: cmd.timestamp_ms)
    return commands


def create_sequence_csv_interactive(data_dir: Path) -> Path:
    """Interactive builder for a sequence CSV file."""
    _ensure_interactive_terminal()
    output_dir = _select_output_dir(data_dir)
    output_path = _prompt_output_filename(output_dir)
    step_ms = _prompt_step_ms(default_ms=500)
    first_speeds = _prompt_first_speeds()

    rows: List[SequenceCommand] = [
        SequenceCommand(timestamp_ms=0, speeds=[0, 0, 0, 0]),
        SequenceCommand(timestamp_ms=1000, speeds=first_speeds),
    ]
    prev_time_ms = 1000
    prev_speeds = first_speeds

    while True:
        next_time_ms = _prompt_next_time_ms(prev_time_ms, step_ms)
        if next_time_ms is None:
            if _is_stop_speeds(prev_speeds):
                break
            print("Last speeds must be 0,0,0,0. Please add a stop node before exiting.")
            continue
        next_speeds = _prompt_speeds()
        rows.extend(
            _interpolate_commands(
                prev_time_ms,
                prev_speeds,
                next_time_ms,
                next_speeds,
                step_ms,
            )
        )
        rows.append(SequenceCommand(timestamp_ms=next_time_ms, speeds=next_speeds))
        prev_time_ms = next_time_ms
        prev_speeds = next_speeds

    _write_sequence_csv(output_path, rows)
    return output_path


def _ensure_interactive_terminal() -> None:
    if not sys.stdin.isatty():
        raise RuntimeError("Interactive mode requires a terminal")


def _select_output_dir(data_dir: Path) -> Path:
    if not data_dir.exists():
        raise FileNotFoundError(f"Data directory not found: {data_dir}")
    if not data_dir.is_dir():
        raise NotADirectoryError(f"Data path is not a directory: {data_dir}")
    directories = [data_dir]
    directories.extend(path for path in data_dir.rglob("*") if path.is_dir())
    directories.sort(key=lambda path: str(path.relative_to(data_dir)))
    print("Select output directory under data/:")
    for idx, path in enumerate(directories, start=1):
        label = _format_relative_dir(path, data_dir)
        print(f"  {idx}. {label}")
    while True:
        choice = input("Enter number (or q to quit): ").strip().lower()
        if choice in {"q", "quit", "exit"}:
            raise RuntimeError("Sequence creation cancelled")
        try:
            index = int(choice)
        except ValueError:
            print("Please enter a valid number.")
            continue
        if 1 <= index <= len(directories):
            return directories[index - 1]
        print("Selection out of range.")


def _format_relative_dir(path: Path, base: Path) -> str:
    if path == base:
        return "."
    return str(path.relative_to(base))


def _prompt_output_filename(output_dir: Path) -> Path:
    while True:
        raw_name = input("Enter output filename (or q to quit): ").strip()
        if raw_name.lower() in {"q", "quit", "exit"}:
            raise RuntimeError("Sequence creation cancelled")
        if not raw_name:
            print("Filename cannot be empty.")
            continue
        if Path(raw_name).name != raw_name:
            print("Please enter a filename without path separators.")
            continue
        filename = raw_name if raw_name.lower().endswith(".csv") else f"{raw_name}.csv"
        output_path = output_dir / filename
        if output_path.exists():
            while True:
                confirm = input(f"{filename} exists. Overwrite? (y/n): ").strip().lower()
                if confirm in {"y", "yes"}:
                    return output_path
                if confirm in {"n", "no"}:
                    break
                print("Please enter y or n.")
            continue
        return output_path


def _prompt_step_ms(default_ms: int) -> int:
    while True:
        raw = input(f"Enter step_ms (default {default_ms}): ").strip()
        if raw == "":
            return default_ms
        try:
            value = int(raw)
        except ValueError:
            print("step_ms must be an integer.")
            continue
        if value <= 0:
            print("step_ms must be a positive integer.")
            continue
        return value


def _prompt_first_speeds() -> List[int]:
    print("Select speeds for 1000ms:")
    return _prompt_speeds()


def _prompt_next_time_ms(prev_time_ms: int, step_ms: int) -> Optional[int]:
    while True:
        raw = input("Enter next time in ms (or q to finish): ").strip().lower()
        if raw in {"q", "quit", "exit"}:
            return None
        try:
            value = int(raw)
        except ValueError:
            print("Time must be an integer in ms.")
            continue
        if value <= prev_time_ms:
            print(f"Time must be greater than {prev_time_ms}ms.")
            continue
        if value % step_ms != 0:
            print(f"Time must be a multiple of {step_ms}ms.")
            continue
        return value


def _is_stop_speeds(speeds: Sequence[int]) -> bool:
    return all(speed == 0 for speed in speeds)


def _prompt_speeds() -> List[int]:
    while True:
        raw = input("Enter speeds m0 m1 m2 m3: ").strip()
        if not raw:
            print("Please enter four integer speeds.")
            continue
        parts = raw.replace(",", " ").split()
        if len(parts) != 4:
            print("Please enter exactly four values.")
            continue
        try:
            speeds = [int(part) for part in parts]
        except ValueError:
            print("All speeds must be integers.")
            continue
        error = _validate_speed_signs(speeds)
        if error is not None:
            print(error)
            continue
        return speeds


def _validate_speed_signs(speeds: Sequence[int]) -> Optional[str]:
    value = speeds[0]
    if value != 0 and value >= 0:
        return "m0 must be < 0 (or 0)."
    value = speeds[1]
    if value != 0 and value <= 0:
        return "m1 must be > 0 (or 0)."
    value = speeds[2]
    if value != 0 and value <= 0:
        return "m2 must be > 0 (or 0)."
    value = speeds[3]
    if value != 0 and value >= 0:
        return "m3 must be < 0 (or 0)."
    return None


def _interpolate_commands(
    prev_time_ms: int,
    prev_speeds: Sequence[int],
    next_time_ms: int,
    next_speeds: Sequence[int],
    step_ms: int,
) -> List[SequenceCommand]:
    if next_time_ms <= prev_time_ms:
        return []
    interval_ms = next_time_ms - prev_time_ms
    commands: List[SequenceCommand] = []
    current_time = prev_time_ms + step_ms
    while current_time < next_time_ms:
        ratio = (current_time - prev_time_ms) / interval_ms
        interpolated = [
            _round_half_up(
                prev + (next_value - prev) * ratio,
            )
            for prev, next_value in zip(prev_speeds, next_speeds)
        ]
        commands.append(SequenceCommand(timestamp_ms=current_time, speeds=interpolated))
        current_time += step_ms
    return commands


def _round_half_up(value: float) -> int:
    if value >= 0:
        return int(math.floor(value + 0.5))
    return int(math.ceil(value - 0.5))


def _write_sequence_csv(csv_path: Path, commands: Sequence[SequenceCommand]) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["time_ms", "m0", "m1", "m2", "m3"])
        for cmd in commands:
            writer.writerow([cmd.timestamp_ms, *cmd.speeds])


class SequenceMode(ControlMode):
    def __init__(
        self,
        client: SerialCommandClient,
        csv_path: Path,
        preloaded_commands: List[SequenceCommand] | None = None,
    ) -> None:
        super().__init__(client)
        self._csv_path = csv_path
        self._preloaded_commands = preloaded_commands

    def run(self) -> None:
        commands = self._load_commands()
        if not commands:
            logger.warning("No commands found in %s", self._csv_path)
            return
        logger.info("Starting sequence with %d steps", len(commands))
        start = time.monotonic()
        for cmd in commands:
            now_ms = (time.monotonic() - start) * 1000.0
            delay_ms = cmd.timestamp_ms - now_ms
            if delay_ms > 0:
                delay_s = delay_ms / 1000.0
                logger.debug("Waiting %.3fs before next command", delay_s)
                time.sleep(delay_s)
            try:
                self._client.send_speeds(cmd.speeds)
            except Exception as exc:  # pylint: disable=broad-except
                logger.error("Failed to send speeds %s: %s", cmd.speeds, exc)
        logger.info("Sequence finished. Sending stop command.")
        self._client.send_speeds([0, 0, 0, 0])

    def _load_commands(self) -> List[SequenceCommand]:
        if self._preloaded_commands is not None:
            return list(self._preloaded_commands)
        return load_sequence_commands(self._csv_path)
