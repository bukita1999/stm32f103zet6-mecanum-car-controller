"""Predefined sequence mode."""
from __future__ import annotations

import csv
import logging
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List

from ..serial_client import SerialCommandClient
from .base import ControlMode

logger = logging.getLogger(__name__)


@dataclass
class SequenceCommand:
    timestamp: float
    speeds: List[int]


def _parse_timestamp(raw: str) -> float:
    """Parse timestamp preferring float; fall back to int."""
    try:
        return float(raw)
    except (TypeError, ValueError):
        # Try int fallback for whole-number strings like "5"
        return float(int(raw))


def load_sequence_commands(csv_path: Path) -> List["SequenceCommand"]:
    """Parse commands from a CSV file."""
    commands: List[SequenceCommand] = []
    with csv_path.open("r", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        required = ["time_s", "m0", "m1", "m2", "m3"]
        if reader.fieldnames is None:
            raise ValueError("CSV must contain a header row")
        missing = [field for field in required if field not in reader.fieldnames]
        if missing:
            raise ValueError(f"CSV is missing fields: {missing}")
        for row in reader:
            if (row.get("time_s") or "").strip() == "time_s":
                logger.debug("Skipping duplicate header row: %s", row)
                continue
            try:
                timestamp = _parse_timestamp(row["time_s"])
                speeds = [int(row[f"m{i}"]) for i in range(4)]
            except (TypeError, ValueError) as exc:
                logger.warning("Skipping invalid row %s: %s", row, exc)
                continue
            commands.append(SequenceCommand(timestamp=timestamp, speeds=speeds))
    commands.sort(key=lambda cmd: cmd.timestamp)
    return commands


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
            now = time.monotonic() - start
            delay = cmd.timestamp - now
            if delay > 0:
                logger.debug("Waiting %.2fs before next command", delay)
                time.sleep(delay)
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
