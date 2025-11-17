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


class SequenceMode(ControlMode):
    def __init__(self, client: SerialCommandClient, csv_path: Path) -> None:
        super().__init__(client)
        self._csv_path = csv_path

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
        commands: List[SequenceCommand] = []
        with self._csv_path.open("r", encoding="utf-8") as fh:
            reader = csv.DictReader(fh)
            required = ["time_s", "m0", "m1", "m2", "m3"]
            if reader.fieldnames is None:
                raise ValueError("CSV must contain a header row")
            missing = [field for field in required if field not in reader.fieldnames]
            if missing:
                raise ValueError(f"CSV is missing fields: {missing}")
            for row in reader:
                timestamp = float(row["time_s"])
                speeds = [int(row[f"m{i}"]) for i in range(4)]
                commands.append(SequenceCommand(timestamp=timestamp, speeds=speeds))
        commands.sort(key=lambda cmd: cmd.timestamp)
        return commands
