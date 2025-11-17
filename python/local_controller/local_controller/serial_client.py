"""Serial helpers for sending commands to the STM32."""
from __future__ import annotations

import threading
import time
from typing import Iterable, List

import serial

from .config import SerialPortConfig


class SerialCommandClient:
    """Wrapper around pyserial with minimal locking."""

    def __init__(self, cfg: SerialPortConfig) -> None:
        self._cfg = cfg
        self._lock = threading.Lock()
        self._serial = serial.Serial(
            port=cfg.port,
            baudrate=cfg.baudrate,
            timeout=cfg.timeout,
            write_timeout=cfg.timeout,
        )

    def close(self) -> None:
        with self._lock:
            if self._serial.is_open:
                self._serial.close()

    def send_speeds(self, speeds: Iterable[int]) -> None:
        values: List[str] = [str(int(v)) for v in speeds]
        if len(values) != 4:
            raise ValueError("Exactly four wheel speeds are required")
        payload = "$SPD," + ",".join(values) + "#"
        data = payload.encode("ascii")
        with self._lock:
            self._serial.write(data)
            self._serial.flush()

    def drain_input(self) -> bytes:
        with self._lock:
            available = self._serial.in_waiting
            if available:
                return self._serial.read(available)
            return b""

    def wait(self, seconds: float) -> None:
        """Utility to sleep while still allowing ctrl+c to stop gracefully."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            time.sleep(min(0.05, end - time.monotonic()))
