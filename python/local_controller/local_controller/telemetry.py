"""Telemetry reception and logging utilities."""
from __future__ import annotations

import csv
import logging
import struct
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional

import serial

from .config import SerialPortConfig

SYNC_WORD = 0xAA55
TRAIL_WORD = 0x55AA
HEADER_FORMAT = "<HBBHI"  # sync, version, reserved, frame_length, timestamp
MOTOR_FORMAT = "<BhhH"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
MOTOR_SIZE = struct.calcsize(MOTOR_FORMAT)
MOTOR_COUNT = 4
FRAME_SIZE = HEADER_SIZE + MOTOR_COUNT * MOTOR_SIZE + 4 + 2  # + crc + trail


@dataclass
class MotorSample:
    motor_id: int
    target_rpm: int
    current_rpm: int
    pwm_percent: int


@dataclass
class TelemetryFrame:
    timestamp_ms: int
    motors: List[MotorSample]


class TelemetryDecoder:
    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, chunk: bytes) -> List[TelemetryFrame]:
        frames: List[TelemetryFrame] = []
        self._buffer.extend(chunk)

        while True:
            if len(self._buffer) < HEADER_SIZE:
                break
            sync, version, reserved, frame_length, timestamp = struct.unpack_from(
                HEADER_FORMAT, self._buffer
            )

            if sync != SYNC_WORD:
                # discard first byte until we hit sync
                self._buffer.pop(0)
                continue

            if frame_length < FRAME_SIZE:
                logger.warning(
                    "Discarding frame: length %d smaller than minimum %d",
                    frame_length,
                    FRAME_SIZE,
                )
                self._buffer.pop(0)
                continue

            if len(self._buffer) < frame_length:
                break  # wait for more data

            frame_bytes = self._buffer[:frame_length]
            trail = struct.unpack_from("<H", frame_bytes, frame_length - 2)[0]
            if trail != TRAIL_WORD:
                # bad alignment: discard sync byte
                logger.warning(
                    "Telemetry trail mismatch (0x%04X), expected 0x%04X", trail, TRAIL_WORD
                )
                self._buffer.pop(0)
                continue

            payload_end = frame_length - 6  # exclude crc (4) + trail (2)
            crc_data = frame_bytes[:payload_end]
            received_crc = struct.unpack_from("<I", frame_bytes, payload_end)[0]
            if _crc32(crc_data) != received_crc:
                logger.warning("Telemetry CRC mismatch, dropping frame")
                self._buffer.pop(0)
                continue

            offset = HEADER_SIZE
            motors: List[MotorSample] = []
            for _ in range(MOTOR_COUNT):
                m_id, target, current, pwm = struct.unpack_from(
                    MOTOR_FORMAT, frame_bytes, offset
                )
                motors.append(
                    MotorSample(
                        motor_id=m_id,
                        target_rpm=target,
                        current_rpm=current,
                        pwm_percent=pwm,
                    )
                )
                offset += MOTOR_SIZE

            frames.append(TelemetryFrame(timestamp_ms=timestamp, motors=motors))
            del self._buffer[:frame_length]
        return frames


def _crc32(data: bytes) -> int:
    # zlib.crc32 uses signed results, so mask to match firmware implementation
    import zlib

    return zlib.crc32(data) & 0xFFFFFFFF


class TelemetryLogger:
    def __init__(self, cfg: SerialPortConfig, output_dir: Path) -> None:
        self._cfg = cfg
        self._output_dir = output_dir
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._serial: Optional[serial.Serial] = None
        self._decoder = TelemetryDecoder()
        self._writer: Optional[csv.DictWriter] = None
        self._csv_file: Optional[object] = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop_event.clear()
        self._output_dir.mkdir(parents=True, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        csv_path = self._output_dir / f"telemetry_{timestamp}.csv"
        self._csv_file = csv_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(
            self._csv_file,
            fieldnames=[
                "host_time",
                "frame_timestamp_ms",
                "motor_id",
                "target_rpm",
                "current_rpm",
                "pwm_percent",
            ],
        )
        self._writer.writeheader()
        try:
            self._serial = serial.Serial(
                port=self._cfg.port,
                baudrate=self._cfg.baudrate,
                timeout=self._cfg.timeout,
            )
        except serial.SerialException:
            self._csv_file.close()
            self._csv_file = None
            self._writer = None
            raise
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
            self._thread = None
        if self._serial and self._serial.is_open:
            self._serial.close()
        if self._csv_file:
            self._csv_file.close()
            self._csv_file = None

    def _run(self) -> None:
        assert self._serial is not None
        while not self._stop_event.is_set():
            data = self._serial.read(64)
            if not data:
                continue
            frames = self._decoder.feed(data)
            if not frames:
                continue
            self._write_frames(frames)

    def _write_frames(self, frames: Iterable[TelemetryFrame]) -> None:
        assert self._writer is not None
        now = time.time()
        for frame in frames:
            for motor in frame.motors:
                self._writer.writerow(
                    {
                        "host_time": now,
                        "frame_timestamp_ms": frame.timestamp_ms,
                        "motor_id": motor.motor_id,
                        "target_rpm": motor.target_rpm,
                        "current_rpm": motor.current_rpm,
                        "pwm_percent": motor.pwm_percent,
                    }
                )
        self._csv_file.flush()
logger = logging.getLogger(__name__)
