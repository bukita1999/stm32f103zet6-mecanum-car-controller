"""Configuration loading utilities."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List
import tomllib


@dataclass
class SerialPortConfig:
    port: str
    baudrate: int = 115200
    timeout: float = 0.1


@dataclass
class RemoteProfileConfig:
    speeds: Dict[str, List[int]] = field(default_factory=dict)

    def get_profile(self, name: str) -> List[int]:
        if name not in self.speeds:
            raise KeyError(f"Unknown remote profile: {name}")
        values = self.speeds[name]
        if len(values) != 4:
            raise ValueError(f"Profile '{name}' must contain 4 wheel speeds, got {len(values)}")
        return values


@dataclass
class AppConfig:
    command_serial: SerialPortConfig
    telemetry_serial: SerialPortConfig
    remote_profiles: RemoteProfileConfig

    @staticmethod
    def load(path: Path) -> "AppConfig":
        with path.open("rb") as fh:
            raw = tomllib.load(fh)

        try:
            serial_cfg = raw["serial"]
            remote_cfg = raw["remote"]
        except KeyError as exc:
            raise KeyError(f"Missing configuration section: {exc.args[0]}") from exc

        cmd_serial = SerialPortConfig(**serial_cfg["command"])
        telemetry_serial = SerialPortConfig(**serial_cfg["telemetry"])
        remote_profiles = RemoteProfileConfig(speeds=remote_cfg["speed_profiles"])
        return AppConfig(
            command_serial=cmd_serial,
            telemetry_serial=telemetry_serial,
            remote_profiles=remote_profiles,
        )


def load_config(config_path: Path | None) -> AppConfig:
    if config_path is None:
        config_path = Path(__file__).resolve().parent.parent / "config.toml"
    return AppConfig.load(config_path)
