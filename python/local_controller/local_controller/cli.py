"""Command line entrypoint."""
from __future__ import annotations

import argparse
import logging
from pathlib import Path
from typing import Optional

from .config import AppConfig, load_config
from .modes.remote import RemoteControlMode
from .modes.sequence import SequenceMode
from .serial_client import SerialCommandClient
from .telemetry import TelemetryLogger

logger = logging.getLogger(__name__)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Control STM32 mecanum car")
    parser.add_argument(
        "--config",
        type=Path,
        help="Path to config.toml (defaults to package config)",
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Log level",
    )

    subparsers = parser.add_subparsers(dest="mode", required=True)

    remote = subparsers.add_parser("remote", help="Manual remote mode")
    remote.add_argument(
        "--action",
        choices=["forward", "backward", "left", "right", "stop"],
        help="Send a single action instead of interactive mode",
    )

    sequence = subparsers.add_parser("sequence", help="Run predefined CSV sequence")
    sequence.add_argument(
        "--csv",
        type=Path,
        required=True,
        help="Path to sequence CSV file",
    )

    return parser


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="[%(levelname)s] %(message)s",
    )

    config = load_config(args.config)
    telemetry_logger = TelemetryLogger(config.telemetry_serial, _default_output_dir())
    telemetry_logger.start()

    client = SerialCommandClient(config.command_serial)

    try:
        if args.mode == "remote":
            mode = RemoteControlMode(client, config.remote_profiles, action=args.action)
        elif args.mode == "sequence":
            mode = SequenceMode(client, csv_path=args.csv)
        else:
            parser.error(f"Unknown mode {args.mode}")
            return 2
        mode.run()
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    finally:
        telemetry_logger.stop()
        client.close()

    return 0


def _default_output_dir() -> Path:
    return Path(__file__).resolve().parent.parent / "output"
