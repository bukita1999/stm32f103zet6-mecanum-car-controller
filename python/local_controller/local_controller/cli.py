"""Command line entrypoint."""
from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path
from typing import Optional

from .config import AppConfig, load_config
from .modes.base import ControlMode
from .modes.remote import RemoteControlMode
from .modes.sequence import SequenceMode, load_sequence_commands
from .modes.webdebug import WebDebugMode
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
    sequence_source = sequence.add_mutually_exclusive_group(required=True)
    sequence_source.add_argument(
        "--csv",
        type=Path,
        help="Path to sequence CSV file",
    )
    sequence_source.add_argument(
        "--tui",
        action="store_true",
        help="Select a CSV file from the data directory",
    )

    webdebug = subparsers.add_parser("webdebug", help="Launch web debug web UI")
    webdebug.add_argument(
        "--host",
        default="0.0.0.0",
        help="Host/IP to bind for the web UI",
    )
    webdebug.add_argument(
        "--port",
        type=int,
        default=5000,
        help="Port for the web UI",
    )
    webdebug.add_argument(
        "--no-newline",
        action="store_true",
        help="Do not append newline when sending typed messages",
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

    preloaded_commands = None
    sequence_csv: Path | None = None
    session_name = args.mode
    if args.mode == "sequence":
        try:
            if args.tui:
                sequence_csv = _select_csv_from_data(_default_data_dir())
            else:
                sequence_csv = args.csv
            session_name = f"{args.mode}_{sequence_csv.stem}"
            preloaded_commands = load_sequence_commands(sequence_csv)
        except Exception as exc:  # pylint: disable=broad-except
            logger.error("Failed to load sequence CSV %s: %s", sequence_csv, exc)
            return 1

    telemetry_logger = TelemetryLogger(
        config.telemetry_serial,
        _default_output_dir(),
        session_name=session_name,
    )
    telemetry_logger.start()

    client = SerialCommandClient(config.command_serial)

    try:
        mode: ControlMode
        if args.mode == "remote":
            mode = RemoteControlMode(client, config.remote_profiles, action=args.action)
        elif args.mode == "sequence":
            if sequence_csv is None:
                raise RuntimeError("Sequence CSV path not set")
            mode = SequenceMode(
                client,
                csv_path=sequence_csv,
                preloaded_commands=preloaded_commands,
            )
        elif args.mode == "webdebug":
            mode = WebDebugMode(
                client,
                host=args.host,
                port=args.port,
                append_newline=not args.no_newline,
            )
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


def _default_data_dir() -> Path:
    return Path(__file__).resolve().parent.parent / "data"


def _select_csv_from_data(data_dir: Path) -> Path:
    if not sys.stdin.isatty():
        raise RuntimeError("--tui requires an interactive terminal")
    if not data_dir.exists():
        raise FileNotFoundError(f"Data directory not found: {data_dir}")
    csv_files = sorted(data_dir.glob("*.csv"))
    if not csv_files:
        raise FileNotFoundError(f"No CSV files found in {data_dir}")
    print("Select a sequence CSV:")
    for idx, path in enumerate(csv_files, start=1):
        print(f"  {idx}. {path.name}")
    while True:
        choice = input("Enter number (or q to quit): ").strip().lower()
        if choice in {"q", "quit", "exit"}:
            raise RuntimeError("Sequence selection cancelled")
        try:
            index = int(choice)
        except ValueError:
            print("Please enter a valid number.")
            continue
        if 1 <= index <= len(csv_files):
            return csv_files[index - 1]
        print("Selection out of range.")
