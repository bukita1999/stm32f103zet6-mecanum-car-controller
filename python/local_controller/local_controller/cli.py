"""Command line entrypoint."""
from __future__ import annotations

import argparse
import logging
from pathlib import Path
from typing import Optional

from .config import AppConfig, load_config
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
    sequence.add_argument(
        "--csv",
        type=Path,
        required=True,
        help="Path to sequence CSV file",
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
    if args.mode == "sequence":
        try:
            preloaded_commands = load_sequence_commands(args.csv)
        except Exception as exc:  # pylint: disable=broad-except
            logger.error("Failed to load sequence CSV %s: %s", args.csv, exc)
            return 1

    telemetry_logger = TelemetryLogger(config.telemetry_serial, _default_output_dir())
    telemetry_logger.start()

    client = SerialCommandClient(config.command_serial)

    try:
        if args.mode == "remote":
            mode = RemoteControlMode(client, config.remote_profiles, action=args.action)
        elif args.mode == "sequence":
            mode = SequenceMode(client, csv_path=args.csv, preloaded_commands=preloaded_commands)
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
