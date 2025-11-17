"""Remote control mode."""
from __future__ import annotations

import logging
from typing import Dict, Iterable, Optional

from ..config import RemoteProfileConfig
from ..serial_client import SerialCommandClient
from .base import ControlMode

logger = logging.getLogger(__name__)


class RemoteControlMode(ControlMode):
    def __init__(
        self,
        client: SerialCommandClient,
        profiles: RemoteProfileConfig,
        action: Optional[str] = None,
    ) -> None:
        super().__init__(client)
        self._profiles = profiles
        self._action = action

    def run(self) -> None:
        if self._action:
            self._send_action(self._action)
            return
        self._interactive_loop()

    def _interactive_loop(self) -> None:
        prompt = (
            "Enter command (w=forward, s=backward, a=left, d=right, x=stop, q=quit): "
        )
        mapping: Dict[str, str] = {
            "w": "forward",
            "s": "backward",
            "a": "left",
            "d": "right",
            "x": "stop",
        }
        print("Remote control mode active. Press q to exit.")
        while True:
            try:
                key = input(prompt).strip().lower()
            except (KeyboardInterrupt, EOFError):
                break
            if key == "q":
                break
            if key not in mapping:
                print("Unknown command. Use w/a/s/d/x or q to quit.")
                continue
            action = mapping[key]
            self._send_action(action)
        self._send_action("stop")

    def _send_action(self, action: str) -> None:
        try:
            speeds = self._profiles.get_profile(action)
        except (KeyError, ValueError) as exc:
            logger.error("%s", exc)
            return
        logger.info("Sending %s -> %s", action, speeds)
        self._client.send_speeds(speeds)
