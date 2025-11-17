"""Base control mode declaration."""
from __future__ import annotations

from abc import ABC, abstractmethod

from ..serial_client import SerialCommandClient


class ControlMode(ABC):
    def __init__(self, client: SerialCommandClient) -> None:
        self._client = client

    @abstractmethod
    def run(self) -> None:
        """Execute the control loop until completion or ctrl+c."""
