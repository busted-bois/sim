"""Minimal fakes for MAVLink unit tests (no dependency on feat branch)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class FakeMessage:
    msg_type: str
    _fields: dict[str, Any] = field(default_factory=dict)

    def __init__(self, msg_type: str, **fields: Any) -> None:
        self.msg_type = msg_type
        self._fields = dict(fields)

    def get_type(self) -> str:
        return self.msg_type

    def __getattr__(self, name: str) -> Any:
        if name in self._fields:
            return self._fields[name]
        raise AttributeError(name)

    def get_srcSystem(self) -> int:
        return int(self._fields.get("source_system", 0))

    def get_srcComponent(self) -> int:
        return int(self._fields.get("source_component", 0))


@dataclass
class FakeMav:
    message_interval_calls: list[tuple[int, int]] = field(default_factory=list)

    def message_interval_send(self, message_id: int, interval_us: int) -> None:
        self.message_interval_calls.append((int(message_id), int(interval_us)))
