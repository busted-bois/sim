"""Resolve log CSV paths under the project root (optional ``{timestamp}`` token)."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path

TIMESTAMP_TOKEN = "{timestamp}"
_DEFAULT_TIMESTAMP_FMT = "%Y-%m-%d_%H-%M-%S"


def csv_timestamp_token(fmt: str = _DEFAULT_TIMESTAMP_FMT) -> str:
    """One-liner-friendly segment, e.g. ``2026-05-23_14-30-00``."""
    return datetime.now().strftime(fmt)


def resolve_log_csv_path(raw_path: str, project_root: Path, *, default: str) -> Path:
    """Expand ``{timestamp}``, then resolve relative paths against ``project_root``."""
    raw = raw_path.strip() or default
    if TIMESTAMP_TOKEN in raw:
        raw = raw.replace(TIMESTAMP_TOKEN, csv_timestamp_token())
    out_path = Path(raw)
    if not out_path.is_absolute():
        out_path = project_root / out_path
    return out_path
