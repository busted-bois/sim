"""Fail if the string 'airsim' appears anywhere in tracked project sources."""

from __future__ import annotations

import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

SKIP_DIRS = {
    ".git",
    ".venv",
    ".ruff_cache",
    ".cursor",
    ".claude",
    ".sisyphus",
    ".codex_worktrees",
    "opensrc",
    "node_modules",
    "logs",
    "__pycache__",
}

SKIP_FILES = {
    "tests/test_no_airsim_strings.py",
}


class NoAirsimStringsTests(unittest.TestCase):
    def test_repo_has_no_airsim_mentions(self) -> None:
        pattern = re.compile(r"airsim", re.IGNORECASE)
        hits: list[str] = []
        for path in ROOT.rglob("*"):
            if not path.is_file():
                continue
            rel = path.relative_to(ROOT).as_posix()
            if any(part in SKIP_DIRS for part in path.parts):
                continue
            if rel in SKIP_FILES:
                continue
            if rel.startswith(".git/"):
                continue
            if path.suffix in {".pyc", ".pyo"}:
                continue
            try:
                text = path.read_text(encoding="utf-8", errors="replace")
            except OSError:
                continue
            for line_no, line in enumerate(text.splitlines(), start=1):
                if pattern.search(line):
                    hits.append(f"{rel}:{line_no}: {line.strip()[:120]}")
        self.assertEqual(
            hits,
            [],
            "Found forbidden 'airsim' references:\n" + "\n".join(hits[:50]),
        )


if __name__ == "__main__":
    unittest.main()
