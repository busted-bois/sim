from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SCAN_SUFFIXES = {".py", ".yml", ".yaml", ".json", ".md", ".toml", ".ini", ".ps1", ".sh"}
SKIP_DIRS = {
    ".venv",
    "airsim",
    "msgpackrpc",
    "opensrc",
    "simple_airsim",
    ".git",
    ".ruff_cache",
    ".claude",
    ".sisyphus",
    ".codex_worktrees",
}
CONFLICT_PREFIXES = ("<<<<<<<", ">>>>>>>")


def _should_scan(path: Path) -> bool:
    if path.suffix.lower() not in SCAN_SUFFIXES:
        return False
    return not any(part in SKIP_DIRS for part in path.parts)


def find_merge_conflict_markers(root: Path = ROOT) -> list[str]:
    hits: list[str] = []
    for path in sorted(root.rglob("*")):
        if not path.is_file() or not _should_scan(path):
            continue
        for line_no, line in enumerate(
            path.read_text(encoding="utf-8", errors="replace").splitlines(),
            start=1,
        ):
            stripped = line.strip()
            if stripped.startswith(CONFLICT_PREFIXES) or stripped == "=======":
                hits.append(f"{path.relative_to(root)}:{line_no}: {stripped}")
    return hits


def main() -> None:
    hits = find_merge_conflict_markers()
    if not hits:
        print("OK: no git merge conflict markers found.")
        return
    print("Git merge conflict markers found:", file=sys.stderr)
    for hit in hits:
        print(f"  {hit}", file=sys.stderr)
    raise SystemExit(1)


if __name__ == "__main__":
    main()
