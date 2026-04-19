"""Stage ``course/`` uassets into Colosseum FlyingExampleMapV2 external actors."""

from __future__ import annotations

import shutil
from pathlib import Path

MAP_EXTERNAL_REL = (
    Path("Content")
    / "__ExternalActors__"
    / "FlyingCPP"
    / "Maps"
    / "FlyingExampleMapV2"
)


def sync_course_assets(
    *,
    repo_root: Path,
    blocks_v2_root: Path,
    course_dir: Path | None = None,
) -> tuple[int, int]:
    """Copy files under ``course/`` into the map external-actors tree.

    Preserves subpaths (e.g. ``course/4/3T/foo.uasset`` →
    ``.../FlyingExampleMapV2/4/3T/foo.uasset``). Skips copy when the destination
    file already exists.

    Returns ``(copied, skipped_existing)``.
    """
    course = course_dir if course_dir is not None else repo_root / "course"
    if not course.is_dir():
        return (0, 0)

    dest_base = blocks_v2_root / MAP_EXTERNAL_REL
    copied = 0
    skipped = 0

    for src in course.rglob("*"):
        if not src.is_file():
            continue
        rel = src.relative_to(course)
        dest = dest_base / rel
        if dest.is_file():
            skipped += 1
            continue
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dest)
        copied += 1

    return (copied, skipped)


def sync_course_from_project_path(project_path: str, repo_root: Path) -> None:
    """If ``PROJECT_PATH`` points at a ``.uproject`` under BlocksV2, stage course assets."""
    raw = project_path.strip()
    if not raw:
        return

    uproject = Path(raw)
    if not uproject.is_file() or uproject.suffix.lower() != ".uproject":
        return

    blocks_v2_root = uproject.parent
    copied, skipped = sync_course_assets(repo_root=repo_root, blocks_v2_root=blocks_v2_root)
    if copied or skipped:
        dest = blocks_v2_root / MAP_EXTERNAL_REL
        print(f"Course → Colosseum: copied {copied}, skipped (already present) {skipped} → {dest}")
