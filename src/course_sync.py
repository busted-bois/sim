"""Stage ``course/<map>/FlyingExampleMapV2`` into Colosseum ``Maps/FlyingExampleMapV2``."""

from __future__ import annotations

import shutil
from pathlib import Path

DEST_MAP_REL = (
    Path("Content")
    / "__ExternalActors__"
    / "FlyingCPP"
    / "Maps"
    / "FlyingExampleMapV2"
)

MAP_FOLDER = "FlyingExampleMapV2"


def _course_sync_disabled(map_token: str) -> bool:
    """No staging: empty, or explicit ``none`` (case-insensitive)."""
    t = map_token.strip()
    return not t or t.lower() == "none"


def _find_variant_dir(course_root: Path, map_name: str) -> Path | None:
    """``course/<map>`` folder; map name matched case-insensitively, exact case preferred."""
    token = map_name.strip()
    if not token or not course_root.is_dir():
        return None
    matches = [p for p in course_root.iterdir() if p.is_dir() and p.name.lower() == token.lower()]
    if not matches:
        return None
    for p in matches:
        if p.name == token:
            return p
    return matches[0]


def _flying_map_source_root(map_variant_dir: Path) -> Path | None:
    """Folder tree to mirror into Colosseum.

    Accepts either ``.../FlyingExampleMapV2/<actors>`` or an extra nested UE export
    ``.../FlyingExampleMapV2/FlyingExampleMapV2/<actors>``.
    """
    outer = map_variant_dir / MAP_FOLDER
    if not outer.is_dir():
        return None
    inner = outer / MAP_FOLDER
    if inner.is_dir():
        return inner
    return outer


def _assert_safe_flying_map_dest(project_root: Path, dest_base: Path) -> None:
    """Ensure we only remove ``.../Maps/FlyingExampleMapV2`` inside the BlocksV2 project tree."""
    if dest_base.name.lower() != MAP_FOLDER.lower():
        raise SystemExit(
            "course_sync safety: refuse to delete path whose final segment is not "
            f"{MAP_FOLDER!r} (got {dest_base.name!r} in {dest_base})"
        )

    try:
        proj = project_root.resolve(strict=False)
        dest = dest_base.resolve(strict=False)
    except OSError as exc:
        raise SystemExit(
            f"course_sync safety: could not resolve project or destination: {exc}"
        ) from exc

    if dest == proj:
        raise SystemExit(
            f"course_sync safety: destination {dest} equals project root; refusing to delete."
        )

    try:
        dest.relative_to(proj)
    except ValueError:
        raise SystemExit(
            "course_sync safety: destination is not inside the project directory "
            f"(project={proj}, destination={dest})"
        ) from None


def sync_course_from_project_path(project_path: str, repo_root: Path, map_name: str) -> None:
    """Replace Colosseum ``FlyingExampleMapV2`` with ``course/<map>/FlyingExampleMapV2``."""
    raw_map = (map_name or "").strip()
    if _course_sync_disabled(raw_map):
        return

    raw_project = project_path.strip()
    if not raw_project:
        print(
            f"Warning: map={raw_map!r} but PROJECT_PATH is unset; skipping course sync."
        )
        return

    uproject = Path(raw_project)
    if not uproject.is_file() or uproject.suffix.lower() != ".uproject":
        print(
            f"Warning: map={raw_map!r} but PROJECT_PATH is not a .uproject file; "
            "skipping course sync."
        )
        return

    course_root = repo_root / "course"
    variant = _find_variant_dir(course_root, raw_map)
    if variant is None:
        avail: list[str] = []
        if course_root.is_dir():
            avail = sorted(p.name for p in course_root.iterdir() if p.is_dir())
        hint = (
            f"Available map folders: {', '.join(avail)}"
            if avail
            else "No subfolders found in course/."
        )
        raise SystemExit(
            f"map={raw_map!r}: no matching folder under {course_root}. "
            f"Expected course/{raw_map}/{MAP_FOLDER}/... "
            f"(map folder name is matched case-insensitively). {hint}"
        )

    source = _flying_map_source_root(variant)
    if source is None:
        raise SystemExit(
            f'Course map "{variant.name}" has no {MAP_FOLDER} folder at {variant / MAP_FOLDER}'
        )

    blocks_v2_root = uproject.parent
    dest_base = blocks_v2_root / DEST_MAP_REL
    _assert_safe_flying_map_dest(blocks_v2_root, dest_base)

    if dest_base.exists():
        if dest_base.is_symlink():
            raise SystemExit("Refusing to delete symlinked directory.")
        if not dest_base.is_dir():
            raise SystemExit(
                f"course_sync: expected a directory at {dest_base}; "
                "refusing to replace a non-directory."
            )
        shutil.rmtree(dest_base)

    shutil.copytree(source, dest_base, copy_function=shutil.copy2)
    n_files = sum(1 for p in dest_base.rglob("*") if p.is_file())
    print(
        f'Course map "{variant.name}": replaced {MAP_FOLDER} ({n_files} files) -> {dest_base}'
    )
