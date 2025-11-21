import os
import time
from collections.abc import Iterable
from datetime import datetime
from fnmatch import fnmatch
from pathlib import Path


def get_latest_file(
    dir_path: str, key: str = None, file_ending: str = None, verbose=True
) -> str | None:
    if not dir_path or not os.path.isdir(dir_path):
        print(f"Save directory {dir_path} does not exist")
        return None

    files = [
        f for f in os.listdir(dir_path) if os.path.isfile(os.path.join(dir_path, f))
    ]
    if verbose:
        print(f"Found {len(files)} files in {dir_path}")
        for file in files:
            print(f"    {file}")

    if key:
        files = [f for f in files if key in f]

    if file_ending:
        files = [f for f in files if f.endswith(file_ending)]

    if not files:
        print(
            f"No matching files found for key: {key}, file ending: {file_ending} in {dir_path}"
        )
        return None

    latest_file = max(files, key=lambda f: os.path.getmtime(os.path.join(dir_path, f)))
    latest_file_path = os.path.join(dir_path, latest_file)
    print(f"Latest file found: {latest_file_path}")
    return latest_file_path


def remove_old_files(save_dir: str, days_old: int = 7, verbose=False):
    """
    Remove files in the save_dir older than days_old days. Avoid memory issues.
    """
    if not save_dir or not os.path.isdir(save_dir):
        print(f"Save directory {save_dir} does not exist")
        return

    now = time.time()
    cutoff = now - (days_old * 86400)

    for filename in os.listdir(save_dir):

        file_path = os.path.join(save_dir, filename)

        if os.path.isfile(file_path):
            file_last_modified = os.path.getmtime(file_path)

            if verbose:
                print(
                    f"File {file_path} last modified: {datetime.fromtimestamp(file_last_modified)}"
                )

            if file_last_modified < cutoff:
                try:
                    os.remove(file_path)
                    print(f"Removed old file: {file_path}")
                except Exception as e:
                    print(f"Failed to remove {file_path}: {e}")


def find_dir(
    path: str,
    include_patterns: Iterable[str] | None = None,
    exclude_patterns: Iterable[str] | None = None,
) -> bool:
    normalized_path = path.rstrip(os.sep)
    name = os.path.basename(normalized_path)

    if exclude_patterns and any(
        fnmatch(normalized_path, pattern) or fnmatch(name, pattern)
        for pattern in exclude_patterns
    ):
        return False

    if not include_patterns:
        return True

    return any(
        fnmatch(normalized_path, pattern) or fnmatch(name, pattern)
        for pattern in include_patterns
    )


def find_file(
    path: str,
    include_patterns: Iterable[str] | None = None,
    exclude_patterns: Iterable[str] | None = None,
) -> bool:
    parent_dir = os.path.dirname(path)
    name = os.path.basename(path)

    if exclude_patterns and any(
        fnmatch(path, pattern)
        or fnmatch(name, pattern)
        or (parent_dir and fnmatch(parent_dir, pattern))
        for pattern in exclude_patterns
    ):
        return False

    if not include_patterns:
        return True

    return any(
        fnmatch(path, pattern)
        or fnmatch(name, pattern)
        or (parent_dir and fnmatch(parent_dir, pattern))
        for pattern in include_patterns
    )


def collect_all_test_files(
    root_path: Path | str,
    include_patterns: Iterable[str] | None = None,
    exclude_patterns: Iterable[str] | None = None,
) -> list[Path]:
    """Collect pytest-target files under test-specific directories."""
    root = Path(root_path).expanduser().resolve()

    if not root.exists():
        return []

    default_include = ("*/tests/test_*.py", "*/test/test_*.py")
    include_patterns = tuple(include_patterns) if include_patterns else default_include
    exclude_patterns = tuple(exclude_patterns) if exclude_patterns else tuple()

    collected: list[Path] = []
    for file_path in root.rglob("test_*.py"):
        if file_path.suffix != ".py":
            continue
        if file_path.parent.name not in {"tests", "test"}:
            continue
        if any(part == "dev" for part in file_path.parts):
            continue
        if not find_file(
            str(file_path),
            include_patterns=include_patterns,
            exclude_patterns=exclude_patterns,
        ):
            continue
        collected.append(file_path)

    return sorted(collected)
