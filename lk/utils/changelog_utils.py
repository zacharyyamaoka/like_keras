#!/usr/bin/env python3

"""
Utilities for working with changelog and version management.
"""

# PYTHON
import subprocess
from typing import Optional


def get_latest_tag() -> Optional[str]:
    """Get the most recent git tag."""
    try:
        result = subprocess.run(
            ["git", "describe", "--tags", "--abbrev=0"],
            capture_output=True,
            text=True,
            check=True,
        )
        return result.stdout.strip()
    except subprocess.CalledProcessError:
        return None


def parse_version(tag: str) -> tuple[int, int, int]:
    """Parse version tag into (major, minor, patch) tuple."""
    if tag.startswith("v"):
        tag = tag[1:]
    parts = tag.split(".")
    if len(parts) != 3:
        raise ValueError(f"Invalid version tag: {tag}")
    return int(parts[0]), int(parts[1]), int(parts[2])


def get_feature_info(commit_msg: str) -> dict[str, str]:
    """Extract feature info from commit message."""
    import re

    pattern = r"^(feat|fix|test|refactor)\(([A-Z0-9]+-[A-Z]-S[0-9]+)\):"
    match = re.match(pattern, commit_msg)

    if match:
        return {
            "type": match.group(1),
            "feature": match.group(2),
        }
    return {}
