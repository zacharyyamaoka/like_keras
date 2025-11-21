#!/usr/bin/env python3

"""
Run several xacro composition scenarios to validate arm+hand composer.
"""

import subprocess
from pathlib import Path


THIS_DIR = Path(__file__).resolve().parent
XACRO = "xacro"
XACRO_FILE = THIS_DIR / "arm_hand.urdf.xacro"


def run_cmd(args: list[str]) -> tuple[int, str, str]:
    proc = subprocess.run(
        args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )
    return proc.returncode, proc.stdout, proc.stderr


def case_default() -> None:
    code, out, err = run_cmd([XACRO, str(XACRO_FILE)])
    if code != 0:
        raise RuntimeError(f"default failed: {err}")
    assert "eef_link_1" in out, "Expected hand link in default composition"


def case_explicit_paths() -> None:
    code, out, err = run_cmd(
        [
            XACRO,
            str(XACRO_FILE),
            f"arm_xacro_path:={THIS_DIR.parent / 'arm_400_v1' / 'arm_400_v1_macro.xacro'}",
            f"hand_xacro_path:={THIS_DIR.parent / 'crab_claw_v1' / 'crab_claw_v1_macro.xacro'}",
            "arm_macro_name:=arm_400_v1",
            "hand_macro_name:=crab_claw_v1",
        ]
    )
    if code != 0:
        raise RuntimeError(f"explicit_paths failed: {err}")
    assert "eef_link_1" in out, "Expected hand link when explicit paths provided"


def case_disabled_hand() -> None:
    code, out, err = run_cmd(
        [
            XACRO,
            str(XACRO_FILE),
            "hand_enabled:=false",
            "prefix:=t_",
        ]
    )
    if code != 0:
        raise RuntimeError(f"disabled_hand failed: {err}")
    assert "t_eef_link_1" not in out, "Hand should be disabled, but link found"


def case_custom_prefix_and_pose() -> None:
    code, out, err = run_cmd(
        [
            XACRO,
            str(XACRO_FILE),
            "prefix:=custom_",
            "hand_xyz:=0.01 0.02 0.03",
            "hand_rpy:=0.1 0.2 0.3",
        ]
    )
    if code != 0:
        raise RuntimeError(f"custom_prefix_and_pose failed: {err}")
    assert "custom_eef_link_1" in out, "Expected prefixed hand link"


def main() -> int:
    cases = [
        ("default", case_default),
        ("explicit_paths", case_explicit_paths),
        ("disabled_hand", case_disabled_hand),
        ("custom_prefix_and_pose", case_custom_prefix_and_pose),
    ]
    for name, fn in cases:
        print(f"[RUN] {name}")
        fn()
        print(f"[OK ] {name}")
    print("All arm_hand xacro tests passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
