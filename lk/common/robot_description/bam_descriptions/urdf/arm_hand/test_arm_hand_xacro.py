#!/usr/bin/env python3

"""
    Smoke test: render arm_hand.urdf.xacro with defaults.
    Fails fast if xacro processing errors occur.
"""

# PYTHON
import subprocess
import sys
from pathlib import Path


def run() -> int:
    xacro = "xacro"
    xacro_file = Path(__file__).resolve().parent / "arm_hand.urdf.xacro"

    cmd = [
        xacro,
        str(xacro_file),
        "arm_xacro_path:=$(find bam_descriptions)/urdf/arm_400_v1/arm_400_v1_macro.xacro",
        "arm_macro_name:=arm_400_v1",
        "hand_xacro_path:=$(find bam_descriptions)/urdf/crab_claw_v1/crab_claw_v1_macro.xacro",
        "hand_macro_name:=crab_claw_v1",
        "prefix:=test_",
        "hand_enabled:=true",
    ]

    print("Running:", " ".join(cmd))
    proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if proc.returncode != 0:
        print(proc.stderr)
        return proc.returncode

    # Optionally verify some expected link names appear in output
    out = proc.stdout
    assert "test_eef_link_1" in out or "test_crab_claw_center" in out, "Expected hand links not found in URDF output"
    return 0


if __name__ == "__main__":
    sys.exit(run())


