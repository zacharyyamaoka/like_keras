#!/usr/bin/env python3

"""
Meshcat examples for viewing composed Arm+Hand xacro.

Usage:
    python3 examples_view_meshcat.py default
    python3 examples_view_meshcat.py explicit
    python3 examples_view_meshcat.py arm_only
    python3 examples_view_meshcat.py custom
"""

# PYTHON
import sys
from pathlib import Path
import numpy as np

# BAM
from pin_utils.meshcat_client import MeshcatClient


HERE = Path(__file__).resolve().parent
XACRO_PATH = HERE / "arm_hand.urdf.xacro"


def view_default():
    xacro_args = {
        "arm_config_file": str(HERE.parent / "yaml_xacro" / "arm_400_v1_default.yaml"),
        "hand_config_file": str(
            HERE.parent / "yaml_xacro" / "crab_claw_v1_default.yaml"
        ),
    }
    client = MeshcatClient.from_xacro(str(XACRO_PATH), "bam_descriptions", xacro_args)
    client.display(np.zeros(client.model.nq))
    print("[OK] default composition displayed")


def view_explicit():
    xacro_args = {
        "arm_xacro_path": str(HERE.parent / "arm_400_v1" / "arm_400_v1_macro.xacro"),
        "hand_xacro_path": str(
            HERE.parent / "crab_claw_v1" / "crab_claw_v1_macro.xacro"
        ),
        "arm_config_file": str(HERE.parent / "yaml_xacro" / "arm_400_v1_default.yaml"),
        "hand_config_file": str(
            HERE.parent / "yaml_xacro" / "crab_claw_v1_default.yaml"
        ),
    }
    client = MeshcatClient.from_xacro(str(XACRO_PATH), "bam_descriptions", xacro_args)
    client.display(np.zeros(client.model.nq))
    print("[OK] explicit composition displayed")


def view_arm_only():
    xacro_args = {
        "arm_config_file": str(HERE.parent / "yaml_xacro" / "arm_400_v1_default.yaml"),
        "hand_config_file": str(
            HERE.parent / "yaml_xacro" / "crab_claw_v1_default.yaml"
        ),
    }
    client = MeshcatClient.from_xacro(str(XACRO_PATH), "bam_descriptions", xacro_args)
    client.display(np.zeros(client.model.nq))
    print("[OK] arm+hand (from YAML) displayed")


def view_custom_pose():
    # Point to edited YAML files where you've customized pose/prefix
    xacro_args = {
        "arm_config_file": str(HERE.parent / "yaml_xacro" / "arm_400_v1_default.yaml"),
        "hand_config_file": str(
            HERE.parent / "yaml_xacro" / "crab_claw_v1_default.yaml"
        ),
    }
    client = MeshcatClient.from_xacro(str(XACRO_PATH), "bam_descriptions", xacro_args)
    client.display(np.zeros(client.model.nq))
    print("[OK] custom pose displayed")


def main():
    cases = {
        "default": view_default,
        "explicit": view_explicit,
        "arm_only": view_arm_only,
        "custom": view_custom_pose,
    }
    key = sys.argv[1] if len(sys.argv) > 1 else "default"
    if key not in cases:
        print(f"Unknown case '{key}'. Options: {list(cases.keys())}")
        return 1
    cases[key]()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
