"""A core for the remote cases: one of backends 1-5 served on like_keras's attach socket until SIGTERM / Ctrl-C.

    bash /home/bam/like_keras/examples/hw_backends/run.sh serve_core --backend analytical
    bash /home/bam/like_keras/examples/hw_backends/run.sh serve_core --backend kinematic --numbers   # no bam_rack_twin (a cloud VM)

Prints one JSON line: {"ready", "address", "fingerprint", "backend", "description"}. The fingerprint is the robot's
(description.robot_fingerprint); a client passes it as RemoteConfig.fingerprint and is refused if it differs.
"""

from __future__ import annotations

import argparse
import json
import signal

from examples.hw_backends.backends import AnalyticalConfig, KinematicConfig, MujocoConfig, RealConfig, hardware, serve
from examples.hw_backends.description import from_numbers, robot_fingerprint

# WHY frozen numbers for --numbers: a cloud VM has no bam_rack_twin (nor /archive), so it rebuilds the nominal description from
# the plain numbers description.nominal().numbers() printed on the workstation (2026-11-28); the analytical plant needs no more.
NOMINAL_NUMBERS = {"label": "nominal, frictionless (frozen numbers)", "gravity_nm": 4.6611971558188605, "bottom_rad": -0.373684718743872,
                   "inertia_kg_m2": 0.12192564375276548, "damping_nm_s_rad": 0.0, "coulomb_nm": 0.0, "kp_nm_rad": 31.830988618379067,
                   "kd_nm_s_rad": 0.6366197723675814, "cap_nm": 3.0, "source": "description.nominal().numbers(), 2026-11-28"}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--backend", choices=("mujoco_nominal", "mujoco_twin", "analytical", "kinematic", "real"), default="analytical")
    parser.add_argument("--numbers", action="store_true", help="the nominal description from frozen numbers (no bam_rack_twin)")
    parser.add_argument("--name", default="pendulum_can11")
    arguments = parser.parse_args(argv)
    if arguments.numbers:
        description = from_numbers(NOMINAL_NUMBERS)
    elif arguments.backend in ("mujoco_twin",):
        from examples.hw_backends.description import twin_of_record  # noqa: PLC0415

        description = twin_of_record()
    elif arguments.backend in ("mujoco_nominal", "analytical"):
        from examples.hw_backends.description import nominal  # noqa: PLC0415

        description = nominal()
    else:
        description = None
    config = {"mujoco_nominal": MujocoConfig(), "mujoco_twin": MujocoConfig(), "analytical": AnalyticalConfig(),
              "kinematic": KinematicConfig(), "real": RealConfig()}[arguments.backend]
    hw = hardware(config, description)
    dep = serve(hw, arguments.name)
    dep.start()
    sheet = dep.describe()
    print(json.dumps({"ready": True, "address": sheet["attach"]["socket"], "fingerprint": robot_fingerprint(hw.joints),
                      "backend": arguments.backend, "description": None if description is None else description.label}), flush=True)
    try:
        signal.sigwait({signal.SIGTERM, signal.SIGINT})
    finally:
        dep.stop()
        hw.close()
    return 0


if __name__ == "__main__":
    signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGTERM, signal.SIGINT})
    raise SystemExit(main())
