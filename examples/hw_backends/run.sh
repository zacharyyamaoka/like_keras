#!/usr/bin/env bash
# Run a module of this example under bam_rack_twin's venv: it carries the MuJoCo twin of record (bam_rack_twin, bam_sim,
# moteus_servo, mujoco 3.11), moteus and robot_msgs; like_keras is put on the path, so its core keeps its one dependency.
#   bash /home/bam/like_keras/examples/hw_backends/run.sh measure --cases mujoco_nominal,kinematic
#   bash /home/bam/like_keras/examples/hw_backends/run.sh test
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd); REPO=$(cd "$HERE/../.." && pwd)
RACK=${BAM_RACK_TWIN:-/home/bam/bam_ws/src/dev/bam_rack_twin}
PY="$RACK/.venv/bin/python"
export PYTHONPATH=$REPO:$REPO/src${PYTHONPATH:+:$PYTHONPATH} MUJOCO_GL=${MUJOCO_GL:-egl}
cd "$REPO"
what=${1:-measure}; shift || true
case "$what" in
  test) exec env -u VIRTUAL_ENV "$PY" -m pytest -q -p no:cacheprovider "$HERE/test_hw_backends.py" "$@" ;;
  *) exec env -u VIRTUAL_ENV "$PY" -m "examples.hw_backends.$what" "$@" ;;
esac
