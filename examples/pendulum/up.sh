#!/usr/bin/env bash
# Bring the always-on pendulum core up on dora and keep it up: a private dora 0.5 coordinator + daemon in their own
# network namespace (like_keras kpi/run.py's reason: the production dora holds 127.0.0.1:6012), then the core's
# dataflow on it. Its attach socket is a file, so scripts and the viewer outside the namespace reach it as usual.
#
#   bash /home/bam/like_keras/examples/pendulum/up.sh twin 11      # or: real 11   [optional 3rd arg: an .mcap path]
#
# Stop it with Ctrl-C or SIGTERM: the core brings the pendulum home and stops the board, THEN dora goes down.
# WHY no --die-with-parent / --unshare-pid (2026-11-28, measured): either one SIGKILLs the core when the launcher
# exits, so the return-home never ran; signals are forwarded to the core instead, and dora is stopped after it.
set -euo pipefail
BIND=${1:-twin}; CAN=${2:-11}; RECORD=${3:-}
HERE=$(cd "$(dirname "$0")" && pwd); REPO=$(cd "$HERE/../.." && pwd)
WORK=$REPO/build/pendulum-dora; mkdir -p "$WORK"
if [[ "${LK_IN_NS:-}" != 1 ]]; then
  rm -f "$WORK/core.pid"
  env LK_IN_NS=1 bwrap --dev-bind / / --unshare-net --chdir "$REPO" -- bash "$0" "$BIND" "$CAN" "$RECORD" &
  NS=$!
  forward() { [[ -s "$WORK/core.pid" ]] && kill -TERM "$(cat "$WORK/core.pid")" 2>/dev/null || kill -TERM "$NS"; }
  trap forward INT TERM
  while kill -0 "$NS" 2>/dev/null; do wait "$NS" || true; done
  exit 0
fi
export PATH=$HERE/.venv/bin:$PATH PYTHONPATH=$REPO:$REPO/src
cd "$WORK"
ip link set lo up 2>/dev/null || true
# WHY setsid: a Ctrl-C in the terminal must reach the core, never dora underneath it
setsid dora coordinator --interface 127.0.0.1 --control-interface 127.0.0.1 --quiet > "$WORK/dora.log" 2>&1 &
COORD=$!
for _ in $(seq 100); do (exec 3<>/dev/tcp/127.0.0.1/6012) 2>/dev/null && break; sleep 0.1; done
setsid dora daemon --coordinator-addr 127.0.0.1 --quiet >> "$WORK/dora.log" 2>&1 &
DAEMON=$!
for _ in $(seq 100); do dora check 2>/dev/null | grep -q "Daemon: Running" && break; sleep 0.2; done
cd "$REPO"
python -m examples.pendulum.core --bind "$BIND" --can-id "$CAN" --backend dora ${RECORD:+--record "$RECORD"} &
CORE=$!
echo "$CORE" > "$WORK/core.pid"
trap 'kill -TERM "$CORE" 2>/dev/null || true' INT TERM
while kill -0 "$CORE" 2>/dev/null; do wait "$CORE" || true; done
kill -TERM "$DAEMON" "$COORD" 2>/dev/null || true
wait || true
