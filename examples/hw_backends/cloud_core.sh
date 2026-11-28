#!/usr/bin/env bash
# Case 6c's far end, run on a Claude Code cloud VM from a like_keras clone: two analytical pendulum cores (stepped and free)
# served on like_keras attach sockets, each carried to TCP by relay.py and out through a Cloudflare quick tunnel (no
# account: cloudflared tunnel --url tcp://...). The tunnel hostnames go home on a pushed branch, claude/hwb-tunnel-<id>,
# because a cloud VM takes no inbound connection and has no other way to say where it is. Everything is torn down after
# <lifetime> seconds. WHY analytical from frozen numbers: the VM has neither bam_rack_twin nor /archive (serve_core --numbers).
#   bash examples/hw_backends/cloud_core.sh <id> [lifetime_s]
set -uo pipefail
ID=${1:?id}; LIFE=${2:-900}
REPO=$(cd "$(dirname "$0")/../.." && pwd); cd "$REPO"
W=/tmp/hwb; mkdir -p $W/xdg; export XDG_RUNTIME_DIR=$W/xdg PYTHONPATH=$REPO:$REPO/src
echo "[cloud_core] host $(hostname) $(nproc) cpus, python $(python3 -V 2>&1)"
python3 -m venv $W/venv && $W/venv/bin/pip install -q numpy || { echo "HWB_FAIL pip numpy"; exit 1; }
if git clone -q --depth 1 https://github.com/zacharyyamaoka/robot_msgs $W/robot_msgs; then
  $W/venv/bin/pip install -q -e $W/robot_msgs || { echo "HWB_FAIL pip robot_msgs"; exit 1; }
else
  $W/venv/bin/pip install -q robot-msgs || { echo "HWB_FAIL robot_msgs"; exit 1; }
fi
PY=$W/venv/bin/python
curl -fsSL -o $W/cloudflared https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-amd64 \
  && chmod +x $W/cloudflared || { echo "HWB_FAIL cloudflared download"; exit 1; }
PIDS=()
declare -A HOST
PORT=47040
for MODE in stepped free; do
  $PY -m examples.hw_backends.serve_core --backend analytical --numbers --mode $MODE --name hwb_$MODE > $W/core_$MODE.json 2> $W/core_$MODE.err &
  PIDS+=($!)
  for _ in $(seq 100); do grep -q ready $W/core_$MODE.json 2>/dev/null && break; sleep 0.2; done
  ADDR=$(python3 -c "import json;print(json.load(open('$W/core_$MODE.json'))['address'])") || { cat $W/core_$MODE.err; echo "HWB_FAIL core $MODE"; exit 1; }
  $PY examples/hw_backends/relay.py --listen tcp:127.0.0.1:$PORT --to "$ADDR" --lifetime "$LIFE" > $W/relay_$MODE.log 2>&1 &
  PIDS+=($!)
  # WHY http2: TCP 7844 to the edge, where a VM may refuse QUIC's UDP
  $W/cloudflared tunnel --no-autoupdate --protocol http2 --url tcp://127.0.0.1:$PORT > $W/cf_$MODE.log 2>&1 &
  PIDS+=($!)
  for _ in $(seq 60); do grep -qo "[a-z0-9-]*\.trycloudflare\.com" $W/cf_$MODE.log && grep -q "Registered tunnel connection" $W/cf_$MODE.log && break; sleep 1; done
  HOST[$MODE]=$(grep -o "[a-z0-9-]*\.trycloudflare\.com" $W/cf_$MODE.log | head -1)
  grep -q "Registered tunnel connection" $W/cf_$MODE.log || { tail -20 $W/cf_$MODE.log; echo "HWB_FAIL tunnel $MODE did not register"; }
  PORT=$((PORT + 1))
done
python3 - "$ID" "${HOST[stepped]:-}" "${HOST[free]:-}" "$W" <<'EOF' > hwb_tunnel.json
import json, socket, sys
id_, stepped, free, w = sys.argv[1:]
cores = {m: json.load(open(f"{w}/core_{m}.json")) for m in ("stepped", "free")}
print(json.dumps({"id": id_, "vm": socket.gethostname(), "hosts": {"stepped": stepped, "free": free}, "cores": cores}))
EOF
cat hwb_tunnel.json
git checkout -q -b "claude/hwb-tunnel-$ID" && git add -f hwb_tunnel.json && git -c user.name=hwb -c user.email=hwb@localhost commit -q -m "hwb tunnel $ID" \
  && git push -q origin "HEAD:refs/heads/claude/hwb-tunnel-$ID" && echo "[cloud_core] pushed claude/hwb-tunnel-$ID" || echo "HWB_FAIL push"
echo "[cloud_core] up for $LIFE s"
sleep "$LIFE"
kill -TERM "${PIDS[@]}" 2>/dev/null; wait
echo "HWB_DONE"
