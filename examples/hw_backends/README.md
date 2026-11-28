# One HardwareInterface, six backends

The rack pendulum (CAN 11) behind one robot_msgs `HardwareInterface`, on each backend Zach listed (Daily Note 2026-11-28):
MuJoCo without friction, MuJoCo realistic, analytical, kinematic, real, and remote to another core. The config object picks
the backend; the description picks the parameters.

```python
from examples.hw_backends.backends import MujocoConfig, AnalyticalConfig, KinematicConfig, RealConfig, RemoteConfig, hardware, hold
from examples.hw_backends.description import nominal, twin_of_record, robot_fingerprint

hw = hardware(MujocoConfig(), nominal())          # 1. no friction
hw = hardware(MujocoConfig(), twin_of_record())   # 2. realistic: the same backend, bam_rack_twin's twin of record
hw = hardware(AnalyticalConfig(), nominal())      # 3. closed-form ODE on the host
hw = hardware(KinematicConfig())                  # 4. measured = commanded
hw = hardware(RealConfig())                       # 5. CAN 11 on fdcanusb 272813EF (rack bus lock, +-3 deg window)
hw = hardware(RemoteConfig(address, robot_fingerprint()))   # 6. like_keras AttachedBackend to a serve_core
state = hw.read(); hw.write(hold(state, 0.01))
```

| File | What |
|---|---|
| `description.py` | `PendulumDescription`: the nominal and twin-of-record parameter sets (bam_rack_twin's, wrapped), `robot_fingerprint()` |
| `backends.py` | the five configs, the backends, the `PENDULUM` HardwareDeployment, `hardware(config, description)`, `serve()` |
| `serve_core.py` | a core: any backend served on like_keras's attach socket (for the remote cases) |
| `measure.py` | the harness: read / write / round-trip latency and loop rate per case, into one JSON |
| `build_report.py` | the report: `reports/hw-backends-latency-2026-11-28.html` |
| `test_hw_backends.py` | read/write on 1-4, nominal MuJoCo = analytical, the real backend's lock refusal, the remote fingerprint refusal |
| `relay.py` | a stdlib byte pipe (unix/TCP) that carries an attach socket to another host: the LAN hairpin, the cloud tunnel |
| `cloud_core.sh` | the cloud VM side of case 6c (cores + relay + a Cloudflare quick tunnel; hostnames pushed on a branch) |
| `run.sh` | runs any of these under bam_rack_twin's venv (MuJoCo 3.11, bam_sim, moteus), like_keras on the path |

Runtime: bam_rack_twin's venv (`/home/bam/bam_ws/src/dev/bam_rack_twin/.venv`), because the realistic twin is bam_rack_twin's
and stays there. Analytical and kinematic need only robot_msgs and numpy (`serve_core --numbers`).

Real runs: write `~/.cache/rack-commissioning/rack_can11_want` first and delete it after (bam_ws memory can11-slot-on-demand);
wait out any full-arm homing.

```bash
bash /home/bam/like_keras/examples/hw_backends/run.sh test
```

Results, 2026-11-28: `reports/hw-backends-latency-2026-11-28.html` (built by `build_report.py` from
`reports/media/hw-backends-2026-11-28/latency.json`). The remote numbers ran on robot_msgs 292b832: robot_msgs 4137538
(GroundTruth.meta: Any) makes ServoState un-encodable for like_keras's codec, so `test_remote_refuses_a_wrong_fingerprint` is
red on robot_msgs main until that is fixed upstream (`PYTHONPATH=<robot_msgs 292b832>/src run.sh test` is green).
