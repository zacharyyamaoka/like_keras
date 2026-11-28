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
hw = hardware(RemoteConfig(address, robot_fingerprint()))   # 6. like_keras AttachedBackend (waits on like_keras main)
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
| `run.sh` | runs any of these under bam_rack_twin's venv (MuJoCo 3.11, bam_sim, moteus), like_keras on the path |

Runtime: bam_rack_twin's venv (`/home/bam/bam_ws/src/dev/bam_rack_twin/.venv`), because the realistic twin is bam_rack_twin's
and stays there. Analytical and kinematic need only robot_msgs and numpy (`serve_core --numbers`).

Real runs: write `~/.cache/rack-commissioning/rack_can11_want` first and delete it after (bam_ws memory can11-slot-on-demand);
wait out any full-arm homing.

```bash
bash /home/bam/like_keras/examples/hw_backends/run.sh test
```
