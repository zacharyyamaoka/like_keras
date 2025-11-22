https://discourse.openrobotics.org/t/needed-ros2-node-management-tool/39889

https://github.com/ros2/design/blob/acb998756be1bdae5ce816794335e3950e3a3ed2/articles/153_roslaunch_remote_launch.md

https://github.com/ros2/design/pull/297


---

1. What ros2 launch can and can’t do

ros2 launch my_pkg my_launch.py
➜ Starts all the nodes on the machine where you run the command.

There’s no official machine tag (like ROS 1) in ROS 2 launch that says “run this node on host X and that node on host Y”.

So for multi-machine setups you typically:

Run one launch file on machine A.

Run another launch file on machine B.

Let DDS discovery handle comms between them (assuming same ROS_DOMAIN_ID, network reachable, etc.).

2. Common ways to simulate “ROS 2 starting processes on other machines”

Because everyone wants this, people layer extra tools on top:

SSH + launch scripts

A small Python/bash script on your main machine:

ssh robot1 ros2 launch pkg_a robot1_bringup.launch.py

ssh robot2 ros2 launch pkg_b robot2_bringup.launch.py

From your perspective, one script kicks off processes on multiple machines.

tmux / screen + ssh

For dev: scripts that create tmux sessions on each remote machine and run launch files there. Nice for attaching and seeing logs.

Container orchestration (Docker Compose / Kubernetes / Nomad)

Run each ROS 2 node in a container.

Use Docker Compose or k8s to bring the whole system up across machines.

ROS 2 just sees multiple nodes on the network.

Systemd services

On each robot/computer, define systemd services that start ros2 launch ….

From your laptop you run a management script (again over ssh) that starts/stops services remotely.

---
Definetly the public API I think should just be a machine..


-- perfect dora already supprots this amazing!

https://dora-rs.ai/docs/api/cli/?utm_source=chatgpt.com


ok they have a central coordinationr.

ok then they have machine id arugment


---
https://github.com/dora-rs/dora/issues/459

https://github.com/dora-rs/dora/issues/512



---
Key Insights for like_keras
What this means for your implementation:
You generate the YAML config - The dataflow file with node definitions
Dora CLI handles coordination - No need to implement coordinator/daemon
Your engine just calls: dora start dataflow.yml --coordinator-addr X.X.X.X
Dora handles:
TCP communication with daemons
Process spawning/monitoring
Log collection
Distributed execution
Your role is purely:
Config generation (YAML dataflow files)
Component wrapper generation (Python files that wrap your components for Dora)
CLI invocation (subprocess.run(['dora', 'start', ...]))
