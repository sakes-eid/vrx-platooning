# VRX Platooning

Autonomous multi-vessel platooning project developed with **ROS 2 Jazzy**, **Gazebo / VRX**, and Python using simulated WAM-V surface vehicles.

The project investigates trajectory tracking and cooperative control for a platoon of autonomous surface vessels. The current architecture consists of:

- **R1 — Leader:** follows a predefined trajectory using GPS-based state estimation, local Cartesian planning, and differential-thrust control.
- **R2 — Follower:** follows R1 while regulating inter-vessel spacing and using proactive information about the leader's motion.
- **R3 — Planned:** will extend the same follower architecture so that additional vessels can follow the preceding vessel.

The project includes trajectory planning, GPS-to-local state estimation, differential-thrust control, experiment logging, RViz visualization, stress testing, and **Optuna-based automated controller tuning**.

> **Control constraint:** WAM-V thruster steering angles remain fixed straight. Vessel turning is achieved using differential left/right thrust rather than thruster-angle steering.

---

## Quick Start

Clone the repository into a ROS 2 workspace:

```bash
mkdir -p ~/vrx_ws/src
cd ~/vrx_ws/src

git clone https://github.com/sakes-eid/vrx-platooning.git
```

The project also requires the **VRX simulator** and its dependencies.

After the required packages are present:

```bash
cd ~/vrx_ws

source /opt/ros/jazzy/setup.bash

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install

source install/setup.bash
```

For the current complete R1 experiment:

```bash
cd ~/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch platoon_bringup leader_stress_full.launch.py
```

For the current R1 + R2 visual experiment:

```bash
cd ~/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch platoon_bringup follower_stress_visual.launch.py
```

---

## Recommended / Current Versions

The repository contains older experimental implementations, diagnostics, and development iterations. For users who want to run the current system, the files below should be treated as the **canonical versions**.

### R1 — Leader

R1 is the currently validated leader implementation.

| Component | Recommended file |
|---|---|
| Main controller | `platoon_control/platoon_control/leader_stress_controller.py` |
| Trajectory planner | `platoon_planner/platoon_planner/stress_course_planner.py` |
| Controller parameters | `platoon_control/config/leader_pid.yaml` |
| Full experiment launch | `platoon_bringup/launch/leader_stress_full.launch.py` |
| Core/headless launch | `platoon_bringup/launch/leader_stress_core.launch.py` |
| Tuning launch | `platoon_bringup/launch/leader_stress_tuning.launch.py` |
| Experiment logger | `platoon_monitor/platoon_monitor/r1_stress_logger.py` |
| Visualization | `platoon_monitor/platoon_monitor/r1_stress_viz.py` |
| RViz configuration | `platoon_bringup/rviz/r1_stress.rviz` |
| Latest autotuner | `platoon_tuning/platoon_tuning/r1_stress_autotune_v3.py` |

`r1_stress_autotune_v3.py` is the latest R1 tuning implementation. Earlier versions such as `r1_stress_autotune.py` and `r1_stress_autotune_v2.py` are retained for development history but should not be considered the current tuner.

---

### R2 — Follower

R2 is the current follower implementation.

R2 development is still active, so these are the **current recommended versions**, rather than a permanently frozen release.

| Component | Recommended file |
|---|---|
| Current proactive controller | `platoon_control/platoon_control/proactive_follower_controller_v21.py` |
| Current proactive planner | `platoon_planner/platoon_planner/proactive_follower_planner_v21.py` |
| Basic/reactive follower | `platoon_control/platoon_control/follower_pid_controller.py` |
| Follower logger | `platoon_monitor/platoon_monitor/follower_logger.py` |
| R2 visualization | `platoon_monitor/platoon_monitor/r2_stress_viz.py` |
| Combined R1/R2 RViz config | `platoon_bringup/rviz/r1_r2_stress.rviz` |
| Current follower autotuner | `platoon_tuning/platoon_tuning/follower_stress_autotune_v1.py` |

The non-versioned proactive files:

```text
platoon_control/platoon_control/proactive_follower_controller.py
platoon_planner/platoon_planner/proactive_follower_planner.py
```

represent earlier development stages.

For current R2 testing, prefer the `v21` implementations.

---

## Available R2 Launch Modes

Several launch files are provided because different stages of follower development require different levels of instrumentation.

### Reactive baseline

```text
platoon_bringup/launch/follower_stress_baseline.launch.py
```

Used to establish the performance of the simpler follower architecture before proactive behaviour is enabled.

### Proactive follower

```text
platoon_bringup/launch/follower_stress_proactive.launch.py
```

Runs the proactive follower architecture.

### Current visual experiment

```text
platoon_bringup/launch/follower_stress_visual.launch.py
```

Recommended when visually inspecting R1 and R2 behaviour together.

### R2 v2.1 diagnostics

```text
platoon_bringup/launch/follower_stress_v21_diag.launch.py
```

Used for detailed diagnostics of the current v2.1 follower controller/planner.

### Automated follower tuning

```text
platoon_bringup/launch/follower_stress_tuning.launch.py
```

Used by the follower autotuning framework.

---

# Installation and Requirements

## Core requirements

The project is developed with:

- Ubuntu
- ROS 2 Jazzy
- Gazebo
- VRX
- Python 3
- NumPy
- PyYAML

The VRX simulator must also exist in the ROS 2 workspace.

Official VRX repository:

```text
https://github.com/osrf/vrx
```

A typical workspace layout is:

```text
~/vrx_ws/
├── src/
│   ├── vrx/
│   ├── vrx_gz/
│   └── vrx_platooning/
├── build/
├── install/
└── log/
```

The exact VRX package layout may vary depending on the VRX version being used.

---

## Additional requirement for automated tuning

Automated tuning additionally requires:

```text
Optuna
```

Verify the main Python dependencies with:

```bash
python3 -c "import numpy, yaml; print('Core Python dependencies OK')"
```

For tuning:

```bash
python3 -c "import optuna; print('Optuna OK')"
```

---

# Building the Workspace

Open a terminal and run:

```bash
cd ~/vrx_ws

source /opt/ros/jazzy/setup.bash

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install

source install/setup.bash
```

After changing Python, launch, configuration, URDF, or package files, rebuild before starting another experiment:

```bash
cd ~/vrx_ws

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source install/setup.bash
```

Every new terminal used for ROS commands should also source both ROS 2 and the workspace:

```bash
source /opt/ros/jazzy/setup.bash
source ~/vrx_ws/install/setup.bash
```

---

# Running R1

The recommended R1 experiment is:

```bash
cd ~/vrx_ws

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch platoon_bringup leader_stress_full.launch.py
```

This launches the complete R1 stress-test stack, including the required simulation, state-estimation, planning, control, logging, and visualization components.

To inspect the available launch arguments:

```bash
cd ~/vrx_ws

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch platoon_bringup leader_stress_full.launch.py --show-args
```

For experiments where visualization is unnecessary, the lighter core launch can be used:

```bash
cd ~/vrx_ws

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch platoon_bringup leader_stress_core.launch.py
```

---

# Running R1 + R2

For visual testing of the current follower architecture:

```bash
cd ~/vrx_ws

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch platoon_bringup follower_stress_visual.launch.py
```

For the current v2.1 diagnostic configuration:

```bash
cd ~/vrx_ws

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch platoon_bringup follower_stress_v21_diag.launch.py
```

For a simpler follower baseline:

```bash
cd ~/vrx_ws

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch platoon_bringup follower_stress_baseline.launch.py
```

For proactive follower testing:

```bash
cd ~/vrx_ws

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch platoon_bringup follower_stress_proactive.launch.py
```

---

# Automated Tuning

## R1

The current R1 autotuner is:

```text
platoon_tuning/platoon_tuning/r1_stress_autotune_v3.py
```

It is based on Optuna and automates repeated simulation experiments while evaluating controller and trajectory-tracking performance.

Earlier R1 tuners are kept in the repository for traceability but are not the recommended versions.

---

## R2

The current follower autotuner is:

```text
platoon_tuning/platoon_tuning/follower_stress_autotune_v1.py
```

The follower tuning system is designed to reuse the validated R1 configuration as a starting point for R2 and, later, allow the same architecture to be applied recursively to:

```text
R3 <- R2
R4 <- R3
...
```

The exact tuner command-line options should be checked directly with:

```bash
cd ~/vrx_ws

source /opt/ros/jazzy/setup.bash
source install/setup.bash

python3 \
src/vrx_platooning/platoon_tuning/platoon_tuning/follower_stress_autotune_v1.py \
--help
```

Likewise for R1:

```bash
cd ~/vrx_ws

source /opt/ros/jazzy/setup.bash
source install/setup.bash

python3 \
src/vrx_platooning/platoon_tuning/platoon_tuning/r1_stress_autotune_v3.py \
--help
```

Using `--help` is recommended before launching an automated tuning session because tuning parameters and experiment options may evolve during development.

---

# Important Control Constraint

The WAM-V thruster steering angles are intentionally kept fixed straight.

The controller must therefore **not steer the vessel by changing the thruster angle**.

Turning is produced exclusively through differential thrust:

```text
left thrust > right thrust  -> yaw in one direction

right thrust > left thrust  -> yaw in the opposite direction
```

This constraint applies to both leader and follower development.

---

# Results and Development Files

The repository contains compact JSON result summaries where useful for documenting validation and development.

Large generated data is intentionally excluded, including:

```text
raw CSV telemetry
ROS / Gazebo logs
Optuna databases
intermediate tuning trials
Python cache files
temporary controller backups
```

This keeps the repository focused on the software required to reproduce the experiments rather than storing every individual simulation run.

---

# Which Files Should I Use?

For someone cloning the repository for the first time:

```text
R1 controller:
    leader_stress_controller.py

R1 planner:
    stress_course_planner.py

R1 autotuner:
    r1_stress_autotune_v3.py

R1 normal launch:
    leader_stress_full.launch.py


R2 controller:
    proactive_follower_controller_v21.py

R2 planner:
    proactive_follower_planner_v21.py

R2 autotuner:
    follower_stress_autotune_v1.py

R2 visual launch:
    follower_stress_visual.launch.py

R2 diagnostic launch:
    follower_stress_v21_diag.launch.py
```

Unless studying the development history, users should start with these files rather than the older implementations retained in the repository.
