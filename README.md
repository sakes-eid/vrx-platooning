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

For the current tuned R1 + R2 visual experiment:

```bash
cd ~/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch platoon_bringup follower_stress_visual.launch.py \
  controller_params_file:=~/vrx_ws/src/vrx_platooning/platoon_control/config/r2_follower_v22_tuned.yaml \
  planner_params_file:=~/vrx_ws/src/vrx_platooning/platoon_planner/config/r2_follower_v22_tuned.yaml \
  follower_id:=r2 predecessor_id:=r1
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

R2 now uses the **V2.2 path-gap architecture**. The longitudinal formation controller regulates an along-path bumper gap on the predecessor's unsmoothed breadcrumb history, while the smoothed breadcrumb curve is used only for lateral guidance. Exact oriented hull clearance is reserved for collision warning / avoidance.

The current tuned R2 files are:

| Component | Recommended file |
|---|---|
| Current proactive controller | `platoon_control/platoon_control/proactive_follower_controller_v22.py` |
| Current proactive planner | `platoon_planner/platoon_planner/proactive_follower_planner_v22.py` |
| Tuned controller parameters | `platoon_control/config/r2_follower_v22_tuned.yaml` |
| Tuned planner parameters | `platoon_planner/config/r2_follower_v22_tuned.yaml` |
| V2.2 follower logger | `platoon_monitor/platoon_monitor/follower_logger_v22.py` |
| V2.2 core launch | `platoon_bringup/launch/follower_stress_pathgap_v22.launch.py` |
| Current visual launch | `platoon_bringup/launch/follower_stress_visual.launch.py` |
| R2 visualization | `platoon_monitor/platoon_monitor/r2_stress_viz.py` |
| Combined R1/R2 RViz config | `platoon_bringup/rviz/r1_r2_stress.rviz` |
| Adaptive follower autotuner | `platoon_tuning/platoon_tuning/follower_stress_autotune_v22.py` |

The V2.2 adaptive tuner uses the hierarchy:

```text
GAP -> HEADING / GUIDANCE -> SPEED -> BRAKING
```

A stage is locked when it satisfies its threshold. Locked stages continue to be monitored and are reopened if a later accepted controller causes them to drift outside the same threshold.

The 36-trial tuning study exhausted its trial budget with GAP locked and HEADING still open, but its final accepted controller was visually better than the previous follower and is retained as the current R2 tuned baseline.

Final accepted run headline metrics:

```text
Path-gap RMSE            = 0.173 m
Path-gap P95 abs error   = 0.361 m
Path-gap bias            = -0.027 m
Initial catch-up time    = 21.8 s
Lost-gap episodes        = 0
Minimum hull clearance   = 3.457 m
Collision warnings       = 0
Avoidance activations    = 0

CTE RMSE                 = 1.004 m
CTE P95                  = 1.942 m
Maximum abs CTE          = 2.179 m
Heading RMSE             = 3.164 deg

Speed RMSE               = 0.115 m/s
Speed P95 abs error      = 0.263 m/s

Terminal speed           = 0.014 m/s
Terminal path-gap error  = 0.179 m
Formation settle time    = 2.10 s
```

The heading/guidance acceptance target was deliberately tightened to CTE RMSE <= 0.90 m, so the optimizer did not formally lock that stage before the 36-trial budget ended. The selected controller is therefore a **validated current baseline**, not a claim that every optimizer acceptance gate was satisfied.

Older V2.1 and reactive implementations remain in the repository for development traceability.
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

### R2 V2.2 path-gap experiment

```text
platoon_bringup/launch/follower_stress_pathgap_v22.launch.py
```

Runs the current V2.2 follower architecture without RViz.

### Automated follower tuning

The adaptive V2.2 autotuner launches:

```text
platoon_bringup/launch/follower_stress_pathgap_v22.launch.py
```

and is implemented by:

```text
platoon_tuning/platoon_tuning/follower_stress_autotune_v22.py
```

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

For visual testing of the tuned V2.2 follower:

```bash
cd ~/vrx_ws

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch platoon_bringup follower_stress_visual.launch.py \
  controller_params_file:=~/vrx_ws/src/vrx_platooning/platoon_control/config/r2_follower_v22_tuned.yaml \
  planner_params_file:=~/vrx_ws/src/vrx_platooning/platoon_planner/config/r2_follower_v22_tuned.yaml \
  follower_id:=r2 \
  predecessor_id:=r1 \
  headless:=False \
  show_map:=True
```

For a headless V2.2 run:

```bash
cd ~/vrx_ws

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch platoon_bringup follower_stress_pathgap_v22.launch.py \
  controller_params_file:=~/vrx_ws/src/vrx_platooning/platoon_control/config/r2_follower_v22_tuned.yaml \
  planner_params_file:=~/vrx_ws/src/vrx_platooning/platoon_planner/config/r2_follower_v22_tuned.yaml \
  follower_id:=r2 \
  predecessor_id:=r1 \
  headless:=True
```

Older reactive and V2.1 launches are retained for comparison and development history.
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
platoon_tuning/platoon_tuning/follower_stress_autotune_v22.py
```

It uses Optuna with an adaptive hierarchical schedule:

```text
gap -> heading/guidance -> speed -> braking
```

and supports pause/resume through its checkpoint and adaptive-state files.

Example:

```bash
cd ~/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run platoon_tuning follower_stress_autotune_v22 \
  --study-name r2_follow_adaptive_v22_01 \
  --trials 36 \
  --seed-controller ~/vrx_ws/src/vrx_platooning/platoon_control/config/r2_follower_v22_tuned.yaml \
  --seed-planner ~/vrx_ws/src/vrx_platooning/platoon_planner/config/r2_follower_v22_tuned.yaml
```

To inspect all options:

```bash
ros2 run platoon_tuning follower_stress_autotune_v22 --help
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
    proactive_follower_controller_v22.py

R2 planner:
    proactive_follower_planner_v22.py

R2 tuned controller YAML:
    r2_follower_v22_tuned.yaml

R2 tuned planner YAML:
    r2_follower_v22_tuned.yaml

R2 autotuner:
    follower_stress_autotune_v22.py

R2 visual launch:
    follower_stress_visual.launch.py

R2 V2.2 core launch:
    follower_stress_pathgap_v22.launch.py
```

Unless studying the development history, users should start with these files rather than the older implementations retained in the repository.
