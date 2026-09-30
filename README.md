# VRX Platooning

Autonomous multi-vessel platooning project developed with **ROS 2 Jazzy**, **Gazebo / VRX**, and Python using simulated WAM-V surface vehicles.

The project investigates trajectory tracking and cooperative control for a platoon of autonomous surface vessels. The current architecture consists of:

- **R1 — Leader:** follows a predefined trajectory using GPS-based state estimation, local Cartesian planning, and differential-thrust control.
- **R2 — Follower:** follows R1 while regulating inter-vessel spacing and using proactive information about the leader's motion.
- **R3 — Follower 2:** follows R2 using the same generic V2.2 follower architecture. R3 receives only predecessor-chain information from R2 and does not subscribe to the global reference path.

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

For the current three-robot visual experiment:

```bash
cd ~/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch platoon_bringup three_robot_stress_visual.launch.py
```

This launches the chained platoon:

```text
R1 -> R2 -> R3
```

R1 is the frozen leader, R2 uses its final V2.2 tune, and R3 follows R2 without access to `/planner/reference_path`.

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

### Q24 distance-definition note

The V2.2 follower regulates a **5 m along-path bumper-to-bumper gap**, not a 5 m center/reference-point Euclidean distance.

For the completed 289.5 s Trial 36 run:

```text
Mean controlled path gap     = 4.973 m
Path-gap RMSE                = 0.173 m

PDF-defined mean d12         = 10.457 m
PDF-defined minimum d12      = 8.267 m
PDF-defined maximum d12      = 17.828 m
```

The handout quantity is:

```text
d12 = sqrt((x2 - x1)^2 + (y2 - y1)^2)
```

With the current WAM-V geometry, a straight aligned 5 m bumper gap corresponds to approximately **10.371 m reference-point separation**:

```text
5.000 + 2.822 + 2.549 = 10.371 m
```

The measured 10.457 m mean Euclidean separation is therefore consistent with the implemented bumper-gap formation. The repository documentation does **not** claim that the literal PDF-defined Euclidean distance converges to 5 m; both definitions are reported explicitly.


Older V2.1 and reactive implementations remain in the repository for development traceability.
---

### R3 — Second Follower / Three-Robot Platoon

The three-robot architecture is implemented as:

```text
R1 -> R2 -> R3
```

R1 remains the frozen leader. R2 remains the frozen V2.2 follower and follows R1. R3 reuses the same generic V2.2 follower planner/controller but is configured with:

```text
follower_id    = r3
predecessor_id = r2
```

R3 has no global reference-path subscription. Its guidance, release, terminal handoff and completion information are propagated through R2.

The main three-robot files are:

| Component | Current file |
|---|---|
| Generic follower controller | `platoon_control/platoon_control/proactive_follower_controller_v22.py` |
| Generic follower planner | `platoon_planner/platoon_planner/proactive_follower_planner_v22.py` |
| R3 seed controller | `platoon_control/config/r3_follower_v22_seed.yaml` |
| R3 seed planner | `platoon_planner/config/r3_follower_v22_seed.yaml` |
| Three-robot smoke launch | `platoon_bringup/launch/three_wamv_smoke.launch.py` |
| Three-robot visual launch | `platoon_bringup/launch/three_robot_stress_visual.launch.py` |
| R3 tuning launch | `platoon_bringup/launch/three_robot_r3_tuning.launch.py` |
| R3 follower logger | `platoon_monitor/platoon_monitor/follower_logger_v22.py` |
| Current R3 autotuner | `platoon_tuning/platoon_tuning/follower_stress_autotune_v23.py` |

### Chained breadcrumb hierarchy

The raw predecessor breadcrumb history is authoritative for longitudinal progress and the 5 m along-path bumper-gap measurement. A smoothed local curve is used only for lateral guidance.

During R3 development, breadcrumb progress was tightened so that after initial acquisition it advances chronologically through local predecessor history. This prevents a follower from jumping to a nonlocal path segment when the stress course passes near itself.

### Synchronized release

R2 publishes a latched release state:

```text
/planner/r2/released
```

R3 uses:

```text
release_mode = predecessor_release_bootstrap
```

so R3 is released with R2 instead of waiting for a second full trail buildup.

Bootstrap guidance is temporary and separate from the authoritative raw R2 breadcrumb history. Once enough genuine R2 history exists, R3 switches to normal chronological breadcrumb following.

### Three-robot terminal behavior

Both followers use:

```text
terminal_behavior = hold
```

R2 brakes and holds behind R1. R3 receives R2 mission state and then brakes and holds behind R2. Pair-success information propagates downstream without either follower reversing or leaving formation.

A complete visual three-robot Gazebo run was completed successfully before R3 autotuning.

### Current R3 tuning checkpoint

R3 uses the cyclic V2.3 Optuna tuner:

```text
4 GAP -> 4 HEADING -> 3 SPEED -> 1 BRAKE -> 3 JOINT -> repeat
```

Locked stages are skipped but continuously checked after accepted candidates. If a later accepted controller degrades a locked stage, that stage is reopened.

The tuner intentionally performs no separate initial or final verification run. The supplied R2-derived R3 seed is protected until a candidate actually satisfies an acceptance condition.

Study:

```text
r3_cyclic_v23_01
```

After the first 30 optimization trials:

```text
BRAKE   = LOCKED
GAP     = OPEN
HEADING = OPEN
SPEED   = OPEN
```

Current best accepted result:

```text
Trial                    = 23
Stage                    = HEADING
Score                    = 31.6835

Mean path gap            = 5.192 m
Path-gap RMSE            = 1.012 m
Path-gap P95 abs error   = 2.670 m
Path-gap bias            = +0.192 m
Initial catch-up         = 33.9 s
Lost-gap time            = 26.0 s
Lost-gap episodes        = 2

CTE RMSE                 = 1.314 m
CTE P95                  = 2.778 m
Maximum abs CTE          = 3.193 m
Heading RMSE             = 5.888 deg

Speed RMSE               = 0.440 m/s
Speed P95 abs error      = 0.827 m/s

Terminal speed           = 0.026 m/s
Terminal path-gap error  = 0.093 m
Formation settle time    = 1.10 s

Minimum hull clearance   = 2.449 m
Collision warnings       = 0
Avoidance activations    = 0
Collision                = false
```

This is an intermediate tuning checkpoint, not the final R3 tune. Only the braking stage has formally locked so far.

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

### R3

The current R3 autotuner is:

```text
platoon_tuning/platoon_tuning/follower_stress_autotune_v23.py
```

It runs the three-robot headless launch:

```text
platoon_bringup/launch/three_robot_r3_tuning.launch.py
```

with R1 and R2 frozen while only R3 is optimized.

V2.3 adds cyclic stage scheduling, locked-stage degradation checks, checkpoint/resume, robust stale-process cleanup, a simulation-time stall watchdog, and successful-summary terminal detection to avoid waiting indefinitely on a terminal callback race.

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

R3 / three-robot platoon:

    generic controller:
        proactive_follower_controller_v22.py

    generic planner:
        proactive_follower_planner_v22.py

    R3 seed controller YAML:
        r3_follower_v22_seed.yaml

    R3 seed planner YAML:
        r3_follower_v22_seed.yaml

    three-robot visual launch:
        three_robot_stress_visual.launch.py

    R3 tuning launch:
        three_robot_r3_tuning.launch.py

    current R3 autotuner:
        follower_stress_autotune_v23.py


Unless studying the development history, users should start with these files rather than the older implementations retained in the repository.
