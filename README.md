# VRX Platooning

Autonomous multi-vessel platooning project developed with **ROS 2 Jazzy**, **Gazebo / VRX**, and Python.

The project investigates trajectory tracking and cooperative control of multiple WAM-V surface vehicles. The current architecture uses one vessel as the trajectory-following leader (**R1**) and develops additional vessels as followers (**R2**, later R3) while maintaining inter-vehicle spacing and stable motion through turns.

The system uses **GPS-based state estimation**, local Cartesian coordinates, trajectory planning, differential-thrust control, experiment logging, visualization, and automated controller tuning.

---

## Current Status

### R1 — Leader

R1 is the reference vehicle and follows a predefined stress-test trajectory.

Implemented features include:

- GPS-based local state estimation
- Straight and curved trajectory generation
- Path look-ahead / turn preview
- Differential-thrust heading control
- Fixed straight thruster steering angles
- Speed and heading control
- Stress-course testing
- Automated parameter tuning with Optuna
- Experiment logging and visualization
- Final validation runs

R1 is currently used as the frozen reference configuration for follower development.

### R2 — Follower

R2 follows R1 rather than directly tracking the global trajectory.

Current R2 development includes:

- Reactive follower controller
- Proactive follower controller
- Inter-vehicle gap regulation
- Leader-motion preview
- Speed-preview diagnostics
- Proactive follower trajectory planning
- Dedicated follower logging
- R1/R2 visualization
- Baseline and manual validation tests
- Generic follower autotuning framework seeded from the validated R1 configuration

R2 is currently undergoing tuning and validation.

### R3 — Planned

The intended architecture is:

```text
Reference trajectory
        |
        v
       R1
        |
        v
       R2
        |
        v
       R3
```

R3 will reuse the follower architecture developed for R2, using R2 as its leader.

---

## Repository Structure

```text
vrx-platooning/
├── platoon_bringup/
│   ├── launch/
│   ├── rviz/
│   └── urdf/
│
├── platoon_control/
│   ├── config/
│   └── platoon_control/
│
├── platoon_interfaces/
│
├── platoon_monitor/
│   └── platoon_monitor/
│
├── platoon_planner/
│   ├── config/
│   └── platoon_planner/
│
├── platoon_state/
│   ├── config/
│   └── platoon_state/
│
├── platoon_tuning/
│   ├── config/
│   └── platoon_tuning/
│
├── docs/
│   ├── PROCESS_HISTORY.md
│   └── QUESTIONS_ANSWERS.md
│
└── results/
```

### Packages

**`platoon_state`**  
Converts simulated GPS and vehicle-state information into a local Cartesian reference frame used by the controllers and planners.

**`platoon_planner`**  
Generates trajectories and follower references, including stress-course trajectories and proactive follower planning.

**`platoon_control`**  
Contains leader and follower controllers. Vessel turning is performed using **differential left/right thrust**, while thruster steering angles remain fixed straight.

**`platoon_monitor`**  
Provides experiment logging, trajectory visualization, follower diagnostics, and R1/R2 monitoring.

**`platoon_tuning`**  
Contains the automated parameter-tuning framework used for R1 and extended for follower tuning.

**`platoon_bringup`**  
Contains launch files, RViz configurations, and vehicle descriptions used to run complete experiments.

---

## Requirements

The project is currently developed using:

- Ubuntu
- ROS 2 Jazzy
- Gazebo
- VRX
- Python 3
- NumPy
- PyYAML
- Optuna
- RViz2

The official VRX simulator is available from:

https://github.com/osrf/vrx

---

## Build

From the ROS 2 workspace:

```bash
cd ~/vrx_ws

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source install/setup.bash
```

---

## Running R1

A full R1 stress-test experiment can be launched with:

```bash
cd ~/vrx_ws

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch platoon_bringup leader_stress_full.launch.py
```

The stress-test framework combines:

- vehicle simulation
- GPS/local-state estimation
- trajectory generation
- leader control
- logging
- visualization

---

## Running R1 + R2

R2 experiments are provided through several launch configurations depending on the test being performed.

Examples include:

```text
follower_stress_baseline.launch.py
follower_stress_proactive.launch.py
follower_stress_tuning.launch.py
follower_stress_v21_diag.launch.py
follower_stress_visual.launch.py
```

For example:

```bash
cd ~/vrx_ws

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch platoon_bringup follower_stress_visual.launch.py
```

---

## Automated Tuning

R1 controller development uses an automated stress-test tuning framework.

The latest R1 tuner is:

```text
platoon_tuning/platoon_tuning/r1_stress_autotune_v3.py
```

Follower tuning is implemented in:

```text
platoon_tuning/platoon_tuning/follower_stress_autotune_v1.py
```

The tuning system uses **Optuna** together with deterministic simulation runs, parameter search spaces, validation metrics, early rejection of poor candidates, and saved experiment summaries.

The follower tuner is designed to allow the same approach to later be reused for **R3 ← R2** tuning.

---

## Results

The repository intentionally does **not** contain the full raw telemetry produced by every simulation.

Large files such as:

- raw trajectory CSV files
- Gazebo logs
- controller logs
- Optuna databases
- intermediate tuning trials
- temporary code backups

are excluded from version control.

Small summary files are retained where useful to document experiment performance and validation.

---

## Documentation

Development history and design decisions are documented in:

```text
docs/PROCESS_HISTORY.md
```

Project questions, experiments, and corresponding answers are documented in:

```text
docs/QUESTIONS_ANSWERS.md
```

---

## Control Constraint

A project requirement is that the WAM-V thruster steering angles remain fixed straight.

Therefore, heading control does **not** steer the left and right thrusters.

Turning is achieved using differential thrust:

```text
Left thrust ≠ Right thrust
        ↓
Yaw moment
        ↓
Vessel turns
```

This constraint is applied throughout the control architecture.

---

## Coordinate System

The vehicles obtain position information from simulated GPS.

GPS coordinates are converted into a local Cartesian coordinate system whose origin is established at the beginning of the experiment.

This allows controllers and planners to operate in metres while retaining GPS as the underlying simulated position source.

---

## Development Goal

The final objective is a reusable autonomous platooning architecture where:

```text
R1 follows the mission trajectory

R2 follows R1

R3 follows R2

...

Rn follows R(n-1)
```

Each follower should maintain stable spacing while anticipating the motion of the vehicle ahead, including acceleration, braking, and turns.

---

## License

A license will be added before the repository is released publicly.

