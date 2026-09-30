
## Phase 1 - State System

### Step 1 - Sensor identification
- GPS source: /wamv/sensors/gps/gps/fix
- GPS type: sensor_msgs/msg/NavSatFix
- GPS update rate: approximately 1.94 Hz
- IMU source: /wamv/sensors/imu/imu/data
- IMU type: sensor_msgs/msg/Imu
- IMU update rate: approximately 10.6 Hz
- Gazebo /wamv/pose is not a global ground-truth position topic.

### Step 2 - Repository structure
- Created vrx_platooning repository.
- Created platoon_state ROS 2 Python package.
- Added docs and results directories.

### Step 3 - Fixed GPS reference frame
- Fixed origin:
  latitude: -33.72275000
  longitude: 150.67400000
- GPS coordinates converted to local metric coordinates.
- Coordinate convention:
  +X = East
  +Y = North
- Test WAM-V position:
  x = -0.813 m
  y = -2.082 m
- Origin is independent of robot spawn position.

### Control constraint
- Thruster steering/rudder angles must remain fixed straight at 0 rad.
- Vehicle steering will use differential left/right thrust only.

## Phase 1 - State System Progress

### Step 4 - Custom VehicleState interface
Created the custom ROS 2 message:

platoon_interfaces/msg/VehicleState.msg

Fields:
- header
- x
- y
- vx
- vy
- speed
- course_angle
- body_yaw

Purpose:
- Provide one unified state message for planner, controller, followers, monitoring and logging.
- Keep actual direction of travel separate from body orientation.

The interface was successfully built and verified using:
ros2 interface show platoon_interfaces/msg/VehicleState

---

### Step 5 - Unified vehicle state node

Created:

platoon_state/vehicle_state_node.py

Inputs:
- /wamv/sensors/gps/gps/fix
- /wamv/sensors/imu/imu/data

Outputs:
- /wamv/state/local_position
- /wamv/state/vehicle

The node performs:

1. GPS latitude/longitude -> local X/Y coordinates in metres.
2. X axis = East.
3. Y axis = North.
4. Velocity estimation from consecutive GPS measurements.
5. Ground speed calculation.
6. Course-angle calculation from inertial velocity.
7. IMU quaternion -> body yaw conversion.

Important distinction:

course_angle
= direction in which the robot is actually travelling.

body_yaw
= direction in which the WAM-V body/bow is pointing.

These values are deliberately kept separate because the robot may drift sideways due to water current, wind or dynamics.

---

### Step 6 - Dynamic state validation

The vehicle-state system was tested while the WAM-V was moving.

Example moving state:

x = 1.803 m
y = 2.371 m

vx = 0.101 m/s
vy = 0.149 m/s

speed = 0.180 m/s

course_angle = 0.977 rad
body_yaw = 1.095 rad

Equivalent approximate angles:

course_angle = 56.0 degrees
body_yaw = 62.7 degrees

This confirmed that:

- GPS-derived position works.
- Velocity increases when the vehicle moves.
- Speed is calculated correctly.
- Course angle represents actual motion.
- Body yaw remains independent of course angle.

---

### Step 7 - Velocity filtering

Decision:
Add a first-order low-pass filter to GPS-derived vx and vy before calculating speed and course angle.

Filter:

filtered_velocity =
alpha * new_velocity
+ (1 - alpha) * previous_filtered_velocity

Initial parameter:

velocity_filter_alpha = 0.35

Reason:
The GPS publishes at approximately 1.94 Hz, so raw position differences can introduce noise into velocity and course estimates.

The filter is applied to vx and vy rather than directly to course_angle in order to avoid angular wraparound problems at +/- pi.

Course angle is only updated when:

speed >= 0.05 m/s

Below this threshold, the last valid course angle is retained because motion direction is not meaningful when the robot is nearly stationary.

Filtering implementation prepared but still requires final rebuild and validation test.

---

## Simulation / Workspace Recovery

During development, the following command was previously executed:

rm -rf install build log

This removed compiled ROS workspace outputs but did not remove source code.

The entire workspace was rebuilt successfully using:

colcon build --symlink-install

Packages successfully rebuilt included:
- vrx_gazebo
- vrx_ros
- control
- planner
- platoon_state
- vrx_gz
- wamv_description
- wamv_gazebo

The old group planner and control packages are still present in the workspace but will not be used as the architecture for the individual project.

---

## Gazebo Resource Path Fix

After the clean rebuild, Gazebo initially failed to resolve WAM-V meshes such as:

model://wamv_description/...
model://wamv_gazebo/...

The issue was fixed by explicitly setting:

GZ_SIM_RESOURCE_PATH

using:

export GZ_SIM_RESOURCE_PATH="$(ros2 pkg prefix wamv_description)/share:$(ros2 pkg prefix wamv_gazebo)/share:$(ros2 pkg prefix vrx_gz)/share:${GZ_SIM_RESOURCE_PATH}"

After this correction, VRX launched successfully with no mesh-loading errors.

---

## Control Constraint

Professor requirement:

Both thruster steering angles must remain fixed straight.

Confirmed from Gazebo:

left thruster initial position = 0 rad
right thruster initial position = 0 rad

Therefore:

/wamv/thrusters/left/pos = 0.0
/wamv/thrusters/right/pos = 0.0

for the entire project.

All steering will be performed using differential thrust:

Left thrust != Right thrust

while both steering angles remain fixed at 0 rad.


## Phase 1 - Final Coordinate System Correction

### World NED requirement

The original local coordinate convention was replaced after clarification from the professor.

Final navigation frame:

- Frame name: world_ned
- X = North [m]
- Y = East [m]
- Positive heading is clockwise from North
- 0 rad = North
- +pi/2 rad = East
- +/-pi rad = South
- -pi/2 rad = West

The previous arbitrary local origin near the WAM-V spawn position is no longer used.

The official Sydney Regatta geographic world origin is used instead:

- Latitude: -33.724223
- Longitude: 150.679736

GPS remains the authoritative navigation source.

Input topic:

/wamv/sensors/gps/gps/fix

The GPS latitude and longitude are converted directly into world NED coordinates.

The Gazebo world pose is not used as the navigation source.

---

### GPS to world NED conversion

The horizontal conversion is:

North = R * delta_latitude

East =
R * cos(origin_latitude)
* delta_longitude

where:

R = 6378137 m

The VehicleState convention is now:

x = North position [m]
y = East position [m]

vx = North velocity [m/s]
vy = East velocity [m/s]

course_angle =
atan2(East velocity, North velocity)

body_yaw =
vehicle bow heading in NED convention

---

### World NED validation

Raw GPS sample:

latitude =
-33.722768757528755

longitude =
150.67399048700395

Converted VehicleState:

frame_id = world_ned

x =
161.88014479808356 m North

y =
-531.973829974782 m East

This agreed with the expected Sydney world position.

---

### Dynamic NED validation

The WAM-V was commanded with equal thrust on both sides while both steering joints remained fixed at 0 rad.

Moving VehicleState sample:

x =
163.12208517731682 m North

y =
-531.1777024033836 m East

vx =
0.6954825701216978 m/s North

vy =
0.43849423051815273 m/s East

speed =
0.8221758908778517 m/s

course_angle =
0.5625368513885165 rad

body_yaw =
0.5628944565140588 rad

The difference between course angle and body heading was approximately 0.00036 rad, confirming that the NED heading and velocity conventions were consistent during straight motion.

---

## Phase 2 - Reference Trajectory Planner

Created ROS 2 package:

platoon_planner

Published topic:

/planner/reference_path

Message type:

nav_msgs/msg/Path

Final planner frame:

world_ned

Coordinate convention:

x = North [m]
y = East [m]

The planner supports the two trajectories required by the project specification:

- straight trajectory
- curved / circular trajectory

---

### Straight trajectory

Initial test path:

Start:
North = 163 m
East = -531 m

End:
North = 183 m
East = -531 m

Waypoint spacing:

0.5 m

The path was successfully published in the world_ned frame and verified using:

ros2 topic echo /planner/reference_path --once

---

### Curved trajectory

A semicircular reference path was implemented and successfully published in world NED coordinates.

Example geometry:

Center:
North = 173 m
East = -531 m

Radius:
10 m

The generated path starts near:

North = 163 m
East = -531 m

passes approximately through:

North = 173 m
East = -521 m

and finishes near:

North = 183 m
East = -531 m

The curved path was successfully verified using:

ros2 topic echo /planner/reference_path --once

---

### Path heading

Each PoseStamped waypoint now contains a heading aligned with the local trajectory direction.

The heading is calculated in NED convention:

heading =
atan2(East delta, North delta)

Therefore:

0 rad = North

+pi/2 rad = East

Waypoint orientation is stored as a quaternion in the world_ned frame.

---

## Phase 2 - Sydney Regatta Visualization

Created ROS 2 package:

platoon_monitor

The old planner occupancy data was reused:

sydney_local_occupancy.npy

The occupancy map was originally generated from the Sydney Regatta shoreline mesh.

Original Gazebo map convention:

- horizontal = East
- vertical = North
- ENU coordinates

The occupancy array is transposed for visualization so the final project map uses:

- horizontal axis = North
- vertical axis = East

This matches the world_ned state and planner coordinate system.

---

### Live map

Created:

platoon_monitor/live_map.py

The live map displays:

- Sydney Regatta shoreline / river boundaries
- planner reference trajectory
- robot position
- robot travelled trail

Axis convention:

X axis = North [m]

Y axis = East [m]

The map refresh period is intentionally limited to:

5 seconds

to reduce computational load while running Gazebo under Ubuntu / WSL.

ROS subscriptions continue receiving data continuously; only the graphical display is refreshed every 5 seconds.

---

### Map validation

The straight NED reference path was successfully displayed on the Sydney Regatta occupancy map.

Observed location:

North approximately 163 to 183 m

East approximately -531 m

The path appeared in the free-water region of the map.

---

### Live GPS robot tracking validation

Gazebo, the GPS-derived state node, the planner and the live map were run together.

The complete data chain was successfully validated:

GPS
-> VehicleState world NED
-> live map

The WAM-V marker appeared at approximately:

North = 162 m

East = -532 m

The vehicle was then manually moved using equal left and right thrust.

The live map correctly updated the vehicle marker and recorded its travelled trail.

This confirmed that:

- GPS position is correctly converted to world NED
- planner and robot state share the same coordinate frame
- the Sydney occupancy map is correctly aligned
- live WAM-V movement is correctly represented on the map

---

## Control Constraint - Final Confirmation

Professor requirement:

Both thruster steering / azimuth angles must remain fixed straight.

Therefore:

/wamv/thrusters/left/pos = 0.0 rad

/wamv/thrusters/right/pos = 0.0 rad

Vehicle turning must use differential thrust only.

The control architecture must therefore generate:

left_thrust

right_thrust

while keeping both thruster steering angles permanently at 0 rad.

---

## Current Project Status

Completed:

- GPS sensor identification
- IMU sensor identification
- GPS-derived world NED state
- NED velocity estimation
- velocity filtering
- NED course angle
- NED body heading
- unified VehicleState message
- dynamic state validation
- straight trajectory planner
- curved trajectory planner
- world NED waypoint headings
- Sydney Regatta occupancy map visualization
- live GPS robot tracking
- 5-second low-load map refresh
- fixed-straight thruster steering constraint validation

Next phase:

Phase 3 - Leader Trajectory Tracking Controller

Planned controller:

- ROS 2 control node
- PID-based trajectory tracking
- planner path input
- VehicleState input
- forward thrust control
- differential thrust heading control
- steering angles fixed at 0 rad
- straight-path tracking test
- curved-path tracking test
- tracking error recording and evaluation


---

## Leader PID optimization and final validation

### Motivation

The initial leader controller was manually selected and then evaluated on both
straight and curved trajectories. The curved trajectory was consistently the
more difficult case and therefore became the primary discriminator during
controller optimization.

The controller was separated into three functional control problems:

- heading/path tracking;
- forward-speed control;
- terminal braking.

A separate RECOVER state remained available if braking overshoot caused the
vehicle to leave the final goal region.

This separation was useful because a poor braking controller did not simply
increase stopping distance: it could force the WAM-V into RECOVER, increasing
mission time and adding recovery penalties to the optimization objective.

### Deterministic automated trials

An automated trial framework was developed around the normal ROS 2 PID
controller.

Each candidate controller was tested in a fresh headless Gazebo simulation.
The simulation was launched paused. The framework waited for:

- Gazebo /clock and required sensor topics;
- vehicle_state_node;
- trajectory_planner;
- trajectory_logger;
- leader_pid_controller.

Only after all required nodes were ready was the simulation unpaused.

This eliminated the startup race observed in earlier experiments and made
repeated trials highly reproducible.

The accelerated project-owned VRX world retained the physical simulation,
including hydrodynamics, buoyancy, vehicle inertia and thruster dynamics.
Only unnecessary visual/sensor workload was reduced. The original VRX files
were not modified.

### Capture-aware tracking metrics

The initial position of the WAM-V is not exactly located on the first reference
point of the planned trajectory.

This is particularly visible at the beginning of the curved trajectory, where
the first samples contain a relatively large position error before the vehicle
has had sufficient time to move onto the reference path.

The robot was intentionally NOT repositioned before each experiment. Moving
the robot or modifying the trajectory purely to reduce this initial error would
alter the common initial condition and artificially improve the reported
tracking statistics.

For this reason, two sets of trajectory metrics were retained:

1. Full-trial metrics, covering the mission from the original spawn condition.
2. Post-capture metrics, beginning after the vehicle first enters and remains
   within the 1.0 m tracking threshold for at least 0.5 s.

The full-trial maximum error in the final validation was approximately 1.43 m,
which corresponds to this initial geometric offset. After capture, the maximum
tracking error was below 1.0 m in all six validation runs.

Therefore the post-capture metric is used to assess steady trajectory-following
performance, while the full-trial metric remains available and is not hidden.

### Initial automated search

The first automated optimizer used staged random exploration for:

1. heading PID and lookahead;
2. speed PID;
3. brake PID;
4. final joint refinement.

This generated useful experimental data, but uniform random sampling did not
make efficient use of previous trials.

After 29 heading-stage evaluations, the best score was:

    1.167604

The best heading gains were also very close to the upper boundaries of the
original search region. This indicated that the original heading search space
was probably too restrictive.

### Smart optimizer

The random optimizer was replaced by an Optuna multivariate TPE optimizer.

The smart optimizer reused the 29 completed heading experiments as prior
observations instead of discarding them.

The heading search range was expanded because the best previous solution was
located close to the former upper limits.

The optimizer then performed staged optimization of:

- heading PID and lookahead;
- speed PID;
- braking PID;
- joint refinement of all optimized parameters.

Candidate selection was model-guided rather than uniform random sampling.

### Curved-first pruning

For each candidate, the curved trajectory was evaluated first.

The combined controller objective is defined using the worst scenario:

    objective = max(curved_score, straight_score)

Therefore, if the curved score alone was already greater than or equal to the
current best complete score, the straight trajectory mathematically could not
make that candidate better.

Such trials were pruned immediately.

This reduced unnecessary simulation work while preserving the optimization
criterion.

### Braking optimization

Brake tuning produced one of the most important behavioral improvements.

Earlier controllers could overshoot the terminal region, enter RECOVER, return
toward the goal and brake a second time.

The brake PID was therefore optimized not only for stopping performance but
also for avoiding unnecessary recovery maneuvers.

The optimized controller reduced the final sequence to:

    TRACK -> BRAKE -> HOLD -> SUCCESS

for both final straight and curved validation trajectories.

No RECOVER state occurred in any of the six final validation experiments.

This also reduced simulated mission duration because successful controllers
spent less time performing corrective maneuvers.

The observed reduction in trial duration was an indirect consequence of
improved tracking, braking and convergence. Simulation runtime itself was not
used as the primary optimization objective.

### Final optimized leader controller

Final parameters:

    Heading:
      Kp = 911.400744
      Ki = 25.651651
      Kd = 98.104932

    Speed:
      Kp = 127.838779
      Ki = 18.383775
      Kd = 49.388692

    Brake:
      Kp = 177.893520
      Ki = 60.634764
      Kd = 10.647183

    Lookahead distance = 5.103256 m

The smart-optimization objective improved from:

    1.167604 -> 0.757925

corresponding to an improvement of approximately 35%.

### Final six-run validation

The final controller was evaluated independently three times on the curved
trajectory and three times on the straight trajectory.

All six experiments completed successfully.

#### Curved trajectory

Run 1:
    Post-capture RMSE       = 0.2999 m
    Post-capture max error  = 0.9856 m
    Heading RMSE            = 0.0370 rad
    Capture time            = 1.496 s
    Recoveries              = 0
    Brake duration          = 3.900 s
    Completion time         = 51.492 s

Run 2:
    Post-capture RMSE       = 0.3000 m
    Post-capture max error  = 0.9857 m
    Heading RMSE            = 0.0370 rad
    Capture time            = 1.444 s
    Recoveries              = 0
    Brake duration          = 3.900 s
    Completion time         = 51.444 s

Run 3:
    Post-capture RMSE       = 0.3003 m
    Post-capture max error  = 0.9856 m
    Heading RMSE            = 0.0368 rad
    Capture time            = 1.500 s
    Recoveries              = 0
    Brake duration          = 3.900 s
    Completion time         = 51.496 s

Curved mean:
    Post-capture RMSE       = 0.3000 m
    Post-capture max error  = 0.9856 m
    Heading RMSE            = 0.0369 rad
    Capture time            = 1.480 s
    Recovery count          = 0 / 3
    Completion time         = 51.477 s

#### Straight trajectory

Run 1:
    Post-capture RMSE       = 0.1702 m
    Post-capture max error  = 0.9732 m
    Heading RMSE            = 0.0179 rad
    Capture time            = 1.352 s
    Recoveries              = 0
    Brake duration          = 1.200 s
    Completion time         = 34.096 s

Run 2:
    Post-capture RMSE       = 0.1751 m
    Post-capture max error  = 0.9998 m
    Heading RMSE            = 0.0184 rad
    Capture time            = 1.296 s
    Recoveries              = 0
    Brake duration          = 1.200 s
    Completion time         = 34.100 s

Run 3:
    Post-capture RMSE       = 0.1698 m
    Post-capture max error  = 0.9731 m
    Heading RMSE            = 0.0180 rad
    Capture time            = 1.348 s
    Recoveries              = 0
    Brake duration          = 1.256 s
    Completion time         = 34.092 s

Straight mean:
    Post-capture RMSE       = 0.1717 m
    Post-capture max error  = 0.9820 m
    Heading RMSE            = 0.0181 rad
    Capture time            = 1.332 s
    Recovery count          = 0 / 3
    Completion time         = 34.096 s

### Final leader acceptance

Acceptance requirements:

    post-capture RMSE <= 0.60 m
    post-capture maximum error <= 1.00 m
    capture time <= 5.0 s
    successful mission completion

Final result:

    Curved validation:  3 / 3 PASS
    Straight validation: 3 / 3 PASS
    Total:              6 / 6 PASS

The leader controller is therefore frozen as the validated baseline for the
leader-follower and platooning stages.


---

## Additional leader stress test — coverage trajectory

Before beginning the multi-robot stage, the final validated leader controller
was tested on a trajectory substantially more complex than the straight and
single-curve validation cases.

### Coverage planner

A coverage planner was implemented using the same general principle as the
original project planner: alternating back-and-forth sweep lines
(boustrophedon / lawnmower coverage).

The new implementation was adapted to the current project architecture:

- output frame: world_ned;
- x coordinate represents North;
- y coordinate represents East;
- output topic remains /planner/reference_path;
- smooth semicircular connectors are used between adjacent sweep lanes;
- the existing tuned leader controller is used without retuning.

Smooth U-turns were intentionally preferred to sharp grid-like transitions so
that the generated trajectory better reflects the turning requirements of the
WAM-V.

### Coverage test 1 — 5 m lane spacing

The initial planner used:

    lane spacing = 5 m
    U-turn radius = 2.5 m

This produced a difficult coverage trajectory containing two consecutive
180-degree turns.

The controller successfully completed the mission but tracking performance
degraded significantly around the U-turns:

    Post-capture position RMSE = 1.086 m
    Maximum position error     = 2.219 m
    Heading RMSE               = 0.394 rad
    Speed RMSE                 = 0.089 m/s
    Recovery count             = 1
    Recovery time              = 9.504 s
    Total completion time      = 100.448 s

The tuned leader lookahead distance is approximately:

    5.10 m

The initial U-turn radius of only:

    2.5 m

was therefore substantially smaller than the controller lookahead distance.

The largest errors occurred during the U-turn transitions rather than during
the straight coverage sweeps.

This showed that trajectory geometry must be compatible with the dynamic and
control characteristics of the vehicle. A controller should not be expected to
compensate indefinitely for arbitrarily tight planner geometry.

### Coverage test 2 — increased turning radius

The lane spacing was increased to:

    12 m

producing:

    U-turn radius = 6 m

The PID gains and all other controller parameters were left unchanged.

The second coverage experiment again reached SUCCESS.

Measured post-capture performance:

    Mean position error        = 0.710 m
    Position RMSE              = 0.813 m
    Maximum position error     = 1.676 m
    Heading RMSE               = 0.203 rad
    Speed RMSE                 = 0.087 m/s
    Capture time               = 1.004 s

Terminal behavior:

    Recovery count             = 1
    Recovery time              = 7.048 s
    Brake duration             = 7.504 s

Mission timing:

    TRACK duration             = 98.500 s
    Total completion time      = 118.000 s

The longer mission duration is expected because increasing the U-turn radius
also increases the total physical coverage-path length.

### Effect of the planner modification

Changing only the coverage geometry produced approximately:

    Position RMSE improvement      = 25%
    Maximum-error improvement      = 24%
    Heading-RMSE improvement       = 48%
    Recovery-time improvement      = 26%

Speed-tracking performance remained nearly unchanged.

When the second test was separated into sweep and turning portions, the
approximate tracking behavior was:

    Straight sweep RMSE  = 0.656 m
    U-turn RMSE          = 0.971 m

Therefore the remaining error is primarily associated with repeated
180-degree direction changes rather than general straight-line instability.

The maximum error of approximately 1.676 m occurred during the second U-turn.

### Interpretation

The coverage trajectory is treated as a stress/generalization test rather than
as part of the PID optimization objective.

The PID controller had been optimized only using the standard straight and
curved validation trajectories. It was intentionally NOT retuned specifically
for coverage, because doing so could over-specialize the validated leader
controller.

Despite this, the WAM-V:

- captured the trajectory;
- completed all three sweep lanes;
- negotiated both 180-degree U-turns;
- reached the final goal;
- achieved SUCCESS.

The only RECOVER event occurred during terminal stopping after the coverage
tracking phase, not during either U-turn.

The experiment therefore demonstrates that the validated controller
generalizes to an unseen multi-turn trajectory while also showing the
importance of planner/controller compatibility.

The final leader controller remains unchanged and is now frozen for the
leader-follower stage.


---

# 2026-09-27 — R1 Leader Finalization / PDF Q1–Q18 Checkpoint

## Purpose

This checkpoint freezes the current Robot 1 (R1) leader implementation before development of Robot 2.

The project PDF is organized progressively as:

Single Robot -> Control -> Trajectory Tracking -> Leader-Follower -> Three-Robot Platoon

Questions Q1-Q18 correspond to the ROS 2 fundamentals, robot modeling/control, and single-robot trajectory-tracking stages.

Q19 begins the two-robot leader-follower stage.

---

## PDF QUESTION STATUS — Q1 TO Q18

### Q1 — Create a ROS 2 workspace
STATUS: COMPLETE

Workspace:

    ~/vrx_ws

Environment:

    Ubuntu 24.04
    ROS 2 Jazzy
    Gazebo Sim / VRX

---

### Q2 — Create a dedicated project package
STATUS: COMPLETE

The authored platooning project is located under:

    ~/vrx_ws/src/vrx_platooning

The architecture has been separated into functional ROS 2 packages including:

    platoon_state
    platoon_planner
    platoon_control
    platoon_monitor
    platoon_bringup
    platoon_tuning

The upstream VRX packages remain separate from the project-specific code.

---

### Q3 — Identify the main ROS 2 concepts used
STATUS: COMPLETE

The implementation uses:

    Nodes
    Topics
    ROS 2 messages
    Publishers
    Subscribers
    Launch files
    ROS parameters

Main information flow for R1:

    VRX sensors
        |
        v
    multi_vehicle_state
        |
        v
    /r1/vehicle_state
        |
        +----------------------+
        |                      |
        v                      v
    stress_course_planner   leader_stress_controller
        |                      |
        | speed/lookahead      |
        +--------------------->|
                               |
                               v
                     left/right differential thrust

The planner and controller therefore operate as separate ROS 2 nodes.

---

### Q4 — Launch a robot in the simulation environment
STATUS: COMPLETE

The VRX WAM-V was successfully launched in the Sydney Regatta Gazebo environment.

The final system can be launched through the project bringup package rather than starting each node manually.

---

### Q5 — Identify robot command and state topics
STATUS: COMPLETE

Important WAM-V command interfaces include:

    /wamv/thrusters/left/thrust
    /wamv/thrusters/right/thrust

The thruster steering positions remain fixed straight.

Turning is performed ONLY through differential left/right thrust, as required by the instructor.

Important state/sensor information includes simulated GPS and IMU data.

The project state-estimation node converts these measurements into the project state topic:

    /r1/vehicle_state

The working navigation frame is:

    world_ned

The controller therefore does not use Gazebo-local position as its primary navigation state.

---

### Q6 — Manually command the robot and verify motion
STATUS: COMPLETE

Manual WAM-V motion and thruster interfaces were investigated during the initial VRX setup.

Subsequent controller tests verified forward motion and differential-thrust turning.

---

### Q7 — Implement the robot model in the simulation environment
STATUS: COMPLETE

The project uses the WAM-V model and its physical dynamics provided by VRX/Gazebo.

The robot motion is represented in the horizontal plane using:

    x / North
    y / East
    psi / yaw

The simulated GPS and IMU provide the measurements used by the project state-estimation layer.

The LIGHT simulation profile removes unnecessary high-cost sensors for control testing but retains the WAM-V physical simulation and required GPS/IMU information.

---

### Q8 — Identify robot inputs and outputs
STATUS: COMPLETE

CONTROL INPUTS:

    Left thruster force
    Right thruster force

Thruster steering angle:

    fixed at 0 rad / straight

Yaw control is therefore generated through:

    T_left != T_right

STATE OUTPUTS USED BY THE CONTROLLER:

    position x
    position y
    yaw / heading
    velocity components
    vehicle speed

These are packaged into the ROS 2 vehicle-state interface.

---

### Q9 — Define a reference trajectory
STATUS: COMPLETE

The trajectory-planning system supports reference-path generation and publishes the reference path for the controller.

A demanding stress trajectory was additionally developed for controller tuning and validation.

It contains:

    long straight section
    90-degree tight turn
    180-degree turns
    directly connected turns
    short connector
    coverage lanes
    tight hairpin
    wider opposite hairpin
    final straight

The stress course deliberately combines straight and strongly curved motion so that the controller is tested under more difficult conditions than a simple isolated line or circle.

Reference frame:

    world_ned

---

### Q10 — Select a suitable trajectory-tracking control method
STATUS: COMPLETE

Selected method:

    PID-based trajectory tracking

The R1 controller contains separate control functions for:

    heading
    forward speed
    braking / terminal stop

The planner additionally generates:

    speed limits
    curvature information
    lookahead distance
    turn preview information

The controller uses differential thrust for heading control and common thrust for longitudinal motion.

---

### Q11 — Briefly justify the selected method from literature
STATUS: PARTIALLY COMPLETE

Engineering justification:

PID control was selected because the WAM-V trajectory-tracking problem can be separated into heading and longitudinal control loops, while differential thrust provides a direct yaw-control mechanism.

The method is computationally lightweight, interpretable, and can be experimentally tuned in simulation.

IMPORTANT:

Formal literature references still need to be added to the final report to fully satisfy Q11.

Do not mark the literature portion complete until references are included.

---

### Q12 — Implement the controller as a ROS 2 node
STATUS: COMPLETE

Main R1 controller:

    platoon_control/
    platoon_control/
    leader_stress_controller.py

ROS executable:

    leader_stress_controller

Node name used by launch system:

    leader_pid_controller

The controller subscribes to vehicle state and planner guidance and publishes WAM-V thruster commands.

---

### Q13 — Test trajectory tracking using a single robot
STATUS: COMPLETE

R1 was extensively tested as a single WAM-V.

Tests progressed from:

    basic movement
    straight path
    waypoint tracking
    curved motion
    chronological waypoint tracking
    terminal stopping
    stress-course tracking

The stress course became the main benchmark used for final controller optimization.

---

### Q14 — Implement the trajectory-tracking node
STATUS: COMPLETE

Trajectory tracking is implemented through the combined planner/controller architecture.

Planner:

    stress_course_planner.py

Controller:

    leader_stress_controller.py

The planner determines the path progression, curvature-aware speed profile and lookahead guidance.

The controller tracks the resulting trajectory using the measured vehicle state.

---

### Q15 — Test the system on straight and curved trajectories
STATUS: FUNCTIONALLY COMPLETE / REPORT EVIDENCE TO ORGANIZE

Straight and curved motion have both been tested during development.

The final stress course contains both long straight portions and multiple curved sections with radii and direction changes considerably more demanding than a simple circular trajectory.

For the final report, dedicated straight and curved figures should still be preserved separately if the instructor expects literal separate demonstrations for Q15.

---

### Q16 — Record simulation data
STATUS: COMPLETE

A dedicated logger records the R1 experiments.

Recorded fields include:

    time
    North position
    East position
    yaw
    vx
    vy
    speed
    controller state
    target speed
    speed error
    desired heading
    heading error
    cross-track error
    planner lookahead
    planner speed limit
    upcoming curvature
    distance to turn
    turn severity
    left thrust
    right thrust
    active waypoint

Each important run produces CSV data and a summary JSON file.

---

### Q17 — Compute the tracking error
STATUS: PARTIALLY COMPLETE

The current monitoring system computes:

    cross-track error
    heading error
    speed error

These metrics were used throughout controller tuning.

The PDF explicitly defines position error as:

    ep = sqrt((x - xd)^2 + (y - yd)^2)

The final report pipeline should therefore also calculate and plot this exact PDF-defined position error using the chosen desired trajectory point.

This should be added even though cross-track error is already available.

---

### Q18 — Evaluate controller performance
STATUS: COMPLETE FOR R1 CONTROL / FINAL REPORT PLOTS STILL TO PREPARE

Final selected autotuning solution:

    Tune 06
    Trial 026
    Stage: joint
    Score: 5.4760059

Tuning-result headline metrics:

    Cross-track RMSE: approximately 0.344 m
    Cross-track P95: approximately 0.575 m
    Maximum cross-track error: approximately 0.934 m
    Heading RMSE: approximately 2.41 deg

A separate final visible Gazebo validation was then performed using the final configuration.

FINAL VISIBLE VALIDATION:

    Result: SUCCESS

    Tracking duration:
        approximately 240.8 s

    Mean vehicle speed:
        0.551 m/s

    Maximum measured speed:
        1.094 m/s

    Cross-track MAE:
        approximately 0.326 m

    Cross-track RMSE:
        0.409 m

    Cross-track P95:
        approximately 0.683 m

    Maximum absolute cross-track error:
        0.980 m

    Heading RMSE:
        2.56 deg

The map showed the measured R1 trajectory remaining close to the reference path through the straight sections, connected turns and hairpins.

The robot reached the terminal portion of the trajectory and entered:

    BRAKE
    HOLD
    SUCCESS

without requiring a recovery state during the final validation.

---

## FINAL R1 CONTROLLER PARAMETERS

### Speed PID

    Kp = 77.49640833032895
    Ki = 16.564166376336193
    Kd = 49.19260599818982

### Heading PID

    Kp = 1388.5269897997275
    Ki = 26.45969942764789
    Kd = 201.59814882921654

### Heading slowdown angle

    0.5283476090300474 rad

### Brake PID

    Kp = 177.8935198208073
    Ki = 60.634764417119634
    Kd = 10.647182804717701

---

## FINAL R1 PLANNER PARAMETERS

    minimum_turn_speed   = 0.5746954085623049
    lateral_accel_limit  = 0.12838915122320071
    accel_limit          = 0.34026720191765825
    decel_limit          = 0.7876767806502798

    lookahead_min        = 2.0921571205428977
    lookahead_max        = 5.322629111496128
    lookahead_speed_gain = 0.8863601910512493

    tight_lookahead      = 1.9166273984905706
    medium_lookahead     = 4.322629111496128

---

## IMPORTANT DEBUGGING RESULT — CONFIGURATION OVERRIDE

After Tune 06, the final gains were initially written into the Python controller defaults.

The first full GUI validation behaved extremely poorly and produced strong lateral oscillation.

The cause was NOT a failure of the Tune 06 optimizer.

The bringup launch file loaded:

    platoon_control/config/leader_pid.yaml

which still contained the original values:

    speed_kp   = 250
    heading_kp = 300
    brake_kp   = 450

These ROS parameters overrode the newly tuned Python defaults.

The YAML configuration was updated with the final Trial 026 values and the platoon_control package was rebuilt.

After correcting the YAML override, R1 returned to stable trajectory tracking.

LESSON:

For all future R2/R3 tuning, verify the parameters loaded by the launch system, not only the defaults inside the Python node.

---

## SIMULATION PERFORMANCE NOTE

The full Gazebo simulation profile caused real-time factor to fall to approximately:

    RTF = 0.09

on the WSL laptop.

This is too slow for useful controller validation.

The final controller validation therefore used the LIGHT simulation profile.

The LIGHT profile keeps the sensors and dynamics required by the controller while avoiding unnecessary high-cost simulation components.

For controller development:

    simulation_profile := light

is therefore the preferred configuration.

---

## R1 DESIGN DECISIONS NOW FROZEN

1. Navigation state is based on simulated GPS/IMU data.

2. Planner/controller coordinates use:

       world_ned

3. Thruster steering angles remain fixed straight.

4. Turning is performed only through differential thrust.

5. R1 uses separate heading, speed and braking PID functions.

6. Planner speed is curvature-aware.

7. Lookahead changes according to speed and upcoming curvature.

8. Waypoint progression is chronological to prevent path-loop jumps.

9. Controller owns authoritative active-waypoint progress.

10. Terminal behavior uses the state sequence:

        TRACK -> BRAKE -> HOLD -> SUCCESS

11. The stress course is retained as the principal R1 robustness benchmark.

12. R1 is now considered sufficiently stable to begin R2 development unless later three-robot testing exposes a specific leader-side problem.

---

## ITEMS STILL NEEDED FOR THE FINAL REPORT

Before submission, retain or generate:

    - literature references supporting the PID choice for Q11
    - dedicated straight-trajectory figure
    - dedicated curved-trajectory figure
    - x(t)
    - y(t)
    - psi(t)
    - PDF-defined position error ep(t)
    - final R1 tracking-error plots
    - controller/planner architecture diagram

These are documentation tasks and do not currently require retuning R1.

---

## NEXT DEVELOPMENT STAGE — PDF Q19

Next project stage:

    Part 4 — Leader-Follower System

Robot 2 will follow Robot 1 while maintaining the required:

    d* = 5 m

Planned R2 architecture:

    predecessor velocity
            |
            v
    base follower speed
            +
    5 m gap error -> distance PID correction
            |
            v
    follower target speed
            |
            v
    speed PID
            |
            v
    forward thrust

Lateral guidance will follow the predecessor's travelled breadcrumb/path rather than simply aiming at the predecessor's instantaneous position.

The 5 m spacing will be controlled primarily as along-path spacing so that hairpins and tight turns do not cause the follower to cut directly across the trajectory.

R2 will be developed before R3.



---

# 2026-09-27 — R1 Documentation Closed

The documentation corresponding to PDF questions Q1-Q18 was reviewed and consolidated into:

    docs/QUESTIONS_ANSWERS.md

Q11 is now supported by literature covering:

    marine heading/autopilot control
    USV PID speed and heading control
    hierarchical guidance + PID control
    adaptive lookahead for USV path following

The final visible R1 validation remains the principal demonstration result:

    SUCCESS
    Cross-track MAE  = 0.326 m
    Cross-track RMSE = 0.409 m
    Cross-track P95  = 0.683 m
    Maximum CTE      = 0.980 m
    Heading RMSE     = 2.56 deg

Important reporting limitation:

The final logger does not explicitly record xd(t) and yd(t). Therefore the exact PDF-defined:

    ep = sqrt((x - xd)^2 + (y - yd)^2)

has not been independently reconstructed from the final CSV.

This will be handled during final report/plot generation and does not require R1 controller retuning.

R1 IMPLEMENTATION STATUS:

    FROZEN

Next stage:

    Q19-Q24
    Two-robot leader-follower development
    R1 -> R2
    desired spacing = 5 m

---

# 2026-09-29 — R2 V2.2 Path-Gap Architecture, Adaptive Tuning, and Current Freeze

## Reason for V2.2

The earlier follower controller used minimum oriented-hull clearance as both the physical safety measure and the nominal 5 m formation-control variable.

This created a geometry problem on tight turns.

Two vessels could have a correct longitudinal formation spacing along the predecessor's travelled path while the Euclidean minimum distance between their oriented hull polygons changed substantially because of the corner geometry.

The distance PID then reacted to a geometric safety distance that was not a clean longitudinal formation coordinate.

This contributed to unnecessary speed correction and poor corner behavior.

V2.2 separates the three concepts:

    RAW breadcrumb path:
        longitudinal formation coordinate

    SMOOTHED breadcrumb path:
        lateral guidance / heading / CTE

    EXACT oriented hull clearance:
        collision warning / avoidance only

This separation is now considered the canonical follower architecture.

---

## V2.2 formation measurement

The planner publishes:

    /planner/r2/path_gap
    /planner/r2/path_gap_valid

The measurement is based on continuous arc-length progress on the unsmoothed predecessor breadcrumb history.

Follower progress is projected onto the current raw breadcrumb segment rather than being quantized to breadcrumb indices.

The current approximation is:

    path_gap
      = leader_s
      - follower_s
      - predecessor_rear_extent
      - follower_front_extent

using:

    predecessor rear extent = 2.822 m
    follower front extent   = 2.549 m

The raw breadcrumb spacing is:

    0.20 m

but continuous segment projection provides sub-breadcrumb resolution.

The implementation does not project the physical rear/front bumper points separately onto the path. It subtracts the known longitudinal extents from center-reference arc-length separation.

This is adequate for the current controller and should not be changed unless a repeatable curvature-specific bias remains after guidance tuning.

---

## V2.2 collision safety

Physical collision safety continues to use exact oriented WAM-V rectangles.

Current thresholds:

    warning clearance   = 0.50 m
    avoidance clearance = 0.20 m
    release clearance   = 0.35 m

A warning does not automatically fail an optimization trial.

Avoidance activation receives a stronger soft penalty.

Actual hull contact is treated as a hard collision and terminates the tuning run with the maximum penalty.

The tuned selected R2 run produced:

    minimum hull clearance = 3.457 m
    collision warnings     = 0
    avoidance activations  = 0
    collision              = false

---

## Adaptive follower autotuner V2.2

New tuner:

    platoon_tuning/platoon_tuning/follower_stress_autotune_v22.py

Hierarchy:

    GAP
      ->
    HEADING / GUIDANCE
      ->
    SPEED
      ->
    BRAKING

The total trial count is a shared maximum budget rather than a fixed allocation per controller.

When one stage reaches threshold it is locked and the next stage immediately receives the remaining budget.

Locked stages continue to be evaluated on every accepted later-stage candidate.

If a later controller causes a previously locked metric to move outside threshold, that earlier stage is reopened and becomes the next stage to tune.

The tuner supports Ctrl+C pause and:

    --resume

through:

    checkpoint.json
    adaptive_state.json
    Optuna SQLite storage

An interrupted candidate is discarded rather than left as a stale RUNNING Optuna trial.

Startup also has a follower-logger timeout and expanded V2.1/V2.2 process cleanup to avoid overlapping stale Gazebo simulations.

---

## Catch-up metric correction

The first adaptive seed run exposed a false acquisition issue.

The V2.2 path-gap signal can be numerically valid while the breadcrumb trail is still being established. During this startup transient it can temporarily be negative.

The first tuner implementation therefore incorrectly treated an early negative gap as "leader acquired" and later interpreted the genuine catch-up transient as losing the leader.

The corrected acquisition condition requires:

    breadcrumb guidance active
    4.75 m <= path gap <= 5.25 m
    continuous residence in band >= 2.0 s

Only after confirmed acquisition does:

    path gap > 5.75 m

count as losing the leader.

This avoids allowing the tunable catchup_distance parameter to define its own performance metric.

---

## Adaptive acceptance thresholds

GAP:

    path-gap RMSE          <= 0.25 m
    path-gap P95 abs error <= 0.50 m
    abs mean bias          <= 0.10 m
    initial catch-up       <= 40 s
    lost-gap time          <= 2 s
    lost-gap episodes      <= 1

HEADING / GUIDANCE:

    CTE RMSE               <= 0.90 m
    CTE P95                <= 2.50 m
    maximum abs CTE        <= 4.00 m
    heading RMSE           <= 2.50 deg

SPEED:

    speed RMSE             <= 0.20 m/s
    speed P95 abs error    <= 0.35 m/s

BRAKING:

    terminal speed         <= 0.10 m/s
    terminal path-gap err  <= 0.75 m
    formation settle time  <= 3.0 s

The heading CTE requirement was deliberately tightened from 1.25 m to:

    0.90 m

because earlier visual tests showed that a small scalar heading error could coexist with a large outside-corner path deviation.

---

## 36-trial study result

Study:

    r2_follow_adaptive_v22_01

The optimizer exhausted the full 36-trial budget.

Final adaptive state:

    GAP     = LOCKED
    HEADING = OPEN
    SPEED   = OPEN
    BRAKE   = OPEN

The study therefore did NOT finish because all stages passed.

It stopped because the maximum trial budget was consumed while continuing to improve the heading/guidance stage.

The final accepted trial was:

    r2_follow_adaptive_v22_01_036_heading

Result:

    SUCCESS

Formation:

    mean path gap              = 4.9730508 m
    path-gap bias              = -0.0269492 m
    path-gap RMSE              = 0.1727075 m
    path-gap P95 abs error     = 0.3609428 m
    maximum abs gap error      = 0.6705460 m

Catch-up:

    initial catch-up time      = 21.8 s
    lost-gap time              = 0.0 s
    lost-gap episodes          = 0

Safety:

    minimum hull clearance     = 3.4573180 m
    collision warning time     = 0.0 s
    avoidance active time      = 0.0 s
    collision                  = false

Guidance:

    CTE RMSE                   = 1.0036579 m
    CTE P95                    = 1.9415105 m
    maximum abs CTE            = 2.1785988 m
    heading RMSE               = 3.1643949 deg

Speed:

    speed RMSE                 = 0.1148075 m/s
    speed P95 abs error        = 0.2628401 m/s

Terminal:

    terminal speed             = 0.0136079 m/s
    terminal path-gap error    = 0.1789193 m
    formation settle time      = 2.10 s

Overall tuner score:

    7.0173683

Interpretation:

- GAP passes the defined acceptance thresholds.
- SPEED metrics already lie inside their intended thresholds even though the formal SPEED stage was never reached.
- BRAKING metrics also lie inside their intended thresholds even though the formal BRAKING stage was never reached.
- HEADING / GUIDANCE improved substantially but did not satisfy the deliberately strict 0.90 m CTE RMSE and 2.50 deg heading-RMSE lock criteria before the trial budget ended.

---

## Current tuned R2 parameters

Frozen configuration files:

    platoon_control/config/r2_follower_v22_tuned.yaml
    platoon_planner/config/r2_follower_v22_tuned.yaml

Gap PID:

    Kp = 1.254616395479059
    Ki = 0.07691598273776513
    Kd = 0.25770214280663706

Catch-up:

    catchup_distance     = 5.345285289922457 m
    catchup_min_speed    = 1.2723843447849745 m/s
    max gap speed corr.  = 0.7765928787144546 m/s

Guidance / heading:

    heading Kp                    = 928.3642353153681
    heading Ki                    = 5.105477736635173
    heading Kd                    = 98.93066529065382
    cross-track heading gain      = 0.2805635257284035
    tangent half window           = 0.40204986097476825
    lookahead distance            = 4.736355241980624 m
    max cross-track correction    = 33.58076064448188 deg

Speed PID:

    Kp = 114.29069944122205
    Ki = 44.09953488498377
    Kd = 111.26620408076914

Brake PID:

    Kp = 177.8935198208073
    Ki = 60.634764417119634
    Kd = 10.647182804717701

Planner preview:

    historical_preview_decel     = 0.2646349942459316
    historical_preview_max_speed = 1.5 m/s

---

## Visual validation

The selected tuned configuration was loaded into:

    follower_stress_visual.launch.py

with R1 frozen.

Visual comparison showed that R2 followed the predecessor path better than the previous pre-tuning configuration, especially through the difficult curved portions.

The controller is therefore retained as the current R2 V2.2 tuned baseline.

This does NOT mean the heading/guidance optimizer acceptance gate was met. It means the selected configuration is the best currently tested two-robot result and is suitable for continuing development.

---

## PDF Q19-Q24 status after V2.2

    Q19 two robots                     COMPLETE
    Q20 leader/follower ROS 2 arch.    COMPLETE
    Q21 leader state publication       COMPLETE
    Q22 follower uses leader info      COMPLETE
    Q23 follower controller            COMPLETE
    Q24 5 m formation verification     COMPLETE WITH DEFINITION NOTE

## Final Q24 Euclidean-distance check

After the tuned R2 visual validation, the handout-defined Euclidean quantity was calculated from the completed 289.5 s Trial 36 CSV:

    d12 = sqrt((x2 - x1)^2 + (y2 - y1)^2)

The reconstruction used the logged predecessor/follower North and East positions and is equivalent to the logger's reference_distance_m field.

Dataset:

    total rows                = 2896
    mission duration          = 289.5 s
    FOLLOW samples            = 2829
    steady FOLLOWING samples  = 2711

Steady-FOLLOWING Euclidean result:

    mean d12                  = 10.4570449 m
    minimum d12               = 8.2674616 m
    maximum d12               = 17.8281911 m

Relative to the handout's literal 5 m reference-point target:

    mean error                = +5.4570449 m
    RMSE                      = 5.5549817 m
    P95 absolute error        = 6.6246705 m

This result is NOT documented as Euclidean convergence to 5 m.

The V2.2 controller instead regulates a 5 m front-to-rear bumper gap along the predecessor's travelled path.

With the current WAM-V collision geometry:

    predecessor rear extent   = 2.822 m
    follower front extent     = 2.549 m

a straight aligned 5 m bumper gap corresponds to:

    reference separation
      = 5.000 + 2.822 + 2.549
      = 10.371 m

The measured mean Euclidean d12 of 10.457 m is therefore consistent with the implemented longitudinal formation geometry.

A literal 5 m reference-point separation would be smaller than the combined 5.371 m longitudinal half-extents and would imply overlap in the straight-aligned collision model.

The generated plot:

    q24_pdf_d12_full_trial.png

is retained as direct evidence of this definition difference.

Final reporting rule:

    Do not claim that PDF-defined Euclidean d12 converges to 5 m.

Instead report both:

    1. 5 m along-path bumper gap used for control;
    2. Euclidean reference-point d12(t) required by the handout;

and explain the geometry/definition discrepancy explicitly.

---

## Current project status

R1:

    FROZEN

R2:

    V2.2 CURRENT TUNED BASELINE

Next implementation stage:

    Q25-Q32
    R1 -> R2 -> R3

Before R3 is considered complete, remaining R2-specific hard-coded launch/topic assumptions and the follower terminal-target logic must be generalized so that R3 follows R2 rather than implicitly depending on R1.

---

# 2026-09-30 — R3 Chained Platooning, Release Synchronization, and V2.3 Tuning

## Objective

The validated two-robot system was extended from:

    R1 -> R2

to:

    R1 -> R2 -> R3

The required information hierarchy is:

    R1 = leader with global/reference trajectory
    R2 = follower of R1
    R3 = follower of R2

A key design rule is that R3 must not receive the global reference path. The existing V2.2 follower architecture was therefore generalized for arbitrary predecessor/follower IDs instead of creating a separate R3 control algorithm.

---

## Generic follower terminal handoff

The follower planner was generalized so its terminal handoff can come from either:

    reference_path
        for R2 <- R1

or:

    predecessor_mission
        for R3 <- R2

R3 therefore uses R2 mission state and pair success rather than the leader's global reference trajectory.

Relevant commits:

    05aa58f  Generalize follower terminal handoff for chained platooning
    ddf72b0  Add platoon hold terminal mode and three-WAM-V smoke launch

---

## Three-robot terminal hold

The two-robot parking sequence is not suitable for the middle vehicle of a three-robot chain because R3 still depends on R2.

A dedicated terminal behavior was added:

    terminal_behavior = hold

In the three-robot experiment:

    R1 reaches its final target and stops
    R2 performs FORMATION_BRAKE -> FORMATION_HOLD
    R3 performs FORMATION_BRAKE -> FORMATION_HOLD

R2 and R3 remain in the platoon rather than reversing or parking.

Pair-success information propagates downstream:

    /r1/success
        ->
    /r2/pair_success
        ->
    /r3/pair_success

---

## Three-WAM-V simulation stack

The three simulated models are:

    R1 = wamv
    R2 = wamv2
    R3 = wamv3

Nominal spawn coordinates:

    R1 = (-532, 162)
    R2 = (-520, 162)
    R3 = (-508, 162)

Each robot has an independent GPS/IMU state-estimation node:

    /r1/vehicle_state
    /r2/vehicle_state
    /r3/vehicle_state

The main three-robot launches are:

    three_wamv_smoke.launch.py
    three_robot_stress_visual.launch.py
    three_robot_r3_tuning.launch.py

R2 is visualized in orange and R3 in blue.

The professor's steering constraint remains unchanged:

    thruster steering angles fixed straight
    yaw generated only by differential left/right thrust

---

## Breadcrumb hierarchy problem

Initial R3 testing exposed a follower-path hierarchy problem on self-near portions of the stress course.

The follower could re-project onto a geometrically nearby but chronologically incorrect section of predecessor history.

This is especially dangerous for chained platooning because R3 must follow the path R2 actually travelled, in order.

An early attempted hierarchy implementation was rejected and reverted:

    5cb80a3  initial hierarchy attempt
    8abf353  revert

The final hierarchy correction was implemented in three steps:

    0722b2c  Lock follower planner to breadcrumb chronology after acquisition
    f0ac93f  Restrict follower guidance to local breadcrumb chronology
    9509cc2  Centralize follower breadcrumb progress updates

The resulting policy is:

    startup:
        one-time acquisition of the correct trail segment

    after acquisition:
        chronological progress only

    steering:
        local smoothed path around current chronological progress

    longitudinal spacing:
        raw unsmoothed predecessor breadcrumb chain

A diagnostic run showed progress advancing from approximately:

    99 -> 381

with:

    maximum forward index jump = 2
    backward jumps             = 0
    large nonlocal jumps       = 0

A later visual test through the self-near loop behaved correctly.

---

## Synchronized R2/R3 release

Waiting for a second complete breadcrumb trail before releasing R3 produced an undesirable delay.

A latched release state was introduced:

    /planner/{follower_id}/released

For the three-robot chain:

    /planner/r2/released

is consumed by R3 using:

    release_mode = predecessor_release_bootstrap

Relevant commits:

    a150adf  Publish follower release state for chained platooning
    4f90139  Add synchronized release bootstrap for chained followers
    643486a  Synchronize R3 release with R2

Measured release timing during validation:

    R2 release ~= 28.267 s
    R3 release ~= 28.383 s
    difference ~= 0.116 s

This achieved the intended near-simultaneous follower release.

---

## Bootstrap-history correction

The first synchronized-release implementation inserted a synthetic straight R3-to-R2 segment into the raw breadcrumb history.

Although release timing was correct, the synthetic segment contaminated the authoritative predecessor path and later produced incorrect visual behavior.

The design was corrected in:

    8dbda59  Separate synchronized release from real breadcrumb history

Current bootstrap policy:

    temporary local bootstrap guidance
        is NOT stored as
    authoritative raw predecessor breadcrumb history

R3 begins moving from local R2/R3 pair geometry while genuine R2 history accumulates independently.

Once enough genuine R2 trail exists:

    bootstrap terminates
    real trail acquisition occurs
    chronological breadcrumb following becomes authoritative

Post-correction diagnostics confirmed that R3 acquired genuine R2 history and continued normally.

---

## Visual three-robot validation

After the hierarchy and release corrections, a complete visual Gazebo run was performed using:

    three_robot_stress_visual.launch.py

Observed behavior matched the intended architecture:

    R1 followed the stress trajectory
    R2 followed R1
    R3 followed R2
    R3 had no global path subscription
    both followers released together
    both followers remained in chronological predecessor history
    terminal behavior ended in formation hold

This closed the main architecture/debugging phase and allowed R1/R2 to remain frozen while R3 tuning began.

---

## R3 tuning infrastructure

R3 is initialized from the final R2 V2.2 gains because both followers use the same WAM-V dynamics and the same generic follower algorithm.

Seed files:

    platoon_control/config/r3_follower_v22_seed.yaml
    platoon_planner/config/r3_follower_v22_seed.yaml

The dedicated tuning launch is:

    three_robot_r3_tuning.launch.py

It freezes:

    R1 leader
    R2 final V2.2 follower

and makes only:

    R3

tunable.

The R3 logger records the R3 <- R2 pair and terminal sequence.

Infrastructure commits:

    b23302a  Add modular three-robot R3 tuning launch
    36879a7  Clean up three-robot R3 tuning launches between trials

---

## V2.3 cyclic tuner

A new tuner version was created rather than overwriting V2.2:

    follower_stress_autotune_v23.py

This preserves R2's historical tuning implementation and evidence.

Relevant commits:

    28da1d6  Add cyclic adaptive follower tuner v23
    0829990  Finalize cyclic tuner v23 bookkeeping

The V2.3 schedule is:

    GAP     x4
    HEADING x4
    SPEED   x3
    BRAKE   x1
    JOINT   x3
    repeat

The same acceptance thresholds used for R2 are reused for R3.

Locked stages are skipped in their normal slots but are checked after every accepted candidate. If an accepted controller causes a locked stage to fail the same threshold, that stage is reopened.

There is intentionally:

    no initial verification simulation
    no final verification simulation

The supplied seed is the starting working configuration.

---

## Tuner self-termination bug

The initial cleanup routine searched for:

    three_robot_r3_tuning.launch.py

using:

    pkill -f

The tuner command line itself contained:

    --launch-file three_robot_r3_tuning.launch.py

so the tuner killed itself before Trial 1 could run.

The cleanup match was narrowed to the actual ROS launch process.

Fix:

    c2c4c0d  Prevent cyclic tuner from killing itself

---

## Seed-baseline protection

Removing the initial verification run exposed a second logic issue.

At startup:

    best score  = infinity
    best result = none

The first successful random candidate could therefore replace the R2-derived seed even when it failed the active stage threshold.

The first observed GAP candidate demonstrated this behavior.

The acceptance logic was corrected so that, before a working accepted result exists:

    ordinary stage:
        candidate must pass that stage's threshold

    JOINT:
        candidate must make at least one currently open stage pass

Only after a valid accepted working result exists do normal score/improvement rules apply.

Fix:

    755ed69  Protect seed baseline in cyclic tuner

---

## Terminal-race and simulation-stall robustness

Unattended tuning exposed two separate long-wait conditions.

### Real simulation/logging stall

One trial reached approximately:

    287.364 s simulated time

and stopped advancing.

The old tuner waited until the full:

    2400 s wall timeout

A simulation-progress watchdog was added:

    stall_wall_timeout = 120 s

If follower CSV simulation time does not advance for the configured wall period:

    termination_reason = SIM_STALL

and the tuner continues to the next candidate.

### Successful terminal callback race

Another trial had actually completed.

The launch log showed:

    R2 FORMATION_HOLD
    R3 FORMATION_HOLD
    R1 SUCCESS
    R2 SUCCESS latched by R3
    R3 follower summary written

However follower_logger_v22 finalizes as soon as pair success arrives.

The final CSV timer row can therefore be written immediately before predecessor_success becomes true.

The tuner previously waited indefinitely even though the authoritative summary JSON already reported success.

V2.3 now treats:

    follower summary success = true

as an authoritative completion signal.

When this fallback is used, the final CSV row is treated as the predecessor-success instant for offline terminal scoring.

Fix:

    96e8bc0  Make cyclic tuner robust to terminal races and stalls

---

## First 30-trial R3 V2.3 checkpoint

Study:

    r3_cyclic_v23_01

Completed optimization trials:

    30

Final lock state:

    GAP     = OPEN
    HEADING = OPEN
    SPEED   = OPEN
    BRAKE   = LOCKED

The study exhausted the requested 30-trial budget. It did NOT finish because all stages locked.

Current best accepted trial:

    r3_cyclic_v23_01_023_heading

Stage:

    HEADING

Score:

    31.6835029

Formation:

    mean path gap              = 5.192119 m
    path-gap bias              = +0.192119 m
    path-gap RMSE              = 1.011707 m
    path-gap P95 abs error     = 2.669615 m
    maximum abs gap error      = 4.304129 m

Catch-up:

    initial catch-up time      = 33.9 s
    lost-gap time              = 26.0 s
    lost-gap episodes          = 2

Safety:

    minimum hull clearance     = 2.448537 m
    collision warnings         = 0
    avoidance activations      = 0
    collision                  = false

Guidance:

    CTE RMSE                   = 1.314310 m
    CTE P95                    = 2.778180 m
    maximum abs CTE            = 3.192600 m
    heading RMSE               = 5.887856 deg

Speed:

    speed RMSE                 = 0.439656 m/s
    speed P95 abs error        = 0.827393 m/s

Terminal:

    terminal speed             = 0.025904 m/s
    terminal path-gap error    = 0.092779 m
    formation settle time      = 1.10 s

The brake stage satisfies all current brake thresholds and is formally locked.

Gap, heading/guidance and speed remain outside their complete acceptance sets and therefore remain open.

Current best controller differs from the R2 seed mainly in heading/guidance and braking parameters. The R2-derived gap and speed gains remain unchanged in the current best accepted result.

The current best is an intermediate R3 tuning result, not the final frozen R3 configuration.

---

## PDF Q25-Q32 status at this checkpoint

    Q25 add third robot                        COMPLETE
    Q26 create required ROS 2 nodes           COMPLETE
    Q27 define inter-robot topics             COMPLETE
    Q28 implement leader-follower strategy    COMPLETE
    Q29 R2 follows R1                         COMPLETE
    Q30 R3 follows R2                         COMPLETE
    Q31 straight platooning test              FUNCTIONALLY TESTED / FINAL REPORT EVIDENCE PENDING
    Q32 curved platooning test                FUNCTIONALLY TESTED / FINAL REPORT EVIDENCE PENDING

The combined stress trajectory contains both long straight segments and demanding curved/hairpin segments, so both behaviors have already been exercised in the three-robot visual and tuning runs.

Dedicated final quantitative Q31/Q32 evidence will be generated after the R3 tuning configuration is frozen.

As with Q24, final reporting must distinguish:

    implemented control variable:
        5 m along-path bumper-to-bumper gap

from:

    handout analysis variable:
        Euclidean reference-point d12(t) / d23(t)

The documentation must not claim that the Euclidean reference-point distances converge to 5 m unless a later implementation changes the formation definition.

---

## Current project state

R1:

    FROZEN validated leader

R2:

    FROZEN V2.2 tuned follower

R3:

    architecture complete
    visual chained-platoon validation complete
    V2.3 tuning in progress
    brake stage locked
    gap / heading / speed stages still open

Next technical step:

    continue the existing r3_cyclic_v23_01 Optuna study
    from the saved 30-trial checkpoint
