import math

import rclpy

from rclpy.node import Node

from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import Path

from std_msgs.msg import (
    Bool,
    Float64,
    Int32,
    String,
)

from platoon_interfaces.msg import VehicleState


# =============================================================
# Utility functions
# =============================================================


def clamp(
    value,
    minimum,
    maximum,
):

    return max(
        minimum,
        min(
            maximum,
            value,
        )
    )


def wrap_angle(angle):

    while angle > math.pi:

        angle -= (
            2.0 * math.pi
        )

    while angle < -math.pi:

        angle += (
            2.0 * math.pi
        )

    return angle


# =============================================================
# Generic PID controller
# =============================================================


class PID:

    def __init__(
        self,
        kp,
        ki,
        kd,
        integral_limit,
    ):

        self.kp = kp

        self.ki = ki

        self.kd = kd

        self.integral_limit = abs(
            integral_limit
        )

        self.integral = 0.0

        self.previous_error = None

    # ---------------------------------------------------------

    def reset(self):

        self.integral = 0.0

        self.previous_error = None

    # ---------------------------------------------------------

    def update(
        self,
        error,
        dt,
    ):

        if dt <= 0.0:

            derivative = 0.0

        else:

            self.integral += (
                error * dt
            )

            self.integral = clamp(
                self.integral,
                -self.integral_limit,
                self.integral_limit,
            )

            if self.previous_error is None:

                derivative = 0.0

            else:

                derivative = (
                    error
                    - self.previous_error
                ) / dt

        self.previous_error = (
            error
        )

        return (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )


# =============================================================
# Leader PID controller
# =============================================================


class LeaderPIDController(Node):

    # ---------------------------------------------------------
    # Main controller states
    # ---------------------------------------------------------

    TRACK = 'TRACK'

    BRAKE = 'BRAKE'

    RECOVER = 'RECOVER'

    HOLD = 'HOLD'

    SUCCESS = 'SUCCESS'

    # =========================================================
    # Initialization
    # =========================================================

    def __init__(self):

        super().__init__(
            'leader_pid_controller'
        )

        # =====================================================
        # Parameter defaults
        # =====================================================

        defaults = {

            # -------------------------------------------------
            # Topics
            # -------------------------------------------------

            'state_topic':
                '/wamv/state/vehicle',

            'path_topic':
                '/planner/reference_path',

            'left_thrust_topic':
                '/wamv/thrusters/left/thrust',

            'right_thrust_topic':
                '/wamv/thrusters/right/thrust',

            'left_position_topic':
                '/wamv/thrusters/left/pos',

            'right_position_topic':
                '/wamv/thrusters/right/pos',

            'success_topic':
                '/experiment/success',

            'controller_state_topic':
                '/experiment/controller_state',

            # -------------------------------------------------
            # Timing
            # -------------------------------------------------

            'control_rate':
                10.0,

            # -------------------------------------------------
            # Normal trajectory tracking
            # -------------------------------------------------

            'target_speed':
                1.0,

            'lookahead_distance':
                3.0,

            'goal_tolerance':
                1.0,

            'heading_slowdown_angle':
                0.5283476090300474,

            # -------------------------------------------------
            # Stress-course integration ONLY
            #
            # These parameters extend the proven controller with
            # chronological path progress and planner advisories.
            # The original BRAKE / RECOVER / HOLD / SUCCESS state
            # machine below is intentionally left unchanged.
            # -------------------------------------------------

            'stress_state_topic':
                '/r1/vehicle_state',

            'planner_speed_limit_topic':
                '/planner/r1/speed_limit',

            'planner_lookahead_topic':
                '/planner/r1/lookahead_distance',

            'max_planner_speed':
                2.0,

            'waypoint_capture_radius':
                0.45,

            'progress_corridor_m':
                3.5,

            'max_waypoint_advances_per_cycle':
                8,

            # -------------------------------------------------
            # Speed PID
            # -------------------------------------------------

            'speed_kp':
                77.49640833032895,

            'speed_ki':
                16.564166376336193,

            'speed_kd':
                49.19260599818982,

            'speed_integral_limit':
                3.0,

            # -------------------------------------------------
            # Heading PID
            # -------------------------------------------------

            'heading_kp':
                1388.5269897997275,

            'heading_ki':
                26.45969942764789,

            'heading_kd':
                201.59814882921654,

            'heading_integral_limit':
                1.5,

            # -------------------------------------------------
            # Brake PID
            # -------------------------------------------------

            'brake_kp':
                177.8935198208073,

            'brake_ki':
                60.634764417119634,

            'brake_kd':
                10.647182804717701,

            'brake_integral_limit':
                1.0,

            # -------------------------------------------------
            # General thruster limits
            # -------------------------------------------------

            'max_forward_thrust':
                700.0,

            'max_turn_thrust':
                400.0,

            'max_total_thrust':
                1000.0,

            # -------------------------------------------------
            # Braking
            # -------------------------------------------------

            'max_brake_thrust':
                500.0,

            'stop_speed_tolerance':
                0.05,

            # -------------------------------------------------
            # Recovery
            # -------------------------------------------------

            'recovery_target_speed':
                0.25,

            'recovery_brake_distance':
                0.75,

            'recovery_align_tolerance':
                0.15,

            'recovery_realign_tolerance':
                0.35,

            'recovery_max_forward_thrust':
                250.0,

            'recovery_max_turn_thrust':
                250.0,

            # -------------------------------------------------
            # Hold / success
            # -------------------------------------------------

            'hold_duration':
                5.0,
        }

        # =====================================================
        # Declare parameters
        # =====================================================

        for name, value in defaults.items():

            self.declare_parameter(
                name,
                value,
            )

        # =====================================================
        # Read topic parameters
        # =====================================================

        self.state_topic = (
            self.get_parameter(
                'state_topic'
            ).value
        )

        self.path_topic = (
            self.get_parameter(
                'path_topic'
            ).value
        )

        self.left_thrust_topic = (
            self.get_parameter(
                'left_thrust_topic'
            ).value
        )

        self.right_thrust_topic = (
            self.get_parameter(
                'right_thrust_topic'
            ).value
        )

        self.left_position_topic = (
            self.get_parameter(
                'left_position_topic'
            ).value
        )

        self.right_position_topic = (
            self.get_parameter(
                'right_position_topic'
            ).value
        )

        self.success_topic = (
            self.get_parameter(
                'success_topic'
            ).value
        )

        self.controller_state_topic = (
            self.get_parameter(
                'controller_state_topic'
            ).value
        )

        # =====================================================
        # Stress-course integration topics
        # =====================================================

        self.stress_state_topic = (
            self.get_parameter(
                'stress_state_topic'
            ).value
        )

        self.planner_speed_limit_topic = (
            self.get_parameter(
                'planner_speed_limit_topic'
            ).value
        )

        self.planner_lookahead_topic = (
            self.get_parameter(
                'planner_lookahead_topic'
            ).value
        )

        # =====================================================
        # Read numeric parameters
        # =====================================================

        self.control_rate = float(
            self.get_parameter(
                'control_rate'
            ).value
        )

        self.target_speed = float(
            self.get_parameter(
                'target_speed'
            ).value
        )

        self.lookahead_distance = float(
            self.get_parameter(
                'lookahead_distance'
            ).value
        )

        self.goal_tolerance = float(
            self.get_parameter(
                'goal_tolerance'
            ).value
        )

        self.heading_slowdown_angle = float(
            self.get_parameter(
                'heading_slowdown_angle'
            ).value
        )

        self.max_forward_thrust = float(
            self.get_parameter(
                'max_forward_thrust'
            ).value
        )

        self.max_turn_thrust = float(
            self.get_parameter(
                'max_turn_thrust'
            ).value
        )

        self.max_total_thrust = float(
            self.get_parameter(
                'max_total_thrust'
            ).value
        )

        self.max_brake_thrust = float(
            self.get_parameter(
                'max_brake_thrust'
            ).value
        )

        self.stop_speed_tolerance = float(
            self.get_parameter(
                'stop_speed_tolerance'
            ).value
        )

        self.recovery_target_speed = float(
            self.get_parameter(
                'recovery_target_speed'
            ).value
        )

        self.recovery_brake_distance = float(
            self.get_parameter(
                'recovery_brake_distance'
            ).value
        )

        self.recovery_align_tolerance = float(
            self.get_parameter(
                'recovery_align_tolerance'
            ).value
        )

        self.recovery_realign_tolerance = float(
            self.get_parameter(
                'recovery_realign_tolerance'
            ).value
        )

        self.recovery_max_forward_thrust = float(
            self.get_parameter(
                'recovery_max_forward_thrust'
            ).value
        )

        self.recovery_max_turn_thrust = float(
            self.get_parameter(
                'recovery_max_turn_thrust'
            ).value
        )

        self.hold_duration = float(
            self.get_parameter(
                'hold_duration'
            ).value
        )

        self.max_planner_speed = float(
            self.get_parameter(
                'max_planner_speed'
            ).value
        )

        self.waypoint_capture_radius = float(
            self.get_parameter(
                'waypoint_capture_radius'
            ).value
        )

        self.progress_corridor_m = float(
            self.get_parameter(
                'progress_corridor_m'
            ).value
        )

        self.max_waypoint_advances_per_cycle = int(
            self.get_parameter(
                'max_waypoint_advances_per_cycle'
            ).value
        )

        # Planner values fall back to the original tuned settings until
        # the first advisory arrives.
        self.planner_target_speed = min(
            self.max_planner_speed,
            self.target_speed,
        )

        self.planner_lookahead_distance = (
            self.lookahead_distance
        )

        # =====================================================
        # Normal speed PID
        # =====================================================

        self.speed_pid = PID(

            kp=float(
                self.get_parameter(
                    'speed_kp'
                ).value
            ),

            ki=float(
                self.get_parameter(
                    'speed_ki'
                ).value
            ),

            kd=float(
                self.get_parameter(
                    'speed_kd'
                ).value
            ),

            integral_limit=float(
                self.get_parameter(
                    'speed_integral_limit'
                ).value
            ),
        )

        # =====================================================
        # Normal heading PID
        # =====================================================

        self.heading_pid = PID(

            kp=float(
                self.get_parameter(
                    'heading_kp'
                ).value
            ),

            ki=float(
                self.get_parameter(
                    'heading_ki'
                ).value
            ),

            kd=float(
                self.get_parameter(
                    'heading_kd'
                ).value
            ),

            integral_limit=float(
                self.get_parameter(
                    'heading_integral_limit'
                ).value
            ),
        )

        # =====================================================
        # Dedicated braking PID
        #
        # Controlled variable:
        #
        #     signed surge velocity
        #
        # Setpoint:
        #
        #     0 m/s
        #
        # During BRAKE its output is clamped to:
        #
        #     [-max_brake_thrust, 0]
        #
        # so BRAKE can never accelerate the boat forward.
        # =====================================================

        self.brake_pid = PID(

            kp=float(
                self.get_parameter(
                    'brake_kp'
                ).value
            ),

            ki=float(
                self.get_parameter(
                    'brake_ki'
                ).value
            ),

            kd=float(
                self.get_parameter(
                    'brake_kd'
                ).value
            ),

            integral_limit=float(
                self.get_parameter(
                    'brake_integral_limit'
                ).value
            ),
        )

        # =====================================================
        # ROS subscriptions
        # =====================================================

        # The stress launch publishes the authoritative GPS/IMU-derived
        # R1 state on /r1/vehicle_state.  Keep the original state_topic
        # parameter untouched for the production controller; this stress
        # derivative listens to its dedicated topic.
        self.create_subscription(
            VehicleState,
            self.stress_state_topic,
            self.state_callback,
            10,
        )

        path_qos = QoSProfile(
            depth=1
        )
        path_qos.reliability = (
            ReliabilityPolicy.RELIABLE
        )
        path_qos.durability = (
            DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            path_qos,
        )

        self.create_subscription(
            Float64,
            self.planner_speed_limit_topic,
            self.planner_speed_limit_callback,
            10,
        )

        self.create_subscription(
            Float64,
            self.planner_lookahead_topic,
            self.planner_lookahead_callback,
            10,
        )

        # =====================================================
        # Thruster publishers
        # =====================================================

        self.left_thrust_publisher = (
            self.create_publisher(
                Float64,
                self.left_thrust_topic,
                10,
            )
        )

        self.right_thrust_publisher = (
            self.create_publisher(
                Float64,
                self.right_thrust_topic,
                10,
            )
        )

        self.left_position_publisher = (
            self.create_publisher(
                Float64,
                self.left_position_topic,
                10,
            )
        )

        self.right_position_publisher = (
            self.create_publisher(
                Float64,
                self.right_position_topic,
                10,
            )
        )

        # =====================================================
        # Experiment-status QoS
        #
        # TRANSIENT_LOCAL keeps the latest state available to
        # subscribers that join after the controller.
        # =====================================================

        status_qos = QoSProfile(
            depth=1
        )

        status_qos.reliability = (
            ReliabilityPolicy.RELIABLE
        )

        status_qos.durability = (
            DurabilityPolicy.TRANSIENT_LOCAL
        )

        # =====================================================
        # Success publisher
        # =====================================================

        self.success_publisher = (
            self.create_publisher(
                Bool,
                self.success_topic,
                status_qos,
            )
        )

        # =====================================================
        # Controller-state publisher
        # =====================================================

        self.controller_state_publisher = (
            self.create_publisher(
                String,
                self.controller_state_topic,
                status_qos,
            )
        )

        # -----------------------------------------------------
        # Stress-run mirror status + diagnostics.  These do not
        # participate in the control law; they support the map/logger.
        # -----------------------------------------------------

        self.r1_success_publisher = self.create_publisher(
            Bool,
            '/r1/success',
            status_qos,
        )

        self.r1_state_publisher = self.create_publisher(
            String,
            '/r1/control/state',
            status_qos,
        )

        self.desired_heading_publisher = self.create_publisher(
            Float64, '/r1/control/desired_heading', 10
        )
        self.heading_error_publisher = self.create_publisher(
            Float64, '/r1/control/heading_error', 10
        )
        self.target_speed_publisher = self.create_publisher(
            Float64, '/r1/control/target_speed', 10
        )
        self.speed_error_publisher = self.create_publisher(
            Float64, '/r1/control/speed_error', 10
        )
        self.lookahead_publisher = self.create_publisher(
            Float64, '/r1/control/lookahead_distance', 10
        )
        self.cross_track_publisher = self.create_publisher(
            Float64, '/r1/control/cross_track_error', 10
        )
        self.reference_point_publisher = self.create_publisher(
            PoseStamped, '/r1/control/reference_point', 10
        )
        self.target_point_publisher = self.create_publisher(
            PoseStamped, '/r1/control/target_point', 10
        )
        self.active_waypoint_publisher = self.create_publisher(
            PoseStamped, '/r1/control/active_waypoint', 10
        )
        self.active_waypoint_index_publisher = self.create_publisher(
            Int32, '/r1/control/active_waypoint_index', 10
        )
        self.active_waypoint_number_publisher = self.create_publisher(
            Int32, '/r1/control/active_waypoint_number', 10
        )

        # =====================================================
        # Runtime state
        # =====================================================

        self.vehicle_state = None

        self.path = []

        # Index of the NEXT chronological waypoint that has not yet been
        # consumed.  This is monotonic for the whole TRACK phase.
        self.active_waypoint_index = 1

        self.controller_state = (
            self.TRACK
        )

        self.previous_control_time = None

        self.hold_start_time = None

        self.success_published = False

        # -----------------------------------------------------
        # RECOVER sub-state
        #
        # False = ALIGN
        # True  = APPROACH
        # -----------------------------------------------------

        self.recovery_aligned = False

        # =====================================================
        # Controller timer
        #
        # Because the node uses use_sim_time=True from the
        # launch file, this timer follows Gazebo simulation
        # time.
        # =====================================================

        self.control_timer = (
            self.create_timer(
                1.0 / self.control_rate,
                self.control_loop,
            )
        )

        # =====================================================
        # Initial published status
        # =====================================================

        self.publish_success(
            False
        )

        self.publish_controller_state()

        self.get_logger().info(
            'Leader stress controller started FROM original validated LeaderPIDController.'
        )

        self.get_logger().info(
            f'Control rate: '
            f'{self.control_rate:.2f} Hz'
        )

        self.get_logger().info(
            'Controller state: TRACK'
        )

    # =========================================================
    # Vehicle-state callback
    # =========================================================

    def state_callback(
        self,
        message,
    ):

        self.vehicle_state = (
            message
        )

    # =========================================================
    # Planner advisory callbacks
    # =========================================================

    def planner_speed_limit_callback(
        self,
        message,
    ):

        self.planner_target_speed = clamp(
            float(message.data),
            0.0,
            self.max_planner_speed,
        )

    def planner_lookahead_callback(
        self,
        message,
    ):

        self.planner_lookahead_distance = max(
            0.05,
            float(message.data),
        )

    # =========================================================
    # Reference-path callback
    # =========================================================

    def path_callback(
        self,
        message,
    ):

        if (
            message.header.frame_id
            and
            message.header.frame_id
            != 'world_ned'
        ):

            self.get_logger().warning(
                'Ignoring path because frame is '
                f'"{message.header.frame_id}", '
                'not "world_ned".'
            )

            return

        new_path = []

        for pose in message.poses:

            new_path.append(
                (
                    float(
                        pose.pose.position.x
                    ),

                    float(
                        pose.pose.position.y
                    ),
                )
            )

        if not new_path:

            return

        # -----------------------------------------------------
        # Planner republishes periodically.
        #
        # Only reset if the actual path changes.
        # -----------------------------------------------------

        if new_path != self.path:

            self.path = (
                new_path
            )

            self.reset_experiment()

            self.get_logger().info(
                f'New reference path received: '
                f'{len(self.path)} waypoints.'
            )

    # =========================================================
    # Experiment reset
    # =========================================================

    def reset_experiment(self):

        self.controller_state = (
            self.TRACK
        )

        self.speed_pid.reset()

        self.heading_pid.reset()

        self.brake_pid.reset()

        self.previous_control_time = None

        self.hold_start_time = None

        self.recovery_aligned = False

        self.success_published = False

        self.active_waypoint_index = (
            1 if len(self.path) > 1 else 0
        )

        self.publish_success(
            False
        )

        self.publish_controller_state()

        self.get_logger().info(
            'Controller state: TRACK'
        )

    # =========================================================
    # Main control loop
    # =========================================================

    def control_loop(self):

        if self.vehicle_state is None:

            return

        if not self.path:

            return

        now = (
            self.get_clock().now()
        )

        # =====================================================
        # Controller time step
        # =====================================================

        if self.previous_control_time is None:

            dt = (
                1.0
                / self.control_rate
            )

        else:

            dt = (
                now
                - self.previous_control_time
            ).nanoseconds / 1e9

            if dt <= 0.0:

                return

        self.previous_control_time = (
            now
        )

        # =====================================================
        # Current vehicle state
        # =====================================================

        north = float(
            self.vehicle_state.x
        )

        east = float(
            self.vehicle_state.y
        )

        vx = float(
            self.vehicle_state.vx
        )

        vy = float(
            self.vehicle_state.vy
        )

        body_yaw = float(
            self.vehicle_state.body_yaw
        )

        speed = math.hypot(
            vx,
            vy,
        )

        # -----------------------------------------------------
        # Signed surge velocity.
        #
        # x = North
        # y = East
        #
        # Positive:
        #     moving forward along the boat's body heading
        #
        # Negative:
        #     moving backwards
        # -----------------------------------------------------

        surge_velocity = (
            vx
            * math.cos(
                body_yaw
            )
            +
            vy
            * math.sin(
                body_yaw
            )
        )

        # =====================================================
        # Goal
        # =====================================================

        goal_north = (
            self.path[-1][0]
        )

        goal_east = (
            self.path[-1][1]
        )

        goal_distance = math.hypot(
            goal_north
            - north,

            goal_east
            - east,
        )

        # =====================================================
        # TRACK
        # =====================================================

        if (
            self.controller_state
            == self.TRACK
        ):

            # -------------------------------------------------
            # Normal tracking brakes as soon as the vehicle
            # first reaches the required 1 m acceptance zone.
            # -------------------------------------------------

            if (
                goal_distance
                <= self.goal_tolerance
            ):

                self.enter_brake(
                    speed=(
                        speed
                    ),

                    surge_velocity=(
                        surge_velocity
                    ),
                )

                self.publish_zero_thrust()

                return

            self.track_path(
                north=(
                    north
                ),

                east=(
                    east
                ),

                speed=(
                    speed
                ),

                body_yaw=(
                    body_yaw
                ),

                dt=(
                    dt
                ),
            )

            return

        # =====================================================
        # BRAKE
        # =====================================================

        if (
            self.controller_state
            == self.BRAKE
        ):

            self.brake(
                speed=(
                    speed
                ),

                surge_velocity=(
                    surge_velocity
                ),

                goal_distance=(
                    goal_distance
                ),

                now=(
                    now
                ),

                dt=(
                    dt
                ),
            )

            return

        # =====================================================
        # RECOVER
        # =====================================================

        if (
            self.controller_state
            == self.RECOVER
        ):

            # -------------------------------------------------
            # Recovery deliberately aims deeper inside the
            # required 1 m circle.
            #
            # This prevents repeated chattering around exactly
            # 1.0 m.
            # -------------------------------------------------

            if (
                goal_distance
                <= self.recovery_brake_distance
            ):

                self.get_logger().info(
                    'RECOVER capture distance reached. '
                    f'Distance={goal_distance:.3f} m. '
                    'Entering BRAKE.'
                )

                self.enter_brake(
                    speed=(
                        speed
                    ),

                    surge_velocity=(
                        surge_velocity
                    ),
                )

                self.publish_zero_thrust()

                return

            self.recover(
                north=(
                    north
                ),

                east=(
                    east
                ),

                surge_velocity=(
                    surge_velocity
                ),

                body_yaw=(
                    body_yaw
                ),

                goal_north=(
                    goal_north
                ),

                goal_east=(
                    goal_east
                ),

                dt=(
                    dt
                ),
            )

            return

        # =====================================================
        # HOLD
        # =====================================================

        if (
            self.controller_state
            == self.HOLD
        ):

            self.publish_zero_thrust()

            # -------------------------------------------------
            # The vehicle must remain inside the ORIGINAL
            # 1.0 m acceptance zone for the entire hold.
            # -------------------------------------------------

            if (
                goal_distance
                > self.goal_tolerance
            ):

                self.get_logger().warning(
                    'Robot left goal tolerance during HOLD. '
                    'Entering RECOVER.'
                )

                self.enter_recover()

                return

            hold_elapsed = (
                now
                - self.hold_start_time
            ).nanoseconds / 1e9

            if (
                hold_elapsed
                >= self.hold_duration
            ):

                self.enter_success(
                    goal_distance=(
                        goal_distance
                    ),

                    speed=(
                        speed
                    ),
                )

            return

        # =====================================================
        # SUCCESS
        # =====================================================

        if (
            self.controller_state
            == self.SUCCESS
        ):

            self.publish_zero_thrust()

            return

    # =========================================================
    # TRACK controller
    # =========================================================

    def track_path(
        self,
        north,
        east,
        speed,
        body_yaw,
        dt,
    ):

        # =====================================================
        # Ordered waypoint progress
        #
        # This is the only path-order change relative to the proven
        # controller.  No global nearest-waypoint search is allowed because
        # the stress course contains self-near loops.
        # =====================================================

        self.advance_ordered_waypoint_progress(
            north,
            east,
        )

        (
            reference_north,
            reference_east,
            cross_track_error,
        ) = self.ordered_reference_point(
            north,
            east,
        )

        (
            target_north,
            target_east,
        ) = self.find_lookahead_target(
            north,
            east,
        )

        # =====================================================
        # Desired NED heading
        # =====================================================

        desired_heading = math.atan2(
            target_east
            - east,

            target_north
            - north,
        )

        heading_error = wrap_angle(
            desired_heading
            - body_yaw
        )

        # =====================================================
        # Heading PID -- ORIGINAL CONTROL LAW
        # =====================================================

        turn_command = (
            self.heading_pid.update(
                heading_error,
                dt,
            )
        )

        turn_command = clamp(
            turn_command,
            -self.max_turn_thrust,
            self.max_turn_thrust,
        )

        # =====================================================
        # Speed PID -- ORIGINAL CONTROL LAW, planner setpoint
        # =====================================================

        active_target_speed = clamp(
            self.planner_target_speed,
            0.0,
            self.max_planner_speed,
        )

        speed_error = (
            active_target_speed
            - speed
        )

        forward_command = (
            self.speed_pid.update(
                speed_error,
                dt,
            )
        )

        forward_command = clamp(
            forward_command,
            0.0,
            self.max_forward_thrust,
        )

        # -----------------------------------------------------
        # ORIGINAL heading-misalignment slowdown.
        # -----------------------------------------------------

        if (
            abs(
                heading_error
            )
            > self.heading_slowdown_angle
        ):

            forward_command *= (
                0.35
            )

        # =====================================================
        # Differential thrust -- ORIGINAL MIXING
        # =====================================================

        left_thrust = (
            forward_command
            + turn_command
        )

        right_thrust = (
            forward_command
            - turn_command
        )

        self.publish_track_diagnostics(
            reference_north=reference_north,
            reference_east=reference_east,
            target_north=target_north,
            target_east=target_east,
            desired_heading=desired_heading,
            heading_error=heading_error,
            target_speed=active_target_speed,
            speed_error=speed_error,
            cross_track_error=cross_track_error,
        )

        self.publish_thrusters(
            left_thrust,
            right_thrust,
        )

    # =========================================================
    # BRAKE controller
    # =========================================================

    def brake(
        self,
        speed,
        surge_velocity,
        goal_distance,
        now,
        dt,
    ):

        # =====================================================
        # Stage 1:
        # Remove forward surge velocity with brake PID.
        # =====================================================

        if (
            surge_velocity
            > self.stop_speed_tolerance
        ):

            # -------------------------------------------------
            # Brake setpoint:
            #
            # u_target = 0
            #
            # e = u_target - u
            #
            # For forward movement:
            #
            # e < 0
            #
            # therefore the PID produces reverse thrust.
            # -------------------------------------------------

            brake_error = (
                0.0
                - surge_velocity
            )

            brake_command = (
                self.brake_pid.update(
                    brake_error,
                    dt,
                )
            )

            # -------------------------------------------------
            # SAFETY:
            #
            # BRAKE is reverse-only.
            #
            # It cannot command positive forward thrust.
            # -------------------------------------------------

            brake_command = clamp(
                brake_command,
                -self.max_brake_thrust,
                0.0,
            )

            # -------------------------------------------------
            # Equal reverse thrust avoids intentionally adding
            # yaw while stopping.
            # -------------------------------------------------

            self.publish_thrusters(
                brake_command,
                brake_command,
            )

            return

        # =====================================================
        # Stage 2:
        # Forward surge has been removed.
        #
        # Reverse thrust MUST now stop.
        # =====================================================

        self.brake_pid.reset()

        self.publish_zero_thrust()

        # -----------------------------------------------------
        # There may still be lateral / residual velocity.
        #
        # Allow hydrodynamic resistance to remove it.
        # -----------------------------------------------------

        if (
            speed
            > self.stop_speed_tolerance
        ):

            return

        # =====================================================
        # Boat is now effectively stopped.
        # =====================================================

        if (
            goal_distance
            <= self.goal_tolerance
        ):

            self.enter_hold(
                now=(
                    now
                ),

                goal_distance=(
                    goal_distance
                ),

                speed=(
                    speed
                ),
            )

        else:

            self.get_logger().info(
                'Boat stopped outside goal tolerance. '
                f'Distance={goal_distance:.3f} m. '
                'Entering RECOVER.'
            )

            self.enter_recover()

    # =========================================================
    # RECOVER controller
    # =========================================================

    def recover(
        self,
        north,
        east,
        surge_velocity,
        body_yaw,
        goal_north,
        goal_east,
        dt,
    ):

        # =====================================================
        # Point directly toward final goal
        # =====================================================

        desired_heading = math.atan2(
            goal_east
            - east,

            goal_north
            - north,
        )

        heading_error = wrap_angle(
            desired_heading
            - body_yaw
        )

        # =====================================================
        # RECOVER / ALIGN
        #
        # Rotate approximately in place.
        # =====================================================

        if not self.recovery_aligned:

            if (
                abs(
                    heading_error
                )
                <= self.recovery_align_tolerance
            ):

                self.recovery_aligned = (
                    True
                )

                self.speed_pid.reset()

                self.heading_pid.reset()

                self.publish_zero_thrust()

                self.get_logger().info(
                    'RECOVER alignment complete. '
                    'Beginning slow approach.'
                )

                return

            # -------------------------------------------------
            # Heading PID controls rotation.
            # -------------------------------------------------

            turn_command = (
                self.heading_pid.update(
                    heading_error,
                    dt,
                )
            )

            turn_command = clamp(
                turn_command,
                -self.recovery_max_turn_thrust,
                self.recovery_max_turn_thrust,
            )

            # -------------------------------------------------
            # Equal and opposite thrust:
            #
            # mean thrust ≈ 0
            #
            # so there is no intentional forward translation.
            # -------------------------------------------------

            self.publish_thrusters(
                turn_command,
                -turn_command,
            )

            return

        # =====================================================
        # RECOVER / APPROACH
        # =====================================================

        # -----------------------------------------------------
        # Hysteresis:
        #
        # alignment achieved:
        #     <= 0.15 rad
        #
        # alignment lost:
        #     > 0.35 rad
        #
        # This prevents constant switching.
        # -----------------------------------------------------

        if (
            abs(
                heading_error
            )
            > self.recovery_realign_tolerance
        ):

            self.recovery_aligned = (
                False
            )

            self.speed_pid.reset()

            self.heading_pid.reset()

            self.publish_zero_thrust()

            self.get_logger().info(
                'RECOVER heading drifted. '
                'Returning to ALIGN.'
            )

            return

        # =====================================================
        # Recovery heading PID
        # =====================================================

        turn_command = (
            self.heading_pid.update(
                heading_error,
                dt,
            )
        )

        turn_command = clamp(
            turn_command,
            -self.recovery_max_turn_thrust,
            self.recovery_max_turn_thrust,
        )

        # =====================================================
        # Recovery speed PID
        #
        # Reuses the normal speed PID structure but with:
        #
        # target = 0.25 m/s
        #
        # and a much smaller thrust cap.
        #
        # Signed surge velocity is used because the boat should
        # already be pointed approximately toward the target.
        # =====================================================

        speed_error = (
            self.recovery_target_speed
            - surge_velocity
        )

        forward_command = (
            self.speed_pid.update(
                speed_error,
                dt,
            )
        )

        forward_command = clamp(
            forward_command,
            0.0,
            self.recovery_max_forward_thrust,
        )

        left_thrust = (
            forward_command
            + turn_command
        )

        right_thrust = (
            forward_command
            - turn_command
        )

        self.publish_thrusters(
            left_thrust,
            right_thrust,
        )

    # =========================================================
    # State transitions
    # =========================================================

    def enter_track(self):

        self.controller_state = (
            self.TRACK
        )

        self.speed_pid.reset()

        self.heading_pid.reset()

        self.brake_pid.reset()

        self.hold_start_time = None

        self.recovery_aligned = False

        self.publish_controller_state()

        self.get_logger().info(
            'Controller state: TRACK'
        )

    # ---------------------------------------------------------

    def enter_brake(
        self,
        speed,
        surge_velocity,
    ):

        self.controller_state = (
            self.BRAKE
        )

        self.speed_pid.reset()

        self.heading_pid.reset()

        self.brake_pid.reset()

        self.hold_start_time = None

        self.recovery_aligned = False

        self.publish_controller_state()

        self.get_logger().info(
            'Controller state: BRAKE'
        )

        self.get_logger().info(
            'Goal braking started. '
            f'Total speed={speed:.3f} m/s, '
            f'surge={surge_velocity:.3f} m/s.'
        )

    # ---------------------------------------------------------

    def enter_recover(self):

        self.controller_state = (
            self.RECOVER
        )

        self.speed_pid.reset()

        self.heading_pid.reset()

        self.brake_pid.reset()

        self.hold_start_time = None

        self.recovery_aligned = False

        self.publish_zero_thrust()

        self.publish_controller_state()

        self.get_logger().info(
            'Controller state: RECOVER'
        )

        self.get_logger().info(
            'RECOVER mode: ALIGN'
        )

    # ---------------------------------------------------------

    def enter_hold(
        self,
        now,
        goal_distance,
        speed,
    ):

        self.controller_state = (
            self.HOLD
        )

        self.speed_pid.reset()

        self.heading_pid.reset()

        self.brake_pid.reset()

        self.recovery_aligned = False

        self.hold_start_time = (
            now
        )

        self.publish_zero_thrust()

        self.publish_controller_state()

        self.get_logger().info(
            'Controller state: HOLD'
        )

        self.get_logger().info(
            'Boat stopped inside target zone. '
            f'Speed={speed:.3f} m/s, '
            f'distance={goal_distance:.3f} m.'
        )

        self.get_logger().info(
            f'Holding for '
            f'{self.hold_duration:.1f} '
            'simulated seconds.'
        )

    # ---------------------------------------------------------

    def enter_success(
        self,
        goal_distance,
        speed,
    ):

        self.controller_state = (
            self.SUCCESS
        )

        self.publish_zero_thrust()

        # -----------------------------------------------------
        # Publish state first so the logger's final sample can
        # explicitly identify SUCCESS.
        # -----------------------------------------------------

        self.publish_controller_state()

        if not self.success_published:

            self.publish_success(
                True
            )

            self.success_published = (
                True
            )

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'EXPERIMENT SUCCESS'
        )

        self.get_logger().info(
            f'Goal distance: '
            f'{goal_distance:.3f} m'
        )

        self.get_logger().info(
            f'Speed: '
            f'{speed:.3f} m/s'
        )

        self.get_logger().info(
            f'Remained within '
            f'{self.goal_tolerance:.2f} m '
            f'for '
            f'{self.hold_duration:.1f} '
            'simulated seconds.'
        )

        self.get_logger().info(
            '========================================'
        )

    # =========================================================
    # Lookahead target
    # =========================================================

    def advance_ordered_waypoint_progress(
        self,
        north,
        east,
    ):

        if len(self.path) < 2:
            return

        index = max(
            1,
            min(
                self.active_waypoint_index,
                len(self.path) - 1,
            ),
        )

        advances = 0

        # Final waypoint remains the active waypoint.  It is consumed by the
        # ORIGINAL goal-tolerance / BRAKE state machine, not by path progress.
        while (
            index < len(self.path) - 1
            and advances < self.max_waypoint_advances_per_cycle
        ):

            previous = self.path[index - 1]
            current = self.path[index]

            dx = current[0] - previous[0]
            dy = current[1] - previous[1]
            segment_length = math.hypot(dx, dy)

            if segment_length <= 1e-9:
                index += 1
                advances += 1
                continue

            endpoint_distance = math.hypot(
                current[0] - north,
                current[1] - east,
            )

            if endpoint_distance <= self.waypoint_capture_radius:
                index += 1
                advances += 1
                continue

            ux = dx / segment_length
            uy = dy / segment_length

            rel_x = north - previous[0]
            rel_y = east - previous[1]

            along = rel_x * ux + rel_y * uy
            lateral = abs(
                rel_x * (-uy) + rel_y * ux
            )

            # A waypoint can be considered passed only when the robot crossed
            # its perpendicular plane while remaining inside the local path
            # corridor.  A nearby later loop cannot satisfy this for the
            # current chronological segment.
            if (
                along >= segment_length
                and lateral <= self.progress_corridor_m
            ):
                index += 1
                advances += 1
                continue

            break

        self.active_waypoint_index = index
        self.publish_active_waypoint()

    def ordered_reference_point(
        self,
        north,
        east,
    ):

        if len(self.path) < 2:
            return north, east, 0.0

        index = max(
            1,
            min(
                self.active_waypoint_index,
                len(self.path) - 1,
            ),
        )

        a = self.path[index - 1]
        b = self.path[index]

        dx = b[0] - a[0]
        dy = b[1] - a[1]
        length2 = dx * dx + dy * dy

        if length2 <= 1e-12:
            return a[0], a[1], 0.0

        t = clamp(
            (
                (north - a[0]) * dx
                + (east - a[1]) * dy
            ) / length2,
            0.0,
            1.0,
        )

        ref_north = a[0] + t * dx
        ref_east = a[1] + t * dy

        length = math.sqrt(length2)
        ux = dx / length
        uy = dy / length

        # Signed lateral error relative to chronological segment tangent.
        cross_track_error = (
            (north - ref_north) * (-uy)
            + (east - ref_east) * ux
        )

        return (
            ref_north,
            ref_east,
            cross_track_error,
        )

    def find_lookahead_target(
        self,
        north,
        east,
    ):

        if not self.path:
            return north, east

        if len(self.path) == 1:
            return self.path[0]

        index = max(
            1,
            min(
                self.active_waypoint_index,
                len(self.path) - 1,
            ),
        )

        lookahead = max(
            0.05,
            self.planner_lookahead_distance,
        )

        # Distance from the robot to the next chronological waypoint, then
        # continue forward ONLY in path order.
        accumulated_distance = math.hypot(
            self.path[index][0] - north,
            self.path[index][1] - east,
        )

        target_index = index

        if accumulated_distance < lookahead:
            for i in range(
                index,
                len(self.path) - 1,
            ):

                accumulated_distance += math.hypot(
                    self.path[i + 1][0] - self.path[i][0],
                    self.path[i + 1][1] - self.path[i][1],
                )

                target_index = i + 1

                if accumulated_distance >= lookahead:
                    break

        return self.path[target_index]

    def publish_active_waypoint(self):

        if not self.path:
            return

        index = max(
            0,
            min(
                self.active_waypoint_index,
                len(self.path) - 1,
            ),
        )

        index_message = Int32()
        index_message.data = int(index)
        self.active_waypoint_index_publisher.publish(index_message)

        number_message = Int32()
        number_message.data = int(index + 1)
        self.active_waypoint_number_publisher.publish(number_message)

        self.publish_pose_point(
            self.active_waypoint_publisher,
            self.path[index][0],
            self.path[index][1],
        )

    def publish_pose_point(
        self,
        publisher,
        north,
        east,
    ):

        message = PoseStamped()
        message.header.frame_id = 'world_ned'
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = float(north)
        message.pose.position.y = float(east)
        message.pose.orientation.w = 1.0
        publisher.publish(message)

    def publish_track_diagnostics(
        self,
        reference_north,
        reference_east,
        target_north,
        target_east,
        desired_heading,
        heading_error,
        target_speed,
        speed_error,
        cross_track_error,
    ):

        message = Float64()
        message.data = float(desired_heading)
        self.desired_heading_publisher.publish(message)

        message = Float64()
        message.data = float(heading_error)
        self.heading_error_publisher.publish(message)

        message = Float64()
        message.data = float(target_speed)
        self.target_speed_publisher.publish(message)

        message = Float64()
        message.data = float(speed_error)
        self.speed_error_publisher.publish(message)

        message = Float64()
        message.data = float(self.planner_lookahead_distance)
        self.lookahead_publisher.publish(message)

        message = Float64()
        message.data = float(cross_track_error)
        self.cross_track_publisher.publish(message)

        self.publish_pose_point(
            self.reference_point_publisher,
            reference_north,
            reference_east,
        )

        self.publish_pose_point(
            self.target_point_publisher,
            target_north,
            target_east,
        )

    # =========================================================
    # Thruster output
    # =========================================================

    def publish_thrusters(
        self,
        left_thrust,
        right_thrust,
    ):

        left_thrust = clamp(
            left_thrust,
            -self.max_total_thrust,
            self.max_total_thrust,
        )

        right_thrust = clamp(
            right_thrust,
            -self.max_total_thrust,
            self.max_total_thrust,
        )

        # =====================================================
        # Professor requirement:
        #
        # Thruster azimuth remains fixed straight.
        #
        # Steering is performed only through differential
        # thrust.
        # =====================================================

        zero_position = (
            Float64()
        )

        zero_position.data = (
            0.0
        )

        self.left_position_publisher.publish(
            zero_position
        )

        self.right_position_publisher.publish(
            zero_position
        )

        # =====================================================
        # Thrust
        # =====================================================

        left_message = (
            Float64()
        )

        left_message.data = float(
            left_thrust
        )

        right_message = (
            Float64()
        )

        right_message.data = float(
            right_thrust
        )

        self.left_thrust_publisher.publish(
            left_message
        )

        self.right_thrust_publisher.publish(
            right_message
        )

    # ---------------------------------------------------------

    def publish_zero_thrust(self):

        self.publish_thrusters(
            0.0,
            0.0,
        )

    # =========================================================
    # Experiment-success publisher
    # =========================================================

    def publish_success(
        self,
        success,
    ):

        message = (
            Bool()
        )

        message.data = bool(
            success
        )

        self.success_publisher.publish(
            message
        )

        self.r1_success_publisher.publish(
            message
        )

    # =========================================================
    # Controller-state publisher
    # =========================================================

    def publish_controller_state(self):

        message = (
            String()
        )

        message.data = str(
            self.controller_state
        )

        self.controller_state_publisher.publish(
            message
        )

        self.r1_state_publisher.publish(
            message
        )

    # =========================================================
    # Safe shutdown
    # =========================================================

    def stop(self):

        self.publish_zero_thrust()


# =============================================================
# Main
# =============================================================


def main(args=None):

    rclpy.init(
        args=args
    )

    node = (
        LeaderPIDController()
    )

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.stop()

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()
