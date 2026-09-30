import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float64, String

from platoon_interfaces.msg import VehicleState

from rclpy.qos import QoSProfile, DurabilityPolicy


# Actual expanded VRX WAM-V geometry [m].
GPS_X_FROM_BASE = -0.85
FRONT_EXTENT = 2.549
REAR_EXTENT = 2.822
HALF_WIDTH = 1.267


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_pi(value):
    return math.atan2(math.sin(value), math.cos(value))


def gps_to_base(state):
    """Convert GPS antenna position to base_link position in world_ned."""
    c = math.cos(state.body_yaw)
    s = math.sin(state.body_yaw)

    # GPS = base + R(yaw) * [-0.85, 0].
    # Therefore base = GPS + 0.85 * forward.
    return (
        float(state.x) - GPS_X_FROM_BASE * c,
        float(state.y) - GPS_X_FROM_BASE * s,
    )


def hitbox_polygon(state):
    """Return the four corners of the oriented WAM-V footprint."""
    cx, cy = gps_to_base(state)
    c = math.cos(state.body_yaw)
    s = math.sin(state.body_yaw)

    local = (
        (FRONT_EXTENT, HALF_WIDTH),
        (FRONT_EXTENT, -HALF_WIDTH),
        (-REAR_EXTENT, -HALF_WIDTH),
        (-REAR_EXTENT, HALF_WIDTH),
    )

    return [
        (
            cx + lx * c - ly * s,
            cy + lx * s + ly * c,
        )
        for lx, ly in local
    ]


def cross(ax, ay, bx, by):
    return ax * by - ay * bx


def segments_intersect(a, b, c, d):
    abx = b[0] - a[0]
    aby = b[1] - a[1]
    acx = c[0] - a[0]
    acy = c[1] - a[1]
    adx = d[0] - a[0]
    ady = d[1] - a[1]

    cdx = d[0] - c[0]
    cdy = d[1] - c[1]
    cax = a[0] - c[0]
    cay = a[1] - c[1]
    cbx = b[0] - c[0]
    cby = b[1] - c[1]

    c1 = cross(abx, aby, acx, acy)
    c2 = cross(abx, aby, adx, ady)
    c3 = cross(cdx, cdy, cax, cay)
    c4 = cross(cdx, cdy, cbx, cby)

    eps = 1e-9

    return (
        (
            (c1 > eps and c2 < -eps)
            or (c1 < -eps and c2 > eps)
        )
        and
        (
            (c3 > eps and c4 < -eps)
            or (c3 < -eps and c4 > eps)
        )
    )


def point_in_polygon(point, polygon):
    sign = None

    for i in range(len(polygon)):
        a = polygon[i]
        b = polygon[(i + 1) % len(polygon)]

        value = cross(
            b[0] - a[0],
            b[1] - a[1],
            point[0] - a[0],
            point[1] - a[1],
        )

        if abs(value) < 1e-9:
            continue

        current = value > 0.0

        if sign is None:
            sign = current
        elif sign != current:
            return False

    return True


def point_segment_distance(point, a, b):
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    length2 = dx * dx + dy * dy

    if length2 < 1e-12:
        return math.hypot(
            point[0] - a[0],
            point[1] - a[1],
        )

    t = (
        (point[0] - a[0]) * dx
        + (point[1] - a[1]) * dy
    ) / length2

    t = clamp(t, 0.0, 1.0)

    qx = a[0] + t * dx
    qy = a[1] + t * dy

    return math.hypot(
        point[0] - qx,
        point[1] - qy,
    )


def polygon_clearance(poly_a, poly_b):
    """Exact planar minimum distance between the two rectangles."""
    for i in range(len(poly_a)):
        a = poly_a[i]
        b = poly_a[(i + 1) % len(poly_a)]

        for j in range(len(poly_b)):
            c = poly_b[j]
            d = poly_b[(j + 1) % len(poly_b)]

            if segments_intersect(a, b, c, d):
                return 0.0

    if (
        point_in_polygon(poly_a[0], poly_b)
        or point_in_polygon(poly_b[0], poly_a)
    ):
        return 0.0

    result = float('inf')

    for point in poly_a:
        for j in range(len(poly_b)):
            result = min(
                result,
                point_segment_distance(
                    point,
                    poly_b[j],
                    poly_b[(j + 1) % len(poly_b)],
                ),
            )

    for point in poly_b:
        for j in range(len(poly_a)):
            result = min(
                result,
                point_segment_distance(
                    point,
                    poly_a[j],
                    poly_a[(j + 1) % len(poly_a)],
                ),
            )

    return result


class FollowerPidController(Node):

    def __init__(self):
        super().__init__('follower_pid_controller')

        # ------------------------------------------------------
        # Generic follower identity / ROS interfaces
        # ------------------------------------------------------
        self.declare_parameter('follower_id', 'r2')
        self.declare_parameter('predecessor_id', 'r1')
        self.declare_parameter('actuator_prefix', '/wamv2')
        self.declare_parameter(
            'mission_state_topic',
            '/platoon/mission_state',
        )

        self.follower_id = str(
            self.get_parameter('follower_id').value
        )
        self.predecessor_id = str(
            self.get_parameter('predecessor_id').value
        )

        self.actuator_prefix = str(
            self.get_parameter('actuator_prefix').value
        ).rstrip('/')

        if not self.actuator_prefix.startswith('/'):
            self.actuator_prefix = '/' + self.actuator_prefix

        self.mission_state_topic = str(
            self.get_parameter('mission_state_topic').value
        )

        self.predecessor_state_topic = (
            f'/{self.predecessor_id}/vehicle_state'
        )
        self.follower_state_topic = (
            f'/{self.follower_id}/vehicle_state'
        )

        self.breadcrumb_topic = (
            f'/planner/{self.follower_id}/breadcrumb_path'
        )
        self.parking_topic = (
            f'/planner/{self.follower_id}/parking_path'
        )

        params = {
            'heading_kp': 911.4007441588406,
            'heading_ki': 25.65165107782674,
            'heading_kd': 98.10493190966012,
            'speed_kp': 127.83877941647718,
            'speed_ki': 18.383775431748894,
            'speed_kd': 49.388692087475725,
            'brake_kp': 177.8935198208073,
            'brake_ki': 60.634764417119634,
            'brake_kd': 10.647182804717701,
            'lookahead_distance': 5.103256211030906,

            # FOLLOW guidance is path-tangent based. The heading
            # follows the local historical trail tangent; lateral
            # offset adds only a bounded cross-track correction.
            # This correction cap does NOT cap the path's own turn.
            'follow_tangent_half_window': 0.75,
            'cross_track_heading_gain': 0.12,
            'max_cross_track_correction_deg': 35.0,

            # The planner already publishes the breadcrumb path
            # beginning just behind the follower's chronological
            # progress. Guidance may search only this local prefix;
            # later self-near loop branches are ineligible.
            'guidance_search_distance': 6.0,

            # V2.2:
            # 5 m formation distance is measured bumper-to-bumper
            # ALONG the unsmoothed breadcrumb path.
            'formation_distance': 5.0,
            'distance_kp': 0.35,
            'distance_ki': 0.02,
            'distance_kd': 0.10,

            # Exact oriented-hull clearance is now SAFETY ONLY.
            # It no longer controls normal formation spacing.
            'collision_warning_clearance': 0.50,
            'collision_avoidance_clearance': 0.20,
            'collision_release_clearance': 0.35,
            'parking_reverse_clearance': 4.5,

            'catchup_distance': 6.0,
            'catchup_min_speed': 1.20,
            'follow_max_speed': 1.50,
            'max_distance_speed_correction': 0.70,
            'catchup_feedforward_thrust': 160.0,

            'parking_speed': 0.70,
            'parking_align_tolerance_deg': 8.0,
            'parking_align_hold': 0.5,
            'parking_reverse_speed': 0.35,
            'parking_reverse_kp': 450.0,

            'goal_tolerance': 1.0,
            'stop_speed_tolerance': 0.05,
            'terminal_settle_speed': 0.08,
            'terminal_settle_hold': 0.50,
            'parking_hold_time': 1.0,

            'max_forward_thrust': 900.0,
            'max_turn_thrust': 400.0,
            'max_total_thrust': 1000.0,
            'max_brake_thrust': 500.0,
            'avoidance_thrust': 260.0,
        }

        for name, default in params.items():
            self.declare_parameter(name, default)

        for name in params:
            setattr(
                self,
                name,
                float(self.get_parameter(name).value),
            )

        self.parking_align_tolerance = math.radians(
            self.parking_align_tolerance_deg
        )

        self.max_cross_track_correction = math.radians(
            self.max_cross_track_correction_deg
        )

        self.predecessor = None
        self.follower = None

        self.path = []
        self.path_source = 'NONE'

        self.mode = 'FOLLOW'
        self.last_time = None

        self.heading_integral = 0.0
        self.heading_previous = 0.0
        self.speed_integral = 0.0
        self.speed_previous = 0.0
        self.distance_integral = 0.0
        self.distance_previous = 0.0
        self.brake_integral = 0.0
        self.brake_previous = 0.0

        self.parking_hold_start = None
        self.align_hold_start = None
        self.success_latched = False
        self.avoidance_latched = False

        self.last_desired_heading = float('nan')
        self.last_path_tangent_heading = float('nan')
        self.last_cross_track_error = float('nan')

        # Terminal formation-stop state. At R1 target capture,
        # the last valid breadcrumb tangent is frozen and used as
        # R2's heading reference while BRAKE/RECOVER/HOLD runs.
        self.terminal_heading = None
        self.longitudinal_state = 'TRACK'
        self.terminal_settle_start = None
        self.terminal_settled = False

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_qos = qos

        self.create_subscription(
            VehicleState,
            self.predecessor_state_topic,
            self.predecessor_callback,
            20,
        )

        self.create_subscription(
            VehicleState,
            self.follower_state_topic,
            self.follower_callback,
            20,
        )

        self.create_subscription(
            String,
            self.mission_state_topic,
            self.mode_callback,
            10,
        )

        # FOLLOW starts with breadcrumb subscription only.
        self.breadcrumb_subscription = self.create_subscription(
            Path,
            self.breadcrumb_topic,
            self.breadcrumb_callback,
            qos,
        )

        self.parking_subscription = None

        self.left_pub = self.create_publisher(
            Float64,
            f'{self.actuator_prefix}/thrusters/left/thrust',
            10,
        )
        self.right_pub = self.create_publisher(
            Float64,
            f'{self.actuator_prefix}/thrusters/right/thrust',
            10,
        )
        self.left_pos_pub = self.create_publisher(
            Float64,
            f'{self.actuator_prefix}/thrusters/left/pos',
            10,
        )
        self.right_pos_pub = self.create_publisher(
            Float64,
            f'{self.actuator_prefix}/thrusters/right/pos',
            10,
        )

        self.success_pub = self.create_publisher(
            Bool,
            f'/{self.follower_id}/success',
            qos,
        )

        self.path_source_pub = self.create_publisher(
            String,
            f'/{self.follower_id}/control/path_source',
            10,
        )
        self.longitudinal_state_pub = self.create_publisher(
            String,
            f'/{self.follower_id}/control/longitudinal_state',
            10,
        )
        self.terminal_settled_pub = self.create_publisher(
            Bool,
            f'/{self.follower_id}/terminal_settled',
            10,
        )
        self.heading_error_pub = self.create_publisher(
            Float64,
            f'/{self.follower_id}/control/heading_error',
            10,
        )
        self.speed_error_pub = self.create_publisher(
            Float64,
            f'/{self.follower_id}/control/speed_error',
            10,
        )
        self.target_speed_pub = self.create_publisher(
            Float64,
            f'/{self.follower_id}/control/target_speed',
            10,
        )

        # V2.2 longitudinal path-gap input.
        self.path_gap = None
        self.path_gap_valid = False
        self.path_gap_time = None

        self.path_gap_sub = self.create_subscription(
            Float64,
            f'/planner/{self.follower_id}/path_gap',
            self.path_gap_callback,
            10,
        )

        self.path_gap_valid_sub = self.create_subscription(
            Bool,
            f'/planner/{self.follower_id}/path_gap_valid',
            self.path_gap_valid_callback,
            10,
        )

        # V2.1 proactive longitudinal preview.
        # May only reduce the existing reactive target speed.
        self.historical_speed_recommended = None
        self.historical_speed_recommended_time = None

        self.historical_speed_recommended_sub = self.create_subscription(
            Float64,
            f'/planner/{self.follower_id}/historical_speed_recommended',
            self.historical_speed_recommended_callback,
            10,
        )

        self.desired_heading_pub = self.create_publisher(
            Float64,
            f'/{self.follower_id}/control/desired_heading',
            10,
        )
        self.path_tangent_heading_pub = self.create_publisher(
            Float64,
            f'/{self.follower_id}/control/path_tangent_heading',
            10,
        )
        self.cross_track_error_pub = self.create_publisher(
            Float64,
            f'/{self.follower_id}/control/cross_track_error',
            10,
        )

        self.clearance_pub = self.create_publisher(
            Float64,
            f'/{self.follower_id}/safety/hitbox_clearance',
            10,
        )
        self.warning_pub = self.create_publisher(
            Bool,
            f'/{self.follower_id}/safety/collision_warning',
            10,
        )
        self.avoidance_pub = self.create_publisher(
            Bool,
            f'/{self.follower_id}/safety/avoidance_active',
            10,
        )
        self.aligned_pub = self.create_publisher(
            Bool,
            f'/{self.follower_id}/parking/aligned',
            10,
        )

        self.timer = self.create_timer(
            0.10,
            self.control_loop,
        )

        self.get_logger().info(
            f'{self.follower_id} follower controller ready; 'f'predecessor={self.predecessor_id}, 'f'actuators={self.actuator_prefix}'
        )

    # ---------------------------------------------------------
    # Inputs / subscription handoff
    # ---------------------------------------------------------

    def predecessor_callback(self, msg):
        self.predecessor = msg

    def follower_callback(self, msg):
        self.follower = msg

    def breadcrumb_callback(self, msg):
        if self.mode != 'FOLLOW':
            return

        self.path = [
            (
                float(p.pose.position.x),
                float(p.pose.position.y),
            )
            for p in msg.poses
        ]
        self.path_source = 'BREADCRUMB'

    def parking_callback(self, msg):
        if self.mode not in ('PARK_ALIGN', 'PARK'):
            return

        self.path = [
            (
                float(p.pose.position.x),
                float(p.pose.position.y),
            )
            for p in msg.poses
        ]
        self.path_source = 'PARKING'

    def destroy_breadcrumb_subscription(self):
        if self.breadcrumb_subscription is not None:
            self.destroy_subscription(
                self.breadcrumb_subscription
            )
            self.breadcrumb_subscription = None

            self.get_logger().info(
                'UNSUBSCRIBED from breadcrumb path.'
            )

    def create_parking_subscription(self):
        if self.parking_subscription is None:
            self.parking_subscription = (
                self.create_subscription(
                    Path,
                    self.parking_topic,
                    self.parking_callback,
                    self.path_qos,
                )
            )

            self.get_logger().info(
                'SUBSCRIBED to parking path.'
            )

    def mode_callback(self, msg):
        new_mode = msg.data

        if new_mode == self.mode:
            return

        self.get_logger().info(
            f'R2 mode -> {new_mode}'
        )

        if new_mode == 'FORMATION_BRAKE':
            # Freeze the latest smooth-trail tangent BEFORE clearing
            # the breadcrumb path. This keeps R2 aligned with the
            # leader trajectory while both robots brake.
            if math.isfinite(
                self.last_path_tangent_heading
            ):
                self.terminal_heading = (
                    self.last_path_tangent_heading
                )
            elif self.follower is not None:
                self.terminal_heading = float(
                    self.follower.body_yaw
                )

            self.destroy_breadcrumb_subscription()
            self.path = []
            self.path_source = 'NONE'
            self.longitudinal_state = 'BRAKE'
            self.terminal_settle_start = None
            self.terminal_settled = False

        elif new_mode == 'FORMATION_HOLD':
            self.path = []
            self.path_source = 'NONE'
            if self.longitudinal_state not in (
                'BRAKE',
                'RECOVER',
                'HOLD',
            ):
                self.longitudinal_state = 'HOLD'

        elif new_mode == 'PARK_REVERSE':
            self.path = []
            self.path_source = 'NONE'
            self.longitudinal_state = 'PARKING'
            self.terminal_settled = False

        elif new_mode == 'PARK_ALIGN':
            self.path = []
            self.path_source = 'NONE'
            self.create_parking_subscription()
            self.align_hold_start = None
            self.longitudinal_state = 'PARKING'
            self.terminal_settled = False

        elif new_mode in ('PARK', 'DWELL', 'SUCCESS'):
            self.longitudinal_state = 'PARKING'
            self.terminal_settled = False

        elif new_mode == 'FOLLOW':
            self.longitudinal_state = 'TRACK'
            self.terminal_heading = None
            self.terminal_settle_start = None
            self.terminal_settled = False

        self.mode = new_mode
        self.reset_pid()


    def path_gap_callback(self, msg):
        self.path_gap = float(msg.data)
        self.path_gap_time = (
            self.get_clock().now().nanoseconds * 1e-9
        )

    def path_gap_valid_callback(self, msg):
        self.path_gap_valid = bool(msg.data)

    # ---------------------------------------------------------
    # Geometry / safety
    # ---------------------------------------------------------

    def hitbox_clearance(self):
        if self.predecessor is None or self.follower is None:
            return float('inf')

        return polygon_clearance(
            hitbox_polygon(self.predecessor),
            hitbox_polygon(self.follower),
        )

    def publish_safety(self, clearance):
        warning = (
            clearance
            <= self.collision_warning_clearance
        )

        # Warning and avoidance are intentionally separate.
        #
        # Hull geometry is no longer involved in the normal
        # 5 m formation controller.
        if (
            clearance
            <= self.collision_avoidance_clearance
        ):
            self.avoidance_latched = True

        elif (
            self.avoidance_latched
            and clearance
            >= self.collision_release_clearance
        ):
            self.avoidance_latched = False

        msg = Float64()
        msg.data = float(clearance)
        self.clearance_pub.publish(msg)

        flag = Bool()
        flag.data = warning
        self.warning_pub.publish(flag)

        flag = Bool()
        flag.data = self.avoidance_latched
        self.avoidance_pub.publish(flag)

    def collision_avoidance(self):
        """
        Move along whichever current body-axis direction
        increases separation from R1 most strongly.
        """
        r1_base = gps_to_base(self.predecessor)
        r2_base = gps_to_base(self.follower)

        ax = r2_base[0] - r1_base[0]
        ay = r2_base[1] - r1_base[1]

        length = math.hypot(ax, ay)

        if length < 1e-6:
            self.stop()
            return

        ax /= length
        ay /= length

        fx = math.cos(self.follower.body_yaw)
        fy = math.sin(self.follower.body_yaw)

        forward_score = fx * ax + fy * ay
        reverse_score = -forward_score

        if forward_score >= reverse_score:
            command = self.avoidance_thrust
        else:
            command = -self.avoidance_thrust

        # Equal thrust = translation/braking only, no yaw.
        self.publish_thrusters(
            command,
            command,
        )

    # ---------------------------------------------------------
    # PID helpers
    # ---------------------------------------------------------

    def reset_pid(self):
        self.heading_integral = 0.0
        self.heading_previous = 0.0
        self.speed_integral = 0.0
        self.speed_previous = 0.0
        self.distance_integral = 0.0
        self.distance_previous = 0.0
        self.brake_integral = 0.0
        self.brake_previous = 0.0

    def publish_terminal_status(self):
        state = String()
        state.data = self.longitudinal_state
        self.longitudinal_state_pub.publish(state)

        settled = Bool()
        settled.data = bool(self.terminal_settled)
        self.terminal_settled_pub.publish(settled)

    def signed_surge_speed(self):
        if self.follower is None:
            return 0.0

        return (
            self.follower.vx * math.cos(self.follower.body_yaw)
            + self.follower.vy * math.sin(self.follower.body_yaw)
        )

    def terminal_heading_turn(self, dt):
        if self.follower is None:
            return 0.0

        desired = (
            self.terminal_heading
            if self.terminal_heading is not None
            else float(self.follower.body_yaw)
        )

        heading_error = wrap_pi(
            desired
            - self.follower.body_yaw
        )

        self.heading_integral = clamp(
            self.heading_integral
            + heading_error * dt,
            -1.5,
            1.5,
        )

        heading_derivative = wrap_pi(
            heading_error
            - self.heading_previous
        ) / max(dt, 1e-6)

        self.heading_previous = heading_error

        turn = (
            self.heading_kp * heading_error
            + self.heading_ki * self.heading_integral
            + self.heading_kd * heading_derivative
        )

        turn = clamp(
            turn,
            -self.max_turn_thrust,
            self.max_turn_thrust,
        )

        # Keep logger guidance explicit after breadcrumbs are removed.
        msg = Float64()
        msg.data = float(desired)
        self.desired_heading_pub.publish(msg)
        self.path_tangent_heading_pub.publish(msg)

        msg = Float64()
        msg.data = float(heading_error)
        self.heading_error_pub.publish(msg)

        msg = Float64()
        msg.data = float('nan')
        self.cross_track_error_pub.publish(msg)

        return turn

    def terminal_brake_forward(self, surge, dt):
        error = -surge

        self.brake_integral = clamp(
            self.brake_integral
            + error * dt,
            -1.0,
            1.0,
        )

        derivative = (
            error
            - self.brake_previous
        ) / max(dt, 1e-6)

        self.brake_previous = error

        command = (
            self.brake_kp * error
            + self.brake_ki * self.brake_integral
            + self.brake_kd * derivative
        )

        # BRAKE may only oppose positive surge. Recovery is handled
        # separately with the normal tuned speed PID.
        return clamp(
            command,
            -self.max_brake_thrust,
            0.0,
        )

    def terminal_recover_forward(self, surge, dt):
        # Same tuned R1/R2 speed PID, target = 0 m/s. When R2
        # overshoots backward, this produces a small forward command.
        speed_error = -surge

        self.speed_integral = clamp(
            self.speed_integral
            + speed_error * dt,
            -3.0,
            3.0,
        )

        derivative = (
            speed_error
            - self.speed_previous
        ) / max(dt, 1e-6)

        self.speed_previous = speed_error

        command = (
            self.speed_kp * speed_error
            + self.speed_ki * self.speed_integral
            + self.speed_kd * derivative
        )

        return clamp(
            command,
            0.0,
            self.max_forward_thrust,
        )

    def terminal_stop_control(self, now, dt):
        """R1-derived BRAKE -> RECOVER -> HOLD terminal stop."""
        if self.follower is None:
            self.stop()
            return

        surge = self.signed_surge_speed()
        total_speed = float(self.follower.speed)

        # State transitions are based on signed surge so braking
        # overshoot can be recovered instead of continuing backward.
        if self.longitudinal_state == 'BRAKE':
            if surge < -self.stop_speed_tolerance:
                self.longitudinal_state = 'RECOVER'
                self.speed_integral = 0.0
                self.speed_previous = -surge
                self.terminal_settle_start = None
            elif (
                abs(surge) <= self.stop_speed_tolerance
                and total_speed <= self.terminal_settle_speed
            ):
                self.longitudinal_state = 'HOLD'
                self.terminal_settle_start = now

        elif self.longitudinal_state == 'RECOVER':
            if surge > self.stop_speed_tolerance:
                self.longitudinal_state = 'BRAKE'
                self.brake_integral = 0.0
                self.brake_previous = -surge
                self.terminal_settle_start = None
            elif (
                abs(surge) <= self.stop_speed_tolerance
                and total_speed <= self.terminal_settle_speed
            ):
                self.longitudinal_state = 'HOLD'
                self.terminal_settle_start = now

        elif self.longitudinal_state == 'HOLD':
            if surge > self.stop_speed_tolerance:
                self.longitudinal_state = 'BRAKE'
                self.brake_integral = 0.0
                self.brake_previous = -surge
                self.terminal_settle_start = None
                self.terminal_settled = False
            elif surge < -self.stop_speed_tolerance:
                self.longitudinal_state = 'RECOVER'
                self.speed_integral = 0.0
                self.speed_previous = -surge
                self.terminal_settle_start = None
                self.terminal_settled = False
            elif total_speed > self.terminal_settle_speed:
                self.terminal_settle_start = None
                self.terminal_settled = False
            elif self.terminal_settle_start is None:
                self.terminal_settle_start = now

        else:
            self.longitudinal_state = 'BRAKE'
            self.terminal_settle_start = None
            self.terminal_settled = False

        turn = self.terminal_heading_turn(dt)

        if self.longitudinal_state == 'BRAKE':
            forward = self.terminal_brake_forward(
                surge,
                dt,
            )
        elif self.longitudinal_state == 'RECOVER':
            forward = self.terminal_recover_forward(
                surge,
                dt,
            )
        else:
            forward = 0.0

        self.publish_thrusters(
            forward + turn,
            forward - turn,
        )

        msg = Float64()
        msg.data = 0.0
        self.target_speed_pub.publish(msg)

        msg = Float64()
        msg.data = float(-surge)
        self.speed_error_pub.publish(msg)

        if (
            self.longitudinal_state == 'HOLD'
            and self.terminal_settle_start is not None
        ):
            held = (
                now
                - self.terminal_settle_start
            ).nanoseconds * 1e-9

            if held >= self.terminal_settle_hold:
                self.terminal_settled = True

    def publish_thrusters(self, left, right):
        left = clamp(
            left,
            -self.max_total_thrust,
            self.max_total_thrust,
        )
        right = clamp(
            right,
            -self.max_total_thrust,
            self.max_total_thrust,
        )

        msg = Float64()
        msg.data = float(left)
        self.left_pub.publish(msg)

        msg = Float64()
        msg.data = float(right)
        self.right_pub.publish(msg)

        pos = Float64()
        pos.data = 0.0

        self.left_pos_pub.publish(pos)
        self.right_pos_pub.publish(pos)

    def stop(self):
        self.publish_thrusters(0.0, 0.0)

    def nearest_index(self):
        if self.follower is None or not self.path:
            return None

        return min(
            range(len(self.path)),
            key=lambda i:
                math.hypot(
                    self.path[i][0] - self.follower.x,
                    self.path[i][1] - self.follower.y,
                ),
        )

    def lookahead_target(self):
        index = self.nearest_index()

        if index is None:
            return None

        travelled = 0.0

        for i in range(index, len(self.path) - 1):
            a = self.path[i]
            b = self.path[i + 1]

            travelled += math.hypot(
                b[0] - a[0],
                b[1] - a[1],
            )

            if travelled >= self.lookahead_distance:
                return b

        return self.path[-1]

    @staticmethod
    def path_cumulative_lengths(path):
        cumulative = [0.0]

        for a, b in zip(
            path[:-1],
            path[1:],
        ):
            cumulative.append(
                cumulative[-1]
                + math.hypot(
                    b[0] - a[0],
                    b[1] - a[1],
                )
            )

        return cumulative

    @staticmethod
    def point_at_path_s(path, cumulative, s_value):
        if not path:
            return None

        if len(path) == 1:
            return path[0]

        s_value = clamp(
            s_value,
            0.0,
            cumulative[-1],
        )

        for i in range(len(path) - 1):
            s0 = cumulative[i]
            s1 = cumulative[i + 1]

            if s_value <= s1:
                segment = s1 - s0

                if segment < 1e-12:
                    return path[i]

                t = (
                    s_value - s0
                ) / segment

                return (
                    path[i][0]
                    + t * (
                        path[i + 1][0]
                        - path[i][0]
                    ),
                    path[i][1]
                    + t * (
                        path[i + 1][1]
                        - path[i][1]
                    ),
                )

        return path[-1]

    def follow_curve_guidance(self):
        """
        Project R2 onto the smooth breadcrumb curve and use the
        local forward tangent for heading.

        Individual breadcrumbs are never steering targets. If R2
        passes one, the planner advances monotonically and this
        projection simply follows the next part of the curve.
        """
        if self.follower is None or len(self.path) < 2:
            return None

        cumulative = self.path_cumulative_lengths(
            self.path
        )

        best = None
        best_distance = float('inf')

        px = float(self.follower.x)
        py = float(self.follower.y)

        # Restrict projection to the local chronological prefix
        # of the planner-provided path. The planner has already
        # removed old history except for a few tangent-support
        # points, so there is no reason to search distant future
        # branches of a self-near loop.
        search_segment_count = len(self.path) - 1

        for i in range(len(cumulative)):
            if cumulative[i] > self.guidance_search_distance:
                search_segment_count = max(1, i)
                break

        for i in range(search_segment_count):
            a = self.path[i]
            b = self.path[i + 1]

            dx = b[0] - a[0]
            dy = b[1] - a[1]
            length2 = dx * dx + dy * dy

            if length2 < 1e-12:
                continue

            t = (
                (px - a[0]) * dx
                + (py - a[1]) * dy
            ) / length2

            t = clamp(t, 0.0, 1.0)

            qx = a[0] + t * dx
            qy = a[1] + t * dy

            d = math.hypot(
                px - qx,
                py - qy,
            )

            if d < best_distance:
                segment_length = math.sqrt(length2)

                best_distance = d
                best = (
                    i,
                    t,
                    qx,
                    qy,
                    cumulative[i]
                    + t * segment_length,
                )

        if best is None:
            return None

        _, _, qx, qy, projection_s = best

        half_window = max(
            0.10,
            self.follow_tangent_half_window,
        )

        before = self.point_at_path_s(
            self.path,
            cumulative,
            projection_s - half_window,
        )

        after = self.point_at_path_s(
            self.path,
            cumulative,
            projection_s + half_window,
        )

        tx = after[0] - before[0]
        ty = after[1] - before[1]
        tangent_length = math.hypot(tx, ty)

        if tangent_length < 1e-6:
            # One-sided fallback at a very short endpoint.
            i = min(
                len(self.path) - 2,
                best[0],
            )
            tx = (
                self.path[i + 1][0]
                - self.path[i][0]
            )
            ty = (
                self.path[i + 1][1]
                - self.path[i][1]
            )
            tangent_length = math.hypot(tx, ty)

        if tangent_length < 1e-6:
            return None

        tx /= tangent_length
        ty /= tangent_length

        tangent_heading = math.atan2(
            ty,
            tx,
        )

        # Positive cross-track means R2 is left of the forward
        # tangent. The correction steers right, hence the minus sign.
        nx = -ty
        ny = tx

        cross_track = (
            (px - qx) * nx
            + (py - qy) * ny
        )

        correction = clamp(
            -self.cross_track_heading_gain
            * cross_track,
            -self.max_cross_track_correction,
            self.max_cross_track_correction,
        )

        desired_heading = wrap_pi(
            tangent_heading
            + correction
        )

        return (
            desired_heading,
            tangent_heading,
            cross_track,
        )

    def publish_follow_guidance(
        self,
        desired_heading,
        tangent_heading,
        cross_track,
    ):
        self.last_desired_heading = desired_heading
        self.last_path_tangent_heading = tangent_heading
        self.last_cross_track_error = cross_track

        msg = Float64()
        msg.data = float(desired_heading)
        self.desired_heading_pub.publish(msg)

        msg = Float64()
        msg.data = float(tangent_heading)
        self.path_tangent_heading_pub.publish(msg)

        msg = Float64()
        msg.data = float(cross_track)
        self.cross_track_error_pub.publish(msg)

    def endpoint_distance(self):
        if self.follower is None or not self.path:
            return float('inf')

        goal = self.path[-1]

        return math.hypot(
            goal[0] - self.follower.x,
            goal[1] - self.follower.y,
        )

    def distance_pid(self, path_gap, dt):
        error = (
            path_gap
            - self.formation_distance
        )

        self.distance_integral = clamp(
            self.distance_integral
            + error * dt,
            -10.0,
            10.0,
        )

        derivative = (
            error
            - self.distance_previous
        ) / max(dt, 1e-6)

        self.distance_previous = error

        output = (
            self.distance_kp * error
            + self.distance_ki * self.distance_integral
            + self.distance_kd * derivative
        )

        return clamp(
            output,
            -self.max_distance_speed_correction,
            self.max_distance_speed_correction,
        )

    def brake(self, dt):
        surge = (
            self.follower.vx * math.cos(self.follower.body_yaw)
            + self.follower.vy * math.sin(self.follower.body_yaw)
        )

        error = -surge

        self.brake_integral = clamp(
            self.brake_integral
            + error * dt,
            -1.0,
            1.0,
        )

        derivative = (
            error
            - self.brake_previous
        ) / max(dt, 1e-6)

        self.brake_previous = error

        command = (
            self.brake_kp * error
            + self.brake_ki * self.brake_integral
            + self.brake_kd * derivative
        )

        command = clamp(
            command,
            -self.max_brake_thrust,
            self.max_brake_thrust,
        )

        self.publish_thrusters(
            command,
            command,
        )

    def reverse_away(self, dt):
        """
        Reverse only if reverse motion increases separation.
        Otherwise rotate in place until the stern points away.
        """
        r1_base = gps_to_base(self.predecessor)
        r2_base = gps_to_base(self.follower)

        away_x = r2_base[0] - r1_base[0]
        away_y = r2_base[1] - r1_base[1]

        length = math.hypot(
            away_x,
            away_y,
        )

        if length < 1e-6:
            self.stop()
            return

        away_x /= length
        away_y /= length

        forward_x = math.cos(self.follower.body_yaw)
        forward_y = math.sin(self.follower.body_yaw)

        reverse_alignment = (
            -forward_x * away_x
            -forward_y * away_y
        )

        # Reverse direction is sufficiently pointed away.
        if reverse_alignment >= 0.65:
            signed_surge = (
                self.follower.vx * forward_x
                + self.follower.vy * forward_y
            )

            error = (
                -self.parking_reverse_speed
                - signed_surge
            )

            command = clamp(
                self.parking_reverse_kp * error,
                -self.max_brake_thrust,
                0.0,
            )

            self.publish_thrusters(
                command,
                command,
            )
            return

        # Point the BOW toward R1, so reversing moves away.
        desired_yaw = math.atan2(
            -away_y,
            -away_x,
        )

        heading_error = wrap_pi(
            desired_yaw
            - self.follower.body_yaw
        )

        turn = clamp(
            500.0 * heading_error,
            -self.max_turn_thrust,
            self.max_turn_thrust,
        )

        self.publish_thrusters(
            turn,
            -turn,
        )

    def parking_alignment(self, now, dt):
        aligned_msg = Bool()
        aligned_msg.data = False

        # Stop reverse momentum before rotating.
        if self.follower.speed > 0.08:
            self.brake(dt)
            self.align_hold_start = None
            self.aligned_pub.publish(aligned_msg)
            return

        if len(self.path) < 2:
            self.stop()
            self.align_hold_start = None
            self.aligned_pub.publish(aligned_msg)
            return

        # Find first parking segment with meaningful length.
        origin = (
            float(self.follower.x),
            float(self.follower.y),
        )
        target = None

        for point in self.path[1:]:
            if math.hypot(
                point[0] - origin[0],
                point[1] - origin[1],
            ) > 1.0:
                target = point
                break

        if target is None:
            target = self.path[-1]

        desired = math.atan2(
            target[1] - self.follower.y,
            target[0] - self.follower.x,
        )

        error = wrap_pi(
            desired
            - self.follower.body_yaw
        )

        if abs(error) <= self.parking_align_tolerance:
            self.stop()

            if self.align_hold_start is None:
                self.align_hold_start = now

            held = (
                now
                - self.align_hold_start
            ).nanoseconds * 1e-9

            if held >= self.parking_align_hold:
                aligned_msg.data = True

            self.aligned_pub.publish(
                aligned_msg
            )
            return

        self.align_hold_start = None

        turn = clamp(
            500.0 * error,
            -self.max_turn_thrust,
            self.max_turn_thrust,
        )

        # Pure rotation: no forward component.
        self.publish_thrusters(
            turn,
            -turn,
        )

        self.aligned_pub.publish(
            aligned_msg
        )

    def track_breadcrumb_curve(
        self,
        requested_speed,
        dt,
        feedforward=0.0,
    ):
        guidance = self.follow_curve_guidance()

        if guidance is None:
            self.stop()
            return

        (
            desired_heading,
            tangent_heading,
            cross_track,
        ) = guidance

        self.publish_follow_guidance(
            desired_heading,
            tangent_heading,
            cross_track,
        )

        heading_error = wrap_pi(
            desired_heading
            - self.follower.body_yaw
        )

        msg = Float64()
        msg.data = float(desired_heading)
        self.desired_heading_pub.publish(msg)

        self.heading_integral = clamp(
            self.heading_integral
            + heading_error * dt,
            -1.5,
            1.5,
        )

        # Wrapped difference avoids an artificial derivative spike
        # when angle error crosses +/-pi.
        heading_derivative = wrap_pi(
            heading_error
            - self.heading_previous
        ) / max(dt, 1e-6)

        self.heading_previous = heading_error

        turn = (
            self.heading_kp * heading_error
            + self.heading_ki * self.heading_integral
            + self.heading_kd * heading_derivative
        )

        turn = clamp(
            turn,
            -self.max_turn_thrust,
            self.max_turn_thrust,
        )

        alignment = max(
            0.15,
            1.0 - abs(heading_error) / 1.1,
        )

        target_speed = (
            requested_speed
            * alignment
        )

        speed_error = (
            target_speed
            - self.follower.speed
        )

        self.speed_integral = clamp(
            self.speed_integral
            + speed_error * dt,
            -3.0,
            3.0,
        )

        speed_derivative = (
            speed_error
            - self.speed_previous
        ) / max(dt, 1e-6)

        self.speed_previous = speed_error

        forward = (
            self.speed_kp * speed_error
            + self.speed_ki * self.speed_integral
            + self.speed_kd * speed_derivative
            + feedforward
        )

        forward = clamp(
            forward,
            0.0,
            self.max_forward_thrust,
        )

        self.publish_thrusters(
            forward + turn,
            forward - turn,
        )

        msg = Float64()
        msg.data = float(heading_error)
        self.heading_error_pub.publish(msg)

        msg = Float64()
        msg.data = float(speed_error)
        self.speed_error_pub.publish(msg)

        msg = Float64()
        msg.data = float(target_speed)
        self.target_speed_pub.publish(msg)

    def track_path(
        self,
        requested_speed,
        dt,
        feedforward=0.0,
    ):
        target = self.lookahead_target()

        if target is None:
            self.stop()
            return

        desired_heading = math.atan2(
            target[1] - self.follower.y,
            target[0] - self.follower.x,
        )

        heading_error = wrap_pi(
            desired_heading
            - self.follower.body_yaw
        )

        msg = Float64()
        msg.data = float(desired_heading)
        self.desired_heading_pub.publish(msg)

        self.heading_integral = clamp(
            self.heading_integral
            + heading_error * dt,
            -1.5,
            1.5,
        )

        heading_derivative = wrap_pi(
            heading_error
            - self.heading_previous
        ) / max(dt, 1e-6)

        self.heading_previous = heading_error

        turn = (
            self.heading_kp * heading_error
            + self.heading_ki * self.heading_integral
            + self.heading_kd * heading_derivative
        )

        turn = clamp(
            turn,
            -self.max_turn_thrust,
            self.max_turn_thrust,
        )

        alignment = max(
            0.15,
            1.0 - abs(heading_error) / 1.1,
        )

        target_speed = (
            requested_speed
            * alignment
        )

        speed_error = (
            target_speed
            - self.follower.speed
        )

        self.speed_integral = clamp(
            self.speed_integral
            + speed_error * dt,
            -3.0,
            3.0,
        )

        speed_derivative = (
            speed_error
            - self.speed_previous
        ) / max(dt, 1e-6)

        self.speed_previous = speed_error

        forward = (
            self.speed_kp * speed_error
            + self.speed_ki * self.speed_integral
            + self.speed_kd * speed_derivative
            + feedforward
        )

        forward = clamp(
            forward,
            0.0,
            self.max_forward_thrust,
        )

        self.publish_thrusters(
            forward + turn,
            forward - turn,
        )

        msg = Float64()
        msg.data = float(heading_error)
        self.heading_error_pub.publish(msg)

        msg = Float64()
        msg.data = float(speed_error)
        self.speed_error_pub.publish(msg)

        msg = Float64()
        msg.data = float(target_speed)
        self.target_speed_pub.publish(msg)

    # ---------------------------------------------------------
    # Main control
    # ---------------------------------------------------------

    def historical_speed_recommended_callback(self, msg):
        value = float(msg.data)

        if not math.isfinite(value):
            return

        self.historical_speed_recommended = value
        self.historical_speed_recommended_time = (
            self.get_clock().now().nanoseconds * 1e-9
        )

    def control_loop(self):
        now = self.get_clock().now()

        source = String()
        source.data = self.path_source
        self.path_source_pub.publish(source)
        self.publish_terminal_status()

        if self.last_time is None:
            self.last_time = now
            return

        dt = (
            now
            - self.last_time
        ).nanoseconds * 1e-9

        self.last_time = now

        if dt <= 0.0 or self.follower is None:
            return

        clearance = self.hitbox_clearance()
        self.publish_safety(clearance)

        # Hard safety override applies in EVERY mission mode.
        if (
            self.predecessor is not None
            and self.avoidance_latched
        ):
            self.collision_avoidance()
            return

        if self.mode == 'FOLLOW':
            self.longitudinal_state = 'TRACK'
            self.terminal_settled = False

            if (
                self.predecessor is None
                or not self.path
            ):
                self.stop()
                return

            # V2.2 longitudinal control:
            # use bumper-to-bumper ARC LENGTH along the raw
            # breadcrumb chain. Hull clearance is safety only.
            if (
                not self.path_gap_valid
                or self.path_gap is None
                or self.path_gap_time is None
            ):
                self.stop()
                return

            path_gap_age = (
                self.get_clock().now().nanoseconds * 1e-9
                - self.path_gap_time
            )

            if path_gap_age > 0.5:
                self.stop()
                return

            correction = self.distance_pid(
                self.path_gap,
                dt,
            )

            target_speed = (
                self.predecessor.speed
                + correction
            )

            feedforward = 0.0

            if (
                self.path_gap
                >= self.catchup_distance
            ):
                target_speed = max(
                    target_speed,
                    self.catchup_min_speed,
                )

                feedforward = (
                    self.catchup_feedforward_thrust
                )

            target_speed = clamp(
                target_speed,
                0.0,
                self.follow_max_speed,
            )

            # V2.1 proactive preview:
            # keep catch-up unchanged; otherwise preview may only
            # reduce the existing reactive speed command.
            if (
                self.path_gap < self.catchup_distance
                and self.historical_speed_recommended is not None
                and self.historical_speed_recommended_time is not None
            ):
                preview_age = (
                    self.get_clock().now().nanoseconds * 1e-9
                    - self.historical_speed_recommended_time
                )

                if preview_age <= 0.5:
                    preview_cap = clamp(
                        self.historical_speed_recommended
                        + max(0.0, correction),
                        0.0,
                        self.follow_max_speed,
                    )

                    target_speed = min(
                        target_speed,
                        preview_cap,
                    )

            self.track_breadcrumb_curve(
                target_speed,
                dt,
                feedforward=feedforward,
            )
            return

        if self.mode in (
            'FORMATION_BRAKE',
            'FORMATION_HOLD',
        ):
            self.terminal_stop_control(
                now,
                dt,
            )
            return

        if self.mode == 'PARK_REVERSE':
            self.longitudinal_state = 'PARKING'
            self.terminal_settled = False

            if (
                clearance
                >= self.parking_reverse_clearance
            ):
                self.stop()
            else:
                self.reverse_away(dt)
            return

        if self.mode == 'PARK_ALIGN':
            self.longitudinal_state = 'PARKING'
            self.terminal_settled = False
            self.parking_alignment(
                now,
                dt,
            )
            return

        if self.mode == 'PARK':
            self.longitudinal_state = 'PARKING'
            self.terminal_settled = False

            if not self.path:
                self.stop()
                return

            remaining = self.endpoint_distance()

            if remaining <= self.goal_tolerance:
                if self.follower.speed > self.stop_speed_tolerance:
                    self.brake(dt)
                    self.parking_hold_start = None
                else:
                    self.stop()

                    if self.parking_hold_start is None:
                        self.parking_hold_start = now

                    held = (
                        now
                        - self.parking_hold_start
                    ).nanoseconds * 1e-9

                    if held >= self.parking_hold_time:
                        self.success_latched = True
            else:
                requested = min(
                    self.parking_speed,
                    max(
                        0.20,
                        remaining * 0.15,
                    ),
                )

                self.track_path(
                    requested,
                    dt,
                )

        elif self.mode in ('DWELL', 'SUCCESS'):
            self.longitudinal_state = 'PARKING'
            self.terminal_settled = False
            self.stop()

        if self.success_latched:
            self.stop()

            msg = Bool()
            msg.data = True
            self.success_pub.publish(msg)


def main():
    rclpy.init()

    node = FollowerPidController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop()
        except Exception:
            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
