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

            # This is now HULL-TO-HULL clearance, not GPS distance.
            'formation_distance': 5.0,
            'distance_kp': 0.35,
            'distance_ki': 0.02,
            'distance_kd': 0.10,

            'collision_warning_clearance': 3.0,
            'collision_release_clearance': 3.5,
            'parking_reverse_clearance': 6.0,

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

        self.r1 = None
        self.r2 = None

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

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_qos = qos

        self.create_subscription(
            VehicleState,
            '/r1/vehicle_state',
            self.r1_callback,
            20,
        )

        self.create_subscription(
            VehicleState,
            '/r2/vehicle_state',
            self.r2_callback,
            20,
        )

        self.create_subscription(
            String,
            '/platoon/mission_state',
            self.mode_callback,
            10,
        )

        # FOLLOW starts with breadcrumb subscription only.
        self.breadcrumb_subscription = self.create_subscription(
            Path,
            '/planner/r2/breadcrumb_path',
            self.breadcrumb_callback,
            qos,
        )

        self.parking_subscription = None

        self.left_pub = self.create_publisher(
            Float64,
            '/wamv2/thrusters/left/thrust',
            10,
        )
        self.right_pub = self.create_publisher(
            Float64,
            '/wamv2/thrusters/right/thrust',
            10,
        )
        self.left_pos_pub = self.create_publisher(
            Float64,
            '/wamv2/thrusters/left/pos',
            10,
        )
        self.right_pos_pub = self.create_publisher(
            Float64,
            '/wamv2/thrusters/right/pos',
            10,
        )

        self.success_pub = self.create_publisher(
            Bool,
            '/r2/success',
            qos,
        )

        self.path_source_pub = self.create_publisher(
            String,
            '/r2/control/path_source',
            10,
        )
        self.heading_error_pub = self.create_publisher(
            Float64,
            '/r2/control/heading_error',
            10,
        )
        self.speed_error_pub = self.create_publisher(
            Float64,
            '/r2/control/speed_error',
            10,
        )
        self.target_speed_pub = self.create_publisher(
            Float64,
            '/r2/control/target_speed',
            10,
        )

        self.clearance_pub = self.create_publisher(
            Float64,
            '/r2/safety/hitbox_clearance',
            10,
        )
        self.warning_pub = self.create_publisher(
            Bool,
            '/r2/safety/collision_warning',
            10,
        )
        self.avoidance_pub = self.create_publisher(
            Bool,
            '/r2/safety/avoidance_active',
            10,
        )
        self.aligned_pub = self.create_publisher(
            Bool,
            '/r2/parking/aligned',
            10,
        )

        self.timer = self.create_timer(
            0.10,
            self.control_loop,
        )

        self.get_logger().info(
            'R2 hitbox-aware follower controller ready.'
        )

    # ---------------------------------------------------------
    # Inputs / subscription handoff
    # ---------------------------------------------------------

    def r1_callback(self, msg):
        self.r1 = msg

    def r2_callback(self, msg):
        self.r2 = msg

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
                    '/planner/r2/parking_path',
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

        if new_mode == 'PARK_BRAKE':
            self.destroy_breadcrumb_subscription()
            self.path = []
            self.path_source = 'NONE'

        elif new_mode == 'PARK_REVERSE':
            self.path = []
            self.path_source = 'NONE'

        elif new_mode == 'PARK_ALIGN':
            self.path = []
            self.path_source = 'NONE'
            self.create_parking_subscription()
            self.align_hold_start = None

        self.mode = new_mode
        self.reset_pid()

    # ---------------------------------------------------------
    # Geometry / safety
    # ---------------------------------------------------------

    def hitbox_clearance(self):
        if self.r1 is None or self.r2 is None:
            return float('inf')

        return polygon_clearance(
            hitbox_polygon(self.r1),
            hitbox_polygon(self.r2),
        )

    def publish_safety(self, clearance):
        warning = (
            clearance
            < self.collision_warning_clearance
        )

        if warning:
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
        r1_base = gps_to_base(self.r1)
        r2_base = gps_to_base(self.r2)

        ax = r2_base[0] - r1_base[0]
        ay = r2_base[1] - r1_base[1]

        length = math.hypot(ax, ay)

        if length < 1e-6:
            self.stop()
            return

        ax /= length
        ay /= length

        fx = math.cos(self.r2.body_yaw)
        fy = math.sin(self.r2.body_yaw)

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
        if self.r2 is None or not self.path:
            return None

        return min(
            range(len(self.path)),
            key=lambda i:
                math.hypot(
                    self.path[i][0] - self.r2.x,
                    self.path[i][1] - self.r2.y,
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

    def endpoint_distance(self):
        if self.r2 is None or not self.path:
            return float('inf')

        goal = self.path[-1]

        return math.hypot(
            goal[0] - self.r2.x,
            goal[1] - self.r2.y,
        )

    def clearance_pid(self, clearance, dt):
        error = (
            clearance
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
            self.r2.vx * math.cos(self.r2.body_yaw)
            + self.r2.vy * math.sin(self.r2.body_yaw)
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
        r1_base = gps_to_base(self.r1)
        r2_base = gps_to_base(self.r2)

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

        forward_x = math.cos(self.r2.body_yaw)
        forward_y = math.sin(self.r2.body_yaw)

        reverse_alignment = (
            -forward_x * away_x
            -forward_y * away_y
        )

        # Reverse direction is sufficiently pointed away.
        if reverse_alignment >= 0.65:
            signed_surge = (
                self.r2.vx * forward_x
                + self.r2.vy * forward_y
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
            - self.r2.body_yaw
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
        if self.r2.speed > 0.08:
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
            float(self.r2.x),
            float(self.r2.y),
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
            target[1] - self.r2.y,
            target[0] - self.r2.x,
        )

        error = wrap_pi(
            desired
            - self.r2.body_yaw
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
            target[1] - self.r2.y,
            target[0] - self.r2.x,
        )

        heading_error = wrap_pi(
            desired_heading
            - self.r2.body_yaw
        )

        self.heading_integral = clamp(
            self.heading_integral
            + heading_error * dt,
            -1.5,
            1.5,
        )

        heading_derivative = (
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
            - self.r2.speed
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

    def control_loop(self):
        now = self.get_clock().now()

        source = String()
        source.data = self.path_source
        self.path_source_pub.publish(source)

        if self.last_time is None:
            self.last_time = now
            return

        dt = (
            now
            - self.last_time
        ).nanoseconds * 1e-9

        self.last_time = now

        if dt <= 0.0 or self.r2 is None:
            return

        clearance = self.hitbox_clearance()
        self.publish_safety(clearance)

        # Hard safety override applies in EVERY mission mode.
        if (
            self.r1 is not None
            and self.avoidance_latched
        ):
            self.collision_avoidance()
            return

        if self.mode == 'FOLLOW':
            if (
                self.r1 is None
                or not self.path
            ):
                self.stop()
                return

            correction = self.clearance_pid(
                clearance,
                dt,
            )

            target_speed = (
                self.r1.speed
                + correction
            )

            feedforward = 0.0

            if (
                clearance
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

            self.track_path(
                target_speed,
                dt,
                feedforward=feedforward,
            )
            return

        if self.mode == 'PARK_BRAKE':
            if self.r2.speed > self.stop_speed_tolerance:
                self.brake(dt)
            else:
                self.stop()
            return

        if self.mode == 'PARK_REVERSE':
            if (
                clearance
                >= self.parking_reverse_clearance
            ):
                self.stop()
            else:
                self.reverse_away(dt)
            return

        if self.mode == 'PARK_ALIGN':
            self.parking_alignment(
                now,
                dt,
            )
            return

        if self.mode == 'PARK':
            if not self.path:
                self.stop()
                return

            remaining = self.endpoint_distance()

            if remaining <= self.goal_tolerance:
                if self.r2.speed > self.stop_speed_tolerance:
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
