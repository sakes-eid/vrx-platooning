#!/usr/bin/env python3
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float64, String

from platoon_interfaces.msg import VehicleState


Point = Tuple[float, float]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_pi(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def distance(a: Point, b: Point) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def quat_from_yaw(yaw: float):
    # Pure yaw around +Z in the visualization frame. world_ned is used as a
    # planar frame here; RViz only needs a consistent orientation marker.
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


class StressCoursePlanner(Node):
    """Publishes a hard R1 test course and anticipatory turn advisories.

    The path is absolute world_ned. It is generated once from R1's first
    world_ned position, so there is no robot-relative fake coordinate frame.

    Geometry, in order:
      1. 80 m straight
      2. tight 90 deg turn
      3. 180 deg semicircle, small radius
      4. 180 deg semicircle, large radius
      5. short coverage section with tight and wide hairpins

    A speed profile is computed from curvature and then backward-propagated
    using a deceleration limit. This means the planner lowers the speed limit
    *before* R1 reaches a tight turn. R1 subscribes to the live advisories.
    """

    def __init__(self):
        # Keep the historical node name so existing launch/config files are
        # less likely to need special cases.
        super().__init__('trajectory_planner')

        defaults = {
            'point_spacing': 0.50,
            'straight_length': 80.0,
            'turn_90_radius': 5.0,
            'semicircle_small_radius': 6.0,
            'semicircle_large_radius': 10.0,
            'coverage_lane_length': 14.0,
            'coverage_tight_radius': 5.0,
            'coverage_wide_radius': 9.0,

            # Speed-profile limits.
            'max_speed': 2.0,
            'minimum_turn_speed': 0.65,
            'lateral_accel_limit': 0.30,
            'accel_limit': 0.45,
            'decel_limit': 0.55,

            # Planner preview / advisory.
            'preview_distance': 20.0,
            'turn_curvature_threshold': 0.040,
            'tight_curvature_threshold': 0.140,
            'medium_curvature_threshold': 0.070,
            'lookahead_min': 2.2,
            'lookahead_max': 7.0,
            'lookahead_speed_gain': 1.8,
            'tight_lookahead': 2.6,
            'medium_lookahead': 3.6,
            'advisory_rate_hz': 10.0,
        }

        for name, default in defaults.items():
            self.declare_parameter(name, default)
            setattr(self, name, float(self.get_parameter(name).value))

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.latched_qos = qos

        self.path_pub = self.create_publisher(
            Path,
            '/planner/reference_path',
            qos,
        )
        self.preview_pub = self.create_publisher(
            Path,
            '/planner/r1/preview_path',
            qos,
        )
        self.speed_limit_pub = self.create_publisher(
            Float64,
            '/planner/r1/speed_limit',
            10,
        )
        self.lookahead_pub = self.create_publisher(
            Float64,
            '/planner/r1/lookahead_distance',
            10,
        )
        self.curvature_pub = self.create_publisher(
            Float64,
            '/planner/r1/upcoming_curvature',
            10,
        )
        self.distance_to_turn_pub = self.create_publisher(
            Float64,
            '/planner/r1/distance_to_turn',
            10,
        )
        self.turn_severity_pub = self.create_publisher(
            String,
            '/planner/r1/turn_severity',
            10,
        )

        self.r1 = None
        self.path_points: List[Point] = []
        self.path_s: List[float] = []
        self.curvature: List[float] = []
        self.speed_profile: List[float] = []
        self.progress_index = 0
        self.initialized = False

        self.create_subscription(
            VehicleState,
            '/r1/vehicle_state',
            self.state_callback,
            20,
        )

        period = 1.0 / max(1.0, self.advisory_rate_hz)
        self.timer = self.create_timer(period, self.advisory_loop)

        self.get_logger().info(
            'R1 stress-course planner waiting for first world_ned state.'
        )

    # ------------------------------------------------------------------
    # Course generation
    # ------------------------------------------------------------------

    def append_straight(
        self,
        points: List[Point],
        pose: Tuple[float, float, float],
        length: float,
    ) -> Tuple[float, float, float]:
        x, y, heading = pose
        n = max(1, int(math.ceil(length / self.point_spacing)))
        start_x, start_y = x, y

        for i in range(1, n + 1):
            s = min(length, i * length / n)
            px = start_x + s * math.cos(heading)
            py = start_y + s * math.sin(heading)
            if distance(points[-1], (px, py)) > 1e-6:
                points.append((px, py))

        return (
            start_x + length * math.cos(heading),
            start_y + length * math.sin(heading),
            heading,
        )

    def append_arc(
        self,
        points: List[Point],
        pose: Tuple[float, float, float],
        radius: float,
        turn_angle: float,
    ) -> Tuple[float, float, float]:
        """Append a tangent-continuous circular arc.

        turn_angle is signed in the x=North,y=East plane. Positive turns
        increase heading (North -> East for +90 deg).
        """
        x, y, heading = pose
        sign = 1.0 if turn_angle >= 0.0 else -1.0

        # Unit normal toward the circle centre.
        nx = -math.sin(heading)
        ny = math.cos(heading)
        cx = x + sign * radius * nx
        cy = y + sign * radius * ny

        rx0 = x - cx
        ry0 = y - cy
        arc_length = abs(radius * turn_angle)
        n = max(2, int(math.ceil(arc_length / self.point_spacing)))

        for i in range(1, n + 1):
            theta = turn_angle * (i / n)
            c = math.cos(theta)
            s = math.sin(theta)
            rx = c * rx0 - s * ry0
            ry = s * rx0 + c * ry0
            p = (cx + rx, cy + ry)
            if distance(points[-1], p) > 1e-6:
                points.append(p)

        end_heading = wrap_pi(heading + turn_angle)
        return (points[-1][0], points[-1][1], end_heading)

    def build_course(self, start_x: float, start_y: float) -> List[Point]:
        points: List[Point] = [(start_x, start_y)]
        pose = (start_x, start_y, 0.0)  # First test leg is due North.

        # 1) Long acceleration / steady-state leg.
        pose = self.append_straight(
            points, pose, self.straight_length
        )

        # 2) Tight quarter-circle, North -> East.
        pose = self.append_arc(
            points,
            pose,
            self.turn_90_radius,
            math.radians(90.0),
        )

        # 3) Small 180 deg semicircle.
        pose = self.append_arc(
            points,
            pose,
            self.semicircle_small_radius,
            math.radians(180.0),
        )

        # 4) Large 180 deg semicircle, connected directly.
        pose = self.append_arc(
            points,
            pose,
            self.semicircle_large_radius,
            math.radians(180.0),
        )

        # Small connector to separate the coverage section visually.
        pose = self.append_straight(points, pose, 8.0)

        # 5) Coverage-style section: tight, wide, then tight hairpin.
        pose = self.append_straight(
            points, pose, self.coverage_lane_length
        )
        pose = self.append_arc(
            points,
            pose,
            self.coverage_tight_radius,
            math.radians(180.0),
        )
        pose = self.append_straight(
            points, pose, self.coverage_lane_length
        )
        pose = self.append_arc(
            points,
            pose,
            self.coverage_wide_radius,
            math.radians(-180.0),
        )
        self.append_straight(
            points, pose, self.coverage_lane_length
        )

        return points

    # ------------------------------------------------------------------
    # Curvature / speed profile
    # ------------------------------------------------------------------

    def cumulative_lengths(self, points: List[Point]) -> List[float]:
        s = [0.0]
        for i in range(1, len(points)):
            s.append(s[-1] + distance(points[i - 1], points[i]))
        return s

    def estimate_curvature(self, points: List[Point]) -> List[float]:
        n = len(points)
        kappa = [0.0] * n

        for i in range(1, n - 1):
            a = points[i - 1]
            b = points[i]
            c = points[i + 1]

            ab = distance(a, b)
            bc = distance(b, c)
            ac = distance(a, c)
            denom = ab * bc * ac

            if denom < 1e-9:
                continue

            cross2 = abs(
                (b[0] - a[0]) * (c[1] - a[1])
                - (b[1] - a[1]) * (c[0] - a[0])
            )

            # kappa = 4A/(abc); cross2 is twice the triangle area.
            kappa[i] = 2.0 * cross2 / denom

        if n >= 2:
            kappa[0] = kappa[1]
            kappa[-1] = kappa[-2]

        return kappa

    def build_speed_profile(
        self,
        points: List[Point],
        path_s: List[float],
        curvature: List[float],
    ) -> List[float]:
        speed = []
        for k in curvature:
            if k <= 1e-6:
                v = self.max_speed
            else:
                v = math.sqrt(
                    max(0.0, self.lateral_accel_limit / k)
                )
                v = clamp(
                    v,
                    self.minimum_turn_speed,
                    self.max_speed,
                )
            speed.append(v)

        # Force terminal stop; backward pass creates a physically sensible
        # heads-up deceleration envelope before the final target.
        speed[-1] = 0.0

        for i in range(len(speed) - 2, -1, -1):
            ds = max(1e-6, path_s[i + 1] - path_s[i])
            reachable = math.sqrt(
                speed[i + 1] ** 2
                + 2.0 * self.decel_limit * ds
            )
            speed[i] = min(speed[i], reachable)

        # Forward acceleration envelope prevents an immediate jump back to
        # 2 m/s after a tight turn.
        for i in range(1, len(speed)):
            ds = max(1e-6, path_s[i] - path_s[i - 1])
            reachable = math.sqrt(
                speed[i - 1] ** 2
                + 2.0 * self.accel_limit * ds
            )
            speed[i] = min(speed[i], reachable)

        return speed

    # ------------------------------------------------------------------
    # ROS publication / live advisory
    # ------------------------------------------------------------------

    def path_message(self, points: List[Point]) -> Path:
        msg = Path()
        msg.header.frame_id = 'world_ned'
        msg.header.stamp = self.get_clock().now().to_msg()

        for i, (x, y) in enumerate(points):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0

            if len(points) >= 2:
                if i < len(points) - 1:
                    dx = points[i + 1][0] - x
                    dy = points[i + 1][1] - y
                else:
                    dx = x - points[i - 1][0]
                    dy = y - points[i - 1][1]
                yaw = math.atan2(dy, dx)
                qx, qy, qz, qw = quat_from_yaw(yaw)
                pose.pose.orientation.x = qx
                pose.pose.orientation.y = qy
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw
            else:
                pose.pose.orientation.w = 1.0

            msg.poses.append(pose)

        return msg

    def publish_full_path(self):
        self.path_pub.publish(self.path_message(self.path_points))

    def nearest_progress_index(self, x: float, y: float) -> int:
        if not self.path_points:
            return 0

        # Monotonic search: never deliberately move progress backwards.
        start = max(0, self.progress_index - 2)
        best_i = self.progress_index
        best_d2 = float('inf')

        # Search a generous forward window. This handles high speed and the
        # sharp turns without scanning the whole course every 0.1 s.
        stop = min(len(self.path_points), start + 180)
        for i in range(start, stop):
            px, py = self.path_points[i]
            d2 = (px - x) ** 2 + (py - y) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_i = i

        self.progress_index = max(self.progress_index, best_i)
        return self.progress_index

    def preview_end_index(self, index: int) -> int:
        end_s = self.path_s[index] + self.preview_distance
        j = index
        while j + 1 < len(self.path_s) and self.path_s[j] < end_s:
            j += 1
        return j

    def turn_advisory(self, index: int):
        end = self.preview_end_index(index)
        max_k = 0.0
        next_turn_i = None

        for i in range(index, end + 1):
            k = self.curvature[i]
            max_k = max(max_k, k)
            if (
                next_turn_i is None
                and k >= self.turn_curvature_threshold
            ):
                next_turn_i = i

        if next_turn_i is None:
            distance_to_turn = self.preview_distance
            severity = 'STRAIGHT'
        else:
            distance_to_turn = max(
                0.0,
                self.path_s[next_turn_i] - self.path_s[index],
            )
            k = self.curvature[next_turn_i]
            if k >= self.tight_curvature_threshold:
                severity = 'TIGHT'
            elif k >= self.medium_curvature_threshold:
                severity = 'MEDIUM'
            else:
                severity = 'WIDE'

        speed_limit = self.speed_profile[index]

        # Base lookahead grows with speed, but upcoming curvature shortens it.
        lookahead = clamp(
            self.lookahead_min
            + self.lookahead_speed_gain * speed_limit,
            self.lookahead_min,
            self.lookahead_max,
        )

        if max_k >= self.tight_curvature_threshold:
            lookahead = min(lookahead, self.tight_lookahead)
        elif max_k >= self.medium_curvature_threshold:
            lookahead = min(lookahead, self.medium_lookahead)

        return (
            speed_limit,
            lookahead,
            max_k,
            distance_to_turn,
            severity,
            end,
        )

    def state_callback(self, msg: VehicleState):
        self.r1 = msg

        if self.initialized:
            return

        start_x = float(msg.x)
        start_y = float(msg.y)
        self.path_points = self.build_course(start_x, start_y)
        self.path_s = self.cumulative_lengths(self.path_points)
        self.curvature = self.estimate_curvature(self.path_points)
        self.speed_profile = self.build_speed_profile(
            self.path_points,
            self.path_s,
            self.curvature,
        )

        self.initialized = True
        self.publish_full_path()

        self.get_logger().info(
            'Stress course published in world_ned: '
            f'{len(self.path_points)} points, '
            f'{self.path_s[-1]:.1f} m total length. '
            f'R1 ceiling={self.max_speed:.2f} m/s.'
        )

    def advisory_loop(self):
        if not self.initialized or self.r1 is None:
            return

        idx = self.nearest_progress_index(
            float(self.r1.x),
            float(self.r1.y),
        )

        (
            speed_limit,
            lookahead,
            max_k,
            distance_to_turn,
            severity,
            preview_end,
        ) = self.turn_advisory(idx)

        msg = Float64()
        msg.data = float(speed_limit)
        self.speed_limit_pub.publish(msg)

        msg = Float64()
        msg.data = float(lookahead)
        self.lookahead_pub.publish(msg)

        msg = Float64()
        msg.data = float(max_k)
        self.curvature_pub.publish(msg)

        msg = Float64()
        msg.data = float(distance_to_turn)
        self.distance_to_turn_pub.publish(msg)

        text = String()
        text.data = severity
        self.turn_severity_pub.publish(text)

        preview_start = max(0, idx - 2)
        preview_points = self.path_points[
            preview_start:preview_end + 1
        ]
        self.preview_pub.publish(
            self.path_message(preview_points)
        )


def main(args=None):
    rclpy.init(args=args)
    node = StressCoursePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
