#!/usr/bin/env python3
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float64, Int32, String

from platoon_interfaces.msg import VehicleState


Point = Tuple[float, float]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_pi(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def hypot2(a: Point, b: Point) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


class LeaderStressController(Node):
    """R1-only PID path tracker with planner turn-preview inputs.

    This controller deliberately lives beside the validated main leader
    controller. It is a tuning/test controller, so the working platooning
    controller is not overwritten while we optimize R1 for hard turns.

    The planner sends two live advisories:
      /planner/r1/speed_limit        curvature-aware target speed
      /planner/r1/lookahead_distance dynamic geometric preview

    The planner's speed profile is anticipatory, so speed is reduced before a
    turn rather than only after R1 is already inside it.
    """

    def __init__(self):
        # Keep the same node name as the production controller so the existing
        # leader_pid_tuned.yaml can seed this test controller automatically.
        super().__init__('leader_pid_controller')

        defaults = {
            # Current validated seed values.
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

            # New stress-test limits.
            'max_speed': 2.0,
            'fallback_speed': 1.0,
            'min_advisory_speed': 0.0,
            'max_forward_thrust': 1000.0,
            'max_turn_thrust': 500.0,
            'max_total_thrust': 1000.0,
            'max_brake_thrust': 600.0,

            # PID anti-windup.
            'heading_integral_limit': 1.5,
            'speed_integral_limit': 3.0,
            'brake_integral_limit': 2.0,

            # End-state logic.
            'goal_tolerance': 1.0,
            'recovery_enter_tolerance': 1.50,
            'recovery_speed': 0.55,
            'stop_speed_tolerance': 0.07,
            'settle_speed_tolerance': 0.09,
            'settle_hold_time': 1.0,
            # Terminal stop commitment.  This is intentionally separate from
            # the tunable brake PID gains: it decides WHEN to stop, not how
            # strongly the brake PID acts.
            'terminal_brake_decel': 0.55,
            'terminal_brake_margin_m': 0.75,

            # Guidance / progress.
            'waypoint_capture_radius': 0.45,
            'progress_corridor_m': 3.50,
            'max_waypoint_advances_per_cycle': 8.0,
            'control_rate_hz': 10.0,
        }

        for name, default in defaults.items():
            self.declare_parameter(name, default)
            setattr(self, name, float(self.get_parameter(name).value))

        self.r1 = None
        self.path: List[Point] = []
        self.path_s: List[float] = []
        self.progress_segment = 0

        self.planner_speed_limit = self.fallback_speed
        self.planner_lookahead = self.lookahead_distance
        self.turn_severity = 'UNKNOWN'
        self.distance_to_turn = float('nan')
        self.upcoming_curvature = 0.0

        self.state = 'TRACK'
        self.success_latched = False
        self.settle_start_time = None
        self.last_time = None

        self.heading_integral = 0.0
        self.heading_previous = 0.0
        self.speed_integral = 0.0
        self.speed_previous = 0.0
        self.brake_integral = 0.0
        self.brake_previous = 0.0

        self.last_reference = (float('nan'), float('nan'))
        self.last_target = (float('nan'), float('nan'))
        self.last_desired_heading = float('nan')
        self.last_heading_error = float('nan')
        self.last_speed_error = float('nan')
        self.last_cross_track = float('nan')

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            VehicleState,
            '/r1/vehicle_state',
            self.state_callback,
            20,
        )
        self.create_subscription(
            Path,
            '/planner/reference_path',
            self.path_callback,
            qos,
        )
        self.create_subscription(
            Float64,
            '/planner/r1/speed_limit',
            self.speed_limit_callback,
            10,
        )
        self.create_subscription(
            Float64,
            '/planner/r1/lookahead_distance',
            self.lookahead_callback,
            10,
        )
        self.create_subscription(
            Float64,
            '/planner/r1/upcoming_curvature',
            self.curvature_callback,
            10,
        )
        self.create_subscription(
            Float64,
            '/planner/r1/distance_to_turn',
            self.distance_to_turn_callback,
            10,
        )
        self.create_subscription(
            String,
            '/planner/r1/turn_severity',
            self.turn_severity_callback,
            10,
        )

        self.left_pub = self.create_publisher(
            Float64, '/wamv/thrusters/left/thrust', 10
        )
        self.right_pub = self.create_publisher(
            Float64, '/wamv/thrusters/right/thrust', 10
        )
        self.left_pos_pub = self.create_publisher(
            Float64, '/wamv/thrusters/left/pos', 10
        )
        self.right_pos_pub = self.create_publisher(
            Float64, '/wamv/thrusters/right/pos', 10
        )

        self.control_state_pub = self.create_publisher(
            String, '/r1/control/state', 10
        )
        self.desired_heading_pub = self.create_publisher(
            Float64, '/r1/control/desired_heading', 10
        )
        self.heading_error_pub = self.create_publisher(
            Float64, '/r1/control/heading_error', 10
        )
        self.target_speed_pub = self.create_publisher(
            Float64, '/r1/control/target_speed', 10
        )
        self.speed_error_pub = self.create_publisher(
            Float64, '/r1/control/speed_error', 10
        )
        self.lookahead_pub = self.create_publisher(
            Float64, '/r1/control/lookahead_distance', 10
        )
        self.cross_track_pub = self.create_publisher(
            Float64, '/r1/control/cross_track_error', 10
        )
        self.reference_point_pub = self.create_publisher(
            PoseStamped, '/r1/control/reference_point', 10
        )
        self.target_point_pub = self.create_publisher(
            PoseStamped, '/r1/control/target_point', 10
        )
        self.active_waypoint_pub = self.create_publisher(
            PoseStamped, '/r1/control/active_waypoint', 10
        )
        self.active_waypoint_index_pub = self.create_publisher(
            Int32, '/r1/control/active_waypoint_index', 10
        )
        self.active_waypoint_number_pub = self.create_publisher(
            Int32, '/r1/control/active_waypoint_number', 10
        )
        self.success_pub = self.create_publisher(
            Bool, '/r1/success', qos
        )

        period = 1.0 / max(1.0, self.control_rate_hz)
        self.timer = self.create_timer(period, self.control_loop)

        self.get_logger().info(
            'R1 stress PID controller ready: planner-aware speed + lookahead.'
        )

    # ------------------------------------------------------------------
    # Inputs
    # ------------------------------------------------------------------

    def state_callback(self, msg: VehicleState):
        self.r1 = msg

    def path_callback(self, msg: Path):
        points = [
            (float(p.pose.position.x), float(p.pose.position.y))
            for p in msg.poses
        ]
        if len(points) < 2:
            return

        self.path = points
        self.path_s = [0.0]
        for i in range(1, len(points)):
            self.path_s.append(
                self.path_s[-1] + hypot2(points[i - 1], points[i])
            )

        self.progress_segment = 0
        self.state = 'TRACK'
        self.success_latched = False
        self.settle_start_time = None
        self.reset_all_pid()

        self.get_logger().info(
            f'New stress reference path received: {len(points)} points, '
            f'{self.path_s[-1]:.1f} m.'
        )

    def speed_limit_callback(self, msg: Float64):
        self.planner_speed_limit = clamp(
            float(msg.data),
            self.min_advisory_speed,
            self.max_speed,
        )

    def lookahead_callback(self, msg: Float64):
        self.planner_lookahead = max(0.5, float(msg.data))

    def curvature_callback(self, msg: Float64):
        self.upcoming_curvature = max(0.0, float(msg.data))

    def distance_to_turn_callback(self, msg: Float64):
        self.distance_to_turn = float(msg.data)

    def turn_severity_callback(self, msg: String):
        self.turn_severity = msg.data

    # ------------------------------------------------------------------
    # Path geometry
    # ------------------------------------------------------------------

    @staticmethod
    def project_to_segment(p: Point, a: Point, b: Point):
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        length2 = dx * dx + dy * dy
        if length2 < 1e-12:
            return a, 0.0, (p[0] - a[0]) ** 2 + (p[1] - a[1]) ** 2

        t = (
            (p[0] - a[0]) * dx
            + (p[1] - a[1]) * dy
        ) / length2
        t = clamp(t, 0.0, 1.0)
        q = (a[0] + t * dx, a[1] + t * dy)
        d2 = (p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2
        return q, t, d2

    def ordered_projection(self, p: Point):
        """Project only onto the next chronological path segment.

        The old controller searched hundreds of future segments for the
        spatially closest one.  On a self-near course that allowed a jump
        from the long straight directly into a later loop.

        Path array order is now the waypoint number.  A waypoint can only be
        consumed after every lower-numbered waypoint has been consumed.  We
        advance one segment at a time and only when R1 is physically close
        enough to the segment endpoint / corridor.
        """
        if len(self.path) < 2:
            return None

        max_advances = max(
            1, int(self.max_waypoint_advances_per_cycle)
        )

        for _ in range(max_advances):
            i = min(self.progress_segment, len(self.path) - 2)
            q, t, d2 = self.project_to_segment(
                p, self.path[i], self.path[i + 1]
            )
            endpoint_distance = hypot2(p, self.path[i + 1])
            corridor_distance = math.sqrt(d2)

            captured = (
                endpoint_distance <= self.waypoint_capture_radius
                or (
                    t >= 0.999
                    and corridor_distance <= self.progress_corridor_m
                )
            )

            if captured and i < len(self.path) - 2:
                self.progress_segment = i + 1
                continue

            break

        i = min(self.progress_segment, len(self.path) - 2)
        q, t, d2 = self.project_to_segment(
            p, self.path[i], self.path[i + 1]
        )
        seg_len = hypot2(self.path[i], self.path[i + 1])
        current_s = self.path_s[i] + t * seg_len
        return (i, q, t, current_s, math.sqrt(d2))

    def publish_ordered_waypoint_progress(self):
        if len(self.path) < 2:
            return

        active_index = min(
            self.progress_segment + 1,
            len(self.path) - 1,
        )

        idx = Int32()
        idx.data = int(active_index)
        self.active_waypoint_index_pub.publish(idx)

        number = Int32()
        number.data = int(active_index + 1)
        self.active_waypoint_number_pub.publish(number)

        self.publish_point(
            self.active_waypoint_pub,
            self.path[active_index],
        )

    def point_at_s(self, target_s: float) -> Point:
        if not self.path:
            return (0.0, 0.0)
        if target_s <= 0.0:
            return self.path[0]
        if target_s >= self.path_s[-1]:
            return self.path[-1]

        lo = max(0, self.progress_segment)
        while lo + 1 < len(self.path_s) and self.path_s[lo + 1] < target_s:
            lo += 1

        hi = min(len(self.path) - 1, lo + 1)
        ds = self.path_s[hi] - self.path_s[lo]
        if ds < 1e-9:
            return self.path[lo]

        t = (target_s - self.path_s[lo]) / ds
        return (
            self.path[lo][0] + t * (self.path[hi][0] - self.path[lo][0]),
            self.path[lo][1] + t * (self.path[hi][1] - self.path[lo][1]),
        )

    def final_tangent_heading(self) -> float:
        if len(self.path) < 2:
            return float(self.r1.body_yaw) if self.r1 is not None else 0.0
        a = self.path[-2]
        b = self.path[-1]
        return math.atan2(b[1] - a[1], b[0] - a[0])

    # ------------------------------------------------------------------
    # PID helpers
    # ------------------------------------------------------------------

    def reset_all_pid(self):
        self.heading_integral = 0.0
        self.heading_previous = 0.0
        self.speed_integral = 0.0
        self.speed_previous = 0.0
        self.brake_integral = 0.0
        self.brake_previous = 0.0

    def heading_pid(self, error: float, dt: float) -> float:
        self.heading_integral = clamp(
            self.heading_integral + error * dt,
            -self.heading_integral_limit,
            self.heading_integral_limit,
        )
        derivative = wrap_pi(error - self.heading_previous) / dt
        self.heading_previous = error

        command = (
            self.heading_kp * error
            + self.heading_ki * self.heading_integral
            + self.heading_kd * derivative
        )
        return clamp(command, -self.max_turn_thrust, self.max_turn_thrust)

    def speed_pid(self, target_speed: float, measured_speed: float, dt: float) -> float:
        error = target_speed - measured_speed
        self.speed_integral = clamp(
            self.speed_integral + error * dt,
            -self.speed_integral_limit,
            self.speed_integral_limit,
        )
        derivative = (error - self.speed_previous) / dt
        self.speed_previous = error

        command = (
            self.speed_kp * error
            + self.speed_ki * self.speed_integral
            + self.speed_kd * derivative
        )

        # TRACK is allowed to apply moderate negative thrust when the planner
        # has warned about an upcoming turn and lowered target speed.
        command = clamp(
            command,
            -self.max_brake_thrust,
            self.max_forward_thrust,
        )
        self.last_speed_error = error
        return command

    def brake_pid(self, surge_speed: float, dt: float) -> float:
        error = -surge_speed
        self.brake_integral = clamp(
            self.brake_integral + error * dt,
            -self.brake_integral_limit,
            self.brake_integral_limit,
        )
        derivative = (error - self.brake_previous) / dt
        self.brake_previous = error

        return clamp(
            self.brake_kp * error
            + self.brake_ki * self.brake_integral
            + self.brake_kd * derivative,
            -self.max_brake_thrust,
            self.max_brake_thrust,
        )

    # ------------------------------------------------------------------
    # Control states
    # ------------------------------------------------------------------

    def surge_speed(self) -> float:
        return (
            float(self.r1.vx) * math.cos(float(self.r1.body_yaw))
            + float(self.r1.vy) * math.sin(float(self.r1.body_yaw))
        )

    def publish_mix(self, forward: float, turn: float):
        left = clamp(
            forward + turn,
            -self.max_total_thrust,
            self.max_total_thrust,
        )
        right = clamp(
            forward - turn,
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

    def track_control(self, dt: float):
        p = (float(self.r1.x), float(self.r1.y))
        projection = self.ordered_projection(p)
        if projection is None:
            self.publish_mix(0.0, 0.0)
            return

        seg_i, ref, seg_t, current_s, cross_track_mag = projection
        self.last_reference = ref
        self.publish_ordered_waypoint_progress()

        lookahead = max(0.5, self.planner_lookahead)
        target = self.point_at_s(current_s + lookahead)
        self.last_target = target

        desired_heading = math.atan2(
            target[1] - p[1],
            target[0] - p[0],
        )
        heading_error = wrap_pi(
            desired_heading - float(self.r1.body_yaw)
        )

        # Signed cross-track error based on local segment tangent.
        a = self.path[seg_i]
        b = self.path[min(seg_i + 1, len(self.path) - 1)]
        tx = b[0] - a[0]
        ty = b[1] - a[1]
        norm = math.hypot(tx, ty)
        if norm > 1e-9:
            tx /= norm
            ty /= norm
            nx, ny = -ty, tx
            signed_cross_track = (
                (p[0] - ref[0]) * nx
                + (p[1] - ref[1]) * ny
            )
        else:
            signed_cross_track = cross_track_mag

        target_speed = clamp(
            self.planner_speed_limit,
            self.min_advisory_speed,
            self.max_speed,
        )
        surge = self.surge_speed()

        turn = self.heading_pid(heading_error, dt)
        forward = self.speed_pid(target_speed, surge, dt)
        self.publish_mix(forward, turn)

        self.last_desired_heading = desired_heading
        self.last_heading_error = heading_error
        self.last_cross_track = signed_cross_track

        final_distance = hypot2(p, self.path[-1])
        remaining_s = max(0.0, self.path_s[-1] - current_s)

        # Commit to terminal braking from ALONG-PATH remaining distance, not
        # only from a small Euclidean target circle.  The previous logic could
        # miss the 1 m goal circle at speed, after which heading guidance kept
        # steering back toward the endpoint and R1 orbited it forever.
        #
        # Ordered progress makes this safe even on self-near loops: current_s
        # can only advance chronologically along the course.
        surge_for_stop = max(0.0, surge)
        stopping_distance = (
            (surge_for_stop * surge_for_stop)
            / max(1e-6, 2.0 * self.terminal_brake_decel)
            + self.terminal_brake_margin_m
        )

        if (
            remaining_s <= stopping_distance
            or (
                final_distance <= self.goal_tolerance
                and remaining_s <= max(2.0, self.goal_tolerance * 2.0)
            )
        ):
            self.set_state('BRAKE')

        self.publish_diagnostics(
            target_speed,
            lookahead,
            desired_heading,
            heading_error,
            signed_cross_track,
            ref,
            target,
        )

    def terminal_control(self, dt: float):
        p = (float(self.r1.x), float(self.r1.y))
        final = self.path[-1]
        final_distance = hypot2(p, final)
        surge = self.surge_speed()
        heading = self.final_tangent_heading()
        heading_error = wrap_pi(heading - float(self.r1.body_yaw))
        turn = self.heading_pid(heading_error, dt)

        if self.state == 'BRAKE':
            # Once terminal braking has been committed, finish the stop first.
            # Do NOT immediately cancel braking merely because R1 is still
            # outside the final tolerance; that old behaviour caused
            # BRAKE->RECOVER cycling and endpoint orbiting.
            if abs(float(self.r1.speed)) <= self.settle_speed_tolerance:
                self.publish_mix(0.0, 0.0)

                if final_distance <= self.goal_tolerance:
                    self.set_state('HOLD')
                    self.settle_start_time = self.get_clock().now()
                else:
                    # Stopped short (or slightly beyond) the final capture
                    # circle.  Re-approach slowly, then brake again inside it.
                    self.set_state('RECOVER')
                return

            forward = self.brake_pid(surge, dt)
            self.publish_mix(forward, turn)

        elif self.state == 'RECOVER':
            if final_distance <= self.goal_tolerance:
                self.set_state('BRAKE')
                return

            desired = math.atan2(
                final[1] - p[1],
                final[0] - p[0],
            )
            heading_error = wrap_pi(desired - float(self.r1.body_yaw))
            turn = self.heading_pid(heading_error, dt)
            forward = self.speed_pid(
                self.recovery_speed,
                surge,
                dt,
            )
            self.publish_mix(forward, turn)
            heading = desired

        elif self.state == 'HOLD':
            if final_distance > self.recovery_enter_tolerance:
                self.set_state('RECOVER')
                return

            if abs(float(self.r1.speed)) > self.stop_speed_tolerance:
                self.set_state('BRAKE')
                return

            self.publish_mix(0.0, 0.0)

            if self.settle_start_time is None:
                self.settle_start_time = self.get_clock().now()

            held = (
                self.get_clock().now() - self.settle_start_time
            ).nanoseconds * 1e-9
            if held >= self.settle_hold_time:
                self.set_state('SUCCESS')
                self.success_latched = True
                msg = Bool()
                msg.data = True
                self.success_pub.publish(msg)

        elif self.state == 'SUCCESS':
            self.publish_mix(0.0, 0.0)

        self.last_desired_heading = heading
        self.last_heading_error = heading_error
        self.last_cross_track = 0.0
        self.last_reference = final
        self.last_target = final
        self.last_speed_error = -surge

        self.publish_ordered_waypoint_progress()

        self.publish_diagnostics(
            0.0,
            self.planner_lookahead,
            heading,
            heading_error,
            0.0,
            final,
            final,
        )

    def set_state(self, new_state: str):
        if new_state == self.state:
            return
        self.get_logger().info(
            f'Controller state: {self.state} -> {new_state}'
        )
        self.state = new_state
        self.reset_all_pid()
        if new_state != 'HOLD':
            self.settle_start_time = None

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    def publish_point(self, publisher, point: Point):
        msg = PoseStamped()
        msg.header.frame_id = 'world_ned'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(point[0])
        msg.pose.position.y = float(point[1])
        msg.pose.orientation.w = 1.0
        publisher.publish(msg)

    def publish_diagnostics(
        self,
        target_speed: float,
        lookahead: float,
        desired_heading: float,
        heading_error: float,
        cross_track: float,
        reference: Point,
        target: Point,
    ):
        text = String()
        text.data = self.state
        self.control_state_pub.publish(text)

        msg = Float64()
        msg.data = float(desired_heading)
        self.desired_heading_pub.publish(msg)

        msg = Float64()
        msg.data = float(heading_error)
        self.heading_error_pub.publish(msg)

        msg = Float64()
        msg.data = float(target_speed)
        self.target_speed_pub.publish(msg)

        msg = Float64()
        msg.data = float(self.last_speed_error)
        self.speed_error_pub.publish(msg)

        msg = Float64()
        msg.data = float(lookahead)
        self.lookahead_pub.publish(msg)

        msg = Float64()
        msg.data = float(cross_track)
        self.cross_track_pub.publish(msg)

        self.publish_point(self.reference_point_pub, reference)
        self.publish_point(self.target_point_pub, target)

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def control_loop(self):
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 1e-4 or dt > 1.0:
            dt = 0.1

        if self.r1 is None or len(self.path) < 2:
            self.publish_mix(0.0, 0.0)
            return

        if self.state == 'TRACK':
            self.track_control(dt)
        else:
            self.terminal_control(dt)


def main(args=None):
    rclpy.init(args=args)
    node = LeaderStressController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_mix(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
