import math
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float64, String

from platoon_interfaces.msg import VehicleState

from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    ReliabilityPolicy,
)


def distance(a, b):
    return math.hypot(
        a[0] - b[0],
        a[1] - b[1],
    )


class TwoRobotPlanner(Node):

    FOLLOW = 'FOLLOW'
    PARK_BRAKE = 'PARK_BRAKE'
    PARK_REVERSE = 'PARK_REVERSE'
    PARK_ALIGN = 'PARK_ALIGN'
    PARK = 'PARK'
    DWELL = 'DWELL'
    SUCCESS = 'SUCCESS'

    def __init__(self):
        super().__init__('two_robot_planner')

        params = {
            # Hull-to-hull target is 5 m. For straight aligned
            # boats this corresponds to ~10.371 m reference lag.
            'formation_distance': 5.0,
            'breadcrumb_lag_distance': 10.371,

            'breadcrumb_spacing': 0.20,
            'filter_alpha': 0.35,
            'jump_limit': 2.0,
            'minimum_extra_trail': 1.0,

            # FOLLOW path is a smooth local curve built from the
            # historical breadcrumb trail. The follower never aims
            # directly at an individual breadcrumb.
            'curve_samples_per_segment': 3.0,
            'curve_tangent_scale': 0.50,
            'curve_backtrack_points': 4.0,
            'passed_point_margin': 0.05,

            'parking_separation': 18.0,
            'parking_escape_distance': 10.0,
            'parking_spacing': 0.5,

            'parking_brake_speed': 0.08,
            'parking_brake_hold': 0.5,

            'parking_reverse_clearance': 6.0,
            'parking_reverse_hold': 0.30,

            'dwell_time': 5.0,
        }

        for name, default in params.items():
            self.declare_parameter(name, default)

        for name in params:
            setattr(
                self,
                name,
                float(self.get_parameter(name).value),
            )

        self.r1 = None
        self.r2 = None

        self.filtered_r1 = None
        self.breadcrumbs = deque(maxlen=8000)

        # Monotonic progress through the leader's historical trail.
        # Once R2 passes a breadcrumb, this index never moves back.
        self.r2_progress_index = 0

        self.mode = self.FOLLOW
        self.previous_mode = None
        self.trail_ready = False

        self.leader_done = False
        self.r2_done = False

        self.hitbox_clearance = float('inf')
        self.parking_aligned = False

        self.brake_start = None
        self.reverse_ready_start = None
        self.dwell_start = None

        self.parking_path = None

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.breadcrumb_pub = self.create_publisher(
            Path,
            '/planner/r2/breadcrumb_path',
            qos,
        )

        self.parking_pub = self.create_publisher(
            Path,
            '/planner/r2/parking_path',
            qos,
        )

        self.mode_pub = self.create_publisher(
            String,
            '/platoon/mission_state',
            10,
        )

        self.success_pub = self.create_publisher(
            Bool,
            '/platoon/success',
            qos,
        )

        self.reference_distance_pub = self.create_publisher(
            Float64,
            '/r2/following/reference_distance',
            10,
        )

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
            Bool,
            '/experiment/success',
            self.r1_success_callback,
            10,
        )

        self.create_subscription(
            Bool,
            '/r2/success',
            self.r2_success_callback,
            10,
        )

        self.create_subscription(
            Float64,
            '/r2/safety/hitbox_clearance',
            self.clearance_callback,
            20,
        )

        self.create_subscription(
            Bool,
            '/r2/parking/aligned',
            self.aligned_callback,
            20,
        )

        self.timer = self.create_timer(
            0.10,
            self.update,
        )

        self.get_logger().info(
            'Two robot hitbox-aware planner ready.'
        )

    # ---------------------------------------------------------
    # Inputs
    # ---------------------------------------------------------

    def append_breadcrumb(self, point):
        # Keep progress valid if the bounded deque ever rolls over.
        if (
            len(self.breadcrumbs)
            == self.breadcrumbs.maxlen
        ):
            self.r2_progress_index = max(
                0,
                self.r2_progress_index - 1,
            )

        self.breadcrumbs.append(point)

    def r1_callback(self, msg):
        self.r1 = msg

        raw = (
            float(msg.x),
            float(msg.y),
        )

        if self.filtered_r1 is None:
            self.filtered_r1 = raw
            self.append_breadcrumb(raw)
            return

        if (
            distance(
                raw,
                self.filtered_r1,
            )
            > self.jump_limit
        ):
            return

        a = self.filter_alpha

        filtered = (
            a * raw[0]
            + (1.0 - a)
            * self.filtered_r1[0],

            a * raw[1]
            + (1.0 - a)
            * self.filtered_r1[1],
        )

        self.filtered_r1 = filtered

        if (
            distance(
                filtered,
                self.breadcrumbs[-1],
            )
            >= self.breadcrumb_spacing
        ):
            self.append_breadcrumb(
                filtered
            )

    def r2_callback(self, msg):
        self.r2 = msg

    def clearance_callback(self, msg):
        self.hitbox_clearance = float(
            msg.data
        )

    def aligned_callback(self, msg):
        self.parking_aligned = bool(
            msg.data
        )

    def r1_success_callback(self, msg):
        if msg.data and not self.leader_done:
            self.leader_done = True
            self.mode = self.PARK_BRAKE
            self.brake_start = None
            self.reverse_ready_start = None

            self.get_logger().info(
                'R1 completion latched; breadcrumb mode closed.'
            )

    def r2_success_callback(self, msg):
        if msg.data:
            self.r2_done = True

    # ---------------------------------------------------------
    # Breadcrumbs
    # ---------------------------------------------------------

    def breadcrumb_length(self):
        points = list(self.breadcrumbs)

        return sum(
            distance(a, b)
            for a, b in zip(
                points[:-1],
                points[1:],
            )
        )

    def historical_target(self):
        points = list(self.breadcrumbs)

        if len(points) < 2:
            return None

        remaining = (
            self.breadcrumb_lag_distance
        )

        for i in range(
            len(points) - 1,
            0,
            -1,
        ):
            newer = points[i]
            older = points[i - 1]

            segment = distance(
                newer,
                older,
            )

            if segment < 1e-9:
                continue

            if remaining <= segment:
                ratio = remaining / segment

                target = (
                    newer[0]
                    + ratio
                    * (
                        older[0]
                        - newer[0]
                    ),

                    newer[1]
                    + ratio
                    * (
                        older[1]
                        - newer[1]
                    ),
                )

                return target, i - 1

            remaining -= segment

        return None

    @staticmethod
    def segment_projection(point, a, b):
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        length2 = dx * dx + dy * dy

        if length2 < 1e-12:
            return 0.0, a, distance(point, a)

        t = (
            (point[0] - a[0]) * dx
            + (point[1] - a[1]) * dy
        ) / length2

        t = max(0.0, min(1.0, t))

        projection = (
            a[0] + t * dx,
            a[1] + t * dy,
        )

        return (
            t,
            projection,
            distance(point, projection),
        )

    def advance_r2_progress(self, history, target_index):
        """
        Advance monotonically through the breadcrumb trail.

        Two checks are used:
        1. nearest forward segment projection, and
        2. the user's skipped-point test: if the vector from R2 to
           the next breadcrumb has a negative dot product with the
           local forward tangent, that breadcrumb is behind R2 and
           is considered passed.
        """
        if self.r2 is None or len(history) < 2:
            return

        max_segment = min(
            target_index,
            len(history) - 2,
        )

        if max_segment < 0:
            return

        # Under normal operation max_segment only increases. The
        # deque-rollover helper above shifts the index when needed, so
        # we never intentionally move progress backwards.
        if self.r2_progress_index > max_segment:
            return

        r2_position = (
            float(self.r2.x),
            float(self.r2.y),
        )

        # First jump forward to the closest admissible segment.
        best_index = self.r2_progress_index
        best_distance = float('inf')

        for i in range(
            self.r2_progress_index,
            max_segment + 1,
        ):
            _, _, d = self.segment_projection(
                r2_position,
                history[i],
                history[i + 1],
            )

            if d < best_distance:
                best_distance = d
                best_index = i

        self.r2_progress_index = max(
            self.r2_progress_index,
            best_index,
        )

        # Then explicitly discard breadcrumbs that are behind R2
        # along the local trail tangent.
        while self.r2_progress_index < max_segment:
            i = self.r2_progress_index
            a = history[i]
            b = history[i + 1]

            tx = b[0] - a[0]
            ty = b[1] - a[1]
            length = math.hypot(tx, ty)

            if length < 1e-9:
                self.r2_progress_index += 1
                continue

            tx /= length
            ty /= length

            vx = b[0] - r2_position[0]
            vy = b[1] - r2_position[1]

            along = vx * tx + vy * ty

            if along < -self.passed_point_margin:
                self.r2_progress_index += 1
            else:
                break

    def smooth_curve(self, points):
        """
        Interpolating cubic Hermite / Catmull-Rom-style curve.

        The curve passes through the breadcrumbs, but heading is
        continuous instead of pointing from waypoint to waypoint.
        """
        if len(points) <= 2:
            return points

        samples = max(
            1,
            int(round(self.curve_samples_per_segment)),
        )

        tangent_scale = self.curve_tangent_scale

        tangents = []

        for i in range(len(points)):
            if i == 0:
                tx = points[1][0] - points[0][0]
                ty = points[1][1] - points[0][1]
            elif i == len(points) - 1:
                tx = points[-1][0] - points[-2][0]
                ty = points[-1][1] - points[-2][1]
            else:
                tx = 0.5 * (
                    points[i + 1][0]
                    - points[i - 1][0]
                )
                ty = 0.5 * (
                    points[i + 1][1]
                    - points[i - 1][1]
                )

            tangents.append((
                tangent_scale * tx,
                tangent_scale * ty,
            ))

        curve = [points[0]]

        for i in range(len(points) - 1):
            p0 = points[i]
            p1 = points[i + 1]
            m0 = tangents[i]
            m1 = tangents[i + 1]

            for step in range(1, samples + 1):
                t = step / samples
                t2 = t * t
                t3 = t2 * t

                h00 = 2.0 * t3 - 3.0 * t2 + 1.0
                h10 = t3 - 2.0 * t2 + t
                h01 = -2.0 * t3 + 3.0 * t2
                h11 = t3 - t2

                point = (
                    h00 * p0[0]
                    + h10 * m0[0]
                    + h01 * p1[0]
                    + h11 * m1[0],

                    h00 * p0[1]
                    + h10 * m0[1]
                    + h01 * p1[1]
                    + h11 * m1[1],
                )

                if distance(curve[-1], point) >= 0.01:
                    curve.append(point)

        return curve

    def build_breadcrumb_path(self):
        """
        Publish the leader's smooth historical centerline ahead of R2.

        Formation spacing is NOT encoded by choosing one breadcrumb as
        a steering target. The 5 m hull gap is handled independently by
        the follower's speed controller. This prevents an overshoot from
        ever creating a "turn around and recover an old waypoint" command.
        """
        if self.r2 is None:
            return None

        history = list(self.breadcrumbs)

        if len(history) < 2:
            return None

        latest_segment = len(history) - 2

        self.advance_r2_progress(
            history,
            latest_segment,
        )

        # Include a few already-passed points only so the interpolated
        # curve has a stable tangent immediately behind the projection.
        # Steering is based on curve tangent, never point chasing.
        start_index = max(
            0,
            self.r2_progress_index
            - int(round(self.curve_backtrack_points)),
        )

        control_points = history[start_index:]

        if len(control_points) < 2:
            return None

        return self.smooth_curve(
            control_points
        )

    # ---------------------------------------------------------
    # Parking
    # ---------------------------------------------------------

    def terminal_heading_vector(self):
        points = list(self.breadcrumbs)

        if len(points) >= 8:
            older = points[-8]
            newer = points[-1]

            dx = newer[0] - older[0]
            dy = newer[1] - older[1]

            length = math.hypot(dx, dy)

            if length > 0.1:
                return (
                    dx / length,
                    dy / length,
                )

        return (
            math.cos(self.r1.course_angle),
            math.sin(self.r1.course_angle),
        )

    def line_path(self, start, goal):
        length = distance(start, goal)

        count = max(
            1,
            int(
                math.ceil(
                    length
                    / self.parking_spacing
                )
            ),
        )

        return [
            (
                start[0]
                + (
                    goal[0] - start[0]
                )
                * i / count,

                start[1]
                + (
                    goal[1] - start[1]
                )
                * i / count,
            )
            for i in range(count + 1)
        ]

    def build_parking_path(self):
        r1 = (
            float(self.r1.x),
            float(self.r1.y),
        )

        r2 = (
            float(self.r2.x),
            float(self.r2.y),
        )

        hx, hy = self.terminal_heading_vector()

        px = -hy
        py = hx

        candidate_a = (
            r1[0]
            + self.parking_separation * px,
            r1[1]
            + self.parking_separation * py,
        )

        candidate_b = (
            r1[0]
            - self.parking_separation * px,
            r1[1]
            - self.parking_separation * py,
        )

        if (
            distance(r2, candidate_a)
            <= distance(r2, candidate_b)
        ):
            final = candidate_a
            sx, sy = px, py
        else:
            final = candidate_b
            sx, sy = -px, -py

        # First parking leg moves sideways AWAY from the
        # leader's final trajectory before heading to final.
        escape = (
            r2[0]
            + self.parking_escape_distance * sx,
            r2[1]
            + self.parking_escape_distance * sy,
        )

        leg1 = self.line_path(
            r2,
            escape,
        )

        leg2 = self.line_path(
            escape,
            final,
        )

        self.get_logger().info(
            f'R2 parking target created; '
            f'reference separation = '
            f'{distance(r1, final):.2f} m.'
        )

        return leg1 + leg2[1:]

    @staticmethod
    def path_message(points):
        msg = Path()
        msg.header.frame_id = 'world_ned'

        for point in points:
            pose = PoseStamped()
            pose.header.frame_id = 'world_ned'
            pose.pose.position.x = float(
                point[0]
            )
            pose.pose.position.y = float(
                point[1]
            )
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        return msg

    # ---------------------------------------------------------
    # Mission
    # ---------------------------------------------------------

    def publish_mode(self):
        msg = String()
        msg.data = self.mode
        self.mode_pub.publish(msg)

        if self.mode != self.previous_mode:
            self.get_logger().info(
                f'Mission state -> {self.mode}'
            )
            self.previous_mode = self.mode

    def publish_reference_distance(self):
        if self.r1 is None or self.r2 is None:
            return

        msg = Float64()
        msg.data = math.hypot(
            self.r1.x - self.r2.x,
            self.r1.y - self.r2.y,
        )
        self.reference_distance_pub.publish(
            msg
        )

    def update(self):
        now = self.get_clock().now()

        self.publish_mode()
        self.publish_reference_distance()

        if self.r1 is None or self.r2 is None:
            return

        if self.mode == self.FOLLOW:
            required = (
                self.breadcrumb_lag_distance
                + self.minimum_extra_trail
            )

            trail = self.breadcrumb_length()

            if trail < required:
                return

            if not self.trail_ready:
                self.trail_ready = True

                self.get_logger().info(
                    f'R1 trail ready: {trail:.2f} m; '
                    'R2 released.'
                )

            path = self.build_breadcrumb_path()

            if path:
                self.breadcrumb_pub.publish(
                    self.path_message(path)
                )

        elif self.mode == self.PARK_BRAKE:
            if self.r2.speed > self.parking_brake_speed:
                self.brake_start = None
                return

            if self.brake_start is None:
                self.brake_start = now
                return

            held = (
                now
                - self.brake_start
            ).nanoseconds * 1e-9

            if held >= self.parking_brake_hold:
                self.mode = self.PARK_REVERSE
                self.reverse_ready_start = None

        elif self.mode == self.PARK_REVERSE:
            if (
                self.hitbox_clearance
                < self.parking_reverse_clearance
            ):
                self.reverse_ready_start = None
                return

            if self.reverse_ready_start is None:
                self.reverse_ready_start = now
                return

            held = (
                now
                - self.reverse_ready_start
            ).nanoseconds * 1e-9

            if held >= self.parking_reverse_hold:
                self.parking_path = (
                    self.build_parking_path()
                )

                self.parking_aligned = False

                # Publish first, then announce PARK_ALIGN.
                # TRANSIENT_LOCAL guarantees a late subscriber
                # receives the path too.
                self.parking_pub.publish(
                    self.path_message(
                        self.parking_path
                    )
                )

                self.mode = self.PARK_ALIGN

        elif self.mode == self.PARK_ALIGN:
            self.parking_pub.publish(
                self.path_message(
                    self.parking_path
                )
            )

            if self.parking_aligned:
                self.mode = self.PARK
                self.r2_done = False

        elif self.mode == self.PARK:
            self.parking_pub.publish(
                self.path_message(
                    self.parking_path
                )
            )

            if self.r2_done:
                self.mode = self.DWELL
                self.dwell_start = now

        elif self.mode == self.DWELL:
            held = (
                now
                - self.dwell_start
            ).nanoseconds * 1e-9

            if held >= self.dwell_time:
                self.mode = self.SUCCESS

        elif self.mode == self.SUCCESS:
            msg = Bool()
            msg.data = True
            self.success_pub.publish(msg)


def main():
    rclpy.init()

    node = TwoRobotPlanner()

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
