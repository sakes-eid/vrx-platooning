import math
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from std_msgs.msg import (
    Bool,
    Float64,
    String,
)

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
    PARK = 'PARK'
    DWELL = 'DWELL'
    SUCCESS = 'SUCCESS'

    def __init__(self):

        super().__init__('two_robot_planner')

        self.declare_parameter(
            'formation_distance',
            5.0
        )

        self.declare_parameter(
            'breadcrumb_spacing',
            0.20
        )

        self.declare_parameter(
            'filter_alpha',
            0.35
        )

        self.declare_parameter(
            'jump_limit',
            2.0
        )

        self.declare_parameter(
            'minimum_extra_trail',
            1.0
        )

        self.declare_parameter(
            'parking_separation',
            18.0
        )

        self.declare_parameter(
            'parking_escape_distance',
            10.0
        )

        self.declare_parameter(
            'parking_spacing',
            0.5
        )

        self.declare_parameter(
            'parking_brake_speed',
            0.08
        )

        self.declare_parameter(
            'parking_brake_hold',
            0.5
        )

        self.declare_parameter(
            'dwell_time',
            5.0
        )

        self.formation_distance = float(
            self.get_parameter(
                'formation_distance'
            ).value
        )

        self.breadcrumb_spacing = float(
            self.get_parameter(
                'breadcrumb_spacing'
            ).value
        )

        self.filter_alpha = float(
            self.get_parameter(
                'filter_alpha'
            ).value
        )

        self.jump_limit = float(
            self.get_parameter(
                'jump_limit'
            ).value
        )

        self.minimum_extra_trail = float(
            self.get_parameter(
                'minimum_extra_trail'
            ).value
        )

        self.parking_separation = float(
            self.get_parameter(
                'parking_separation'
            ).value
        )

        self.parking_escape_distance = float(
            self.get_parameter(
                'parking_escape_distance'
            ).value
        )

        self.parking_spacing = float(
            self.get_parameter(
                'parking_spacing'
            ).value
        )

        self.parking_brake_speed = float(
            self.get_parameter(
                'parking_brake_speed'
            ).value
        )

        self.parking_brake_hold = float(
            self.get_parameter(
                'parking_brake_hold'
            ).value
        )

        self.dwell_time = float(
            self.get_parameter(
                'dwell_time'
            ).value
        )

        self.r1 = None
        self.r2 = None

        self.filtered_r1 = None

        self.breadcrumbs = deque(
            maxlen=8000
        )

        self.mode = self.FOLLOW

        self.previous_mode = None

        self.trail_ready = False

        self.leader_done = False

        self.r2_done = False

        self.brake_start = None

        self.parking_path = None

        self.dwell_start = None

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

        self.distance_pub = self.create_publisher(
            Float64,
            '/r2/following/distance',
            10,
        )

        self.distance_error_pub = self.create_publisher(
            Float64,
            '/r2/following/distance_error',
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

        self.timer = self.create_timer(
            0.1,
            self.update,
        )

        self.get_logger().info(
            'Two robot planner ready.'
        )


    def r1_callback(self, msg):

        self.r1 = msg

        raw = (
            float(msg.x),
            float(msg.y),
        )

        if self.filtered_r1 is None:

            self.filtered_r1 = raw

            self.breadcrumbs.append(raw)

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

            self.breadcrumbs.append(
                filtered
            )


    def r2_callback(self, msg):

        self.r2 = msg


    def r1_success_callback(self, msg):

        if msg.data and not self.leader_done:

            self.leader_done = True

            self.mode = self.PARK_BRAKE

            self.brake_start = None

            self.get_logger().info(
                'R1 completion latched.'
            )

            self.get_logger().info(
                'Breadcrumb publication permanently stopped.'
            )


    def r2_success_callback(self, msg):

        if msg.data:
            self.r2_done = True


    def breadcrumb_length(self):

        total = 0.0

        points = list(
            self.breadcrumbs
        )

        for a, b in zip(
            points[:-1],
            points[1:],
        ):

            total += distance(
                a,
                b,
            )

        return total


    def historical_target(self):

        points = list(
            self.breadcrumbs
        )

        if len(points) < 2:
            return None

        remaining = self.formation_distance

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


    def build_breadcrumb_path(self):

        if self.r2 is None:
            return None

        result = self.historical_target()

        if result is None:
            return None

        target, target_index = result

        history = list(
            self.breadcrumbs
        )

        allowed = history[
            :target_index + 1
        ]

        r2_position = (
            self.r2.x,
            self.r2.y,
        )

        if not allowed:

            return [
                r2_position,
                target,
            ]

        nearest = min(
            range(
                len(allowed)
            ),
            key=lambda i:
                distance(
                    r2_position,
                    allowed[i],
                )
        )

        path = [
            r2_position
        ]

        for point in allowed[
            nearest:
        ]:

            if (
                distance(
                    path[-1],
                    point,
                )
                >= 0.05
            ):

                path.append(point)

        if (
            distance(
                path[-1],
                target,
            )
            >= 0.05
        ):

            path.append(target)

        return path


    def terminal_heading_vector(self):

        points = list(
            self.breadcrumbs
        )

        if len(points) >= 8:

            older = points[-8]
            newer = points[-1]

            dx = newer[0] - older[0]
            dy = newer[1] - older[1]

            length = math.hypot(
                dx,
                dy,
            )

            if length > 0.1:

                return (
                    dx / length,
                    dy / length,
                )

        return (
            math.cos(
                self.r1.course_angle
            ),
            math.sin(
                self.r1.course_angle
            ),
        )


    def line_path(
        self,
        start,
        goal,
    ):

        length = distance(
            start,
            goal,
        )

        count = max(
            1,
            int(
                math.ceil(
                    length
                    / self.parking_spacing
                )
            )
        )

        return [
            (
                start[0]
                + (
                    goal[0]
                    - start[0]
                )
                * i / count,

                start[1]
                + (
                    goal[1]
                    - start[1]
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

        hx, hy = (
            self.terminal_heading_vector()
        )

        # Perpendicular to R1 path.
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

        # Use the side already closest to R2.
        #
        # This avoids forcing R2 across R1's final location.
        if (
            distance(
                r2,
                candidate_a,
            )
            <=
            distance(
                r2,
                candidate_b,
            )
        ):

            final = candidate_a

            sx = px
            sy = py

        else:

            final = candidate_b

            sx = -px
            sy = -py

        # First move away from R1 laterally.
        escape = (
            r2[0]
            + self.parking_escape_distance
            * sx,

            r2[1]
            + self.parking_escape_distance
            * sy,
        )

        leg1 = self.line_path(
            r2,
            escape,
        )

        leg2 = self.line_path(
            escape,
            final,
        )

        final_distance = distance(
            r1,
            final,
        )

        self.get_logger().info(
            f'R1 final position: '
            f'{r1[0]:.2f}, {r1[1]:.2f}'
        )

        self.get_logger().info(
            f'R2 parking target: '
            f'{final[0]:.2f}, {final[1]:.2f}'
        )

        self.get_logger().info(
            f'Parking separation: '
            f'{final_distance:.2f} m'
        )

        return (
            leg1
            + leg2[1:]
        )


    @staticmethod
    def path_message(points):

        msg = Path()

        msg.header.frame_id = 'world_ned'

        for point in points:

            pose = PoseStamped()

            pose.header.frame_id = (
                'world_ned'
            )

            pose.pose.position.x = float(
                point[0]
            )

            pose.pose.position.y = float(
                point[1]
            )

            pose.pose.orientation.w = 1.0

            msg.poses.append(pose)

        return msg


    def publish_mode(self):

        msg = String()

        msg.data = self.mode

        self.mode_pub.publish(msg)

        if self.mode != self.previous_mode:

            self.get_logger().info(
                f'Mission state -> {self.mode}'
            )

            self.previous_mode = self.mode


    def publish_distance(self):

        if (
            self.r1 is None
            or self.r2 is None
        ):

            return

        d12 = math.hypot(
            self.r1.x - self.r2.x,
            self.r1.y - self.r2.y,
        )

        msg = Float64()
        msg.data = d12

        self.distance_pub.publish(msg)

        msg = Float64()

        msg.data = (
            d12
            - self.formation_distance
        )

        self.distance_error_pub.publish(
            msg
        )


    def update(self):

        now = self.get_clock().now()

        self.publish_mode()
        self.publish_distance()

        if (
            self.r1 is None
            or self.r2 is None
        ):

            return

        # =====================================================
        # FOLLOW
        # =====================================================

        if self.mode == self.FOLLOW:

            required = (
                self.formation_distance
                + self.minimum_extra_trail
            )

            trail = self.breadcrumb_length()

            if trail < required:
                return

            if not self.trail_ready:

                self.trail_ready = True

                self.get_logger().info(
                    f'R1 breadcrumb trail ready: '
                    f'{trail:.2f} m'
                )

                self.get_logger().info(
                    'R2 released.'
                )

            path = (
                self.build_breadcrumb_path()
            )

            if path:

                self.breadcrumb_pub.publish(
                    self.path_message(path)
                )

        # =====================================================
        # PARK_BRAKE
        # =====================================================

        elif self.mode == self.PARK_BRAKE:

            if (
                self.r2.speed
                > self.parking_brake_speed
            ):

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

                self.parking_path = (
                    self.build_parking_path()
                )

                self.r2_done = False

                # Publish parking path on a COMPLETELY
                # different topic.
                self.parking_pub.publish(
                    self.path_message(
                        self.parking_path
                    )
                )

                self.mode = self.PARK

        # =====================================================
        # PARK
        # =====================================================

        elif self.mode == self.PARK:

            # Keep publishing; also TRANSIENT_LOCAL.
            self.parking_pub.publish(
                self.path_message(
                    self.parking_path
                )
            )

            if self.r2_done:

                self.mode = self.DWELL

                self.dwell_start = now

        # =====================================================
        # DWELL
        # =====================================================

        elif self.mode == self.DWELL:

            held = (
                now
                - self.dwell_start
            ).nanoseconds * 1e-9

            if held >= self.dwell_time:

                self.mode = self.SUCCESS

        # =====================================================
        # SUCCESS
        # =====================================================

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
