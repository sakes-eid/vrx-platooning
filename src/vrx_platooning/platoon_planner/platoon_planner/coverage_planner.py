import math

import rclpy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    ReliabilityPolicy,
)


class CoveragePlanner(Node):

    def __init__(self):

        super().__init__(
            'trajectory_planner'
        )

        # Keep compatibility with the existing launch architecture.
        self.declare_parameter(
            'trajectory_type',
            'coverage'
        )

        self.declare_parameter(
            'frame_id',
            'world_ned'
        )

        self.declare_parameter(
            'waypoint_spacing',
            0.5
        )

        self.declare_parameter(
            'coverage_north_min',
            163.0
        )

        self.declare_parameter(
            'coverage_north_max',
            178.0
        )

        self.declare_parameter(
            'coverage_east_start',
            -531.0
        )

        self.declare_parameter(
            'coverage_lane_spacing',
            12.0
        )

        self.declare_parameter(
            'coverage_lanes',
            3
        )

        self.frame_id = (
            self.get_parameter(
                'frame_id'
            ).value
        )

        self.spacing = float(
            self.get_parameter(
                'waypoint_spacing'
            ).value
        )

        self.north_min = float(
            self.get_parameter(
                'coverage_north_min'
            ).value
        )

        self.north_max = float(
            self.get_parameter(
                'coverage_north_max'
            ).value
        )

        self.east_start = float(
            self.get_parameter(
                'coverage_east_start'
            ).value
        )

        self.lane_spacing = float(
            self.get_parameter(
                'coverage_lane_spacing'
            ).value
        )

        self.lanes = int(
            self.get_parameter(
                'coverage_lanes'
            ).value
        )

        if self.spacing <= 0.0:
            raise RuntimeError(
                'waypoint_spacing must be > 0'
            )

        if self.lane_spacing <= 0.0:
            raise RuntimeError(
                'coverage_lane_spacing must be > 0'
            )

        if self.lanes < 1:
            raise RuntimeError(
                'coverage_lanes must be >= 1'
            )

        qos = QoSProfile(
            depth=1
        )

        qos.reliability = (
            ReliabilityPolicy.RELIABLE
        )

        qos.durability = (
            DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.publisher = (
            self.create_publisher(
                Path,
                '/planner/reference_path',
                qos
            )
        )

        self.points = (
            self.generate_coverage_path()
        )

        self.path_length = (
            self.calculate_length(
                self.points
            )
        )

        self.path_msg = (
            self.make_path_message(
                self.points
            )
        )

        # Repeat publishing as an extra safeguard.
        self.timer = (
            self.create_timer(
                1.0,
                self.publish_path
            )
        )

        self.publish_path()

        self.get_logger().info(
            'Coverage planner ready.'
        )

        self.get_logger().info(
            f'Frame: {self.frame_id}'
        )

        self.get_logger().info(
            f'Lanes: {self.lanes}'
        )

        self.get_logger().info(
            f'Lane spacing: '
            f'{self.lane_spacing:.2f} m'
        )

        self.get_logger().info(
            f'Waypoints: '
            f'{len(self.points)}'
        )

        self.get_logger().info(
            f'Path length: '
            f'{self.path_length:.2f} m'
        )

        self.get_logger().info(
            'Pattern: smooth boustrophedon'
        )


    def append_point(
        self,
        points,
        north,
        east,
    ):

        point = (
            float(north),
            float(east),
        )

        if points:

            dn = (
                point[0]
                - points[-1][0]
            )

            de = (
                point[1]
                - points[-1][1]
            )

            if math.hypot(
                dn,
                de
            ) < 1e-9:

                return

        points.append(
            point
        )


    def add_line(
        self,
        points,
        n0,
        e0,
        n1,
        e1,
    ):

        distance = math.hypot(
            n1 - n0,
            e1 - e0,
        )

        steps = max(
            1,
            int(
                math.ceil(
                    distance
                    / self.spacing
                )
            )
        )

        for i in range(
            steps + 1
        ):

            t = (
                i
                / steps
            )

            north = (
                n0
                + t
                * (n1 - n0)
            )

            east = (
                e0
                + t
                * (e1 - e0)
            )

            self.append_point(
                points,
                north,
                east,
            )


    def add_u_turn(
        self,
        points,
        north_end,
        east_current,
        direction,
    ):

        radius = (
            self.lane_spacing
            / 2.0
        )

        arc_length = (
            math.pi
            * radius
        )

        steps = max(
            12,
            int(
                math.ceil(
                    arc_length
                    / self.spacing
                )
            )
        )

        centre_east = (
            east_current
            + radius
        )

        for i in range(
            1,
            steps + 1
        ):

            phi = (
                math.pi
                * i
                / steps
            )

            # direction +1:
            # northbound lane -> U-turn above rectangle
            #
            # direction -1:
            # southbound lane -> U-turn below rectangle

            north = (
                north_end
                + direction
                * radius
                * math.sin(phi)
            )

            east = (
                centre_east
                - radius
                * math.cos(phi)
            )

            self.append_point(
                points,
                north,
                east,
            )


    def generate_coverage_path(
        self
    ):

        points = []

        for lane in range(
            self.lanes
        ):

            east = (
                self.east_start
                + lane
                * self.lane_spacing
            )

            northbound = (
                lane % 2 == 0
            )

            if northbound:

                n_start = (
                    self.north_min
                )

                n_end = (
                    self.north_max
                )

                direction = 1.0

            else:

                n_start = (
                    self.north_max
                )

                n_end = (
                    self.north_min
                )

                direction = -1.0

            self.add_line(
                points,
                n_start,
                east,
                n_end,
                east,
            )

            if lane < (
                self.lanes - 1
            ):

                self.add_u_turn(
                    points,
                    n_end,
                    east,
                    direction,
                )

        return points


    @staticmethod
    def calculate_length(
        points
    ):

        distance = 0.0

        for p0, p1 in zip(
            points[:-1],
            points[1:]
        ):

            distance += math.hypot(
                p1[0] - p0[0],
                p1[1] - p0[1],
            )

        return distance


    def make_path_message(
        self,
        points,
    ):

        msg = Path()

        msg.header.frame_id = (
            self.frame_id
        )

        for i, point in enumerate(
            points
        ):

            pose = PoseStamped()

            pose.header.frame_id = (
                self.frame_id
            )

            pose.pose.position.x = (
                point[0]
            )

            pose.pose.position.y = (
                point[1]
            )

            pose.pose.position.z = 0.0

            if i < (
                len(points) - 1
            ):

                dn = (
                    points[i + 1][0]
                    - point[0]
                )

                de = (
                    points[i + 1][1]
                    - point[1]
                )

            else:

                dn = (
                    point[0]
                    - points[i - 1][0]
                )

                de = (
                    point[1]
                    - points[i - 1][1]
                )

            # NED heading:
            # 0 = North
            # +pi/2 = East
            heading = math.atan2(
                de,
                dn
            )

            pose.pose.orientation.z = (
                math.sin(
                    heading / 2.0
                )
            )

            pose.pose.orientation.w = (
                math.cos(
                    heading / 2.0
                )
            )

            msg.poses.append(
                pose
            )

        return msg


    def publish_path(
        self
    ):

        stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        self.path_msg.header.stamp = (
            stamp
        )

        for pose in (
            self.path_msg.poses
        ):

            pose.header.stamp = (
                stamp
            )

        self.publisher.publish(
            self.path_msg
        )


def main():

    rclpy.init()

    node = CoveragePlanner()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()
