import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class TrajectoryPlanner(Node):

    def __init__(self):

        super().__init__('trajectory_planner')

        # ---------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------

        self.declare_parameter(
            'trajectory_type',
            'straight'
        )

        self.declare_parameter(
            'frame_id',
            'world_ned'
        )

        self.declare_parameter(
            'waypoint_spacing',
            0.5
        )

        # Straight trajectory

        self.declare_parameter(
            'straight_start_x',
            163.0
        )

        self.declare_parameter(
            'straight_start_y',
            -531.0
        )

        self.declare_parameter(
            'straight_end_x',
            183.0
        )

        self.declare_parameter(
            'straight_end_y',
            -531.0
        )

        # Circular / curved trajectory

        self.declare_parameter(
            'circle_center_x',
            173.0
        )

        self.declare_parameter(
            'circle_center_y',
            -531.0
        )

        self.declare_parameter(
            'circle_radius',
            10.0
        )

        self.declare_parameter(
            'circle_start_angle',
            math.pi
        )

        self.declare_parameter(
            'circle_end_angle',
            0.0
        )

        # ---------------------------------------------------------
        # Read parameters
        # ---------------------------------------------------------

        self.trajectory_type = (
            self.get_parameter(
                'trajectory_type'
            ).value
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

        # ---------------------------------------------------------
        # Publisher
        # ---------------------------------------------------------

        self.path_pub = self.create_publisher(
            Path,
            '/planner/reference_path',
            10
        )

        # Re-publish so late subscribers can still receive it.
        self.timer = self.create_timer(
            1.0,
            self.publish_path
        )

        self.get_logger().info(
            f'Trajectory planner started: '
            f'{self.trajectory_type}'
        )

        self.get_logger().info(
            f'Frame: {self.frame_id} '
            f'(x=North, y=East)'
        )

    # -------------------------------------------------------------
    # Straight trajectory
    # -------------------------------------------------------------

    def generate_straight_path(self):

        x0 = float(
            self.get_parameter(
                'straight_start_x'
            ).value
        )

        y0 = float(
            self.get_parameter(
                'straight_start_y'
            ).value
        )

        x1 = float(
            self.get_parameter(
                'straight_end_x'
            ).value
        )

        y1 = float(
            self.get_parameter(
                'straight_end_y'
            ).value
        )

        dx = x1 - x0
        dy = y1 - y0

        length = math.hypot(
            dx,
            dy
        )

        number_of_segments = max(
            1,
            math.ceil(
                length / self.spacing
            )
        )

        points = []

        for i in range(
            number_of_segments + 1
        ):

            ratio = (
                i / number_of_segments
            )

            x = (
                x0
                + ratio * dx
            )

            y = (
                y0
                + ratio * dy
            )

            points.append(
                (x, y)
            )

        return points

    # -------------------------------------------------------------
    # Circular / curved trajectory
    # -------------------------------------------------------------

    def generate_circle_path(self):

        center_x = float(
            self.get_parameter(
                'circle_center_x'
            ).value
        )

        center_y = float(
            self.get_parameter(
                'circle_center_y'
            ).value
        )

        radius = float(
            self.get_parameter(
                'circle_radius'
            ).value
        )

        start_angle = float(
            self.get_parameter(
                'circle_start_angle'
            ).value
        )

        end_angle = float(
            self.get_parameter(
                'circle_end_angle'
            ).value
        )

        arc_angle = abs(
            end_angle
            - start_angle
        )

        arc_length = (
            radius
            * arc_angle
        )

        number_of_segments = max(
            1,
            math.ceil(
                arc_length
                / self.spacing
            )
        )

        points = []

        for i in range(
            number_of_segments + 1
        ):

            ratio = (
                i / number_of_segments
            )

            angle = (
                start_angle
                +
                ratio
                * (
                    end_angle
                    - start_angle
                )
            )

            # WORLD NED:
            #
            # x = North
            # y = East
            #
            # angle 0 = North
            # angle +pi/2 = East

            x = (
                center_x
                + radius
                * math.cos(angle)
            )

            y = (
                center_y
                + radius
                * math.sin(angle)
            )

            points.append(
                (x, y)
            )

        return points

    # -------------------------------------------------------------
    # NED path heading
    # -------------------------------------------------------------

    def calculate_heading(
        self,
        points,
        index
    ):

        if len(points) < 2:
            return 0.0

        if index < len(points) - 1:

            x0, y0 = points[index]
            x1, y1 = points[index + 1]

        else:

            x0, y0 = points[index - 1]
            x1, y1 = points[index]

        north_delta = (
            x1 - x0
        )

        east_delta = (
            y1 - y0
        )

        # NED heading:
        #
        # atan2(East, North)
        #
        # 0 = North
        # +pi/2 = East

        return math.atan2(
            east_delta,
            north_delta
        )

    # -------------------------------------------------------------
    # ROS Path publication
    # -------------------------------------------------------------

    def publish_path(self):

        if (
            self.trajectory_type
            == 'straight'
        ):

            points = (
                self.generate_straight_path()
            )

        elif self.trajectory_type in [
            'circle',
            'curved'
        ]:

            points = (
                self.generate_circle_path()
            )

        else:

            self.get_logger().error(
                'Unknown trajectory_type: '
                f'{self.trajectory_type}'
            )

            return

        path_msg = Path()

        path_msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        path_msg.header.frame_id = (
            self.frame_id
        )

        for index, (x, y) in enumerate(
            points
        ):

            heading = (
                self.calculate_heading(
                    points,
                    index
                )
            )

            pose = PoseStamped()

            pose.header = (
                path_msg.header
            )

            pose.pose.position.x = x
            pose.pose.position.y = y

            # Surface vessel:
            # Down coordinate not used here.
            pose.pose.position.z = 0.0

            # NED heading quaternion.
            #
            # Because world_ned is right-handed:
            # +Z points Down.
            #
            # Positive yaw therefore corresponds
            # to clockwise heading when viewed
            # from above.

            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0

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

            path_msg.poses.append(
                pose
            )

        self.path_pub.publish(
            path_msg
        )


def main(args=None):

    rclpy.init(args=args)

    node = TrajectoryPlanner()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()