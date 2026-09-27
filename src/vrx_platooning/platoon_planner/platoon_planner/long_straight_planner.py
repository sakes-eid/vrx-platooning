import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    ReliabilityPolicy,
)


class LongStraightPlanner(Node):

    def __init__(self):

        super().__init__('long_straight_planner')

        self.declare_parameter('start_north', 163.0)
        self.declare_parameter('start_east', -531.0)

        self.declare_parameter('end_north', 263.0)
        self.declare_parameter('end_east', -531.0)

        self.declare_parameter('spacing', 0.5)

        self.start = (
            float(self.get_parameter('start_north').value),
            float(self.get_parameter('start_east').value),
        )

        self.end = (
            float(self.get_parameter('end_north').value),
            float(self.get_parameter('end_east').value),
        )

        self.spacing = float(
            self.get_parameter('spacing').value
        )

        qos = QoSProfile(depth=1)

        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.publisher = self.create_publisher(
            Path,
            '/planner/reference_path',
            qos,
        )

        self.path = self.create_path()

        self.timer = self.create_timer(
            0.5,
            self.publish_path,
        )

        self.published = False

        self.get_logger().info(
            '100 m R1 planner ready.'
        )


    def create_path(self):

        length = math.hypot(
            self.end[0] - self.start[0],
            self.end[1] - self.start[1],
        )

        count = max(
            1,
            int(math.ceil(length / self.spacing))
        )

        heading = math.atan2(
            self.end[1] - self.start[1],
            self.end[0] - self.start[0],
        )

        msg = Path()

        msg.header.frame_id = 'world_ned'

        for i in range(count + 1):

            ratio = i / count

            north = (
                self.start[0]
                + ratio
                * (
                    self.end[0]
                    - self.start[0]
                )
            )

            east = (
                self.start[1]
                + ratio
                * (
                    self.end[1]
                    - self.start[1]
                )
            )

            pose = PoseStamped()

            pose.header.frame_id = 'world_ned'

            pose.pose.position.x = north
            pose.pose.position.y = east

            pose.pose.orientation.z = math.sin(
                heading / 2.0
            )

            pose.pose.orientation.w = math.cos(
                heading / 2.0
            )

            msg.poses.append(pose)

        return msg


    def publish_path(self):

        if self.published:
            return

        stamp = self.get_clock().now().to_msg()

        self.path.header.stamp = stamp

        for pose in self.path.poses:
            pose.header.stamp = stamp

        self.publisher.publish(self.path)

        self.published = True

        self.timer.cancel()

        self.get_logger().info(
            f'Published {len(self.path.poses)} waypoints.'
        )


def main():

    rclpy.init()

    node = LongStraightPlanner()

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
