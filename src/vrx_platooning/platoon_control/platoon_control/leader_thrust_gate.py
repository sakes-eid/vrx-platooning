import rclpy
from rclpy.node import Node

from std_msgs.msg import (
    Bool,
    Float64,
)


class LeaderThrustGate(Node):

    def __init__(self):
        super().__init__(
            'leader_thrust_gate'
        )

        self.left = 0.0
        self.right = 0.0

        self.finished = False

        self.create_subscription(
            Float64,
            '/r1/raw/left_thrust',
            self.left_callback,
            10,
        )

        self.create_subscription(
            Float64,
            '/r1/raw/right_thrust',
            self.right_callback,
            10,
        )

        self.create_subscription(
            Bool,
            '/r1/mission_complete_latched',
            self.done_callback,
            10,
        )

        self.left_pub = (
            self.create_publisher(
                Float64,
                '/wamv/thrusters/left/thrust',
                10,
            )
        )

        self.right_pub = (
            self.create_publisher(
                Float64,
                '/wamv/thrusters/right/thrust',
                10,
            )
        )

        self.left_pos_pub = (
            self.create_publisher(
                Float64,
                '/wamv/thrusters/left/pos',
                10,
            )
        )

        self.right_pos_pub = (
            self.create_publisher(
                Float64,
                '/wamv/thrusters/right/pos',
                10,
            )
        )

        self.timer = self.create_timer(
            0.05,
            self.publish,
        )

        self.get_logger().info(
            'R1 thrust gate started.'
        )


    def left_callback(
        self,
        msg,
    ):
        if not self.finished:
            self.left = float(
                msg.data
            )


    def right_callback(
        self,
        msg,
    ):
        if not self.finished:
            self.right = float(
                msg.data
            )


    def done_callback(
        self,
        msg,
    ):
        if msg.data and not self.finished:
            self.finished = True

            self.left = 0.0
            self.right = 0.0

            self.get_logger().info(
                'R1 completion latched: '
                'thrusters permanently gated to zero.'
            )


    def publish(self):
        left = Float64()
        right = Float64()
        pos = Float64()

        if self.finished:
            left.data = 0.0
            right.data = 0.0
        else:
            left.data = self.left
            right.data = self.right

        pos.data = 0.0

        self.left_pub.publish(left)
        self.right_pub.publish(right)

        self.left_pos_pub.publish(pos)
        self.right_pos_pub.publish(pos)


def main():
    rclpy.init()

    node = LeaderThrustGate()

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
