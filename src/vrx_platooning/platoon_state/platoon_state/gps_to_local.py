import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped


class GpsToLocal(Node):

    def __init__(self):
        super().__init__('gps_to_local')

        # Parameters
        self.declare_parameter('origin_latitude', -33.72275000)
        self.declare_parameter('origin_longitude', 150.67400000)

        self.declare_parameter(
            'gps_topic',
            '/wamv/sensors/gps/gps/fix'
        )

        self.declare_parameter(
            'local_position_topic',
            '/wamv/state/local_position'
        )

        self.origin_lat = (
            self.get_parameter('origin_latitude')
            .get_parameter_value()
            .double_value
        )

        self.origin_lon = (
            self.get_parameter('origin_longitude')
            .get_parameter_value()
            .double_value
        )

        gps_topic = (
            self.get_parameter('gps_topic')
            .get_parameter_value()
            .string_value
        )

        output_topic = (
            self.get_parameter('local_position_topic')
            .get_parameter_value()
            .string_value
        )

        self.publisher = self.create_publisher(
            PointStamped,
            output_topic,
            10
        )

        self.subscription = self.create_subscription(
            NavSatFix,
            gps_topic,
            self.gps_callback,
            10
        )

        self.get_logger().info(
            f'Fixed GPS origin: '
            f'lat={self.origin_lat:.8f}, '
            f'lon={self.origin_lon:.8f}'
        )

        self.get_logger().info(
            f'GPS input: {gps_topic}'
        )

        self.get_logger().info(
            f'Local position output: {output_topic}'
        )

    def gps_callback(self, msg):

        lat = msg.latitude
        lon = msg.longitude

        # Mean Earth radius [m]
        earth_radius = 6378137.0

        lat0_rad = math.radians(self.origin_lat)

        delta_lat = math.radians(
            lat - self.origin_lat
        )

        delta_lon = math.radians(
            lon - self.origin_lon
        )

        # Local tangent-plane approximation
        # x = East
        # y = North
        x = (
            earth_radius
            * delta_lon
            * math.cos(lat0_rad)
        )

        y = earth_radius * delta_lat

        local_msg = PointStamped()

        # Preserve GPS timestamp
        local_msg.header.stamp = msg.header.stamp
        local_msg.header.frame_id = 'local_map'

        local_msg.point.x = x
        local_msg.point.y = y

        # Altitude is deliberately not being used for 2D navigation.
        local_msg.point.z = 0.0

        self.publisher.publish(local_msg)


def main(args=None):

    rclpy.init(args=args)

    node = GpsToLocal()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
