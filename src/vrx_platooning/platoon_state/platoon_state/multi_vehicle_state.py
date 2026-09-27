import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, Imu

from platoon_interfaces.msg import VehicleState


EARTH_RADIUS_M = 6378137.0


def wrap_pi(value):
    return math.atan2(
        math.sin(value),
        math.cos(value),
    )


class MultiVehicleState(Node):

    def __init__(self):
        super().__init__('multi_vehicle_state')

        self.declare_parameter('vehicle_id', 'r1')
        self.declare_parameter(
            'gps_topic',
            '/wamv/sensors/gps/gps/fix'
        )
        self.declare_parameter(
            'imu_topic',
            '/wamv/sensors/imu/imu/data'
        )
        self.declare_parameter(
            'state_topic',
            '/r1/vehicle_state'
        )

        # Sydney Regatta common geographic origin.
        self.declare_parameter(
            'origin_latitude_deg',
            -33.724223
        )
        self.declare_parameter(
            'origin_longitude_deg',
            150.679736
        )

        self.declare_parameter(
            'velocity_filter_alpha',
            0.35
        )

        self.vehicle_id = str(
            self.get_parameter(
                'vehicle_id'
            ).value
        )

        self.gps_topic = str(
            self.get_parameter(
                'gps_topic'
            ).value
        )

        self.imu_topic = str(
            self.get_parameter(
                'imu_topic'
            ).value
        )

        self.state_topic = str(
            self.get_parameter(
                'state_topic'
            ).value
        )

        self.lat0 = math.radians(
            float(
                self.get_parameter(
                    'origin_latitude_deg'
                ).value
            )
        )

        self.lon0 = math.radians(
            float(
                self.get_parameter(
                    'origin_longitude_deg'
                ).value
            )
        )

        self.alpha = float(
            self.get_parameter(
                'velocity_filter_alpha'
            ).value
        )

        self.body_yaw = 0.0
        self.have_imu = False

        self.previous_x = None
        self.previous_y = None
        self.previous_time = None

        self.vx = 0.0
        self.vy = 0.0

        self.publisher = self.create_publisher(
            VehicleState,
            self.state_topic,
            10,
        )

        self.create_subscription(
            NavSatFix,
            self.gps_topic,
            self.gps_callback,
            20,
        )

        self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            50,
        )

        self.get_logger().info(
            f'{self.vehicle_id} GPS state estimator started'
        )
        self.get_logger().info(
            f'GPS   : {self.gps_topic}'
        )
        self.get_logger().info(
            f'IMU   : {self.imu_topic}'
        )
        self.get_logger().info(
            f'State : {self.state_topic}'
        )
        self.get_logger().info(
            'Frame : world_ned'
        )


    def imu_callback(self, msg):
        q = msg.orientation

        siny_cosp = 2.0 * (
            q.w * q.z
            + q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y
            + q.z * q.z
        )

        yaw_enu = math.atan2(
            siny_cosp,
            cosy_cosp,
        )

        # ENU:
        # yaw=0 East, +pi/2 North
        #
        # NED required by project:
        # yaw=0 North, +pi/2 East
        self.body_yaw = wrap_pi(
            math.pi / 2.0
            - yaw_enu
        )

        self.have_imu = True


    def gps_to_ned(self, latitude, longitude):
        lat = math.radians(latitude)
        lon = math.radians(longitude)

        north = (
            EARTH_RADIUS_M
            * (lat - self.lat0)
        )

        east = (
            EARTH_RADIUS_M
            * math.cos(self.lat0)
            * (lon - self.lon0)
        )

        return north, east


    def gps_callback(self, msg):
        if (
            not math.isfinite(msg.latitude)
            or not math.isfinite(msg.longitude)
        ):
            return

        x, y = self.gps_to_ned(
            msg.latitude,
            msg.longitude,
        )

        now = self.get_clock().now()

        if (
            self.previous_time is not None
            and self.previous_x is not None
        ):
            dt = (
                now - self.previous_time
            ).nanoseconds * 1e-9

            if 0.001 < dt < 1.0:
                raw_vx = (
                    x - self.previous_x
                ) / dt

                raw_vy = (
                    y - self.previous_y
                ) / dt

                a = self.alpha

                self.vx = (
                    a * raw_vx
                    + (1.0 - a) * self.vx
                )

                self.vy = (
                    a * raw_vy
                    + (1.0 - a) * self.vy
                )

        self.previous_x = x
        self.previous_y = y
        self.previous_time = now

        speed = math.hypot(
            self.vx,
            self.vy,
        )

        if speed > 0.05:
            course = math.atan2(
                self.vy,
                self.vx,
            )
        else:
            course = self.body_yaw

        state = VehicleState()

        state.header.stamp = now.to_msg()
        state.header.frame_id = 'world_ned'

        state.x = float(x)
        state.y = float(y)
        state.vx = float(self.vx)
        state.vy = float(self.vy)
        state.speed = float(speed)
        state.course_angle = float(
            wrap_pi(course)
        )
        state.body_yaw = float(
            self.body_yaw
        )

        self.publisher.publish(state)


def main():
    rclpy.init()

    node = MultiVehicleState()

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
