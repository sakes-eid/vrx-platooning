import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix

from platoon_interfaces.msg import VehicleState


EARTH_RADIUS = 6378137.0


def wrap_angle(angle):
    """
    Wrap angle to [-pi, pi].
    """
    return math.atan2(
        math.sin(angle),
        math.cos(angle)
    )


class VehicleStateNode(Node):

    def __init__(self):

        super().__init__('vehicle_state_node')

        # ---------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------

        self.declare_parameter(
            'origin_latitude',
            -33.724223
        )

        self.declare_parameter(
            'origin_longitude',
            150.679736
        )

        self.declare_parameter(
            'gps_topic',
            '/wamv/sensors/gps/gps/fix'
        )

        self.declare_parameter(
            'imu_topic',
            '/wamv/sensors/imu/imu/data'
        )

        self.declare_parameter(
            'world_position_topic',
            '/wamv/state/world_position'
        )

        self.declare_parameter(
            'vehicle_state_topic',
            '/wamv/state/vehicle'
        )

        self.declare_parameter(
            'velocity_filter_alpha',
            0.35
        )

        self.declare_parameter(
            'minimum_course_speed',
            0.05
        )

        # ---------------------------------------------------------
        # Read parameters
        # ---------------------------------------------------------

        self.origin_latitude = float(
            self.get_parameter(
                'origin_latitude'
            ).value
        )

        self.origin_longitude = float(
            self.get_parameter(
                'origin_longitude'
            ).value
        )

        self.gps_topic = self.get_parameter(
            'gps_topic'
        ).value

        self.imu_topic = self.get_parameter(
            'imu_topic'
        ).value

        self.world_position_topic = (
            self.get_parameter(
                'world_position_topic'
            ).value
        )

        self.vehicle_state_topic = (
            self.get_parameter(
                'vehicle_state_topic'
            ).value
        )

        self.alpha = float(
            self.get_parameter(
                'velocity_filter_alpha'
            ).value
        )

        self.minimum_course_speed = float(
            self.get_parameter(
                'minimum_course_speed'
            ).value
        )

        # ---------------------------------------------------------
        # Internal state
        # ---------------------------------------------------------

        self.x = 0.0
        self.y = 0.0

        self.vx = 0.0
        self.vy = 0.0

        self.speed = 0.0

        self.course_angle = 0.0
        self.body_yaw = 0.0

        self.previous_x = None
        self.previous_y = None
        self.previous_time = None

        self.velocity_initialized = False

        # ---------------------------------------------------------
        # Publishers
        # ---------------------------------------------------------

        self.position_pub = self.create_publisher(
            PointStamped,
            self.world_position_topic,
            10
        )

        self.state_pub = self.create_publisher(
            VehicleState,
            self.vehicle_state_topic,
            10
        )

        # ---------------------------------------------------------
        # Subscribers
        # ---------------------------------------------------------

        self.create_subscription(
            NavSatFix,
            self.gps_topic,
            self.gps_callback,
            10
        )

        self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            10
        )

        self.get_logger().info(
            'Vehicle state node started in WORLD NED frame.'
        )

        self.get_logger().info(
            f'World origin: '
            f'lat={self.origin_latitude:.6f}, '
            f'lon={self.origin_longitude:.6f}'
        )

        self.get_logger().info(
            'Convention: x=North, y=East, '
            'heading 0=North, +pi/2=East'
        )

    # -------------------------------------------------------------
    # GPS -> world NED
    # -------------------------------------------------------------

    def gps_to_ned(
        self,
        latitude,
        longitude
    ):

        origin_lat_rad = math.radians(
            self.origin_latitude
        )

        delta_lat = math.radians(
            latitude
            - self.origin_latitude
        )

        delta_lon = math.radians(
            longitude
            - self.origin_longitude
        )

        # NED horizontal coordinates
        north = (
            EARTH_RADIUS
            * delta_lat
        )

        east = (
            EARTH_RADIUS
            * math.cos(origin_lat_rad)
            * delta_lon
        )

        return north, east

    # -------------------------------------------------------------
    # GPS callback
    # -------------------------------------------------------------

    def gps_callback(self, msg):

        if (
            not math.isfinite(msg.latitude)
            or
            not math.isfinite(msg.longitude)
        ):
            return

        self.x, self.y = self.gps_to_ned(
            msg.latitude,
            msg.longitude
        )

        current_time = (
            float(msg.header.stamp.sec)
            +
            float(msg.header.stamp.nanosec)
            * 1e-9
        )

        # ---------------------------------------------------------
        # GPS-derived world NED velocity
        # ---------------------------------------------------------

        if (
            self.previous_x is not None
            and
            self.previous_y is not None
            and
            self.previous_time is not None
        ):

            dt = (
                current_time
                - self.previous_time
            )

            if dt > 1e-6:

                raw_vx = (
                    self.x
                    - self.previous_x
                ) / dt

                raw_vy = (
                    self.y
                    - self.previous_y
                ) / dt

                if not self.velocity_initialized:

                    self.vx = raw_vx
                    self.vy = raw_vy

                    self.velocity_initialized = True

                else:

                    self.vx = (
                        self.alpha
                        * raw_vx
                        +
                        (1.0 - self.alpha)
                        * self.vx
                    )

                    self.vy = (
                        self.alpha
                        * raw_vy
                        +
                        (1.0 - self.alpha)
                        * self.vy
                    )

                self.speed = math.hypot(
                    self.vx,
                    self.vy
                )

                # NED course:
                #
                # atan2(East, North)
                #
                # North = 0
                # East = +pi/2
                # South = +/-pi
                # West = -pi/2

                if (
                    self.speed
                    >= self.minimum_course_speed
                ):

                    self.course_angle = wrap_angle(
                        math.atan2(
                            self.vy,
                            self.vx
                        )
                    )

        self.previous_x = self.x
        self.previous_y = self.y
        self.previous_time = current_time

        # ---------------------------------------------------------
        # World position publication
        # ---------------------------------------------------------

        point_msg = PointStamped()

        point_msg.header.stamp = (
            msg.header.stamp
        )

        point_msg.header.frame_id = (
            'world_ned'
        )

        point_msg.point.x = self.x
        point_msg.point.y = self.y

        # Surface vehicle:
        # down coordinate is not currently used.
        point_msg.point.z = 0.0

        self.position_pub.publish(
            point_msg
        )

        # ---------------------------------------------------------
        # Unified vehicle state publication
        # ---------------------------------------------------------

        state_msg = VehicleState()

        state_msg.header.stamp = (
            msg.header.stamp
        )

        state_msg.header.frame_id = (
            'world_ned'
        )

        state_msg.x = self.x
        state_msg.y = self.y

        state_msg.vx = self.vx
        state_msg.vy = self.vy

        state_msg.speed = self.speed

        state_msg.course_angle = (
            self.course_angle
        )

        state_msg.body_yaw = (
            self.body_yaw
        )

        self.state_pub.publish(
            state_msg
        )

    # -------------------------------------------------------------
    # IMU callback
    # -------------------------------------------------------------

    def imu_callback(self, msg):

        q = msg.orientation

        # Gazebo / ROS yaw in ENU:
        #
        # 0 = East
        # +pi/2 = North

        siny_cosp = (
            2.0
            * (
                q.w * q.z
                +
                q.x * q.y
            )
        )

        cosy_cosp = (
            1.0
            -
            2.0
            * (
                q.y * q.y
                +
                q.z * q.z
            )
        )

        yaw_enu = math.atan2(
            siny_cosp,
            cosy_cosp
        )

        # ENU yaw -> NED heading
        #
        # heading_NED = pi/2 - yaw_ENU

        self.body_yaw = wrap_angle(
            (math.pi / 2.0)
            - yaw_enu
        )


def main(args=None):

    rclpy.init(args=args)

    node = VehicleStateNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()