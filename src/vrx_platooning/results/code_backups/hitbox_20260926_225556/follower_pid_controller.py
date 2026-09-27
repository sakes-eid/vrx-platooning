import math

import rclpy
from rclpy.node import Node

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
)


def wrap_pi(value):

    return math.atan2(
        math.sin(value),
        math.cos(value),
    )


def clamp(
    value,
    low,
    high,
):

    return max(
        low,
        min(
            high,
            value,
        ),
    )


class FollowerPidController(Node):

    def __init__(self):

        super().__init__(
            'follower_pid_controller'
        )

        params = {

            'heading_kp':
                911.4007441588406,

            'heading_ki':
                25.65165107782674,

            'heading_kd':
                98.10493190966012,

            'speed_kp':
                127.83877941647718,

            'speed_ki':
                18.383775431748894,

            'speed_kd':
                49.388692087475725,

            'brake_kp':
                177.8935198208073,

            'brake_ki':
                60.634764417119634,

            'brake_kd':
                10.647182804717701,

            'lookahead_distance':
                5.103256211030906,

            'distance_kp':
                0.35,

            'distance_ki':
                0.02,

            'distance_kd':
                0.10,

            'formation_distance':
                5.0,

            'catchup_distance':
                6.0,

            'catchup_min_speed':
                1.20,

            'follow_max_speed':
                1.50,

            'max_distance_speed_correction':
                0.70,

            'collision_brake_distance':
                4.25,

            'parking_speed':
                0.70,

            'goal_tolerance':
                1.0,

            'stop_speed_tolerance':
                0.05,

            'parking_hold_time':
                1.0,

            'max_forward_thrust':
                700.0,

            'max_turn_thrust':
                400.0,

            'max_total_thrust':
                1000.0,

            'max_brake_thrust':
                500.0,
        }

        for name, default in params.items():

            self.declare_parameter(
                name,
                default,
            )

        for name in params:

            setattr(
                self,
                name,
                float(
                    self.get_parameter(
                        name
                    ).value
                ),
            )

        self.r1 = None
        self.r2 = None

        self.path = []

        self.path_source = 'NONE'

        self.mode = 'FOLLOW'
        self.previous_mode = None

        self.last_time = None

        self.heading_integral = 0.0
        self.heading_previous = 0.0

        self.speed_integral = 0.0
        self.speed_previous = 0.0

        self.distance_integral = 0.0
        self.distance_previous = 0.0

        self.brake_integral = 0.0
        self.brake_previous = 0.0

        self.parking_hold_start = None

        self.success_latched = False

        qos = QoSProfile(
            depth=1
        )

        qos.durability = (
            DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.path_qos = qos

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
            String,
            '/platoon/mission_state',
            self.mode_callback,
            10,
        )

        # =====================================================
        # IMPORTANT:
        # R2 begins subscribed ONLY to breadcrumbs.
        # =====================================================

        self.breadcrumb_subscription = (
            self.create_subscription(
                Path,
                '/planner/r2/breadcrumb_path',
                self.breadcrumb_callback,
                qos,
            )
        )

        self.parking_subscription = None

        self.left_pub = self.create_publisher(
            Float64,
            '/wamv2/thrusters/left/thrust',
            10,
        )

        self.right_pub = self.create_publisher(
            Float64,
            '/wamv2/thrusters/right/thrust',
            10,
        )

        self.left_pos_pub = self.create_publisher(
            Float64,
            '/wamv2/thrusters/left/pos',
            10,
        )

        self.right_pos_pub = self.create_publisher(
            Float64,
            '/wamv2/thrusters/right/pos',
            10,
        )

        self.success_pub = self.create_publisher(
            Bool,
            '/r2/success',
            qos,
        )

        self.path_source_pub = (
            self.create_publisher(
                String,
                '/r2/control/path_source',
                10,
            )
        )

        self.path_error_pub = (
            self.create_publisher(
                Float64,
                '/r2/control/path_error',
                10,
            )
        )

        self.heading_error_pub = (
            self.create_publisher(
                Float64,
                '/r2/control/heading_error',
                10,
            )
        )

        self.speed_error_pub = (
            self.create_publisher(
                Float64,
                '/r2/control/speed_error',
                10,
            )
        )

        self.target_speed_pub = (
            self.create_publisher(
                Float64,
                '/r2/control/target_speed',
                10,
            )
        )

        self.timer = self.create_timer(
            0.1,
            self.control_loop,
        )

        self.get_logger().info(
            'R2 follower controller ready.'
        )

        self.get_logger().info(
            'Subscribed to BREADCRUMB path only.'
        )


    def r1_callback(self, msg):

        self.r1 = msg


    def r2_callback(self, msg):

        self.r2 = msg


    def breadcrumb_callback(self, msg):

        if self.mode != 'FOLLOW':
            return

        self.path = [
            (
                p.pose.position.x,
                p.pose.position.y,
            )
            for p in msg.poses
        ]

        self.path_source = (
            'BREADCRUMB'
        )


    def parking_callback(self, msg):

        if self.mode != 'PARK':
            return

        self.path = [
            (
                p.pose.position.x,
                p.pose.position.y,
            )
            for p in msg.poses
        ]

        self.path_source = (
            'PARKING'
        )


    def destroy_breadcrumb_subscription(
        self,
    ):

        if (
            self.breadcrumb_subscription
            is not None
        ):

            self.destroy_subscription(
                self.breadcrumb_subscription
            )

            self.breadcrumb_subscription = (
                None
            )

            self.get_logger().info(
                'UNSUBSCRIBED from breadcrumb path.'
            )


    def create_parking_subscription(
        self,
    ):

        if (
            self.parking_subscription
            is None
        ):

            self.parking_subscription = (
                self.create_subscription(
                    Path,
                    '/planner/r2/parking_path',
                    self.parking_callback,
                    self.path_qos,
                )
            )

            self.get_logger().info(
                'SUBSCRIBED to parking path.'
            )


    def mode_callback(self, msg):

        new_mode = msg.data

        if new_mode == self.mode:
            return

        self.get_logger().info(
            f'R2 mode -> {new_mode}'
        )

        # =====================================================
        # FOLLOW -> PARK_BRAKE
        #
        # Permanently destroy breadcrumb subscription.
        # =====================================================

        if new_mode == 'PARK_BRAKE':

            self.destroy_breadcrumb_subscription()

            self.path = []

            self.path_source = 'NONE'

        # =====================================================
        # PARK_BRAKE -> PARK
        #
        # Only now create parking subscription.
        # =====================================================

        elif new_mode == 'PARK':

            self.path = []

            self.path_source = 'NONE'

            self.create_parking_subscription()

        self.mode = new_mode

        self.reset_pid()


    def reset_pid(self):

        self.heading_integral = 0.0
        self.heading_previous = 0.0

        self.speed_integral = 0.0
        self.speed_previous = 0.0

        self.distance_integral = 0.0
        self.distance_previous = 0.0

        self.brake_integral = 0.0
        self.brake_previous = 0.0


    def publish_thrusters(
        self,
        left,
        right,
    ):

        left = clamp(
            left,
            -self.max_total_thrust,
            self.max_total_thrust,
        )

        right = clamp(
            right,
            -self.max_total_thrust,
            self.max_total_thrust,
        )

        l = Float64()
        l.data = left

        r = Float64()
        r.data = right

        pos = Float64()
        pos.data = 0.0

        self.left_pub.publish(l)
        self.right_pub.publish(r)

        self.left_pos_pub.publish(pos)
        self.right_pos_pub.publish(pos)


    def stop(self):

        self.publish_thrusters(
            0.0,
            0.0,
        )


    def nearest_index(self):

        if (
            self.r2 is None
            or not self.path
        ):

            return None

        return min(
            range(len(self.path)),
            key=lambda i:
                math.hypot(
                    self.path[i][0]
                    - self.r2.x,

                    self.path[i][1]
                    - self.r2.y,
                )
        )


    def lookahead_target(self):

        index = self.nearest_index()

        if index is None:
            return None

        travelled = 0.0

        for i in range(
            index,
            len(self.path) - 1,
        ):

            a = self.path[i]
            b = self.path[i + 1]

            travelled += math.hypot(
                b[0] - a[0],
                b[1] - a[1],
            )

            if (
                travelled
                >= self.lookahead_distance
            ):

                return b

        return self.path[-1]


    def endpoint_distance(self):

        if (
            self.r2 is None
            or not self.path
        ):

            return float('inf')

        target = self.path[-1]

        return math.hypot(
            target[0] - self.r2.x,
            target[1] - self.r2.y,
        )


    def distance_pid(self, dt):

        d12 = math.hypot(
            self.r1.x - self.r2.x,
            self.r1.y - self.r2.y,
        )

        error = (
            d12
            - self.formation_distance
        )

        self.distance_integral = clamp(
            self.distance_integral
            + error * dt,
            -10.0,
            10.0,
        )

        derivative = (
            error
            - self.distance_previous
        ) / max(dt, 1e-6)

        self.distance_previous = error

        output = (
            self.distance_kp * error
            + self.distance_ki
            * self.distance_integral
            + self.distance_kd
            * derivative
        )

        output = clamp(
            output,
            -self.max_distance_speed_correction,
            self.max_distance_speed_correction,
        )

        return (
            d12,
            output,
        )


    def brake(self, dt):

        surge = (
            self.r2.vx
            * math.cos(
                self.r2.body_yaw
            )

            + self.r2.vy
            * math.sin(
                self.r2.body_yaw
            )
        )

        error = -surge

        self.brake_integral = clamp(
            self.brake_integral
            + error * dt,
            -1.0,
            1.0,
        )

        derivative = (
            error
            - self.brake_previous
        ) / max(dt, 1e-6)

        self.brake_previous = error

        command = (
            self.brake_kp * error
            + self.brake_ki
            * self.brake_integral
            + self.brake_kd
            * derivative
        )

        command = clamp(
            command,
            -self.max_brake_thrust,
            self.max_brake_thrust,
        )

        # Equal commands:
        # braking WITHOUT turning.
        self.publish_thrusters(
            command,
            command,
        )


    def track_path(
        self,
        requested_speed,
        dt,
    ):

        target = self.lookahead_target()

        if target is None:

            self.stop()

            return

        desired_heading = math.atan2(
            target[1] - self.r2.y,
            target[0] - self.r2.x,
        )

        heading_error = wrap_pi(
            desired_heading
            - self.r2.body_yaw
        )

        self.heading_integral = clamp(
            self.heading_integral
            + heading_error * dt,
            -1.5,
            1.5,
        )

        derivative = (
            heading_error
            - self.heading_previous
        ) / max(dt, 1e-6)

        self.heading_previous = (
            heading_error
        )

        turn = (
            self.heading_kp
            * heading_error

            + self.heading_ki
            * self.heading_integral

            + self.heading_kd
            * derivative
        )

        turn = clamp(
            turn,
            -self.max_turn_thrust,
            self.max_turn_thrust,
        )

        alignment = max(
            0.15,
            1.0
            - abs(
                heading_error
            ) / 1.1,
        )

        target_speed = (
            requested_speed
            * alignment
        )

        speed_error = (
            target_speed
            - self.r2.speed
        )

        self.speed_integral = clamp(
            self.speed_integral
            + speed_error * dt,
            -3.0,
            3.0,
        )

        derivative = (
            speed_error
            - self.speed_previous
        ) / max(dt, 1e-6)

        self.speed_previous = speed_error

        forward = (
            self.speed_kp * speed_error
            + self.speed_ki
            * self.speed_integral
            + self.speed_kd
            * derivative
        )

        forward = clamp(
            forward,
            0.0,
            self.max_forward_thrust,
        )

        self.publish_thrusters(
            forward + turn,
            forward - turn,
        )

        msg = Float64()
        msg.data = heading_error

        self.heading_error_pub.publish(msg)

        msg = Float64()
        msg.data = speed_error

        self.speed_error_pub.publish(msg)

        msg = Float64()
        msg.data = target_speed

        self.target_speed_pub.publish(msg)


    def control_loop(self):

        now = self.get_clock().now()

        source = String()
        source.data = self.path_source

        self.path_source_pub.publish(
            source
        )

        if self.last_time is None:

            self.last_time = now

            return

        dt = (
            now
            - self.last_time
        ).nanoseconds * 1e-9

        self.last_time = now

        if (
            dt <= 0.0
            or self.r2 is None
        ):

            return

        # =====================================================
        # FOLLOW
        # =====================================================

        if self.mode == 'FOLLOW':

            if (
                self.r1 is None
                or not self.path
            ):

                self.stop()

                return

            d12, correction = (
                self.distance_pid(dt)
            )

            if (
                d12
                <= self.collision_brake_distance
            ):

                self.brake(dt)

                return

            target_speed = (
                self.r1.speed
                + correction
            )

            if (
                d12
                >= self.catchup_distance
            ):

                target_speed = max(
                    target_speed,
                    self.catchup_min_speed,
                )

            target_speed = clamp(
                target_speed,
                0.0,
                self.follow_max_speed,
            )

            self.track_path(
                target_speed,
                dt,
            )

            return

        # =====================================================
        # PARK_BRAKE
        # =====================================================

        if self.mode == 'PARK_BRAKE':

            self.path = []

            self.path_source = 'NONE'

            if (
                self.r2.speed
                > self.stop_speed_tolerance
            ):

                self.brake(dt)

            else:

                self.stop()

            return

        # =====================================================
        # PARK
        # =====================================================

        if self.mode == 'PARK':

            if not self.path:

                self.stop()

                return

            remaining = (
                self.endpoint_distance()
            )

            if (
                remaining
                <= self.goal_tolerance
            ):

                if (
                    self.r2.speed
                    > self.stop_speed_tolerance
                ):

                    self.brake(dt)

                    self.parking_hold_start = (
                        None
                    )

                else:

                    self.stop()

                    if (
                        self.parking_hold_start
                        is None
                    ):

                        self.parking_hold_start = (
                            now
                        )

                    held = (
                        now
                        - self.parking_hold_start
                    ).nanoseconds * 1e-9

                    if (
                        held
                        >= self.parking_hold_time
                    ):

                        self.success_latched = (
                            True
                        )

            else:

                target_speed = min(
                    self.parking_speed,
                    max(
                        0.20,
                        remaining * 0.15,
                    ),
                )

                self.track_path(
                    target_speed,
                    dt,
                )

        elif self.mode in (
            'DWELL',
            'SUCCESS',
        ):

            self.stop()

        if self.success_latched:

            msg = Bool()
            msg.data = True

            self.success_pub.publish(msg)


def main():

    rclpy.init()

    node = FollowerPidController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        try:
            node.stop()

        except Exception:
            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
