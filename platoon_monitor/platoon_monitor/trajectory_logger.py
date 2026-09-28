import csv
import math
import os

import rclpy

from rclpy.executors import (
    ExternalShutdownException,
)

from rclpy.node import Node

from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from nav_msgs.msg import Path

from std_msgs.msg import (
    Bool,
    Float64,
    String,
)

from platoon_interfaces.msg import VehicleState


# =============================================================
# Utilities
# =============================================================


def wrap_angle(angle):

    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


# =============================================================
# Trajectory logger
# =============================================================


class TrajectoryLogger(Node):

    def __init__(self):

        super().__init__(
            'trajectory_logger'
        )

        # =====================================================
        # Parameters
        # =====================================================

        self.declare_parameter(
            'output_file',
            'trajectory.csv'
        )

        self.declare_parameter(
            'state_topic',
            '/wamv/state/vehicle'
        )

        self.declare_parameter(
            'path_topic',
            '/planner/reference_path'
        )

        self.declare_parameter(
            'left_thrust_topic',
            '/wamv/thrusters/left/thrust'
        )

        self.declare_parameter(
            'right_thrust_topic',
            '/wamv/thrusters/right/thrust'
        )

        self.declare_parameter(
            'controller_state_topic',
            '/experiment/controller_state'
        )

        self.declare_parameter(
            'success_topic',
            '/experiment/success'
        )

        self.declare_parameter(
            'lookahead_distance',
            3.0
        )

        self.declare_parameter(
            'target_speed',
            1.0
        )

        # =====================================================
        # Read parameters
        # =====================================================

        self.output_file = os.path.expanduser(
            self.get_parameter(
                'output_file'
            ).value
        )

        self.state_topic = (
            self.get_parameter(
                'state_topic'
            ).value
        )

        self.path_topic = (
            self.get_parameter(
                'path_topic'
            ).value
        )

        self.left_thrust_topic = (
            self.get_parameter(
                'left_thrust_topic'
            ).value
        )

        self.right_thrust_topic = (
            self.get_parameter(
                'right_thrust_topic'
            ).value
        )

        self.controller_state_topic = (
            self.get_parameter(
                'controller_state_topic'
            ).value
        )

        self.success_topic = (
            self.get_parameter(
                'success_topic'
            ).value
        )

        self.lookahead_distance = float(
            self.get_parameter(
                'lookahead_distance'
            ).value
        )

        self.target_speed = float(
            self.get_parameter(
                'target_speed'
            ).value
        )

        # =====================================================
        # Runtime state
        # =====================================================

        self.vehicle_state = None

        self.path = []

        self.left_thrust = 0.0
        self.right_thrust = 0.0

        self.controller_state = 'UNKNOWN'

        self.start_time = None

        self.last_sample_time_ns = None

        self.finished = False

        self.shutdown_timer = None

        # =====================================================
        # Summary data
        # =====================================================

        self.position_errors = []

        self.heading_errors = []

        self.speeds = []

        self.sample_count = 0

        # =====================================================
        # CSV
        # =====================================================

        output_directory = os.path.dirname(
            self.output_file
        )

        if output_directory:

            os.makedirs(
                output_directory,
                exist_ok=True
            )

        self.csv_file = open(
            self.output_file,
            'w',
            newline='',
            encoding='utf-8',
        )

        self.csv_writer = csv.writer(
            self.csv_file
        )

        self.csv_writer.writerow(
            [
                'time_s',

                'north_m',
                'east_m',

                'body_yaw_rad',
                'course_angle_rad',

                'vx_mps',
                'vy_mps',
                'speed_mps',

                'reference_north_m',
                'reference_east_m',
                'position_error_m',

                'target_north_m',
                'target_east_m',

                'desired_heading_rad',
                'heading_error_rad',

                'speed_error_mps',

                'left_thrust',
                'right_thrust',

                'controller_state',
            ]
        )

        self.csv_file.flush()

        # =====================================================
        # QoS for latched experiment-state topics
        # =====================================================

        status_qos = QoSProfile(
            depth=1
        )

        status_qos.reliability = (
            ReliabilityPolicy.RELIABLE
        )

        status_qos.durability = (
            DurabilityPolicy.TRANSIENT_LOCAL
        )

        # =====================================================
        # Subscribers
        # =====================================================

        self.create_subscription(
            VehicleState,
            self.state_topic,
            self.state_callback,
            20,
        )

        self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            10,
        )

        self.create_subscription(
            Float64,
            self.left_thrust_topic,
            self.left_thrust_callback,
            10,
        )

        self.create_subscription(
            Float64,
            self.right_thrust_topic,
            self.right_thrust_callback,
            10,
        )

        self.create_subscription(
            String,
            self.controller_state_topic,
            self.controller_state_callback,
            status_qos,
        )

        self.create_subscription(
            Bool,
            self.success_topic,
            self.success_callback,
            status_qos,
        )

        self.get_logger().info(
            'Trajectory logger started.'
        )

        self.get_logger().info(
            f'CSV output: {self.output_file}'
        )

    # =========================================================
    # ROS callbacks
    # =========================================================

    def path_callback(
        self,
        message,
    ):

        if (
            message.header.frame_id
            and
            message.header.frame_id
            != 'world_ned'
        ):

            return

        self.path = [

            (
                float(
                    pose.pose.position.x
                ),

                float(
                    pose.pose.position.y
                ),
            )

            for pose in message.poses
        ]

    # ---------------------------------------------------------

    def left_thrust_callback(
        self,
        message,
    ):

        self.left_thrust = float(
            message.data
        )

    # ---------------------------------------------------------

    def right_thrust_callback(
        self,
        message,
    ):

        self.right_thrust = float(
            message.data
        )

    # ---------------------------------------------------------

    def controller_state_callback(
        self,
        message,
    ):

        self.controller_state = str(
            message.data
        )

    # ---------------------------------------------------------

    def state_callback(
        self,
        message,
    ):

        if self.finished:
            return

        self.vehicle_state = message

        self.record_sample()

    # ---------------------------------------------------------

    def success_callback(
        self,
        message,
    ):

        if not message.data:
            return

        if self.finished:
            return

        # Ensure the final CSV row explicitly represents
        # experiment completion.

        self.controller_state = (
            'SUCCESS'
        )

        self.record_sample(
            force=True
        )

        self.finish_logging(
            reason='SUCCESS'
        )

        # Give stdout / filesystem a brief simulated-time
        # opportunity to finish, then terminate this logger
        # process cleanly.

        self.shutdown_timer = (
            self.create_timer(
                0.1,
                self.shutdown_after_success,
            )
        )

    # =========================================================
    # Sample creation
    # =========================================================

    def record_sample(
        self,
        force=False,
    ):

        if self.finished:
            return

        if self.vehicle_state is None:
            return

        if not self.path:
            return

        now = self.get_clock().now()

        now_ns = now.nanoseconds

        if (
            not force
            and
            self.last_sample_time_ns
            == now_ns
        ):

            return

        self.last_sample_time_ns = (
            now_ns
        )

        if self.start_time is None:

            self.start_time = now

        time_s = (
            now
            - self.start_time
        ).nanoseconds / 1e9

        north = float(
            self.vehicle_state.x
        )

        east = float(
            self.vehicle_state.y
        )

        vx = float(
            self.vehicle_state.vx
        )

        vy = float(
            self.vehicle_state.vy
        )

        speed = math.hypot(
            vx,
            vy,
        )

        body_yaw = float(
            self.vehicle_state.body_yaw
        )

        course_angle = float(
            self.vehicle_state.course_angle
        )

        # -----------------------------------------------------
        # Nearest point on the reference path
        # -----------------------------------------------------

        (
            reference_north,
            reference_east,
            position_error,
        ) = self.nearest_point_on_path(
            north,
            east,
        )

        # -----------------------------------------------------
        # Lookahead target, reconstructed using the same basic
        # path logic as the controller.
        # -----------------------------------------------------

        (
            target_north,
            target_east,
        ) = self.find_lookahead_target(
            north,
            east,
        )

        desired_heading = math.atan2(
            target_east - east,
            target_north - north,
        )

        heading_error = wrap_angle(
            desired_heading
            - body_yaw
        )

        speed_error = (
            self.target_speed
            - speed
        )

        # -----------------------------------------------------
        # Write row
        # -----------------------------------------------------

        self.csv_writer.writerow(
            [
                time_s,

                north,
                east,

                body_yaw,
                course_angle,

                vx,
                vy,
                speed,

                reference_north,
                reference_east,
                position_error,

                target_north,
                target_east,

                desired_heading,
                heading_error,

                speed_error,

                self.left_thrust,
                self.right_thrust,

                self.controller_state,
            ]
        )

        self.csv_file.flush()

        # -----------------------------------------------------
        # Summary statistics
        # -----------------------------------------------------

        self.sample_count += 1

        self.position_errors.append(
            position_error
        )

        self.heading_errors.append(
            abs(
                heading_error
            )
        )

        self.speeds.append(
            speed
        )

    # =========================================================
    # Nearest point on path
    # =========================================================

    def nearest_point_on_path(
        self,
        north,
        east,
    ):

        if len(self.path) == 1:

            ref_north = (
                self.path[0][0]
            )

            ref_east = (
                self.path[0][1]
            )

            error = math.hypot(
                north - ref_north,
                east - ref_east,
            )

            return (
                ref_north,
                ref_east,
                error,
            )

        best_distance = float(
            'inf'
        )

        best_north = self.path[0][0]
        best_east = self.path[0][1]

        for index in range(
            len(self.path) - 1
        ):

            start_north = (
                self.path[index][0]
            )

            start_east = (
                self.path[index][1]
            )

            end_north = (
                self.path[index + 1][0]
            )

            end_east = (
                self.path[index + 1][1]
            )

            segment_north = (
                end_north
                - start_north
            )

            segment_east = (
                end_east
                - start_east
            )

            segment_length_squared = (
                segment_north
                * segment_north
                +
                segment_east
                * segment_east
            )

            if (
                segment_length_squared
                <= 0.0
            ):

                projection = 0.0

            else:

                projection = (
                    (
                        (
                            north
                            - start_north
                        )
                        * segment_north
                    )
                    +
                    (
                        (
                            east
                            - start_east
                        )
                        * segment_east
                    )
                ) / segment_length_squared

                projection = max(
                    0.0,
                    min(
                        1.0,
                        projection,
                    )
                )

            projected_north = (
                start_north
                +
                projection
                * segment_north
            )

            projected_east = (
                start_east
                +
                projection
                * segment_east
            )

            distance = math.hypot(
                north
                - projected_north,

                east
                - projected_east,
            )

            if distance < best_distance:

                best_distance = (
                    distance
                )

                best_north = (
                    projected_north
                )

                best_east = (
                    projected_east
                )

        return (
            best_north,
            best_east,
            best_distance,
        )

    # =========================================================
    # Lookahead target
    # =========================================================

    def find_lookahead_target(
        self,
        north,
        east,
    ):

        nearest_index = 0

        nearest_distance = float(
            'inf'
        )

        for index, waypoint in enumerate(
            self.path
        ):

            distance = math.hypot(
                waypoint[0] - north,
                waypoint[1] - east,
            )

            if distance < nearest_distance:

                nearest_distance = (
                    distance
                )

                nearest_index = (
                    index
                )

        accumulated_distance = 0.0

        target_index = (
            nearest_index
        )

        for index in range(
            nearest_index,
            len(self.path) - 1,
        ):

            segment_length = math.hypot(
                self.path[index + 1][0]
                - self.path[index][0],

                self.path[index + 1][1]
                - self.path[index][1],
            )

            accumulated_distance += (
                segment_length
            )

            target_index = (
                index + 1
            )

            if (
                accumulated_distance
                >= self.lookahead_distance
            ):

                break

        return self.path[
            target_index
        ]

    # =========================================================
    # Completion
    # =========================================================

    def finish_logging(
        self,
        reason,
    ):

        if self.finished:
            return

        self.finished = True

        self.close_file()

        self.print_summary(
            reason
        )

    # ---------------------------------------------------------

    def close_file(self):

        if (
            hasattr(
                self,
                'csv_file'
            )
            and
            self.csv_file is not None
            and
            not self.csv_file.closed
        ):

            self.csv_file.flush()

            self.csv_file.close()

    # ---------------------------------------------------------

    def print_summary(
        self,
        reason,
    ):

        print(
            '\n'
            '============================================\n'
            ' TRAJECTORY LOGGER SUMMARY\n'
            '============================================'
        )

        print(
            f'Completion reason       : {reason}'
        )

        print(
            f'CSV                     : {self.output_file}'
        )

        print(
            f'Samples                 : {self.sample_count}'
        )

        if self.sample_count > 0:

            mean_position_error = (
                sum(
                    self.position_errors
                )
                /
                len(
                    self.position_errors
                )
            )

            max_position_error = max(
                self.position_errors
            )

            rmse_position_error = math.sqrt(
                sum(
                    error * error
                    for error
                    in self.position_errors
                )
                /
                len(
                    self.position_errors
                )
            )

            mean_heading_error = (
                sum(
                    self.heading_errors
                )
                /
                len(
                    self.heading_errors
                )
            )

            mean_speed = (
                sum(
                    self.speeds
                )
                /
                len(
                    self.speeds
                )
            )

            print(
                f'Mean position error     : '
                f'{mean_position_error:.3f} m'
            )

            print(
                f'Max position error      : '
                f'{max_position_error:.3f} m'
            )

            print(
                f'Position RMSE           : '
                f'{rmse_position_error:.3f} m'
            )

            print(
                f'Mean |heading error|    : '
                f'{mean_heading_error:.3f} rad'
            )

            print(
                f'Mean speed              : '
                f'{mean_speed:.3f} m/s'
            )

        print(
            '============================================\n'
        )

    # ---------------------------------------------------------

    def shutdown_after_success(self):

        if self.shutdown_timer is not None:

            self.shutdown_timer.cancel()

        if rclpy.ok():

            rclpy.shutdown()

    # =========================================================
    # Node shutdown
    # =========================================================

    def shutdown_logger(self):

        if not self.finished:

            self.finish_logging(
                reason='MANUAL_STOP'
            )

        else:

            self.close_file()


# =============================================================
# Main
# =============================================================


def main(args=None):

    rclpy.init(
        args=args
    )

    node = TrajectoryLogger()

    try:

        rclpy.spin(
            node
        )

    except (
        KeyboardInterrupt,
        ExternalShutdownException,
    ):

        pass

    finally:

        node.shutdown_logger()

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()
