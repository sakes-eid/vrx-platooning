import csv
import json
import math
import os
import statistics

import rclpy
from rclpy.node import Node

from std_msgs.msg import (
    Bool,
    Float64,
    String,
)

from platoon_interfaces.msg import VehicleState


def rmse(values):

    if not values:
        return None

    return math.sqrt(
        sum(
            value * value
            for value in values
        )
        / len(values)
    )


class TwoRobotLogger(Node):

    def __init__(self):

        super().__init__(
            'two_robot_logger'
        )

        self.declare_parameter(
            'output_file',
            '/tmp/two_robot.csv'
        )

        self.declare_parameter(
            'formation_distance',
            5.0
        )

        self.output_file = os.path.expanduser(
            str(
                self.get_parameter(
                    'output_file'
                ).value
            )
        )

        self.target_distance = float(
            self.get_parameter(
                'formation_distance'
            ).value
        )

        os.makedirs(
            os.path.dirname(
                self.output_file
            ),
            exist_ok=True,
        )

        if os.path.exists(
            self.output_file
        ):

            raise RuntimeError(
                'File already exists: '
                + self.output_file
            )

        self.file = open(
            self.output_file,
            'w',
            newline='',
        )

        self.writer = csv.writer(
            self.file
        )

        self.writer.writerow([
            'time_s',
            'mission_state',
            'r2_path_source',

            'r1_north_m',
            'r1_east_m',
            'r1_speed_mps',
            'r1_yaw_rad',

            'r2_north_m',
            'r2_east_m',
            'r2_speed_mps',
            'r2_yaw_rad',

            'd12_m',
            'd12_error_m',

            'r2_target_speed_mps',
            'r2_heading_error_rad',
            'r2_speed_error_mps',

            'r1_left_thrust',
            'r1_right_thrust',

            'r2_left_thrust',
            'r2_right_thrust',

            'r1_success',
            'r2_success',
            'platoon_success',
        ])

        self.r1 = None
        self.r2 = None

        self.mode = 'UNKNOWN'

        self.path_source = 'NONE'

        self.r2_target_speed = float('nan')
        self.r2_heading_error = float('nan')
        self.r2_speed_error = float('nan')

        self.r1_left = 0.0
        self.r1_right = 0.0

        self.r2_left = 0.0
        self.r2_right = 0.0

        self.r1_success = False
        self.r2_success = False
        self.platoon_success = False

        self.start_time = None

        self.follow_errors = []
        self.follow_distances = []

        self.r1_speeds = []
        self.r2_speeds = []

        self.minimum_distance = None

        self.finalized = False

        self.create_subscription(
            VehicleState,
            '/r1/vehicle_state',
            lambda msg:
                setattr(
                    self,
                    'r1',
                    msg,
                ),
            20,
        )

        self.create_subscription(
            VehicleState,
            '/r2/vehicle_state',
            lambda msg:
                setattr(
                    self,
                    'r2',
                    msg,
                ),
            20,
        )

        self.create_subscription(
            String,
            '/platoon/mission_state',
            lambda msg:
                setattr(
                    self,
                    'mode',
                    msg.data,
                ),
            10,
        )

        self.create_subscription(
            String,
            '/r2/control/path_source',
            lambda msg:
                setattr(
                    self,
                    'path_source',
                    msg.data,
                ),
            10,
        )

        self.create_subscription(
            Float64,
            '/r2/control/target_speed',
            lambda msg:
                setattr(
                    self,
                    'r2_target_speed',
                    msg.data,
                ),
            10,
        )

        self.create_subscription(
            Float64,
            '/r2/control/heading_error',
            lambda msg:
                setattr(
                    self,
                    'r2_heading_error',
                    msg.data,
                ),
            10,
        )

        self.create_subscription(
            Float64,
            '/r2/control/speed_error',
            lambda msg:
                setattr(
                    self,
                    'r2_speed_error',
                    msg.data,
                ),
            10,
        )

        self.create_subscription(
            Float64,
            '/wamv/thrusters/left/thrust',
            lambda msg:
                setattr(
                    self,
                    'r1_left',
                    msg.data,
                ),
            10,
        )

        self.create_subscription(
            Float64,
            '/wamv/thrusters/right/thrust',
            lambda msg:
                setattr(
                    self,
                    'r1_right',
                    msg.data,
                ),
            10,
        )

        self.create_subscription(
            Float64,
            '/wamv2/thrusters/left/thrust',
            lambda msg:
                setattr(
                    self,
                    'r2_left',
                    msg.data,
                ),
            10,
        )

        self.create_subscription(
            Float64,
            '/wamv2/thrusters/right/thrust',
            lambda msg:
                setattr(
                    self,
                    'r2_right',
                    msg.data,
                ),
            10,
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

        self.create_subscription(
            Bool,
            '/platoon/success',
            self.platoon_success_callback,
            10,
        )

        self.timer = self.create_timer(
            0.1,
            self.log,
        )

        self.get_logger().info(
            f'Logging to {self.output_file}'
        )


    def r1_success_callback(self, msg):

        if msg.data:
            self.r1_success = True


    def r2_success_callback(self, msg):

        if msg.data:
            self.r2_success = True


    def platoon_success_callback(self, msg):

        if msg.data:

            self.platoon_success = True

            self.finalize()


    def log(self):

        if (
            self.finalized
            or self.r1 is None
            or self.r2 is None
        ):

            return

        now = self.get_clock().now()

        if self.start_time is None:
            self.start_time = now

        time_s = (
            now
            - self.start_time
        ).nanoseconds * 1e-9

        d12 = math.hypot(
            self.r1.x - self.r2.x,
            self.r1.y - self.r2.y,
        )

        error = (
            d12
            - self.target_distance
        )

        if self.minimum_distance is None:

            self.minimum_distance = d12

        else:

            self.minimum_distance = min(
                self.minimum_distance,
                d12,
            )

        if self.mode == 'FOLLOW':

            self.follow_distances.append(
                d12
            )

            self.follow_errors.append(
                error
            )

            self.r1_speeds.append(
                self.r1.speed
            )

            self.r2_speeds.append(
                self.r2.speed
            )

        self.writer.writerow([
            time_s,
            self.mode,
            self.path_source,

            self.r1.x,
            self.r1.y,
            self.r1.speed,
            self.r1.body_yaw,

            self.r2.x,
            self.r2.y,
            self.r2.speed,
            self.r2.body_yaw,

            d12,
            error,

            self.r2_target_speed,
            self.r2_heading_error,
            self.r2_speed_error,

            self.r1_left,
            self.r1_right,

            self.r2_left,
            self.r2_right,

            self.r1_success,
            self.r2_success,
            self.platoon_success,
        ])

        self.file.flush()


    def finalize(self):

        if self.finalized:
            return

        self.finalized = True

        self.file.flush()
        self.file.close()

        summary_file = (
            os.path.splitext(
                self.output_file
            )[0]
            + '.summary.json'
        )

        summary = {

            'success':
                self.platoon_success,

            'minimum_distance_m':
                self.minimum_distance,

            'follow': {

                'samples':
                    len(
                        self.follow_errors
                    ),

                'mean_distance_m':
                    (
                        statistics.mean(
                            self.follow_distances
                        )
                        if self.follow_distances
                        else None
                    ),

                'distance_rmse_m':
                    rmse(
                        self.follow_errors
                    ),

                'maximum_absolute_error_m':
                    (
                        max(
                            abs(x)
                            for x
                            in self.follow_errors
                        )
                        if self.follow_errors
                        else None
                    ),

                'r1_mean_speed_mps':
                    (
                        statistics.mean(
                            self.r1_speeds
                        )
                        if self.r1_speeds
                        else None
                    ),

                'r2_mean_speed_mps':
                    (
                        statistics.mean(
                            self.r2_speeds
                        )
                        if self.r2_speeds
                        else None
                    ),

                'r2_max_speed_mps':
                    (
                        max(
                            self.r2_speeds
                        )
                        if self.r2_speeds
                        else None
                    ),
            },
        }

        with open(
            summary_file,
            'w',
        ) as handle:

            json.dump(
                summary,
                handle,
                indent=2,
            )

        self.get_logger().info(
            f'Summary: {summary_file}'
        )


def main():

    rclpy.init()

    node = TwoRobotLogger()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.finalize()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
