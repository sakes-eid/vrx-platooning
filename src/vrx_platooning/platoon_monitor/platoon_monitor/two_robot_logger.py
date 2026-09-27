import csv
import json
import math
import os
import statistics

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float64, String
from platoon_interfaces.msg import VehicleState


def rmse(values):
    if not values:
        return None

    return math.sqrt(
        sum(v * v for v in values)
        / len(values)
    )


class TwoRobotLogger(Node):

    def __init__(self):
        super().__init__('two_robot_logger')

        self.declare_parameter(
            'output_file',
            '/tmp/two_robot.csv'
        )

        self.declare_parameter(
            'formation_distance',
            5.0
        )

        self.declare_parameter(
            'catchup_distance',
            6.0
        )

        self.output_file = os.path.expanduser(
            str(
                self.get_parameter(
                    'output_file'
                ).value
            )
        )

        self.target_clearance = float(
            self.get_parameter(
                'formation_distance'
            ).value
        )

        self.catchup_distance = float(
            self.get_parameter(
                'catchup_distance'
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
            'follow_phase',
            'r2_path_source',
            'r1_target_capture',
            'r2_longitudinal_state',
            'distance_error_valid',

            'r1_north_m',
            'r1_east_m',
            'r1_speed_mps',
            'r1_yaw_rad',

            'r2_north_m',
            'r2_east_m',
            'r2_speed_mps',
            'r2_yaw_rad',

            'reference_distance_m',
            'hitbox_clearance_m',
            'hitbox_clearance_error_m',

            'collision_warning',
            'avoidance_active',

            'r2_target_speed_mps',
            'r2_heading_error_rad',
            'r2_desired_heading_rad',
            'r2_path_tangent_heading_rad',
            'r2_cross_track_error_m',
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
        self.r1_target_capture = False
        self.r2_longitudinal_state = 'UNKNOWN'

        self.hitbox_clearance = float('nan')
        self.collision_warning = False
        self.avoidance_active = False

        self.r2_target_speed = float('nan')
        self.r2_heading_error = float('nan')
        self.r2_desired_heading = float('nan')
        self.r2_path_tangent_heading = float('nan')
        self.r2_cross_track_error = float('nan')
        self.r2_speed_error = float('nan')

        self.r1_left = 0.0
        self.r1_right = 0.0
        self.r2_left = 0.0
        self.r2_right = 0.0

        self.r1_success = False
        self.r2_success = False
        self.platoon_success = False

        self.start_time = None
        self.finalized = False

        self.follow_clearance_errors = []
        self.follow_clearances = []
        self.follow_reference_distances = []
        self.follow_heading_errors = []
        self.follow_cross_track_errors = []

        self.catchup_clearances = []
        self.catchup_r1_speeds = []
        self.catchup_r2_speeds = []

        self.following_clearances = []
        self.following_clearance_errors = []
        self.following_heading_errors = []
        self.following_cross_track_errors = []
        self.following_r1_speeds = []
        self.following_r2_speeds = []

        self.follow_phase_first_time = {}
        self.follow_phase_last_time = {}
        self.longitudinal_state_first_time = {}
        self.longitudinal_state_last_time = {}
        self.longitudinal_state_samples = {}
        self.target_capture_time_s = None

        self.r1_speeds = []
        self.r2_speeds = []

        self.minimum_clearance = None
        self.minimum_reference_distance = None

        self.warning_samples = 0
        self.avoidance_samples = 0

        self.mode_first_time = {}

        self.create_subscription(
            VehicleState,
            '/r1/vehicle_state',
            lambda msg:
                setattr(self, 'r1', msg),
            20,
        )

        self.create_subscription(
            VehicleState,
            '/r2/vehicle_state',
            lambda msg:
                setattr(self, 'r2', msg),
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
            Bool,
            '/r1/target_capture',
            self.target_capture_callback,
            10,
        )

        self.create_subscription(
            String,
            '/r2/control/longitudinal_state',
            lambda msg:
                setattr(
                    self,
                    'r2_longitudinal_state',
                    msg.data,
                ),
            10,
        )

        self.create_subscription(
            Float64,
            '/r2/safety/hitbox_clearance',
            lambda msg:
                setattr(
                    self,
                    'hitbox_clearance',
                    float(msg.data),
                ),
            20,
        )

        self.create_subscription(
            Bool,
            '/r2/safety/collision_warning',
            lambda msg:
                setattr(
                    self,
                    'collision_warning',
                    bool(msg.data),
                ),
            20,
        )

        self.create_subscription(
            Bool,
            '/r2/safety/avoidance_active',
            lambda msg:
                setattr(
                    self,
                    'avoidance_active',
                    bool(msg.data),
                ),
            20,
        )

        self.create_subscription(
            Float64,
            '/r2/control/target_speed',
            lambda msg:
                setattr(
                    self,
                    'r2_target_speed',
                    float(msg.data),
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
                    float(msg.data),
                ),
            10,
        )

        self.create_subscription(
            Float64,
            '/r2/control/desired_heading',
            lambda msg:
                setattr(
                    self,
                    'r2_desired_heading',
                    float(msg.data),
                ),
            10,
        )

        self.create_subscription(
            Float64,
            '/r2/control/path_tangent_heading',
            lambda msg:
                setattr(
                    self,
                    'r2_path_tangent_heading',
                    float(msg.data),
                ),
            10,
        )

        self.create_subscription(
            Float64,
            '/r2/control/cross_track_error',
            lambda msg:
                setattr(
                    self,
                    'r2_cross_track_error',
                    float(msg.data),
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
                    float(msg.data),
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
                    float(msg.data),
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
                    float(msg.data),
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
                    float(msg.data),
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
                    float(msg.data),
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
            0.10,
            self.log,
        )

        self.get_logger().info(
            f'Hitbox-aware logger: {self.output_file}'
        )

    def target_capture_callback(self, msg):
        if msg.data:
            self.r1_target_capture = True

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

        if self.mode not in self.mode_first_time:
            self.mode_first_time[
                self.mode
            ] = time_s

        reference_distance = math.hypot(
            self.r1.x - self.r2.x,
            self.r1.y - self.r2.y,
        )

        raw_clearance_error = (
            self.hitbox_clearance
            - self.target_clearance
        )

        if math.isfinite(
            self.hitbox_clearance
        ):
            if self.minimum_clearance is None:
                self.minimum_clearance = (
                    self.hitbox_clearance
                )
            else:
                self.minimum_clearance = min(
                    self.minimum_clearance,
                    self.hitbox_clearance,
                )

        if self.minimum_reference_distance is None:
            self.minimum_reference_distance = (
                reference_distance
            )
        else:
            self.minimum_reference_distance = min(
                self.minimum_reference_distance,
                reference_distance,
            )

        if self.collision_warning:
            self.warning_samples += 1

        if self.avoidance_active:
            self.avoidance_samples += 1

        if (
            self.r1_target_capture
            and self.target_capture_time_s is None
        ):
            self.target_capture_time_s = time_s

        state = self.r2_longitudinal_state
        if state not in self.longitudinal_state_first_time:
            self.longitudinal_state_first_time[state] = time_s
        self.longitudinal_state_last_time[state] = time_s
        self.longitudinal_state_samples[state] = (
            self.longitudinal_state_samples.get(state, 0)
            + 1
        )

        follow_phase = 'NOT_FOLLOW'

        if self.mode == 'FOLLOW':
            if math.isfinite(self.hitbox_clearance):
                if self.hitbox_clearance >= self.catchup_distance:
                    follow_phase = 'CATCHUP'
                else:
                    follow_phase = 'FOLLOWING'
            else:
                follow_phase = 'UNKNOWN'

        if follow_phase in ('CATCHUP', 'FOLLOWING'):
            if follow_phase not in self.follow_phase_first_time:
                self.follow_phase_first_time[follow_phase] = time_s
            self.follow_phase_last_time[follow_phase] = time_s

        # The 5 m platooning error is intentionally VALID ONLY
        # during steady FOLLOWING. Catch-up is a transient acquisition
        # phase, and everything after breadcrumb unsubscribe is terminal
        # braking/parking rather than platooning regulation.
        distance_error_valid = (
            follow_phase == 'FOLLOWING'
            and self.path_source == 'BREADCRUMB'
            and not self.r1_target_capture
            and math.isfinite(self.hitbox_clearance)
        )

        clearance_error = (
            raw_clearance_error
            if distance_error_valid
            else float('nan')
        )

        if (
            self.mode == 'FOLLOW'
            and math.isfinite(self.hitbox_clearance)
        ):
            if follow_phase == 'CATCHUP':
                # Raw clearance and speed remain useful, but there is
                # deliberately NO 5 m error metric during catch-up.
                self.catchup_clearances.append(
                    self.hitbox_clearance
                )
                self.catchup_r1_speeds.append(self.r1.speed)
                self.catchup_r2_speeds.append(self.r2.speed)

            elif distance_error_valid:
                self.following_clearances.append(
                    self.hitbox_clearance
                )
                self.following_clearance_errors.append(
                    raw_clearance_error
                )
                self.following_r1_speeds.append(self.r1.speed)
                self.following_r2_speeds.append(self.r2.speed)

                if math.isfinite(self.r2_heading_error):
                    self.following_heading_errors.append(
                        self.r2_heading_error
                    )

                if math.isfinite(self.r2_cross_track_error):
                    self.following_cross_track_errors.append(
                        self.r2_cross_track_error
                    )

                # Backward-compatible aggregate now means VALID
                # platooning-error samples only.
                self.follow_clearances.append(
                    self.hitbox_clearance
                )
                self.follow_clearance_errors.append(
                    raw_clearance_error
                )
                self.follow_reference_distances.append(
                    reference_distance
                )

                if math.isfinite(self.r2_heading_error):
                    self.follow_heading_errors.append(
                        self.r2_heading_error
                    )

                if math.isfinite(self.r2_cross_track_error):
                    self.follow_cross_track_errors.append(
                        self.r2_cross_track_error
                    )

                self.r1_speeds.append(self.r1.speed)
                self.r2_speeds.append(self.r2.speed)

        self.writer.writerow([
            time_s,
            self.mode,
            follow_phase,
            self.path_source,
            self.r1_target_capture,
            self.r2_longitudinal_state,
            distance_error_valid,

            self.r1.x,
            self.r1.y,
            self.r1.speed,
            self.r1.body_yaw,

            self.r2.x,
            self.r2.y,
            self.r2.speed,
            self.r2.body_yaw,

            reference_distance,
            self.hitbox_clearance,
            clearance_error,

            self.collision_warning,
            self.avoidance_active,

            self.r2_target_speed,
            self.r2_heading_error,
            self.r2_desired_heading,
            self.r2_path_tangent_heading,
            self.r2_cross_track_error,
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

        try:
            self.file.flush()
            self.file.close()
        except Exception:
            pass

        summary_file = (
            os.path.splitext(
                self.output_file
            )[0]
            + '.summary.json'
        )

        summary = {
            'success':
                self.platoon_success,

            'minimum_hitbox_clearance_m':
                self.minimum_clearance,

            'minimum_reference_distance_m':
                self.minimum_reference_distance,

            'collision_warning_samples':
                self.warning_samples,

            'avoidance_active_samples':
                self.avoidance_samples,

            'mission_state_first_time_s':
                self.mode_first_time,

            'follow_phase_first_time_s':
                self.follow_phase_first_time,

            'follow_phase_last_time_s':
                self.follow_phase_last_time,

            'catchup_distance_m':
                self.catchup_distance,

            'r1_target_capture_time_s':
                self.target_capture_time_s,

            'r2_longitudinal_state_first_time_s':
                self.longitudinal_state_first_time,

            'r2_longitudinal_state_last_time_s':
                self.longitudinal_state_last_time,

            'r2_longitudinal_state_samples':
                self.longitudinal_state_samples,

            'distance_error_policy': (
                '5 m hitbox-clearance error is recorded only during '
                'FOLLOWING while breadcrumb subscription is active; '
                'CATCHUP and post-unsubscribe phases are excluded.'
            ),

            'catchup': {
                'samples':
                    len(self.catchup_clearances),

                'mean_hitbox_clearance_m':
                    (statistics.mean(self.catchup_clearances)
                     if self.catchup_clearances else None),

                'r1_mean_speed_mps':
                    (statistics.mean(self.catchup_r1_speeds)
                     if self.catchup_r1_speeds else None),

                'r2_mean_speed_mps':
                    (statistics.mean(self.catchup_r2_speeds)
                     if self.catchup_r2_speeds else None),

                'r2_max_speed_mps':
                    (max(self.catchup_r2_speeds)
                     if self.catchup_r2_speeds else None),
            },

            'following': {
                'samples':
                    len(self.following_clearances),

                'target_hitbox_clearance_m':
                    self.target_clearance,

                'mean_hitbox_clearance_m':
                    (statistics.mean(self.following_clearances)
                     if self.following_clearances else None),

                'hitbox_clearance_rmse_m':
                    rmse(self.following_clearance_errors),

                'r1_mean_speed_mps':
                    (statistics.mean(self.following_r1_speeds)
                     if self.following_r1_speeds else None),

                'r2_mean_speed_mps':
                    (statistics.mean(self.following_r2_speeds)
                     if self.following_r2_speeds else None),

                'r2_max_speed_mps':
                    (max(self.following_r2_speeds)
                     if self.following_r2_speeds else None),

                'heading_error_rmse_deg':
                    (math.degrees(rmse(self.following_heading_errors))
                     if self.following_heading_errors else None),

                'cross_track_error_rmse_m':
                    rmse(self.following_cross_track_errors),
            },

            'follow': {
                'note': (
                    'Backward-compatible alias containing only valid '
                    'steady FOLLOWING samples.'
                ),

                'samples':
                    len(
                        self.follow_clearance_errors
                    ),

                'target_hitbox_clearance_m':
                    self.target_clearance,

                'mean_hitbox_clearance_m':
                    (
                        statistics.mean(
                            self.follow_clearances
                        )
                        if self.follow_clearances
                        else None
                    ),

                'hitbox_clearance_rmse_m':
                    rmse(
                        self.follow_clearance_errors
                    ),

                'maximum_abs_clearance_error_m':
                    (
                        max(
                            abs(v)
                            for v
                            in self.follow_clearance_errors
                        )
                        if self.follow_clearance_errors
                        else None
                    ),

                'mean_reference_distance_m':
                    (
                        statistics.mean(
                            self.follow_reference_distances
                        )
                        if self.follow_reference_distances
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
                        max(self.r2_speeds)
                        if self.r2_speeds
                        else None
                    ),

                'heading_error_rmse_deg':
                    (
                        math.degrees(
                            rmse(self.follow_heading_errors)
                        )
                        if self.follow_heading_errors
                        else None
                    ),

                'cross_track_error_rmse_m':
                    rmse(
                        self.follow_cross_track_errors
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
