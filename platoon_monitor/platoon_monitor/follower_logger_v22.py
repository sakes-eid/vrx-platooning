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


def percentile(values, q):
    if not values:
        return None

    data = sorted(values)

    if len(data) == 1:
        return data[0]

    pos = (len(data) - 1) * q
    lo = math.floor(pos)
    hi = math.ceil(pos)

    if lo == hi:
        return data[lo]

    w = pos - lo

    return (
        data[lo] * (1.0 - w)
        + data[hi] * w
    )


class FollowerLogger(Node):

    def __init__(self):
        super().__init__('follower_logger')

        # ------------------------------------------------------
        # Generic predecessor -> follower identity
        # ------------------------------------------------------
        self.declare_parameter('follower_id', 'r2')
        self.declare_parameter('predecessor_id', 'r1')

        self.declare_parameter(
            'predecessor_actuator_prefix',
            '/wamv'
        )

        self.declare_parameter(
            'follower_actuator_prefix',
            '/wamv2'
        )

        self.declare_parameter(
            'mission_state_topic',
            '/platoon/mission_state'
        )

        self.declare_parameter(
            'predecessor_success_topic',
            '/experiment/success'
        )

        self.declare_parameter(
            'pair_success_topic',
            '/platoon/success'
        )

        self.follower_id = str(
            self.get_parameter('follower_id').value
        )

        self.predecessor_id = str(
            self.get_parameter('predecessor_id').value
        )

        self.predecessor_actuator_prefix = str(
            self.get_parameter(
                'predecessor_actuator_prefix'
            ).value
        ).rstrip('/')

        self.follower_actuator_prefix = str(
            self.get_parameter(
                'follower_actuator_prefix'
            ).value
        ).rstrip('/')

        self.mission_state_topic = str(
            self.get_parameter(
                'mission_state_topic'
            ).value
        )

        self.predecessor_success_topic = str(
            self.get_parameter(
                'predecessor_success_topic'
            ).value
        )

        self.pair_success_topic = str(
            self.get_parameter(
                'pair_success_topic'
            ).value
        )

        self.predecessor_state_topic = (
            f'/{self.predecessor_id}/vehicle_state'
        )

        self.follower_state_topic = (
            f'/{self.follower_id}/vehicle_state'
        )

        self.target_capture_topic = (
            f'/{self.predecessor_id}/target_capture'
        )

        self.declare_parameter(
            'output_file',
            '/tmp/follower.csv'
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

        self.target_path_gap = float(
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
            'follower_path_source',
            'predecessor_target_capture',
            'follower_longitudinal_state',
            'distance_error_valid',

            'predecessor_north_m',
            'predecessor_east_m',
            'predecessor_speed_mps',
            'predecessor_yaw_rad',

            'follower_north_m',
            'follower_east_m',
            'follower_speed_mps',
            'follower_yaw_rad',

            'reference_distance_m',

            # V2.2 longitudinal formation measurement.
            'path_gap_m',
            'path_gap_error_m',
            'path_gap_valid',

            # Exact polygon clearance is safety only.
            'hitbox_clearance_m',

            'collision_warning',
            'avoidance_active',

            'follower_target_speed_mps',
            'follower_heading_error_rad',
            'follower_desired_heading_rad',
            'follower_path_tangent_heading_rad',
            'follower_cross_track_error_m',
            'follower_speed_error_mps',

            'predecessor_left_thrust',
            'predecessor_right_thrust',
            'follower_left_thrust',
            'follower_right_thrust',

            'predecessor_success',
            'follower_success',
            'pair_success',
        ])

        self.predecessor = None
        self.follower = None

        self.mode = 'UNKNOWN'
        self.path_source = 'NONE'
        self.predecessor_target_capture = False
        self.follower_longitudinal_state = 'UNKNOWN'

        # V2.2 formation measurement from raw breadcrumb arc length.
        self.path_gap = float('nan')
        self.path_gap_valid = False

        # Physical oriented-hull safety measurement.
        self.hitbox_clearance = float('nan')
        self.collision_warning = False
        self.avoidance_active = False

        self.follower_target_speed = float('nan')
        self.follower_heading_error = float('nan')
        self.follower_desired_heading = float('nan')
        self.follower_path_tangent_heading = float('nan')
        self.follower_cross_track_error = float('nan')
        self.follower_speed_error = float('nan')

        self.predecessor_left = 0.0
        self.predecessor_right = 0.0
        self.follower_left = 0.0
        self.follower_right = 0.0

        self.predecessor_success = False
        self.follower_success = False
        self.pair_success = False

        self.start_time = None
        self.finalized = False

        # Number of successfully written CSV data rows.
        self.rows = 0

        # V2.2 formation metrics.
        self.follow_path_gaps = []
        self.follow_path_gap_errors = []
        self.follow_reference_distances = []
        self.follow_heading_errors = []
        self.follow_cross_track_errors = []

        self.catchup_path_gaps = []
        self.catchup_predecessor_speeds = []
        self.catchup_follower_speeds = []

        self.following_path_gaps = []
        self.following_path_gap_errors = []
        self.following_heading_errors = []
        self.following_cross_track_errors = []
        self.following_predecessor_speeds = []
        self.following_follower_speeds = []

        self.follow_phase_first_time = {}
        self.follow_phase_last_time = {}
        self.longitudinal_state_first_time = {}
        self.longitudinal_state_last_time = {}
        self.longitudinal_state_samples = {}
        self.target_capture_time_s = None

        self.predecessor_speeds = []
        self.follower_speeds = []

        self.minimum_path_gap = None
        self.minimum_clearance = None
        self.minimum_reference_distance = None

        self.warning_samples = 0
        self.avoidance_samples = 0

        self.mode_first_time = {}

        self.create_subscription(
            VehicleState,
            self.predecessor_state_topic,
            lambda msg:
                setattr(self, 'predecessor', msg),
            20,
        )

        self.create_subscription(
            VehicleState,
            self.follower_state_topic,
            lambda msg:
                setattr(self, 'follower', msg),
            20,
        )

        self.create_subscription(
            String,
            self.mission_state_topic,
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
            f'/{self.follower_id}/control/path_source',
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
            self.target_capture_topic,
            self.target_capture_callback,
            10,
        )

        self.create_subscription(
            String,
            f'/{self.follower_id}/control/longitudinal_state',
            lambda msg:
                setattr(
                    self,
                    'follower_longitudinal_state',
                    msg.data,
                ),
            10,
        )

        # V2.2 unsmoothed breadcrumb arc-length formation gap.
        self.create_subscription(
            Float64,
            f'/planner/{self.follower_id}/path_gap',
            lambda msg:
                setattr(
                    self,
                    'path_gap',
                    float(msg.data),
                ),
            20,
        )

        self.create_subscription(
            Bool,
            f'/planner/{self.follower_id}/path_gap_valid',
            lambda msg:
                setattr(
                    self,
                    'path_gap_valid',
                    bool(msg.data),
                ),
            20,
        )

        self.create_subscription(
            Float64,
            f'/{self.follower_id}/safety/hitbox_clearance',
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
            f'/{self.follower_id}/safety/collision_warning',
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
            f'/{self.follower_id}/safety/avoidance_active',
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
            f'/{self.follower_id}/control/target_speed',
            lambda msg:
                setattr(
                    self,
                    'follower_target_speed',
                    float(msg.data),
                ),
            10,
        )

        self.create_subscription(
            Float64,
            f'/{self.follower_id}/control/heading_error',
            lambda msg:
                setattr(
                    self,
                    'follower_heading_error',
                    float(msg.data),
                ),
            10,
        )

        self.create_subscription(
            Float64,
            f'/{self.follower_id}/control/desired_heading',
            lambda msg:
                setattr(
                    self,
                    'follower_desired_heading',
                    float(msg.data),
                ),
            10,
        )

        self.create_subscription(
            Float64,
            f'/{self.follower_id}/control/path_tangent_heading',
            lambda msg:
                setattr(
                    self,
                    'follower_path_tangent_heading',
                    float(msg.data),
                ),
            10,
        )

        self.create_subscription(
            Float64,
            f'/{self.follower_id}/control/cross_track_error',
            lambda msg:
                setattr(
                    self,
                    'follower_cross_track_error',
                    float(msg.data),
                ),
            10,
        )

        self.create_subscription(
            Float64,
            f'/{self.follower_id}/control/speed_error',
            lambda msg:
                setattr(
                    self,
                    'follower_speed_error',
                    float(msg.data),
                ),
            10,
        )

        self.create_subscription(
            Float64,
            f'{self.predecessor_actuator_prefix}/thrusters/left/thrust',
            lambda msg:
                setattr(
                    self,
                    'predecessor_left',
                    float(msg.data),
                ),
            10,
        )

        self.create_subscription(
            Float64,
            f'{self.predecessor_actuator_prefix}/thrusters/right/thrust',
            lambda msg:
                setattr(
                    self,
                    'predecessor_right',
                    float(msg.data),
                ),
            10,
        )

        self.create_subscription(
            Float64,
            f'{self.follower_actuator_prefix}/thrusters/left/thrust',
            lambda msg:
                setattr(
                    self,
                    'follower_left',
                    float(msg.data),
                ),
            10,
        )

        self.create_subscription(
            Float64,
            f'{self.follower_actuator_prefix}/thrusters/right/thrust',
            lambda msg:
                setattr(
                    self,
                    'follower_right',
                    float(msg.data),
                ),
            10,
        )

        self.create_subscription(
            Bool,
            self.predecessor_success_topic,
            self.predecessor_success_callback,
            10,
        )

        self.create_subscription(
            Bool,
            f'/{self.follower_id}/success',
            self.follower_success_callback,
            10,
        )

        self.create_subscription(
            Bool,
            self.pair_success_topic,
            self.pair_success_callback,
            10,
        )

        self.timer = self.create_timer(
            0.10,
            self.log,
        )

        self.get_logger().info(
            f'Follower logger: {self.follower_id} <- {self.predecessor_id}; {self.output_file}'
        )

    def target_capture_callback(self, msg):
        if msg.data:
            self.predecessor_target_capture = True

    def predecessor_success_callback(self, msg):
        if msg.data:
            self.predecessor_success = True

    def follower_success_callback(self, msg):
        if msg.data:
            self.follower_success = True

    def pair_success_callback(self, msg):
        if msg.data:
            self.pair_success = True
            self.finalize()

    def log(self):
        if (
            self.finalized
            or self.predecessor is None
            or self.follower is None
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
            self.predecessor.x - self.follower.x,
            self.predecessor.y - self.follower.y,
        )

        raw_path_gap_error = float('nan')

        if (
            self.path_gap_valid
            and math.isfinite(self.path_gap)
        ):
            raw_path_gap_error = (
                self.path_gap
                - self.target_path_gap
            )

            if self.minimum_path_gap is None:
                self.minimum_path_gap = self.path_gap
            else:
                self.minimum_path_gap = min(
                    self.minimum_path_gap,
                    self.path_gap,
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
            self.predecessor_target_capture
            and self.target_capture_time_s is None
        ):
            self.target_capture_time_s = time_s

        state = self.follower_longitudinal_state
        if state not in self.longitudinal_state_first_time:
            self.longitudinal_state_first_time[state] = time_s
        self.longitudinal_state_last_time[state] = time_s
        self.longitudinal_state_samples[state] = (
            self.longitudinal_state_samples.get(state, 0)
            + 1
        )

        follow_phase = 'NOT_FOLLOW'

        if self.mode == 'FOLLOW':
            if (
                self.path_gap_valid
                and math.isfinite(self.path_gap)
            ):
                if self.path_gap >= self.catchup_distance:
                    follow_phase = 'CATCHUP'
                else:
                    follow_phase = 'FOLLOWING'
            else:
                follow_phase = 'UNKNOWN'

        if follow_phase in ('CATCHUP', 'FOLLOWING'):
            if follow_phase not in self.follow_phase_first_time:
                self.follow_phase_first_time[follow_phase] = time_s
            self.follow_phase_last_time[follow_phase] = time_s

        # The 5 m formation error is now the bumper-to-bumper
        # ARC-LENGTH gap along the raw breadcrumb chain.
        #
        # It is valid only during steady FOLLOWING. Catch-up and
        # terminal braking/parking are excluded from regulation metrics.
        distance_error_valid = (
            follow_phase == 'FOLLOWING'
            and self.path_source == 'BREADCRUMB'
            and not self.predecessor_target_capture
            and self.path_gap_valid
            and math.isfinite(self.path_gap)
        )

        path_gap_error = (
            raw_path_gap_error
            if distance_error_valid
            else float('nan')
        )

        if (
            self.mode == 'FOLLOW'
            and self.path_gap_valid
            and math.isfinite(self.path_gap)
        ):
            if follow_phase == 'CATCHUP':
                # Acquisition is logged separately and does not
                # contribute to the steady 5 m regulation score.
                self.catchup_path_gaps.append(
                    self.path_gap
                )
                self.catchup_predecessor_speeds.append(
                    self.predecessor.speed
                )
                self.catchup_follower_speeds.append(
                    self.follower.speed
                )

            elif distance_error_valid:
                self.following_path_gaps.append(
                    self.path_gap
                )
                self.following_path_gap_errors.append(
                    raw_path_gap_error
                )

                self.following_predecessor_speeds.append(
                    self.predecessor.speed
                )
                self.following_follower_speeds.append(
                    self.follower.speed
                )

                if math.isfinite(
                    self.follower_heading_error
                ):
                    self.following_heading_errors.append(
                        self.follower_heading_error
                    )

                if math.isfinite(
                    self.follower_cross_track_error
                ):
                    self.following_cross_track_errors.append(
                        self.follower_cross_track_error
                    )

                # Aggregate steady-follow metrics.
                self.follow_path_gaps.append(
                    self.path_gap
                )
                self.follow_path_gap_errors.append(
                    raw_path_gap_error
                )
                self.follow_reference_distances.append(
                    reference_distance
                )

                if math.isfinite(
                    self.follower_heading_error
                ):
                    self.follow_heading_errors.append(
                        self.follower_heading_error
                    )

                if math.isfinite(
                    self.follower_cross_track_error
                ):
                    self.follow_cross_track_errors.append(
                        self.follower_cross_track_error
                    )

                self.predecessor_speeds.append(
                    self.predecessor.speed
                )
                self.follower_speeds.append(
                    self.follower.speed
                )

        self.writer.writerow([
            time_s,
            self.mode,
            follow_phase,
            self.path_source,
            self.predecessor_target_capture,
            self.follower_longitudinal_state,
            distance_error_valid,

            self.predecessor.x,
            self.predecessor.y,
            self.predecessor.speed,
            self.predecessor.body_yaw,

            self.follower.x,
            self.follower.y,
            self.follower.speed,
            self.follower.body_yaw,

            reference_distance,

            self.path_gap,
            path_gap_error,
            self.path_gap_valid,

            self.hitbox_clearance,

            self.collision_warning,
            self.avoidance_active,

            self.follower_target_speed,
            self.follower_heading_error,
            self.follower_desired_heading,
            self.follower_path_tangent_heading,
            self.follower_cross_track_error,
            self.follower_speed_error,

            self.predecessor_left,
            self.predecessor_right,
            self.follower_left,
            self.follower_right,

            self.predecessor_success,
            self.follower_success,
            self.pair_success,
        ])

        self.rows += 1
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
                bool(self.pair_success and self.rows > 0),

            # -------------------------------------------------
            # Formation
            # -------------------------------------------------
            'target_path_gap_m':
                self.target_path_gap,

            'minimum_path_gap_m':
                self.minimum_path_gap,

            'distance_error_policy': (
                '5 m formation error is bumper-to-bumper arc-length '
                'along the unsmoothed breadcrumb chain. It is recorded '
                'only during steady FOLLOWING while breadcrumb guidance '
                'is active. Catch-up and terminal phases are excluded.'
            ),

            # -------------------------------------------------
            # Physical safety
            # -------------------------------------------------
            'minimum_hitbox_clearance_m':
                self.minimum_clearance,

            'minimum_reference_distance_m':
                self.minimum_reference_distance,

            'collision_warning_samples':
                self.warning_samples,

            'avoidance_active_samples':
                self.avoidance_samples,

            # -------------------------------------------------
            # Mission timing/state
            # -------------------------------------------------
            'mission_state_first_time_s':
                self.mode_first_time,

            'follow_phase_first_time_s':
                self.follow_phase_first_time,

            'follow_phase_last_time_s':
                self.follow_phase_last_time,

            'catchup_distance_m':
                self.catchup_distance,

            'predecessor_target_capture_time_s':
                self.target_capture_time_s,

            'follower_longitudinal_state_first_time_s':
                self.longitudinal_state_first_time,

            'follower_longitudinal_state_last_time_s':
                self.longitudinal_state_last_time,

            'follower_longitudinal_state_samples':
                self.longitudinal_state_samples,

            # -------------------------------------------------
            # Acquisition
            # -------------------------------------------------
            'catchup': {
                'samples':
                    len(self.catchup_path_gaps),

                'mean_path_gap_m':
                    (
                        statistics.mean(
                            self.catchup_path_gaps
                        )
                        if self.catchup_path_gaps
                        else None
                    ),

                'predecessor_mean_speed_mps':
                    (
                        statistics.mean(
                            self.catchup_predecessor_speeds
                        )
                        if self.catchup_predecessor_speeds
                        else None
                    ),

                'follower_mean_speed_mps':
                    (
                        statistics.mean(
                            self.catchup_follower_speeds
                        )
                        if self.catchup_follower_speeds
                        else None
                    ),

                'follower_max_speed_mps':
                    (
                        max(self.catchup_follower_speeds)
                        if self.catchup_follower_speeds
                        else None
                    ),
            },

            # -------------------------------------------------
            # Steady platooning
            # -------------------------------------------------
            'following': {
                'samples':
                    len(self.following_path_gap_errors),

                'target_path_gap_m':
                    self.target_path_gap,

                'mean_path_gap_m':
                    (
                        statistics.mean(
                            self.following_path_gaps
                        )
                        if self.following_path_gaps
                        else None
                    ),

                'path_gap_bias_m':
                    (
                        statistics.mean(
                            self.following_path_gap_errors
                        )
                        if self.following_path_gap_errors
                        else None
                    ),

                'path_gap_rmse_m':
                    rmse(
                        self.following_path_gap_errors
                    ),

                'path_gap_p95_abs_error_m':
                    percentile(
                        [
                            abs(v)
                            for v
                            in self.following_path_gap_errors
                        ],
                        0.95,
                    ),

                'maximum_abs_path_gap_error_m':
                    (
                        max(
                            abs(v)
                            for v
                            in self.following_path_gap_errors
                        )
                        if self.following_path_gap_errors
                        else None
                    ),

                'predecessor_mean_speed_mps':
                    (
                        statistics.mean(
                            self.following_predecessor_speeds
                        )
                        if self.following_predecessor_speeds
                        else None
                    ),

                'follower_mean_speed_mps':
                    (
                        statistics.mean(
                            self.following_follower_speeds
                        )
                        if self.following_follower_speeds
                        else None
                    ),

                'follower_max_speed_mps':
                    (
                        max(
                            self.following_follower_speeds
                        )
                        if self.following_follower_speeds
                        else None
                    ),

                'heading_error_rmse_deg':
                    (
                        math.degrees(
                            rmse(
                                self.following_heading_errors
                            )
                        )
                        if self.following_heading_errors
                        else None
                    ),

                'cross_track_error_rmse_m':
                    rmse(
                        self.following_cross_track_errors
                    ),
            },

            # -------------------------------------------------
            # Aggregate steady FOLLOW metrics
            # -------------------------------------------------
            'follow': {
                'note': (
                    'Steady FOLLOWING samples using V2.2 '
                    'path-gap formation semantics.'
                ),

                'samples':
                    len(self.follow_path_gap_errors),

                'target_path_gap_m':
                    self.target_path_gap,

                'mean_path_gap_m':
                    (
                        statistics.mean(
                            self.follow_path_gaps
                        )
                        if self.follow_path_gaps
                        else None
                    ),

                'path_gap_bias_m':
                    (
                        statistics.mean(
                            self.follow_path_gap_errors
                        )
                        if self.follow_path_gap_errors
                        else None
                    ),

                'path_gap_rmse_m':
                    rmse(
                        self.follow_path_gap_errors
                    ),

                'path_gap_p95_abs_error_m':
                    percentile(
                        [
                            abs(v)
                            for v
                            in self.follow_path_gap_errors
                        ],
                        0.95,
                    ),

                'maximum_abs_path_gap_error_m':
                    (
                        max(
                            abs(v)
                            for v
                            in self.follow_path_gap_errors
                        )
                        if self.follow_path_gap_errors
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

                'heading_error_rmse_deg':
                    (
                        math.degrees(
                            rmse(
                                self.follow_heading_errors
                            )
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

    node = FollowerLogger()

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
