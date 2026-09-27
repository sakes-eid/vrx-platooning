#!/usr/bin/env python3
import csv
import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float64, String
from platoon_interfaces.msg import VehicleState


class R1StressLogger(Node):
    def __init__(self):
        super().__init__('r1_stress_logger')

        self.declare_parameter('run_name', 'r1_stress_baseline_01')
        self.declare_parameter(
            'results_root',
            str(Path.home() / 'vrx_ws/src/vrx_platooning/results'),
        )

        run_name = str(self.get_parameter('run_name').value)
        results_root = Path(str(self.get_parameter('results_root').value))
        self.run_dir = results_root / run_name
        self.run_dir.mkdir(parents=True, exist_ok=True)

        self.csv_path = self.run_dir / 'r1_stress.csv'
        self.summary_path = self.run_dir / 'r1_stress.summary.json'

        self.file = self.csv_path.open('w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            'time_s',
            'north_m', 'east_m', 'body_yaw_rad',
            'vx_mps', 'vy_mps', 'speed_mps',
            'controller_state',
            'target_speed_mps', 'speed_error_mps',
            'desired_heading_rad', 'heading_error_rad',
            'cross_track_error_m', 'lookahead_distance_m',
            'planner_speed_limit_mps',
            'planner_upcoming_curvature_1pm',
            'planner_distance_to_turn_m', 'planner_turn_severity',
            'left_thrust', 'right_thrust',
        ])

        self.r1 = None
        self.start_time = None
        self.state = 'UNKNOWN'
        self.target_speed = float('nan')
        self.speed_error = float('nan')
        self.desired_heading = float('nan')
        self.heading_error = float('nan')
        self.cross_track = float('nan')
        self.lookahead = float('nan')
        self.speed_limit = float('nan')
        self.curvature = float('nan')
        self.distance_to_turn = float('nan')
        self.turn_severity = 'UNKNOWN'
        self.left_thrust = float('nan')
        self.right_thrust = float('nan')
        self.success = False

        self.rows = 0
        self.track_cross = []
        self.track_heading = []
        self.track_speed_error = []
        self.track_speed = []
        self.max_speed_seen = 0.0
        self.state_first = {}
        self.state_last = {}

        self.create_subscription(VehicleState, '/r1/vehicle_state', self.r1_cb, 20)
        self.create_subscription(String, '/r1/control/state', self.state_cb, 10)
        self.create_subscription(Float64, '/r1/control/target_speed', lambda m: setattr(self, 'target_speed', float(m.data)), 10)
        self.create_subscription(Float64, '/r1/control/speed_error', lambda m: setattr(self, 'speed_error', float(m.data)), 10)
        self.create_subscription(Float64, '/r1/control/desired_heading', lambda m: setattr(self, 'desired_heading', float(m.data)), 10)
        self.create_subscription(Float64, '/r1/control/heading_error', lambda m: setattr(self, 'heading_error', float(m.data)), 10)
        self.create_subscription(Float64, '/r1/control/cross_track_error', lambda m: setattr(self, 'cross_track', float(m.data)), 10)
        self.create_subscription(Float64, '/r1/control/lookahead_distance', lambda m: setattr(self, 'lookahead', float(m.data)), 10)
        self.create_subscription(Float64, '/planner/r1/speed_limit', lambda m: setattr(self, 'speed_limit', float(m.data)), 10)
        self.create_subscription(Float64, '/planner/r1/upcoming_curvature', lambda m: setattr(self, 'curvature', float(m.data)), 10)
        self.create_subscription(Float64, '/planner/r1/distance_to_turn', lambda m: setattr(self, 'distance_to_turn', float(m.data)), 10)
        self.create_subscription(String, '/planner/r1/turn_severity', lambda m: setattr(self, 'turn_severity', m.data), 10)
        self.create_subscription(Float64, '/wamv/thrusters/left/thrust', lambda m: setattr(self, 'left_thrust', float(m.data)), 10)
        self.create_subscription(Float64, '/wamv/thrusters/right/thrust', lambda m: setattr(self, 'right_thrust', float(m.data)), 10)
        self.create_subscription(Bool, '/r1/success', self.success_cb, 10)

        self.timer = self.create_timer(0.10, self.sample)
        self.get_logger().info(f'R1 stress CSV: {self.csv_path}')

    def r1_cb(self, msg):
        self.r1 = msg

    def state_cb(self, msg):
        self.state = msg.data

    def success_cb(self, msg):
        if msg.data:
            self.success = True

    def sim_time(self):
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now
            return 0.0
        return (now - self.start_time).nanoseconds * 1e-9

    def sample(self):
        if self.r1 is None:
            return

        t = self.sim_time()
        self.writer.writerow([
            t,
            float(self.r1.x), float(self.r1.y), float(self.r1.body_yaw),
            float(self.r1.vx), float(self.r1.vy), float(self.r1.speed),
            self.state,
            self.target_speed, self.speed_error,
            self.desired_heading, self.heading_error,
            self.cross_track, self.lookahead,
            self.speed_limit, self.curvature,
            self.distance_to_turn, self.turn_severity,
            self.left_thrust, self.right_thrust,
        ])
        self.file.flush()
        self.rows += 1

        self.state_first.setdefault(self.state, t)
        self.state_last[self.state] = t
        self.max_speed_seen = max(self.max_speed_seen, float(self.r1.speed))

        if self.state == 'TRACK':
            if math.isfinite(self.cross_track):
                self.track_cross.append(self.cross_track)
            if math.isfinite(self.heading_error):
                self.track_heading.append(self.heading_error)
            if math.isfinite(self.speed_error):
                self.track_speed_error.append(self.speed_error)
            self.track_speed.append(float(self.r1.speed))

    @staticmethod
    def rmse(values):
        if not values:
            return None
        return math.sqrt(sum(v * v for v in values) / len(values))

    def write_summary(self):
        summary = {
            'success': bool(self.success),
            'samples': self.rows,
            'state_first_time_s': self.state_first,
            'state_last_time_s': self.state_last,
            'max_speed_mps': self.max_speed_seen,
            'track': {
                'samples': len(self.track_speed),
                'mean_speed_mps': (
                    sum(self.track_speed) / len(self.track_speed)
                    if self.track_speed else None
                ),
                'cross_track_rmse_m': self.rmse(self.track_cross),
                'maximum_abs_cross_track_error_m': (
                    max(abs(v) for v in self.track_cross)
                    if self.track_cross else None
                ),
                'heading_error_rmse_deg': (
                    math.degrees(self.rmse(self.track_heading))
                    if self.track_heading else None
                ),
                'speed_error_rmse_mps': self.rmse(self.track_speed_error),
            },
        }
        self.summary_path.write_text(json.dumps(summary, indent=2))

    def destroy_node(self):
        try:
            self.write_summary()
            if not self.file.closed:
                self.file.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = R1StressLogger()
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
