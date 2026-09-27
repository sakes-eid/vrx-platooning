#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float64, Int32, String
from visualization_msgs.msg import Marker, MarkerArray

from platoon_interfaces.msg import VehicleState


class R1StressViz(Node):
    def __init__(self):
        super().__init__('r1_stress_viz')

        self.r1 = None
        self.speed_limit = float('nan')
        self.lookahead = float('nan')
        self.distance_to_turn = float('nan')
        self.turn_severity = 'UNKNOWN'
        self.controller_state = 'UNKNOWN'
        self.active_waypoint_number = -1
        self.waypoint_count = -1

        self.trail = Path()
        self.trail.header.frame_id = 'world_ned'

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            VehicleState, '/r1/vehicle_state', self.r1_callback, 20
        )
        self.create_subscription(
            Float64, '/planner/r1/speed_limit', self.speed_callback, 10
        )
        self.create_subscription(
            Float64, '/planner/r1/lookahead_distance', self.lookahead_callback, 10
        )
        self.create_subscription(
            Float64, '/planner/r1/distance_to_turn', self.distance_callback, 10
        )
        self.create_subscription(
            String, '/planner/r1/turn_severity', self.turn_callback, 10
        )
        self.create_subscription(
            String, '/r1/control/state', self.state_callback, 10
        )
        self.create_subscription(
            Int32, '/r1/control/active_waypoint_number', self.active_waypoint_callback, 10
        )
        self.create_subscription(
            Int32, '/planner/r1/waypoint_count', self.waypoint_count_callback, qos
        )

        self.marker_pub = self.create_publisher(
            MarkerArray, '/viz/r1/markers', 10
        )
        self.trail_pub = self.create_publisher(
            Path, '/viz/r1/trail', qos
        )

        self.last_trail_point = None
        self.timer = self.create_timer(0.10, self.publish_viz)

    def r1_callback(self, msg):
        self.r1 = msg

    def speed_callback(self, msg):
        self.speed_limit = float(msg.data)

    def lookahead_callback(self, msg):
        self.lookahead = float(msg.data)

    def distance_callback(self, msg):
        self.distance_to_turn = float(msg.data)

    def turn_callback(self, msg):
        self.turn_severity = msg.data

    def state_callback(self, msg):
        self.controller_state = msg.data

    def active_waypoint_callback(self, msg):
        self.active_waypoint_number = int(msg.data)

    def waypoint_count_callback(self, msg):
        self.waypoint_count = int(msg.data)

    def publish_viz(self):
        if self.r1 is None:
            return

        now = self.get_clock().now().to_msg()
        x = float(self.r1.x)
        y = float(self.r1.y)
        yaw = float(self.r1.body_yaw)

        if (
            self.last_trail_point is None
            or math.hypot(x - self.last_trail_point[0], y - self.last_trail_point[1]) >= 0.20
        ):
            p = PoseStamped()
            p.header.frame_id = 'world_ned'
            p.header.stamp = now
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.orientation.w = 1.0
            self.trail.poses.append(p)
            self.last_trail_point = (x, y)

            # Keep enough history for the complete stress course without
            # growing forever in repeated experiments.
            if len(self.trail.poses) > 5000:
                self.trail.poses = self.trail.poses[-5000:]

        self.trail.header.stamp = now
        self.trail_pub.publish(self.trail)

        markers = MarkerArray()

        hull = Marker()
        hull.header.frame_id = 'world_ned'
        hull.header.stamp = now
        hull.ns = 'r1'
        hull.id = 0
        hull.type = Marker.ARROW
        hull.action = Marker.ADD
        hull.pose.position.x = x
        hull.pose.position.y = y
        hull.pose.position.z = 0.15
        hull.pose.orientation.z = math.sin(0.5 * yaw)
        hull.pose.orientation.w = math.cos(0.5 * yaw)
        hull.scale.x = 4.0
        hull.scale.y = 1.0
        hull.scale.z = 0.8
        hull.color.r = 0.10
        hull.color.g = 0.55
        hull.color.b = 1.00
        hull.color.a = 0.95
        markers.markers.append(hull)

        text = Marker()
        text.header.frame_id = 'world_ned'
        text.header.stamp = now
        text.ns = 'r1'
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = x
        text.pose.position.y = y
        text.pose.position.z = 3.0
        text.scale.z = 1.8
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0

        dist_text = (
            f'{self.distance_to_turn:.1f} m'
            if math.isfinite(self.distance_to_turn)
            else '--'
        )
        speed_text = (
            f'{self.speed_limit:.2f}'
            if math.isfinite(self.speed_limit)
            else '--'
        )
        lookahead_text = (
            f'{self.lookahead:.2f}'
            if math.isfinite(self.lookahead)
            else '--'
        )
        wp_text = (
            f'{self.active_waypoint_number}/{self.waypoint_count}'
            if self.active_waypoint_number > 0 and self.waypoint_count > 0
            else '--/--'
        )
        text.text = (
            f'R1  {self.controller_state}  WP={wp_text}\n'
            f'v={float(self.r1.speed):.2f}  limit={speed_text} m/s\n'
            f'turn={self.turn_severity} in {dist_text}  L={lookahead_text} m'
        )
        markers.markers.append(text)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = R1StressViz()
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
