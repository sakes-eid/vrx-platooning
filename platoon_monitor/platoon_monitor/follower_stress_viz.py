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


class FollowerStressViz(Node):

    def __init__(self):
        super().__init__('follower_stress_viz')

        self.declare_parameter('vehicle_id', 'r2')
        self.declare_parameter('marker_r', 0.10)
        self.declare_parameter('marker_g', 0.65)
        self.declare_parameter('marker_b', 1.00)

        self.vehicle_id = str(
            self.get_parameter('vehicle_id').value
        )

        self.marker_r = float(
            self.get_parameter('marker_r').value
        )
        self.marker_g = float(
            self.get_parameter('marker_g').value
        )
        self.marker_b = float(
            self.get_parameter('marker_b').value
        )

        vid = self.vehicle_id

        self.vehicle = None
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
            VehicleState,
            f'/{vid}/vehicle_state',
            self.vehicle_callback,
            20,
        )

        self.create_subscription(
            Float64,
            f'/planner/{vid}/speed_limit',
            lambda m: setattr(self, 'speed_limit', float(m.data)),
            10,
        )

        self.create_subscription(
            Float64,
            f'/planner/{vid}/lookahead_distance',
            lambda m: setattr(self, 'lookahead', float(m.data)),
            10,
        )

        self.create_subscription(
            Float64,
            f'/planner/{vid}/distance_to_turn',
            lambda m: setattr(
                self,
                'distance_to_turn',
                float(m.data),
            ),
            10,
        )

        self.create_subscription(
            String,
            f'/planner/{vid}/turn_severity',
            lambda m: setattr(self, 'turn_severity', m.data),
            10,
        )

        self.create_subscription(
            String,
            f'/{vid}/control/state',
            lambda m: setattr(
                self,
                'controller_state',
                m.data,
            ),
            10,
        )

        self.create_subscription(
            Int32,
            f'/{vid}/control/active_waypoint_number',
            lambda m: setattr(
                self,
                'active_waypoint_number',
                int(m.data),
            ),
            10,
        )

        self.create_subscription(
            Int32,
            f'/planner/{vid}/waypoint_count',
            lambda m: setattr(
                self,
                'waypoint_count',
                int(m.data),
            ),
            qos,
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            f'/viz/{vid}/markers',
            10,
        )

        self.trail_pub = self.create_publisher(
            Path,
            f'/viz/{vid}/trail',
            qos,
        )

        self.last_trail_point = None
        self.timer = self.create_timer(
            0.10,
            self.publish_viz,
        )

        self.get_logger().info(
            f'Follower visualization ready for {vid}.'
        )

    def vehicle_callback(self, msg):
        self.vehicle = msg

    def publish_viz(self):

        if self.vehicle is None:
            return

        now = self.get_clock().now().to_msg()

        x = float(self.vehicle.x)
        y = float(self.vehicle.y)
        yaw = float(self.vehicle.body_yaw)

        if (
            self.last_trail_point is None
            or math.hypot(
                x - self.last_trail_point[0],
                y - self.last_trail_point[1],
            ) >= 0.20
        ):
            p = PoseStamped()
            p.header.frame_id = 'world_ned'
            p.header.stamp = now
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.orientation.w = 1.0

            self.trail.poses.append(p)
            self.last_trail_point = (x, y)

            if len(self.trail.poses) > 5000:
                self.trail.poses = self.trail.poses[-5000:]

        self.trail.header.stamp = now
        self.trail_pub.publish(self.trail)

        markers = MarkerArray()

        hull = Marker()
        hull.header.frame_id = 'world_ned'
        hull.header.stamp = now
        hull.ns = self.vehicle_id
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

        hull.color.r = self.marker_r
        hull.color.g = self.marker_g
        hull.color.b = self.marker_b
        hull.color.a = 0.95

        markers.markers.append(hull)

        label = Marker()
        label.header.frame_id = 'world_ned'
        label.header.stamp = now
        label.ns = self.vehicle_id
        label.id = 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD

        label.pose.position.x = x
        label.pose.position.y = y
        label.pose.position.z = 3.0

        label.scale.z = 1.8

        label.color.r = self.marker_r
        label.color.g = self.marker_g
        label.color.b = self.marker_b
        label.color.a = 1.0

        label.text = (
            f'{self.vehicle_id.upper()}  '
            f'{self.controller_state}\n'
            f'v={float(self.vehicle.speed):.2f} m/s'
        )

        markers.markers.append(label)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)

    node = FollowerStressViz()

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
