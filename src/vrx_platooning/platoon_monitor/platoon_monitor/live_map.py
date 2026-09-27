import os

import matplotlib.pyplot as plt
import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from platoon_interfaces.msg import VehicleState


class LiveMap(Node):

    def __init__(self):
        super().__init__('live_map')

        self.declare_parameter(
            'refresh_period',
            5.0
        )

        self.declare_parameter(
            'path_topic',
            '/planner/reference_path'
        )

        self.declare_parameter(
            'robot_topics',
            ['/wamv/state/vehicle']
        )

        self.refresh_period = float(
            self.get_parameter(
                'refresh_period'
            ).value
        )

        self.path_topic = self.get_parameter(
            'path_topic'
        ).value

        self.robot_topics = self.get_parameter(
            'robot_topics'
        ).value

        # ---------------------------------------------------------
        # Load Sydney occupancy map
        #
        # Original occupancy map is Gazebo ENU:
        # horizontal = East
        # vertical   = North
        #
        # Our project is WORLD NED:
        # x = North
        # y = East
        # ---------------------------------------------------------

        package_dir = os.path.dirname(
            os.path.dirname(
                os.path.abspath(__file__)
            )
        )

        occupancy_file = os.path.join(
            package_dir,
            'data',
            'sydney_local_occupancy.npy'
        )

        self.occupancy = np.load(
            occupancy_file
        )

        # Original ENU map limits
        self.enu_east_min = -700.0
        self.enu_east_max = -300.0

        self.enu_north_min = 100.0
        self.enu_north_max = 400.0

        # Transpose so plot axes become:
        # horizontal = North
        # vertical   = East
        self.occupancy_ned = (
            self.occupancy.T
        )

        self.north_min = (
            self.enu_north_min
        )

        self.north_max = (
            self.enu_north_max
        )

        self.east_min = (
            self.enu_east_min
        )

        self.east_max = (
            self.enu_east_max
        )

        # ---------------------------------------------------------
        # Planner path
        # ---------------------------------------------------------

        self.path_x = []
        self.path_y = []

        self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            10
        )

        # ---------------------------------------------------------
        # Robot states
        # ---------------------------------------------------------

        self.robot_states = {}
        self.robot_trails = {}

        for index, topic in enumerate(
            self.robot_topics
        ):

            robot_name = (
                f'Robot {index + 1}'
            )

            self.robot_states[
                robot_name
            ] = None

            self.robot_trails[
                robot_name
            ] = []

            self.create_subscription(
                VehicleState,
                topic,
                lambda msg,
                name=robot_name:
                    self.robot_callback(
                        msg,
                        name
                    ),
                10
            )

            self.get_logger().info(
                f'{robot_name}: {topic}'
            )

        # ---------------------------------------------------------
        # GUI
        # ---------------------------------------------------------

        plt.ion()

        self.figure, self.ax = (
            plt.subplots(
                figsize=(10, 8)
            )
        )

        # GUI redraw only every 5 s by default.
        # ROS subscriptions still update continuously.
        self.create_timer(
            self.refresh_period,
            self.refresh_map
        )

        self.get_logger().info(
            'Live map started in world_ned'
        )

        self.get_logger().info(
            f'Map refresh every '
            f'{self.refresh_period:.1f} s'
        )

    # -------------------------------------------------------------
    # Planner callback
    # -------------------------------------------------------------

    def path_callback(
        self,
        msg
    ):

        if (
            msg.header.frame_id
            != 'world_ned'
        ):

            self.get_logger().warning(
                f'Path frame is '
                f'{msg.header.frame_id}, '
                f'expected world_ned'
            )

        self.path_x = [
            pose.pose.position.x
            for pose in msg.poses
        ]

        self.path_y = [
            pose.pose.position.y
            for pose in msg.poses
        ]

    # -------------------------------------------------------------
    # Robot callback
    # -------------------------------------------------------------

    def robot_callback(
        self,
        msg,
        robot_name
    ):

        self.robot_states[
            robot_name
        ] = msg

    # -------------------------------------------------------------
    # Map refresh
    # -------------------------------------------------------------

    def refresh_map(self):

        self.ax.clear()

        # ---------------------------------------------------------
        # Sydney shoreline / river
        # ---------------------------------------------------------

        self.ax.imshow(
            self.occupancy_ned,
            origin='lower',
            extent=[
                self.north_min,
                self.north_max,
                self.east_min,
                self.east_max
            ],
            cmap='gray_r',
            interpolation='nearest',
            aspect='equal'
        )

        # ---------------------------------------------------------
        # Reference path
        # ---------------------------------------------------------

        if self.path_x:

            self.ax.plot(
                self.path_x,
                self.path_y,
                '-',
                linewidth=2,
                label='Reference path'
            )

            self.ax.scatter(
                self.path_x,
                self.path_y,
                s=10
            )

        # ---------------------------------------------------------
        # Robots and trails
        # ---------------------------------------------------------

        for (
            robot_name,
            state
        ) in self.robot_states.items():

            if state is None:
                continue

            north = state.x
            east = state.y

            self.robot_trails[
                robot_name
            ].append(
                (
                    north,
                    east
                )
            )

            trail = self.robot_trails[
                robot_name
            ]

            if len(trail) > 1000:

                trail = trail[-1000:]

                self.robot_trails[
                    robot_name
                ] = trail

            trail_north = [
                point[0]
                for point in trail
            ]

            trail_east = [
                point[1]
                for point in trail
            ]

            self.ax.plot(
                trail_north,
                trail_east,
                '--',
                linewidth=1,
                label=(
                    f'{robot_name} trail'
                )
            )

            self.ax.scatter(
                north,
                east,
                s=90,
                marker='o',
                label=robot_name
            )

            self.ax.annotate(
                robot_name,
                (
                    north,
                    east
                ),
                xytext=(6, 6),
                textcoords='offset points'
            )

        # ---------------------------------------------------------
        # Plot settings
        # ---------------------------------------------------------

        self.ax.set_xlabel(
            'North [m]'
        )

        self.ax.set_ylabel(
            'East [m]'
        )

        self.ax.set_title(
            'Sydney Regatta - '
            'Platoon World NED Map'
        )

        self.ax.set_xlim(
            self.north_min,
            self.north_max
        )

        self.ax.set_ylim(
            self.east_min,
            self.east_max
        )

        self.ax.set_aspect(
            'equal',
            adjustable='box'
        )

        self.ax.grid(
            True,
            alpha=0.3
        )

        self.ax.legend(
            loc='best'
        )

        self.figure.tight_layout()

        self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()


def main(args=None):

    rclpy.init(args=args)

    node = LiveMap()

    try:

        while rclpy.ok():

            rclpy.spin_once(
                node,
                timeout_sec=0.1
            )

            plt.pause(0.01)

            if not plt.fignum_exists(
                node.figure.number
            ):
                break

    except KeyboardInterrupt:
        pass

    plt.close('all')

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
