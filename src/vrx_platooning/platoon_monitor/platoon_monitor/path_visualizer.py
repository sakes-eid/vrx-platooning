import rclpy

from rclpy.node import Node
from nav_msgs.msg import Path

import matplotlib.pyplot as plt


class PathVisualizer(Node):

    def __init__(self):
        super().__init__('path_visualizer')

        # ---------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------

        self.declare_parameter(
            'path_topic',
            '/planner/reference_path'
        )

        self.declare_parameter(
            'refresh_rate',
            10.0
        )

        self.path_topic = self.get_parameter(
            'path_topic'
        ).value

        self.refresh_rate = self.get_parameter(
            'refresh_rate'
        ).value

        # ---------------------------------------------------------
        # Path storage
        # ---------------------------------------------------------

        self.path_x = []
        self.path_y = []

        self.path_received = False

        # ---------------------------------------------------------
        # ROS subscription
        # ---------------------------------------------------------

        self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            10
        )

        # ---------------------------------------------------------
        # Matplotlib setup
        # ---------------------------------------------------------

        plt.ion()

        self.figure, self.ax = plt.subplots()

        self.path_line, = self.ax.plot(
            [],
            [],
            '-o',
            markersize=3,
            linewidth=1.5,
            label='Reference trajectory'
        )

        self.ax.set_title(
            'Platoon Reference Trajectory'
        )

        self.ax.set_xlabel(
            'X - East [m]'
        )

        self.ax.set_ylabel(
            'Y - North [m]'
        )

        self.ax.grid(True)

        self.ax.set_aspect(
            'equal',
            adjustable='box'
        )

        self.ax.legend()

        self.figure.tight_layout()

        self.get_logger().info(
            f'Path visualizer listening to '
            f'{self.path_topic}'
        )

    # -------------------------------------------------------------
    # Path callback
    # -------------------------------------------------------------

    def path_callback(self, msg):

        self.path_x = [
            pose.pose.position.x
            for pose in msg.poses
        ]

        self.path_y = [
            pose.pose.position.y
            for pose in msg.poses
        ]

        self.path_received = True

        self.get_logger().info(
            f'Received path with '
            f'{len(msg.poses)} waypoints',
            once=True
        )

    # -------------------------------------------------------------
    # Plot update
    # -------------------------------------------------------------

    def update_plot(self):

        if not self.path_received:
            return

        self.path_line.set_data(
            self.path_x,
            self.path_y
        )

        self.ax.relim()
        self.ax.autoscale_view()

        # Give the trajectory some visual margin.
        if self.path_x and self.path_y:

            x_min = min(self.path_x)
            x_max = max(self.path_x)

            y_min = min(self.path_y)
            y_max = max(self.path_y)

            x_range = max(
                x_max - x_min,
                1.0
            )

            y_range = max(
                y_max - y_min,
                1.0
            )

            x_margin = 0.10 * x_range
            y_margin = 0.10 * y_range

            self.ax.set_xlim(
                x_min - x_margin,
                x_max + x_margin
            )

            self.ax.set_ylim(
                y_min - y_margin,
                y_max + y_margin
            )

        self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()


def main(args=None):

    rclpy.init(args=args)

    node = PathVisualizer()

    try:

        while rclpy.ok():

            rclpy.spin_once(
                node,
                timeout_sec=0.05
            )

            node.update_plot()

            plt.pause(
                1.0 / node.refresh_rate
            )

            # Stop cleanly if user closes the plot window.
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
