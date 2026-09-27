import csv
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from nav_msgs.msg import Path as RosPath
from geometry_msgs.msg import PoseStamped


class TrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('trajectory_publisher')

        # QoS:
        # transient_local = 后启动的 subscriber 也可以收到已经发布过的路径
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.publisher_ = self.create_publisher(
            RosPath,
            '/coverage_path',
            qos
        )

        # output 文件夹
        output_dir = Path(
            '~/vrx_ws/src/planner/planner/ky_py_pkg/output'
        ).expanduser()

        # 声明参数：默认文件名
        self.declare_parameter('csv_file', 'sydney_coverage_path.csv')
        csv_file = self.get_parameter('csv_file').value

        # 如果传进来的是相对文件名，就默认去 output 里找
        self.csv_path = Path(csv_file)
        if not self.csv_path.is_absolute():
            self.csv_path = output_dir / csv_file

        # 读取 CSV
        self.path_msg = self.load_csv()

        # 稍微等一下 ROS publisher 初始化，然后发布
        self.timer = self.create_timer(1.0, self.publish_path)

        self.published = False

    def load_csv(self):

        path_msg = RosPath()

        # Gazebo world 坐标通常可以先使用 world
        path_msg.header.frame_id = 'world'

        if not self.csv_path.exists():
            self.get_logger().error(
                f'CSV file not found: {self.csv_path}'
            )
            return path_msg

        self.get_logger().info(
            f'Reading trajectory from: {self.csv_path}'
        )

        with open(self.csv_path, 'r') as csvfile:

            reader = csv.reader(csvfile)

            for row in reader:

                # 跳过空行
                if len(row) < 2:
                    continue

                try:
                    x = float(row[0])
                    y = float(row[1])

                # 如果第一行是 x,y 这种 header，会自动跳过
                except ValueError:
                    continue

                pose = PoseStamped()

                pose.header.frame_id = 'world'

                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0

                # 暂时不考虑 waypoint 的朝向
                # 合法 quaternion: yaw = 0
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = 0.0
                pose.pose.orientation.w = 1.0

                path_msg.poses.append(pose)

        self.get_logger().info(
            f'Loaded {len(path_msg.poses)} waypoints'
        )

        return path_msg

    def publish_path(self):

        # 只发布一次
        if self.published:
            return

        if len(self.path_msg.poses) == 0:
            self.get_logger().error(
                'No waypoints available. Path not published.'
            )
            self.published = True
            return

        now = self.get_clock().now().to_msg()

        self.path_msg.header.stamp = now

        for pose in self.path_msg.poses:
            pose.header.stamp = now

        self.publisher_.publish(self.path_msg)

        self.get_logger().info(
            f'Published trajectory with '
            f'{len(self.path_msg.poses)} waypoints '
            f'on /coverage_path'
        )

        self.published = True


def main(args=None):

    rclpy.init(args=args)

    node = TrajectoryPublisher()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()