import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
from tf2_ros import Buffer, TransformListener


class MergerNode(Node):

    def __init__(self):
        super().__init__('merger_node')

        self.map_size = 200
        self.resolution = 0.05
        self.center = self.map_size // 2

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_lidar = None
        self.latest_camera = None

        self.lidar_sub = self.create_subscription(
            OccupancyGrid, '/lidar_map', self.lidar_map_callback, 10)

        self.camera_sub = self.create_subscription(
            OccupancyGrid, '/camera_map', self.camera_map_callback, 10)

        self.merged_pub = self.create_publisher(OccupancyGrid, '/merged_map', 10)

        self.get_logger().info('MergerNode started — waiting for lidar_map and camera_map...')

    def lidar_map_callback(self, msg):
        self.latest_lidar = msg
        self.try_merge()

    def camera_map_callback(self, msg):
        self.latest_camera = msg
        self.try_merge()

    def try_merge(self):
        if self.latest_lidar is None or self.latest_camera is None:
            return

        lidar_data = np.array(self.latest_lidar.data, dtype=np.int8)
        camera_data = np.array(self.latest_camera.data, dtype=np.int8)
        merged = np.maximum(lidar_data, camera_data)

        self.publish_grid(merged.reshape(self.map_size, self.map_size))

    def publish_grid(self, grid):
        try:
            t = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            robot_x = t.transform.translation.x
            robot_y = t.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.info.resolution = self.resolution
        msg.info.width = self.map_size
        msg.info.height = self.map_size
        msg.info.origin.position.x = robot_x - (self.center * self.resolution)
        msg.info.origin.position.y = robot_y - (self.center * self.resolution)
        msg.info.origin.position.z = 0.0
        msg.data = grid.flatten().tolist()
        self.merged_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MergerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()