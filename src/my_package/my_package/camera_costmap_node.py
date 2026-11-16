import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import math
from tf2_ros import Buffer, TransformListener


class CameraCostmapNode(Node):

    def __init__(self):
        super().__init__('camera_costmap_node')

        self.map_size = 200
        self.resolution = 0.05
        self.center = self.map_size // 2
        self.inflation_radius = 0.3

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.camera_map = np.zeros((self.map_size, self.map_size), dtype=np.int8)

        self.camera_sub = self.create_subscription(
            PointCloud2, '/rosbot/camera_depth/point_cloud', self.pointcloud_callback, 10)

        self.camera_pub = self.create_publisher(OccupancyGrid, '/camera_map', 10)

        self.get_logger().info('CameraCostmapNode started — waiting for point cloud...')

    def pointcloud_callback(self, msg: PointCloud2):
        self.camera_map.fill(0)

        for point in pc2.read_points(msg, field_names=('x', 'y', 'z')):
            x, y, z = point

            if math.isnan(x) or math.isnan(y) or math.isnan(z):
                continue
            if math.isinf(x) or math.isinf(y) or math.isinf(z):
                continue
            if z < -0.1 or z > 0.3:
                continue

            gx = int(self.center + x / self.resolution)
            gy = int(self.center + y / self.resolution)

            if 0 <= gx < self.map_size and 0 <= gy < self.map_size:
                self.camera_map[gy][gx] = 100

        self.inflate(self.camera_map)
        self.publish_grid(self.camera_map, 'camera_depth')

    def inflate(self, grid):
        inflation_cells = int(self.inflation_radius / self.resolution)
        inflated = grid.copy()

        for row in range(self.map_size):
            for col in range(self.map_size):
                if grid[row][col] == 100:
                    for drow in range(-inflation_cells, inflation_cells + 1):
                        for dcol in range(-inflation_cells, inflation_cells + 1):
                            nrow = row + drow
                            ncol = col + dcol
                            if 0 <= nrow < self.map_size and 0 <= ncol < self.map_size:
                                distance = math.sqrt(drow**2 + dcol**2)
                                if distance <= inflation_cells:
                                    cost = int(100 * (1 - distance / inflation_cells))
                                    if cost > inflated[nrow][ncol]:
                                        inflated[nrow][ncol] = cost
        grid[:] = inflated

    def publish_grid(self, grid, sensor_frame):
        try:
            t = self.tf_buffer.lookup_transform('odom', sensor_frame, rclpy.time.Time())
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
        self.camera_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraCostmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()