import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from tf2_ros import Buffer, TransformListener


class LidarCostmapNode(Node):

    def __init__(self):
        super().__init__('lidar_costmap_node')

        self.map_size = 200
        self.resolution = 0.05
        self.center = self.map_size // 2
        self.inflation_radius = 0.3

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.lidar_map = np.zeros((self.map_size, self.map_size), dtype=np.int8)

        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.lidar_pub = self.create_publisher(OccupancyGrid, '/lidar_map', 10)

        self.get_logger().info('LidarCostmapNode started — waiting for scan...')

    def scan_callback(self, msg: LaserScan):
        self.lidar_map.fill(0)

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            x_end = int(self.center + (r * math.cos(angle)) / self.resolution)
            y_end = int(self.center + (r * math.sin(angle)) / self.resolution)
            self.trace_ray(self.lidar_map, self.center, self.center, x_end, y_end)

        self.inflate(self.lidar_map)
        self.publish_grid(self.lidar_map, 'laser')

    def trace_ray(self, grid, x0, y0, x1, y1):
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            if x0 == x1 and y0 == y1:
                if 0 <= x0 < self.map_size and 0 <= y0 < self.map_size:
                    grid[y0, x0] = 100
                break
            if 0 <= x0 < self.map_size and 0 <= y0 < self.map_size:
                grid[y0, x0] = 0
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

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
        self.lidar_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarCostmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()