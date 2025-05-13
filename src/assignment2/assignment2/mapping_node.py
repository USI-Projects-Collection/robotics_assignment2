import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Range
from geometry_msgs.msg import Pose
import numpy as np
import math
import tf_transformations
import tf2_ros
from builtin_interfaces.msg import Time

class MappingNode(Node):
    def __init__(self):
        super().__init__('mapping_node')

        # Map parameters
        self.map_size_m = 10.0  # 10 meters
        self.resolution = 0.1   # meters/cell
        self.width = int(self.map_size_m / self.resolution)
        self.height = int(self.map_size_m / self.resolution)
        self.origin = (-self.map_size_m / 2.0, -self.map_size_m / 2.0)

        self.map = -1 * np.ones((self.height, self.width), dtype=np.int8)  # unknown

        # Publishers
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)

        # Subscriptions
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.sensor_topics = ['/range_0', '/range_1', '/range_2', '/range_3']
        self.sensor_angles = [0.0, math.pi/2, math.pi, 3*math.pi/2]  # in radians

        self.latest_ranges = [None] * 4
        self.latest_pose = None

        for i, topic in enumerate(self.sensor_topics):
            self.create_subscription(Range, topic, self.make_range_callback(i), 10)

        # Timer to update map
        self.create_timer(0.2, self.update_map)  # 5 Hz

    def make_range_callback(self, index):
        def callback(msg):
            self.latest_ranges[index] = msg
        return callback

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        self.latest_pose = (position.x, position.y, yaw)

    def update_map(self):
        if self.latest_pose is None or any(r is None for r in self.latest_ranges):
            return

        x_robot, y_robot, yaw = self.latest_pose
        for i, sensor_msg in enumerate(self.latest_ranges):
            range_val = sensor_msg.range
            if range_val >= sensor_msg.max_range or math.isnan(range_val):
                continue

            angle = yaw + self.sensor_angles[i]
            x_end = x_robot + range_val * math.cos(angle)
            y_end = y_robot + range_val * math.sin(angle)

            self.update_cells(x_robot, y_robot, x_end, y_end)

        self.publish_map()

    def world_to_map(self, x, y):
        mx = int((x - self.origin[0]) / self.resolution)
        my = int((y - self.origin[1]) / self.resolution)
        if 0 <= mx < self.width and 0 <= my < self.height:
            return mx, my
        return None, None

    def update_cells(self, x0, y0, x1, y1):
        # Convert to map coordinates
        mx0, my0 = self.world_to_map(x0, y0)
        mx1, my1 = self.world_to_map(x1, y1)
        if None in [mx0, my0, mx1, my1]:
            return

        # Bresenham's line algorithm to mark free cells
        points = self.bresenham(mx0, my0, mx1, my1)
        for px, py in points[:-1]:  # all except last are free
            self.map[py, px] = 0
        # Last point is occupied
        if 0 <= points[-1][0] < self.width and 0 <= points[-1][1] < self.height:
            self.map[points[-1][1], points[-1][0]] = 100

    def publish_map(self):
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = self.get_clock().now().to_msg()

        grid.info.resolution = self.resolution
        grid.info.width = self.width
        grid.info.height = self.height

        grid.info.origin.position.x = self.origin[0]
        grid.info.origin.position.y = self.origin[1]
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0  # no rotation

        grid.data = self.map.flatten().tolist()
        self.map_pub.publish(grid)

    def bresenham(self, x0, y0, x1, y1):
        """ Bresenham's line algorithm """
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1

        if dx > dy:
            err = dx / 2.0
            while x != x1:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        points.append((x1, y1))
        return points

def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()