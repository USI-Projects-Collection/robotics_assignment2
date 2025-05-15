import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Range
import numpy as np
import math
import tf2_ros
from geometry_msgs.msg import Pose, Twist, TransformStamped
from transforms3d._gohlketransforms import euler_from_quaternion, quaternion_from_euler
import time

class MappingNode(Node):
    def __init__(self):
        super().__init__('mapping_node')
        
        # Map parameters - with larger cell size to match RViz2 grid
        self.map_resolution = 0.05   # meters per cell
        self.map_width = 200        # cells (10m x 10m area)
        self.map_height = 200       # cells
        self.map_origin_x = -5.0    # meters
        self.map_origin_y = -5.0    # meters
        
        # Occupancy values (direct probability representation 0-100)
        self.OCCUPIED = 100      # Definitely obstacle
        self.FREE = 0            # Definitely free
        self.UNKNOWN = -1        # Unknown/unexplored
        
        # Create grid map (using direct occupancy values)
        self.grid_map = np.ones((self.map_height, self.map_width), dtype=np.int8) * self.UNKNOWN
        
        # Robot state
        self.pose_2d = None
        
        # Sensor ranges (in meters)
        self.max_sensor_range = 2.0
        self.min_sensor_range = 0.02
        # Range sensors
        self.range_0 = self.max_sensor_range  # back-right
        self.range_1 = self.max_sensor_range  # front-right
        self.range_2 = self.max_sensor_range  # back-left
        self.range_3 = self.max_sensor_range  # front-left
        
        # Sensor positions relative to robot center (x, y, theta)
        self.sensor_poses = {
            'range_0': (-0.1, -0.1, math.pi),    # back-right (pointing backward)
            'range_1': (0.1, -0.1, 0.0),         # front-right (pointing forward)
            'range_2': (-0.1, 0.1, math.pi),     # back-left (pointing backward) 
            'range_3': (0.1, 0.1, 0.0)           # front-left (pointing forward)
        }
        
        # Publishers
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.map_publisher = self.create_publisher(OccupancyGrid, 'map', 10)
        
        # Subscribers
        self.odom_subscriber = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        
        # Range sensor subscribers
        self.front_right_range_sub = self.create_subscription(
            Range, '/rm0/range_1', self.scan_range1_callback, 10)
        self.front_left_range_sub = self.create_subscription(
            Range, '/rm0/range_3', self.scan_range3_callback, 10)
        self.back_right_range_sub = self.create_subscription(
            Range, '/rm0/range_0', self.scan_range0_callback, 10)
        self.back_left_range_sub = self.create_subscription(
            Range, '/rm0/range_2', self.scan_range2_callback, 10)
        
        # TF2 broadcaster for map->odom transform
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Movement control
        self.linear_speed = 0.1  # meters/second
        self.obstacle_threshold = 0.3  # meters - stop when obstacle is this close
        
        self.get_logger().info("Mpping node initialized")

    def scan_range0_callback(self, msg):
        if msg.range >= self.min_sensor_range and msg.range <= self.max_sensor_range:
            self.range_0 = msg.range
        else:
            self.range_0 = self.max_sensor_range
    
    def scan_range1_callback(self, msg):
        if msg.range >= self.min_sensor_range and msg.range <= self.max_sensor_range:
            self.range_1 = msg.range
        else:
            self.range_1 = self.max_sensor_range
    
    def scan_range2_callback(self, msg):
        if msg.range >= self.min_sensor_range and msg.range <= self.max_sensor_range:
            self.range_2 = msg.range
        else:
            self.range_2 = self.max_sensor_range
    
    def scan_range3_callback(self, msg):
        if msg.range >= self.min_sensor_range and msg.range <= self.max_sensor_range:
            self.range_3 = msg.range
        else:
            self.range_3 = self.max_sensor_range

    def start(self):
        while rclpy.ok() and self.pose_2d is None:
            self.get_logger().info("Waiting for initial odometry...")
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(f"Initial pose received: {self.pose_2d}")

        # Set map origin so the robot is at the center of the map
        x_init, y_init, _ = self.pose_2d
        self.map_origin_x = x_init - (self.map_width * self.map_resolution) / 2.0
        self.map_origin_y = y_init - (self.map_height * self.map_resolution) / 2.0

        # Mark the robot's initial position as free
        center_x, center_y = self.world_to_map(x_init, y_init)
        radius = 5  # cells
        
        for i in range(-radius, radius+1):
            for j in range(-radius, radius+1):
                if 0 <= center_y + i < self.map_height and 0 <= center_x + j < self.map_width:
                    self.grid_map[center_y + i, center_x + j] = self.FREE
        
        # Log map resolution and size for debugging
        self.get_logger().info(f"Map size: {self.map_width}x{self.map_height} cells")
        self.get_logger().info(f"Map resolution: {self.map_resolution} meters/cell")
        self.get_logger().info(f"Map dimensions: {self.map_width * self.map_resolution}x{self.map_height * self.map_resolution} meters")
        self.get_logger().info(f"Map origin: ({self.map_origin_x}, {self.map_origin_y}) meters")
                    
        self.timer = self.create_timer(0.05, self.update_callback)  # 20Hz control loop
        self.map_timer = self.create_timer(0.1, self.publish_map)   # 10Hz map publishing
        self.tf_timer = self.create_timer(0.05, self.publish_tf)    # 20Hz TF publishing
        self.get_logger().info("Simple mapping node started!")
        
    def stop(self):
        stop_cmd = Twist()
        self.vel_publisher.publish(stop_cmd)
        self.get_logger().info("Mapping node stopped.")

    def odom_callback(self, msg):
        # Update current pose
        quat = msg.pose.pose.orientation
        position = msg.pose.pose.position
        
        # Convert quaternion to euler angles
        quaternion = (quat.x, quat.y, quat.z, quat.w)
        _, _, yaw = euler_from_quaternion(quaternion)
        
        # Update 2D pose
        self.pose_2d = (position.x, position.y, yaw)
        
        # Debug position information
        self.get_logger().info(f"Robot position: x={position.x:.2f}, y={position.y:.2f}, yaw={yaw:.2f}")
        
        # Convert to map coordinates for debugging
        map_x, map_y = self.world_to_map(position.x, position.y)
        self.get_logger().info(f"Map coordinates: x={map_x}, y={map_y}")

    def normalize_angle(self, angle):
        """Normalize angle to be between [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def world_to_map(self, x_world, y_world):
        """Convert world coordinates to map cell coordinates."""
        x_map = int((x_world - self.map_origin_x) / self.map_resolution)
        y_map = int((y_world - self.map_origin_y) / self.map_resolution)
        
        # Ensure coordinates are within map bounds
        x_map = max(0, min(x_map, self.map_width - 1))
        y_map = max(0, min(y_map, self.map_height - 1))
        
        return x_map, y_map
    
    def update_map_with_range_sensors(self):
        """Update the occupancy grid map using range sensor readings."""
        if self.pose_2d is None:
            return
        
        # Get robot position in world coordinates
        robot_x, robot_y, robot_theta = self.pose_2d
        
        # Convert robot position to map coordinates
        robot_map_x, robot_map_y = self.world_to_map(robot_x, robot_y)
        
        # Mark robot position and surrounding area as free to create more visible path
        cell_radius = 2  # Make a larger area visible around the robot
        for i in range(-cell_radius, cell_radius+1):
            for j in range(-cell_radius, cell_radius+1):
                if (0 <= robot_map_y + i < self.map_height and 
                    0 <= robot_map_x + j < self.map_width):
                    self.grid_map[robot_map_y + i, robot_map_x + j] = self.FREE
        
        # Update map based on each range sensor
        sensor_readings = {
            'range_0': self.range_0,
            'range_1': self.range_1,
            'range_2': self.range_2,
            'range_3': self.range_3
        }
        
        for sensor_name, reading in sensor_readings.items():
            # Get sensor position relative to robot
            sensor_rel_x, sensor_rel_y, sensor_rel_theta = self.sensor_poses[sensor_name]
            
            # Calculate sensor position in world coordinates
            sensor_world_theta = self.normalize_angle(robot_theta + sensor_rel_theta)
            sensor_world_x = robot_x + sensor_rel_x * math.cos(robot_theta) - sensor_rel_y * math.sin(robot_theta)
            sensor_world_y = robot_y + sensor_rel_x * math.sin(robot_theta) + sensor_rel_y * math.cos(robot_theta)
            
            # If sensor detected an obstacle
            if reading < self.max_sensor_range:
                # Calculate obstacle position in world coordinates
                obstacle_world_x = sensor_world_x + reading * math.cos(sensor_world_theta)
                obstacle_world_y = sensor_world_y + reading * math.sin(sensor_world_theta)
                
                # Convert obstacle position to map coordinates
                obstacle_map_x, obstacle_map_y = self.world_to_map(obstacle_world_x, obstacle_world_y)
                
                # Mark obstacle cell and surrounding cells as occupied for better visibility
                for i in range(-1, 2):
                    for j in range(-1, 2):
                        if (0 <= obstacle_map_y + i < self.map_height and 
                            0 <= obstacle_map_x + j < self.map_width):
                            self.grid_map[obstacle_map_y + i, obstacle_map_x + j] = self.OCCUPIED
                
                # Mark cells between sensor and obstacle as free
                self.mark_free_cells(sensor_world_x, sensor_world_y, obstacle_world_x, obstacle_world_y)
            else:
                # If no obstacle detected, mark cells along the ray as free up to max range
                max_range_world_x = sensor_world_x + self.max_sensor_range * math.cos(sensor_world_theta)
                max_range_world_y = sensor_world_y + self.max_sensor_range * math.sin(sensor_world_theta)
                
                # Mark cells along the ray as free
                self.mark_free_cells(sensor_world_x, sensor_world_y, max_range_world_x, max_range_world_y)
    
    def mark_free_cells(self, x1, y1, x2, y2):
        """Mark cells along a line as free using Bresenham's algorithm."""
        # Convert world coordinates to map coordinates
        x1_map, y1_map = self.world_to_map(x1, y1)
        x2_map, y2_map = self.world_to_map(x2, y2)
        
        # Use Bresenham's line algorithm
        cells = self.bresenham_line(x1_map, y1_map, x2_map, y2_map)
        
        # Mark cells as free (excluding the last point which might be an obstacle)
        for i, (x, y) in enumerate(cells):
            # Skip the last cell (potential obstacle)
            if i == len(cells) - 1:
                continue
                
            if 0 <= y < self.map_height and 0 <= x < self.map_width:
                # For thicker lines, mark neighboring cells too
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        nx, ny = x + dx, y + dy
                        if 0 <= ny < self.map_height and 0 <= nx < self.map_width:
                            # Only mark if not already occupied (to prevent flickering)
                            if self.grid_map[ny, nx] != self.OCCUPIED:
                                self.grid_map[ny, nx] = self.FREE
    
    def bresenham_line(self, x0, y0, x1, y1):
        """Bresenham's line algorithm for finding cells in a line."""
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        while True:
            cells.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                if x0 == x1:
                    break
                err -= dy
                x0 += sx
            if e2 < dx:
                if y0 == y1:
                    break
                err += dx
                y0 += sy
                
        return cells
    
    def publish_map(self):
        """Publish the occupancy grid map."""
        # Create OccupancyGrid message
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = "map"
        
        grid_msg.info.resolution = self.map_resolution
        grid_msg.info.width = self.map_width
        grid_msg.info.height = self.map_height
        
        # Set map origin to align with RViz2 grid
        grid_msg.info.origin.position.x = self.map_origin_x
        grid_msg.info.origin.position.y = self.map_origin_y
        grid_msg.info.origin.position.z = 0.0
        
        # Convert orientation to quaternion
        q = quaternion_from_euler(0, 0, 0)
        grid_msg.info.origin.orientation.x = q[0]
        grid_msg.info.origin.orientation.y = q[1]
        grid_msg.info.origin.orientation.z = q[2]
        grid_msg.info.origin.orientation.w = q[3]
        
        # Flatten the numpy array to a list
        grid_msg.data = self.grid_map.flatten().tolist()
        
        # Publish the map
        self.map_publisher.publish(grid_msg)
        self.get_logger().info("Published map")
    
    def publish_tf(self):
        """Publish the TF transform between map and odom frames."""
        # Create transform message
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        
        # Set transform to align map with RViz2 grid
        # This assumes your robot starts at (0,0) in the odom frame
        # and we want that to correspond to the center of our map
        # Shift the map frame so the robot starts at the center
        t.transform.translation.x = self.map_origin_x 
        t.transform.translation.y = self.map_origin_y 
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        # Publish the transform
        self.tf_broadcaster.sendTransform(t)

    def update_callback(self):
        """Main control loop."""
        # Check if we have valid pose
        if self.pose_2d is None:
            self.get_logger().warn("No valid pose received yet. Waiting...")
            return
        
        # Update map with new sensor readings
        self.update_map_with_range_sensors()
        
        # Check if there's an obstacle in front
        obstacle_detected = min(self.range_1, self.range_3) < self.obstacle_threshold
        
        # Create velocity command
        cmd_vel = Twist()
        
        if obstacle_detected:
            # Stop when obstacle detected
            cmd_vel.linear.x = 0.0
            self.get_logger().info("Obstacle detected! Stopping.")
        else:
            # Move forward
            cmd_vel.linear.x = self.linear_speed
            
            # Small angular corrections to keep straight
            if self.range_1 < self.range_3:
                cmd_vel.angular.z = 0.05  # Slight turn left
            elif self.range_3 < self.range_1:
                cmd_vel.angular.z = -0.05  # Slight turn right
        
        # Publish velocity commands
        self.vel_publisher.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    mapping_node = MappingNode()
    mapping_node.start()

    try:
        rclpy.spin(mapping_node)
    except KeyboardInterrupt:
        pass
    finally:
        mapping_node.stop()
        mapping_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()