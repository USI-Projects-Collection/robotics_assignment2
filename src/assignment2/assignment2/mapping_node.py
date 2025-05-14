import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Range
import numpy as np
import math
import tf2_ros
from geometry_msgs.msg import Pose, Twist, TransformStamped
from transforms3d._gohlketransforms import euler_from_quaternion, quaternion_from_euler
from builtin_interfaces.msg import Time
import time

class MappingNode(Node):
    def __init__(self):
        super().__init__('mapping_node')
        
        # Map parameters
        self.map_resolution = 0.05  # meters per cell
        self.map_width = 200        # cells
        self.map_height = 200       # cells
        self.map_origin_x = -5.0    # meters
        self.map_origin_y = -5.0    # meters
        
        # Occupancy grid probabilities (log-odds)
        self.l_occ = 0.85           # Log-odds increase for occupied cells
        self.l_free = -0.4          # Log-odds decrease for free cells
        self.l_0 = 0.0              # Initial log-odds value (p=0.5)
        self.l_min = -2.0           # Minimum log-odds value
        self.l_max = 3.5            # Maximum log-odds value
        
        # Create local grid map (in log-odds form)
        self.grid_map = np.zeros((self.map_height, self.map_width), dtype=float)
        self.grid_map.fill(self.l_0)  # Initialize with prior log odds
        
        # Safety boundary to prevent falls
        self.boundary_size = 40     # Size of the safety boundary in cells
        self.create_safety_boundary()
        
        # Robot state
        self.odom_pose = None
        self.odom_velocity = None
        self.pose_2d = (0.0, 0.0, 0.0)  # x, y, theta
        
        # Sensor ranges (in meters)
        self.max_sensor_range = 2.0
        self.min_sensor_range = 0.02
        self.range_0 = self.max_sensor_range  # back-right
        self.range_1 = self.max_sensor_range  # front-right
        self.range_2 = self.max_sensor_range  # back-left
        self.range_3 = self.max_sensor_range  # front-left
        
        # Sensor positions relative to robot center (x, y, theta)
        self.sensor_poses = {
            'range_0': (-0.1, -0.1, math.pi),      # back-right (pointing backward)
            'range_1': (0.1, -0.1, 0.0),          # front-right (pointing forward)
            'range_2': (-0.1, 0.1, math.pi),      # back-left (pointing backward) 
            'range_3': (0.1, 0.1, 0.0)            # front-left (pointing forward)
        }
        
        # Publishers
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.map_publisher = self.create_publisher(OccupancyGrid, 'local_map', 10)
        
        # Subscribers
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.map_subscriber = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        
        # Range sensor subscribers
        self.front_right_range_sub = self.create_subscription(Range, '/rm0/range_1', self.scan_range1_callback, 10)
        self.front_left_range_sub = self.create_subscription(Range, '/rm0/range_3', self.scan_range3_callback, 10)
        self.back_right_range_sub = self.create_subscription(Range, '/rm0/range_0', self.scan_range0_callback, 10)
        self.back_left_range_sub = self.create_subscription(Range, '/rm0/range_2', self.scan_range2_callback, 10)
        
        # Variables for exploration strategy
        self.last_mapping_time = time.time()
        self.mapping_update_rate = 0.5  # Update map every 0.5 seconds
        self.exploration_state = "forward"
        self.turn_direction = 1  # 1 for left, -1 for right
        self.time_in_state = 0.0
        self.random_turn_probability = 0.2
        
        # Initialize navigation parameters
        self.obstacle_distance_threshold = 0.3  # meters
        self.edge_distance_threshold = 0.5     # meters to keep from map edges
        self.linear_speed = 0.1              # Reduced speed for safety
        self.angular_speed = 0.4              # rad/s
        
        # Add edge detection
        self.last_position = None
        self.possible_edge_detected = False
        self.edge_recovery_time = 0.0

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
        self.timer = self.create_timer(1/20, self.update_callback)  # 20Hz control loop
        self.map_timer = self.create_timer(1/2, self.publish_map)   # 2Hz map publishing
        self.get_logger().info("Mapping node started with edge detection safety enabled.")
        self.last_position = (0.0, 0.0, 0.0)  # Initialize last position

    def stop(self):
        stop_cmd = Twist()
        self.vel_publisher.publish(stop_cmd)
        self.get_logger().info("Mapping node stopped.")

    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_velocity = msg.twist.twist
        self.pose_2d = self.pose3d_to_2d(self.odom_pose)
        
        # Log position periodically
        # self.get_logger().info(
        #     f"Position: x={self.pose_2d[0]:.2f}, y={self.pose_2d[1]:.2f}, yaw={self.pose_2d[2]:.2f}"
        # )

    def map_callback(self, msg):
        # Keep this callback in case we want to merge maps later
        pass

    def pose3d_to_2d(self, pose3):
        quaternion = (
            pose3.orientation.x,
            pose3.orientation.y,
            pose3.orientation.z,
            pose3.orientation.w
        )
        
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        
        pose2 = (
            pose3.position.x,  # x position
            pose3.position.y,  # y position
            yaw                # theta orientation
        )
        
        return pose2
    
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
    
    def create_safety_boundary(self):
        """Create a virtual boundary around the map to prevent falls."""
        # Mark cells around the perimeter as occupied
        boundary = self.boundary_size
        
        # Top and bottom boundaries
        self.grid_map[:boundary, :] = self.l_max  # Top boundary
        self.grid_map[-boundary:, :] = self.l_max  # Bottom boundary
        
        # Left and right boundaries
        self.grid_map[:, :boundary] = self.l_max  # Left boundary
        self.grid_map[:, -boundary:] = self.l_max  # Right boundary
        
        self.get_logger().info("Created safety boundary around the map")
    
    def update_map_with_range_sensors(self):
        """Update the occupancy grid map using range sensor readings."""
        if self.pose_2d is None:
            return
        
        # Get robot position in world coordinates
        robot_x, robot_y, robot_theta = self.pose_2d
        
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
            
            # Convert sensor position to map coordinates
            sensor_map_x, sensor_map_y = self.world_to_map(sensor_world_x, sensor_world_y)
            
            # If sensor detected an obstacle
            if reading < self.max_sensor_range:
                # Calculate obstacle position in world coordinates
                obstacle_world_x = sensor_world_x + reading * math.cos(sensor_world_theta)
                obstacle_world_y = sensor_world_y + reading * math.sin(sensor_world_theta)
                
                # Convert obstacle position to map coordinates
                obstacle_map_x, obstacle_map_y = self.world_to_map(obstacle_world_x, obstacle_world_y)
                
                # Mark obstacle cell as occupied
                if 0 <= obstacle_map_x < self.map_width and 0 <= obstacle_map_y < self.map_height:
                    self.grid_map[obstacle_map_y, obstacle_map_x] = min(
                        self.grid_map[obstacle_map_y, obstacle_map_x] + self.l_occ, 
                        self.l_max
                    )
                
                # Use Bresenham's line algorithm to mark cells between sensor and obstacle as free
                free_cells = self.bresenham_line(sensor_map_x, sensor_map_y, obstacle_map_x, obstacle_map_y)
                for (x, y) in free_cells[:-1]:  # Exclude the last cell (obstacle)
                    if 0 <= x < self.map_width and 0 <= y < self.map_height:
                        self.grid_map[y, x] = max(
                            self.grid_map[y, x] + self.l_free, 
                            self.l_min
                        )
            else:
                # If no obstacle detected, mark cells along the ray as free up to max range
                max_range_world_x = sensor_world_x + self.max_sensor_range * math.cos(sensor_world_theta)
                max_range_world_y = sensor_world_y + self.max_sensor_range * math.sin(sensor_world_theta)
                max_range_map_x, max_range_map_y = self.world_to_map(max_range_world_x, max_range_world_y)
                
                # Use Bresenham's line algorithm
                free_cells = self.bresenham_line(sensor_map_x, sensor_map_y, max_range_map_x, max_range_map_y)
                for (x, y) in free_cells:
                    if 0 <= x < self.map_width and 0 <= y < self.map_height:
                        self.grid_map[y, x] = max(
                            self.grid_map[y, x] + self.l_free, 
                            self.l_min
                        )
    
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
        if self.pose_2d is None:
            return
        
        # Convert log-odds to probabilities and then to occupancy values (0-100)
        prob_map = 1.0 - (1.0 / (1.0 + np.exp(self.grid_map)))
        occupancy_map = (prob_map * 100).astype(np.int8)
        
        # Create OccupancyGrid message
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = "map"
        
        grid_msg.info.resolution = self.map_resolution
        grid_msg.info.width = self.map_width
        grid_msg.info.height = self.map_height
        
        grid_msg.info.origin.position.x = self.map_origin_x
        grid_msg.info.origin.position.y = self.map_origin_y
        grid_msg.info.origin.position.z = 0.0
        
        # Convert flat array orientation to quaternion
        q = quaternion_from_euler(0, 0, 0)
        grid_msg.info.origin.orientation.x = q[0]
        grid_msg.info.origin.orientation.y = q[1]
        grid_msg.info.origin.orientation.z = q[2]
        grid_msg.info.origin.orientation.w = q[3]
        
        # Flatten the numpy array to a list
        grid_msg.data = occupancy_map.flatten().tolist()
        
        # Publish the map
        self.map_publisher.publish(grid_msg)
        self.get_logger().info("Published occupancy grid map")

    def detect_possible_edge(self):
        """Detect if the robot is near the edge of the map."""
        if self.pose_2d is None:
            return False
        
        # Get robot position in map coordinates
        robot_x, robot_y, _ = self.pose_2d
        map_x, map_y = self.world_to_map(robot_x, robot_y)
        
        # Check if the robot is within the safety boundary
        if (map_x < self.boundary_size or 
            map_x >= self.map_width - self.boundary_size or 
            map_y < self.boundary_size or 
            map_y >= self.map_height - self.boundary_size):
            return True
        
        # Log edge detection
        self.get_logger().info("Edge detected! Robot is near the map boundary.")
        return False
    
    def update_exploration_strategy(self, dt):
        """Update the exploration state based on sensor readings and time."""
        # Define minimum safe distances
        front_min = min(self.range_1, self.range_3)  # Front sensors
        back_min = min(self.range_0, self.range_2)   # Back sensors
        right_min = min(self.range_0, self.range_1)  # Right sensors
        left_min = min(self.range_2, self.range_3)   # Left sensors
        
        # Update time in current state
        self.time_in_state += dt
        
        # Check for edge detection
        if self.pose_2d is not None:
            self.possible_edge_detected = self.detect_possible_edge()
            
        # Handle edge recovery if needed
        if self.possible_edge_detected:
            self.edge_recovery_time += dt
            
            if self.edge_recovery_time < 2.0:
                # First back up from the edge
                self.exploration_state = "edge_recovery"
                self.get_logger().warn("Edge detected! Backing away from edge.")
                return self.get_exploration_velocity()
            else:
                # After backing up, reset edge detection and turn away
                self.possible_edge_detected = False
                self.edge_recovery_time = 0.0
                self.exploration_state = "turning"
                self.time_in_state = 0.0
                # Turn away from the edge
                if left_min > right_min:
                    self.turn_direction = 1  # Turn left
                else:
                    self.turn_direction = -1  # Turn right
                
        # State machine for exploration
        if self.exploration_state == "forward":
            # If obstacle ahead or we've been going forward for too long
            if (front_min < self.obstacle_distance_threshold or 
                self.time_in_state > 3.0 or 
                np.random.random() < 0.02):  # Random chance to change direction
                
                # Choose whether to turn or reverse
                if back_min > 0.4 and np.random.random() < 0.2:
                    self.exploration_state = "backward"
                else:
                    # Decide which way to turn based on available space
                    if left_min > right_min:
                        self.turn_direction = 1  # Turn left
                    else:
                        self.turn_direction = -1  # Turn right
                    self.exploration_state = "turning"
                self.time_in_state = 0.0
                
        elif self.exploration_state == "backward":
            # If obstacle behind or we've been reversing for too long
            if (back_min < self.obstacle_distance_threshold or 
                self.time_in_state > 1.5):
                
                # Decide which way to turn based on available space
                if left_min > right_min:
                    self.turn_direction = 1  # Turn left
                else:
                    self.turn_direction = -1  # Turn right
                self.exploration_state = "turning"
                self.time_in_state = 0.0
                
        elif self.exploration_state == "turning":
            # If we've been turning for long enough or front is clear
            if (self.time_in_state > 1.0 and front_min > 0.5):
                self.exploration_state = "forward"
                self.time_in_state = 0.0
        
        elif self.exploration_state == "edge_recovery":
            # This state is handled above in the edge detection section
            pass
                
        # Log state changes
        self.get_logger().info(f"Exploration state: {self.exploration_state}, "
                              f"Front: {front_min:.2f}, Back: {back_min:.2f}, "
                              f"Left: {left_min:.2f}, Right: {right_min:.2f}")
        
        return self.get_exploration_velocity()
    
    def get_exploration_velocity(self):
        """Get velocity commands based on current exploration state."""
        cmd_vel = Twist()
        
        if self.exploration_state == "forward":
            cmd_vel.linear.x = self.linear_speed
            
            # Slight adjustment based on side sensors to avoid walls
            if self.range_1 < self.range_3:  # Right front closer than left front
                cmd_vel.angular.z = 0.2  # Turn slightly left
            elif self.range_3 < self.range_1:  # Left front closer than right front
                cmd_vel.angular.z = -0.2  # Turn slightly right
            
        elif self.exploration_state == "backward":
            cmd_vel.linear.x = -self.linear_speed
            
        elif self.exploration_state == "turning":
            cmd_vel.angular.z = self.turn_direction * self.angular_speed
            
        elif self.exploration_state == "edge_recovery":
            # Back away from the edge
            cmd_vel.linear.x = -self.linear_speed * 0.8
            
            # Add a slight turn to avoid going straight back
            cmd_vel.angular.z = 0.2 * self.turn_direction
            
        return cmd_vel
    
    def update_callback(self):
        """Main control loop."""
        # Check if we have valid pose
        if self.pose_2d is None:
            self.get_logger().warn("No valid pose received yet. Waiting...")
            return
        
        # Log range readings periodically
        # self.get_logger().info(
        #     f"Ranges: front-right={self.range_1:.2f}, front-left={self.range_3:.2f}, " 
        #     f"back-right={self.range_0:.2f}, back-left={self.range_2:.2f}"
        # )
        
        # Update map with new sensor readings
        current_time = time.time()
        if current_time - self.last_mapping_time > self.mapping_update_rate:
            self.update_map_with_range_sensors()
            self.last_mapping_time = current_time
        
        # Calculate time since last update
        dt = 1/20.0  # Assuming 20Hz update rate
        
        # Get velocity commands based on exploration strategy
        cmd_vel = self.update_exploration_strategy(dt)
        
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