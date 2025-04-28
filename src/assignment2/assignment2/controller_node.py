import rclpy
from rclpy.node import Node
from transforms3d._gohlketransforms import euler_from_quaternion
import math

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

import sys

class ControllerNodeWall(Node):
    def __init__(self):
        super().__init__('controller_node_wall')
        
        # Create attributes to store odometry pose and velocity
        self.odom_pose = None
        self.odom_velocity = None
                
        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create a subscriber to the topic 'odom', which will call 
        # self.odom_callback every time a message is received
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # Subscribe to the individual range sensors
        self.front_right_range_sub = self.create_subscription(Range, '/rm0/range_1', self.scan_range1_callback, 10)
        self.front_left_range_sub = self.create_subscription(Range, '/rm0/range_3', self.scan_range3_callback, 10)
        self.back_right_range_sub = self.create_subscription(Range, '/rm0/range_0', self.scan_range0_callback, 10)
        self.back_left_range_sub = self.create_subscription(Range, '/rm0/range_2', self.scan_range2_callback, 10)

        # Add attributes to store sensor readings
        self.range_0 = 10.0
        self.range_1 = 10.0
        self.range_2 = 10.0
        self.range_3 = 10.0

        self.state = "drive_forward"  # Initialize robot state
        self.start_positioning_pose = None  # Attribute for positioning phase
        self.rotation_start_time = None
        self.rotation_direction = None
        self.rotation_start_yaw = None
        self.rotation_target_yaw = None

    def scan_range0_callback(self, msg):
        self.range_0 = msg.range

    def scan_range1_callback(self, msg):
        self.range_1 = msg.range

    def scan_range2_callback(self, msg):
        self.range_2 = msg.range

    def scan_range3_callback(self, msg):
        self.range_3 = msg.range
        
    def start(self):
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(1/60, self.update_callback)
    
    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)
    
    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_velocity = msg.twist.twist
        
        pose2d = self.pose3d_to_2d(self.odom_pose)
        
        self.get_logger().info(
            "odometry: received pose (x: {:.2f}, y: {:.2f}, theta: {:.2f})".format(*pose2d),
             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        )
    
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
        
    def update_callback(self):
        cmd_vel = Twist()

        self.get_logger().info(f"State: {self.state} | Range readings: r0={self.range_0}, r1={self.range_1}, r2={self.range_2}, r3={self.range_3}")

        if self.state == "drive_forward":
            if (self.range_1 is not None and self.range_3 is not None) and (self.range_1 > 0.2 and self.range_3 > 0.2):
                cmd_vel.linear.x = 0.05
                cmd_vel.angular.z = 0.0
            else:
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.state = "align"

        elif self.state == "align":
            if self.range_1 is not None and self.range_3 is not None:
                diff = self.range_1 - self.range_3
                if abs(diff) < 0.10:
                    self.get_logger().info("Alignment complete! Waiting 1 second before rotating...")
                    self.start_positioning_pose = self.pose3d_to_2d(self.odom_pose)
                    self.rotation_start_yaw = self.start_positioning_pose[2]
                    # Calcola l'obiettivo ruotando di 180 gradi (pi radianti)
                    self.rotation_target_yaw = self.normalize_angle(self.rotation_start_yaw + math.pi)
                    self.rotation_start_time = self.get_clock().now().seconds_nanoseconds()[0]
                    self.state = "wait_before_rotate"
                elif diff > 0:
                    cmd_vel.angular.z = 0.1
                else:
                    cmd_vel.angular.z = -0.1

        elif self.state == "wait_before_rotate":
            current_time = self.get_clock().now().seconds_nanoseconds()[0]
            if current_time - self.rotation_start_time >= 1:
                self.state = "rotate_180"

        elif self.state == "rotate_180":
            if self.odom_pose is not None:
                current_pose = self.pose3d_to_2d(self.odom_pose)
                current_yaw = current_pose[2]
                
                # Calcola la differenza tra l'angolo corrente e l'obiettivo
                angle_diff = self.normalize_angle(current_yaw - self.rotation_target_yaw)
                
                self.get_logger().info(f"Rotating: current_yaw={current_yaw:.2f}, target_yaw={self.rotation_target_yaw:.2f}, diff={angle_diff:.2f}")
                
                # Quando la differenza di angolo è vicina a zero, abbiamo completato la rotazione
                if abs(angle_diff) < 0.1:  # Tolleranza di 0.1 radianti (circa 5.7 gradi)
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.0
                    self.get_logger().info("Rotation complete!")
                    self.start_positioning_pose = self.pose3d_to_2d(self.odom_pose)
                    self.state = "positioning"
                else:
                    cmd_vel.linear.x = 0.0
                    # Rotazione a velocità costante
                    cmd_vel.angular.z = 0.3
                    
                # Verifica anche il timeout come misura di sicurezza
                current_time = self.get_clock().now().seconds_nanoseconds()[0]
                if current_time - self.rotation_start_time > 10:  # 10 secondi di timeout per la rotazione
                    self.get_logger().info("Rotation timeout reached!")
                    cmd_vel.angular.z = 0.0
                    self.start_positioning_pose = self.pose3d_to_2d(self.odom_pose)
                    self.state = "positioning"

        elif self.state == "positioning":
            if self.odom_pose is not None and self.start_positioning_pose is not None:
                current_pose = self.pose3d_to_2d(self.odom_pose)
                distance_moved = ((current_pose[0] - self.start_positioning_pose[0])**2 + (current_pose[1] - self.start_positioning_pose[1])**2)**0.5
                if distance_moved < 2.0:
                    cmd_vel.linear.x = 0.05  # Move forward
                    cmd_vel.angular.z = 0.0
                else:
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.0
                    self.get_logger().info("Reached 2 meters away!")
                    self.state = "stop"

        elif self.state == "stop":
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0

        self.vel_publisher.publish(cmd_vel)


def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)
    
    # Create an instance of your node class
    node = ControllerNodeWall()
    node.start()
    
    # Keep processings events until someone manually shuts down the node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Ensure the RoboMaster is stopped before exiting
    node.stop()


if __name__ == '__main__':
    main()