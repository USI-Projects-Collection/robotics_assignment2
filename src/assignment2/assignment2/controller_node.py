import rclpy
from rclpy.node import Node
from transforms3d._gohlketransforms import euler_from_quaternion

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

import sys

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
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

        self.state = "drive_forward"  # New: initialize robot state

    def scan_range0_callback(self, msg):
        self.range_0 = msg.range

    def scan_range1_callback(self, msg):
        self.range_1 = msg.range

    def scan_range2_callback(self, msg):
        self.range_2 = msg.range

    def scan_range3_callback(self, msg):
        self.range_3 = msg.range
        
        
        # NOTE: we're using relative names to specify the topics (i.e., without a 
        # leading /). ROS resolves relative names by concatenating them with the 
        # namespace in which this node has been started, thus allowing us to 
        # specify which RoboMaster should be controlled.
        
    def start(self):
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(1/60, self.update_callback)
    
    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)
    
    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_valocity = msg.twist.twist
        
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
        
    def update_callback(self):
        cmd_vel = Twist()

        self.get_logger().info("state: " + self.state)

        # DEBUG: print current state and sensor values
        self.get_logger().info(f"State: {self.state} | Range readings: r0={self.range_0}, r1={self.range_1}, r2={self.range_2}, r3={self.range_3}")

        if self.state == "drive_forward":
            if (self.range_1 is not None and self.range_3 is not None) and (self.range_1 > 0.5 and self.range_3 > 0.5):
                cmd_vel.linear.x = 0.05
                cmd_vel.angular.z = 0.0
            else:
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.state = "align"  # transition to alignment phase

        elif self.state == "align":
            if self.range_1 is not None and self.range_3 is not None:
                diff = self.range_1 - self.range_3
                if abs(diff) < 0.05:  # if approximately aligned
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.0
                    self.get_logger().info("Alignment complete!")
                    self.state = "stop"
                elif diff > 0:
                    cmd_vel.angular.z = 0.1  # rotate left
                else:
                    cmd_vel.angular.z = -0.1  # rotate right

        elif self.state == "stop":
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0

        self.vel_publisher.publish(cmd_vel)


def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)
    
    # Create an instance of your node class
    node = ControllerNode()
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
