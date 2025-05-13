import rclpy
from rclpy.node import Node
from transforms3d._gohlketransforms import euler_from_quaternion

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry

import sys

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        # Create attributes to store odometry pose and velocity
        self.odom_pose = None
        self.odom_velocity = None

        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create a subscriber to the topic 'odom'
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # Internal timer for controlling the movement
        self.time_elapsed = 0.0

    def start(self):
        # Start a timer at 60 Hz
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
            throttle_duration_sec=0.5
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
        # Update the elapsed time
        self.time_elapsed += 1/60  # Timer is at 60Hz

        cmd_vel = Twist()

        cycle_duration = 20.0  # 10s forward + 10s backward
        half_cycle = cycle_duration / 2

        # Reset after full forward-backward cycle
        if self.time_elapsed > cycle_duration:
            self.time_elapsed = 0.0

        # Forward or backward phase
        forward = self.time_elapsed <= half_cycle

        # Time inside current phase
        t = self.time_elapsed if forward else self.time_elapsed - half_cycle

        # Moving FORWARD phase
        if forward:
            if t <= 5.0:
                cmd_vel.linear.x = 0.3
                cmd_vel.angular.z = 0.6  # left circle
            elif t <= 10.0:
                cmd_vel.linear.x = 0.3
                cmd_vel.angular.z = -0.6  # right circle

        # Moving BACKWARD phase
        else:
            if t <= 5.0:
                cmd_vel.linear.x = -0.3
                cmd_vel.angular.z = 0.6  # right circle backward
            elif t <= 10.0:
                cmd_vel.linear.x = -0.3
                cmd_vel.angular.z = -0.6  # left circle backward

        self.vel_publisher.publish(cmd_vel)


def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)
    
    # Create an instance of your node class
    node = ControllerNode()
    node.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Ensure the RoboMaster is stopped before exiting
    node.stop()


if __name__ == '__main__':
    main()