#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        
        self.speed = 0.
        
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,  # message type
            '/drive',               # topic name
            10                      # queue size
        )

        self.scan_sub = self.create_subscription(
            LaserScan,              # message type
            '/scan',                # topic name
            self.scan_callback,     # callback function
            10                      # queue size
        )

        self.odom_sub = self.create_subscription(
            Odometry,               # message type
            '/ego_racecar/odom',    # topic name
            self.odom_callback,     # callback function
            10                      # queue size
        )
       
     

    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
    ranges = np.array(scan_msg.ranges)
    angles = np.arange(scan_msg.angle_min,
                       scan_msg.angle_max,
                       scan_msg.angle_increment)
    v_proj = self.speed * np.cos(angles)
    ttc = ranges / np.maximum(v_proj, 1e-6)
    min_ttc = np.min(ttc)
    
    if min_ttc < 1.0:
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 0.0
        self.drive_pub.publish(drive_msg)
        self.get_logger().info("⚠️ Emergency brake! TTC: {:.2f}s".format(min_ttc))


def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
