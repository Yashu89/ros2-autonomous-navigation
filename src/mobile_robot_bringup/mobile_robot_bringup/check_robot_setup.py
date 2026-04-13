#!/usr/bin/env python3
"""
Diagnostic script to check robot setup
Run this after launching your robot to verify everything is working
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
import subprocess
import time


class RobotDiagnostics(Node):
    def __init__(self):
        super().__init__('check_robot_setup')
        
        self.laser_received = False
        self.odom_received = False
        self.map_received = False
        
        # Subscribe to topics
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        
        self.timer = self.create_timer(5.0, self.print_diagnostics)
        
        self.get_logger().info("Diagnostics node started. Checking robot setup...")
    
    def laser_callback(self, msg):
        if not self.laser_received:
            self.get_logger().info(f"✓ LaserScan received! Frame: {msg.header.frame_id}")
            self.laser_received = True
    
    def odom_callback(self, msg):
        if not self.odom_received:
            self.get_logger().info(f"✓ Odometry received! Frame: {msg.header.frame_id} -> {msg.child_frame_id}")
            self.odom_received = True
    
    def map_callback(self, msg):
        if not self.map_received:
            self.get_logger().info(f"✓ Map received! Frame: {msg.header.frame_id}")
            self.map_received = True
    
    def print_diagnostics(self):
        self.get_logger().info("\n" + "="*50)
        self.get_logger().info("DIAGNOSTICS SUMMARY:")
        self.get_logger().info(f"LaserScan (/scan): {'✓ OK' if self.laser_received else '✗ NOT RECEIVED'}")
        self.get_logger().info(f"Odometry (/odom): {'✓ OK' if self.odom_received else '✗ NOT RECEIVED'}")
        self.get_logger().info(f"Map (/map): {'✓ OK' if self.map_received else '✗ NOT RECEIVED (Normal if SLAM not started)'}")
        self.get_logger().info("="*50 + "\n")
        
        # Run TF tree check
        self.get_logger().info("Checking TF tree...")
        try:
            result = subprocess.run(['ros2', 'run', 'tf2_tools', 'view_frames',  '--ros-args', '-p', 'duration:=3.0'],
                                    capture_output=True, text=True, timeout=5)
            self.get_logger().info("TF tree check completed. Check frames.pdf for visualization.")
            self.get_logger().info(f"result : {result}")
        except Exception as e:
            self.get_logger().warn(f"Could not generate TF tree: {e}")


def main():
    rclpy.init()
    node = RobotDiagnostics()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
