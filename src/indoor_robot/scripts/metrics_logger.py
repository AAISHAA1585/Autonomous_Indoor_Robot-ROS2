#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math
import time

class MetricsLogger(Node):
    def __init__(self):
        super().__init__('metrics_logger')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        self.start_time = time.time()
        self.total_distance = 0.0
        self.last_pose = None
        self.goals_reached = 0
        
        self.timer = self.create_timer(5.0, self.log_metrics)
        
    def odom_callback(self, msg):
        current_pose = msg.pose.pose.position
        
        if self.last_pose:
            dx = current_pose.x - self.last_pose.x
            dy = current_pose.y - self.last_pose.y
            self.total_distance += math.sqrt(dx*dx + dy*dy)
        
        self.last_pose = current_pose
    
    def goal_callback(self, msg):
        self.goals_reached += 1
    
    def log_metrics(self):
        elapsed = time.time() - self.start_time
        self.get_logger().info(f'=== PERFORMANCE METRICS ===')
        self.get_logger().info(f'Runtime: {elapsed:.1f}s')
        self.get_logger().info(f'Distance Traveled: {self.total_distance:.2f}m')
        self.get_logger().info(f'Goals Reached: {self.goals_reached}')
        if elapsed > 0:
            self.get_logger().info(f'Avg Speed: {self.total_distance/elapsed:.2f}m/s')

def main():
    rclpy.init()
    node = MetricsLogger()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
