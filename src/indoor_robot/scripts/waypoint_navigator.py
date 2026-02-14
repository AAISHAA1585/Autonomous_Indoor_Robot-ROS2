#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self._action_client = ActionClient(
            self, 
            NavigateThroughPoses, 
            'navigate_through_poses'
        )
        
        # Define waypoints (x, y, orientation_z, orientation_w)
        self.waypoints = [
            (-5.0, 5.0, 0.0, 1.0),   # Top left
            (5.0, 5.0, 0.707, 0.707), # Top right
            (5.0, -5.0, 1.0, 0.0),    # Bottom right
            (-5.0, -5.0, -0.707, 0.707) # Bottom left
        ]
        
        self.get_logger().info('Waypoint Navigator Started')
        self.execute_waypoints()
    
    def execute_waypoints(self):
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = []
        
        for i, (x, y, oz, ow) in enumerate(self.waypoints):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = oz
            pose.pose.orientation.w = ow
            goal_msg.poses.append(pose)
            self.get_logger().info(f'Waypoint {i+1}: ({x}, {y})')
        
        self._action_client.wait_for_server()
        self.get_logger().info('Sending waypoints to robot...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Waypoints rejected!')
            return
        
        self.get_logger().info('Waypoints accepted! Robot navigating...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Waypoint navigation complete!')
        rclpy.shutdown()

def main():
    rclpy.init()
    navigator = WaypointNavigator()
    rclpy.spin(navigator)

if __name__ == '__main__':
    main()
