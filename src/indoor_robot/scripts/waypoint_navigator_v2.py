#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
import math


class FullMapCoverage(Node):

    def __init__(self):
        super().__init__('full_map_coverage')

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # PRECISE STRUCTURED COVERAGE
        self.waypoints = [

            # Bottom sweep
            (-4.0, -4.0),
            ( 4.0, -4.0),

            # Right wall up
            ( 4.0,  4.0),

            # Top sweep
            (-4.0,  4.0),

            # Left wall down
            (-4.0, -4.0),

            # Left corridor
            (-2.5, -3.0),
            (-2.5,  3.0),

            # Right corridor
            ( 2.5, -4.0),
            ( 2.5,  4.0),

            # Return to center
            (0.0, 0.0)
        ]

        self.current_waypoint = 0
        self.timer = None

        self.get_logger().info(f'Full coverage started: {len(self.waypoints)} waypoints')

        self._action_client.wait_for_server()
        self.send_next_waypoint()

    # --------------------------------------------------

    def compute_orientation(self, x1, y1, x2, y2):
        yaw = math.atan2(y2 - y1, x2 - x1)
        return math.sin(yaw/2.0), math.cos(yaw/2.0)

    # --------------------------------------------------

    def send_next_waypoint(self):

        if self.current_waypoint >= len(self.waypoints):
            self.get_logger().info('MAP COVERAGE COMPLETE')
            rclpy.shutdown()
            return

        x, y = self.waypoints[self.current_waypoint]

        if self.current_waypoint < len(self.waypoints) - 1:
            nx, ny = self.waypoints[self.current_waypoint + 1]
            oz, ow = self.compute_orientation(x, y, nx, ny)
        else:
            oz, ow = 0.0, 1.0

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = oz
        goal.pose.pose.orientation.w = ow

        self.get_logger().info(
            f'Waypoint {self.current_waypoint+1}/{len(self.waypoints)} → ({x:.2f}, {y:.2f})'
        )

        send_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        send_future.add_done_callback(self.goal_response_callback)

    # --------------------------------------------------

    def feedback_callback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        if dist > 0.3:
            self.get_logger().info(
                f'Remaining: {dist:.2f} m',
                throttle_duration_sec=2.0
            )

    # --------------------------------------------------

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected — skipping')
            self.current_waypoint += 1
            self.send_next_waypoint()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    # --------------------------------------------------

    def get_result_callback(self, future):
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Reached')
        else:
            self.get_logger().warn(f'Failed (status: {status})')

        self.current_waypoint += 1
        self.timer = self.create_timer(2.0, self.timer_callback)

    # --------------------------------------------------

    def timer_callback(self):
        self.timer.cancel()
        self.send_next_waypoint()


def main():
    rclpy.init()
    node = FullMapCoverage()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
