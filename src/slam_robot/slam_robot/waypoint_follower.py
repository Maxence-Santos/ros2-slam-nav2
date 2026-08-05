#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import time

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Define some waypoints for the warehouse
        self.waypoints = [
            self.create_pose(2.0, 0.0, 0.0),
            self.create_pose(4.0, 2.0, 1.57),
            self.create_pose(0.0, 2.0, 3.14),
            self.create_pose(0.0, 0.0, -1.57)
        ]

    def create_pose(self, x, y, theta):
        """Helper to create a PoseStamped message."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        # Simple yaw to quaternion conversion for 2D
        pose.pose.orientation.z = theta / 2.0
        pose.pose.orientation.w = 1.0 # Approximated for simplicity, ideally use tf-transformations
        return pose

    def run(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        for idx, wp in enumerate(self.waypoints):
            self.get_logger().info(f'Navigating to waypoint {idx+1}/{len(self.waypoints)}...')
            
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = wp
            wp.header.stamp = self.get_clock().now().to_msg()
            
            send_goal_future = self._action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_goal_future)
            
            goal_handle = send_goal_future.result()
            
            if not goal_handle.accepted:
                self.get_logger().error('Goal rejected :(')
                return

            self.get_logger().info('Goal accepted, waiting for result...')
            
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future)
            
            result = get_result_future.result().result
            self.get_logger().info(f'Arrived at waypoint {idx+1}!')
            time.sleep(2.0) # Pause before next waypoint

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
