#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import random
import math

class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        
        # Create action client for Nav2
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Wait for action server
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for Nav2 action server...')
        
        # Create timer for sending new goals
        self.timer = self.create_timer(5.0, self.send_new_goal)  # Send new goal every 5 seconds
        
        # Initialize goal parameters
        self.min_x = -5.0  # Minimum x coordinate
        self.max_x = 5.0   # Maximum x coordinate
        self.min_y = -5.0  # Minimum y coordinate
        self.max_y = 5.0   # Maximum y coordinate
        
        self.get_logger().info('Explorer node started')
    
    def send_new_goal(self):
        # Create random goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Random position within bounds
        goal_pose.pose.position.x = random.uniform(self.min_x, self.max_x)
        goal_pose.pose.position.y = random.uniform(self.min_y, self.max_y)
        
        # Random orientation
        yaw = random.uniform(-math.pi, math.pi)
        goal_pose.pose.orientation.z = math.sin(yaw/2)
        goal_pose.pose.orientation.w = math.cos(yaw/2)
        
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        # Send goal
        self.get_logger().info(f'Sending new goal: x={goal_pose.pose.position.x:.2f}, y={goal_pose.pose.position.y:.2f}')
        self.nav_to_pose_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Explorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 