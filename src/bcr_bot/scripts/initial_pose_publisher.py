#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        
        # Publisher for initial pose
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            10
        )
        
        # Wait a bit for subscribers to connect
        time.sleep(1.0)
        
        # Publish initial pose
        self.publish_initial_pose()
        
        self.get_logger().info('Initial pose published!')
        
        # Shutdown after publishing
        self.create_timer(2.0, self.shutdown_node)
    
    def publish_initial_pose(self):
        pose_msg = PoseWithCovarianceStamped()
        
        # Header
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        # Robot spawns at (0,0) in simulation
        # Map origin is at (-21.2, -21.2)
        # So robot position in map coordinates is (21.2, 21.2)
        pose_msg.pose.pose.position.x = 21.2
        pose_msg.pose.pose.position.y = 21.2
        pose_msg.pose.pose.position.z = 0.0
        
        # Orientation (facing forward)
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = 0.0
        pose_msg.pose.pose.orientation.w = 1.0
        
        # Covariance (uncertainty in the pose)
        pose_msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787
        ]
        
        # Publish the pose
        self.initial_pose_pub.publish(pose_msg)
        
        self.get_logger().info(f'Published initial pose: x={pose_msg.pose.pose.position.x}, y={pose_msg.pose.pose.position.y}')
    
    def shutdown_node(self):
        self.get_logger().info('Initial pose publisher shutting down')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    initial_pose_publisher = InitialPosePublisher()
    
    try:
        rclpy.spin(initial_pose_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        initial_pose_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main() 