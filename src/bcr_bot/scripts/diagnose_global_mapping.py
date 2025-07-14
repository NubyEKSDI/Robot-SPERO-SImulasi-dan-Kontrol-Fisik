#!/usr/bin/env python3
"""
Global Mapping Diagnostic Script
Diagnoses issues with global mapping system including:
- Transform chain problems
- LiDAR data flow issues
- Obstacle persistence problems
- Robot movement in RViz
"""

import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import TransformStamped
import time
import math

class GlobalMappingDiagnostic(Node):
    def __init__(self):
        super().__init__('global_mapping_diagnostic')
        
        # TF2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Data tracking
        self.lidar_data_received = False
        self.odom_data_received = False
        self.map_data_received = False
        self.robot_pose = None
        self.last_lidar_time = 0
        self.last_odom_time = 0
        self.last_map_time = 0
        
        # Subscribers
        self.lidar_sub = self.create_subscription(
            LaserScan, '/bcr_bot/scan', self.lidar_callback, 10)
        self.a1m8_sub = self.create_subscription(
            LaserScan, '/scan', self.a1m8_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/bcr_bot/odom', self.odom_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/global_map', self.map_callback, 10)
        
        # Timer for continuous diagnostics
        self.diagnostic_timer = self.create_timer(5.0, self.run_diagnostics)
        
        self.get_logger().info('Global Mapping Diagnostic started')
        
    def lidar_callback(self, msg):
        self.lidar_data_received = True
        self.last_lidar_time = time.time()
        
    def a1m8_callback(self, msg):
        self.get_logger().info(f'A1M8 LiDAR data received: {len(msg.ranges)} points')
        
    def odom_callback(self, msg):
        self.odom_data_received = True
        self.last_odom_time = time.time()
        self.robot_pose = msg.pose.pose
        
    def map_callback(self, msg):
        self.map_data_received = True
        self.last_map_time = time.time()
        
    def run_diagnostics(self):
        self.get_logger().info("=== GLOBAL MAPPING DIAGNOSTICS ===")
        
        # Check data flow
        self.check_data_flow()
        
        # Check transform chain
        self.check_transform_chain()
        
        # Check robot movement
        self.check_robot_movement()
        
        # Provide recommendations
        self.provide_recommendations()
        
    def check_data_flow(self):
        """Check if data is flowing properly"""
        current_time = time.time()
        
        self.get_logger().info("--- DATA FLOW CHECK ---")
        
        # LiDAR data
        if self.lidar_data_received:
            age = current_time - self.last_lidar_time
            if age < 5.0:
                self.get_logger().info(f"✅ Gazebo LiDAR: Recent data ({age:.1f}s ago)")
            else:
                self.get_logger().warn(f"⚠️ Gazebo LiDAR: Stale data ({age:.1f}s ago)")
        else:
            self.get_logger().error("❌ Gazebo LiDAR: No data received")
            
        # Odometry data
        if self.odom_data_received:
            age = current_time - self.last_odom_time
            if age < 5.0:
                self.get_logger().info(f"✅ Odometry: Recent data ({age:.1f}s ago)")
            else:
                self.get_logger().warn(f"⚠️ Odometry: Stale data ({age:.1f}s ago)")
        else:
            self.get_logger().error("❌ Odometry: No data received")
            
        # Map data
        if self.map_data_received:
            age = current_time - self.last_map_time
            if age < 10.0:
                self.get_logger().info(f"✅ Global Map: Recent data ({age:.1f}s ago)")
            else:
                self.get_logger().warn(f"⚠️ Global Map: Stale data ({age:.1f}s ago)")
        else:
            self.get_logger().error("❌ Global Map: No data received")
            
    def check_transform_chain(self):
        """Check transform chain integrity"""
        self.get_logger().info("--- TRANSFORM CHAIN CHECK ---")
        
        transforms_to_check = [
            ('map', 'odom'),
            ('odom', 'base_footprint'),
            ('base_footprint', 'base_link'),
            ('base_link', 'laser'),
            ('base_link', 'middle_left_wheel'),
            ('base_link', 'middle_right_wheel'),
            ('map', 'base_footprint'),  # End-to-end check
            ('map', 'laser'),  # End-to-end check
        ]
        
        for parent, child in transforms_to_check:
            try:
                transform = self.tf_buffer.lookup_transform(
                    parent, child, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
                self.get_logger().info(f"✅ Transform {parent} → {child}: OK")
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().error(f"❌ Transform {parent} → {child}: FAILED ({str(e)})")
                
    def check_robot_movement(self):
        """Check if robot is moving in RViz coordinate system"""
        self.get_logger().info("--- ROBOT MOVEMENT CHECK ---")
        
        if self.robot_pose is not None:
            x = self.robot_pose.position.x
            y = self.robot_pose.position.y
            
            # Convert quaternion to yaw
            qx = self.robot_pose.orientation.x
            qy = self.robot_pose.orientation.y
            qz = self.robot_pose.orientation.z
            qw = self.robot_pose.orientation.w
            yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
            
            self.get_logger().info(f"Robot Position: ({x:.3f}, {y:.3f})")
            self.get_logger().info(f"Robot Orientation: {yaw:.3f} rad ({math.degrees(yaw):.1f}°)")
            
            # Check if robot is at origin (potential issue)
            if abs(x) < 0.001 and abs(y) < 0.001:
                self.get_logger().warn("⚠️ Robot at origin - check if odometry is working")
            else:
                self.get_logger().info("✅ Robot position looks normal")
        else:
            self.get_logger().error("❌ No robot pose available")
            
    def provide_recommendations(self):
        """Provide recommendations based on diagnostic results"""
        self.get_logger().info("--- RECOMMENDATIONS ---")
        
        issues_found = []
        
        if not self.lidar_data_received:
            issues_found.append("No LiDAR data")
            self.get_logger().info("🔧 Start global_mapping_node to receive LiDAR data")
            
        if not self.odom_data_received:
            issues_found.append("No odometry data")
            self.get_logger().info("🔧 Check if robot simulation is running")
            
        if not self.map_data_received:
            issues_found.append("No map data")
            self.get_logger().info("🔧 Check if global_mapping_node is publishing maps")
            
        # Check specific transform issues
        try:
            self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except:
            issues_found.append("Missing map→base_footprint transform")
            self.get_logger().info("🔧 Global mapping node needs to publish complete transform chain")
            
        try:
            self.tf_buffer.lookup_transform('base_link', 'middle_left_wheel', rclpy.time.Time())
        except:
            issues_found.append("Missing wheel transforms")
            self.get_logger().info("🔧 Global mapping node needs to publish wheel transforms")
            
        if not issues_found:
            self.get_logger().info("✅ All systems appear to be working correctly!")
        else:
            self.get_logger().warn(f"⚠️ Found {len(issues_found)} issues that need attention")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        diagnostic = GlobalMappingDiagnostic()
        
        print("=== GLOBAL MAPPING DIAGNOSTIC TOOL ===")
        print("This tool will:")
        print("  - Check LiDAR data flow")
        print("  - Verify transform chain")
        print("  - Monitor robot movement")
        print("  - Provide fix recommendations")
        print("Running diagnostics every 5 seconds...")
        print("Press Ctrl+C to stop")
        
        rclpy.spin(diagnostic)
        
    except KeyboardInterrupt:
        print("\nDiagnostic complete!")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 