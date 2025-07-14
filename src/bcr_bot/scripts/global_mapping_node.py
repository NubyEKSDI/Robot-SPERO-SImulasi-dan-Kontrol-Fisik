#!/usr/bin/env python3
"""
Global Mapping Node for BCR Bot

This node transforms LiDAR scan data from robot coordinate frame to global map frame,
ensuring that obstacles remain at their absolute positions even when the robot rotates.

Features:
- Transforms LiDAR data from robot frame to map frame
- Creates persistent occupancy grid map
- Publishes global point cloud for visualization
- Tracks robot trajectory
- Prevents obstacles from rotating with robot
- Robot orientation and direction indicators
- Waypoint and obstacle visualization markers
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped, TransformStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import tf2_ros
import numpy as np
import math
import time
from collections import defaultdict
import threading

class GlobalMappingNode(Node):
    def __init__(self):
        super().__init__('global_mapping_node')
        
        # Initialize map parameters
        self.map_resolution = 0.05  # 5cm per pixel
        self.map_width = 1000  # 50m x 50m map
        self.map_height = 1000
        self.map_origin_x = -25.0  # Map center at (0,0)
        self.map_origin_y = -25.0
        
        # Initialize robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_pose_received = False
        
        # Initialize map data
        self.occupancy_map = np.full((self.map_height, self.map_width), -1, dtype=np.int8)  # Unknown
        self.hit_count = np.zeros((self.map_height, self.map_width), dtype=np.int32)
        self.miss_count = np.zeros((self.map_height, self.map_width), dtype=np.int32)
        
        # Global point cloud storage
        self.global_points = []
        self.max_points = 50000  # Limit for performance
        
        # Robot trajectory
        self.robot_trajectory = []
        self.max_trajectory_points = 1000
        
        # Obstacle tracking
        self.obstacle_positions = []
        self.max_obstacle_markers = 200
        
        # Waypoint tracking
        self.waypoints = [
            (0.0, 0.0, "Start"),
            (-12.06, -0.02, "Bottom-left"),
            (-12.06, -1.79, "Bottom-right"),
            (15.31, -2.10, "Top-right"),
            (3.57, -2.10, "Mid"),
            (3.57, 0.0, "Mid-left")
        ]
        
        # Thread safety
        self.map_lock = threading.Lock()
        
        # TF2 broadcaster
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        # Initialize subscribers with optimized queue sizes to prevent overflow
        self.lidar_sub = self.create_subscription(
            LaserScan, 
            '/bcr_bot/scan', 
            self.lidar_callback, 
            3  # Reduced queue size from 10 to 3
        )
        
        # Subscribe to A1M8 real-world LiDAR data via MQTT bridge
        self.a1m8_sub = self.create_subscription(
            LaserScan,
            '/scan',  # From MQTT-ROS2 bridge
            self.a1m8_lidar_callback,
            5
        )
        
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/bcr_bot/odom', 
            self.odom_callback, 
            5  # Reduced queue size from 10 to 5
        )
        
        # Initialize publishers with optimized queue sizes
        self.global_scan_pub = self.create_publisher(
            LaserScan, 
            '/bcr_bot/scan', 
            3  # Reduced queue size
        )
        
        self.global_pointcloud_pub = self.create_publisher(
            PointCloud2, 
            '/bcr_bot/scan_pointcloud', 
            3  # Reduced queue size
        )
        
        # ROS2 Publishers
        self.map_pub = self.create_publisher(OccupancyGrid, '/global_map', 10)
        self.global_cloud_pub = self.create_publisher(PointCloud2, '/global_pointcloud', 10)
        self.trajectory_pub = self.create_publisher(Path, '/global_robot_path', 10)
        self.orientation_marker_pub = self.create_publisher(Marker, '/robot_orientation_marker', 10)
        self.direction_markers_pub = self.create_publisher(MarkerArray, '/robot_direction_markers', 10)
        self.waypoint_markers_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)
        self.obstacle_markers_pub = self.create_publisher(MarkerArray, '/obstacle_markers', 10)
        
        # Timer for publishing
        self.map_timer = self.create_timer(2.0, self.publish_map)  # Publish every 2 seconds
        self.cloud_timer = self.create_timer(0.5, self.publish_global_pointcloud)  # Publish every 0.5 seconds
        self.trajectory_timer = self.create_timer(1.0, self.publish_trajectory)  # Publish every 1 second
        self.markers_timer = self.create_timer(0.2, self.publish_markers)  # Publish markers every 0.2 seconds
        self.tf_timer = self.create_timer(0.1, self.publish_transforms)  # Publish TF every 0.1 seconds
        
        # Publish static waypoint markers
        self.waypoint_timer = self.create_timer(5.0, self.publish_waypoint_markers)
        
        # Logging
        self.scan_count = 0
        self.last_log_time = time.time()
        
        # Initialize timing variables for throttling
        self.last_scan_time = 0.0
        self.last_error_time = 0.0
        self.last_publish_time = 0.0
        
        # Initialize state variables
        self.robot_pose = None
        self.scan_data = None
        self.mapping_active = True
        
        self.get_logger().info('Global Mapping Node started')
        self.get_logger().info(f'Map size: {self.map_width}x{self.map_height} pixels')
        self.get_logger().info(f'Map resolution: {self.map_resolution}m/pixel')
        self.get_logger().info(f'Map coverage: {self.map_width*self.map_resolution:.1f}m x {self.map_height*self.map_resolution:.1f}m')

    def publish_transforms(self):
        """Publish complete TF transforms for global mapping including wheel frames"""
        if not self.robot_pose_received:
            return
        
        current_time = self.get_clock().now()
        
        # Create complete transform chain
        transforms = []
        
        # 1. map to odom transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        transforms.append(t)
        
        # 2. odom to base_footprint transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.robot_x
        t.transform.translation.y = self.robot_y
        t.transform.translation.z = 0.0
        # Convert yaw to quaternion
        qz = math.sin(self.robot_yaw / 2.0)
        qw = math.cos(self.robot_yaw / 2.0)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        transforms.append(t)
        
        # 3. base_footprint to base_link transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'base_footprint'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.1  # 10cm above ground
        t.transform.rotation.w = 1.0
        transforms.append(t)
        
        # 4. base_link to laser transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser'
        t.transform.translation.x = 0.0  # LiDAR at center
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.1  # 10cm above base_link
        t.transform.rotation.w = 1.0
        transforms.append(t)
        
        # 5. base_link to wheel transforms
        # Middle left wheel
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'middle_left_wheel'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.15  # 15cm to the left
        t.transform.translation.z = -0.05  # 5cm below base_link
        t.transform.rotation.w = 1.0
        transforms.append(t)
        
        # Middle right wheel
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'middle_right_wheel'
        t.transform.translation.x = 0.0
        t.transform.translation.y = -0.15  # 15cm to the right
        t.transform.translation.z = -0.05  # 5cm below base_link
        t.transform.rotation.w = 1.0
        transforms.append(t)
        
        # 6. Additional wheel frames if needed
        # Front left wheel
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'front_left_wheel'
        t.transform.translation.x = 0.15  # 15cm forward
        t.transform.translation.y = 0.15  # 15cm to the left
        t.transform.translation.z = -0.05
        t.transform.rotation.w = 1.0
        transforms.append(t)
        
        # Front right wheel
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'front_right_wheel'
        t.transform.translation.x = 0.15  # 15cm forward
        t.transform.translation.y = -0.15  # 15cm to the right
        t.transform.translation.z = -0.05
        t.transform.rotation.w = 1.0
        transforms.append(t)
        
        # Rear left wheel
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'rear_left_wheel'
        t.transform.translation.x = -0.15  # 15cm backward
        t.transform.translation.y = 0.15  # 15cm to the left
        t.transform.translation.z = -0.05
        t.transform.rotation.w = 1.0
        transforms.append(t)
        
        # Rear right wheel
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'rear_right_wheel'
        t.transform.translation.x = -0.15  # 15cm backward
        t.transform.translation.y = -0.15  # 15cm to the right
        t.transform.translation.z = -0.05
        t.transform.rotation.w = 1.0
        transforms.append(t)
        
        # Publish all transforms
        self.tf_broadcaster.sendTransform(transforms)

    def publish_markers(self):
        """Publish robot orientation and direction markers"""
        if not self.robot_pose_received:
            return
        
        # Robot orientation marker (large arrow)
        marker = Marker()
        marker.header.frame_id = 'global_map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'robot_orientation'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Position at robot
        marker.pose.position.x = self.robot_x
        marker.pose.position.y = self.robot_y
        marker.pose.position.z = 0.2
        
        # Orientation (quaternion from yaw)
        qz = math.sin(self.robot_yaw / 2.0)
        qw = math.cos(self.robot_yaw / 2.0)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        
        # Scale
        marker.scale.x = 1.0  # Length
        marker.scale.y = 0.1  # Width
        marker.scale.z = 0.1  # Height
        
        # Color (bright red)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
        
        self.orientation_marker_pub.publish(marker)
        
        # Direction indicators (multiple arrows)
        marker_array = MarkerArray()
        
        # Front direction arrow
        front_marker = Marker()
        front_marker.header.frame_id = 'global_map'
        front_marker.header.stamp = self.get_clock().now().to_msg()
        front_marker.ns = 'direction_indicators'
        front_marker.id = 1
        front_marker.type = Marker.ARROW
        front_marker.action = Marker.ADD
        
        # Position in front of robot
        front_x = self.robot_x + 0.5 * math.cos(self.robot_yaw)
        front_y = self.robot_y + 0.5 * math.sin(self.robot_yaw)
        front_marker.pose.position.x = front_x
        front_marker.pose.position.y = front_y
        front_marker.pose.position.z = 0.1
        
        front_marker.pose.orientation.x = 0.0
        front_marker.pose.orientation.y = 0.0
        front_marker.pose.orientation.z = qz
        front_marker.pose.orientation.w = qw
        
        front_marker.scale.x = 0.5
        front_marker.scale.y = 0.05
        front_marker.scale.z = 0.05
        
        front_marker.color.r = 0.0
        front_marker.color.g = 1.0
        front_marker.color.b = 0.0
        front_marker.color.a = 1.0
        
        front_marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
        marker_array.markers.append(front_marker)
        
        # Robot body outline
        body_marker = Marker()
        body_marker.header.frame_id = 'global_map'
        body_marker.header.stamp = self.get_clock().now().to_msg()
        body_marker.ns = 'robot_body'
        body_marker.id = 2
        body_marker.type = Marker.CUBE
        body_marker.action = Marker.ADD
        
        body_marker.pose.position.x = self.robot_x
        body_marker.pose.position.y = self.robot_y
        body_marker.pose.position.z = 0.05
        
        body_marker.pose.orientation.x = 0.0
        body_marker.pose.orientation.y = 0.0
        body_marker.pose.orientation.z = qz
        body_marker.pose.orientation.w = qw
        
        body_marker.scale.x = 0.4  # Robot length
        body_marker.scale.y = 0.3  # Robot width
        body_marker.scale.z = 0.1  # Robot height
        
        body_marker.color.r = 0.0
        body_marker.color.g = 0.0
        body_marker.color.b = 1.0
        body_marker.color.a = 0.5
        
        body_marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
        marker_array.markers.append(body_marker)
        
        self.direction_markers_pub.publish(marker_array)
        
        # Publish obstacle markers
        self.publish_obstacle_markers()

    def publish_waypoint_markers(self):
        """Publish waypoint markers"""
        marker_array = MarkerArray()
        
        for i, (x, y, name) in enumerate(self.waypoints):
            # Waypoint marker
            marker = Marker()
            marker.header.frame_id = 'global_map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'waypoints'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.5
            
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            
            # Color coding
            if i == 0:  # Start point
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif i == len(self.waypoints) - 1:  # End point
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:  # Intermediate points
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            
            marker.color.a = 0.8
            marker_array.markers.append(marker)
            
            # Waypoint label
            text_marker = Marker()
            text_marker.header.frame_id = 'global_map'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'waypoint_labels'
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = x
            text_marker.pose.position.y = y
            text_marker.pose.position.z = 0.8
            
            text_marker.pose.orientation.w = 1.0
            
            text_marker.scale.z = 0.3
            
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            text_marker.text = f"{i}: {name}"
            marker_array.markers.append(text_marker)
        
        self.waypoint_markers_pub.publish(marker_array)

    def publish_obstacle_markers(self):
        """Publish obstacle markers from recent detections"""
        marker_array = MarkerArray()
        
        # Get recent obstacle points
        recent_obstacles = self.global_points[-min(100, len(self.global_points)):]
        
        for i, (x, y) in enumerate(recent_obstacles):
            if i >= self.max_obstacle_markers:
                break
                
            marker = Marker()
            marker.header.frame_id = 'global_map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'obstacles'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.05
            
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            
            # Color based on distance from robot
            distance = math.sqrt((x - self.robot_x)**2 + (y - self.robot_y)**2)
            if distance < 0.5:  # Very close
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            elif distance < 1.0:  # Close
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0
            else:  # Far
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            
            marker.color.a = 0.6
            marker.lifetime = rclpy.duration.Duration(seconds=2.0).to_msg()
            marker_array.markers.append(marker)
        
        self.obstacle_markers_pub.publish(marker_array)

    def odom_callback(self, msg):
        """Update robot pose from odometry"""
        # Extract position
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Extract orientation (convert quaternion to yaw)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Convert quaternion to yaw angle
        self.robot_yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        
        self.robot_pose_received = True
        
        # Add to trajectory
        if len(self.robot_trajectory) == 0 or \
           math.sqrt((self.robot_x - self.robot_trajectory[-1][0])**2 + 
                    (self.robot_y - self.robot_trajectory[-1][1])**2) > 0.1:  # 10cm threshold
            self.robot_trajectory.append((self.robot_x, self.robot_y, self.robot_yaw))
            if len(self.robot_trajectory) > self.max_trajectory_points:
                self.robot_trajectory.pop(0)

    def a1m8_lidar_callback(self, msg):
        """Process A1M8 real-world LiDAR data"""
        try:
            current_time = time.time()
            
            # Throttle to prevent queue overflow - only process every 0.1 seconds (10Hz)
            if current_time - getattr(self, 'last_a1m8_scan_time', 0) < 0.1:
                return
                
            self.last_a1m8_scan_time = current_time
            
            # Ensure proper frame_id for A1M8 data
            msg.header.frame_id = 'laser'
            msg.header.stamp = self.get_clock().now().to_msg()
            
            # Publish A1M8 data to bcr_bot/scan topic for consistency with RViz config
            self.global_scan_pub.publish(msg)
            
            # Convert to point cloud and publish
            point_cloud = self.laser_scan_to_point_cloud(msg)
            if point_cloud is not None:
                self.global_pointcloud_pub.publish(point_cloud)
            
            # Process for mapping if we have robot pose
            if self.robot_pose_received:
                # Transform scan to global coordinates for mapping
                global_points = self.transform_scan_to_global(msg)
                
                # Update occupancy map
                with self.map_lock:
                    self.update_occupancy_map_absolute(global_points, msg.range_max)
                    
                    # Add to global point cloud (for visualization)
                    self.global_points.extend(global_points)
                    
                    # Limit global points for performance
                    if len(self.global_points) > self.max_points:
                        self.global_points = self.global_points[-self.max_points//2:]
            
            # Update scan count
            self.scan_count += 1
            
            # Log status every 50 scans for A1M8
            if self.scan_count % 50 == 0:
                self.get_logger().info(f"A1M8 Global Mapping: Processed {self.scan_count} A1M8 scans, publishing to RViz")
                
        except Exception as e:
            current_time = time.time()
            if current_time - getattr(self, 'last_a1m8_error_time', 0) > 5.0:
                self.get_logger().warn(f"Error processing A1M8 LiDAR data: {str(e)}")
                self.last_a1m8_error_time = current_time

    def lidar_callback(self, msg):
        """Process LiDAR scan data with throttling to prevent queue overflow"""
        try:
            current_time = time.time()
            
            # Throttle to prevent queue overflow - only process every 0.05 seconds (20Hz)
            if current_time - getattr(self, 'last_gazebo_scan_time', 0) < 0.05:
                return
                
            self.last_gazebo_scan_time = current_time
            
            # Process the scan data immediately
            self.process_scan_data(msg)
            
        except Exception as e:
            current_time = time.time()
            if current_time - getattr(self, 'last_gazebo_error_time', 0) > 5.0:
                self.get_logger().warn(f"Error processing Gazebo LiDAR data: {str(e)}")
                self.last_gazebo_error_time = current_time

    def process_scan_data(self, scan_msg):
        """Process scan data efficiently and publish for RViz visualization - FIXED VERSION"""
        try:
            # Store the scan data
            self.scan_data = scan_msg
            
            # IMPORTANT: Keep frame_id as 'laser' for proper RViz display
            # This ensures the scan is displayed relative to the laser frame
            scan_msg.header.frame_id = 'laser'
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            
            # Publish scan data for RViz visualization
            # This shows the "raw" LiDAR view relative to robot
            self.global_scan_pub.publish(scan_msg)
            
            # Convert to point cloud for additional visualization
            point_cloud = self.laser_scan_to_point_cloud(scan_msg)
            if point_cloud is not None:
                self.global_pointcloud_pub.publish(point_cloud)
            
            # Process for GLOBAL mapping if we have robot pose
            if self.robot_pose_received:
                # Transform scan to ABSOLUTE global coordinates
                # This is where we fix the "obstacle following robot" issue
                global_points = self.transform_scan_to_global(scan_msg)
                
                # Update occupancy map with ABSOLUTE coordinates
                with self.map_lock:
                    self.update_occupancy_map_absolute(global_points, scan_msg.range_max)
                    
                    # Add to global point cloud (ABSOLUTE positions)
                    # Filter recent points to prevent duplicates
                    new_points = []
                    for x, y in global_points:
                        # Check if this point is significantly different from recent points
                        is_new = True
                        for recent_x, recent_y in self.global_points[-100:]:  # Check last 100 points
                            if abs(x - recent_x) < 0.05 and abs(y - recent_y) < 0.05:  # 5cm threshold
                                is_new = False
                                break
                        if is_new:
                            new_points.append((x, y))
                    
                    self.global_points.extend(new_points)
                    
                    # Limit global points for performance
                    if len(self.global_points) > self.max_points:
                        self.global_points = self.global_points[-self.max_points//2:]
            
            # Update scan count
            self.scan_count += 1
            
            # Log status every 100 scans to confirm everything works
            if self.scan_count % 100 == 0:
                self.get_logger().info(f"Global Mapping: Processed {self.scan_count} scans")
                self.get_logger().info(f"Robot position: ({self.robot_x:.2f}, {self.robot_y:.2f}), yaw: {self.robot_yaw:.2f}")
                self.get_logger().info(f"Global points stored: {len(self.global_points)}")
                
        except Exception as e:
            # Throttle error messages
            current_time = time.time()
            if current_time - getattr(self, 'last_process_error_time', 0) > 5.0:
                self.get_logger().warn(f"Error processing scan data: {str(e)}")
                self.last_process_error_time = current_time

    def laser_scan_to_point_cloud(self, scan_msg):
        """Convert LaserScan to PointCloud2 for visualization"""
        try:
            from sensor_msgs.msg import PointCloud2, PointField
            import struct
            
            # Create PointCloud2 message
            cloud_msg = PointCloud2()
            cloud_msg.header = scan_msg.header
            cloud_msg.header.frame_id = 'laser'  # Use laser frame for consistency
            
            # Define point fields (x, y, z)
            cloud_msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            
            cloud_msg.point_step = 12  # 3 floats * 4 bytes each
            
            # Convert scan points to cartesian coordinates
            points = []
            angle = scan_msg.angle_min
            
            for i, r in enumerate(scan_msg.ranges):
                # Skip invalid points
                if r < scan_msg.range_min or r > scan_msg.range_max or r == float('inf') or r != r:  # r != r checks for NaN
                    angle += scan_msg.angle_increment
                    continue
                
                # Convert to cartesian coordinates
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                z = 0.0
                
                # Pack as binary data
                point_data = struct.pack('fff', x, y, z)
                points.append(point_data)
                
                angle += scan_msg.angle_increment
            
            # Set cloud properties
            cloud_msg.width = len(points)
            cloud_msg.height = 1
            cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
            cloud_msg.is_dense = True
            
            # Combine all point data
            cloud_msg.data = b''.join(points)
            
            return cloud_msg
            
        except Exception as e:
            self.get_logger().warn(f"Error converting LaserScan to PointCloud2: {str(e)}")
            return None

    def transform_scan_to_global(self, scan_msg):
        """Transform laser scan points from robot frame to ABSOLUTE global frame"""
        global_points = []
        
        angle = scan_msg.angle_min
        for i, range_val in enumerate(scan_msg.ranges):
            # Skip invalid readings
            if range_val < scan_msg.range_min or range_val > scan_msg.range_max or \
               range_val == float('inf') or range_val != range_val:  # NaN check
                angle += scan_msg.angle_increment
                continue
            
            # Point in robot frame (relative to robot center)
            x_robot = range_val * math.cos(angle)
            y_robot = range_val * math.sin(angle)
            
            # Transform to ABSOLUTE global frame
            # This ensures obstacles stay in their real-world positions
            cos_yaw = math.cos(self.robot_yaw)
            sin_yaw = math.sin(self.robot_yaw)
            
            # Apply rotation matrix and translation to get absolute coordinates
            x_global = self.robot_x + (x_robot * cos_yaw - y_robot * sin_yaw)
            y_global = self.robot_y + (x_robot * sin_yaw + y_robot * cos_yaw)
            
            # Store absolute global coordinates
            global_points.append((x_global, y_global))
            angle += scan_msg.angle_increment
        
        return global_points

    def world_to_map(self, x, y):
        """Convert world coordinates to map pixel coordinates"""
        map_x = int((x - self.map_origin_x) / self.map_resolution)
        map_y = int((y - self.map_origin_y) / self.map_resolution)
        return map_x, map_y

    def map_to_world(self, map_x, map_y):
        """Convert map pixel coordinates to world coordinates"""
        x = map_x * self.map_resolution + self.map_origin_x
        y = map_y * self.map_resolution + self.map_origin_y
        return x, y

    def is_valid_map_coord(self, map_x, map_y):
        """Check if map coordinates are within bounds"""
        return 0 <= map_x < self.map_width and 0 <= map_y < self.map_height

    def update_occupancy_map(self, global_points, max_range):
        """Update occupancy map using ray tracing"""
        robot_map_x, robot_map_y = self.world_to_map(self.robot_x, self.robot_y)
        
        if not self.is_valid_map_coord(robot_map_x, robot_map_y):
            return
        
        # Mark occupied cells (obstacle points)
        for x_global, y_global in global_points:
            map_x, map_y = self.world_to_map(x_global, y_global)
            if self.is_valid_map_coord(map_x, map_y):
                self.hit_count[map_y, map_x] += 1
                
                # Ray tracing from robot to obstacle (mark free space)
                self.trace_ray(robot_map_x, robot_map_y, map_x, map_y)
        
        # Update occupancy probabilities
        self.update_map_probabilities()

    def trace_ray(self, x0, y0, x1, y1):
        """Trace ray from (x0,y0) to (x1,y1) and mark free space"""
        # Bresenham's line algorithm
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        n = 1 + dx + dy
        x_inc = 1 if x1 > x0 else -1
        y_inc = 1 if y1 > y0 else -1
        error = dx - dy
        
        dx *= 2
        dy *= 2
        
        for _ in range(n):
            if self.is_valid_map_coord(x, y):
                self.miss_count[y, x] += 1
            
            if error > 0:
                x += x_inc
                error -= dy
            else:
                y += y_inc
                error += dx

    def update_occupancy_map_absolute(self, global_points, max_range):
        """Update occupancy map using ABSOLUTE global coordinates"""
        robot_map_x, robot_map_y = self.world_to_map(self.robot_x, self.robot_y)
        
        if not self.is_valid_map_coord(robot_map_x, robot_map_y):
            return
        
        # Mark occupied cells at ABSOLUTE positions
        for x_global, y_global in global_points:
            map_x, map_y = self.world_to_map(x_global, y_global)
            if self.is_valid_map_coord(map_x, map_y):
                self.hit_count[map_y, map_x] += 1
                
                # Ray tracing from robot to obstacle (mark free space)
                # This ensures free space is marked correctly in absolute coordinates
                self.trace_ray_absolute(robot_map_x, robot_map_y, map_x, map_y)
        
        # Update occupancy probabilities
        self.update_map_probabilities()

    def trace_ray_absolute(self, x0, y0, x1, y1):
        """Trace ray in absolute coordinates - prevents free space from moving with robot"""
        # Bresenham's line algorithm for absolute coordinate mapping
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        n = 1 + dx + dy
        x_inc = 1 if x1 > x0 else -1
        y_inc = 1 if y1 > y0 else -1
        error = dx - dy
        
        dx *= 2
        dy *= 2
        
        for _ in range(n):
            if self.is_valid_map_coord(x, y):
                # Only mark as free if we haven't seen an obstacle here recently
                if self.hit_count[y, x] < 2:  # Allow some obstacle persistence
                    self.miss_count[y, x] += 1
            
            if error > 0:
                x += x_inc
                error -= dy
            else:
                y += y_inc
                error += dx

    def update_map_probabilities(self):
        """Update occupancy probabilities based on hit/miss counts"""
        # Calculate probabilities
        total_count = self.hit_count + self.miss_count
        probability = np.divide(self.hit_count, total_count, 
                               out=np.full_like(self.hit_count, 0.5, dtype=np.float64), 
                               where=total_count!=0)
        
        # Convert to occupancy grid format (-1: unknown, 0-100: probability)
        self.occupancy_map = np.where(total_count == 0, -1, 
                                     np.clip(probability * 100, 0, 100).astype(np.int8))

    def publish_map(self):
        """Publish occupancy grid map"""
        with self.map_lock:
            map_msg = OccupancyGrid()
            map_msg.header.frame_id = 'global_map'
            map_msg.header.stamp = self.get_clock().now().to_msg()
            
            map_msg.info.resolution = self.map_resolution
            map_msg.info.width = self.map_width
            map_msg.info.height = self.map_height
            map_msg.info.origin.position.x = self.map_origin_x
            map_msg.info.origin.position.y = self.map_origin_y
            map_msg.info.origin.orientation.w = 1.0
            
            # Flatten map data
            map_msg.data = self.occupancy_map.flatten().tolist()
            
            self.map_pub.publish(map_msg)

    def publish_global_pointcloud(self):
        """Publish global point cloud"""
        if not self.global_points:
            return
        
        with self.map_lock:
            cloud_msg = PointCloud2()
            cloud_msg.header.frame_id = 'global_map'
            cloud_msg.header.stamp = self.get_clock().now().to_msg()
            
            # Point cloud fields
            cloud_msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            
            cloud_msg.height = 1
            cloud_msg.width = len(self.global_points)
            cloud_msg.point_step = 12
            cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
            cloud_msg.is_dense = True
            
            # Pack point data
            data = []
            for x, y in self.global_points:
                data.extend([x, y, 0.0])  # z = 0 for 2D
            
            cloud_msg.data = np.array(data, dtype=np.float32).tobytes()
            
            self.global_cloud_pub.publish(cloud_msg)

    def publish_trajectory(self):
        """Publish robot trajectory"""
        if not self.robot_trajectory:
            return
        
        path_msg = Path()
        path_msg.header.frame_id = 'global_map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for x, y, yaw in self.robot_trajectory:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'global_map'
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0
            
            # Convert yaw to quaternion
            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = qz
            pose_stamped.pose.orientation.w = qw
            
            path_msg.poses.append(pose_stamped)
        
        self.trajectory_pub.publish(path_msg)

    def save_map(self, filename=None):
        """Save occupancy map to file"""
        if filename is None:
            filename = f'global_map_{int(time.time())}.npy'
        
        with self.map_lock:
            np.save(filename, self.occupancy_map)
            self.get_logger().info(f'Map saved to {filename}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = GlobalMappingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 