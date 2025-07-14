#!/usr/bin/env python3
"""
Script untuk mengkonversi LaserScan dari A1M8 ke PointCloud2
Untuk mempertahankan titik-titik scan lebih lama di RViz
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from geometry_msgs.msg import PoseWithCovariance, PoseStamped, PolygonStamped, Point32
from nav_msgs.msg import Path
from std_msgs.msg import Header
import tf2_ros
import tf2_geometry_msgs
import math
import numpy as np
from builtin_interfaces.msg import Time
import time

class ScanToPointcloudConverter(Node):
    def __init__(self):
        super().__init__('scan_to_pointcloud_converter')
        
        # Publishers
        self.pointcloud_pub = self.create_publisher(PointCloud2, 'scan_pointcloud', 10)
        self.robot_pose_pub = self.create_publisher(PoseWithCovariance, 'robot_pose', 10)
        self.robot_path_pub = self.create_publisher(Path, 'robot_path', 10)
        self.robot_footprint_pub = self.create_publisher(PolygonStamped, 'robot_footprint', 10)
        
        # Subscriber untuk LaserScan
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        
        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Path untuk tracking robot
        self.robot_path = Path()
        self.robot_path.header.frame_id = 'laser'
        self.last_pose_time = time.time()
        
        # Accumulated pointcloud untuk persistence
        self.accumulated_points = []
        self.max_points = 50000  # Maksimum titik yang disimpan
        self.point_age_limit = 60.0  # Umur maksimum titik dalam detik
        
        # Timer untuk publish robot footprint dan orientasi
        self.timer = self.create_timer(0.1, self.publish_robot_info)
        
        self.get_logger().info('Scan to PointCloud converter started')

    def scan_callback(self, msg):
        """Callback untuk LaserScan messages"""
        try:
            # Konversi LaserScan ke PointCloud2
            pointcloud = self.laserscan_to_pointcloud(msg)
            
            # Publish pointcloud
            self.pointcloud_pub.publish(pointcloud)
            
            # Tambahkan ke accumulated points
            self.add_to_accumulated_points(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in scan_callback: {e}')

    def laserscan_to_pointcloud(self, scan_msg):
        """Konversi LaserScan ke PointCloud2"""
        # Create PointCloud2 message
        pointcloud = PointCloud2()
        pointcloud.header = scan_msg.header
        pointcloud.height = 1
        
        # Define fields
        pointcloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        pointcloud.is_bigendian = False
        pointcloud.point_step = 16
        
        # Convert scan to points
        points = []
        current_time = time.time()
        
        # Add current scan points
        for i, distance in enumerate(scan_msg.ranges):
            if distance < scan_msg.range_min or distance > scan_msg.range_max:
                continue
                
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            z = 0.0
            
            # Intensity berdasarkan jarak (semakin dekat semakin terang)
            intensity = max(0.0, 1.0 - (distance / scan_msg.range_max))
            
            points.append([x, y, z, intensity])
        
        # Add accumulated points (untuk persistence)
        for point_data in self.accumulated_points:
            if current_time - point_data['timestamp'] < self.point_age_limit:
                x, y, z, intensity = point_data['point']
                # Fade intensity based on age
                age_factor = 1.0 - (current_time - point_data['timestamp']) / self.point_age_limit
                points.append([x, y, z, intensity * age_factor * 0.5])  # Reduce intensity for old points
        
        # Convert to byte array
        if points:
            pointcloud.width = len(points)
            pointcloud.row_step = pointcloud.point_step * pointcloud.width
            pointcloud.data = np.array(points, dtype=np.float32).tobytes()
        else:
            pointcloud.width = 0
            pointcloud.row_step = 0
            pointcloud.data = b''
        
        return pointcloud
    
    def add_to_accumulated_points(self, scan_msg):
        """Tambahkan titik scan ke accumulated points untuk persistence"""
        current_time = time.time()
        
        # Tambahkan titik baru dari scan
        for i, distance in enumerate(scan_msg.ranges):
            if distance < scan_msg.range_min or distance > scan_msg.range_max:
                continue
                
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            z = 0.0
            
            intensity = max(0.0, 1.0 - (distance / scan_msg.range_max))
            
            self.accumulated_points.append({
                'point': [x, y, z, intensity],
                'timestamp': current_time
            })
        
        # Hapus titik yang terlalu lama
        self.accumulated_points = [
            p for p in self.accumulated_points 
            if current_time - p['timestamp'] < self.point_age_limit
        ]
        
        # Batasi jumlah titik
        if len(self.accumulated_points) > self.max_points:
            self.accumulated_points = self.accumulated_points[-self.max_points:]
        
        self.get_logger().info(f'Accumulated points: {len(self.accumulated_points)}', throttle_duration_sec=2.0)
    
    def publish_robot_info(self):
        """Publish robot pose, path, dan footprint"""
        try:
            # Publish robot pose
            self.publish_robot_pose()
            
            # Publish robot path
            self.publish_robot_path()
            
            # Publish robot footprint
            self.publish_robot_footprint()
            
        except Exception as e:
            self.get_logger().error(f'Error in publish_robot_info: {e}')
    
    def publish_robot_pose(self):
        """Publish robot pose dengan orientasi"""
        try:
            # Get transform from laser to laser frame (identity for current pose)
            pose_msg = PoseWithCovariance()
            pose_msg.pose.position.x = 0.0
            pose_msg.pose.position.y = 0.0
            pose_msg.pose.position.z = 0.0
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0
            pose_msg.pose.orientation.w = 1.0
            
            # Add some covariance for visualization
            pose_msg.covariance[0] = 0.1  # x
            pose_msg.covariance[7] = 0.1  # y
            pose_msg.covariance[35] = 0.1  # yaw
            
            self.robot_pose_pub.publish(pose_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing robot pose: {e}')
    
    def publish_robot_path(self):
        """Publish robot path untuk tracking"""
        try:
            current_time = time.time()
            
            # Hanya tambahkan pose baru setiap 0.5 detik
            if current_time - self.last_pose_time > 0.5:
                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = self.get_clock().now().to_msg()
                pose_stamped.header.frame_id = 'laser'
                pose_stamped.pose.position.x = 0.0
                pose_stamped.pose.position.y = 0.0
                pose_stamped.pose.position.z = 0.0
                pose_stamped.pose.orientation.w = 1.0
                
                self.robot_path.poses.append(pose_stamped)
                self.last_pose_time = current_time
                
                # Batasi panjang path
                if len(self.robot_path.poses) > 1000:
                    self.robot_path.poses = self.robot_path.poses[-1000:]
            
            # Update header dan publish
            self.robot_path.header.stamp = self.get_clock().now().to_msg()
            self.robot_path_pub.publish(self.robot_path)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing robot path: {e}')
    
    def publish_robot_footprint(self):
        """Publish robot footprint untuk menunjukkan ukuran dan orientasi robot"""
        try:
            footprint = PolygonStamped()
            footprint.header.stamp = self.get_clock().now().to_msg()
            footprint.header.frame_id = 'laser'
            
            # Buat footprint persegi panjang untuk robot dengan arrow di depan
            # Ukuran robot BCR: 0.5m x 0.5m
            footprint.polygon.points = [
                # Body utama robot
                Point32(x=0.20, y=0.25, z=0.0),   # Depan kiri
                Point32(x=0.20, y=-0.25, z=0.0),  # Depan kanan
                Point32(x=-0.25, y=-0.25, z=0.0), # Belakang kanan
                Point32(x=-0.25, y=0.25, z=0.0),  # Belakang kiri
                Point32(x=0.20, y=0.25, z=0.0),   # Kembali ke depan kiri
                
                # Arrow di depan robot untuk menunjukkan arah
                Point32(x=0.20, y=0.1, z=0.0),    # Mulai arrow
                Point32(x=0.35, y=0.0, z=0.0),    # Ujung arrow (depan)
                Point32(x=0.20, y=-0.1, z=0.0),   # Akhir arrow
            ]
            
            self.robot_footprint_pub.publish(footprint)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing robot footprint: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    converter = ScanToPointcloudConverter()
    
    try:
        rclpy.spin(converter)
    except KeyboardInterrupt:
        pass
    finally:
        converter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 