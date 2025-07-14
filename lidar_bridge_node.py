#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import time

class LidarBridgeNode(Node):
    def __init__(self):
        super().__init__('lidar_bridge_node')
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.scan_file = 'scan.txt'  # File yang berisi data lidar
        self.angle_min = 0.0
        self.angle_max = 2 * math.pi
        self.angle_increment = math.radians(1)  # 1 derajat per step
        self.range_min = 0.15
        self.range_max = 6.0

    def timer_callback(self):
        try:
            with open(self.scan_file, 'r') as f:
                lines = f.readlines()
            ranges = []
            for line in lines:
                parts = line.strip().split(',')
                if len(parts) != 2:
                    continue
                angle, distance = float(parts[0]), float(parts[1])
                ranges.append(distance)
            scan_msg = LaserScan()
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.header.frame_id = 'laser'
            scan_msg.angle_min = self.angle_min
            scan_msg.angle_max = self.angle_max
            scan_msg.angle_increment = self.angle_increment
            scan_msg.time_increment = 0.0
            scan_msg.scan_time = 0.1
            scan_msg.range_min = self.range_min
            scan_msg.range_max = self.range_max
            scan_msg.ranges = ranges
            self.publisher_.publish(scan_msg)
        except Exception as e:
            self.get_logger().warn(f'Gagal membaca file scan.txt: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = LidarBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 