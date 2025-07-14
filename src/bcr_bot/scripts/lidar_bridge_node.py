#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rplidar import RPLidar
import math

PORT_NAME = '/dev/ttyUSB0'  # Ganti jika port berbeda

class LidarBridgeNode(Node):
    def __init__(self):
        super().__init__('lidar_bridge_node')
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        self.lidar = RPLidar(PORT_NAME)
        self.angle_min = 0.0
        self.angle_max = 2 * math.pi
        self.angle_increment = math.radians(1)  # 1 derajat per step
        self.range_min = 0.15
        self.range_max = 12.0  # A1M8 max range
        self.timer = self.create_timer(0.1, self.publish_scan)  # 10 Hz
        self.latest_ranges = [float('inf')] * 360
        self.scan_iter = self.lidar.iter_scans(max_buf_meas=360)

    def publish_scan(self):
        try:
            scan = next(self.scan_iter)
            ranges = [float('inf')] * 360
            for (_, angle, distance) in scan:
                if distance > 0:
                    idx = int(angle) % 360
                    ranges[idx] = distance / 1000.0  # mm to m
            self.latest_ranges = ranges
        except Exception as e:
            self.get_logger().warn(f'Error reading lidar: {repr(e)}')
            return

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
        scan_msg.ranges = self.latest_ranges
        self.publisher_.publish(scan_msg)

    def destroy_node(self):
        self.lidar.stop()
        self.lidar.disconnect()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LidarBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 