#!/usr/bin/env python3
"""
LiDAR A1M8 Data Processor - NO REAR DETECTION
This script processes A1M8 LiDAR data while excluding the rear sector (135°-225°)
to prevent the robot from detecting obstacles behind it.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import threading
import time
import paho.mqtt.client as mqtt
import json
import sys
import os

# Set ROS_DOMAIN_ID untuk komunikasi dengan Raspberry Pi
os.environ['ROS_DOMAIN_ID'] = '0'

class A1M8LiDARProcessor(Node):
    def __init__(self):
        super().__init__('a1m8_lidar_processor')
        
        # Initialize ROS publishers and subscribers
        self.scan_sub = self.create_subscription(LaserScan, '/bcr_bot/scan', self.scan_callback, 10)
        self.filtered_scan_pub = self.create_publisher(LaserScan, '/bcr_bot/filtered_scan', 10)
        
        # Initialize MQTT client
        self.mqtt_client = None
        self.mqtt_connected = False
        
        # Initialize MQTT client
        try:
            self.mqtt_client = mqtt.Client()
            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
            self.mqtt_client.connect("codex.petra.ac.id", 1883, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info("Successfully connected to MQTT broker")
            self.print_status("MQTT: Connected to broker")
        except Exception as e:
            self.get_logger().warning(f"Failed to connect to MQTT broker: {str(e)}. Continuing without MQTT support.")
            self.print_status("MQTT: Connection failed")
            self.mqtt_client = None
        
        # Define rear sector to exclude (behind the robot)
        # Rear sector: 135° to 225° (or -45° to +45° from back)
        # This excludes the back 90 degrees of the robot
        self.rear_start_angle = 135  # degrees
        self.rear_end_angle = 225    # degrees
        
        # Convert to radians for processing
        self.rear_start_rad = math.radians(self.rear_start_angle)
        self.rear_end_rad = math.radians(self.rear_end_angle)
        
        self.print_status("A1M8 LiDAR Processor started - REAR DETECTION DISABLED")
        self.print_status(f"Rear sector excluded: {self.rear_start_angle}° to {self.rear_end_angle}°")

    def print_status(self, message):
        """Print status message with timestamp"""
        timestamp = time.strftime("%H:%M:%S")
        status_line = f'[{timestamp}] A1M8-LiDAR: {message}'
        print(status_line)
        
        # Send status via MQTT if available
        if self.mqtt_client is not None:
            try:
                self.mqtt_client.publish("robot/lidar/a1m8_status", json.dumps({
                    'status': status_line,
                    'timestamp': time.time(),
                    'rear_excluded': True,
                    'rear_sector': f"{self.rear_start_angle}°-{self.rear_end_angle}°"
                }))
            except Exception as e:
                self.get_logger().error(f"Error sending status: {str(e)}")

    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback when MQTT client connects"""
        if rc == 0:
            self.mqtt_connected = True
            self.get_logger().info("Connected to MQTT broker successfully")
            self.print_status("MQTT: Connected to broker")
            
            # Subscribe to topics
            topics = [
                ("robot/lidar/a1m8_control", 1),  # For control commands
            ]
            
            for topic, qos in topics:
                try:
                    client.subscribe(topic, qos)
                    self.get_logger().info(f"Subscribed to {topic} with QoS {qos}")
                except Exception as e:
                    self.get_logger().error(f"Failed to subscribe to {topic}: {str(e)}")
            
            self.print_status("MQTT: All topics subscribed successfully")
            
        else:
            self.mqtt_connected = False
            self.get_logger().error(f"Failed to connect to MQTT broker with result code: {rc}")
            self.print_status("MQTT: Connection failed")

    def on_mqtt_disconnect(self, client, userdata, rc):
        """Callback when MQTT client disconnects"""
        self.mqtt_connected = False
        self.get_logger().warning(f"MQTT client disconnected with code: {rc}")
        self.print_status("MQTT: Disconnected from broker")

    def is_in_rear_sector(self, angle_rad):
        """Check if angle is in the rear sector that should be excluded"""
        # Normalize angle to 0-2π
        angle_rad = angle_rad % (2 * math.pi)
        
        # Convert rear sector to 0-2π range
        rear_start = self.rear_start_rad
        rear_end = self.rear_end_rad
        
        # Handle wrap-around case
        if rear_start > rear_end:
            # Sector wraps around 0
            return angle_rad >= rear_start or angle_rad <= rear_end
        else:
            # Normal sector
            return rear_start <= angle_rad <= rear_end

    def filter_lidar_data(self, ranges, angle_min, angle_max, angle_increment):
        """Filter LiDAR data to exclude rear sector"""
        filtered_ranges = []
        n = len(ranges)
        
        for i in range(n):
            # Calculate current angle
            current_angle = angle_min + i * angle_increment
            
            # Check if this angle is in the rear sector
            if self.is_in_rear_sector(current_angle):
                # Set to infinity (no detection) for rear sector
                filtered_ranges.append(float('inf'))
            else:
                # Keep original value for non-rear sectors
                filtered_ranges.append(ranges[i])
        
        return filtered_ranges

    def scan_callback(self, msg):
        """Process LiDAR data from A1M8 and filter out rear sector"""
        try:
            # Filter out rear sector
            filtered_ranges = self.filter_lidar_data(
                msg.ranges, 
                msg.angle_min, 
                msg.angle_max, 
                msg.angle_increment
            )
            
            # Create filtered LaserScan message
            filtered_msg = LaserScan()
            filtered_msg.header = msg.header
            filtered_msg.angle_min = msg.angle_min
            filtered_msg.angle_max = msg.angle_max
            filtered_msg.angle_increment = msg.angle_increment
            filtered_msg.time_increment = msg.time_increment
            filtered_msg.scan_time = msg.scan_time
            filtered_msg.range_min = msg.range_min
            filtered_msg.range_max = msg.range_max
            filtered_msg.ranges = filtered_ranges
            filtered_msg.intensities = msg.intensities
            
            # Publish filtered data
            self.filtered_scan_pub.publish(filtered_msg)
            
            # Send filtered data via MQTT for real-world processing
            if self.mqtt_client is not None:
                try:
                    # Create data packet for real-world LiDAR (no rear)
                    lidar_data = {
                        "timestamp": time.time(),
                        "ranges": filtered_ranges,
                        "angle_min": msg.angle_min,
                        "angle_max": msg.angle_max,
                        "angle_increment": msg.angle_increment,
                        "range_min": msg.range_min,
                        "range_max": msg.range_max,
                        "source": "a1m8_lidar_no_rear",
                        "rear_excluded": True,
                        "rear_sector": f"{self.rear_start_angle}°-{self.rear_end_angle}°"
                    }
                    
                    self.mqtt_client.publish("robot/lidar/real_world_data", json.dumps(lidar_data))
                    
                except Exception as e:
                    self.get_logger().error(f"Error sending filtered LiDAR data: {str(e)}")
            
            # Log processing statistics periodically
            current_time = time.time()
            if not hasattr(self, 'last_stats_time'):
                self.last_stats_time = 0
            
            if current_time - self.last_stats_time > 5.0:  # Log every 5 seconds
                original_valid = sum(1 for r in msg.ranges if r != float('inf'))
                filtered_valid = sum(1 for r in filtered_ranges if r != float('inf'))
                excluded_count = original_valid - filtered_valid
                
                self.print_status(f"Processing stats: Original={original_valid}, Filtered={filtered_valid}, Excluded={excluded_count} points")
                self.last_stats_time = current_time
                
        except Exception as e:
            self.get_logger().error(f"Error processing LiDAR scan: {str(e)}")
            self.print_status(f"Error processing scan: {str(e)}")

    def cleanup(self):
        """Cleanup resources before shutdown"""
        try:
            if self.mqtt_client is not None:
                try:
                    self.mqtt_client.loop_stop()
                    self.mqtt_client.disconnect()
                except Exception as e:
                    self.get_logger().warning(f"Could not disconnect MQTT client: {str(e)}")
            
            self.print_status("A1M8 LiDAR Processor shutdown complete")
            
        except Exception as e:
            self.get_logger().error(f"Error during cleanup: {str(e)}")

def main(args=None):
    """Main function to run the A1M8 LiDAR processor node"""
    try:
        # Initialize ROS2
        rclpy.init(args=args)
        node = A1M8LiDARProcessor()
        
        print("=== A1M8 LIDAR PROCESSOR STARTED ===")
        print("Rear detection disabled (135°-225° sector excluded)")
        print("Filtered data published to /bcr_bot/filtered_scan")
        print("=" * 50)
        
        # Main loop
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            print('\nKeyboard interrupt received')
        except Exception as e:
            print(f'\nError in main loop: {str(e)}')
            
    except Exception as e:
        print(f'\nError in main: {str(e)}')
    finally:
        try:
            print('\n=== SHUTDOWN SEQUENCE ===')
            # Cleanup node
            if 'node' in locals():
                print("Cleaning up node...")
                node.cleanup()
                node.destroy_node()
            # Shutdown ROS2
            print("Shutting down ROS2...")
            rclpy.shutdown()
            print("✓ Cleanup completed")
        except Exception as e:
            print(f'\nError during final cleanup: {str(e)}')
        sys.exit(0)

if __name__ == '__main__':
    main() 