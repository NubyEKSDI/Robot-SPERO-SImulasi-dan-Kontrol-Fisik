#!/usr/bin/env python3
"""
MQTT to ROS2 Bridge untuk RPLIDAR A1M8
Subscribe ke MQTT topic dari robot fisik, publish ke ROS2 topic /scan
Agar data lidar bisa divisualisasikan di RViz
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import paho.mqtt.client as mqtt
import json
import threading
import time
import numpy as np
import signal
import sys
import os

class MQTTToROS2Bridge(Node):
    def __init__(self):
        super().__init__('mqtt_to_ros2_bridge')
        
        # ROS2 Publishers
        self.scan_publisher = self.create_publisher(LaserScan, 'scan', 10)
        self.status_publisher = self.create_publisher(String, 'bridge_status', 10)
        
        # MQTT Configuration
        self.mqtt_broker = "spin5.petra.ac.id"
        self.mqtt_port = 1883
        self.mqtt_topic = "robot/lidar/real_world_data"
        
        # Performance tracking
        self.scan_count = 0
        self.last_status_time = time.time()
        
        # Throttling to prevent queue overflow
        self.last_publish_time = 0
        
        # Check for mapping mode from environment variable or parameter
        import os
        env_mapping_mode = os.environ.get('MAPPING_MODE', 'false').lower() == 'true'
        
        self.declare_parameter('mapping_mode', env_mapping_mode)
        self.mapping_mode = self.get_parameter('mapping_mode').get_parameter_value().bool_value
        self.publish_interval = 0.2 if self.mapping_mode else 0.1  # 5Hz for mapping, 10Hz for visualization
        
        # Initialize MQTT client
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        self.mqtt_client.on_subscribe = self.on_mqtt_subscribe
        self.mqtt_client.on_message = self.on_mqtt_message
        
        # Connect to MQTT
        self.connect_mqtt()
        
        # Start status publishing timer
        self.status_timer = self.create_timer(10.0, self.publish_status)
        
        # Handle shutdown gracefully
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        mode_str = "MAPPING" if self.mapping_mode else "VISUALIZATION"
        self.get_logger().info(f'A1M8 MQTT-ROS2 Bridge started in {mode_str} mode')
        self.get_logger().info(f'Publishing at {1/self.publish_interval:.1f}Hz for optimal performance')
    
    def connect_mqtt(self):
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info(f'Connecting to MQTT broker at {self.mqtt_broker}:{self.mqtt_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to MQTT broker: {e}')
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info('Connected to MQTT broker successfully')
            client.subscribe(self.mqtt_topic)
            self.get_logger().info(f'Subscribed to topic: {self.mqtt_topic}')
        else:
            self.get_logger().error(f'Failed to connect to MQTT broker. Return code: {rc}')
    
    def on_mqtt_disconnect(self, client, userdata, rc):
        if rc != 0:
            self.get_logger().warn(f'Unexpected disconnection from MQTT broker: {rc}')
        else:
            self.get_logger().info('Disconnected from MQTT broker')
    
    def on_mqtt_subscribe(self, client, userdata, mid, granted_qos):
        self.get_logger().info(f'Subscription confirmed with QoS: {granted_qos}')
    
    def on_mqtt_message(self, client, userdata, msg):
        try:
            # Check throttling to prevent queue overflow
            current_time = time.time()
            if current_time - self.last_publish_time < self.publish_interval:
                return  # Skip this message to reduce frequency
            
            self.last_publish_time = current_time
            
            # Decode and parse JSON
            json_data = msg.payload.decode('utf-8')
            data = json.loads(json_data)
            
            # Convert to LaserScan and publish
            self.publish_laser_scan(data)
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON decode error: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing MQTT message: {e}')
    
    def publish_laser_scan(self, data):
        try:
            # Create LaserScan message
            scan_msg = LaserScan()
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.header.frame_id = 'laser'
            
            # Extract ranges from data
            ranges = data.get('ranges', [])
            if not ranges:
                self.get_logger().warn('No range data in MQTT message')
                return
            
            # A1M8 parameters (from MQTT data or defaults)
            scan_msg.angle_min = float(data.get('angle_min', 0.0))
            scan_msg.angle_max = float(data.get('angle_max', 2 * np.pi))
            scan_msg.range_min = float(data.get('range_min', 0.15))
            scan_msg.range_max = float(data.get('range_max', 12.0))
            
            # Standard resolution to prevent queue overflow
            scan_msg.ranges = ranges
            scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / len(ranges)
            scan_msg.intensities = []
            
            # Publish the scan
            self.scan_publisher.publish(scan_msg)
            self.scan_count += 1
            
            # Reduced logging to prevent spam
            if self.scan_count % 50 == 0:
                self.get_logger().info(f'Published {self.scan_count} scans to ROS2')
            
        except Exception as e:
            self.get_logger().error(f'Error publishing LaserScan: {e}')
    
    def publish_status(self):
        try:
            current_time = time.time()
            time_since_last = current_time - self.last_status_time
            
            if time_since_last > 30:  # No data for 30 seconds
                status_msg = String()
                status_msg.data = "No A1M8 data received - Check Raspberry Pi connection"
                self.status_publisher.publish(status_msg)
                self.get_logger().warn('No recent A1M8 data - Check MQTT connection')
            else:
                status_msg = String()
                mode_str = "MAPPING" if self.mapping_mode else "VISUALIZATION"
                status_msg.data = f"Bridge OK - {mode_str} mode - Receiving data at {1/self.publish_interval:.1f}Hz"
                self.status_publisher.publish(status_msg)
                self.get_logger().info(f'Bridge OK - {mode_str} mode - Receiving data')
            
            self.last_status_time = current_time
            
        except Exception as e:
            self.get_logger().error(f'Error publishing status: {e}')

    def signal_handler(self, signum, frame):
        self.get_logger().info('Shutting down MQTT-ROS2 Bridge...')
        self.destroy_node()
        sys.exit(0)
    
    def destroy_node(self):
        try:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
            self.get_logger().info('MQTT client disconnected')
        except Exception as e:
            self.get_logger().error(f'Error during cleanup: {e}')
        finally:
            super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        bridge = MQTTToROS2Bridge()
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 