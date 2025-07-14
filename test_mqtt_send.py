#!/usr/bin/env python3
"""
Test script untuk mengirim data MQTT manual
Untuk memastikan mqtt_to_ros2_bridge berfungsi dengan benar
"""

import paho.mqtt.client as mqtt
import json
import time
import math

MQTT_BROKER = "spin5.petra.ac.id"
MQTT_PORT = 1883
MQTT_TOPIC = "robot/lidar/real_world_data"

def send_test_data():
    """Kirim data test ke MQTT"""
    try:
        # Connect to MQTT
        client = mqtt.Client()
        print(f"Connecting to {MQTT_BROKER}:{MQTT_PORT}...")
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        
        # Generate test lidar data (lingkaran dengan jari-jari 2 meter)
        ranges = []
        for i in range(360):
            # Simulasi objek di jarak 2 meter di semua arah
            distance = 2.0 + 0.5 * math.sin(math.radians(i * 4))  # Variasi jarak
            ranges.append(distance)
        
        # Format data sesuai yang diharapkan bridge
        data = {
            "timestamp": time.time(),
            "ranges": ranges,
            "angle_min": 0.0,
            "angle_max": 2 * math.pi,
            "angle_increment": math.radians(1),
            "range_min": 0.15,
            "range_max": 12.0,
            "source": "test_data"
        }
        
        # Kirim data
        print(f"Sending test data to topic: {MQTT_TOPIC}")
        client.publish(MQTT_TOPIC, json.dumps(data))
        print("✓ Test data sent successfully!")
        
        # Disconnect
        client.disconnect()
        
    except Exception as e:
        print(f"✗ Error sending test data: {e}")

if __name__ == "__main__":
    print("=== MQTT Test Data Sender ===")
    print("Sending test lidar data to MQTT broker")
    print("This should appear in RViz if bridge is working")
    print("=" * 40)
    
    send_test_data()
    
    print("\nIf bridge is running, you should see:")
    print("- Bridge log: 'Published 1 scans to ROS2'")
    print("- RViz: Circle pattern of points") 