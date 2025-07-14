#!/usr/bin/env python3
"""
Quick test script untuk verifikasi integrasi LiDAR dan mapping
"""

import time
import json
import paho.mqtt.client as mqtt
import math

# MQTT Configuration
MQTT_BROKER = "codex.petra.ac.id"
MQTT_PORT = 1883
TOPIC_LIDAR_REAL_WORLD_DATA = "robot/lidar/real_world_data"
TOPIC_MAPPING_DATA = "robot/mapping/data"

def quick_test():
    """Quick test untuk verifikasi integrasi"""
    print("=== QUICK INTEGRATION TEST ===")
    print("Testing LiDAR and mapping integration")
    print("=" * 40)
    
    # Initialize MQTT
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
    
    try:
        print(f"Connecting to MQTT broker {MQTT_BROKER}...")
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_start()
        time.sleep(2)
        
        print("✓ MQTT connected")
        
        # Test 1: Send LiDAR data
        print("\n1. Sending LiDAR data...")
        ranges = [float('inf')] * 360
        ranges[0] = 3.0  # Wall 3m in front
        
        lidar_data = {
            "timestamp": time.time(),
            "ranges": ranges,
            "angle_min": -math.pi,
            "angle_max": math.pi,
            "angle_increment": 2 * math.pi / 360,
            "range_min": 0.15,
            "range_max": 12.0,
            "source": "real_world_lidar"
        }
        client.publish(TOPIC_LIDAR_REAL_WORLD_DATA, json.dumps(lidar_data))
        time.sleep(1)
        
        # Test 2: Send mapping command
        print("2. Sending mapping command...")
        mapping_data = {
            "command": "start_mapping",
            "timestamp": time.time()
        }
        client.publish(TOPIC_MAPPING_DATA, json.dumps(mapping_data))
        time.sleep(1)
        
        # Test 3: Send more LiDAR data
        print("3. Sending more LiDAR data...")
        ranges[90] = 2.0  # Wall 2m on left
        ranges[270] = 2.5  # Wall 2.5m on right
        
        lidar_data["ranges"] = ranges
        lidar_data["timestamp"] = time.time()
        client.publish(TOPIC_LIDAR_REAL_WORLD_DATA, json.dumps(lidar_data))
        time.sleep(1)
        
        # Test 4: Stop mapping
        print("4. Stopping mapping...")
        mapping_data = {
            "command": "stop_mapping",
            "timestamp": time.time()
        }
        client.publish(TOPIC_MAPPING_DATA, json.dumps(mapping_data))
        time.sleep(1)
        
        # Test 5: Save mapping data
        print("5. Saving mapping data...")
        mapping_data = {
            "command": "save_map",
            "timestamp": time.time()
        }
        client.publish(TOPIC_MAPPING_DATA, json.dumps(mapping_data))
        time.sleep(1)
        
        print("\n✓ Quick test completed!")
        print("Check test_robot.py output for results")
        
    except Exception as e:
        print(f"✗ Error: {str(e)}")
    finally:
        client.loop_stop()
        client.disconnect()

if __name__ == "__main__":
    quick_test() 