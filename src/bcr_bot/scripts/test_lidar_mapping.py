#!/usr/bin/env python3
"""
Simple test script untuk verifikasi integrasi LiDAR dan mapping
Digunakan untuk menguji fitur LiDAR real-world dan mapping di test_robot.py
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

def test_lidar_integration():
    """Test LiDAR integration dengan test_robot.py"""
    print("=== TEST LIDAR INTEGRATION ===")
    print("Pastikan test_robot.py sudah berjalan")
    print("=" * 50)
    
    # Initialize MQTT
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
    
    try:
        print(f"Connecting to MQTT broker {MQTT_BROKER}...")
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_start()
        time.sleep(2)
        
        print("✓ MQTT connected")
        
        # Test 1: Send empty space data
        print("\n1. Testing empty space...")
        ranges = [float('inf')] * 360
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
        time.sleep(2)
        
        # Test 2: Send wall in front
        print("2. Testing wall in front...")
        ranges[0] = 2.0  # 2m wall in front
        lidar_data["ranges"] = ranges
        lidar_data["timestamp"] = time.time()
        client.publish(TOPIC_LIDAR_REAL_WORLD_DATA, json.dumps(lidar_data))
        time.sleep(2)
        
        # Test 3: Send very close obstacle (should trigger emergency stop)
        print("3. Testing very close obstacle (should trigger emergency stop)...")
        ranges[0] = 0.2  # 20cm - very close
        lidar_data["ranges"] = ranges
        lidar_data["timestamp"] = time.time()
        client.publish(TOPIC_LIDAR_REAL_WORLD_DATA, json.dumps(lidar_data))
        time.sleep(3)
        
        # Test 4: Return to safe distance
        print("4. Returning to safe distance...")
        ranges[0] = 5.0  # 5m - safe distance
        lidar_data["ranges"] = ranges
        lidar_data["timestamp"] = time.time()
        client.publish(TOPIC_LIDAR_REAL_WORLD_DATA, json.dumps(lidar_data))
        time.sleep(2)
        
        print("✓ LiDAR integration test completed")
        
    except Exception as e:
        print(f"✗ Error: {str(e)}")
    finally:
        client.loop_stop()
        client.disconnect()

def test_mapping_integration():
    """Test mapping integration dengan test_robot.py"""
    print("\n=== TEST MAPPING INTEGRATION ===")
    print("=" * 50)
    
    # Initialize MQTT
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
    
    try:
        print(f"Connecting to MQTT broker {MQTT_BROKER}...")
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_start()
        time.sleep(2)
        
        print("✓ MQTT connected")
        
        # Test 1: Start mapping
        print("\n1. Starting mapping...")
        mapping_data = {
            "command": "start_mapping",
            "timestamp": time.time()
        }
        client.publish(TOPIC_MAPPING_DATA, json.dumps(mapping_data))
        time.sleep(2)
        
        # Test 2: Send some LiDAR data while mapping
        print("2. Sending LiDAR data during mapping...")
        ranges = [float('inf')] * 360
        ranges[0] = 3.0  # Wall 3m in front
        ranges[90] = 2.0  # Wall 2m on left
        ranges[270] = 2.5  # Wall 2.5m on right
        
        for i in range(10):
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
            time.sleep(0.5)
        
        # Test 3: Stop mapping
        print("3. Stopping mapping...")
        mapping_data = {
            "command": "stop_mapping",
            "timestamp": time.time()
        }
        client.publish(TOPIC_MAPPING_DATA, json.dumps(mapping_data))
        time.sleep(2)
        
        # Test 4: Save mapping data
        print("4. Saving mapping data...")
        mapping_data = {
            "command": "save_map",
            "timestamp": time.time()
        }
        client.publish(TOPIC_MAPPING_DATA, json.dumps(mapping_data))
        time.sleep(2)
        
        print("✓ Mapping integration test completed")
        
    except Exception as e:
        print(f"✗ Error: {str(e)}")
    finally:
        client.loop_stop()
        client.disconnect()

def main():
    """Main test function"""
    print("LiDAR & Mapping Integration Test")
    print("=" * 40)
    print("Pastikan test_robot.py sudah berjalan sebelum menjalankan test ini")
    print("=" * 40)
    
    try:
        # Test LiDAR integration
        test_lidar_integration()
        
        # Test mapping integration
        test_mapping_integration()
        
        print("\n" + "=" * 40)
        print("SEMUA TEST SELESAI!")
        print("Cek output di test_robot.py untuk melihat hasil")
        print("=" * 40)
        
    except KeyboardInterrupt:
        print("\n\nTest dihentikan oleh user")
    except Exception as e:
        print(f"\nError: {str(e)}")

if __name__ == "__main__":
    main() 