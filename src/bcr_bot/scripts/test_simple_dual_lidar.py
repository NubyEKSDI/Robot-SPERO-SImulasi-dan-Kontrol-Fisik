#!/usr/bin/env python3
"""
Simple test script untuk memverifikasi logika dual LiDAR di roam.py
Mengirim data LiDAR real-world via MQTT untuk menguji logika OR
"""

import time
import json
import paho.mqtt.client as mqtt

# MQTT Configuration
MQTT_BROKER = "codex.petra.ac.id"
MQTT_PORT = 1883
MQTT_TOPIC_LIDAR_DATA = "robot/lidar/real_world_data"

def send_lidar_data(mqtt_client, front_distance, left_distance, right_distance):
    """Send LiDAR data with specific distances"""
    try:
        # Create ranges array with 360 points
        ranges = [float('inf')] * 360
        
        # Set front distance (angle 0°)
        if front_distance != float('inf'):
            ranges[0] = front_distance
            # Also set nearby angles for better detection
            for i in range(1, 11):
                ranges[i] = front_distance
                ranges[360-i] = front_distance
        
        # Set left distance (angle 90°)
        if left_distance != float('inf'):
            left_idx = 90
            ranges[left_idx] = left_distance
            # Also set nearby angles
            for i in range(1, 11):
                ranges[(left_idx + i) % 360] = left_distance
                ranges[(left_idx - i) % 360] = left_distance
        
        # Set right distance (angle 270°)
        if right_distance != float('inf'):
            right_idx = 270
            ranges[right_idx] = right_distance
            # Also set nearby angles
            for i in range(1, 11):
                ranges[(right_idx + i) % 360] = right_distance
                ranges[(right_idx - i) % 360] = right_distance
        
        # Create data packet
        lidar_data = {
            "timestamp": time.time(),
            "ranges": ranges,
            "angle_min": -3.14159,
            "angle_max": 3.14159,
            "angle_increment": 0.0174533,
            "range_min": 0.15,
            "range_max": 12.0,
            "source": "real_world_lidar"
        }
        
        # Send via MQTT
        mqtt_client.publish(MQTT_TOPIC_LIDAR_DATA, json.dumps(lidar_data))
        
        print(f"[LIDAR] Sent data - Front: {front_distance:.2f}m, Left: {left_distance:.2f}m, Right: {right_distance:.2f}m")
        
    except Exception as e:
        print(f"[ERROR] Failed to send LiDAR data: {str(e)}")

def main():
    """Main function"""
    print("Simple Dual LiDAR Test")
    print("=" * 30)
    print("Testing OR logic: if either Gazebo OR real-world LiDAR detects obstacle → avoid")
    print("=" * 30)
    
    # Initialize MQTT
    try:
        mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        mqtt_client.loop_start()
        print("[MQTT] Connected successfully")
    except Exception as e:
        print(f"[ERROR] MQTT connection failed: {str(e)}")
        return
    
    try:
        # Wait for connection
        time.sleep(2)
        
        print("\n=== TESTING DUAL LIDAR LOGIC ===")
        print("Make sure roam.py is running and check its logs")
        print("Expected: Robot should avoid obstacles detected by either LiDAR source")
        
        # Test 1: Clear path
        print("\n1. Sending clear path (no obstacles)...")
        send_lidar_data(mqtt_client, float('inf'), float('inf'), float('inf'))
        time.sleep(3)
        
        # Test 2: Front obstacle
        print("\n2. Sending front obstacle at 0.5m...")
        send_lidar_data(mqtt_client, 0.5, float('inf'), float('inf'))
        time.sleep(5)
        
        # Test 3: Left obstacle
        print("\n3. Sending left obstacle at 0.3m...")
        send_lidar_data(mqtt_client, float('inf'), 0.3, float('inf'))
        time.sleep(5)
        
        # Test 4: Right obstacle
        print("\n4. Sending right obstacle at 0.3m...")
        send_lidar_data(mqtt_client, float('inf'), float('inf'), 0.3)
        time.sleep(5)
        
        # Test 5: Emergency stop (very close)
        print("\n5. Sending emergency stop obstacle at 0.2m...")
        send_lidar_data(mqtt_client, 0.2, float('inf'), float('inf'))
        time.sleep(5)
        
        # Test 6: Clear path again
        print("\n6. Sending clear path again...")
        send_lidar_data(mqtt_client, float('inf'), float('inf'), float('inf'))
        time.sleep(3)
        
        print("\n=== TEST COMPLETED ===")
        print("Check roam.py logs to verify:")
        print("- Robot detected obstacles from real-world LiDAR")
        print("- Robot avoided obstacles (turned or stopped)")
        print("- Logs show 'Real-World LiDAR' as the source")
        print("- Logs show 'DUAL LIDAR ACTIVE' when both sources are available")
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\nError: {str(e)}")
    finally:
        # Cleanup
        try:
            mqtt_client.loop_stop()
            mqtt_client.disconnect()
        except:
            pass
        print("\nTest completed")

if __name__ == "__main__":
    main() 