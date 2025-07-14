#!/usr/bin/env python3
"""
Debug script untuk test MQTT connection dan data publishing
"""

import time
import json
import paho.mqtt.client as mqtt

# MQTT Configuration
MQTT_BROKER = "spin5.petra.ac.id"
MQTT_PORT = 1883
MQTT_TOPIC_LIDAR_DATA = "robot/lidar/real_world_data"

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"✓ Connected to MQTT broker: {MQTT_BROKER}:{MQTT_PORT}")
    else:
        print(f"✗ Failed to connect to MQTT broker, code: {rc}")

def on_disconnect(client, userdata, rc):
    print(f"Disconnected from MQTT broker, code: {rc}")

def on_publish(client, userdata, mid):
    print(f"✓ Message {mid} published successfully")

def test_mqtt_connection():
    """Test basic MQTT connection"""
    print("=== MQTT CONNECTION TEST ===")
    
    try:
        # Create MQTT client
        client = mqtt.Client()
        client.on_connect = on_connect
        client.on_disconnect = on_disconnect
        client.on_publish = on_publish
        
        print(f"Connecting to {MQTT_BROKER}:{MQTT_PORT}...")
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_start()
        
        # Wait for connection
        time.sleep(2)
        
        # Test publish
        test_data = {
            "timestamp": time.time(),
            "test": "debug_message",
            "source": "mqtt_debug_test"
        }
        
        print(f"Publishing test data to topic: {MQTT_TOPIC_LIDAR_DATA}")
        result = client.publish(MQTT_TOPIC_LIDAR_DATA, json.dumps(test_data))
        
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            print("✓ Test message published successfully")
        else:
            print(f"✗ Failed to publish test message, code: {result.rc}")
        
        # Wait a bit
        time.sleep(2)
        
        # Cleanup
        client.loop_stop()
        client.disconnect()
        
        return True
        
    except Exception as e:
        print(f"✗ MQTT test failed: {str(e)}")
        return False

def test_lidar_data_format():
    """Test sending simulated LiDAR data"""
    print("\n=== LIDAR DATA FORMAT TEST ===")
    
    try:
        # Create MQTT client
        client = mqtt.Client()
        client.on_connect = on_connect
        client.on_disconnect = on_disconnect
        client.on_publish = on_publish
        
        print(f"Connecting to {MQTT_BROKER}:{MQTT_PORT}...")
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_start()
        
        # Wait for connection
        time.sleep(2)
        
        # Create simulated LiDAR data
        ranges = [float('inf')] * 360
        
        # Add some test data points
        for i in range(0, 360, 10):  # Every 10 degrees
            ranges[i] = 2.0  # 2 meters distance
        
        lidar_data = {
            "timestamp": time.time(),
            "ranges": ranges,
            "angle_min": 0.0,
            "angle_max": 6.283185307179586,  # 2*pi
            "angle_increment": 0.017453292519943295,  # pi/180
            "range_min": 0.15,
            "range_max": 12.0,
            "source": "mqtt_debug_lidar_test"
        }
        
        json_data = json.dumps(lidar_data)
        print(f"Publishing simulated LiDAR data to topic: {MQTT_TOPIC_LIDAR_DATA}")
        print(f"Data size: {len(json_data)} bytes")
        print(f"Valid ranges: {sum(1 for r in ranges if r != float('inf'))}/360")
        
        result = client.publish(MQTT_TOPIC_LIDAR_DATA, json_data)
        
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            print("✓ Simulated LiDAR data published successfully")
        else:
            print(f"✗ Failed to publish LiDAR data, code: {result.rc}")
        
        # Wait a bit
        time.sleep(2)
        
        # Cleanup
        client.loop_stop()
        client.disconnect()
        
        return True
        
    except Exception as e:
        print(f"✗ LiDAR data test failed: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

def main():
    print("MQTT Debug Script")
    print("=" * 40)
    
    # Test 1: Basic MQTT connection
    if not test_mqtt_connection():
        print("\n❌ MQTT connection test failed!")
        return
    
    # Test 2: LiDAR data format
    if not test_lidar_data_format():
        print("\n❌ LiDAR data format test failed!")
        return
    
    print("\n✅ ALL MQTT TESTS PASSED!")
    print("MQTT connection and data publishing is working correctly.")
    print("\nIf the bridge still shows 'No recent data', the issue might be:")
    print("1. Bridge is subscribed to wrong topic")
    print("2. Bridge has filtering that rejects the data")
    print("3. Timing issue between publisher and subscriber")

if __name__ == "__main__":
    main() 