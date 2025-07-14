#!/usr/bin/env python3
"""
Test script untuk memverifikasi bahwa test_robot.py sekarang berfungsi dengan benar
"""

import paho.mqtt.client as mqtt
import json
import time
import math

# MQTT Configuration
MQTT_BROKER = "codex.petra.ac.id"
MQTT_PORT = 1883
TOPIC_LIDAR_REAL_WORLD_DATA = "robot/lidar/real_world_data"

class TestRobotVerifier:
    def __init__(self):
        self.mqtt_client = None
        self.received_messages = []
        self.init_mqtt()
    
    def init_mqtt(self):
        """Initialize MQTT connection"""
        try:
            self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.on_message = self.on_mqtt_message
            
            print(f"[MQTT] Connecting to {MQTT_BROKER}:{MQTT_PORT}...")
            self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.mqtt_client.loop_start()
            print("[MQTT] Connected successfully")
            
        except Exception as e:
            print(f"[ERROR] MQTT connection failed: {str(e)}")
            self.mqtt_client = None
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("[MQTT] Connected to broker")
            # Subscribe to LiDAR real-world data topic
            client.subscribe(TOPIC_LIDAR_REAL_WORLD_DATA)
            print(f"[MQTT] Subscribed to {TOPIC_LIDAR_REAL_WORLD_DATA}")
        else:
            print(f"[MQTT] Connection failed with code {rc}")
    
    def on_mqtt_message(self, client, userdata, msg):
        """Handle incoming MQTT messages"""
        try:
            if msg.topic == TOPIC_LIDAR_REAL_WORLD_DATA:
                lidar_data = json.loads(msg.payload.decode())
                self.received_messages.append(lidar_data)
                
                print(f"[MQTT] Received LiDAR data:")
                print(f"  Source: {lidar_data.get('source', 'unknown')}")
                print(f"  Ranges length: {len(lidar_data.get('ranges', []))}")
                print(f"  Timestamp: {lidar_data.get('timestamp', 'unknown')}")
                
                # Check if data is valid
                ranges = lidar_data.get('ranges', [])
                valid_ranges = [r for r in ranges if r != float('inf')]
                
                if valid_ranges:
                    min_dist = min(valid_ranges)
                    max_dist = max(valid_ranges)
                    print(f"  Valid ranges: {len(valid_ranges)}/{len(ranges)}")
                    print(f"  Distance range: {min_dist:.3f}m to {max_dist:.3f}m")
                    
                    # Check specific directions
                    front_ranges = ranges[0:30] + ranges[330:360]  # 60° front
                    left_ranges = ranges[30:90]  # 60° left
                    right_ranges = ranges[270:330]  # 60° right
                    
                    front_min = min([r for r in front_ranges if r != float('inf')], default=float('inf'))
                    left_min = min([r for r in left_ranges if r != float('inf')], default=float('inf'))
                    right_min = min([r for r in right_ranges if r != float('inf')], default=float('inf'))
                    
                    print(f"  Front: {front_min:.3f}m, Left: {left_min:.3f}m, Right: {right_min:.3f}m")
                else:
                    print(f"  WARNING: No valid distance data!")
                
                print("-" * 50)
                
        except Exception as e:
            print(f"[ERROR] Failed to process message: {str(e)}")
    
    def run_test(self, duration=30):
        """Run test for specified duration"""
        print(f"=== TEST ROBOT VERIFICATION ===")
        print(f"Testing for {duration} seconds...")
        print(f"Waiting for LiDAR data from test_robot.py...")
        print("=" * 50)
        
        start_time = time.time()
        
        while time.time() - start_time < duration:
            time.sleep(1)
            
            # Print status every 5 seconds
            elapsed = time.time() - start_time
            if int(elapsed) % 5 == 0:
                print(f"[STATUS] Test running... {elapsed:.1f}s elapsed, {len(self.received_messages)} messages received")
        
        # Print final results
        print("\n=== TEST RESULTS ===")
        print(f"Total messages received: {len(self.received_messages)}")
        
        if self.received_messages:
            print("✓ test_robot.py is successfully sending LiDAR data")
            
            # Analyze data quality
            total_ranges = 0
            total_valid = 0
            
            for msg in self.received_messages:
                ranges = msg.get('ranges', [])
                total_ranges += len(ranges)
                valid_ranges = [r for r in ranges if r != float('inf')]
                total_valid += len(valid_ranges)
            
            if total_ranges > 0:
                quality_percent = (total_valid / total_ranges) * 100
                print(f"Data quality: {quality_percent:.1f}% valid ranges")
                
                if quality_percent > 50:
                    print("✓ LiDAR data quality is good")
                else:
                    print("⚠ LiDAR data quality is low - check LiDAR hardware")
        else:
            print("✗ No LiDAR data received from test_robot.py")
            print("  Possible issues:")
            print("  1. test_robot.py is not running")
            print("  2. LiDAR hardware is not connected")
            print("  3. MQTT connection issues")
            print("  4. LiDAR streaming is not started")
        
        print("=" * 50)
    
    def cleanup(self):
        """Cleanup MQTT connection"""
        if self.mqtt_client:
            try:
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
                print("[MQTT] Disconnected")
            except:
                pass

def main():
    verifier = TestRobotVerifier()
    
    try:
        verifier.run_test(30)  # Test for 30 seconds
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    finally:
        verifier.cleanup()

if __name__ == "__main__":
    main() 