#!/usr/bin/env python3
"""
Test script sederhana untuk memverifikasi bahwa roam.py menerima data LiDAR real-world
"""

import time
import json
import math
import paho.mqtt.client as mqtt

# MQTT Configuration
MQTT_BROKER = "codex.petra.ac.id"
MQTT_PORT = 1883
MQTT_TOPIC_LIDAR_DATA = "robot/lidar/real_world_data"
MQTT_TOPIC_LIDAR_STATUS = "robot/lidar/status"

class LiDARDataTester:
    """Class untuk menguji pengiriman data LiDAR ke roam.py"""
    
    def __init__(self):
        self.mqtt_client = None
        self.init_mqtt()
    
    def init_mqtt(self):
        """Initialize MQTT connection"""
        try:
            self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
            
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
            # Publish status
            status_data = {
                "status": "connected",
                "timestamp": time.time(),
                "message": "LiDAR data tester connected"
            }
            self.mqtt_client.publish(MQTT_TOPIC_LIDAR_STATUS, json.dumps(status_data))
        else:
            print(f"[MQTT] Connection failed with code {rc}")
    
    def on_mqtt_disconnect(self, client, userdata, rc):
        print(f"[MQTT] Disconnected with code {rc}")
    
    def send_test_lidar_data(self, front_distance, left_distance, right_distance, test_name=""):
        """Send test LiDAR data to roam.py"""
        if self.mqtt_client is None:
            return
        
        try:
            # Create ranges array (360 degrees)
            ranges = [float('inf')] * 360
            
            # Set specific distances at key angles
            ranges[0] = front_distance      # Front (0 degrees)
            ranges[90] = left_distance      # Left (90 degrees)
            ranges[270] = right_distance    # Right (270 degrees)
            
            # Create data packet
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
            
            # Send via MQTT
            self.mqtt_client.publish(MQTT_TOPIC_LIDAR_DATA, json.dumps(lidar_data))
            
            print(f"[TEST] Sent LiDAR data: Front={front_distance:.2f}m, Left={left_distance:.2f}m, Right={right_distance:.2f}m ({test_name})")
            
        except Exception as e:
            print(f"[ERROR] Failed to send LiDAR data: {str(e)}")
    
    def test_basic_data_reception(self):
        """Test basic data reception"""
        print("\n=== TEST BASIC DATA RECEPTION ===")
        print("Sending simple LiDAR data to verify roam.py receives it")
        print("Expected: roam.py should show 'Real-World LiDAR' in logs")
        
        # Test 1: Clear path
        print("\n1. Sending clear path data...")
        for i in range(5):
            self.send_test_lidar_data(5.0, 3.0, 3.0, "Clear Path")
            time.sleep(1.0)
        
        # Test 2: Front obstacle
        print("\n2. Sending front obstacle data...")
        for i in range(5):
            self.send_test_lidar_data(0.8, 2.0, 2.0, "Front Obstacle")
            time.sleep(1.0)
        
        # Test 3: Side obstacles
        print("\n3. Sending side obstacle data...")
        for i in range(5):
            self.send_test_lidar_data(2.0, 0.3, 2.0, "Left Obstacle")
            time.sleep(1.0)
        
        # Test 4: Both sides blocked
        print("\n4. Sending both sides blocked data...")
        for i in range(5):
            self.send_test_lidar_data(1.0, 0.3, 0.3, "Both Sides Blocked")
            time.sleep(1.0)
        
        # Test 5: Emergency stop
        print("\n5. Sending emergency stop data...")
        for i in range(3):
            self.send_test_lidar_data(0.2, 1.5, 1.5, "Emergency Stop")
            time.sleep(1.0)
        
        # Test 6: Clear path again
        print("\n6. Sending clear path data again...")
        for i in range(5):
            self.send_test_lidar_data(5.0, 3.0, 3.0, "Clear Path")
            time.sleep(1.0)
    
    def test_continuous_data_stream(self):
        """Test continuous data streaming"""
        print("\n=== TEST CONTINUOUS DATA STREAM ===")
        print("Sending continuous LiDAR data for 30 seconds")
        print("This simulates real LiDAR streaming")
        
        start_time = time.time()
        counter = 0
        
        try:
            while time.time() - start_time < 30:
                # Simulate varying distances
                front_dist = 2.0 + math.sin(counter * 0.1) * 1.5
                left_dist = 1.5 + math.sin(counter * 0.15) * 1.0
                right_dist = 1.5 + math.cos(counter * 0.12) * 1.0
                
                self.send_test_lidar_data(front_dist, left_dist, right_dist, f"Stream {counter}")
                
                counter += 1
                time.sleep(0.5)  # Send every 0.5 seconds
                
        except KeyboardInterrupt:
            print("\n[TEST] Continuous stream interrupted by user")
    
    def run_tests(self):
        """Run all tests"""
        print("=== LIDAR DATA RECEPTION TEST ===")
        print("This script will test if roam.py receives real-world LiDAR data")
        print("Make sure roam.py is running and check its logs")
        print("=" * 60)
        
        try:
            # Wait for MQTT connection
            time.sleep(2.0)
            
            # Run tests
            self.test_basic_data_reception()
            self.test_continuous_data_stream()
            
            print("\n=== ALL TESTS COMPLETED ===")
            print("Check roam.py logs for:")
            print("- 'Real-World LiDAR' in data source")
            print("- '📡 Using Real-World LiDAR' messages")
            print("- Obstacle detection messages with real-world distances")
            
        except KeyboardInterrupt:
            print("\n[TEST] Stopped by user")
        finally:
            # Send clear path data to stop any avoidance behavior
            print("\n[TEST] Sending final clear path data...")
            for i in range(3):
                self.send_test_lidar_data(5.0, 3.0, 3.0, "Final Clear")
                time.sleep(1.0)
            
            # Cleanup
            if self.mqtt_client:
                try:
                    status_data = {
                        "status": "disconnected",
                        "timestamp": time.time(),
                        "message": "LiDAR data tester disconnected"
                    }
                    self.mqtt_client.publish(MQTT_TOPIC_LIDAR_STATUS, json.dumps(status_data))
                    self.mqtt_client.loop_stop()
                    self.mqtt_client.disconnect()
                except:
                    pass
            
            print("[TEST] Cleanup completed")

def main():
    """Main function"""
    tester = LiDARDataTester()
    tester.run_tests()

if __name__ == "__main__":
    main() 