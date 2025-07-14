#!/usr/bin/env python3
"""
Test script untuk memverifikasi integrasi data Gazebo dan LiDAR real-world di roam.py
Mengirim data LiDAR simulasi untuk menguji semua mode navigasi
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

class RoamIntegrationTester:
    """Class untuk menguji integrasi roam.py dengan data LiDAR real-world"""
    
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
                "message": "Roam integration tester connected"
            }
            self.mqtt_client.publish(MQTT_TOPIC_LIDAR_STATUS, json.dumps(status_data))
        else:
            print(f"[MQTT] Connection failed with code {rc}")
    
    def on_mqtt_disconnect(self, client, userdata, rc):
        print(f"[MQTT] Disconnected with code {rc}")
    
    def send_lidar_data(self, front_distance, left_distance, right_distance, scenario_name=""):
        """Send simulated LiDAR data to roam.py"""
        if self.mqtt_client is None:
            return
        
        try:
            # Create ranges array (360 degrees)
            ranges = [float('inf')] * 360
            
            # Set front distance (0 degrees)
            ranges[0] = front_distance
            
            # Set left distance (90 degrees)
            ranges[90] = left_distance
            
            # Set right distance (270 degrees)
            ranges[270] = right_distance
            
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
            
            print(f"[TEST] Sent LiDAR data: Front={front_distance:.2f}m, Left={left_distance:.2f}m, Right={right_distance:.2f}m ({scenario_name})")
            
        except Exception as e:
            print(f"[ERROR] Failed to send LiDAR data: {str(e)}")
    
    def test_scenario_1_clear_path(self):
        """Test 1: Clear path - no obstacles"""
        print("\n=== TEST SCENARIO 1: Clear Path ===")
        print("Sending data: Front=5.0m, Left=3.0m, Right=3.0m")
        print("Expected: Robot should move forward normally")
        
        for i in range(10):  # Send for 10 seconds
            self.send_lidar_data(5.0, 3.0, 3.0, "Clear Path")
            time.sleep(1.0)
    
    def test_scenario_2_front_obstacle(self):
        """Test 2: Front obstacle"""
        print("\n=== TEST SCENARIO 2: Front Obstacle ===")
        print("Sending data: Front=0.8m, Left=2.0m, Right=2.0m")
        print("Expected: Robot should turn left or right to avoid")
        
        for i in range(10):  # Send for 10 seconds
            self.send_lidar_data(0.8, 2.0, 2.0, "Front Obstacle")
            time.sleep(1.0)
    
    def test_scenario_3_emergency_stop(self):
        """Test 3: Emergency stop - very close front obstacle"""
        print("\n=== TEST SCENARIO 3: Emergency Stop ===")
        print("Sending data: Front=0.2m, Left=1.5m, Right=1.5m")
        print("Expected: Robot should emergency stop")
        
        for i in range(5):  # Send for 5 seconds
            self.send_lidar_data(0.2, 1.5, 1.5, "Emergency Stop")
            time.sleep(1.0)
    
    def test_scenario_4_left_obstacle(self):
        """Test 4: Left side obstacle"""
        print("\n=== TEST SCENARIO 4: Left Side Obstacle ===")
        print("Sending data: Front=2.0m, Left=0.3m, Right=2.0m")
        print("Expected: Robot should turn right to avoid")
        
        for i in range(10):  # Send for 10 seconds
            self.send_lidar_data(2.0, 0.3, 2.0, "Left Obstacle")
            time.sleep(1.0)
    
    def test_scenario_5_right_obstacle(self):
        """Test 5: Right side obstacle"""
        print("\n=== TEST SCENARIO 5: Right Side Obstacle ===")
        print("Sending data: Front=2.0m, Left=2.0m, Right=0.3m")
        print("Expected: Robot should turn left to avoid")
        
        for i in range(10):  # Send for 10 seconds
            self.send_lidar_data(2.0, 2.0, 0.3, "Right Obstacle")
            time.sleep(1.0)
    
    def test_scenario_6_both_sides_blocked(self):
        """Test 6: Both sides blocked"""
        print("\n=== TEST SCENARIO 6: Both Sides Blocked ===")
        print("Sending data: Front=1.0m, Left=0.3m, Right=0.3m")
        print("Expected: Robot should scan 360 degrees or backup")
        
        for i in range(10):  # Send for 10 seconds
            self.send_lidar_data(1.0, 0.3, 0.3, "Both Sides Blocked")
            time.sleep(1.0)
    
    def test_scenario_7_charging_navigation(self):
        """Test 7: Charging navigation with obstacles"""
        print("\n=== TEST SCENARIO 7: Charging Navigation ===")
        print("Sending data: Front=0.6m, Left=1.0m, Right=1.0m")
        print("Expected: Robot should avoid obstacles while navigating to charging station")
        print("Note: Press 't' in roam.py to activate charging navigation mode")
        
        for i in range(15):  # Send for 15 seconds
            self.send_lidar_data(0.6, 1.0, 1.0, "Charging Navigation")
            time.sleep(1.0)
    
    def test_scenario_8_r_mode_navigation(self):
        """Test 8: R-mode navigation with obstacles"""
        print("\n=== TEST SCENARIO 8: R-Mode Navigation ===")
        print("Sending data: Front=0.7m, Left=0.8m, Right=0.8m")
        print("Expected: Robot should avoid obstacles while navigating to waypoints")
        print("Note: Press 'r' in roam.py to activate R-mode")
        
        for i in range(15):  # Send for 15 seconds
            self.send_lidar_data(0.7, 0.8, 0.8, "R-Mode Navigation")
            time.sleep(1.0)
    
    def test_scenario_9_secondary_control(self):
        """Test 9: Secondary control activation"""
        print("\n=== TEST SCENARIO 9: Secondary Control ===")
        print("Sending data: Front=2.0m, Left=0.4m, Right=2.0m")
        print("Expected: Secondary control should activate for 40 seconds")
        
        for i in range(15):  # Send for 15 seconds
            self.send_lidar_data(2.0, 0.4, 2.0, "Secondary Control")
            time.sleep(1.0)
    
    def test_scenario_10_dynamic_obstacles(self):
        """Test 10: Dynamic obstacles - changing distances"""
        print("\n=== TEST SCENARIO 10: Dynamic Obstacles ===")
        print("Sending data: Changing distances to simulate moving obstacles")
        print("Expected: Robot should adapt to changing obstacle positions")
        
        for i in range(20):  # Send for 20 seconds
            # Simulate obstacle moving closer then farther
            front_dist = 2.0 + math.sin(i * 0.5) * 1.5  # Varies between 0.5 and 3.5m
            left_dist = 1.5 + math.sin(i * 0.3) * 1.0   # Varies between 0.5 and 2.5m
            right_dist = 1.5 + math.cos(i * 0.4) * 1.0  # Varies between 0.5 and 2.5m
            
            self.send_lidar_data(front_dist, left_dist, right_dist, f"Dynamic {i}")
            time.sleep(1.0)
    
    def run_all_tests(self):
        """Run all test scenarios"""
        print("=== ROAM.PY INTEGRATION TEST ===")
        print("This script will test roam.py's integration with real-world LiDAR data")
        print("Make sure roam.py is running and listening to MQTT topics")
        print("=" * 60)
        
        try:
            # Wait for MQTT connection
            time.sleep(2.0)
            
            # Run all test scenarios
            self.test_scenario_1_clear_path()
            self.test_scenario_2_front_obstacle()
            self.test_scenario_3_emergency_stop()
            self.test_scenario_4_left_obstacle()
            self.test_scenario_5_right_obstacle()
            self.test_scenario_6_both_sides_blocked()
            self.test_scenario_7_charging_navigation()
            self.test_scenario_8_r_mode_navigation()
            self.test_scenario_9_secondary_control()
            self.test_scenario_10_dynamic_obstacles()
            
            print("\n=== ALL TESTS COMPLETED ===")
            print("Check roam.py output to verify obstacle avoidance behavior")
            print("Expected behaviors:")
            print("- Clear path: Robot moves forward")
            print("- Front obstacle: Robot turns left/right")
            print("- Emergency stop: Robot stops immediately")
            print("- Side obstacles: Robot turns away from obstacle")
            print("- Both sides blocked: Robot scans or backs up")
            print("- Charging/R-mode: Robot avoids obstacles while navigating")
            print("- Secondary control: Activates for side obstacles")
            
        except KeyboardInterrupt:
            print("\n[TEST] Stopped by user")
        finally:
            # Send clear path data to stop any avoidance behavior
            print("\n[TEST] Sending clear path data to stop robot...")
            for i in range(5):
                self.send_lidar_data(5.0, 3.0, 3.0, "Clear Path (Stop)")
                time.sleep(1.0)
            
            # Cleanup
            if self.mqtt_client:
                try:
                    status_data = {
                        "status": "disconnected",
                        "timestamp": time.time(),
                        "message": "Roam integration tester disconnected"
                    }
                    self.mqtt_client.publish(MQTT_TOPIC_LIDAR_STATUS, json.dumps(status_data))
                    self.mqtt_client.loop_stop()
                    self.mqtt_client.disconnect()
                except:
                    pass
            
            print("[TEST] Cleanup completed")

def main():
    """Main function"""
    tester = RoamIntegrationTester()
    tester.run_all_tests()

if __name__ == "__main__":
    main() 