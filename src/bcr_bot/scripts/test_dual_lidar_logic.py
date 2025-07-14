#!/usr/bin/env python3
"""
Test script untuk memverifikasi logika dual LiDAR di roam.py
Mengirim data LiDAR real-world via MQTT untuk menguji logika OR:
- Jika Gazebo ATAU real-world LiDAR mendeteksi obstacle → robot avoid
- Bukan menggabungkan data, tapi menggunakan logika OR
"""

import time
import json
import paho.mqtt.client as mqtt
import random
import math

# MQTT Configuration
MQTT_BROKER = "codex.petra.ac.id"
MQTT_PORT = 1883
MQTT_TOPIC_LIDAR_DATA = "robot/lidar/real_world_data"

class DualLiDARTester:
    """Test class untuk memverifikasi logika dual LiDAR"""
    
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
        else:
            print(f"[MQTT] Connection failed with code {rc}")
    
    def on_mqtt_disconnect(self, client, userdata, rc):
        print(f"[MQTT] Disconnected with code {rc}")
    
    def send_lidar_data(self, front_distance, left_distance, right_distance):
        """Send LiDAR data with specific distances"""
        if self.mqtt_client is None:
            return
        
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
            self.mqtt_client.publish(MQTT_TOPIC_LIDAR_DATA, json.dumps(lidar_data))
            
            print(f"[LIDAR] Sent data - Front: {front_distance:.2f}m, Left: {left_distance:.2f}m, Right: {right_distance:.2f}m")
            
        except Exception as e:
            print(f"[ERROR] Failed to send LiDAR data: {str(e)}")
    
    def test_scenario_1_clear_path(self):
        """Test 1: Clear path - no obstacles from real-world LiDAR"""
        print("\n=== TEST 1: CLEAR PATH ===")
        print("Real-World LiDAR: No obstacles (all inf)")
        print("Expected: Robot should move forward (using Gazebo data if available)")
        
        self.send_lidar_data(float('inf'), float('inf'), float('inf'))
        time.sleep(3)
    
    def test_scenario_2_front_obstacle(self):
        """Test 2: Front obstacle detected by real-world LiDAR"""
        print("\n=== TEST 2: FRONT OBSTACLE ===")
        print("Real-World LiDAR: Front obstacle at 0.5m")
        print("Expected: Robot should detect obstacle and turn")
        
        self.send_lidar_data(0.5, float('inf'), float('inf'))
        time.sleep(5)
    
    def test_scenario_3_left_obstacle(self):
        """Test 3: Left obstacle detected by real-world LiDAR"""
        print("\n=== TEST 3: LEFT OBSTACLE ===")
        print("Real-World LiDAR: Left obstacle at 0.3m")
        print("Expected: Robot should detect left obstacle and turn right")
        
        self.send_lidar_data(float('inf'), 0.3, float('inf'))
        time.sleep(5)
    
    def test_scenario_4_right_obstacle(self):
        """Test 4: Right obstacle detected by real-world LiDAR"""
        print("\n=== TEST 4: RIGHT OBSTACLE ===")
        print("Real-World LiDAR: Right obstacle at 0.3m")
        print("Expected: Robot should detect right obstacle and turn left")
        
        self.send_lidar_data(float('inf'), float('inf'), 0.3)
        time.sleep(5)
    
    def test_scenario_5_both_sides_blocked(self):
        """Test 5: Both sides blocked by real-world LiDAR"""
        print("\n=== TEST 5: BOTH SIDES BLOCKED ===")
        print("Real-World LiDAR: Both sides blocked (left: 0.2m, right: 0.2m)")
        print("Expected: Robot should detect both sides blocked and scan 360°")
        
        self.send_lidar_data(float('inf'), 0.2, 0.2)
        time.sleep(5)
    
    def test_scenario_6_close_front_obstacle(self):
        """Test 6: Very close front obstacle (emergency stop)"""
        print("\n=== TEST 6: EMERGENCY STOP ===")
        print("Real-World LiDAR: Very close front obstacle at 0.2m")
        print("Expected: Robot should emergency stop")
        
        self.send_lidar_data(0.2, float('inf'), float('inf'))
        time.sleep(5)
    
    def test_scenario_7_real_world_closer_than_gazebo(self):
        """Test 7: Real-world LiDAR detects closer obstacle than Gazebo"""
        print("\n=== TEST 7: REAL-WORLD CLOSER THAN GAZEBO ===")
        print("Real-World LiDAR: Front obstacle at 0.8m")
        print("Gazebo: Front obstacle at 1.5m (simulated)")
        print("Expected: Robot should use real-world data (0.8m) and avoid")
        
        self.send_lidar_data(0.8, float('inf'), float('inf'))
        time.sleep(5)
    
    def test_scenario_8_gazebo_closer_than_real_world(self):
        """Test 8: Gazebo detects closer obstacle than real-world LiDAR"""
        print("\n=== TEST 8: GAZEBO CLOSER THAN REAL-WORLD ===")
        print("Real-World LiDAR: Front obstacle at 2.0m")
        print("Gazebo: Front obstacle at 0.6m (simulated)")
        print("Expected: Robot should use Gazebo data (0.6m) and avoid")
        
        self.send_lidar_data(2.0, float('inf'), float('inf'))
        time.sleep(5)
    
    def test_scenario_9_continuous_obstacle_detection(self):
        """Test 9: Continuous obstacle detection and avoidance"""
        print("\n=== TEST 9: CONTINUOUS OBSTACLE DETECTION ===")
        print("Real-World LiDAR: Moving obstacle simulation")
        print("Expected: Robot should continuously avoid obstacles")
        
        for i in range(10):
            # Simulate moving obstacle
            distance = 1.0 + 0.5 * math.sin(i * 0.5)
            self.send_lidar_data(distance, float('inf'), float('inf'))
            print(f"[LIDAR] Moving obstacle at {distance:.2f}m")
            time.sleep(1)
    
    def run_all_tests(self):
        """Run all test scenarios"""
        print("=== DUAL LIDAR LOGIC TEST ===")
        print("Testing OR logic: if either Gazebo OR real-world LiDAR detects obstacle → avoid")
        print("=" * 60)
        
        if self.mqtt_client is None:
            print("[ERROR] MQTT not connected, cannot run tests")
            return
        
        try:
            # Run all test scenarios
            self.test_scenario_1_clear_path()
            self.test_scenario_2_front_obstacle()
            self.test_scenario_3_left_obstacle()
            self.test_scenario_4_right_obstacle()
            self.test_scenario_5_both_sides_blocked()
            self.test_scenario_6_close_front_obstacle()
            self.test_scenario_7_real_world_closer_than_gazebo()
            self.test_scenario_8_gazebo_closer_than_real_world()
            self.test_scenario_9_continuous_obstacle_detection()
            
            print("\n=== ALL TESTS COMPLETED ===")
            print("Check roam.py logs to verify dual LiDAR logic is working correctly")
            print("Expected behavior:")
            print("- Robot should avoid obstacles detected by either LiDAR source")
            print("- Robot should use the closer distance when both sources detect obstacles")
            print("- Robot should show which LiDAR source is being used in logs")
            
        except KeyboardInterrupt:
            print("\n[TEST] Tests interrupted by user")
        except Exception as e:
            print(f"[ERROR] Test error: {str(e)}")
        finally:
            # Send clear path at the end
            self.send_lidar_data(float('inf'), float('inf'), float('inf'))
            print("[TEST] Sent clear path signal")
    
    def cleanup(self):
        """Cleanup MQTT connection"""
        if self.mqtt_client:
            try:
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
            except:
                pass

def main():
    """Main function"""
    print("Dual LiDAR Logic Test Script")
    print("=" * 40)
    print("This script tests the OR logic in roam.py:")
    print("- If Gazebo OR real-world LiDAR detects obstacle → robot avoids")
    print("- Robot uses the closer distance when both sources detect obstacles")
    print("=" * 40)
    
    tester = DualLiDARTester()
    
    try:
        # Wait for MQTT connection
        time.sleep(2)
        
        if tester.mqtt_client is None:
            print("[ERROR] Failed to connect to MQTT broker")
            return
        
        # Run tests
        tester.run_all_tests()
        
    except KeyboardInterrupt:
        print("\n\nScript interrupted by user")
    except Exception as e:
        print(f"\nError: {str(e)}")
    finally:
        tester.cleanup()
        print("\nTest completed")

if __name__ == "__main__":
    main() 