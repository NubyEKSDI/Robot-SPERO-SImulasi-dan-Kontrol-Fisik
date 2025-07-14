#!/usr/bin/env python3
"""
Test script untuk memverifikasi sistem LiDAR real-world ke roam.py
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

class LiDARRealWorldTest:
    """Test class untuk memverifikasi sistem LiDAR real-world"""
    
    def __init__(self):
        self.mqtt_client = None
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
            # Subscribe to status topic
            client.subscribe(MQTT_TOPIC_LIDAR_STATUS)
        else:
            print(f"[MQTT] Connection failed with code {rc}")
    
    def on_mqtt_message(self, client, userdata, msg):
        """Handle incoming MQTT messages"""
        if msg.topic == MQTT_TOPIC_LIDAR_STATUS:
            try:
                status_data = json.loads(msg.payload.decode())
                print(f"[STATUS] {status_data.get('message', 'Unknown status')}")
            except Exception as e:
                print(f"[ERROR] Failed to parse status message: {str(e)}")
    
    def send_test_lidar_data(self, ranges):
        """Send test LiDAR data to roam.py"""
        if self.mqtt_client is None:
            print("[ERROR] MQTT not connected")
            return False
        
        try:
            # Create test LiDAR data
            lidar_data = {
                "timestamp": time.time(),
                "ranges": ranges,
                "angle_min": -math.pi,
                "angle_max": math.pi,
                "angle_increment": 2 * math.pi / 360,
                "range_min": 0.15,
                "range_max": 12.0,
                "source": "test_lidar_data"
            }
            
            # Send via MQTT
            self.mqtt_client.publish(MQTT_TOPIC_LIDAR_DATA, json.dumps(lidar_data))
            return True
            
        except Exception as e:
            print(f"[ERROR] Failed to send test LiDAR data: {str(e)}")
            return False
    
    def create_test_obstacle_data(self, obstacle_type="wall"):
        """Create test obstacle data"""
        ranges = [float('inf')] * 360
        
        if obstacle_type == "wall":
            # Create a wall in front (0 degrees)
            for angle in range(-10, 11):  # 20 degree wall
                idx = (angle + 360) % 360
                ranges[idx] = 2.0  # Wall at 2 meters
            
        elif obstacle_type == "corner":
            # Create a corner (wall on left and front)
            # Front wall
            for angle in range(-10, 11):
                idx = (angle + 360) % 360
                ranges[idx] = 2.0
            
            # Left wall
            for angle in range(80, 101):  # 90 degrees
                idx = (angle + 360) % 360
                ranges[idx] = 1.5
        
        elif obstacle_type == "narrow_passage":
            # Create a narrow passage
            for angle in range(-5, 6):  # 10 degree passage
                idx = (angle + 360) % 360
                ranges[idx] = 3.0  # Clear path
            
            # Add obstacles on sides
            for angle in range(-90, -5):
                idx = (angle + 360) % 360
                ranges[idx] = 0.8  # Close obstacle on left
            
            for angle in range(6, 91):
                idx = (angle + 360) % 360
                ranges[idx] = 0.8  # Close obstacle on right
        
        elif obstacle_type == "open_space":
            # Open space with some distant objects
            for angle in range(0, 360):
                if angle % 30 == 0:  # Every 30 degrees
                    ranges[angle] = 5.0  # Distant objects
                else:
                    ranges[angle] = float('inf')  # No obstacles
        
        return ranges
    
    def test_obstacle_scenarios(self):
        """Test different obstacle scenarios"""
        print("=== TESTING OBSTACLE SCENARIOS ===")
        
        scenarios = [
            ("wall", "Dinding di depan"),
            ("corner", "Sudut (dinding depan dan kiri)"),
            ("narrow_passage", "Jalur sempit"),
            ("open_space", "Ruang terbuka")
        ]
        
        for scenario_name, description in scenarios:
            print(f"\n[TEST] Scenario: {description}")
            
            # Create test data
            ranges = self.create_test_obstacle_data(scenario_name)
            
            # Send data
            if self.send_test_lidar_data(ranges):
                print(f"[TEST] ✓ Sent {scenario_name} scenario data")
                
                # Analyze data
                valid_ranges = [r for r in ranges if r != float('inf')]
                if valid_ranges:
                    min_dist = min(valid_ranges)
                    max_dist = max(valid_ranges)
                    avg_dist = sum(valid_ranges) / len(valid_ranges)
                    
                    close_objects = [r for r in valid_ranges if r < 1.0]
                    medium_objects = [r for r in valid_ranges if 1.0 <= r < 3.0]
                    far_objects = [r for r in valid_ranges if r >= 3.0]
                    
                    print(f"  Data: {len(valid_ranges)}/360 points")
                    print(f"  Min: {min_dist:.2f}m, Max: {max_dist:.2f}m, Avg: {avg_dist:.2f}m")
                    print(f"  Objects: Close({len(close_objects)}), Medium({len(medium_objects)}), Far({len(far_objects)})")
                
                # Wait before next scenario
                time.sleep(2)
            else:
                print(f"[TEST] ✗ Failed to send {scenario_name} scenario data")
    
    def test_continuous_data_stream(self):
        """Test continuous data streaming"""
        print("\n=== TESTING CONTINUOUS DATA STREAM ===")
        print("Sending continuous LiDAR data for 10 seconds...")
        
        start_time = time.time()
        data_count = 0
        
        try:
            while time.time() - start_time < 10.0:
                # Create rotating obstacle data
                ranges = [float('inf')] * 360
                
                # Create a moving obstacle (rotates around robot)
                current_time = time.time() - start_time
                obstacle_angle = int((current_time * 30) % 360)  # Rotate 30 degrees per second
                
                # Add obstacle at current angle
                for angle_offset in range(-5, 6):
                    angle = (obstacle_angle + angle_offset + 360) % 360
                    ranges[angle] = 1.5  # Obstacle at 1.5 meters
                
                # Send data
                if self.send_test_lidar_data(ranges):
                    data_count += 1
                    
                    if data_count % 10 == 0:
                        print(f"[STREAM] Sent {data_count} data packets")
                
                time.sleep(0.1)  # 10 Hz
                
        except KeyboardInterrupt:
            print("\n[STREAM] Interrupted by user")
        
        print(f"[STREAM] Completed: {data_count} data packets sent")
    
    def run_all_tests(self):
        """Run all LiDAR real-world tests"""
        print("LIDAR REAL-WORLD SYSTEM TEST")
        print("=" * 60)
        
        if self.mqtt_client is None:
            print("[ERROR] MQTT connection failed. Cannot run tests.")
            return
        
        try:
            # Test 1: Obstacle scenarios
            self.test_obstacle_scenarios()
            
            # Test 2: Continuous streaming
            self.test_continuous_data_stream()
            
            print("\n" + "=" * 60)
            print("SUMMARY:")
            print("✓ MQTT connection established")
            print("✓ Test obstacle scenarios sent")
            print("✓ Continuous data streaming tested")
            print("\nTo use with roam.py:")
            print("1. Run this script to send test data")
            print("2. Run roam.py in another terminal")
            print("3. Robot should detect obstacles from real-world LiDAR data")
            print("4. For real LiDAR data, use test_lidar.py with streaming mode")
            
        except KeyboardInterrupt:
            print("\n[TEST] Tests interrupted by user")
        finally:
            if self.mqtt_client:
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
            print("[TEST] Cleanup completed")

if __name__ == "__main__":
    test = LiDARRealWorldTest()
    test.run_all_tests() 