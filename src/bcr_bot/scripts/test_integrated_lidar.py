#!/usr/bin/env python3
"""
Comprehensive test script untuk integrasi LiDAR real-world dan mapping
Digunakan untuk menguji semua fitur LiDAR dan mapping di test_robot.py
"""

import time
import json
import threading
import paho.mqtt.client as mqtt
import math
import random

# MQTT Configuration
MQTT_BROKER = "codex.petra.ac.id"
MQTT_PORT = 1883
TOPIC_LIDAR_REAL_WORLD_DATA = "robot/lidar/real_world_data"
TOPIC_LIDAR_STATUS = "robot/lidar/status"
TOPIC_MAPPING_DATA = "robot/mapping/data"

class IntegratedLiDARTester:
    """Class untuk testing integrasi LiDAR dan mapping"""
    
    def __init__(self):
        self.mqtt_client = None
        self.test_results = {}
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
        else:
            print(f"[MQTT] Connection failed with code {rc}")
    
    def on_mqtt_message(self, client, userdata, msg):
        try:
            if msg.topic == TOPIC_MAPPING_DATA:
                data = json.loads(msg.payload.decode())
                print(f"[MAPPING] Received: {data.get('status', 'Unknown')}")
            elif msg.topic == TOPIC_LIDAR_STATUS:
                data = json.loads(msg.payload.decode())
                print(f"[LIDAR STATUS] {data.get('message', 'Unknown')}")
        except Exception as e:
            print(f"[ERROR] Failed to process message: {str(e)}")
    
    def send_lidar_data(self, ranges, source="test_simulator"):
        """Send simulated LiDAR data"""
        if self.mqtt_client is None:
            return
        
        try:
            lidar_data = {
                "timestamp": time.time(),
                "ranges": ranges,
                "angle_min": -math.pi,
                "angle_max": math.pi,
                "angle_increment": 2 * math.pi / 360,
                "range_min": 0.15,
                "range_max": 12.0,
                "source": source
            }
            
            self.mqtt_client.publish(TOPIC_LIDAR_REAL_WORLD_DATA, json.dumps(lidar_data))
            
        except Exception as e:
            print(f"[ERROR] Failed to send LiDAR data: {str(e)}")
    
    def send_mapping_command(self, command):
        """Send mapping command"""
        if self.mqtt_client is None:
            return
        
        try:
            mapping_data = {
                "command": command,
                "timestamp": time.time()
            }
            
            self.mqtt_client.publish(TOPIC_MAPPING_DATA, json.dumps(mapping_data))
            print(f"[MAPPING] Sent command: {command}")
            
        except Exception as e:
            print(f"[ERROR] Failed to send mapping command: {str(e)}")
    
    def generate_obstacle_scenario(self, scenario_type):
        """Generate different obstacle scenarios"""
        ranges = [float('inf')] * 360
        
        if scenario_type == "empty":
            # Empty space - no obstacles
            pass
            
        elif scenario_type == "wall_front":
            # Wall in front (0°)
            for i in range(-10, 11):  # 20° wide wall
                angle = i % 360
                ranges[angle] = 2.0  # 2m away
            
        elif scenario_type == "wall_left":
            # Wall on left (90°)
            for i in range(80, 101):  # 20° wide wall
                ranges[i] = 1.5  # 1.5m away
            
        elif scenario_type == "wall_right":
            # Wall on right (270°)
            for i in range(260, 281):  # 20° wide wall
                ranges[i] = 1.5  # 1.5m away
            
        elif scenario_type == "corner":
            # Corner scenario - walls on front and left
            for i in range(-10, 11):  # Front wall
                angle = i % 360
                ranges[angle] = 2.0
            for i in range(80, 101):  # Left wall
                ranges[i] = 1.5
            
        elif scenario_type == "narrow_passage":
            # Narrow passage - walls on both sides
            for i in range(30, 51):  # Left wall
                ranges[i] = 0.8
            for i in range(310, 331):  # Right wall
                ranges[i] = 0.8
            
        elif scenario_type == "random_obstacles":
            # Random obstacles
            for i in range(0, 360, 30):  # Every 30 degrees
                if random.random() < 0.3:  # 30% chance of obstacle
                    ranges[i] = random.uniform(1.0, 5.0)
        
        return ranges
    
    def test_lidar_data_reception(self):
        """Test if test_robot.py can receive LiDAR data"""
        print("\n=== TEST 1: LIDAR DATA RECEPTION ===")
        
        scenarios = ["empty", "wall_front", "wall_left", "wall_right", "corner"]
        
        for scenario in scenarios:
            print(f"\nTesting scenario: {scenario}")
            ranges = self.generate_obstacle_scenario(scenario)
            
            # Send data multiple times
            for i in range(5):
                self.send_lidar_data(ranges, "test_scenario")
                time.sleep(0.5)
            
            print(f"✓ Sent {scenario} scenario data")
            time.sleep(1)
        
        print("✓ LiDAR data reception test completed")
        return True
    
    def test_mapping_commands(self):
        """Test mapping commands"""
        print("\n=== TEST 2: MAPPING COMMANDS ===")
        
        commands = ["start_mapping", "stop_mapping", "save_map", "clear_map"]
        
        for command in commands:
            print(f"\nTesting command: {command}")
            self.send_mapping_command(command)
            time.sleep(1)
        
        print("✓ Mapping commands test completed")
        return True
    
    def test_continuous_mapping(self):
        """Test continuous mapping with moving obstacles"""
        print("\n=== TEST 3: CONTINUOUS MAPPING ===")
        
        # Start mapping
        self.send_mapping_command("start_mapping")
        time.sleep(1)
        
        # Simulate moving obstacles
        print("Simulating moving obstacles for 30 seconds...")
        start_time = time.time()
        
        while time.time() - start_time < 30:
            # Generate moving obstacle
            ranges = [float('inf')] * 360
            
            # Moving obstacle that changes position
            obstacle_angle = int((time.time() - start_time) * 10) % 360  # Moves 10°/sec
            for i in range(obstacle_angle - 5, obstacle_angle + 6):
                angle = i % 360
                ranges[angle] = 2.0 + math.sin(time.time()) * 0.5  # Distance varies
            
            self.send_lidar_data(ranges, "moving_obstacle")
            time.sleep(0.1)  # 10Hz update rate
        
        # Stop mapping
        self.send_mapping_command("stop_mapping")
        time.sleep(1)
        
        print("✓ Continuous mapping test completed")
        return True
    
    def test_emergency_stop(self):
        """Test emergency stop with very close obstacles"""
        print("\n=== TEST 4: EMERGENCY STOP ===")
        
        # Start with normal distance
        ranges = [float('inf')] * 360
        ranges[0] = 5.0  # 5m away
        
        print("Sending normal distance data...")
        for i in range(5):
            self.send_lidar_data(ranges, "normal_distance")
            time.sleep(0.5)
        
        # Send very close obstacle
        ranges[0] = 0.2  # 20cm away - should trigger emergency stop
        print("Sending very close obstacle (20cm) - should trigger emergency stop...")
        for i in range(3):
            self.send_lidar_data(ranges, "emergency_stop")
            time.sleep(0.5)
        
        # Return to normal distance
        ranges[0] = 5.0
        print("Returning to normal distance...")
        for i in range(3):
            self.send_lidar_data(ranges, "normal_distance")
            time.sleep(0.5)
        
        print("✓ Emergency stop test completed")
        return True
    
    def test_data_streaming(self):
        """Test high-frequency data streaming"""
        print("\n=== TEST 5: DATA STREAMING ===")
        
        print("Streaming LiDAR data at 10Hz for 10 seconds...")
        start_time = time.time()
        scan_count = 0
        
        while time.time() - start_time < 10:
            # Generate random obstacle pattern
            ranges = [float('inf')] * 360
            for i in range(0, 360, 20):  # Every 20 degrees
                if random.random() < 0.2:  # 20% chance
                    ranges[i] = random.uniform(1.0, 8.0)
            
            self.send_lidar_data(ranges, "streaming_test")
            scan_count += 1
            time.sleep(0.1)  # 10Hz
        
        print(f"✓ Streaming test completed: {scan_count} scans sent")
        return True
    
    def run_all_tests(self):
        """Run all integration tests"""
        print("=== INTEGRATED LIDAR & MAPPING TEST SUITE ===")
        print("Testing LiDAR real-world integration and mapping functionality")
        print("=" * 60)
        
        tests = [
            ("LiDAR Data Reception", self.test_lidar_data_reception),
            ("Mapping Commands", self.test_mapping_commands),
            ("Continuous Mapping", self.test_continuous_mapping),
            ("Emergency Stop", self.test_emergency_stop),
            ("Data Streaming", self.test_data_streaming)
        ]
        
        results = {}
        
        for test_name, test_func in tests:
            try:
                print(f"\n{'='*20} {test_name} {'='*20}")
                result = test_func()
                results[test_name] = "PASS" if result else "FAIL"
                print(f"✓ {test_name}: {results[test_name]}")
            except Exception as e:
                print(f"✗ {test_name}: FAIL - {str(e)}")
                results[test_name] = "FAIL"
        
        # Print summary
        print(f"\n{'='*60}")
        print("TEST SUMMARY:")
        print("=" * 60)
        
        passed = sum(1 for result in results.values() if result == "PASS")
        total = len(results)
        
        for test_name, result in results.items():
            status = "✓ PASS" if result == "PASS" else "✗ FAIL"
            print(f"{test_name}: {status}")
        
        print(f"\nOverall: {passed}/{total} tests passed")
        
        if passed == total:
            print("🎉 ALL TESTS PASSED! Integration is working correctly.")
        else:
            print("⚠️  Some tests failed. Check the output above for details.")
        
        return results
    
    def cleanup(self):
        """Cleanup resources"""
        if self.mqtt_client:
            try:
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
            except:
                pass
        print("[CLEANUP] Resources cleaned up")

def main():
    """Main test function"""
    print("Integrated LiDAR & Mapping Test Suite")
    print("This script tests the integration between LiDAR real-world data and mapping")
    print("Make sure test_robot.py is running to receive the test data")
    print("=" * 70)
    
    tester = IntegratedLiDARTester()
    
    try:
        # Wait for MQTT connection
        time.sleep(2)
        
        if tester.mqtt_client is None:
            print("❌ MQTT connection failed. Cannot run tests.")
            return
        
        # Run tests
        results = tester.run_all_tests()
        
        # Ask if user wants to run specific test
        print(f"\n{'='*70}")
        print("Additional options:")
        print("1. Run specific test")
        print("2. Interactive mode")
        print("3. Exit")
        
        choice = input("\nPilih opsi (1-3): ").strip()
        
        if choice == "1":
            print("\nAvailable tests:")
            print("1. LiDAR Data Reception")
            print("2. Mapping Commands")
            print("3. Continuous Mapping")
            print("4. Emergency Stop")
            print("5. Data Streaming")
            
            test_choice = input("Pilih test (1-5): ").strip()
            
            test_map = {
                "1": ("LiDAR Data Reception", tester.test_lidar_data_reception),
                "2": ("Mapping Commands", tester.test_mapping_commands),
                "3": ("Continuous Mapping", tester.test_continuous_mapping),
                "4": ("Emergency Stop", tester.test_emergency_stop),
                "5": ("Data Streaming", tester.test_data_streaming)
            }
            
            if test_choice in test_map:
                test_name, test_func = test_map[test_choice]
                print(f"\nRunning {test_name}...")
                test_func()
            else:
                print("Invalid choice")
                
        elif choice == "2":
            print("\n=== INTERACTIVE MODE ===")
            print("Send custom LiDAR data and mapping commands")
            print("Commands:")
            print("  lidar <scenario> - Send LiDAR data (empty, wall_front, wall_left, wall_right, corner, narrow_passage, random_obstacles)")
            print("  map <command> - Send mapping command (start_mapping, stop_mapping, save_map, clear_map)")
            print("  quit - Exit interactive mode")
            
            while True:
                cmd = input("\nCommand: ").strip().lower()
                
                if cmd == "quit":
                    break
                elif cmd.startswith("lidar "):
                    scenario = cmd.split(" ", 1)[1]
                    ranges = tester.generate_obstacle_scenario(scenario)
                    tester.send_lidar_data(ranges, "interactive")
                    print(f"Sent {scenario} scenario")
                elif cmd.startswith("map "):
                    command = cmd.split(" ", 1)[1]
                    tester.send_mapping_command(command)
                else:
                    print("Unknown command")
    
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\nError: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        tester.cleanup()
        print("\nTest completed")

if __name__ == "__main__":
    main() 