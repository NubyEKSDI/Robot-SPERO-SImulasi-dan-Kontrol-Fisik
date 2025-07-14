#!/usr/bin/env python3
"""
Full test runner untuk BCR Bot dengan integrasi LiDAR dan mapping
Script ini akan menjalankan test_robot.py dan mengirim test data
"""

import time
import json
import paho.mqtt.client as mqtt
import math
import random
import threading

# MQTT Configuration
MQTT_BROKER = "codex.petra.ac.id"
MQTT_PORT = 1883
TOPIC_LIDAR_REAL_WORLD_DATA = "robot/lidar/real_world_data"
TOPIC_MAPPING_DATA = "robot/mapping/data"

class FullTestRunner:
    """Class untuk menjalankan full test dengan data streaming"""
    
    def __init__(self):
        self.mqtt_client = None
        self.is_running = False
        self.test_phase = 0
        
    def init_mqtt(self):
        """Initialize MQTT connection"""
        try:
            self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.on_message = self.on_mqtt_message
            
            print(f"[MQTT] Connecting to {MQTT_BROKER}...")
            self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.mqtt_client.loop_start()
            time.sleep(2)
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
                print(f"[MAPPING] {data.get('status', 'Unknown')}")
        except Exception as e:
            print(f"[ERROR] Failed to process message: {str(e)}")
    
    def send_lidar_data(self, ranges, source="test_runner"):
        """Send LiDAR data"""
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
    
    def generate_test_scenario(self, scenario_type, time_offset=0):
        """Generate test scenarios"""
        ranges = [float('inf')] * 360
        
        if scenario_type == "empty":
            # Empty space
            pass
            
        elif scenario_type == "wall_front":
            # Wall in front
            for i in range(-15, 16):
                angle = i % 360
                ranges[angle] = 2.5 + math.sin(time_offset * 0.1) * 0.5
            
        elif scenario_type == "corridor":
            # Corridor with walls on sides
            for i in range(30, 61):  # Left wall
                ranges[i] = 1.2
            for i in range(300, 331):  # Right wall
                ranges[i] = 1.2
            
        elif scenario_type == "moving_obstacle":
            # Moving obstacle
            obstacle_angle = int(time_offset * 5) % 360  # Moves 5°/sec
            for i in range(obstacle_angle - 10, obstacle_angle + 11):
                angle = i % 360
                ranges[angle] = 1.5 + math.sin(time_offset * 0.2) * 0.3
            
        elif scenario_type == "emergency_stop":
            # Very close obstacle
            ranges[0] = 0.2  # 20cm - should trigger emergency stop
            
        elif scenario_type == "complex_environment":
            # Complex environment with multiple obstacles
            # Front wall
            for i in range(-20, 21):
                angle = i % 360
                ranges[angle] = 3.0
            # Left wall
            for i in range(60, 91):
                ranges[i] = 2.0
            # Right wall
            for i in range(270, 301):
                ranges[i] = 2.5
            # Random obstacles
            for i in range(0, 360, 45):
                if random.random() < 0.3:
                    ranges[i] = random.uniform(1.5, 4.0)
        
        return ranges
    
    def run_test_phase(self, phase_name, duration, scenario_type, mapping_active=True):
        """Run a test phase"""
        print(f"\n=== PHASE: {phase_name} ===")
        print(f"Duration: {duration} seconds")
        print(f"Scenario: {scenario_type}")
        print(f"Mapping: {'Active' if mapping_active else 'Inactive'}")
        print("=" * 50)
        
        start_time = time.time()
        
        if mapping_active:
            self.send_mapping_command("start_mapping")
            time.sleep(1)
        
        while time.time() - start_time < duration and self.is_running:
            time_offset = time.time() - start_time
            ranges = self.generate_test_scenario(scenario_type, time_offset)
            
            self.send_lidar_data(ranges, f"test_{scenario_type}")
            
            # Print progress every 5 seconds
            elapsed = time.time() - start_time
            if int(elapsed) % 5 == 0 and elapsed > 0:
                print(f"  Progress: {elapsed:.1f}/{duration}s")
            
            time.sleep(0.1)  # 10Hz update rate
        
        if mapping_active:
            self.send_mapping_command("stop_mapping")
            time.sleep(1)
        
        print(f"✓ Phase completed: {phase_name}")
    
    def run_full_test_sequence(self):
        """Run the complete test sequence"""
        print("=== BCR BOT FULL INTEGRATION TEST ===")
        print("This test will demonstrate all integrated features:")
        print("1. LiDAR real-world data processing")
        print("2. Automatic mapping data collection")
        print("3. Obstacle detection and avoidance")
        print("4. Emergency stop functionality")
        print("5. Complex environment handling")
        print("=" * 60)
        
        print("\nINSTRUCTIONS:")
        print("1. Start test_robot.py in another terminal")
        print("2. This script will send test data")
        print("3. Watch test_robot.py output for results")
        print("4. Press Ctrl+C to stop")
        print("=" * 60)
        
        input("\nPress Enter to start the test sequence...")
        
        try:
            self.is_running = True
            
            # Phase 1: Empty space
            self.run_test_phase("Empty Space", 10, "empty", mapping_active=True)
            
            # Phase 2: Wall in front
            self.run_test_phase("Wall Detection", 15, "wall_front", mapping_active=True)
            
            # Phase 3: Corridor navigation
            self.run_test_phase("Corridor Navigation", 20, "corridor", mapping_active=True)
            
            # Phase 4: Moving obstacle
            self.run_test_phase("Moving Obstacle", 15, "moving_obstacle", mapping_active=True)
            
            # Phase 5: Emergency stop test
            self.run_test_phase("Emergency Stop Test", 5, "emergency_stop", mapping_active=False)
            
            # Phase 6: Complex environment
            self.run_test_phase("Complex Environment", 25, "complex_environment", mapping_active=True)
            
            # Final phase: Save mapping data
            print("\n=== FINAL PHASE: SAVE MAPPING DATA ===")
            self.send_mapping_command("save_map")
            time.sleep(2)
            
            print("\n" + "=" * 60)
            print("🎉 FULL TEST SEQUENCE COMPLETED!")
            print("=" * 60)
            print("Check test_robot.py output for detailed results")
            print("Mapping data should be saved to a JSON file")
            print("=" * 60)
            
        except KeyboardInterrupt:
            print("\n\nTest interrupted by user")
        finally:
            self.is_running = False
    
    def cleanup(self):
        """Cleanup resources"""
        self.is_running = False
        if self.mqtt_client:
            try:
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
            except:
                pass
        print("[CLEANUP] Resources cleaned up")

def main():
    """Main function"""
    print("BCR Bot Full Integration Test Runner")
    print("=" * 50)
    
    runner = FullTestRunner()
    
    try:
        runner.init_mqtt()
        
        if runner.mqtt_client is None:
            print("❌ MQTT connection failed. Cannot run test.")
            return
        
        runner.run_full_test_sequence()
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\nError: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        runner.cleanup()

if __name__ == "__main__":
    main() 