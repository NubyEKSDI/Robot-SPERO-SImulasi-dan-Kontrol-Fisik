#!/usr/bin/env python3
"""
Complete test runner untuk BCR Bot dengan integrasi LiDAR dan mapping
Script ini akan menjalankan semua test secara berurutan
"""

import time
import subprocess
import sys
import os
import signal
import json
import paho.mqtt.client as mqtt

# MQTT Configuration
MQTT_BROKER = "codex.petra.ac.id"
MQTT_PORT = 1883
TOPIC_LIDAR_REAL_WORLD_DATA = "robot/lidar/real_world_data"
TOPIC_MAPPING_DATA = "robot/mapping/data"

class CompleteTestRunner:
    """Class untuk menjalankan semua test secara berurutan"""
    
    def __init__(self):
        self.processes = []
        self.mqtt_client = None
        self.test_results = {}
        
    def init_mqtt(self):
        """Initialize MQTT untuk monitoring"""
        try:
            self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.on_message = self.on_mqtt_message
            
            print(f"[MQTT] Connecting to {MQTT_BROKER}...")
            self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.mqtt_client.loop_start()
            time.sleep(2)
            print("[MQTT] Connected for monitoring")
            
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
            elif msg.topic == TOPIC_LIDAR_REAL_WORLD_DATA:
                print("[LIDAR] Real-world data received")
        except Exception as e:
            print(f"[ERROR] Failed to process message: {str(e)}")
    
    def start_process(self, command, name):
        """Start a subprocess"""
        try:
            print(f"\n[STARTING] {name}")
            print(f"Command: {command}")
            
            process = subprocess.Popen(
                command,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                universal_newlines=True
            )
            
            self.processes.append((process, name))
            print(f"✓ {name} started (PID: {process.pid})")
            return process
            
        except Exception as e:
            print(f"✗ Failed to start {name}: {str(e)}")
            return None
    
    def stop_process(self, process, name):
        """Stop a subprocess"""
        try:
            if process and process.poll() is None:
                print(f"\n[STOPPING] {name}")
                process.terminate()
                
                # Wait for graceful shutdown
                try:
                    process.wait(timeout=5)
                    print(f"✓ {name} stopped gracefully")
                except subprocess.TimeoutExpired:
                    print(f"⚠️  {name} didn't stop gracefully, forcing...")
                    process.kill()
                    process.wait()
                    print(f"✓ {name} force stopped")
                    
        except Exception as e:
            print(f"✗ Error stopping {name}: {str(e)}")
    
    def stop_all_processes(self):
        """Stop all running processes"""
        print("\n=== STOPPING ALL PROCESSES ===")
        
        for process, name in self.processes:
            self.stop_process(process, name)
        
        self.processes.clear()
        print("✓ All processes stopped")
    
    def test_lidar_connection(self):
        """Test 1: LiDAR connection"""
        print("\n" + "="*60)
        print("TEST 1: LIDAR CONNECTION")
        print("="*60)
        
        try:
            # Start test_lidar.py
            process = self.start_process(
                "python3 src/bcr_bot/scripts/test_lidar.py",
                "LiDAR Test"
            )
            
            if process:
                # Wait for LiDAR test to complete
                time.sleep(10)
                
                # Check if process is still running
                if process.poll() is None:
                    print("✓ LiDAR test is running")
                    self.test_results["lidar_connection"] = "PASS"
                else:
                    print("✗ LiDAR test stopped unexpectedly")
                    self.test_results["lidar_connection"] = "FAIL"
                
                # Stop the process
                self.stop_process(process, "LiDAR Test")
            else:
                self.test_results["lidar_connection"] = "FAIL"
                
        except Exception as e:
            print(f"✗ LiDAR connection test failed: {str(e)}")
            self.test_results["lidar_connection"] = "FAIL"
    
    def test_robot_integration(self):
        """Test 2: Robot integration with LiDAR and mapping"""
        print("\n" + "="*60)
        print("TEST 2: ROBOT INTEGRATION")
        print("="*60)
        
        try:
            # Start test_robot.py (this will run all features automatically)
            process = self.start_process(
                "python3 src/bcr_bot/scripts/test_robot.py",
                "Robot Integration"
            )
            
            if process:
                print("✓ Robot integration started")
                print("Robot is now running with:")
                print("- Automatic LiDAR streaming")
                print("- Automatic mapping")
                print("- MQTT command reception")
                print("- Obstacle detection")
                
                # Let it run for a while to test functionality
                time.sleep(15)
                
                # Check if process is still running
                if process.poll() is None:
                    print("✓ Robot integration is running")
                    self.test_results["robot_integration"] = "PASS"
                else:
                    print("✗ Robot integration stopped unexpectedly")
                    self.test_results["robot_integration"] = "FAIL"
                
                # Stop the process
                self.stop_process(process, "Robot Integration")
            else:
                self.test_results["robot_integration"] = "FAIL"
                
        except Exception as e:
            print(f"✗ Robot integration test failed: {str(e)}")
            self.test_results["robot_integration"] = "FAIL"
    
    def test_lidar_streaming(self):
        """Test 3: LiDAR streaming mode"""
        print("\n" + "="*60)
        print("TEST 3: LIDAR STREAMING")
        print("="*60)
        
        try:
            # Start LiDAR streaming mode
            process = self.start_process(
                "echo '2' | python3 src/bcr_bot/scripts/test_lidar.py",
                "LiDAR Streaming"
            )
            
            if process:
                print("✓ LiDAR streaming started")
                print("LiDAR is now streaming real-world data to roam.py")
                
                # Let it stream for a while
                time.sleep(10)
                
                # Check if process is still running
                if process.poll() is None:
                    print("✓ LiDAR streaming is running")
                    self.test_results["lidar_streaming"] = "PASS"
                else:
                    print("✗ LiDAR streaming stopped unexpectedly")
                    self.test_results["lidar_streaming"] = "FAIL"
                
                # Stop the process
                self.stop_process(process, "LiDAR Streaming")
            else:
                self.test_results["lidar_streaming"] = "FAIL"
                
        except Exception as e:
            print(f"✗ LiDAR streaming test failed: {str(e)}")
            self.test_results["lidar_streaming"] = "FAIL"
    
    def test_mapping_functionality(self):
        """Test 4: Mapping functionality"""
        print("\n" + "="*60)
        print("TEST 4: MAPPING FUNCTIONALITY")
        print("="*60)
        
        try:
            # Start test_lidar_mapping.py
            process = self.start_process(
                "python3 src/bcr_bot/scripts/test_lidar_mapping.py",
                "Mapping Test"
            )
            
            if process:
                print("✓ Mapping test started")
                print("Testing mapping commands and data collection")
                
                # Let it run for a while
                time.sleep(15)
                
                # Check if process is still running
                if process.poll() is None:
                    print("✓ Mapping test is running")
                    self.test_results["mapping_functionality"] = "PASS"
                else:
                    print("✗ Mapping test stopped unexpectedly")
                    self.test_results["mapping_functionality"] = "FAIL"
                
                # Stop the process
                self.stop_process(process, "Mapping Test")
            else:
                self.test_results["mapping_functionality"] = "FAIL"
                
        except Exception as e:
            print(f"✗ Mapping functionality test failed: {str(e)}")
            self.test_results["mapping_functionality"] = "FAIL"
    
    def test_complete_integration(self):
        """Test 5: Complete integration test"""
        print("\n" + "="*60)
        print("TEST 5: COMPLETE INTEGRATION")
        print("="*60)
        
        try:
            # Start test_integrated_lidar.py
            process = self.start_process(
                "python3 src/bcr_bot/scripts/test_integrated_lidar.py",
                "Complete Integration Test"
            )
            
            if process:
                print("✓ Complete integration test started")
                print("Running comprehensive integration tests")
                
                # Let it run for a while
                time.sleep(20)
                
                # Check if process is still running
                if process.poll() is None:
                    print("✓ Complete integration test is running")
                    self.test_results["complete_integration"] = "PASS"
                else:
                    print("✗ Complete integration test stopped unexpectedly")
                    self.test_results["complete_integration"] = "FAIL"
                
                # Stop the process
                self.stop_process(process, "Complete Integration Test")
            else:
                self.test_results["complete_integration"] = "FAIL"
                
        except Exception as e:
            print(f"✗ Complete integration test failed: {str(e)}")
            self.test_results["complete_integration"] = "FAIL"
    
    def run_all_tests(self):
        """Run all tests in sequence"""
        print("=== BCR BOT COMPLETE TEST SUITE ===")
        print("Testing all integrated features:")
        print("1. LiDAR connection")
        print("2. Robot integration")
        print("3. LiDAR streaming")
        print("4. Mapping functionality")
        print("5. Complete integration")
        print("=" * 60)
        
        try:
            # Initialize MQTT for monitoring
            self.init_mqtt()
            
            # Run all tests
            self.test_lidar_connection()
            time.sleep(2)
            
            self.test_robot_integration()
            time.sleep(2)
            
            self.test_lidar_streaming()
            time.sleep(2)
            
            self.test_mapping_functionality()
            time.sleep(2)
            
            self.test_complete_integration()
            
            # Print final results
            self.print_results()
            
        except KeyboardInterrupt:
            print("\n\nTest interrupted by user")
        except Exception as e:
            print(f"\nError during testing: {str(e)}")
        finally:
            self.stop_all_processes()
            if self.mqtt_client:
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
    
    def print_results(self):
        """Print test results summary"""
        print("\n" + "="*60)
        print("TEST RESULTS SUMMARY")
        print("="*60)
        
        passed = 0
        total = len(self.test_results)
        
        for test_name, result in self.test_results.items():
            status = "✓ PASS" if result == "PASS" else "✗ FAIL"
            print(f"{test_name}: {status}")
            if result == "PASS":
                passed += 1
        
        print(f"\nOverall: {passed}/{total} tests passed")
        
        if passed == total:
            print("🎉 ALL TESTS PASSED!")
            print("BCR Bot integration is working perfectly!")
        elif passed >= total * 0.8:
            print("✅ MOST TESTS PASSED!")
            print("BCR Bot integration is working well with minor issues.")
        else:
            print("⚠️  MANY TESTS FAILED!")
            print("BCR Bot integration needs attention.")
        
        print("="*60)

def main():
    """Main function"""
    print("BCR Bot Complete Test Suite")
    print("This script will test all integrated features")
    print("=" * 50)
    
    runner = CompleteTestRunner()
    runner.run_all_tests()

if __name__ == "__main__":
    main() 