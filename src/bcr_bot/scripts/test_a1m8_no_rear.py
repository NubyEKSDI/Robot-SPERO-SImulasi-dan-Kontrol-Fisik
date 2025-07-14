#!/usr/bin/env python3
"""
Test A1M8 LiDAR Data Sender - NO REAR DETECTION
This script simulates A1M8 LiDAR data and sends it via MQTT while excluding the rear sector
to test the no-rear detection functionality.
"""

import math
import time
import paho.mqtt.client as mqtt
import json
import sys
import os
import select
import tty
import termios
import threading

# MQTT Configuration
MQTT_BROKER = "codex.petra.ac.id"
MQTT_PORT = 1883
MQTT_TOPIC_LIDAR_DATA = "robot/lidar/real_world_data"
MQTT_TOPIC_STATUS = "robot/lidar/a1m8_status"

class A1M8TestSender:
    def __init__(self):
        self.mqtt_client = None
        self.mqtt_connected = False
        self.running = True
        
        # Define rear sector to exclude (behind the robot)
        # Rear sector: 135° to 225° (or -45° to +45° from back)
        # This excludes the back 90 degrees of the robot
        self.rear_start_angle = 135  # degrees
        self.rear_end_angle = 225    # degrees
        
        # Convert to radians for processing
        self.rear_start_rad = math.radians(self.rear_start_angle)
        self.rear_end_rad = math.radians(self.rear_end_angle)
        
        # Test scenarios
        self.test_scenarios = {
            'clear': {'front': 5.0, 'left': 3.0, 'right': 3.0, 'name': 'Clear Path'},
            'front_obstacle': {'front': 0.8, 'left': 3.0, 'right': 3.0, 'name': 'Front Obstacle'},
            'left_obstacle': {'front': 5.0, 'left': 0.5, 'right': 3.0, 'name': 'Left Obstacle'},
            'right_obstacle': {'front': 5.0, 'left': 3.0, 'right': 0.5, 'name': 'Right Obstacle'},
            'both_sides': {'front': 5.0, 'left': 0.4, 'right': 0.4, 'name': 'Both Sides Blocked'},
            'close_front': {'front': 0.3, 'left': 3.0, 'right': 3.0, 'name': 'Close Front Obstacle'},
        }
        
        self.current_scenario = 'clear'
        self.scan_count = 0
        
        # Initialize MQTT client
        try:
            self.mqtt_client = mqtt.Client()
            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
            self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.mqtt_client.loop_start()
            print("[MQTT] Successfully connected to MQTT broker")
            self.print_status("MQTT: Connected to broker")
        except Exception as e:
            print(f"[MQTT] Failed to connect to MQTT broker: {str(e)}")
            self.print_status("MQTT: Connection failed")
            self.mqtt_client = None

    def print_status(self, message):
        """Print status message with timestamp"""
        timestamp = time.strftime("%H:%M:%S")
        status_line = f'[{timestamp}] A1M8-Test: {message}'
        print(status_line)
        
        # Send status via MQTT if available
        if self.mqtt_client is not None:
            try:
                self.mqtt_client.publish(MQTT_TOPIC_STATUS, json.dumps({
                    'status': status_line,
                    'timestamp': time.time(),
                    'rear_excluded': True,
                    'rear_sector': f"{self.rear_start_angle}°-{self.rear_end_angle}°"
                }))
            except Exception as e:
                print(f"[ERROR] Error sending status: {str(e)}")

    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback when MQTT client connects"""
        if rc == 0:
            self.mqtt_connected = True
            print("[MQTT] Connected to broker successfully")
            self.print_status("MQTT: Connected to broker")
        else:
            self.mqtt_connected = False
            print(f"[MQTT] Connection failed with code {rc}")
            self.print_status("MQTT: Connection failed")

    def on_mqtt_disconnect(self, client, userdata, rc):
        """Callback when MQTT client disconnects"""
        self.mqtt_connected = False
        print(f"[MQTT] Disconnected with code {rc}")
        self.print_status("MQTT: Disconnected from broker")

    def is_in_rear_sector(self, angle_deg):
        """Check if angle is in the rear sector that should be excluded"""
        # Normalize angle to 0-360
        angle_deg = angle_deg % 360
        
        # Check if angle is in rear sector
        return self.rear_start_angle <= angle_deg <= self.rear_end_angle

    def create_test_lidar_data(self, front_distance, left_distance, right_distance):
        """Create test LiDAR data with specified distances, excluding rear sector"""
        ranges = [float('inf')] * 360
        
        # Set front distance (0 degrees)
        if front_distance != float('inf'):
            ranges[0] = front_distance
            # Also set nearby angles for better detection
            for i in range(1, 11):
                ranges[i] = front_distance
                ranges[360-i] = front_distance
        
        # Set left distance (90 degrees)
        if left_distance != float('inf'):
            left_idx = 90
            ranges[left_idx] = left_distance
            # Also set nearby angles
            for i in range(1, 11):
                ranges[(left_idx + i) % 360] = left_distance
                ranges[(left_idx - i) % 360] = left_distance
        
        # Set right distance (270 degrees)
        if right_distance != float('inf'):
            right_idx = 270
            ranges[right_idx] = right_distance
            # Also set nearby angles
            for i in range(1, 11):
                ranges[(right_idx + i) % 360] = right_distance
                ranges[(right_idx - i) % 360] = right_distance
        
        # Filter out rear sector (set to infinity)
        for i in range(360):
            if self.is_in_rear_sector(i):
                ranges[i] = float('inf')
        
        return ranges

    def send_test_lidar_data(self, scenario_name):
        """Send test LiDAR data for a specific scenario"""
        if self.mqtt_client is None:
            print("[ERROR] MQTT not connected")
            return False
        
        if scenario_name not in self.test_scenarios:
            print(f"[ERROR] Unknown scenario: {scenario_name}")
            return False
        
        try:
            scenario = self.test_scenarios[scenario_name]
            front_dist = scenario['front']
            left_dist = scenario['left']
            right_dist = scenario['right']
            scenario_desc = scenario['name']
            
            # Create filtered LiDAR data
            ranges = self.create_test_lidar_data(front_dist, left_dist, right_dist)
            
            # Create data packet
            lidar_data = {
                "timestamp": time.time(),
                "ranges": ranges,
                "angle_min": -math.pi,
                "angle_max": math.pi,
                "angle_increment": 2 * math.pi / 360,
                "range_min": 0.15,
                "range_max": 12.0,
                "source": "a1m8_test_no_rear",
                "rear_excluded": True,
                "rear_sector": f"{self.rear_start_angle}°-{self.rear_end_angle}°",
                "scenario": scenario_name,
                "scenario_desc": scenario_desc
            }
            
            # Send via MQTT
            self.mqtt_client.publish(MQTT_TOPIC_LIDAR_DATA, json.dumps(lidar_data))
            
            # Update statistics
            self.scan_count += 1
            
            # Print status
            valid_points = sum(1 for r in ranges if r != float('inf'))
            self.print_status(f"Sent scenario '{scenario_desc}': Front={front_dist:.2f}m, Left={left_dist:.2f}m, Right={right_dist:.2f}m, Valid={valid_points} points")
            
            return True
            
        except Exception as e:
            print(f"[ERROR] Failed to send test LiDAR data: {str(e)}")
            return False

    def print_instructions(self):
        """Print available commands"""
        print('\n' + '='*60)
        print(' '*15 + 'A1M8 LIDAR TEST - NO REAR DETECTION')
        print('='*60)
        print('Available scenarios:')
        for key, scenario in self.test_scenarios.items():
            print(f'  {key}: {scenario["name"]}')
        print('\nCommands:')
        print('  [scenario_name]: Switch to scenario (e.g., "front_obstacle")')
        print('  s: Send current scenario data')
        print('  c: Continuous sending (every 2 seconds)')
        print('  q: Quit')
        print(f'\nRear sector excluded: {self.rear_start_angle}° to {self.rear_end_angle}°')
        print('='*60 + '\n')

    def keyboard_listener(self):
        """Separate thread for handling keyboard input"""
        print("[KEYBOARD] Keyboard listener thread started")
        
        # Set up terminal for non-blocking input
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            
            while self.running:
                try:
                    # Check for keyboard input with timeout
                    if select.select([sys.stdin], [], [], 0.1)[0]:
                        key = sys.stdin.read(1)
                        
                        if key.lower() == 'q':
                            print("[KEYBOARD] Quit command received")
                            self.running = False
                            break
                        elif key.lower() == 's':
                            # Send current scenario
                            self.send_test_lidar_data(self.current_scenario)
                        elif key.lower() == 'c':
                            # Toggle continuous sending
                            if hasattr(self, 'continuous_sending'):
                                self.continuous_sending = not self.continuous_sending
                                status = "ON" if self.continuous_sending else "OFF"
                                self.print_status(f"Continuous sending: {status}")
                            else:
                                self.continuous_sending = True
                                self.print_status("Continuous sending: ON")
                        else:
                            # Check if it's a scenario name
                            scenario_name = key.lower()
                            if scenario_name in self.test_scenarios:
                                self.current_scenario = scenario_name
                                scenario_desc = self.test_scenarios[scenario_name]['name']
                                self.print_status(f"Switched to scenario: {scenario_name} ({scenario_desc})")
                            else:
                                print(f"[KEYBOARD] Unknown key: '{key}'")
                                
                except Exception as e:
                    print(f"[KEYBOARD] Error in keyboard listener: {str(e)}")
                    time.sleep(0.1)
                    continue
                    
                time.sleep(0.05)
                
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        
        print("[KEYBOARD] Keyboard listener thread stopped")

    def continuous_send_loop(self):
        """Background thread for continuous sending"""
        while self.running:
            if hasattr(self, 'continuous_sending') and self.continuous_sending:
                self.send_test_lidar_data(self.current_scenario)
                time.sleep(2.0)  # Send every 2 seconds
            else:
                time.sleep(0.1)

    def cleanup(self):
        """Cleanup resources before shutdown"""
        try:
            if self.mqtt_client is not None:
                try:
                    self.mqtt_client.loop_stop()
                    self.mqtt_client.disconnect()
                except Exception as e:
                    print(f"[WARNING] Could not disconnect MQTT client: {str(e)}")
            
            self.print_status("A1M8 Test Sender shutdown complete")
            
        except Exception as e:
            print(f"[ERROR] Error during cleanup: {str(e)}")

def main():
    """Main function"""
    try:
        sender = A1M8TestSender()
        
        # Start keyboard listener in a separate thread
        keyboard_thread = threading.Thread(target=sender.keyboard_listener)
        keyboard_thread.daemon = True
        keyboard_thread.start()
        
        # Start continuous send loop in a separate thread
        continuous_thread = threading.Thread(target=sender.continuous_send_loop)
        continuous_thread.daemon = True
        continuous_thread.start()
        
        print("=== A1M8 LIDAR TEST SENDER STARTED ===")
        print("Rear detection disabled (135°-225° sector excluded)")
        print("Test data sent via MQTT to robot/lidar/real_world_data")
        sender.print_instructions()
        
        # Main loop
        while sender.running:
            try:
                time.sleep(0.1)
            except KeyboardInterrupt:
                print('\nKeyboard interrupt received')
                break
            except Exception as e:
                print(f'\nError in main loop: {str(e)}')
                time.sleep(1.0)
                continue
                
    except Exception as e:
        print(f'\nError in main: {str(e)}')
    finally:
        try:
            print('\n=== SHUTDOWN SEQUENCE ===')
            # Cleanup
            if 'sender' in locals():
                print("Cleaning up...")
                sender.cleanup()
            print("✓ Cleanup completed")
        except Exception as e:
            print(f'\nError during final cleanup: {str(e)}')
        sys.exit(0)

if __name__ == '__main__':
    main() 