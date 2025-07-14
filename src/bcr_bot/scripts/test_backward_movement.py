#!/usr/bin/env python3
"""
Test script untuk memverifikasi bahwa tombol 's' (mundur) berfungsi dengan benar di roam.py
"""

import time
import json
import paho.mqtt.client as mqtt

# MQTT Configuration
MQTT_BROKER = "codex.petra.ac.id"
MQTT_PORT = 1883
MQTT_TOPIC_CMD_VEL = "robot/physical/cmd_vel"

class BackwardMovementTester:
    """Class untuk menguji pergerakan mundur"""
    
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
    
    def send_command(self, linear_x, angular_z, command_name=""):
        """Send command to robot"""
        if self.mqtt_client is None:
            return
        
        try:
            cmd_data = {
                'linear': {
                    'x': linear_x,
                    'y': 0.0,
                    'z': 0.0
                },
                'angular': {
                    'x': 0.0,
                    'y': 0.0,
                    'z': angular_z
                },
                'mode': command_name
            }
            self.mqtt_client.publish(MQTT_TOPIC_CMD_VEL, json.dumps(cmd_data))
            print(f"[TEST] Sent command: {command_name} - Linear: {linear_x}, Angular: {angular_z}")
            
        except Exception as e:
            print(f"[ERROR] Failed to send command: {str(e)}")
    
    def test_backward_movement(self):
        """Test backward movement"""
        print("\n=== TEST BACKWARD MOVEMENT ===")
        print("Testing 's' key functionality in roam.py")
        print("Expected: Robot should move backward (negative linear velocity)")
        
        print("\n1. Testing backward movement (s key)...")
        print("Press 's' in roam.py and observe robot movement")
        print("Expected: Robot should move backward")
        time.sleep(5)
        
        print("\n2. Testing forward movement (w key) for comparison...")
        print("Press 'w' in roam.py and observe robot movement")
        print("Expected: Robot should move forward")
        time.sleep(5)
        
        print("\n3. Testing left turn (a key)...")
        print("Press 'a' in roam.py and observe robot movement")
        print("Expected: Robot should turn left")
        time.sleep(5)
        
        print("\n4. Testing right turn (d key)...")
        print("Press 'd' in roam.py and observe robot movement")
        print("Expected: Robot should turn right")
        time.sleep(5)
        
        print("\n5. Testing stop (any other key)...")
        print("Press any other key in roam.py to stop")
        print("Expected: Robot should stop")
        time.sleep(3)
    
    def test_direct_commands(self):
        """Test direct MQTT commands"""
        print("\n=== TEST DIRECT MQTT COMMANDS ===")
        print("Sending direct commands via MQTT to test robot movement")
        
        print("\n1. Sending forward command...")
        self.send_command(2.0, 0.0, "Forward")
        time.sleep(3)
        
        print("\n2. Sending backward command...")
        self.send_command(-2.0, 0.0, "Backward")
        time.sleep(3)
        
        print("\n3. Sending left turn command...")
        self.send_command(0.0, 2.0, "Left Turn")
        time.sleep(3)
        
        print("\n4. Sending right turn command...")
        self.send_command(0.0, -2.0, "Right Turn")
        time.sleep(3)
        
        print("\n5. Sending stop command...")
        self.send_command(0.0, 0.0, "Stop")
        time.sleep(2)
    
    def run_tests(self):
        """Run all tests"""
        print("=== BACKWARD MOVEMENT TEST ===")
        print("This script will test backward movement functionality")
        print("Make sure roam.py is running and robot is ready")
        print("=" * 60)
        
        try:
            # Wait for MQTT connection
            time.sleep(2.0)
            
            # Run tests
            self.test_backward_movement()
            self.test_direct_commands()
            
            print("\n=== ALL TESTS COMPLETED ===")
            print("Test results:")
            print("- 's' key should move robot backward (negative linear velocity)")
            print("- 'w' key should move robot forward (positive linear velocity)")
            print("- 'a' key should turn robot left (positive angular velocity)")
            print("- 'd' key should turn robot right (negative angular velocity)")
            
        except KeyboardInterrupt:
            print("\n[TEST] Stopped by user")
        finally:
            # Send stop command
            print("\n[TEST] Sending final stop command...")
            self.send_command(0.0, 0.0, "Final Stop")
            time.sleep(1.0)
            
            # Cleanup
            if self.mqtt_client:
                try:
                    self.mqtt_client.loop_stop()
                    self.mqtt_client.disconnect()
                except:
                    pass
            
            print("[TEST] Cleanup completed")

def main():
    """Main function"""
    tester = BackwardMovementTester()
    tester.run_tests()

if __name__ == "__main__":
    main() 