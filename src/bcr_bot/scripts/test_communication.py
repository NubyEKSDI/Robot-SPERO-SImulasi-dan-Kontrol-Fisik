#!/usr/bin/env python3
"""
Simple test script to verify MQTT communication between roam.py and test_robot.py
"""
import paho.mqtt.client as mqtt
import json
import time

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT broker")
        client.subscribe("robot/physical/cmd_vel")
    else:
        print(f"Connection failed with code {rc}")

def on_message(client, userdata, msg):
    try:
        if msg.topic == "robot/physical/cmd_vel":
            data = json.loads(msg.payload.decode())
            print(f"Received command: {data}")
            
            # Check if it's a movement command
            if 'linear' in data and 'angular' in data:
                linear_x = data['linear'].get('x', 0)
                angular_z = data['angular'].get('z', 0)
                mode = data.get('mode', 'Unknown')
                
                print(f"Movement: linear_x={linear_x}, angular_z={angular_z}, mode={mode}")
                
                # Determine what the robot should do
                if abs(linear_x) > abs(angular_z):
                    if linear_x > 0:
                        print("  -> Robot should move FORWARD")
                    else:
                        print("  -> Robot should move BACKWARD")
                else:
                    if angular_z > 0:
                        print("  -> Robot should turn LEFT")
                    else:
                        print("  -> Robot should turn RIGHT")
                        
                if abs(linear_x) < 0.01 and abs(angular_z) < 0.01:
                    print("  -> Robot should STOP")
                    
    except Exception as e:
        print(f"Error processing message: {e}")

def main():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    
    try:
        client.connect("codex.petra.ac.id", 1883, 60)
        client.loop_start()
        
        print("MQTT Communication Test")
        print("=======================")
        print("This script will monitor commands sent from roam.py to test_robot.py")
        print("Start roam.py in another terminal and use WASD keys to test manual control")
        print("Or let roam.py run in autonomous mode to see automatic commands")
        print("Press Ctrl+C to exit")
        print()
        
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nTest stopped by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        client.loop_stop()
        client.disconnect()

if __name__ == "__main__":
    main() 