#!/usr/bin/env python3
"""
Test script for MQTT waypoint commands
This script sends waypoint navigation commands to roam.py via MQTT
"""

import paho.mqtt.client as mqtt
import json
import time
import sys

# MQTT Configuration
MQTT_BROKER = "codex.petra.ac.id"
MQTT_PORT = 1883
MQTT_TOPIC = "test/topic"

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"[MQTT] Connected to broker successfully")
    else:
        print(f"[MQTT] Connection failed with code {rc}")

def on_publish(client, userdata, mid):
    print(f"[MQTT] Message published with ID: {mid}")

def send_waypoint_command(client, waypoint):
    """Send waypoint navigation command"""
    command = {
        "command": f"waypoint_{waypoint}"
    }
    
    message = json.dumps(command)
    result = client.publish(MQTT_TOPIC, message)
    
    if result.rc == mqtt.MQTT_ERR_SUCCESS:
        print(f"[SENT] {message}")
    else:
        print(f"[ERROR] Failed to send message: {result.rc}")

def send_control_command(client, command):
    """Send control command (stop/start)"""
    cmd_data = {
        "command": command
    }
    
    message = json.dumps(cmd_data)
    result = client.publish(MQTT_TOPIC, message)
    
    if result.rc == mqtt.MQTT_ERR_SUCCESS:
        print(f"[SENT] {message}")
    else:
        print(f"[ERROR] Failed to send message: {result.rc}")

def main():
    print("MQTT Waypoint Command Test")
    print("=" * 40)
    
    # Create MQTT client
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_publish = on_publish
    
    try:
        # Connect to MQTT broker
        print(f"[MQTT] Connecting to {MQTT_BROKER}:{MQTT_PORT}...")
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_start()
        
        # Wait for connection
        time.sleep(2)
        
        print("\nAvailable commands:")
        print("1. waypoint_i - Go to Waypoint 0 (Start point)")
        print("2. waypoint_q - Go to Waypoint 5 (MID LEFT)")
        print("3. waypoint_p - Go to Waypoint 2 (Bottom-right point)")
        print("4. stop - Stop robot")
        print("5. start - Start robot")
        print("6. quit - Exit program")
        
        while True:
            print("\nEnter command (1-6): ", end="")
            choice = input().strip()
            
            if choice == "1":
                send_waypoint_command(client, "i")
            elif choice == "2":
                send_waypoint_command(client, "q")
            elif choice == "3":
                send_waypoint_command(client, "p")
            elif choice == "4":
                send_control_command(client, "stop")
            elif choice == "5":
                send_control_command(client, "start")
            elif choice == "6" or choice.lower() == "quit":
                print("Exiting...")
                break
            else:
                print("Invalid choice. Please enter 1-6.")
            
            time.sleep(1)  # Brief delay between commands
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"[ERROR] {str(e)}")
    finally:
        # Cleanup
        client.loop_stop()
        client.disconnect()
        print("[MQTT] Disconnected")

if __name__ == "__main__":
    main() 