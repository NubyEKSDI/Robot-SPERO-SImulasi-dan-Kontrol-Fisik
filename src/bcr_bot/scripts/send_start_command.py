#!/usr/bin/env python3
"""
Simple script to send START command to unpause the robot
"""
import paho.mqtt.client as mqtt
import json
import time

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT broker")
    else:
        print(f"Connection failed with code {rc}")

def main():
    client = mqtt.Client()
    client.on_connect = on_connect
    
    try:
        client.connect("codex.petra.ac.id", 1883, 60)
        client.loop_start()
        
        print("Sending START command to robot...")
        
        # Send start command
        start_data = {
            'command': 'start'
        }
        client.publish("test/topic", json.dumps(start_data))
        
        print("START command sent successfully!")
        print("Robot should now be active and ready to receive movement commands.")
        
        time.sleep(2)  # Wait a bit to ensure message is sent
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        client.loop_stop()
        client.disconnect()

if __name__ == "__main__":
    main() 