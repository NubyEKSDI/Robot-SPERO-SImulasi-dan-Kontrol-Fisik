#!/usr/bin/env python3
"""
Test MQTT reception untuk memverifikasi data dari test_robot.py
"""

import time
import json
import paho.mqtt.client as mqtt

# MQTT Configuration
MQTT_BROKER = "codex.petra.ac.id"
MQTT_PORT = 1883
TOPIC_LIDAR_REAL_WORLD_DATA = "robot/lidar/real_world_data"
TOPIC_LIDAR_STATUS = "robot/lidar/status"

class MQTTReceptionTester:
    def __init__(self):
        self.mqtt_client = None
        self.received_messages = 0
        self.last_message_time = None
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
            # Subscribe to topics
            client.subscribe(TOPIC_LIDAR_REAL_WORLD_DATA)
            client.subscribe(TOPIC_LIDAR_STATUS)
            print(f"[MQTT] Subscribed to {TOPIC_LIDAR_REAL_WORLD_DATA}")
            print(f"[MQTT] Subscribed to {TOPIC_LIDAR_STATUS}")
        else:
            print(f"[MQTT] Connection failed with code {rc}")
    
    def on_mqtt_message(self, client, userdata, msg):
        """Handle incoming MQTT messages"""
        try:
            self.received_messages += 1
            self.last_message_time = time.time()
            
            if msg.topic == TOPIC_LIDAR_REAL_WORLD_DATA:
                print(f"[LIDAR DATA] Received message #{self.received_messages}")
                
                # Parse JSON data
                data = json.loads(msg.payload.decode())
                
                # Check if it's from test_robot.py
                source = data.get('source', 'unknown')
                print(f"[LIDAR DATA] Source: {source}")
                
                if 'ranges' in data:
                    ranges = data['ranges']
                    print(f"[LIDAR DATA] Ranges array length: {len(ranges)}")
                    
                    # Count valid and invalid values
                    valid_ranges = [r for r in ranges if r != float('inf')]
                    invalid_ranges = [r for r in ranges if r == float('inf')]
                    print(f"[LIDAR DATA] Valid ranges: {len(valid_ranges)}, Invalid ranges: {len(invalid_ranges)}")
                    
                    if valid_ranges:
                        min_dist = min(valid_ranges)
                        max_dist = max(valid_ranges)
                        print(f"[LIDAR DATA] Distance range: {min_dist:.3f}m to {max_dist:.3f}m")
                        
                        # Show sample ranges
                        print(f"[LIDAR DATA] Sample ranges (0°, 90°, 180°, 270°): {ranges[0]:.3f}, {ranges[90]:.3f}, {ranges[180]:.3f}, {ranges[270]:.3f}")
                    else:
                        print(f"[LIDAR DATA] WARNING: No valid distance data!")
                else:
                    print(f"[LIDAR DATA] No ranges field in data")
                
                print("-" * 50)
                
            elif msg.topic == TOPIC_LIDAR_STATUS:
                print(f"[LIDAR STATUS] Received status message #{self.received_messages}")
                data = json.loads(msg.payload.decode())
                print(f"[LIDAR STATUS] Status: {data.get('status', 'unknown')}")
                print(f"[LIDAR STATUS] Message: {data.get('message', 'no message')}")
                print("-" * 50)
                
        except Exception as e:
            print(f"[ERROR] Failed to process message: {str(e)}")
            import traceback
            traceback.print_exc()

def main():
    print("=== MQTT RECEPTION TESTER ===")
    print("Testing reception of LiDAR data from test_robot.py")
    print("=" * 50)
    
    tester = MQTTReceptionTester()
    
    print("\n=== MONITORING MQTT MESSAGES ===")
    print("Waiting for messages from test_robot.py...")
    print("Make sure test_robot.py is running on Raspberry Pi")
    print("=" * 50)
    
    try:
        while True:
            time.sleep(5)
            
            print(f"\n=== STATUS UPDATE ===")
            print(f"Total messages received: {tester.received_messages}")
            
            if tester.last_message_time:
                elapsed = time.time() - tester.last_message_time
                print(f"Last message received: {elapsed:.1f} seconds ago")
                
                if elapsed > 30:
                    print("⚠️  No messages received for 30+ seconds")
                    print("Check if test_robot.py is running and LiDAR is working")
            else:
                print("No messages received yet")
            
            print("=" * 50)
            
    except KeyboardInterrupt:
        print("\n\n=== SHUTDOWN ===")
        if tester.mqtt_client:
            tester.mqtt_client.loop_stop()
            tester.mqtt_client.disconnect()
        print("✓ Cleanup completed")

if __name__ == "__main__":
    main() 