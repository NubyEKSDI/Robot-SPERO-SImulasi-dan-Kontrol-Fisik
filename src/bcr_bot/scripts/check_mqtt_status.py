#!/usr/bin/env python3
"""
Script untuk mengecek status koneksi MQTT
Usage: python3 check_mqtt_status.py
"""

import paho.mqtt.client as mqtt
import json
import time
import sys

# MQTT Configuration
MQTT_BROKER = "codex.petra.ac.id"
MQTT_PORT = 1883
MQTT_TIMEOUT = 60

# Topics to check
TOPICS = [
    "robot/physical/scan",
    "robot/physical/status", 
    "robot/physical/battery",
    "robot/lidar/real_world_data",
    "robot/lidar/status",
    "test/topic"
]

class MQTTStatusChecker:
    def __init__(self):
        self.connected = False
        self.messages_received = {}
        self.last_message_time = {}
        
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.connected = True
            print(f"✅ Connected to MQTT broker: {MQTT_BROKER}")
            
            # Subscribe to all topics
            for topic in TOPICS:
                client.subscribe(topic, 1)
                print(f"📡 Subscribed to: {topic}")
        else:
            print(f"❌ Failed to connect to MQTT broker. Return code: {rc}")
            
    def on_message(self, client, userdata, msg):
        topic = msg.topic
        timestamp = time.time()
        
        if topic not in self.messages_received:
            self.messages_received[topic] = 0
        self.messages_received[topic] += 1
        self.last_message_time[topic] = timestamp
        
        try:
            payload = json.loads(msg.payload.decode())
            print(f"📨 Received message on {topic}: {len(str(payload))} chars")
        except:
            print(f"📨 Received message on {topic}: {len(msg.payload)} bytes")
            
    def on_disconnect(self, client, userdata, rc):
        self.connected = False
        print(f"❌ Disconnected from MQTT broker. Return code: {rc}")
        
    def check_connection(self):
        print("=== MQTT Connection Status Check ===")
        print(f"Broker: {MQTT_BROKER}:{MQTT_PORT}")
        print(f"Timeout: {MQTT_TIMEOUT} seconds")
        print()
        
        # Create MQTT client
        client = mqtt.Client()
        client.on_connect = self.on_connect
        client.on_message = self.on_message
        client.on_disconnect = self.on_disconnect
        
        try:
            # Connect to broker
            print("🔌 Connecting to MQTT broker...")
            client.connect(MQTT_BROKER, MQTT_PORT, MQTT_TIMEOUT)
            client.loop_start()
            
            # Wait for connection
            timeout = 10
            start_time = time.time()
            while not self.connected and (time.time() - start_time) < timeout:
                time.sleep(0.1)
                
            if not self.connected:
                print("❌ Connection timeout")
                return False
                
            # Monitor messages for 30 seconds
            print("\n📡 Monitoring MQTT messages for 30 seconds...")
            print("Press Ctrl+C to stop early")
            
            monitor_time = 30
            start_time = time.time()
            
            while (time.time() - start_time) < monitor_time:
                try:
                    time.sleep(1)
                    
                    # Print status every 5 seconds
                    elapsed = int(time.time() - start_time)
                    if elapsed % 5 == 0:
                        print(f"⏱️  {elapsed}s elapsed...")
                        
                except KeyboardInterrupt:
                    print("\n⏹️  Stopped by user")
                    break
                    
        except Exception as e:
            print(f"❌ Error: {str(e)}")
            return False
        finally:
            client.loop_stop()
            client.disconnect()
            
        # Print final status
        print("\n=== Final Status ===")
        print(f"Connection: {'✅ Connected' if self.connected else '❌ Disconnected'}")
        print("\nMessages received:")
        
        if not self.messages_received:
            print("❌ No messages received")
        else:
            for topic in TOPICS:
                count = self.messages_received.get(topic, 0)
                last_time = self.last_message_time.get(topic, 0)
                
                if count > 0:
                    time_ago = time.time() - last_time if last_time > 0 else 0
                    print(f"✅ {topic}: {count} messages (last: {time_ago:.1f}s ago)")
                else:
                    print(f"❌ {topic}: No messages")
                    
        return self.connected

def main():
    checker = MQTTStatusChecker()
    success = checker.check_connection()
    
    if success:
        print("\n✅ MQTT connection test completed successfully")
        sys.exit(0)
    else:
        print("\n❌ MQTT connection test failed")
        sys.exit(1)

if __name__ == "__main__":
    main() 