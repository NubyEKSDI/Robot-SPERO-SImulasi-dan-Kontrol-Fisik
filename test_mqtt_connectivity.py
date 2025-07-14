#!/usr/bin/env python3
"""
Simple MQTT connectivity test
Test apakah robot bisa mengirim pesan yang diterima oleh bridge
"""

import paho.mqtt.client as mqtt
import json
import time
import sys

# MQTT Configuration
MQTT_BROKER = "spin5.petra.ac.id"
MQTT_PORT = 1883
MQTT_TOPIC_LIDAR_DATA = "robot/lidar/real_world_data"
MQTT_TOPIC_STATUS = "robot/lidar/status"

def test_mqtt_send():
    """Test sending data to MQTT broker"""
    print("=== MQTT Connectivity Test (Sender) ===")
    print(f"Broker: {MQTT_BROKER}:{MQTT_PORT}")
    print(f"Topic: {MQTT_TOPIC_LIDAR_DATA}")
    print("=" * 50)
    
    try:
        # Create MQTT client
        client = mqtt.Client()
        
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                print("✓ Connected to MQTT broker")
            else:
                print(f"✗ Failed to connect: {rc}")
        
        def on_publish(client, userdata, mid):
            print(f"✓ Message {mid} published successfully")
        
        def on_disconnect(client, userdata, rc):
            print(f"Disconnected: {rc}")
        
        client.on_connect = on_connect
        client.on_publish = on_publish
        client.on_disconnect = on_disconnect
        
        # Connect
        print("Connecting to MQTT broker...")
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_start()
        
        # Wait for connection
        time.sleep(2)
        
        # Send test messages
        for i in range(5):
            # Create test lidar data
            test_data = {
                "timestamp": time.time(),
                "ranges": [1.0] * 360,  # Simple test data
                "angle_min": 0.0,
                "angle_max": 6.28,
                "angle_increment": 0.0175,
                "range_min": 0.15,
                "range_max": 12.0,
                "source": "connectivity_test"
            }
            
            # Send data
            json_data = json.dumps(test_data)
            result = client.publish(MQTT_TOPIC_LIDAR_DATA, json_data)
            
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                print(f"✓ Sent test message {i+1}/5 (ID: {result.mid})")
            else:
                print(f"✗ Failed to send message {i+1}: {result.rc}")
            
            time.sleep(1)
        
        # Send status message
        status_data = {
            "status": "test_completed",
            "timestamp": time.time(),
            "message": "MQTT connectivity test completed"
        }
        client.publish(MQTT_TOPIC_STATUS, json.dumps(status_data))
        print("✓ Sent status message")
        
        # Wait for messages to be sent
        time.sleep(2)
        
        # Disconnect
        client.loop_stop()
        client.disconnect()
        
        print("\n=== Test completed ===")
        print("Check the bridge logs to see if messages were received")
        
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()

def test_mqtt_receive():
    """Test receiving data from MQTT broker"""
    print("=== MQTT Connectivity Test (Receiver) ===")
    print(f"Broker: {MQTT_BROKER}:{MQTT_PORT}")
    print(f"Subscribing to: {MQTT_TOPIC_LIDAR_DATA}")
    print("=" * 50)
    
    message_count = 0
    
    try:
        # Create MQTT client
        client = mqtt.Client()
        
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                print("✓ Connected to MQTT broker")
                client.subscribe(MQTT_TOPIC_LIDAR_DATA)
                client.subscribe(MQTT_TOPIC_STATUS)
                print(f"✓ Subscribed to topics")
            else:
                print(f"✗ Failed to connect: {rc}")
        
        def on_message(client, userdata, msg):
            nonlocal message_count
            message_count += 1
            
            try:
                data = json.loads(msg.payload.decode())
                source = data.get('source', 'unknown')
                timestamp = data.get('timestamp', 0)
                
                print(f"✓ Received message {message_count}")
                print(f"  Topic: {msg.topic}")
                print(f"  Source: {source}")
                print(f"  Size: {len(msg.payload)} bytes")
                print(f"  Timestamp: {timestamp}")
                
                if msg.topic == MQTT_TOPIC_LIDAR_DATA:
                    ranges = data.get('ranges', [])
                    print(f"  Ranges: {len(ranges)} points")
                
            except Exception as e:
                print(f"✗ Error parsing message: {e}")
                print(f"  Raw payload: {msg.payload[:100]}...")
        
        def on_disconnect(client, userdata, rc):
            print(f"Disconnected: {rc}")
        
        client.on_connect = on_connect
        client.on_message = on_message
        client.on_disconnect = on_disconnect
        
        # Connect
        print("Connecting to MQTT broker...")
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        
        # Listen for messages
        print("Listening for messages... (Press Ctrl+C to stop)")
        client.loop_forever()
        
    except KeyboardInterrupt:
        print(f"\n✓ Stopped listening. Received {message_count} messages total.")
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()

def main():
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python test_mqtt_connectivity.py send    # Test sending messages")
        print("  python test_mqtt_connectivity.py receive # Test receiving messages")
        return
    
    mode = sys.argv[1].lower()
    
    if mode == "send":
        test_mqtt_send()
    elif mode == "receive":
        test_mqtt_receive()
    else:
        print("Invalid mode. Use 'send' or 'receive'")

if __name__ == "__main__":
    main() 