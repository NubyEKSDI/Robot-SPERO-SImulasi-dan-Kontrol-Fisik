#!/usr/bin/env python3
"""
Debug script untuk memverifikasi apakah test_robot.py bisa menerima data MQTT
"""

import paho.mqtt.client as mqtt
import json
import time
import threading

# MQTT Configuration (sama dengan test_robot.py)
MQTT_BROKER = "codex.petra.ac.id"
MQTT_PORT = 1883
TOPIC_LIDAR_REAL_WORLD_DATA = "robot/lidar/real_world_data"
TOPIC_LIDAR_STATUS = "robot/lidar/status"

class DebugMQTTTester:
    def __init__(self):
        self.mqtt_client = None
        self.received_messages = []
        self.init_mqtt()
    
    def init_mqtt(self):
        """Initialize MQTT connection dengan konfigurasi yang sama dengan test_robot.py"""
        try:
            # Gunakan konfigurasi yang sama dengan test_robot.py
            self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.on_message = self.on_mqtt_message
            
            print(f"[DEBUG] Connecting to {MQTT_BROKER}:{MQTT_PORT}...")
            self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.mqtt_client.loop_start()
            print("[DEBUG] MQTT connected successfully")
            
        except Exception as e:
            print(f"[ERROR] MQTT connection failed: {str(e)}")
            self.mqtt_client = None
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("[DEBUG] Connected to MQTT broker")
            # Subscribe ke topic yang sama dengan test_robot.py
            client.subscribe(TOPIC_LIDAR_REAL_WORLD_DATA)
            client.subscribe(TOPIC_LIDAR_STATUS)
            print(f"[DEBUG] Subscribed to {TOPIC_LIDAR_REAL_WORLD_DATA}")
            print(f"[DEBUG] Subscribed to {TOPIC_LIDAR_STATUS}")
        else:
            print(f"[ERROR] MQTT connection failed with code {rc}")
    
    def on_mqtt_message(self, client, userdata, msg):
        """Handle incoming MQTT messages dengan logging yang detail"""
        try:
            timestamp = time.time()
            print(f"\n[DEBUG] Received message at {time.strftime('%H:%M:%S')}:")
            print(f"  Topic: {msg.topic}")
            print(f"  Payload length: {len(msg.payload)} bytes")
            
            if msg.topic == TOPIC_LIDAR_REAL_WORLD_DATA:
                try:
                    lidar_data = json.loads(msg.payload.decode())
                    print(f"  [LIDAR DATA] Successfully parsed JSON:")
                    print(f"    Source: {lidar_data.get('source', 'unknown')}")
                    print(f"    Ranges: {len(lidar_data.get('ranges', []))} points")
                    print(f"    Timestamp: {lidar_data.get('timestamp', 'unknown')}")
                    
                    # Check data validity
                    ranges = lidar_data.get('ranges', [])
                    if ranges:
                        valid_ranges = [r for r in ranges if r != float('inf')]
                        print(f"    Valid points: {len(valid_ranges)}/{len(ranges)}")
                        if valid_ranges:
                            min_dist = min(valid_ranges)
                            print(f"    Min distance: {min_dist:.3f}m")
                        else:
                            print(f"    WARNING: No valid distance data!")
                    
                    # Simulate test_robot.py's handle_real_world_lidar_data function
                    self.simulate_handle_real_world_lidar_data(lidar_data)
                    
                except json.JSONDecodeError as e:
                    print(f"  [ERROR] Failed to parse JSON: {str(e)}")
                    print(f"  [ERROR] Raw payload: {msg.payload}")
                except Exception as e:
                    print(f"  [ERROR] Failed to process LiDAR data: {str(e)}")
            
            elif msg.topic == TOPIC_LIDAR_STATUS:
                try:
                    status_data = json.loads(msg.payload.decode())
                    print(f"  [LIDAR STATUS] {status_data.get('message', 'Unknown status')}")
                except Exception as e:
                    print(f"  [ERROR] Failed to process status data: {str(e)}")
            
            # Store message
            self.received_messages.append({
                'timestamp': timestamp,
                'topic': msg.topic,
                'payload_length': len(msg.payload)
            })
            
        except Exception as e:
            print(f"[ERROR] Failed to process MQTT message: {str(e)}")
            import traceback
            traceback.print_exc()
    
    def simulate_handle_real_world_lidar_data(self, lidar_data):
        """Simulate test_robot.py's handle_real_world_lidar_data function"""
        try:
            print(f"  [SIMULATION] Simulating test_robot.py's handle_real_world_lidar_data:")
            
            if 'ranges' in lidar_data and 'source' in lidar_data:
                if lidar_data['source'] == 'real_world_lidar':
                    print(f"    ✓ Source matches 'real_world_lidar'")
                    
                    ranges = lidar_data['ranges']
                    min_distance = min([r for r in ranges if r != float('inf')], default=float('inf'))
                    
                    if min_distance != float('inf'):
                        print(f"    ✓ Closest obstacle: {min_distance:.3f}m")
                        
                        # Check for obstacles in different directions (sama dengan test_robot.py)
                        front_range = ranges[0:30] + ranges[330:360]  # 60° front
                        left_range = ranges[30:90]  # 60° left
                        right_range = ranges[270:330]  # 60° right
                        back_range = ranges[150:210]  # 60° back
                        
                        front_min = min([r for r in front_range if r != float('inf')], default=float('inf'))
                        left_min = min([r for r in left_range if r != float('inf')], default=float('inf'))
                        right_min = min([r for r in right_range if r != float('inf')], default=float('inf'))
                        back_min = min([r for r in back_range if r != float('inf')], default=float('inf'))
                        
                        print(f"    ✓ Direction analysis:")
                        print(f"      Front: {front_min:.3f}m, Left: {left_min:.3f}m, Right: {right_min:.3f}m, Back: {back_min:.3f}m")
                        
                        # Emergency stop check
                        if min_distance < 0.3:  # 30cm
                            print(f"    ⚠️ EMERGENCY: Very close obstacle detected: {min_distance:.3f}m")
                        else:
                            print(f"    ✓ Safe distance maintained")
                    else:
                        print(f"    ⚠️ No valid distance data found")
                else:
                    print(f"    ⚠️ Source doesn't match 'real_world_lidar': {lidar_data.get('source', 'unknown')}")
            else:
                print(f"    ⚠️ Missing required fields: 'ranges' or 'source'")
                
        except Exception as e:
            print(f"    [ERROR] Simulation failed: {str(e)}")
    
    def run_test(self, duration=30):
        """Run the debug test"""
        print(f"=== DEBUG MQTT TEST ===")
        print(f"Testing MQTT reception for {duration} seconds...")
        print(f"This simulates test_robot.py's MQTT setup")
        print(f"Make sure test_lidar.py is running to send data")
        print("=" * 50)
        
        start_time = time.time()
        message_count = 0
        
        try:
            while time.time() - start_time < duration:
                current_count = len(self.received_messages)
                if current_count > message_count:
                    print(f"[PROGRESS] Received {current_count} messages so far...")
                    message_count = current_count
                
                time.sleep(1)
                
        except KeyboardInterrupt:
            print("\n[TEST] Interrupted by user")
        
        # Print summary
        print("\n" + "=" * 50)
        print("DEBUG TEST SUMMARY:")
        print(f"Total messages received: {len(self.received_messages)}")
        
        if self.received_messages:
            print("\nLast 5 messages:")
            for i, msg in enumerate(self.received_messages[-5:]):
                print(f"  {i+1}. {time.strftime('%H:%M:%S', time.localtime(msg['timestamp']))} - {msg['topic']} ({msg['payload_length']} bytes)")
        
        if len(self.received_messages) == 0:
            print("\n❌ NO MESSAGES RECEIVED!")
            print("This means test_robot.py will also not receive messages")
            print("Possible issues:")
            print("  1. test_lidar.py is not running")
            print("  2. MQTT broker is not accessible")
            print("  3. Network connectivity issues")
            print("  4. Topic subscription failed")
        else:
            print(f"\n✅ SUCCESS! Received {len(self.received_messages)} messages")
            print("test_robot.py should be able to receive the same data")
            print("If test_robot.py is not receiving data, check:")
            print("  1. MQTT client initialization")
            print("  2. Message processing in on_mqtt_message")
            print("  3. handle_real_world_lidar_data function")
        
        # Cleanup
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()

def main():
    tester = DebugMQTTTester()
    tester.run_test(30)  # Test for 30 seconds

if __name__ == "__main__":
    main() 