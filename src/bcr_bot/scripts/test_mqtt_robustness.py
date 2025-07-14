#!/usr/bin/env python3
"""
Test MQTT Robustness for LiDAR A1M8
This script tests the robust MQTT connection handling with disconnect/reconnect scenarios
"""

import time
import json
import math
import threading
import paho.mqtt.client as mqtt
import uuid
import sys
import os

# MQTT Configuration
MQTT_BROKER = "codex.petra.ac.id"
MQTT_PORT = 1883
MQTT_TOPIC_LIDAR_DATA = "robot/lidar/real_world_data"
MQTT_TOPIC_STATUS = "robot/lidar/status"

class MQTTRobustnessTester:
    def __init__(self):
        self.mqtt_client = None
        self.mqtt_connected = False
        self.running = True
        
        # MQTT reconnection tracking
        self.mqtt_reconnect_attempts = 0
        self.mqtt_max_reconnect_attempts = 10
        self.mqtt_reconnect_delay = 1.0
        self.mqtt_reconnect_timer = None
        self.mqtt_connection_time = None
        
        # Test statistics
        self.message_count = 0
        self.disconnect_count = 0
        self.reconnect_count = 0
        self.last_test_time = time.time()
        
        # Initialize MQTT with robust connection handling
        self.init_mqtt()
    
    def init_mqtt(self):
        """Initialize MQTT client with robust connection handling"""
        try:
            # Generate unique client ID
            client_id = f"robust_test_{uuid.uuid4().hex[:8]}"
            
            self.mqtt_client = mqtt.Client(client_id=client_id)
            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
            self.mqtt_client.on_message = self.on_mqtt_message
            
            # Set MQTT options for better stability
            self.mqtt_client.reconnect_delay_set(min_delay=1, max_delay=120)
            self.mqtt_client._clean_session = True
            
            # Connect to MQTT broker
            self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.mqtt_client.loop_start()
            
            print(f"[MQTT] Successfully connected to MQTT broker with client ID: {client_id}")
            
        except Exception as e:
            print(f"[ERROR] Failed to connect to MQTT broker: {str(e)}")
            self.mqtt_client = None
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback when MQTT client connects"""
        if rc == 0:
            self.mqtt_connected = True
            self.mqtt_connection_time = time.time()
            self.reconnect_count += 1
            
            print(f"[MQTT] Connected successfully (reconnect #{self.reconnect_count})")
            
            # Subscribe to topics
            topics = [
                (MQTT_TOPIC_LIDAR_DATA, 1),
                (MQTT_TOPIC_STATUS, 1)
            ]
            
            for topic, qos in topics:
                try:
                    client.subscribe(topic, qos)
                    print(f"[MQTT] Subscribed to {topic} with QoS {qos}")
                except Exception as e:
                    print(f"[ERROR] Failed to subscribe to {topic}: {str(e)}")
            
            # Send ready message
            try:
                ready_message = {
                    "status": "ready",
                    "timestamp": time.time(),
                    "message": "MQTT robustness tester connected",
                    "source": "robust_test",
                    "reconnect": self.reconnect_count > 1
                }
                client.publish(MQTT_TOPIC_STATUS, json.dumps(ready_message))
                print("[MQTT] Sent ready message")
            except Exception as e:
                print(f"[ERROR] Failed to send ready message: {str(e)}")
                
        else:
            self.mqtt_connected = False
            print(f"[ERROR] Failed to connect to MQTT broker with result code: {rc}")
    
    def on_mqtt_disconnect(self, client, userdata, rc):
        """Callback when MQTT client disconnects"""
        self.mqtt_connected = False
        self.disconnect_count += 1
        
        print(f"[MQTT] Disconnected with code {rc} (disconnect #{self.disconnect_count})")
        
        # Try to reconnect if not shutting down
        if self.running and rc != 0:
            print("[MQTT] Attempting to reconnect...")
            try:
                time.sleep(2.0)
                client.reconnect()
            except Exception as e:
                print(f"[ERROR] Failed to reconnect: {str(e)}")
                try:
                    time.sleep(5.0)
                    client.reconnect()
                except Exception as e2:
                    print(f"[ERROR] Second reconnection attempt failed: {str(e2)}")
                    self.schedule_reconnection()
    
    def schedule_reconnection(self):
        """Schedule another reconnection attempt with exponential backoff"""
        if not self.running or self.mqtt_reconnect_attempts >= self.mqtt_max_reconnect_attempts:
            print("[MQTT] Max reconnection attempts reached, giving up")
            return
        
        self.mqtt_reconnect_attempts += 1
        
        # Calculate delay with exponential backoff
        delay = min(self.mqtt_reconnect_delay * (2 ** (self.mqtt_reconnect_attempts - 1)), 60.0)
        
        print(f"[MQTT] Scheduling reconnection attempt {self.mqtt_reconnect_attempts}/{self.mqtt_max_reconnect_attempts} in {delay:.1f}s")
        
        # Schedule reconnection using a timer
        if self.mqtt_reconnect_timer is not None:
            self.mqtt_reconnect_timer.cancel()
        
        self.mqtt_reconnect_timer = threading.Timer(delay, self.attempt_reconnection)
        self.mqtt_reconnect_timer.daemon = True
        self.mqtt_reconnect_timer.start()
    
    def attempt_reconnection(self):
        """Attempt to reconnect to MQTT broker"""
        if not self.running or self.mqtt_client is None:
            return
        
        try:
            print(f"[MQTT] Attempting reconnection (attempt {self.mqtt_reconnect_attempts})")
            self.mqtt_client.reconnect()
            
            # If successful, reset reconnection attempts
            self.mqtt_reconnect_attempts = 0
            self.mqtt_reconnect_delay = 1.0
            print("[MQTT] Reconnection successful")
            
        except Exception as e:
            print(f"[ERROR] Reconnection attempt {self.mqtt_reconnect_attempts} failed: {str(e)}")
            self.schedule_reconnection()
    
    def on_mqtt_message(self, client, userdata, msg):
        """Callback when MQTT message is received"""
        try:
            self.message_count += 1
            
            if msg.topic == MQTT_TOPIC_LIDAR_DATA:
                data = json.loads(msg.payload.decode())
                ranges = data.get('ranges', [])
                valid_points = sum(1 for r in ranges if r != float('inf'))
                
                print(f"[MQTT] Received LiDAR data: {len(ranges)} points, {valid_points} valid")
                
            elif msg.topic == MQTT_TOPIC_STATUS:
                data = json.loads(msg.payload.decode())
                print(f"[MQTT] Received status: {data.get('message', 'Unknown')}")
                
        except Exception as e:
            print(f"[ERROR] Error processing MQTT message: {str(e)}")
    
    def send_test_lidar_data(self, scenario_name="test"):
        """Send test LiDAR data"""
        if not self.mqtt_connected or self.mqtt_client is None:
            print("[ERROR] MQTT not connected, cannot send data")
            return False
        
        try:
            # Create test LiDAR data (360 points, excluding rear sector 135°-225°)
            ranges = [float('inf')] * 360
            
            # Set some test distances
            ranges[0] = 2.0      # Front
            ranges[90] = 1.5     # Left
            ranges[270] = 1.5    # Right
            
            # Filter out rear sector (135°-225°)
            for i in range(135, 226):
                ranges[i] = float('inf')
            
            # Create data packet
            lidar_data = {
                "timestamp": time.time(),
                "ranges": ranges,
                "angle_min": -math.pi,
                "angle_max": math.pi,
                "angle_increment": 2 * math.pi / 360,
                "range_min": 0.15,
                "range_max": 12.0,
                "source": "robust_test",
                "scenario": scenario_name,
                "rear_excluded": True
            }
            
            # Send via MQTT
            self.mqtt_client.publish(MQTT_TOPIC_LIDAR_DATA, json.dumps(lidar_data))
            
            print(f"[TEST] Sent LiDAR data: {scenario_name}")
            return True
            
        except Exception as e:
            print(f"[ERROR] Failed to send test LiDAR data: {str(e)}")
            return False
    
    def print_statistics(self):
        """Print test statistics"""
        current_time = time.time()
        uptime = current_time - self.last_test_time
        
        print("\n" + "="*60)
        print("MQTT ROBUSTNESS TEST STATISTICS")
        print("="*60)
        print(f"Uptime: {uptime:.1f} seconds")
        print(f"Messages received: {self.message_count}")
        print(f"Disconnects: {self.disconnect_count}")
        print(f"Reconnects: {self.reconnect_count}")
        print(f"Current connection: {'Connected' if self.mqtt_connected else 'Disconnected'}")
        print(f"Reconnection attempts: {self.mqtt_reconnect_attempts}/{self.mqtt_max_reconnect_attempts}")
        
        if self.mqtt_connected and self.mqtt_connection_time:
            connection_uptime = current_time - self.mqtt_connection_time
            print(f"Current connection uptime: {connection_uptime:.1f} seconds")
        
        print("="*60)
    
    def run_test_scenarios(self):
        """Run various test scenarios"""
        print("\n=== RUNNING MQTT ROBUSTNESS TESTS ===")
        
        scenarios = [
            ("clear", "Clear path"),
            ("front_obstacle", "Front obstacle"),
            ("left_obstacle", "Left obstacle"),
            ("right_obstacle", "Right obstacle"),
            ("both_sides", "Both sides blocked")
        ]
        
        try:
            while self.running:
                for scenario_name, scenario_desc in scenarios:
                    if not self.running:
                        break
                    
                    print(f"\n[TEST] Running scenario: {scenario_desc}")
                    success = self.send_test_lidar_data(scenario_name)
                    
                    if success:
                        print(f"[TEST] ✓ Scenario '{scenario_desc}' sent successfully")
                    else:
                        print(f"[TEST] ✗ Scenario '{scenario_desc}' failed")
                    
                    # Print statistics every few scenarios
                    if scenario_name in ["right_obstacle", "both_sides"]:
                        self.print_statistics()
                    
                    time.sleep(3.0)  # Wait between scenarios
                
                # Print final statistics
                self.print_statistics()
                
        except KeyboardInterrupt:
            print("\n[TEST] Stopped by user")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Cleanup resources"""
        print("\n[TEST] Cleaning up...")
        self.running = False
        
        if self.mqtt_reconnect_timer is not None:
            self.mqtt_reconnect_timer.cancel()
        
        if self.mqtt_client is not None:
            try:
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
            except Exception as e:
                print(f"[WARNING] Could not disconnect MQTT client: {str(e)}")
        
        print("[TEST] Cleanup completed")

def main():
    """Main function"""
    print("=== MQTT ROBUSTNESS TESTER FOR LIDAR A1M8 ===")
    print("This script tests robust MQTT connection handling")
    print("Press Ctrl+C to stop")
    print("=" * 60)
    
    tester = MQTTRobustnessTester()
    
    try:
        # Wait for initial connection
        time.sleep(2.0)
        
        # Run test scenarios
        tester.run_test_scenarios()
        
    except KeyboardInterrupt:
        print("\n[MAIN] Stopped by user")
    except Exception as e:
        print(f"\n[ERROR] Main error: {str(e)}")
    finally:
        tester.cleanup()

if __name__ == '__main__':
    main() 