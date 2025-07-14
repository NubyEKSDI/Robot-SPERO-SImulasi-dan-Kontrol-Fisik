#!/usr/bin/env python3
"""
A1M8 LiDAR Robust MQTT Handler
This script ensures A1M8 LiDAR never disconnects and auto-reconnects if connection is lost
"""

import time
import json
import math
import threading
import paho.mqtt.client as mqtt
import uuid
import sys
import os
import signal
from datetime import datetime

# MQTT Configuration
MQTT_BROKER = "codex.petra.ac.id"
MQTT_PORT = 1883
MQTT_TOPIC_LIDAR_DATA = "robot/lidar/real_world_data"
MQTT_TOPIC_STATUS = "robot/lidar/status"
MQTT_TOPIC_HEALTH = "robot/lidar/health"

class A1M8RobustLiDAR:
    def __init__(self):
        self.mqtt_client = None
        self.mqtt_connected = False
        self.running = True
        self.client_id = f"a1m8_lidar_{uuid.uuid4().hex[:8]}"
        
        # Connection tracking
        self.connection_start_time = None
        self.last_connection_time = None
        self.disconnect_count = 0
        self.reconnect_count = 0
        self.total_messages_sent = 0
        self.last_message_time = time.time()
        
        # Reconnection settings
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 999999  # Never give up
        self.reconnect_delay = 1.0
        self.max_reconnect_delay = 60.0
        self.reconnect_timer = None
        
        # Health monitoring
        self.health_check_interval = 30.0  # seconds
        self.last_health_check = time.time()
        self.health_status = "OK"
        
        # LiDAR data simulation (for testing)
        self.lidar_data_counter = 0
        self.simulation_mode = True  # Set to False for real LiDAR
        
        # Set up signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        print(f"[A1M8] Initializing robust LiDAR handler with client ID: {self.client_id}")
        self.init_mqtt()
    
    def signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully"""
        print(f"\n[A1M8] Received signal {signum}, shutting down gracefully...")
        self.running = False
        self.cleanup()
        sys.exit(0)
    
    def init_mqtt(self):
        """Initialize MQTT client with robust connection handling"""
        try:
            # Create MQTT client with unique ID
            self.mqtt_client = mqtt.Client(client_id=self.client_id)
            
            # Set up callbacks
            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
            self.mqtt_client.on_message = self.on_mqtt_message
            
            # Configure for maximum reliability
            self.mqtt_client.reconnect_delay_set(min_delay=1, max_delay=120)
            self.mqtt_client._clean_session = True
            
            # Set QoS for reliable delivery
            self.mqtt_client.qos = 1
            
            print(f"[A1M8] MQTT client initialized with client ID: {self.client_id}")
            
        except Exception as e:
            print(f"[A1M8 ERROR] Failed to initialize MQTT client: {str(e)}")
            self.schedule_reconnection()
    
    def connect_mqtt(self):
        """Connect to MQTT broker with error handling"""
        if not self.running:
            return False
        
        try:
            print(f"[A1M8] Attempting to connect to MQTT broker: {MQTT_BROKER}:{MQTT_PORT}")
            
            # Connect to broker
            result = self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
            
            if result == mqtt.MQTT_ERR_SUCCESS:
                # Start the network loop
                self.mqtt_client.loop_start()
                print(f"[A1M8] MQTT connection initiated successfully")
                return True
            else:
                print(f"[A1M8 ERROR] MQTT connection failed with result: {result}")
                return False
                
        except Exception as e:
            print(f"[A1M8 ERROR] Exception during MQTT connection: {str(e)}")
            return False
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback when MQTT client connects"""
        if rc == 0:
            self.mqtt_connected = True
            self.connection_start_time = time.time()
            self.last_connection_time = time.time()
            self.reconnect_count += 1
            
            # Reset reconnection attempts on successful connection
            self.reconnect_attempts = 0
            self.reconnect_delay = 1.0
            
            print(f"[A1M8] ✓ Connected to MQTT broker successfully (reconnect #{self.reconnect_count})")
            
            # Subscribe to topics with QoS 1 for reliability
            topics = [
                (MQTT_TOPIC_STATUS, 1),
                (MQTT_TOPIC_HEALTH, 1)
            ]
            
            for topic, qos in topics:
                try:
                    client.subscribe(topic, qos)
                    print(f"[A1M8] Subscribed to {topic} with QoS {qos}")
                except Exception as e:
                    print(f"[A1M8 ERROR] Failed to subscribe to {topic}: {str(e)}")
            
            # Send ready message
            self.send_status_message("ready", "A1M8 LiDAR is ready and connected")
            
            # Start health monitoring
            self.start_health_monitoring()
            
        else:
            self.mqtt_connected = False
            print(f"[A1M8 ERROR] Failed to connect to MQTT broker with result code: {rc}")
            self.schedule_reconnection()
    
    def on_mqtt_disconnect(self, client, userdata, rc):
        """Callback when MQTT client disconnects"""
        self.mqtt_connected = False
        self.disconnect_count += 1
        
        print(f"[A1M8] ⚠️ Disconnected from MQTT broker (disconnect #{self.disconnect_count}, code: {rc})")
        
        # Always try to reconnect unless shutting down
        if self.running:
            print(f"[A1M8] 🔄 Initiating automatic reconnection...")
            self.schedule_reconnection()
    
    def schedule_reconnection(self):
        """Schedule reconnection with exponential backoff"""
        if not self.running:
            return
        
        self.reconnect_attempts += 1
        
        # Calculate delay with exponential backoff (1s, 2s, 4s, 8s, 16s, 32s, 60s, 60s...)
        delay = min(self.reconnect_delay * (2 ** (self.reconnect_attempts - 1)), self.max_reconnect_delay)
        
        print(f"[A1M8] 🔄 Scheduling reconnection attempt {self.reconnect_attempts} in {delay:.1f}s")
        
        # Cancel existing timer if any
        if self.reconnect_timer is not None:
            self.reconnect_timer.cancel()
        
        # Schedule reconnection
        self.reconnect_timer = threading.Timer(delay, self.attempt_reconnection)
        self.reconnect_timer.daemon = True
        self.reconnect_timer.start()
    
    def attempt_reconnection(self):
        """Attempt to reconnect to MQTT broker"""
        if not self.running:
            return
        
        print(f"[A1M8] 🔄 Attempting reconnection (attempt {self.reconnect_attempts})")
        
        try:
            # Try to connect
            if self.connect_mqtt():
                print(f"[A1M8] ✓ Reconnection attempt {self.reconnect_attempts} successful")
            else:
                print(f"[A1M8] ✗ Reconnection attempt {self.reconnect_attempts} failed")
                # Schedule next attempt
                self.schedule_reconnection()
                
        except Exception as e:
            print(f"[A1M8 ERROR] Exception during reconnection attempt {self.reconnect_attempts}: {str(e)}")
            # Schedule next attempt
            self.schedule_reconnection()
    
    def on_mqtt_message(self, client, userdata, msg):
        """Callback when MQTT message is received"""
        try:
            if msg.topic == MQTT_TOPIC_STATUS:
                data = json.loads(msg.payload.decode())
                print(f"[A1M8] Received status: {data.get('message', 'Unknown')}")
                
            elif msg.topic == MQTT_TOPIC_HEALTH:
                data = json.loads(msg.payload.decode())
                print(f"[A1M8] Health check: {data.get('status', 'Unknown')}")
                
        except Exception as e:
            print(f"[A1M8 ERROR] Error processing MQTT message: {str(e)}")
    
    def send_lidar_data(self, ranges=None):
        """Send LiDAR data via MQTT"""
        if not self.mqtt_connected or self.mqtt_client is None:
            print("[A1M8] Cannot send LiDAR data - MQTT not connected")
            return False
        
        try:
            # Generate or use provided LiDAR data
            if ranges is None:
                ranges = self.generate_test_lidar_data()
            
            # Create LiDAR data packet
            lidar_data = {
                "timestamp": time.time(),
                "ranges": ranges,
                "angle_min": -math.pi,
                "angle_max": math.pi,
                "angle_increment": 2 * math.pi / 360,
                "range_min": 0.15,
                "range_max": 12.0,
                "source": "A1M8",
                "client_id": self.client_id,
                "message_id": self.lidar_data_counter,
                "rear_excluded": True  # Exclude rear sector
            }
            
            # Send via MQTT with QoS 1 for reliability
            result = self.mqtt_client.publish(MQTT_TOPIC_LIDAR_DATA, json.dumps(lidar_data), qos=1)
            
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                self.total_messages_sent += 1
                self.last_message_time = time.time()
                self.lidar_data_counter += 1
                return True
            else:
                print(f"[A1M8 ERROR] Failed to publish LiDAR data: {result.rc}")
                return False
                
        except Exception as e:
            print(f"[A1M8 ERROR] Exception sending LiDAR data: {str(e)}")
            return False
    
    def generate_test_lidar_data(self):
        """Generate test LiDAR data (360 points, excluding rear sector)"""
        import random
        
        ranges = [float('inf')] * 360
        
        # Set some test distances
        ranges[0] = 2.0      # Front
        ranges[90] = 1.5     # Left
        ranges[270] = 1.5    # Right
        
        # Add some random obstacles
        for i in range(10):
            angle = random.randint(0, 359)
            if 135 <= angle <= 225:  # Skip rear sector
                continue
            ranges[angle] = random.uniform(0.5, 3.0)
        
        # Filter out rear sector (135°-225°) - set to inf
        for i in range(135, 226):
            ranges[i] = float('inf')
        
        return ranges
    
    def send_status_message(self, status, message):
        """Send status message via MQTT"""
        if not self.mqtt_connected or self.mqtt_client is None:
            return False
        
        try:
            status_data = {
                "status": status,
                "message": message,
                "timestamp": time.time(),
                "source": "A1M8",
                "client_id": self.client_id,
                "connection_uptime": time.time() - self.connection_start_time if self.connection_start_time else 0,
                "total_messages": self.total_messages_sent,
                "disconnect_count": self.disconnect_count,
                "reconnect_count": self.reconnect_count
            }
            
            self.mqtt_client.publish(MQTT_TOPIC_STATUS, json.dumps(status_data), qos=1)
            return True
            
        except Exception as e:
            print(f"[A1M8 ERROR] Failed to send status message: {str(e)}")
            return False
    
    def start_health_monitoring(self):
        """Start health monitoring thread"""
        def health_monitor():
            while self.running:
                try:
                    current_time = time.time()
                    
                    # Check if we need to send health update
                    if current_time - self.last_health_check >= self.health_check_interval:
                        self.send_health_update()
                        self.last_health_check = current_time
                    
                    # Check connection health
                    if self.mqtt_connected:
                        # Check if we haven't sent data recently
                        if current_time - self.last_message_time > 60.0:  # 1 minute
                            print("[A1M8] ⚠️ No LiDAR data sent recently, sending test data")
                            self.send_lidar_data()
                    
                    time.sleep(5.0)  # Check every 5 seconds
                    
                except Exception as e:
                    print(f"[A1M8 ERROR] Health monitoring error: {str(e)}")
                    time.sleep(5.0)
        
        # Start health monitoring in separate thread
        health_thread = threading.Thread(target=health_monitor, daemon=True)
        health_thread.start()
        print("[A1M8] Health monitoring started")
    
    def send_health_update(self):
        """Send health update via MQTT"""
        if not self.mqtt_connected or self.mqtt_client is None:
            return False
        
        try:
            health_data = {
                "status": self.health_status,
                "timestamp": time.time(),
                "source": "A1M8",
                "client_id": self.client_id,
                "connection_uptime": time.time() - self.connection_start_time if self.connection_start_time else 0,
                "total_messages": self.total_messages_sent,
                "disconnect_count": self.disconnect_count,
                "reconnect_count": self.reconnect_count,
                "last_message_time": self.last_message_time,
                "mqtt_connected": self.mqtt_connected
            }
            
            self.mqtt_client.publish(MQTT_TOPIC_HEALTH, json.dumps(health_data), qos=1)
            return True
            
        except Exception as e:
            print(f"[A1M8 ERROR] Failed to send health update: {str(e)}")
            return False
    
    def start_lidar_streaming(self):
        """Start continuous LiDAR data streaming"""
        print("[A1M8] Starting LiDAR data streaming...")
        
        def lidar_stream():
            while self.running:
                try:
                    if self.mqtt_connected:
                        # Send LiDAR data
                        success = self.send_lidar_data()
                        if success:
                            print(f"[A1M8] ✓ LiDAR data sent (message #{self.lidar_data_counter})")
                        else:
                            print("[A1M8] ✗ Failed to send LiDAR data")
                    else:
                        print("[A1M8] ⏳ Waiting for MQTT connection...")
                    
                    # Send data every 100ms (10Hz)
                    time.sleep(0.1)
                    
                except Exception as e:
                    print(f"[A1M8 ERROR] LiDAR streaming error: {str(e)}")
                    time.sleep(1.0)
        
        # Start LiDAR streaming in separate thread
        lidar_thread = threading.Thread(target=lidar_stream, daemon=True)
        lidar_thread.start()
        print("[A1M8] LiDAR streaming started")
    
    def print_statistics(self):
        """Print connection statistics"""
        current_time = time.time()
        uptime = current_time - self.connection_start_time if self.connection_start_time else 0
        
        print("\n" + "="*60)
        print("A1M8 LIDAR ROBUST CONNECTION STATISTICS")
        print("="*60)
        print(f"Client ID: {self.client_id}")
        print(f"MQTT Connected: {'✓ Yes' if self.mqtt_connected else '✗ No'}")
        print(f"Connection Uptime: {uptime:.1f} seconds")
        print(f"Total Messages Sent: {self.total_messages_sent}")
        print(f"Disconnects: {self.disconnect_count}")
        print(f"Reconnects: {self.reconnect_count}")
        print(f"Current Reconnection Attempts: {self.reconnect_attempts}")
        print(f"Last Message Time: {datetime.fromtimestamp(self.last_message_time).strftime('%H:%M:%S')}")
        print(f"Health Status: {self.health_status}")
        print("="*60)
    
    def cleanup(self):
        """Cleanup resources"""
        print("\n[A1M8] Cleaning up...")
        self.running = False
        
        # Cancel reconnection timer
        if self.reconnect_timer is not None:
            self.reconnect_timer.cancel()
        
        # Disconnect MQTT
        if self.mqtt_client is not None:
            try:
                # Send final status message
                self.send_status_message("shutdown", "A1M8 LiDAR shutting down")
                
                # Stop loop and disconnect
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
                print("[A1M8] ✓ MQTT client disconnected")
            except Exception as e:
                print(f"[A1M8 WARNING] Could not disconnect MQTT client: {str(e)}")
        
        # Print final statistics
        self.print_statistics()
        print("[A1M8] ✓ Cleanup completed")

def main():
    """Main function"""
    print("="*60)
    print("A1M8 LIDAR ROBUST MQTT HANDLER")
    print("="*60)
    print("This script ensures A1M8 LiDAR never disconnects")
    print("Features:")
    print("- Automatic reconnection with exponential backoff")
    print("- Never gives up trying to reconnect")
    print("- Health monitoring and status reporting")
    print("- Robust error handling")
    print("="*60)
    
    # Create A1M8 LiDAR handler
    lidar_handler = A1M8RobustLiDAR()
    
    try:
        # Start LiDAR streaming
        lidar_handler.start_lidar_streaming()
        
        print("\n[A1M8] LiDAR handler is running...")
        print("Press Ctrl+C to stop")
        
        # Main loop - just keep the program running
        while lidar_handler.running:
            try:
                # Print statistics every 30 seconds
                time.sleep(30.0)
                lidar_handler.print_statistics()
                
            except KeyboardInterrupt:
                print("\n[A1M8] Received keyboard interrupt")
                break
            except Exception as e:
                print(f"[A1M8 ERROR] Main loop error: {str(e)}")
                time.sleep(5.0)
                continue
                
    except KeyboardInterrupt:
        print("\n[A1M8] Stopped by user")
    except Exception as e:
        print(f"\n[A1M8 ERROR] Main error: {str(e)}")
    finally:
        lidar_handler.cleanup()

if __name__ == '__main__':
    main() 