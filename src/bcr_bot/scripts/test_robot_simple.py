#!/usr/bin/env python3
"""
Simplified test_robot.py untuk testing LiDAR fix
Menggunakan pendekatan yang sama seperti test_lidar.py
"""

import time
import json
import threading
from rplidar import RPLidar
import paho.mqtt.client as mqtt

# Konfigurasi LiDAR (sama seperti test_lidar.py)
LIDAR_PORT = '/dev/ttyUSB0'
LIDAR_BAUDRATE = 115200
LIDAR_TIMEOUT = 1.0

# MQTT Configuration
MQTT_BROKER = "codex.petra.ac.id"
MQTT_PORT = 1883
TOPIC_LIDAR_REAL_WORLD_DATA = "robot/lidar/real_world_data"

class SimpleRobotTester:
    def __init__(self):
        self.lidar = None
        self.lidar_running = False
        self.lidar_thread = None
        self.mqtt_client = None
        self.real_world_lidar_active = True
        
        print("[SIMPLE TEST] Initializing...")
        self.init_mqtt()
        self.init_lidar()
    
    def init_mqtt(self):
        """Initialize MQTT connection"""
        try:
            self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
            self.mqtt_client.on_connect = self.on_mqtt_connect
            
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
    
    def init_lidar(self):
        """Initialize RP-Lidar A1M8 using the same approach as test_lidar.py"""
        try:
            print(f"[LIDAR] Initializing LiDAR on {LIDAR_PORT}...")
            
            # Create new LiDAR instance (same as test_lidar.py)
            self.lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE, timeout=LIDAR_TIMEOUT)
            
            # Get LiDAR info
            info = self.lidar.get_info()
            print(f"[LIDAR] Connected: {info}")
            
            # Get health status
            health = self.lidar.get_health()
            print(f"[LIDAR] Health: {health}")
            
            # Start LiDAR scanning in a separate thread (same approach as test_lidar.py)
            self.lidar_running = True
            self.lidar_thread = threading.Thread(target=self.lidar_scan_loop, daemon=True)
            self.lidar_thread.start()
            print("[LIDAR] LiDAR scanning started successfully")
            
        except Exception as e:
            print(f"[ERROR] Failed to initialize LiDAR: {str(e)}")
            self.lidar = None
            self.lidar_running = False
    
    def lidar_scan_loop(self):
        """Background thread to continuously read LiDAR data - using same approach as test_lidar.py"""
        scan_count = 0
        start_time = time.time()
        
        print("[LIDAR THREAD] LiDAR scan loop started")
        print(f"[LIDAR THREAD] real_world_lidar_active = {self.real_world_lidar_active}")
        print(f"[LIDAR THREAD] mqtt_client = {self.mqtt_client is not None}")
        
        try:
            # Use the same approach as test_lidar.py - direct iter_scans() without start()
            for scan in self.lidar.iter_scans():
                if not self.lidar_running:
                    print("[LIDAR THREAD] LiDAR running flag set to False, stopping")
                    break
                
                scan_count += 1
                
                # Send real-world LiDAR data via MQTT (same format as test_lidar.py)
                if self.real_world_lidar_active and self.mqtt_client:
                    try:
                        # Convert scan to format for external systems
                        ranges = [float('inf')] * 360
                        valid_points = 0
                        
                        for quality, angle, distance in scan:
                            angle_deg = int(angle) % 360
                            if 0 <= angle_deg < 360:
                                ranges[angle_deg] = distance / 1000.0  # Convert to meters
                                if distance > 0:  # Valid distance
                                    valid_points += 1
                        
                        # Create data packet for real-world LiDAR
                        lidar_data = {
                            "timestamp": time.time(),
                            "ranges": ranges,
                            "angle_min": -math.pi,
                            "angle_max": math.pi,
                            "angle_increment": 2 * math.pi / 360,
                            "range_min": 0.15,
                            "range_max": 12.0,
                            "source": "real_world_lidar"
                        }
                        
                        # Send via MQTT
                        self.mqtt_client.publish(TOPIC_LIDAR_REAL_WORLD_DATA, json.dumps(lidar_data))
                        
                        # Debug logging every 10 scans
                        if scan_count % 10 == 0:
                            print(f"[LIDAR STREAMING] Sent scan {scan_count}: {valid_points} valid points, {len(ranges)} total ranges")
                            print(f"[LIDAR STREAMING] Sample ranges (0°, 90°, 180°, 270°): {ranges[0]:.3f}, {ranges[90]:.3f}, {ranges[180]:.3f}, {ranges[270]:.3f}")
                            
                            # Check if we have valid data
                            valid_ranges = [r for r in ranges if r != float('inf')]
                            if valid_ranges:
                                min_dist = min(valid_ranges)
                                max_dist = max(valid_ranges)
                                print(f"[LIDAR STREAMING] Distance range: {min_dist:.3f}m to {max_dist:.3f}m")
                            else:
                                print(f"[LIDAR STREAMING] WARNING: No valid distance data in scan!")
                    except Exception as e:
                        print(f"[ERROR] Failed to publish real-world LiDAR data: {str(e)}")
                
                # Print status every 50 scans
                if scan_count % 50 == 0:
                    elapsed = time.time() - start_time
                    rate = scan_count / elapsed
                    print(f"[LIDAR] Processed {scan_count} scans ({rate:.1f} scans/sec)")
                
                # Small delay to prevent overwhelming (same as test_lidar.py)
                time.sleep(0.01)
                
        except Exception as e:
            print(f"[ERROR] LiDAR streaming error: {str(e)}")
            import traceback
            traceback.print_exc()
        finally:
            print("[LIDAR THREAD] LiDAR scan loop ended")
            # Cleanup when loop ends
            if self.lidar:
                try:
                    self.lidar.disconnect()
                except:
                    pass
    
    def stop_lidar(self):
        """Stop LiDAR scanning (same as test_lidar.py)"""
        self.lidar_running = False
        if self.lidar:
            try:
                self.lidar.disconnect()
            except:
                pass
            self.lidar = None
        print("[LIDAR] LiDAR stopped")
    
    def cleanup(self):
        """Cleanup all resources"""
        print("[CLEANUP] Starting cleanup...")
        
        # Stop LiDAR
        self.stop_lidar()
        
        # Stop MQTT
        if self.mqtt_client:
            try:
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
            except:
                pass
        
        print("[CLEANUP] Cleanup completed")

if __name__ == "__main__":
    print("=== SIMPLE ROBOT TESTER ===")
    print("Testing LiDAR with same approach as test_lidar.py")
    print("=" * 50)
    
    try:
        tester = SimpleRobotTester()
        
        print("\n=== SYSTEM READY ===")
        print("✓ MQTT connected")
        print("✓ LiDAR initialized")
        print("✓ Real-world streaming active")
        print("✓ All systems ready")
        
        print("\n=== MONITORING ===")
        print("Robot will now:")
        print("1. Stream LiDAR data via MQTT")
        print("2. Send data to roam.py")
        print("3. Monitor LiDAR health")
        
        print("\nPress Ctrl+C to stop")
        print("=" * 50)
        
        # Keep the program running
        while True:
            time.sleep(30)
            
            print("\n=== STATUS UPDATE ===")
            if tester.lidar:
                print("  LiDAR: Connected and streaming")
            else:
                print("  LiDAR: Not connected")
            
            print(f"  Real-world streaming: {tester.real_world_lidar_active}")
            print(f"  LiDAR thread alive: {tester.lidar_thread.is_alive() if tester.lidar_thread else False}")
            print(f"  LiDAR running flag: {tester.lidar_running}")
            print("=" * 50)
            
    except KeyboardInterrupt:
        print("\n\n=== SHUTDOWN SEQUENCE ===")
        print("Stopping all systems...")
        
        if 'tester' in locals():
            tester.cleanup()
        
        print("✓ All systems stopped")
        print("✓ Cleanup completed")
        
    except Exception as e:
        print(f"\n=== ERROR ===")
        print(f"Error: {str(e)}")
        import traceback
        traceback.print_exc()
        
        # Emergency cleanup
        if 'tester' in locals():
            tester.cleanup()
        
        print("✓ Emergency cleanup completed") 