#!/usr/bin/env python3
"""
Standalone LiDAR test script untuk RP-Lidar A1M8
Mengirim data LiDAR ke roam.py via MQTT
"""

import time
import json
import math
import threading
import paho.mqtt.client as mqtt
from rplidar import RPLidar

# Konfigurasi
LIDAR_PORT = '/dev/ttyUSB0'
LIDAR_BAUDRATE = 115200
LIDAR_TIMEOUT = 1.0
MQTT_BROKER = "codex.petra.ac.id"
MQTT_PORT = 1883

class LidarTester:
    def __init__(self):
        self.lidar = None
        self.mqtt_client = None
        self.running = False
        self.scan_count = 0
        self.error_count = 0
        
    def init_mqtt(self):
        """Initialize MQTT connection"""
        try:
            self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.mqtt_client.loop_start()
            print("[MQTT] Connected to broker")
            return True
        except Exception as e:
            print(f"[ERROR] MQTT connection failed: {str(e)}")
            return False
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("[MQTT] Connected successfully")
        else:
            print(f"[MQTT] Connection failed with code {rc}")
    
    def init_lidar(self):
        """Initialize LiDAR with error handling"""
        max_retries = 3
        retry_count = 0
        
        while retry_count < max_retries:
            try:
                print(f"[LIDAR] Attempting to connect to LiDAR on {LIDAR_PORT} (attempt {retry_count + 1}/{max_retries})...")
                
                # Create LiDAR instance
                self.lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE, timeout=LIDAR_TIMEOUT)
                
                # Get LiDAR info
                info = self.lidar.get_info()
                print(f"[LIDAR] Connected to LiDAR: {info}")
                
                # Get health status
                health = self.lidar.get_health()
                print(f"[LIDAR] Health status: {health}")
                
                # Test basic functionality
                self.lidar.start()
                time.sleep(1.0)
                
                # Try to get one scan to verify it's working
                for scan in self.lidar.iter_scans():
                    if len(scan) > 0:
                        print(f"[LIDAR] First scan successful: {len(scan)} points")
                        break
                
                print("[LIDAR] LiDAR initialization successful!")
                return True
                
            except Exception as e:
                retry_count += 1
                error_msg = str(e)
                print(f"[ERROR] LiDAR initialization attempt {retry_count} failed: {error_msg}")
                
                # Clean up failed connection
                if self.lidar:
                    try:
                        self.lidar.stop()
                        self.lidar.disconnect()
                    except:
                        pass
                    self.lidar = None
                
                # Wait before retry
                if retry_count < max_retries:
                    print(f"[LIDAR] Waiting 2 seconds before retry...")
                    time.sleep(2.0)
        
        print(f"[ERROR] Failed to initialize LiDAR after {max_retries} attempts")
        return False
    
    def process_scan(self, scan):
        """Process raw LiDAR scan data"""
        try:
            # Convert scan data to ranges array (360 degrees)
            ranges = [float('inf')] * 360
            intensities = [0.0] * 360
            
            for quality, angle, distance in scan:
                # Convert angle to degrees and normalize to 0-359
                angle_deg = int(angle) % 360
                if 0 <= angle_deg < 360:
                    ranges[angle_deg] = distance / 1000.0  # Convert mm to meters
                    intensities[angle_deg] = quality / 255.0  # Normalize quality to 0-1
            
            return ranges, intensities
            
        except Exception as e:
            print(f"[ERROR] Error processing LiDAR scan: {str(e)}")
            return None, None
    
    def send_scan_data(self, scan):
        """Send LiDAR data to roam.py via MQTT"""
        if self.mqtt_client is None:
            return
        
        try:
            ranges, intensities = self.process_scan(scan)
            if ranges is None:
                return
            
            # Create LaserScan format data
            scan_data = {
                'header': {
                    'stamp': time.time(),
                    'frame_id': 'lidar_link'
                },
                'angle_min': -math.pi,
                'angle_max': math.pi,
                'angle_increment': 2 * math.pi / 360,  # 1 degree increments
                'time_increment': 0.0,
                'scan_time': 0.0,
                'range_min': 0.15,  # 15cm minimum range for A1M8
                'range_max': 12.0,  # 12m maximum range for A1M8
                'ranges': ranges,
                'intensities': intensities
            }
            
            # Send via MQTT
            self.mqtt_client.publish("robot/physical/scan", json.dumps(scan_data))
            
            # Print status every 10 scans
            self.scan_count += 1
            if self.scan_count % 10 == 0:
                valid_points = sum(1 for r in ranges if r != float('inf'))
                coverage = (valid_points / 360) * 100
                print(f"[LIDAR] Scan {self.scan_count}: {len(scan)} points, coverage: {coverage:.1f}%")
            
        except Exception as e:
            print(f"[ERROR] Error sending LiDAR data: {str(e)}")
    
    def lidar_scan_loop(self):
        """Main LiDAR scanning loop with error recovery"""
        consecutive_errors = 0
        max_consecutive_errors = 5
        
        while self.running:
            try:
                # Start scanning if not already started
                if not hasattr(self.lidar, '_scanning') or not self.lidar._scanning:
                    self.lidar.start()
                    print("[LIDAR] Scanning started")
                
                for scan in self.lidar.iter_scans():
                    if not self.running:
                        break
                    
                    # Reset error counter on successful scan
                    consecutive_errors = 0
                    self.error_count = 0
                    
                    # Send scan data
                    self.send_scan_data(scan)
                    
            except Exception as e:
                consecutive_errors += 1
                self.error_count += 1
                error_msg = str(e)
                
                print(f"[ERROR] LiDAR scan error ({consecutive_errors}/{max_consecutive_errors}): {error_msg}")
                
                # Handle specific errors
                if "Wrong body size" in error_msg:
                    print("[LIDAR] Wrong body size error, attempting reset...")
                    try:
                        self.lidar.stop()
                        time.sleep(2.0)
                        self.lidar.disconnect()
                        time.sleep(1.0)
                        
                        # Reinitialize LiDAR
                        if self.init_lidar():
                            print("[LIDAR] LiDAR reset successful")
                            continue
                        else:
                            print("[ERROR] Failed to reset LiDAR")
                    except Exception as reset_error:
                        print(f"[ERROR] Reset failed: {str(reset_error)}")
                
                # If too many consecutive errors, stop
                if consecutive_errors >= max_consecutive_errors:
                    print(f"[ERROR] Too many consecutive errors ({consecutive_errors}), stopping")
                    break
                
                # Wait before retry
                time.sleep(2.0)
        
        # Cleanup
        if self.lidar:
            try:
                self.lidar.stop()
                self.lidar.disconnect()
            except:
                pass
    
    def run(self):
        """Main run function"""
        print("=== RP-Lidar A1M8 Test with MQTT ===")
        
        # Initialize MQTT
        if not self.init_mqtt():
            print("Failed to initialize MQTT, continuing without MQTT...")
        
        # Initialize LiDAR
        if not self.init_lidar():
            print("Failed to initialize LiDAR")
            return
        
        # Start scanning
        self.running = True
        print("[LIDAR] Starting scan loop...")
        print("Press Ctrl+C to stop")
        
        try:
            self.lidar_scan_loop()
        except KeyboardInterrupt:
            print("\n[LIDAR] Stopping by user request...")
        finally:
            self.running = False
            if self.lidar:
                try:
                    self.lidar.stop()
                    self.lidar.disconnect()
                except:
                    pass
            if self.mqtt_client:
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
            
            print(f"[LIDAR] Final stats: {self.scan_count} scans, {self.error_count} errors")
            print("[LIDAR] Cleanup completed")

def main():
    tester = LidarTester()
    tester.run()

if __name__ == "__main__":
    main() 