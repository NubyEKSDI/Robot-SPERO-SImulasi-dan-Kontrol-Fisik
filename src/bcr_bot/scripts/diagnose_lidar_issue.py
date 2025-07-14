#!/usr/bin/env python3
"""
LiDAR A1M8 Diagnostic Script
Khusus untuk menganalisis masalah deteksi obstacle 0.108m
"""

import time
import math
import json
import paho.mqtt.client as mqtt
from collections import defaultdict, deque
import threading
import numpy as np

class LiDARDiagnostic:
    def __init__(self):
        self.readings_history = deque(maxlen=100)  # Keep last 100 readings
        self.angle_readings = defaultdict(list)    # Track readings by angle
        self.suspicious_readings = []              # Track consistent readings
        self.total_scans = 0
        self.problem_readings = 0
        
        # Analysis parameters
        self.target_distance = 0.108  # The problematic distance
        self.distance_tolerance = 0.005  # ±5mm tolerance
        self.consistency_threshold = 10  # How many times to see same reading
        
        # Initialize MQTT
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        
        # Connect to MQTT
        try:
            self.mqtt_client.connect("spin5.petra.ac.id", 1883, 60)
            self.mqtt_client.loop_start()
            print("[DIAGNOSTIC] Connected to MQTT broker")
        except Exception as e:
            print(f"[ERROR] Failed to connect to MQTT: {e}")
            return
        
        # Start analysis thread
        self.analysis_thread = threading.Thread(target=self.continuous_analysis)
        self.analysis_thread.daemon = True
        self.analysis_thread.start()
        
    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("[DIAGNOSTIC] MQTT connected successfully")
            client.subscribe("robot/lidar/real_world_data")
            print("[DIAGNOSTIC] Subscribed to LiDAR data topic")
        else:
            print(f"[ERROR] MQTT connection failed: {rc}")
            
    def on_mqtt_message(self, client, userdata, msg):
        try:
            if msg.topic == "robot/lidar/real_world_data":
                data = json.loads(msg.payload.decode())
                self.analyze_lidar_data(data)
        except Exception as e:
            print(f"[ERROR] Failed to process MQTT message: {e}")
            
    def analyze_lidar_data(self, data):
        """Analyze LiDAR data for the 0.108m issue"""
        try:
            ranges = data.get('ranges', [])
            if not ranges or len(ranges) < 360:
                return
                
            self.total_scans += 1
            timestamp = time.time()
            
            # Find all readings close to 0.108m
            problem_readings = []
            for angle, distance in enumerate(ranges):
                if abs(distance - self.target_distance) < self.distance_tolerance:
                    problem_readings.append((angle, distance))
                    self.problem_readings += 1
                    
            # Store in history
            scan_data = {
                'timestamp': timestamp,
                'total_ranges': len(ranges),
                'problem_readings': problem_readings,
                'problem_count': len(problem_readings)
            }
            self.readings_history.append(scan_data)
            
            # Track by angle
            for angle, distance in problem_readings:
                self.angle_readings[angle].append((timestamp, distance))
                
            # Real-time analysis
            if problem_readings:
                self.analyze_problem_readings(problem_readings, timestamp)
                
        except Exception as e:
            print(f"[ERROR] Analysis failed: {e}")
            
    def analyze_problem_readings(self, problem_readings, timestamp):
        """Analyze problematic readings in real-time"""
        print(f"\n[PROBLEM DETECTED] Scan #{self.total_scans} at {time.strftime('%H:%M:%S')}")
        print(f"[PROBLEM] Found {len(problem_readings)} readings near 0.108m:")
        
        # Group by sector
        front_sector = []    # -30° to +30° (330°-360° and 0°-30°)
        left_sector = []     # 60° to 120°
        right_sector = []    # 240° to 300°
        back_sector = []     # 135° to 225°
        other_sector = []    # Everything else
        
        for angle, distance in problem_readings:
            if (330 <= angle <= 360) or (0 <= angle <= 30):
                front_sector.append((angle, distance))
            elif 60 <= angle <= 120:
                left_sector.append((angle, distance))
            elif 240 <= angle <= 300:
                right_sector.append((angle, distance))
            elif 135 <= angle <= 225:
                back_sector.append((angle, distance))
            else:
                other_sector.append((angle, distance))
                
        # Report by sector
        if front_sector:
            print(f"  🔴 FRONT SECTOR: {len(front_sector)} readings")
            for angle, dist in front_sector[:3]:  # Show first 3
                print(f"    Angle {angle}°: {dist:.3f}m")
                
        if left_sector:
            print(f"  🟡 LEFT SECTOR: {len(left_sector)} readings")
            for angle, dist in left_sector[:3]:
                print(f"    Angle {angle}°: {dist:.3f}m")
                
        if right_sector:
            print(f"  🟡 RIGHT SECTOR: {len(right_sector)} readings")
            for angle, dist in right_sector[:3]:
                print(f"    Angle {angle}°: {dist:.3f}m")
                
        if back_sector:
            print(f"  🟢 BACK SECTOR: {len(back_sector)} readings (probably robot body)")
            for angle, dist in back_sector[:3]:
                print(f"    Angle {angle}°: {dist:.3f}m")
                
        if other_sector:
            print(f"  🔵 OTHER SECTORS: {len(other_sector)} readings")
            for angle, dist in other_sector[:3]:
                print(f"    Angle {angle}°: {dist:.3f}m")
                
        # Check if this is likely robot body detection
        if len(back_sector) > len(front_sector) + len(left_sector) + len(right_sector):
            print(f"  💡 ANALYSIS: Most readings are in back sector - likely ROBOT BODY detection")
        elif len(front_sector) > 5:
            print(f"  ⚠️ ANALYSIS: Many front readings - possible HARDWARE ISSUE or MOUNTING PROBLEM")
        elif len(problem_readings) > 20:
            print(f"  ⚠️ ANALYSIS: Too many readings - possible REFLECTION or INTERFERENCE")
            
    def continuous_analysis(self):
        """Continuous analysis and reporting"""
        while True:
            time.sleep(30)  # Report every 30 seconds
            self.generate_report()
            
    def generate_report(self):
        """Generate comprehensive analysis report"""
        if self.total_scans == 0:
            print("[REPORT] No data received yet...")
            return
            
        print(f"\n{'='*60}")
        print(f"LiDAR A1M8 DIAGNOSTIC REPORT - {time.strftime('%H:%M:%S')}")
        print(f"{'='*60}")
        
        # Basic stats
        problem_rate = (self.problem_readings / (self.total_scans * 360)) * 100
        print(f"Total scans analyzed: {self.total_scans}")
        print(f"Total problem readings: {self.problem_readings}")
        print(f"Problem rate: {problem_rate:.2f}% of all readings")
        
        # Most problematic angles
        print(f"\nMOST PROBLEMATIC ANGLES:")
        angle_counts = {angle: len(readings) for angle, readings in self.angle_readings.items()}
        sorted_angles = sorted(angle_counts.items(), key=lambda x: x[1], reverse=True)
        
        for angle, count in sorted_angles[:10]:  # Top 10
            print(f"  Angle {angle}°: {count} readings ({count/self.total_scans*100:.1f}% of scans)")
            
        # Sector analysis
        print(f"\nSECTOR ANALYSIS:")
        front_angles = [a for a in angle_counts.keys() if (330 <= a <= 360) or (0 <= a <= 30)]
        left_angles = [a for a in angle_counts.keys() if 60 <= a <= 120]
        right_angles = [a for a in angle_counts.keys() if 240 <= a <= 300]
        back_angles = [a for a in angle_counts.keys() if 135 <= a <= 225]
        
        front_count = sum(angle_counts[a] for a in front_angles)
        left_count = sum(angle_counts[a] for a in left_angles)
        right_count = sum(angle_counts[a] for a in right_angles)
        back_count = sum(angle_counts[a] for a in back_angles)
        
        print(f"  Front sector (330°-30°): {front_count} readings")
        print(f"  Left sector (60°-120°): {left_count} readings")
        print(f"  Right sector (240°-300°): {right_count} readings")
        print(f"  Back sector (135°-225°): {back_count} readings")
        
        # Diagnosis
        print(f"\nDIAGNOSIS:")
        if back_count > front_count + left_count + right_count:
            print(f"  🔍 PRIMARY ISSUE: ROBOT BODY DETECTION")
            print(f"    - Most readings are from back sector")
            print(f"    - LiDAR is detecting parts of the robot itself")
            print(f"    - SOLUTION: Adjust LiDAR mounting position or add exclusion zones")
        elif front_count > back_count * 2:
            print(f"  🔍 PRIMARY ISSUE: FRONT OBSTRUCTION")
            print(f"    - Many readings from front sector")
            print(f"    - Possible hardware mounting issue")
            print(f"    - SOLUTION: Check LiDAR mounting and clearance")
        elif len(sorted_angles) > 50:
            print(f"  🔍 PRIMARY ISSUE: WIDESPREAD INTERFERENCE")
            print(f"    - Readings spread across many angles")
            print(f"    - Possible reflections or interference")
            print(f"    - SOLUTION: Check environment for reflective surfaces")
        else:
            print(f"  🔍 PRIMARY ISSUE: UNKNOWN PATTERN")
            print(f"    - Need more data to determine root cause")
            
        # Recommendations
        print(f"\nRECOMMENDATIONS:")
        print(f"  1. Check LiDAR mounting - ensure no robot parts in scan area")
        print(f"  2. Verify LiDAR height and angle")
        print(f"  3. Check for reflective surfaces in environment")
        print(f"  4. Consider adding rear sector exclusion (135°-225°)")
        print(f"  5. Increase minimum distance filter from 0.15m to 0.25m")
        
        print(f"{'='*60}\n")
        
    def save_analysis_data(self):
        """Save analysis data to file"""
        try:
            filename = f"lidar_diagnostic_{int(time.time())}.json"
            data = {
                'timestamp': time.time(),
                'total_scans': self.total_scans,
                'problem_readings': self.problem_readings,
                'angle_readings': dict(self.angle_readings),
                'recent_history': list(self.readings_history)
            }
            
            with open(filename, 'w') as f:
                json.dump(data, f, indent=2)
                
            print(f"[DIAGNOSTIC] Analysis data saved to {filename}")
            
        except Exception as e:
            print(f"[ERROR] Failed to save analysis data: {e}")

def main():
    print("=" * 60)
    print("LiDAR A1M8 DIAGNOSTIC TOOL")
    print("Analyzing 0.108m obstacle detection issue")
    print("=" * 60)
    
    try:
        diagnostic = LiDARDiagnostic()
        
        print("\n[DIAGNOSTIC] Starting analysis...")
        print("[DIAGNOSTIC] This tool will:")
        print("  - Monitor LiDAR data in real-time")
        print("  - Identify 0.108m readings")
        print("  - Analyze patterns by angle/sector")
        print("  - Provide diagnosis and recommendations")
        print("  - Generate reports every 30 seconds")
        
        print("\nPress Ctrl+C to stop and save analysis data...")
        
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n[DIAGNOSTIC] Stopping diagnostic tool...")
        diagnostic.save_analysis_data()
        print("[DIAGNOSTIC] Analysis complete!")
        
    except Exception as e:
        print(f"[ERROR] Diagnostic failed: {e}")

if __name__ == "__main__":
    main() 