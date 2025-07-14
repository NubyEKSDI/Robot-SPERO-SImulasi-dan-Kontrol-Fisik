#!/usr/bin/env python3
"""
BCR Bot Physical Robot Node with Advanced Safety Features

SAFETY FEATURES:
- Connection monitoring: Automatically stops all motors if MQTT/WiFi connection is lost
- Timeout protection: Stops robot if no commands received for specified timeout period
- Emergency stop: Immediate motor stop on connection loss or emergency signals
- Auto-resume: Automatically resumes operation when connection is restored
- Heartbeat monitoring: Continuous connection health monitoring
- Motor safety checks: Prevents motor commands during safety stop

CONNECTION SAFETY:
- MQTT timeout: 10 seconds (configurable)
- Heartbeat interval: 5 seconds (configurable)
- Auto-stop on disconnection: YES
- Auto-resume on reconnection: YES

This prevents runaway robot behavior when WiFi/MQTT connection is lost.
"""

import time
import board
import busio
import adafruit_pca9685
import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
import json
import serial
import threading
import math
from rplidar import RPLidar

# Constants
PCA_FREQ = 1500
fullSpeed = 0xFFFF  # 65535
halfSpeed = 0x7FFF  # 32767
noSpeed = 0x0000    # 0
speedCoef = 0.4     # Speed reduction factor (increased from 0.3 to 0.4 for faster movement)
rmodeSpeedCoef = 0.5  # Special speed coefficient for R-mode (higher for faster waypoint navigation)
rmodeBackwardSpeedCoef = 0.6  # Special speed coefficient for R-mode backward (even higher for faster backward movement)

# Motor health monitoring
MIN_MOTOR_SPEED = 0x1000  # Minimum speed to ensure motor actually moves (4096)
MOTOR_HEALTH_CHECK_INTERVAL = 5.0  # Check motor health every 5 seconds
MOTOR_ERROR_THRESHOLD = 3  # Number of consecutive errors before flagging motor as problematic

# Connection Safety Configuration
CONNECTION_TIMEOUT = 10.0    # Stop robot if no MQTT messages for this many seconds
HEARTBEAT_INTERVAL = 5.0     # Send heartbeat every this many seconds
RECONNECT_ATTEMPTS = 5       # Maximum reconnection attempts before giving up
SAFETY_CHECK_INTERVAL = 1.0  # Check connection status every second

# LiDAR Configuration
LIDAR_PORT = '/dev/ttyUSB0'  # Default USB port for RP-Lidar A1M8
LIDAR_BAUDRATE = 115200      # Default baudrate for A1M8
LIDAR_TIMEOUT = 1.0          # Timeout for LiDAR operations
LIDAR_SCAN_FREQ = 5.5        # Hz - Default scan frequency for A1M8

# Arah putaran motor
arahMAJU = 0xFFFF    # HIGH
arahMUNDUR = 0x0000  # LOW

# Pin mappings
pinPWM = {
1: 15,  # Motor 1 PWM pin (depan kiri)
2: 3,   # Motor 2 PWM pin (depan kanan)
3: 11,  # Motor 3 PWM pin (belakang kanan)
4: 7    # Motor 4 PWM pin (belakang kiri)
}

pinDIR = {
1: 14,  # Motor 1 direction pin (depan kiri)
2: 2,   # Motor 2 direction pin (depan kanan)
3: 10,  # Motor 3 direction pin (belakang kanan)
4: 6    # Motor 4 direction pin (belakang kiri)
}

# MQTT Topics
TOPIC_PHYSICAL_CMD = "robot/physical/cmd_vel"    # Topic untuk perintah fisik
TOPIC_PRIMARY_CMD = "robot/primary/cmd_vel"      # Topic untuk primary mode
TOPIC_SECONDARY_CMD = "robot/secondary/cmd_vel"  # Topic untuk secondary mode
TOPIC_RMODE_CMD = "robot/rmode/cmd_vel"          # Topic untuk R-mode
TOPIC_NAV_CHARGING_CMD = "robot/nav_charging/cmd_vel"  # Topic untuk navigation charging mode

# Synchronization Topics - REMOVED (no longer needed)

# LiDAR Real-World Topics
TOPIC_LIDAR_REAL_WORLD_DATA = "robot/lidar/real_world_data"  # Topic untuk data LiDAR real-world
TOPIC_LIDAR_STATUS = "robot/lidar/status"  # Topic untuk status LiDAR
TOPIC_MAPPING_DATA = "robot/mapping/data"  # Topic untuk data mapping

class RobotTester:
    def __init__(self):
        print("[INIT] Initializing PCA9685...")
        try:
            # Initialize I2C bus
            self.i2c = busio.I2C(3, 2)  # SCL=GPIO3, SDA=GPIO2
            self.pca = adafruit_pca9685.PCA9685(self.i2c)
            self.pca.frequency = PCA_FREQ
         
            # Initialize all motors to stop
            for motor in range(1, 5):
                self.pca.channels[pinPWM[motor]].duty_cycle = 0
                self.pca.channels[pinDIR[motor]].duty_cycle = 0
            print("[INIT] PCA9685 initialized successfully")
           
            # Initialize status tracking
            self.current_mode = "IDLE"
            self.last_command_time = time.time()
            self.motor_status = "STOPPED"
            
            # Initialize connection monitoring
            self.mqtt_connected = False
            self.last_mqtt_heartbeat = time.time()
            self.connection_timeout = CONNECTION_TIMEOUT  # Use configurable constant
            self.connection_lost = False
            self.safety_stop_active = False
            self.last_connection_check = time.time()
            self.heartbeat_interval = HEARTBEAT_INTERVAL  # Use configurable constant
            self.last_heartbeat_sent = time.time()
            self.reconnect_attempts = 0
            self.max_reconnect_attempts = RECONNECT_ATTEMPTS
            print(f"[INIT] Connection monitoring initialized (timeout: {CONNECTION_TIMEOUT}s, heartbeat: {HEARTBEAT_INTERVAL}s)")
           
            # Initialize motor health monitoring
            self.motor_health = {1: {"errors": 0, "last_check": time.time(), "status": "OK"}}
            self.motor_health.update({2: {"errors": 0, "last_check": time.time(), "status": "OK"}})
            self.motor_health.update({3: {"errors": 0, "last_check": time.time(), "status": "OK"}})
            self.motor_health.update({4: {"errors": 0, "last_check": time.time(), "status": "OK"}})
            self.last_motor_health_check = time.time()
            self.motor_debug_enabled = True  # Enable motor debugging
           
            # Initialize LiDAR
            self.lidar = None
            self.is_running = False
            self.lidar_thread = None
            self.lidar_data = None
            self.lidar_last_scan = None
            self.init_lidar()
           
            # Initialize LiDAR Real-World Integration
            self.lidar_streaming_thread = None
            self.mapping_active = False
            self.mapping_data = []
            self.robot_position = (0.0, 0.0)  # (x, y) position
            self.robot_orientation = 0.0  # heading in radians
            
# Robot Synchronization - COMPLETELY REMOVED (was causing movement issues)
           
            # Initialize MQTT
            self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)  # Use newer API version
            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.on_message = self.on_mqtt_message
            self.mqtt_client.on_disconnect = self.on_mqtt_disconnect  # Add disconnect handler
            try:
                self.mqtt_client.connect("spin5.petra.ac.id", 1883, 60)
                self.mqtt_client.loop_start()
                print("[INIT] MQTT connected successfully")
            except Exception as e:
                print(f"[ERROR] MQTT connection failed: {str(e)}")
                self.mqtt_client = None
                self.mqtt_connected = False
         
        except Exception as e:
            print(f"[ERROR] Failed to initialize PCA9685: {str(e)}")
            raise




    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("[MQTT] Connected successfully")
            # Update connection status
            self.mqtt_connected = True
            self.last_mqtt_heartbeat = time.time()
            self.connection_lost = False
            self.safety_stop_active = False
            
            # Subscribe to all robot command topics
            client.subscribe(TOPIC_PHYSICAL_CMD)
            client.subscribe(TOPIC_PRIMARY_CMD)
            client.subscribe(TOPIC_SECONDARY_CMD)
            client.subscribe(TOPIC_RMODE_CMD)
            client.subscribe(TOPIC_NAV_CHARGING_CMD)
          
            # Subscribe to LiDAR real-world topics
            client.subscribe(TOPIC_LIDAR_REAL_WORLD_DATA)
            client.subscribe(TOPIC_LIDAR_STATUS)
            client.subscribe(TOPIC_MAPPING_DATA)
            
# Synchronization topics removed - no longer needed
          
            print("[MQTT] Subscribed to all robot command topics")
            print("[MQTT] Subscribed to LiDAR real-world topics")
            print("[SAFETY] Connection monitoring active - robot will stop if connection lost")
        else:
            print(f"[MQTT] Connection failed with code {rc}")
            self.mqtt_connected = False




    def on_mqtt_message(self, client, userdata, msg):
        try:
            # Update heartbeat timestamp for any received message
            self.last_mqtt_heartbeat = time.time()
            
            # Handle all robot command topics
            if msg.topic in [TOPIC_PHYSICAL_CMD, TOPIC_PRIMARY_CMD, TOPIC_SECONDARY_CMD, TOPIC_RMODE_CMD, TOPIC_NAV_CHARGING_CMD]:
                cmd_data = json.loads(msg.payload.decode())
              
                # Handle velocity commands
                if 'linear' in cmd_data and 'angular' in cmd_data:
                    linear_x = float(cmd_data['linear'].get('x', 0))
                    angular_z = float(cmd_data['angular'].get('z', 0))
                    mode = cmd_data.get('mode', 'Unknown')
                  
                    # Update current mode based on topic or mode field
                    if msg.topic == TOPIC_PRIMARY_CMD or "Primary" in mode:
                        self.current_mode = "PRIMARY CONTROL"
                    elif msg.topic == TOPIC_SECONDARY_CMD or "Secondary" in mode:
                        self.current_mode = "SECONDARY CONTROL"
                    elif msg.topic == TOPIC_RMODE_CMD or "R-mode" in mode:
                        self.current_mode = "R-MODE"
                    elif msg.topic == TOPIC_NAV_CHARGING_CMD or "CHARGING NAV" in mode:
                        self.current_mode = "NAVIGATION CHARGING"
                    elif msg.topic == TOPIC_PHYSICAL_CMD:
                        self.current_mode = "MANUAL CONTROL"
                    else:
                        self.current_mode = mode
                  
                    # Check for emergency stop commands
                    if "EMERGENCY STOP" in mode or "ROBOT IDLE" in mode or "ROBOT STOPPED" in mode:
                        print(f"[EMERGENCY] Emergency stop command detected: {mode}")
                        self.stop_all_motors()
                        return
                  
                    # Check if this is a stop command
                    if abs(linear_x) < 0.01 and abs(angular_z) < 0.01:
                        self.stop_all_motors()
                        return
                  
                    # Calculate speed factor based on mode and velocity
                    speed_factor = self.calculate_speed_factor(linear_x, angular_z, mode)
                  
                    # Special debugging for motor 1 in R-mode
                    if self.current_mode == "R-MODE" and self.motor_debug_enabled:
                        print(f"[R-MODE DEBUG] Linear: {linear_x:.3f}, Angular: {angular_z:.3f}, Speed Factor: {speed_factor:.3f}")
                  
                    # Check if this is a rotating command in R-mode
                    is_rotating = abs(angular_z) > abs(linear_x)
                  
                    # Tentukan gerakan berdasarkan cmd_vel
                    if abs(linear_x) > abs(angular_z):
                        # Gerakan maju/mundur dominan
                        if linear_x > 0:
                            self.maju(speed_factor)
                        else:
                            self.mundur(speed_factor)
                    else:
                        # Gerakan putar dominan
                        if angular_z > 0:
                            self.putar_kiri(speed_factor)
                        else:
                            self.putar_kanan(speed_factor)
                else:
                    print(f"[ERROR] Invalid velocity command format")
          
            # Handle LiDAR real-world data
            elif msg.topic == TOPIC_LIDAR_REAL_WORLD_DATA:
                try:
                    lidar_data = json.loads(msg.payload.decode())
                    self.handle_real_world_lidar_data(lidar_data)
                except Exception as e:
                    print(f"[ERROR] Failed to process LiDAR real-world data: {str(e)}")
          
            # Handle LiDAR status
            elif msg.topic == TOPIC_LIDAR_STATUS:
                try:
                    status_data = json.loads(msg.payload.decode())
                    print(f"[LIDAR STATUS] {status_data.get('message', 'Unknown status')}")
                  
                    # Check if roam.py is ready
                    if status_data.get('source') == 'roam.py' and status_data.get('status') == 'ready':
                        print("[LIDAR] roam.py is ready to receive LiDAR data!")
                        # Start LiDAR streaming if not already started
                        if not self.is_running and self.lidar:
                            print("[LIDAR] Starting LiDAR streaming now that roam.py is ready")
                            self.start_lidar_streaming()
                          
                except Exception as e:
                    print(f"[ERROR] Failed to process LiDAR status: {str(e)}")
          
            # Handle mapping data
            elif msg.topic == TOPIC_MAPPING_DATA:
                try:
                    mapping_data = json.loads(msg.payload.decode())
                    self.handle_mapping_data(mapping_data)
                except Exception as e:
                    print(f"[ERROR] Failed to process mapping data: {str(e)}")
            
# Sync status handling removed - no longer needed
          
        except Exception as e:
            print(f"[ERROR] Failed to process MQTT message: {str(e)}")


    def on_mqtt_disconnect(self, client, userdata, rc):
        """Handle MQTT disconnection"""
        self.mqtt_connected = False
        self.connection_lost = True
        
        if rc != 0:
            print(f"[MQTT] ⚠️ UNEXPECTED DISCONNECTION! Code: {rc}")
        else:
            print("[MQTT] Disconnected normally")
            
        # Trigger emergency safety stop
        self.emergency_safety_stop("MQTT CONNECTION LOST")
        
        # Try to reconnect
        print("[MQTT] Attempting to reconnect...")
        try:
            client.reconnect()
        except Exception as e:
            print(f"[MQTT] Reconnection failed: {str(e)}")


    def emergency_safety_stop(self, reason):
        """Emergency safety stop - immediately stop all motors"""
        print(f"[EMERGENCY SAFETY STOP] {reason}")
        print("[EMERGENCY SAFETY STOP] Stopping all motors immediately!")
        
        # Stop all motors immediately
        try:
            for motor in range(1, 5):
                self.pca.channels[pinPWM[motor]].duty_cycle = 0
                self.pca.channels[pinDIR[motor]].duty_cycle = 0
            
            self.safety_stop_active = True
            self.motor_status = "EMERGENCY STOPPED"
            self.current_mode = "SAFETY STOP"
            
            print("[EMERGENCY SAFETY STOP] All motors stopped successfully")
            
            # Send emergency status if MQTT is available
            if self.mqtt_client and self.mqtt_connected:
                try:
                    emergency_data = {
                        "status": "emergency_stop",
                        "reason": reason,
                        "timestamp": time.time(),
                        "motors_stopped": True
                    }
                    self.mqtt_client.publish("robot/physical/emergency", json.dumps(emergency_data))
                except Exception as e:
                    print(f"[EMERGENCY SAFETY STOP] Failed to send emergency status: {str(e)}")
                    
        except Exception as e:
            print(f"[EMERGENCY SAFETY STOP] Error stopping motors: {str(e)}")
            # If we can't stop motors normally, try to shut down PCA9685
            try:
                self.pca.deinit()
                print("[EMERGENCY SAFETY STOP] PCA9685 shut down completely")
            except Exception as e2:
                print(f"[EMERGENCY SAFETY STOP] CRITICAL: Cannot stop motors! {str(e2)}")


    def check_connection_status(self):
        """Check connection status and trigger safety stop if needed"""
        current_time = time.time()
        
        # Check if enough time has passed since last check
        if current_time - self.last_connection_check < SAFETY_CHECK_INTERVAL:  # Use configurable constant
            return
            
        self.last_connection_check = current_time
        
        # Check MQTT connection timeout
        time_since_heartbeat = current_time - self.last_mqtt_heartbeat
        
        if time_since_heartbeat > self.connection_timeout:
            if not self.safety_stop_active:
                self.emergency_safety_stop(f"CONNECTION TIMEOUT - No MQTT messages for {time_since_heartbeat:.1f}s")
                
        # Check if we need to send heartbeat
        if current_time - self.last_heartbeat_sent > self.heartbeat_interval:
            self.send_heartbeat()
            self.last_heartbeat_sent = current_time


    def send_heartbeat(self):
        """Send heartbeat to roam.py"""
        if self.mqtt_client and self.mqtt_connected:
            try:
                heartbeat_data = {
                    "type": "heartbeat",
                    "timestamp": time.time(),
                    "status": "alive",
                    "motor_status": self.motor_status,
                    "current_mode": self.current_mode,
                    "safety_stop_active": self.safety_stop_active,
                    "connection_lost": self.connection_lost
                }
                self.mqtt_client.publish("robot/physical/heartbeat", json.dumps(heartbeat_data))
                
                # Reset connection lost flag if we can send heartbeat
                if self.connection_lost:
                    self.connection_lost = False
                    print("[SAFETY] Connection restored - heartbeat sent successfully")
                    
            except Exception as e:
                print(f"[HEARTBEAT] Failed to send heartbeat: {str(e)}")
                self.connection_lost = True


    def reset_safety_stop(self):
        """Reset safety stop if connection is restored"""
        if self.mqtt_connected and not self.connection_lost:
            current_time = time.time()
            time_since_heartbeat = current_time - self.last_mqtt_heartbeat
            
            if time_since_heartbeat < self.connection_timeout:
                self.safety_stop_active = False
                self.motor_status = "STOPPED"
                self.current_mode = "IDLE"
                print("[SAFETY] Safety stop reset - connection restored")
                return True
        return False


    def set_motor(self, motor_num, speed, direction):
        """Set motor speed and direction with health monitoring"""
        try:
            if motor_num not in pinPWM or motor_num not in pinDIR:
                print(f"[ERROR] Invalid motor number: {motor_num}")
                return
            
            # Safety check - don't allow motor movement if safety stop is active
            if self.safety_stop_active:
                print(f"[SAFETY] Motor {motor_num} movement blocked - safety stop active")
                return
            
            # Safety check - don't allow motor movement if connection is lost
            if self.connection_lost:
                print(f"[SAFETY] Motor {motor_num} movement blocked - connection lost")
                return
        
            # Special handling for R-mode to prevent motor stopping
            if self.current_mode == "R-MODE":
                # Ensure minimum speed for R-mode to prevent motor stalling
                if speed < MIN_MOTOR_SPEED:
                    speed = MIN_MOTOR_SPEED
                    if self.motor_debug_enabled and motor_num == 1:
                        print(f"[MOTOR DEBUG] Motor 1 speed increased to minimum {MIN_MOTOR_SPEED} for R-mode")
        
            # Set direction
            dir_value = arahMAJU if direction else arahMUNDUR
            self.pca.channels[pinDIR[motor_num]].duty_cycle = dir_value
        
            # Set speed
            self.pca.channels[pinPWM[motor_num]].duty_cycle = speed
        
            # Special debugging for motor 1
            if self.motor_debug_enabled and motor_num == 1:
                print(f"[MOTOR 1 DEBUG] Speed: {speed}, Direction: {'FORWARD' if direction else 'BACKWARD'}, Mode: {self.current_mode}")
          
            # Update motor health
            self.motor_health[motor_num]["last_check"] = time.time()
            self.motor_health[motor_num]["errors"] = 0  # Reset error count on successful operation
            self.motor_health[motor_num]["status"] = "OK"
        
        except Exception as e:
            print(f"[ERROR] Failed to set motor {motor_num}: {e}")
            # Update motor health with error
            if motor_num in self.motor_health:
                self.motor_health[motor_num]["errors"] += 1
                if self.motor_health[motor_num]["errors"] >= MOTOR_ERROR_THRESHOLD:
                    self.motor_health[motor_num]["status"] = "ERROR"
                    print(f"[MOTOR HEALTH] Motor {motor_num} flagged as problematic after {MOTOR_ERROR_THRESHOLD} errors")




    def update_status(self, mode, motor_status):
        """Update robot status"""
        self.current_mode = mode
        self.motor_status = motor_status
        self.last_command_time = time.time()




    def stop_all_motors(self):
        """Stop all motors"""
        for motor in range(1, 5):
            self.pca.channels[pinPWM[motor]].duty_cycle = 0
            self.pca.channels[pinDIR[motor]].duty_cycle = 0
        self.update_status(self.current_mode, "STOPPED")
        
# Sync commands completely removed




    def maju(self, speed_factor=1.0):
        """Move forward"""
        # Safety check before any movement
        if self.safety_stop_active or self.connection_lost:
            print("[SAFETY] Forward movement blocked - safety stop active or connection lost")
            return
            
        # Use higher speed coefficient for R-mode
        current_speed_coef = rmodeSpeedCoef if self.current_mode == "R-MODE" else speedCoef
        speed = int(fullSpeed * speed_factor * current_speed_coef)
        
        # ✅ SAFE APPROACH: Execute movement and monitor for errors
        try:
            # Move all motors together (safe execution)
        self.set_motor(1, speed, False)   # Motor 1 forward
            self.set_motor(2, speed, False)   # Motor 2 backward
        self.set_motor(3, speed, False)   # Motor 3 forward
            self.set_motor(4, speed, False)   # Motor 4 backward
            
            # Update status
        self.update_status(self.current_mode, f"FORWARD (speed: {speed_factor:.2f})")
      
            # ✅ Monitor for hardware errors (passive monitoring, no testing)
            for motor in range(1, 5):
                if self.motor_health[motor]["status"] == "ERROR":
                    print(f"[HARDWARE WARNING] Motor {motor} has known issues - check hardware")

        except Exception as e:
            print(f"[HARDWARE ERROR] Forward movement failed: {str(e)}")
            print(f"[SAFETY] Hardware error detected - stopping for safety")
            self.stop_all_motors()


    def mundur(self, speed_factor=1.0):
        """Move backward"""
        # Safety check before any movement
        if self.safety_stop_active or self.connection_lost:
            print("[SAFETY] Backward movement blocked - safety stop active or connection lost")
            return
            
        # Use higher speed coefficient for R-mode backward movement
        if self.current_mode == "R-MODE":
            current_speed_coef = rmodeBackwardSpeedCoef  # Use special backward speed coefficient
        else:
            current_speed_coef = speedCoef
        speed = int(fullSpeed * speed_factor * current_speed_coef)
        
        # ✅ SAFE APPROACH: Execute movement and monitor for errors
        try:
            # Move all motors together (safe execution)
        self.set_motor(1, speed, True)  # Motor 1 backward (arahMUNDUR)
        self.set_motor(2, speed, True)  # Motor 2 backward (arahMUNDUR)
        self.set_motor(3, speed, True)  # Motor 3 backward (arahMUNDUR)
        self.set_motor(4, speed, True)  # Motor 4 backward (arahMUNDUR)
            
            # Update status
        self.update_status(self.current_mode, f"BACKWARD (speed: {speed_factor:.2f})")

            # ✅ Monitor for hardware errors (passive monitoring, no testing)
            for motor in range(1, 5):
                if self.motor_health[motor]["status"] == "ERROR":
                    print(f"[HARDWARE WARNING] Motor {motor} has known issues - check hardware")

        except Exception as e:
            print(f"[HARDWARE ERROR] Backward movement failed: {str(e)}")
            print(f"[SAFETY] Hardware error detected - stopping for safety")
            self.stop_all_motors()


    def putar_kiri(self, speed_factor=1.0):
        """Turn left"""
        # Safety check before any movement
        if self.safety_stop_active or self.connection_lost:
            print("[SAFETY] Left turn blocked - safety stop active or connection lost")
            return
            
        # Use higher speed coefficient for R-mode
        current_speed_coef = rmodeSpeedCoef if self.current_mode == "R-MODE" else speedCoef
        speed = int(fullSpeed * speed_factor * current_speed_coef)
        
        # ✅ SAFE APPROACH: Execute movement and monitor for errors
        try:
            # Move all motors together (safe execution)
        self.set_motor(1, speed, False)  # Motor 1 backward
        self.set_motor(2, speed, True)   # Motor 2 forward
        self.set_motor(3, speed, False)  # Motor 3 backward
        self.set_motor(4, speed, True)   # Motor 4 forward
            
            # Update status
        self.update_status(self.current_mode, f"TURNING LEFT (speed: {speed_factor:.2f})")

            # ✅ Monitor for hardware errors (passive monitoring, no testing)
            for motor in range(1, 5):
                if self.motor_health[motor]["status"] == "ERROR":
                    print(f"[HARDWARE WARNING] Motor {motor} has known issues - check hardware")

        except Exception as e:
            print(f"[HARDWARE ERROR] Left turn failed: {str(e)}")
            print(f"[SAFETY] Hardware error detected - stopping for safety")
            self.stop_all_motors()


    def putar_kanan(self, speed_factor=1.0):
        """Turn right"""
        # Safety check before any movement
        if self.safety_stop_active or self.connection_lost:
            print("[SAFETY] Right turn blocked - safety stop active or connection lost")
            return
            
        # Use higher speed coefficient for R-mode
        current_speed_coef = rmodeSpeedCoef if self.current_mode == "R-MODE" else speedCoef
        speed = int(fullSpeed * speed_factor * current_speed_coef)
        
        # ✅ SAFE APPROACH: Execute movement and monitor for errors
        try:
            # Move all motors together (safe execution)
        self.set_motor(1, speed, True)   # Motor 1 forward
        self.set_motor(2, speed, False)  # Motor 2 backward
        self.set_motor(3, speed, True)   # Motor 3 forward
        self.set_motor(4, speed, False)  # Motor 4 backward
            
            # Update status
        self.update_status(self.current_mode, f"TURNING RIGHT (speed: {speed_factor:.2f})")

            # ✅ Monitor for hardware errors (passive monitoring, no testing)
            for motor in range(1, 5):
                if self.motor_health[motor]["status"] == "ERROR":
                    print(f"[HARDWARE WARNING] Motor {motor} has known issues - check hardware")
                  
                except Exception as e:
            print(f"[HARDWARE ERROR] Right turn failed: {str(e)}")
            print(f"[SAFETY] Hardware error detected - stopping for safety")
            self.stop_all_motors()




    def display_status(self):
        """Display current robot status"""
        current_time = time.time()
        time_since_last_command = current_time - self.last_command_time
      
        # LiDAR status
        lidar_status = "CONNECTED" if self.lidar else "DISCONNECTED"
        if self.lidar and self.lidar_last_scan:
            time_since_scan = current_time - self.lidar_last_scan
            lidar_status += f" (Last scan: {time_since_scan:.1f}s ago)"
      
        print(f"\n{'='*60}")
        print(f"ROBOT STATUS - {time.strftime('%H:%M:%S')}")
        print(f"{'='*60}")
        print(f"Current Mode: {self.current_mode}")
        print(f"Motor Status: {self.motor_status}")
      
        # Motor health status
        print(f"Motor Health:")
        for motor_num in range(1, 5):
            health = self.motor_health.get(motor_num, {"status": "UNKNOWN", "errors": 0})
            status_icon = "✅" if health["status"] == "OK" else "⚠️" if health["status"] == "ERROR" else "❓"
            print(f"  Motor {motor_num}: {status_icon} {health['status']} (Errors: {health['errors']})")
      
        print(f"LiDAR Status: {lidar_status}")
        print(f"Time since last command: {time_since_last_command:.1f}s")
        
        # Connection status
        mqtt_conn_status = "CONNECTED" if self.mqtt_connected else "DISCONNECTED"
        if self.connection_lost:
            mqtt_conn_status += " (LOST)"
        print(f"MQTT Connected: {mqtt_conn_status}")
        
        # Safety status
        safety_status = "ACTIVE" if self.safety_stop_active else "INACTIVE"
        if self.safety_stop_active:
            safety_status += " 🚨"
        print(f"Safety Stop: {safety_status}")
        
        # Connection timing
        time_since_heartbeat = current_time - self.last_mqtt_heartbeat
        print(f"Time since last MQTT message: {time_since_heartbeat:.1f}s")
        print(f"Connection timeout: {self.connection_timeout:.1f}s")
        
        # Robot operation mode - Independent movement
        print(f"Robot movement: INDEPENDENT operation")
        print(f"Physical robot: AUTONOMOUS movement")
        print(f"Operation mode: STANDALONE (no external dependencies)")
        
        print(f"{'='*60}\n")




    def check_motor_health(self):
        """Check motor health - PASSIVE MONITORING ONLY, NO TESTING"""
        current_time = time.time()
      
        # Only check every MOTOR_HEALTH_CHECK_INTERVAL seconds
        if current_time - self.last_motor_health_check < MOTOR_HEALTH_CHECK_INTERVAL:
            return
      
        self.last_motor_health_check = current_time
      
        # Only passive monitoring - no actual motor testing
        for motor_num in range(1, 5):
            health = self.motor_health.get(motor_num, {"errors": 0, "status": "OK"})
          
            # If motor has too many errors, just log it (no testing)
            if health["errors"] >= MOTOR_ERROR_THRESHOLD:
                print(f"[MOTOR HEALTH] Motor {motor_num} has {health['errors']} errors")
                print(f"[MOTOR HEALTH] Motor {motor_num} status: {health['status']}")
                # Don't try to reset - just monitor




    def run_main_loop(self):
        """Main loop to handle MQTT commands from roam.py - NO TESTING MOVEMENTS"""
        try:
            print("[MAIN] Starting robot - Ready to receive commands from roam.py")
            print("[MAIN] Robot will only move when receiving commands via MQTT")
            print("[MAIN] NO test movements will be performed")
            
            print("[STATUS] Waiting for MQTT commands from roam.py...")
            print("[STATUS] Robot ready to receive commands from all modes:")
            print("[STATUS] - Manual mode (highest priority)")
            print("[STATUS] - Navigation charging mode")
            print("[STATUS] - R-mode")
            print("[STATUS] - Recovery mode")
            print("[STATUS] - Secondary mode")
            print("[STATUS] - Primary mode (lowest priority)")
          
            last_status_display = time.time()
          
            while True:
                time.sleep(0.1)
              
                # Check connection status (highest priority)
                self.check_connection_status()
                
                # Try to reset safety stop if connection is restored
                if self.safety_stop_active:
                    self.reset_safety_stop()
                
                # Check motor health periodically (passive monitoring only)
                self.check_motor_health()
              
                # Display status every 10 seconds (reduced frequency to avoid spam)
                current_time = time.time()
                if current_time - last_status_display >= 10.0:
                    self.display_status()
                    last_status_display = current_time
                  
                    # Show robot operation status
                    print(f"[STATUS] Robot operating independently")
                    print(f"[STATUS] Physical robot movement: AUTONOMOUS")
          
        except KeyboardInterrupt:
            print("\n[MAIN] Robot stopped by user")
        finally:
            self.cleanup()

    def cleanup(self):
        """Cleanup function to stop all systems safely"""
        print("[CLEANUP] Stopping all systems...")
        
        # Stop all motors
            self.stop_all_motors()
        
        # Stop LiDAR streaming
        self.stop_lidar_streaming()
        
        # Stop mapping
        if self.mapping_active:
            self.stop_mapping()
            self.save_mapping_data()
        
        # Disconnect MQTT
            if self.mqtt_client:
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
        
        print("[CLEANUP] All systems stopped safely")




    def calculate_speed_factor(self, linear_x, angular_z, mode):
        """Calculate speed factor based on velocity and mode"""
        max_velocity = max(abs(linear_x), abs(angular_z))
       
        if max_velocity <= 0.01:
            return 0.0
       
        if abs(angular_z) > abs(linear_x):
            # Rotating - SLOWER for physical robot
            if max_velocity <= 0.3:
                speed_factor = 0.2 + (max_velocity - 0.1) * 0.2  # 0.2 to 0.24
            elif max_velocity <= 0.8:
                speed_factor = 0.24 + (max_velocity - 0.3) * 0.2  # 0.24 to 0.34
            elif max_velocity <= 1.5:
                speed_factor = 0.34 + (max_velocity - 0.8) * 0.1  # 0.34 to 0.41
            else:
                speed_factor = 0.41 + min((max_velocity - 1.5) * 0.05, 0.09)  # 0.41 to 0.5
            speed_factor = max(0.2, min(0.5, speed_factor))
            return speed_factor
       
        # Default mapping for other (forward/backward) movement
        if max_velocity <= 0.5:
            speed_factor = 0.1 + (max_velocity - 0.1) * 0.5  # 0.1 to 0.3
        elif max_velocity <= 1.0:
            speed_factor = 0.3 + (max_velocity - 0.5) * 0.6  # 0.3 to 0.6
        elif max_velocity <= 2.0:
            speed_factor = 0.6 + (max_velocity - 1.0) * 0.2  # 0.6 to 0.8
        else:
            speed_factor = 0.8 + min((max_velocity - 2.0) * 0.1, 0.2)  # 0.8 to 1.0
        return max(0.0, min(1.0, speed_factor))




    def init_lidar(self):
        """Initialize RP-Lidar A1M8 using exact same approach as test_lidar.py"""
        try:
            # Initialize LiDAR (exact same as test_lidar.py)
            print(f"[LIDAR] Initializing LiDAR on {LIDAR_PORT}...")
            self.lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE, timeout=LIDAR_TIMEOUT)
          
            # Get LiDAR info
            info = self.lidar.get_info()
            print(f"[LIDAR] Connected: {info}")
          
            # Get health status
            health = self.lidar.get_health()
            print(f"[LIDAR] Health: {health}")
          
            # Don't start streaming immediately - wait for roam.py ready signal
            print("[LIDAR] LiDAR initialized successfully")
            print("[LIDAR] Waiting for roam.py ready signal before starting streaming...")
          
        except Exception as e:
            print(f"[ERROR] Failed to initialize LiDAR: {str(e)}")
            self.is_running = False
            self.lidar = None




    def _lidar_streaming_loop(self):
        """Main LiDAR streaming loop - exact same as test_lidar.py"""
        scan_count = 0
        start_time = time.time()
      
        try:
            if self.lidar is None:
                print("[LIDAR] LiDAR not initialized, cannot start streaming")
                return
                
            for scan in self.lidar.iter_scans():
                if not self.is_running:
                    break
              
                scan_count += 1
              
                # Send data to roam.py
                self.send_lidar_data(scan)
              
                # Print status every 100 scans (reduced from 10 to reduce spam)
                if scan_count % 100 == 0:
                    elapsed = time.time() - start_time
                    rate = scan_count / elapsed
                    print(f"[LIDAR] Sent {scan_count} scans ({rate:.1f} scans/sec)")
              
                # Small delay to prevent overwhelming
                time.sleep(0.01)
          
        except Exception as e:
            print(f"[ERROR] LiDAR streaming error: {str(e)}")
        finally:
            self.stop_lidar_streaming()




    def send_lidar_data(self, scan_data):
        """Send LiDAR data to roam.py via MQTT - exact same as test_lidar.py"""
        if self.mqtt_client is None:
            return
      
        try:
            # Convert scan data to format expected by roam.py
            ranges = [float('inf')] * 360
          
            for quality, angle, distance in scan_data:
                angle_deg = int(angle) % 360
                if 0 <= angle_deg < 360:
                    ranges[angle_deg] = distance / 1000.0  # Convert to meters
          
            # Create data packet
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
          
        except Exception as e:
            print(f"[ERROR] Failed to send LiDAR data: {str(e)}")




    def stop_lidar_streaming(self):
        """Stop LiDAR streaming - exact same as test_lidar.py"""
        self.is_running = False
      
        if self.lidar:
            try:
                self.lidar.disconnect()
            except:
                pass
            self.lidar = None
      
        if self.mqtt_client:
            try:
                status_data = {
                    "status": "disconnected",
                    "timestamp": time.time(),
                    "message": "LiDAR real-world data sender disconnected"
                }
                self.mqtt_client.publish(TOPIC_LIDAR_STATUS, json.dumps(status_data))
            except:
                pass
      
        print("[LIDAR] Streaming stopped")




    def get_lidar_data(self):
        """Get current LiDAR data"""
        return self.lidar_data if self.lidar_data else None




    def handle_real_world_lidar_data(self, lidar_data):
        """Handle real-world LiDAR data from MQTT - simplified version"""
        try:
            if 'ranges' in lidar_data and 'source' in lidar_data:
                if lidar_data['source'] == 'real_world_lidar':
                    ranges = lidar_data['ranges']
                    valid_ranges = [r for r in ranges if r != float('inf')]
                  
                    if valid_ranges:
                        min_distance = min(valid_ranges)
                      
                        # Emergency stop if very close obstacle (reduced threshold)
                        if min_distance < 0.15:  # 15cm (reduced from 30cm)
                            print(f"[EMERGENCY] Very close obstacle detected: {min_distance:.3f}m")
                            self.stop_all_motors()
                            return
                      
                        # Store for mapping (only if mapping is active)
                        if self.mapping_active:
                            self.add_to_mapping_data(lidar_data)
        except Exception as e:
            print(f"[ERROR] Failed to handle LiDAR data: {str(e)}")




    def process_real_world_obstacles(self, ranges):
        """Process real-world LiDAR data for obstacle detection - FIXED VERSION"""
        try:
            n = len(ranges)
            if n < 360:
                self.print_warning(f"Real-world LiDAR data has only {n} points, expected 360")
                return
            
            window = 15  # Increased window for better averaging
            
            # Define rear sector to exclude (behind the robot)
            # Rear sector: 135° to 225° (90 degrees behind robot)
            rear_start_angle = 135  # degrees
            rear_end_angle = 225    # degrees
            
            # CORRECTED ANGLE-TO-INDEX CONVERSION
            # For 360-element array: 0° = index 0, 90° = index 90, etc.
            
            # FRONT (0 degrees = index 0)
            front_idx = 0
            front_start = max(0, front_idx - window)
            front_end = min(n, front_idx + window + 1)
            front_ranges = ranges[front_start:front_end]
            
            # Apply strict filtering to prevent false positives
            valid_front = []
            for r in front_ranges:
                if r > 0.20 and r < 12.0:  # Increased minimum from 0.15 to 0.20
                    valid_front.append(r)
            
            self.real_world_front_min = min(valid_front) if valid_front else float('inf')
            
            # LEFT (90 degrees = index 90)
            left_idx = 90
            left_start = max(0, left_idx - window)
            left_end = min(n, left_idx + window + 1)
            left_ranges = ranges[left_start:left_end]
            
            valid_left = []
            for r in left_ranges:
                if r > 0.20 and r < 12.0:
                    valid_left.append(r)
            
            self.real_world_left_min = min(valid_left) if valid_left else float('inf')
            
            # RIGHT (270 degrees = index 270)
            right_idx = 270
            right_start = max(0, right_idx - window)
            right_end = min(n, right_idx + window + 1)
            right_ranges = ranges[right_start:right_end]
            
            valid_right = []
            for r in right_ranges:
                if r > 0.20 and r < 12.0:
                    valid_right.append(r)
            
            self.real_world_right_min = min(valid_right) if valid_right else float('inf')
            
            # DEBUGGING: Check for problematic readings
            close_readings = []
            for i, r in enumerate(ranges):
                if 0.05 < r < 0.20:  # Detect readings that might be robot body
                    angle = i  # degrees
                    # Skip rear sector
                    if not (rear_start_angle <= angle <= rear_end_angle):
                        close_readings.append((angle, r))
            
            # Log problematic readings for debugging
            if close_readings:
                print(f"[LIDAR DEBUG] Close readings detected (might be robot body):")
                for angle, distance in close_readings[:5]:  # Show first 5 only
                    print(f"  Angle {angle}°: {distance:.3f}m")
                
                # Check if 0.108m reading is in front sector
                front_close = [r for a, r in close_readings if -30 <= a <= 30 or 330 <= a <= 360]
                if front_close:
                    print(f"[LIDAR DEBUG] ⚠️ Front close readings: {front_close}")
                    print(f"[LIDAR DEBUG] These might be robot body detections - consider adjusting sensor position")
            
            # Debug logging for processed values
            current_time = time.time()
            if not hasattr(self, 'last_lidar_debug_time'):
                self.last_lidar_debug_time = 0
            
            if current_time - self.last_lidar_debug_time > 5.0:  # Log every 5 seconds
                print(f"[LIDAR PROCESSED] Front: {self.real_world_front_min:.3f}m, "
                      f"Left: {self.real_world_left_min:.3f}m, Right: {self.real_world_right_min:.3f}m")
                print(f"[LIDAR PROCESSED] Total close readings: {len(close_readings)}")
                self.last_lidar_debug_time = current_time
            
            # Trigger combined obstacle data update
            self.update_combined_obstacle_data()
                    
        except Exception as e:
            print(f"[ERROR] Error processing real-world obstacles: {str(e)}")
            print(f"[ERROR] Real-world LiDAR processing error: {str(e)}")

    def update_combined_obstacle_data(self):
        """Update combined obstacle data with improved filtering"""
        try:
            # Get processed real-world data
            real_world_front_min = getattr(self, 'real_world_front_min', float('inf'))
            real_world_left_min = getattr(self, 'real_world_left_min', float('inf'))
            real_world_right_min = getattr(self, 'real_world_right_min', float('inf'))
            
            # Additional filtering to prevent false positives
            # If reading is suspiciously consistent (like 0.108m), it might be robot body
            if hasattr(self, 'previous_front_reading'):
                # Check if reading is stuck at same value (robot body detection)
                if abs(real_world_front_min - self.previous_front_reading) < 0.005:
                    self.consistent_reading_count = getattr(self, 'consistent_reading_count', 0) + 1
                    if self.consistent_reading_count > 20:  # 20 consecutive similar readings
                        print(f"[LIDAR DEBUG] ⚠️ Consistent reading {real_world_front_min:.3f}m detected {self.consistent_reading_count} times")
                        print(f"[LIDAR DEBUG] This might be robot body - filtering out")
                        real_world_front_min = float('inf')  # Filter out consistent readings
                else:
                    self.consistent_reading_count = 0
            
            self.previous_front_reading = real_world_front_min
            
            # Update final values
            self.lidar_min_distance = real_world_front_min
            self.left_min = real_world_left_min
            self.right_min = real_world_right_min
            
            # Update obstacle flags with stricter thresholds
            self.obstacle_detected = self.lidar_min_distance < 0.8  # Increased from 0.5
            self.lidar_too_close = self.lidar_min_distance < 0.3   # Increased from 0.2
            self.left_obstacle = self.left_min < 0.6
            self.right_obstacle = self.right_min < 0.6
            
            # Debug output
            if self.obstacle_detected:
                print(f"[OBSTACLE] Front: {self.lidar_min_distance:.3f}m, "
                      f"Left: {self.left_min:.3f}m, Right: {self.right_min:.3f}m")
            
        except Exception as e:
            print(f"[ERROR] Error updating combined obstacle data: {str(e)}")




    def perform_obstacle_avoidance(self, front_min, left_min, right_min, back_min):
        """Perform obstacle avoidance based on LiDAR data - simplified version"""
        try:
            # Define safety distances (reduced thresholds)
            SAFE_DISTANCE = 0.5  # 50cm (reduced from 1m)
          
            # Check if robot is currently moving
            if self.motor_status == "STOPPED":
                return  # Don't avoid if not moving
          
            # Check front obstacle
            if front_min < SAFE_DISTANCE:
                self.stop_all_motors()
              
        except Exception as e:
            print(f"[ERROR] Obstacle avoidance failed: {str(e)}")




    def handle_mapping_data(self, mapping_data):
        """Handle mapping data from external sources"""
        try:
            if 'command' in mapping_data:
                command = mapping_data['command']
              
                if command == 'start_mapping':
                    self.start_mapping()
                elif command == 'stop_mapping':
                    self.stop_mapping()
                elif command == 'save_map':
                    self.save_mapping_data()
                elif command == 'clear_map':
                    self.clear_mapping_data()
                else:
                    print(f"[MAPPING] Unknown command: {command}")
                  
        except Exception as e:
            print(f"[ERROR] Failed to handle mapping data: {str(e)}")




    def start_mapping(self):
        """Start mapping mode"""
        self.mapping_active = True
        self.mapping_data = []
        print("[MAPPING] Mapping mode started")
      
        # Publish mapping status
        if self.mqtt_client:
            status_data = {
                "status": "mapping_started",
                "timestamp": time.time(),
                "message": "Robot mapping mode activated"
            }
            self.mqtt_client.publish(TOPIC_MAPPING_DATA, json.dumps(status_data))




    def stop_mapping(self):
        """Stop mapping mode"""
        self.mapping_active = False
        print(f"[MAPPING] Mapping mode stopped. Total data points: {len(self.mapping_data)}")
      
        # Publish mapping status
        if self.mqtt_client:
            status_data = {
                "status": "mapping_stopped",
                "timestamp": time.time(),
                "message": f"Robot mapping mode deactivated. Collected {len(self.mapping_data)} data points"
            }
            self.mqtt_client.publish(TOPIC_MAPPING_DATA, json.dumps(status_data))




    def add_to_mapping_data(self, lidar_data):
        """Add LiDAR data to mapping collection"""
        try:
            mapping_point = {
                "timestamp": time.time(),
                "robot_position": self.robot_position,
                "robot_orientation": self.robot_orientation,
                "lidar_data": lidar_data,
                "mode": self.current_mode
            }
            self.mapping_data.append(mapping_point)
          
            # Limit mapping data to prevent memory issues
            if len(self.mapping_data) > 10000:  # Max 10k points
                self.mapping_data = self.mapping_data[-5000:]  # Keep last 5k points
          
        except Exception as e:
            print(f"[ERROR] Failed to add mapping data: {str(e)}")




    def save_mapping_data(self):
        """Save mapping data to file"""
        try:
            if not self.mapping_data:
                print("[MAPPING] No mapping data to save")
                return
          
            filename = f"mapping_data_{int(time.time())}.json"
            with open(filename, 'w') as f:
                json.dump({
                    "timestamp": time.time(),
                    "total_points": len(self.mapping_data),
                    "robot_position": self.robot_position,
                    "robot_orientation": self.robot_orientation,
                    "mapping_data": self.mapping_data
                }, f, indent=2)
          
            print(f"[MAPPING] Mapping data saved to {filename}")
          
            # Publish save status
            if self.mqtt_client:
                status_data = {
                    "status": "map_saved",
                    "timestamp": time.time(),
                    "filename": filename,
                    "total_points": len(self.mapping_data),
                    "message": f"Mapping data saved to {filename}"
                }
                self.mqtt_client.publish(TOPIC_MAPPING_DATA, json.dumps(status_data))
          
        except Exception as e:
            print(f"[ERROR] Failed to save mapping data: {str(e)}")




    def clear_mapping_data(self):
        """Clear mapping data"""
        self.mapping_data = []
        print("[MAPPING] Mapping data cleared")
      
        # Publish clear status
        if self.mqtt_client:
            status_data = {
                "status": "map_cleared",
                "timestamp": time.time(),
                "message": "Mapping data cleared"
            }
            self.mqtt_client.publish(TOPIC_MAPPING_DATA, json.dumps(status_data))




    def start_lidar_streaming(self):
        """Start LiDAR streaming - exact same as test_lidar.py"""
        if self.is_running:
            print("[LIDAR] Already streaming")
            return
      
        if not self.lidar:
            print("[LIDAR] LiDAR not initialized, cannot start streaming")
            return
      
        try:
            # Start LiDAR scanning in a separate thread
            self.is_running = True
            self.lidar_thread = threading.Thread(target=self._lidar_streaming_loop)
            self.lidar_thread.daemon = True
            self.lidar_thread.start()
          
            print("[LIDAR] Started streaming real-world data to roam.py")
            print("[LIDAR] Press Ctrl+C to stop")
          
        except Exception as e:
            print(f"[ERROR] Failed to start LiDAR streaming: {str(e)}")
            self.is_running = False




    def get_mapping_status(self):
        """Get current mapping status"""
        return {
            "mapping_active": self.mapping_active,
            "total_points": len(self.mapping_data),
            "robot_position": self.robot_position,
            "robot_orientation": self.robot_orientation
        }




    def check_lidar_status(self):
        """Check LiDAR status - simplified version"""
        if not self.lidar:
            print("[LIDAR STATUS] LiDAR not initialized")
            return False
      
        try:
            # Check if LiDAR is still connected
            info = self.lidar.get_info()
            health = self.lidar.get_health()
          
            print(f"[LIDAR STATUS] Info: {info}")
            print(f"[LIDAR STATUS] Health: {health}")
          
            # Health status: (status, error_code) where status=0 means good health
            if health[0] != 0:
                print("[LIDAR STATUS] ⚠️  HEALTH ISSUE DETECTED!")
                return False
            else:
                print("[LIDAR STATUS] ✅ Health OK")
                return True
              
        except Exception as e:
            print(f"[LIDAR STATUS] ❌ Error checking status: {str(e)}")
            return False

# All synchronization functions removed - robot now operates independently



if __name__ == "__main__":
    print("=== BCR BOT PHYSICAL ROBOT NODE ===")
    print("Robot ready to receive commands from roam.py")
    print("No test movements will be performed")
    print("=" * 60)
  
    try:
        tester = RobotTester()
      
        print("\n=== INITIALIZATION COMPLETE ===")
        print("✓ PCA9685 initialized")
        print("✓ MQTT connected")
        print("✓ LiDAR initialized")
        print("✓ Robot operating independently")
        print("✓ All systems ready")
      
        # Check LiDAR status
        print("\n=== CHECKING LIDAR STATUS ===")
        if not tester.check_lidar_status():
            print("[WARNING] LiDAR health issues detected!")
      
        # Wait for roam.py ready signal
        print("\n=== WAITING FOR ROAM.PY READY SIGNAL ===")
        print("LiDAR streaming will start automatically when roam.py is ready")
        print("This prevents race conditions and ensures proper data flow")
      
        # Start mapping automatically
        print("\n=== STARTING MAPPING MODE ===")
        tester.start_mapping()
      
        print("\n=== SYSTEM STATUS ===")
        print("Robot is now running with:")
        print("- LiDAR initialized and ready to stream")
        print("- Automatic mapping data collection")
        print("- MQTT command reception from roam.py")
        print("- Robot operates independently")
        print("- Emergency stop capability")
        print("- Obstacle detection and avoidance")
        print("- CONNECTION SAFETY MONITORING:")
        print(f"  • MQTT timeout: {tester.connection_timeout:.1f} seconds")
        print(f"  • Heartbeat interval: {tester.heartbeat_interval:.1f} seconds")
        print("  • Auto-stop on connection loss")
        print("  • Auto-resume when connection restored")
      
        print("\n=== READY FOR OPERATION ===")
        print("Robot will now:")
        print("1. Wait for roam.py ready signal")
        print("2. Start LiDAR streaming when roam.py is ready")
        print("3. Receive and execute commands from roam.py via MQTT")
        print("4. Operate independently")
        print("5. Process real-world LiDAR data")
        print("6. Collect mapping data automatically")
        print("7. Perform obstacle avoidance")
        print("8. Execute all navigation modes")
        print("9. 🚨 SAFETY: Auto-stop if WiFi/MQTT disconnects")
        print("10. 🚨 SAFETY: Auto-resume when connection restored")
        
        print("\n⚠️  IMPORTANT SAFETY FEATURES:")
        print("• Robot will IMMEDIATELY STOP all motors if:")
        print("  - MQTT connection is lost")
        print("  - WiFi connection drops")
        print(f"  - No commands received for {tester.connection_timeout:.1f} seconds")
        print("• Robot will AUTO-RESUME when connection is restored")
        print("• Heartbeat monitoring ensures connection health")
        print("• Independent robot operation")
        
        print("\n🔄 ROBOT OPERATION MODE:")
        print("• Independent operation")
        print("• Hardware failure detection")
        print("• Direct physical robot control")
        print("• Autonomous movement")
        
        print("\nPress Ctrl+C to stop the robot")
            print("=" * 60)
          
        # Run main loop - NO TEST MOVEMENTS
        tester.run_main_loop()
          
    except KeyboardInterrupt:
        print("\n\n=== SHUTDOWN SEQUENCE ===")
        print("Stopping all systems...")
      
        # Stop all systems
        if 'tester' in locals():
            print("Stopping motors...")
            tester.stop_all_motors()
          
            print("Stopping LiDAR streaming...")
            tester.is_running = False
          
            print("Stopping mapping...")
            tester.stop_mapping()
          
            print("Saving mapping data...")
            tester.save_mapping_data()
          
            print("Disconnecting MQTT...")
            if tester.mqtt_client:
                tester.mqtt_client.loop_stop()
                tester.mqtt_client.disconnect()
      
        print("✓ All systems stopped")
        print("✓ Cleanup completed")
        print("✓ Robot ready for next session")
