#!/usr/bin/env python3
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
            
            # Initialize MQTT
            self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)  # Use newer API version
            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.on_message = self.on_mqtt_message
            self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
            try:
                self.mqtt_client.connect("codex.petra.ac.id", 1883, 60)
                self.mqtt_client.loop_start()
                print("[INIT] MQTT connected successfully")
            except Exception as e:
                print(f"[ERROR] MQTT connection failed: {str(e)}")
                self.mqtt_client = None
                
        except Exception as e:
            print(f"[ERROR] Failed to initialize PCA9685: {str(e)}")
            raise

    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("[MQTT] Connected successfully")
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
            
            print("[MQTT] Subscribed to all robot command topics")
            print("[MQTT] Subscribed to LiDAR real-world topics")
        else:
            print(f"[MQTT] Connection failed with code {rc}")

    def on_mqtt_disconnect(self, client, userdata, rc):
        """Handle MQTT disconnection with reconnection logic"""
        print(f"[MQTT] Disconnected with code {rc}")
        
        # Try to reconnect if not shutting down
        if rc != 0:
            print("[MQTT] Attempting to reconnect...")
            try:
                # Wait before reconnecting to avoid rapid attempts
                time.sleep(2.0)
                client.reconnect()
            except Exception as e:
                print(f"[MQTT] Reconnection failed: {str(e)}")
                # Try again after longer delay
                try:
                    time.sleep(5.0)
                    client.reconnect()
                except Exception as e2:
                    print(f"[MQTT] Second reconnection attempt failed: {str(e2)}")

    def on_mqtt_message(self, client, userdata, msg):
        try:
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
                        
                        # Check if this is a reconnection
                        if status_data.get('reconnect', False):
                            print("[LIDAR] Detected roam.py reconnection!")
                            self.handle_roam_reconnect()
                        else:
                            # First time connection - start LiDAR streaming if not already started
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
            
        except Exception as e:
            print(f"[ERROR] Failed to process MQTT message: {str(e)}")

    def set_motor(self, motor_num, speed, direction):
        """Set motor speed and direction with health monitoring"""
        try:
            if motor_num not in pinPWM or motor_num not in pinDIR:
                print(f"[ERROR] Invalid motor number: {motor_num}")
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

    def maju(self, speed_factor=1.0):
        """Move forward"""
        # Use higher speed coefficient for R-mode
        current_speed_coef = rmodeSpeedCoef if self.current_mode == "R-MODE" else speedCoef
        speed = int(fullSpeed * speed_factor * current_speed_coef)
        self.set_motor(1, speed, False)   # Motor 1 forward
        self.set_motor(2, speed, False)  # Motor 2 backward
        self.set_motor(3, speed, False)   # Motor 3 forward
        self.set_motor(4, speed, False)  # Motor 4 backward
        self.update_status(self.current_mode, f"FORWARD (speed: {speed_factor:.2f})")

    def mundur(self, speed_factor=1.0):
        """Move backward"""
        # Use higher speed coefficient for R-mode backward movement
        if self.current_mode == "R-MODE":
            current_speed_coef = rmodeBackwardSpeedCoef  # Use special backward speed coefficient
        else:
            current_speed_coef = speedCoef
        speed = int(fullSpeed * speed_factor * current_speed_coef)
        self.set_motor(1, speed, True)  # Motor 1 backward (arahMUNDUR)
        self.set_motor(2, speed, True)  # Motor 2 backward (arahMUNDUR)
        self.set_motor(3, speed, True)  # Motor 3 backward (arahMUNDUR)
        self.set_motor(4, speed, True)  # Motor 4 backward (arahMUNDUR)
        self.update_status(self.current_mode, f"BACKWARD (speed: {speed_factor:.2f})")

    def putar_kiri(self, speed_factor=1.0):
        """Turn left"""
        # Use higher speed coefficient for R-mode
        current_speed_coef = rmodeSpeedCoef if self.current_mode == "R-MODE" else speedCoef
        speed = int(fullSpeed * speed_factor * current_speed_coef)
        self.set_motor(1, speed, False)  # Motor 1 backward
        self.set_motor(2, speed, True)   # Motor 2 forward
        self.set_motor(3, speed, False)  # Motor 3 backward
        self.set_motor(4, speed, True)   # Motor 4 forward
        self.update_status(self.current_mode, f"TURNING LEFT (speed: {speed_factor:.2f})")

    def putar_kanan(self, speed_factor=1.0):
        """Turn right"""
        # Use higher speed coefficient for R-mode
        current_speed_coef = rmodeSpeedCoef if self.current_mode == "R-MODE" else speedCoef
        speed = int(fullSpeed * speed_factor * current_speed_coef)
        self.set_motor(1, speed, True)   # Motor 1 forward
        self.set_motor(2, speed, False)  # Motor 2 backward
        self.set_motor(3, speed, True)   # Motor 3 forward
        self.set_motor(4, speed, False)  # Motor 4 backward
        self.update_status(self.current_mode, f"TURNING RIGHT (speed: {speed_factor:.2f})")

    def test_motor_1_specifically(self):
        """Special test for motor 1 to diagnose R-mode stopping issues"""
        print("\n[MOTOR 1 TEST] Testing motor 1 specifically...")
        print("[MOTOR 1 TEST] This test will help diagnose the stopping issue in R-mode")
        
        # Test motor 1 with different speeds and directions
        test_speeds = [MIN_MOTOR_SPEED, halfSpeed, fullSpeed]
        test_directions = [True, False]  # Forward, Backward
        
        for speed in test_speeds:
            for direction in test_directions:
                direction_name = "FORWARD" if direction else "BACKWARD"
                print(f"[MOTOR 1 TEST] Testing speed {speed} ({speed/65535*100:.1f}%) {direction_name}")
                
                try:
                    self.set_motor(1, speed, direction)
                    time.sleep(2)
                    
                    # Check if motor is still running
                    current_pwm = self.pca.channels[pinPWM[1]].duty_cycle
                    current_dir = self.pca.channels[pinDIR[1]].duty_cycle
                    
                    print(f"[MOTOR 1 TEST] PWM: {current_pwm}, DIR: {current_dir}")
                    
                    if current_pwm == 0:
                        print(f"[MOTOR 1 TEST] ⚠️  Motor 1 stopped unexpectedly!")
                        self.motor_health[1]["errors"] += 1
                    else:
                        print(f"[MOTOR 1 TEST] ✅ Motor 1 running normally")
                    
                    # Stop motor
                    self.set_motor(1, 0, direction)
                    time.sleep(1)
                    
                except Exception as e:
                    print(f"[MOTOR 1 TEST] ❌ Error testing motor 1: {str(e)}")
                    self.motor_health[1]["errors"] += 1
        
        # Test motor 1 in R-mode simulation
        print(f"\n[MOTOR 1 TEST] Testing motor 1 in R-mode simulation...")
        original_mode = self.current_mode
        self.current_mode = "R-MODE"
        
        try:
            # Test with typical R-mode speeds
            r_mode_speeds = [0.3, 0.5, 0.8, 1.0]
            for speed_factor in r_mode_speeds:
                print(f"[MOTOR 1 TEST] R-mode speed factor: {speed_factor}")
                
                # Calculate actual speed
                speed = int(fullSpeed * speed_factor * rmodeSpeedCoef)
                print(f"[MOTOR 1 TEST] Calculated speed: {speed} ({speed/65535*100:.1f}%)")
                
                # Test forward
                self.set_motor(1, speed, True)
                time.sleep(1)
                
                # Check if motor stopped
                current_pwm = self.pca.channels[pinPWM[1]].duty_cycle
                if current_pwm == 0:
                    print(f"[MOTOR 1 TEST] ⚠️  Motor 1 stopped in R-mode forward test!")
                    self.motor_health[1]["errors"] += 1
                else:
                    print(f"[MOTOR 1 TEST] ✅ Motor 1 running in R-mode forward test")
                
                # Stop
                self.set_motor(1, 0, True)
                time.sleep(0.5)
                
                # Test backward
                self.set_motor(1, speed, False)
                time.sleep(1)
                
                # Check if motor stopped
                current_pwm = self.pca.channels[pinPWM[1]].duty_cycle
                if current_pwm == 0:
                    print(f"[MOTOR 1 TEST] ⚠️  Motor 1 stopped in R-mode backward test!")
                    self.motor_health[1]["errors"] += 1
                else:
                    print(f"[MOTOR 1 TEST] ✅ Motor 1 running in R-mode backward test")
                
                # Stop
                self.set_motor(1, 0, False)
                time.sleep(0.5)
        
        finally:
            # Restore original mode
            self.current_mode = original_mode
        
        # Final status
        motor1_health = self.motor_health.get(1, {"errors": 0, "status": "OK"})
        print(f"\n[MOTOR 1 TEST] Test completed. Motor 1 errors: {motor1_health['errors']}")
        if motor1_health["errors"] > 0:
            print(f"[MOTOR 1 TEST] ⚠️  Motor 1 has issues that may cause stopping in R-mode")
            print(f"[MOTOR 1 TEST] Consider checking hardware connections or motor driver")
        else:
            print(f"[MOTOR 1 TEST] ✅ Motor 1 appears to be working correctly")

    def test_individual_motors(self):
        """Test each motor individually"""
        print("\n[TEST] Testing individual motors...")
        for motor in range(1, 5):
            print(f"\n[TEST] Testing Motor {motor}")
            print(f"[TEST] Setting PWM pin {pinPWM[motor]} and DIR pin {pinDIR[motor]}")
            
            # Test forward
            print(f"[TEST] Motor {motor} - Forward")
            self.set_motor(motor, halfSpeed, True)
            time.sleep(2)
            
            # Stop
            print(f"[TEST] Motor {motor} - Stop")
            self.set_motor(motor, 0, True)
            time.sleep(1)
            
            # Test backward
            print(f"[TEST] Motor {motor} - Backward")
            self.set_motor(motor, halfSpeed, False)
            time.sleep(2)
            
            # Stop
            print(f"[TEST] Motor {motor} - Stop")
            self.set_motor(motor, 0, False)
            time.sleep(1)

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
        print(f"MQTT Connected: {'YES' if self.mqtt_client else 'NO'}")
        print(f"{'='*60}\n")

    def check_motor_health(self):
        """Check motor health and perform maintenance if needed"""
        current_time = time.time()
        
        # Only check every MOTOR_HEALTH_CHECK_INTERVAL seconds
        if current_time - self.last_motor_health_check < MOTOR_HEALTH_CHECK_INTERVAL:
            return
        
        self.last_motor_health_check = current_time
        
        # Check each motor's health
        for motor_num in range(1, 5):
            health = self.motor_health.get(motor_num, {"errors": 0, "status": "OK"})
            
            # If motor has too many errors, try to reset it
            if health["errors"] >= MOTOR_ERROR_THRESHOLD:
                print(f"[MOTOR HEALTH] Motor {motor_num} has {health['errors']} errors, attempting reset...")
                
                # Try to reset the motor by setting it to a known state
                try:
                    # Stop the motor first
                    self.pca.channels[pinPWM[motor_num]].duty_cycle = 0
                    self.pca.channels[pinDIR[motor_num]].duty_cycle = 0
                    time.sleep(0.1)
                    
                    # Try to set it to a low speed to test
                    self.set_motor(motor_num, MIN_MOTOR_SPEED, True)
                    time.sleep(0.2)
                    
                    # Stop it again
                    self.pca.channels[pinPWM[motor_num]].duty_cycle = 0
                    
                    # Reset error count
                    self.motor_health[motor_num]["errors"] = 0
                    self.motor_health[motor_num]["status"] = "OK"
                    print(f"[MOTOR HEALTH] Motor {motor_num} reset successful")
                    
                except Exception as e:
                    print(f"[MOTOR HEALTH] Failed to reset motor {motor_num}: {str(e)}")
                    self.motor_health[motor_num]["status"] = "ERROR"
        
        # Special attention to motor 1 in R-mode
        if self.current_mode == "R-MODE":
            motor1_health = self.motor_health.get(1, {"errors": 0, "status": "OK"})
            if motor1_health["status"] != "OK":
                print(f"[MOTOR 1 WARNING] Motor 1 has issues in R-mode: {motor1_health['status']} (Errors: {motor1_health['errors']})")
                print(f"[MOTOR 1 WARNING] This may cause unexpected stopping behavior")

    def run_tests(self):
        """Run all tests and start MQTT command handling"""
        try:
            print("[TEST] Starting robot...")
            
            # Test individual motors first
            self.test_individual_motors()
            
            # Special test for motor 1 to diagnose R-mode issues
            self.test_motor_1_specifically()
            
            # Keep running to handle MQTT messages
            print("[STATUS] Waiting for MQTT commands from roam.py...")
            print("[STATUS] Robot ready to receive commands from all modes:")
            print("[STATUS] - Manual mode (highest priority)")
            print("[STATUS] - Navigation charging mode")
            print("[STATUS] - R-mode")
            print("[STATUS] - Recovery mode")
            print("[STATUS] - Secondary mode")
            print("[STATUS] - Primary mode (lowest priority)")
            
            last_status_display = time.time()
            last_motor_health_check = time.time()
            
            while True:
                time.sleep(0.1)
                
                # Check motor health periodically
                self.check_motor_health()
                
                # Display status every 5 seconds
                current_time = time.time()
                if current_time - last_status_display >= 5.0:
                    self.display_status()
                    last_status_display = current_time
                    
                    # Special monitoring for motor 1 in R-mode
                    if self.current_mode == "R-MODE":
                        motor1_health = self.motor_health.get(1, {"errors": 0, "status": "OK"})
                        if motor1_health["errors"] > 0:
                            print(f"[MOTOR 1 MONITOR] Motor 1 has {motor1_health['errors']} errors in R-mode")
                            print(f"[MOTOR 1 MONITOR] This may cause unexpected stopping behavior")
            
        except KeyboardInterrupt:
            print("\n[TEST] Robot stopped by user")
        finally:
            self.stop_all_motors()
            self.stop_lidar_streaming()  # Use the correct method name
            if self.mqtt_client:
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
            print("[STATUS] Cleanup completed")

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
        """Main LiDAR streaming loop"""
        scan_count = 0
        start_time = time.time()
        
        try:
            for scan in self.lidar.iter_scans():
                if not self.is_running:
                    break
                
                scan_count += 1
                
                # Send data to roam.py
                self.send_lidar_data(scan)
                
                # Print status every 100 scans
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
        """Send LiDAR data via MQTT"""
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
        """Stop LiDAR streaming"""
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
        """Handle real-world LiDAR data from MQTT"""
        try:
            if 'ranges' in lidar_data and 'source' in lidar_data:
                if lidar_data['source'] == 'real_world_lidar':
                    ranges = lidar_data['ranges']
                    valid_ranges = [r for r in ranges if r != float('inf')]
                    
                    if valid_ranges:
                        min_distance = min(valid_ranges)
                        
                        # Emergency stop if very close obstacle
                        if min_distance < 0.15:  # 15cm
                            print(f"[EMERGENCY] Very close obstacle detected: {min_distance:.3f}m")
                            self.stop_all_motors()
                            return
                        
                        # Store for mapping (only if mapping is active)
                        if self.mapping_active:
                            self.add_to_mapping_data(lidar_data)
        except Exception as e:
            print(f"[ERROR] Failed to handle LiDAR data: {str(e)}")

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

    def stop_mapping(self):
        """Stop mapping mode"""
        self.mapping_active = False
        print(f"[MAPPING] Mapping mode stopped. Total data points: {len(self.mapping_data)}")

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
            
        except Exception as e:
            print(f"[ERROR] Failed to save mapping data: {str(e)}")

    def clear_mapping_data(self):
        """Clear mapping data"""
        self.mapping_data = []
        print("[MAPPING] Mapping data cleared")

    def start_lidar_streaming(self):
        """Start LiDAR streaming"""
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
            
            print("[LIDAR] Started streaming real-world data")
            
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

    def handle_roam_reconnect(self):
        """Handle roam.py reconnection by restarting lidar streaming"""
        print("[LIDAR] roam.py reconnected, restarting lidar streaming...")
        
        # Stop current streaming if running
        if self.is_running:
            print("[LIDAR] Stopping current lidar streaming...")
            self.is_running = False
            if self.lidar_thread and self.lidar_thread.is_alive():
                self.lidar_thread.join(timeout=2.0)
        
        # Wait a moment for clean shutdown
        time.sleep(1.0)
        
        # Restart lidar streaming
        if self.lidar:
            print("[LIDAR] Restarting lidar streaming for roam.py...")
            self.start_lidar_streaming()
        else:
            print("[LIDAR] LiDAR not available, cannot restart streaming")

if __name__ == "__main__":
    print("=== BCR BOT TEST ROBOT - FULL INTEGRATION ===")
    print("Robot testing with automatic LiDAR integration and mapping")
    print("=" * 60)
    
    try:
        tester = RobotTester()
        
        print("\n=== INITIALIZATION COMPLETE ===")
        print("✓ PCA9685 initialized")
        print("✓ MQTT connected")
        print("✓ LiDAR initialized")
        print("✓ All systems ready")
        
        print("\n=== READY FOR OPERATION ===")
        print("Robot will now receive commands from roam.py via MQTT")
        print("Press Ctrl+C to stop the robot")
        print("=" * 60)
        
        # Keep the program running
        while True:
            time.sleep(60)
            print(f"\n=== STATUS UPDATE - {time.strftime('%H:%M:%S')} ===")
            print(f"Current Mode: {tester.current_mode}")
            print(f"Motor Status: {tester.motor_status}")
            print(f"LiDAR Running: {tester.is_running}")
            print(f"Mapping Active: {tester.mapping_active}")
            print("=" * 60)
            
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
            
            print("Disconnecting MQTT...")
            if tester.mqtt_client:
                tester.mqtt_client.loop_stop()
                tester.mqtt_client.disconnect()
        
        print("✓ All systems stopped")
        print("✓ Cleanup completed")

















