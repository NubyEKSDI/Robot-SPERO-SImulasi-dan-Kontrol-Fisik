#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import threading
import sys
import select
import time
import tty
import termios
import os
import signal
import queue
import random
import paho.mqtt.client as mqtt
import json
import uuid

# Set ROS_DOMAIN_ID untuk komunikasi dengan Raspberry Pi
os.environ['ROS_DOMAIN_ID'] = '0'  # Sesuaikan dengan domain ID Raspberry Pi

class RoamingRobot(Node):
    def __init__(self):
        super().__init__('roaming_robot')
        
        # Initialize position tracking first
        self.last_odom_pos = None
        self.last_position = None
        self.position_threshold = 0.1
        self.stuck_time = 0
        self.stuck_threshold = 5.0
        self.stuck = False
        
        # Initialize status tracking
        self.status_queue = queue.Queue()
        self.stuck_counter = 0
        self.max_stuck = 20
        self.last_turn_time = time.time()
        self.scan_data = None
        self.rotation_progress = 0
        self.last_input_time = time.time()
        self.status = 'Waiting for input...'
        self.last_status = ''
        self.last_status_update = time.time()
        self.status_update_interval = 2.0  # Increased from 0.5 to 2.0 seconds
        
        # Initialize turn cooldown
        self.last_turn_direction = None  # 'left' or 'right'
        self.turn_cooldown = 10.0  # Increased from 5.0 to 10.0 seconds
        self.last_turn_time = 0.0
        
        # Initialize control mode flags
        self.primary_active = True  # Primary control starts active
        self.secondary_control_active = False
        self.manual_mode = False
        self.r_mode = False
        
        # Initialize navigation charging attributes
        self.nav_charging_mode = False
        self.charging_station_pos = (0.0, 0.0)
        self.charging_threshold = 0.5  # Distance threshold to consider "at charging station"
        self.charging_angle_threshold = 0.3  # Angle threshold for charging navigation
        self.charging_forward_speed = 0.5  # Same as R-mode forward speed
        self.charging_turn_speed = 0.8  # Slower rotation (reduced from 1.5)
        self.charging_obstacle_threshold = 1.2  # Dinaikkan dari 0.8 ke 1.2
        self.charging_emergency_stop_threshold = 0.5  # Dinaikkan dari 0.3 ke 0.5
        self.battery_low = False
        self.battery_voltage = 12.0  # Default battery voltage
        self.battery_low_threshold = 11.0  # Voltage threshold for low battery
        
        # Initialize ROS publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/bcr_bot/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/bcr_bot/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/bcr_bot/odom', self.odom_callback, 10)
        # Publisher for physical lidar data (removed duplicate)
        self.physical_scan_pub = self.create_publisher(LaserScan, '/bcr_bot/physical_scan', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.status_timer = self.create_timer(0.1, self.status_update_callback)
        
        # Initialize MQTT state
        self.mqtt_paused = False
        self.mqtt_client = None
        self.last_mqtt_message = None
        self.mqtt_connected = False  # Track MQTT connection status
        self.mqtt_connection_time = None  # Track when MQTT connected
        self.mqtt_reconnect_attempts = 0  # Track reconnection attempts
        self.mqtt_max_reconnect_attempts = 10  # Maximum reconnection attempts
        self.mqtt_reconnect_delay = 1.0  # Initial reconnection delay
        self.mqtt_reconnect_timer = None  # Timer for reconnection attempts
        
        # Initialize MQTT client with better error handling
        try:
            # Generate unique client ID to prevent conflicts when restarting
            client_id = f"roam_py_{uuid.uuid4().hex[:8]}"
            
            self.mqtt_client = mqtt.Client(client_id=client_id)
            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.on_message = self.on_mqtt_message
            self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
            
            # Set MQTT options for better stability
            self.mqtt_client.reconnect_delay_set(min_delay=1, max_delay=120)
            
            # Set clean session to True to ensure fresh connection
            self.mqtt_client._clean_session = True
            
            # Connect to MQTT broker
            self.mqtt_client.connect("spin5.petra.ac.id", 1883, 60)
            self.mqtt_client.loop_start()
            
            self.get_logger().info(f"Successfully connected to MQTT broker with client ID: {client_id}")
            self.print_status("MQTT: Connected to broker")
            
        except Exception as e:
            self.get_logger().warning(f"Failed to connect to MQTT broker: {str(e)}. Continuing without MQTT support.")
            self.print_status("MQTT: Connection failed")
            self.mqtt_client = None
        
        # Initialize turning behavior attributes
        self.consecutive_turns = 0
        self.max_consecutive_turns = 3
        self.backup_duration = 2.0
        self.backup_start_time = None
        self.is_backing_up = False
        self.turning_to_best = False
        self.recovery_end_time = 0
        self.recovery_phase = 0
        self.recovery_active = False
        self.recovery_start_time = None
        self.recovery_timeout = 10.0
        self.recovery_phase_start = None
        self.recovery_phase_duration = 2.0
        
        # Initialize obstacle detection variables
        self.obstacle_detected = False
        self.lidar_too_close = False
        self.lidar_min_distance = float('inf')
        self.left_min = float('inf')
        self.right_min = float('inf')
        self.left_obstacle = False
        self.right_obstacle = False
        self.obstacle_threshold = 1.5  # Dinaikkan dari 0.9 ke 1.5
        self.emergency_stop_threshold = 0.4  # Dinaikkan dari 0.2 ke 0.4
        
        # Enhanced obstacle detection variables
        self.front_obstacle_close = False
        self.front_obstacle_warning = False
        self.side_obstacle_close = False
        self.both_sides_blocked = False
        
        # Data source tracking
        self.front_source = "Gazebo"
        self.left_source = "Gazebo"
        self.right_source = "Gazebo"
        
        # Initialize Gazebo data
        self.gazebo_front_min = float('inf')
        self.gazebo_left_min = float('inf')
        self.gazebo_right_min = float('inf')
        
        # Initialize real-world LiDAR data
        self.real_world_front_min = float('inf')
        self.real_world_left_min = float('inf')
        self.real_world_right_min = float('inf')
        
        # Initialize movement control variables
        self.turning = False
        self.is_scanning = False
        self.scan_start_time = 0
        self.scan_duration = 3.0
        self.turn_start_time = 0
        self.turn_duration = 1.0
        
        # Initialize basic robot state
        self.running = True
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        signal.signal(signal.SIGINT, self.signal_handler)
        os.system('clear')
        self.print_instructions()
        
        # Initialize secondary control attributes
        self.secondary_active = False
        self.secondary_start_time = None
        self.secondary_timeout = 45.0
        
        # Initialize Twist message for cmd_vel
        self.cmd_vel = Twist()
        
        self.print_status('Waiting for input...')
        
        # Initialize waypoint attributes
        self.waypoint_mode = False
        self.current_waypoint = None
        self.waypoints = {
            'U': (2.0, 2.0),
            'I': (2.0, -2.0),
            'O': (-2.0, 0.0)
        }
        
        # Initialize line following attributes
        self.line_following_mode = False
        self.line_angle = 0.0
        self.line_center = (0.0, 0.0)
        self.line_radius = 2.0
        
        # Initialize R-mode attributes
        self.r_mode_timeout = 45.0
        self.r_mode_avoiding = False
        self.r_mode_avoid_start = None
        self.r_mode_avoid_phase = None
        self.r_mode_avoid_duration = 1.0
        self.r_mode_turning_duration = 0.0
        self.r_mode_turning_start = None
        self.r_mode_turning_direction = 0.0
        self.r_mode_avoid_speed = 0.3
        self.r_mode_turn_speed = 1.5
        self.r_mode_forward_speed = 1.6
        self.r_mode_obstacle_threshold = 1.2  # Dinaikkan dari 0.8 ke 1.2
        self.r_mode_navigation_phase = 'align_to_waypoint'
        self.r_mode_y_threshold = 0.1
        self.r_mode_x_threshold = 0.1
        self.r_mode_angle_threshold = 0.3
        self.r_mode_front_sector_size = 45
        self.r_mode_emergency_stop = 0.6  # Dinaikkan dari 0.5 ke 0.6
        self.r_mode_rotation_complete = False
        self.r_mode_target_angle = 0.0
        self.r_mode_last_angle_diff = 0.0
        self.specific_waypoint_navigation = False  # Flag for i, u, o waypoint navigation
        
        self.virtual_path = [
            (0.0, 0.0),    # Start point
            (-12.06, -0.02),    # Bottom-left point
            (-12.06,-1.79),    # Bottom-right point
            (15.31, -2.10),    # TOP RIGHT
            (3.57, -2.10),    # MID
            (3.57, 0.0),    # MID LEFT
        ]
        self.virtual_path_index = 0
        self.virtual_path_threshold = 0.3
        self.r_mode_waypoint_threshold = 0.3
        
        # Initialize secondary control attributes
        self.secondary_active = False
        self.secondary_start_time = None
        self.secondary_timeout = 45.0

    def signal_handler(self, signum, frame):
        """Handle Ctrl+C and other termination signals"""
        try:
            self.running = False
            print('\nShutdown signal received, cleaning up...')
            
            # Call cleanup to properly disconnect MQTT and stop robot
            self.cleanup()
            
            # Exit gracefully
            sys.exit(0)
        except Exception as e:
            print(f'\nError during signal handling: {str(e)}')
            sys.exit(1)

    def __del__(self):
        # Ensure MQTT client is disconnected
        try:
            if hasattr(self, 'mqtt_client') and self.mqtt_client is not None:
                try:
                    self.mqtt_client.loop_stop()
                    self.mqtt_client.disconnect()
                except Exception as e:
                    print(f'\nWarning: Could not disconnect MQTT client in __del__: {str(e)}')
        except Exception as e:
            pass  # Ignore errors in __del__
        
        # Restore terminal settings
        try:
            if hasattr(self, 'old_settings') and self.old_settings is not None:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        except Exception as e:
            print(f'\nWarning: Could not restore terminal settings: {str(e)}')

    def getch(self):
        """Get character from stdin with improved error handling"""
        try:
            return sys.stdin.read(1)
        except KeyboardInterrupt:
            print("[KEYBOARD] Keyboard interrupt in getch")
            return None
        except Exception as e:
            print(f"[KEYBOARD] Error reading from stdin: {str(e)}")
            return None

    def update_status(self, message):
        self.status_queue.put(message)

    def print_status(self, message):
        # Always print status for realtime updates
        timestamp = time.strftime("%H:%M:%S")
        
        # Add MQTT status
        mqtt_status = "MQTT: Connected" if self.mqtt_client is not None else "MQTT: Disconnected"
        if self.mqtt_paused:
            mqtt_status += " (Paused)"
        
        # Safely handle position display
        try:
            if hasattr(self, 'last_odom_pos') and self.last_odom_pos is not None:
                status_line = f'[{timestamp}] {mqtt_status} | {message} | Pos: x={self.last_odom_pos[0]:.2f} y={self.last_odom_pos[1]:.2f}'
            else:
                status_line = f'[{timestamp}] {mqtt_status} | {message} | Pos: unknown'
        except (AttributeError, TypeError):
            status_line = f'[{timestamp}] {mqtt_status} | {message} | Pos: unknown'
        
        # Store last status
        self.last_status = message
        
        # Clear line and print new status
        sys.stdout.write('\r\033[K' + status_line)
        sys.stdout.flush()

        # Send status only to Raspberry Pi
        if self.mqtt_client is not None:
            try:
                self.mqtt_client.publish("robot/physical/status", json.dumps({
                    'status': status_line,
                    'timestamp': time.time()
                }))
            except Exception as e:
                self.get_logger().error(f"Error sending status: {str(e)}")

    def print_warning(self, message):
        # Print warning on new line
        timestamp = time.strftime("%H:%M:%S")
        print(f'\n[{timestamp}] WARNING: {message}')
        # Reprint last status
        if hasattr(self, 'last_status'):
            self.print_status(self.last_status)

    def keyboard_listener(self):
        """Separate thread for handling keyboard input"""
        print("[KEYBOARD] Keyboard listener thread started")
        
        while self.running:
            try:
                # Check for keyboard input with timeout
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = self.getch()
                    if key is None:
                        continue
                    
                    # Convert to lowercase and handle
                    key = key.lower()
                    print(f"[KEYBOARD] Key pressed: '{key}'")
                    
                    # Handle all keys, not just WASD
                    self.handle_key(key)
                else:
                    # Check for timeout (5 seconds) - but don't timeout if in navigation charging mode
                    if self.manual_mode and not self.nav_charging_mode and (time.time() - self.last_input_time > 5.0):
                        self.manual_mode = False
                        # Only resume primary control if not in navigation charging mode
                        if not self.nav_charging_mode:
                            self.primary_active = True
                            self.print_status('Primary mode')
                            
            except KeyboardInterrupt:
                print("[KEYBOARD] Keyboard interrupt received in keyboard listener")
                break
            except Exception as e:
                print(f"[KEYBOARD] Error in keyboard listener: {str(e)}")
                # Don't break, just continue
                time.sleep(0.1)
                continue
                
            time.sleep(0.05)
        
        print("[KEYBOARD] Keyboard listener thread stopped")

    def handle_key(self, key):
        """Handle keyboard input with improved error handling"""
        try:
            # Set manual mode and update last input time for any key press
            self.manual_mode = True
            self.last_input_time = time.time()
            
            # Unpause robot if it was paused by MQTT
            if self.mqtt_paused:
                self.mqtt_paused = False
                self.primary_active = True  # Resume primary control
                self.print_status("Robot unpaused by keyboard input")
            
            # Handle navigation charging mode toggle
            if key == 't':
                if not self.nav_charging_mode:
                    self.nav_charging_mode = True
                    self.battery_low = True
                    # Disable manual mode and other modes that might interfere
                    self.manual_mode = False  # Prevent manual mode timeout from interfering
                    self.primary_active = False  # Disable primary control during navigation charging
                    self.secondary_control_active = False  # Disable secondary control
                    self.r_mode = False  # Disable R-mode
                    self.print_status("BATTERY LOW! Navigating to charging station (0,0)")
                else:
                    self.nav_charging_mode = False
                    self.battery_low = False
                    # Resume primary control when exiting navigation charging
                    self.primary_active = True
                    self.print_status("Charging navigation cancelled, resuming normal operation")
                return
            
            # Handle R mode toggle
            if key == 'r':
                if not self.mqtt_paused:  # Only allow R mode if not paused
                    self.r_mode = not self.r_mode
                    if self.r_mode:
                        self.virtual_path_index = self.find_closest_waypoint()
                        self.specific_waypoint_navigation = False  # Normal R-mode, not specific waypoint navigation
                        self.print_status(f"Entering R mode - Starting from waypoint {self.virtual_path_index}")
                        # Completely disable all other modes
                        self.secondary_control_active = False
                        self.recovery_active = False
                        self.waypoint_mode = False
                        self.line_following_mode = False
                        self.primary_active = False
                        self.secondary_active = False
                        self.obstacle_detected = False  # Reset obstacle detection
                        self.consecutive_turns = 0  # Reset consecutive turns
                        self.manual_mode = False  # Prevent any mode switching
                        self.status = f"R-mode active - Waypoint {self.virtual_path_index}"
                    else:
                        self.print_status("Exiting R mode - Resuming primary control")
                        # Resume primary control
                        self.primary_active = True
                        self.specific_waypoint_navigation = False
                        self.status = 'Primary mode'
                return
            
            # Handle waypoint navigation
            if key == 'i':
                # Go to waypoint 0
                self.virtual_path_index = 0
                self.r_mode = True
                self.specific_waypoint_navigation = True
                self.primary_active = False
                self.secondary_control_active = False
                self.manual_mode = False
                self.print_status("Navigating to waypoint 0 (Start point)")
            elif key == 'q':
                # Go to waypoint 5
                self.virtual_path_index = 5
                self.r_mode = True
                self.specific_waypoint_navigation = True
                self.primary_active = False
                self.secondary_control_active = False
                self.manual_mode = False
                self.print_status("Navigating to waypoint 5 (MID LEFT)")
            elif key == 'p':
                # Go to waypoint 2
                self.virtual_path_index = 2
                self.r_mode = True
                self.specific_waypoint_navigation = True
                self.primary_active = False
                self.secondary_control_active = False
                self.manual_mode = False
                self.print_status("Navigating to waypoint 2 (Bottom-right point)")
                
            # Handle manual control keys
            if key == 'w':
                self.send_command_to_robot(3.5, 0.0, "Manual: Moving forward")
            elif key == 's':
                self.send_command_to_robot(-3.5, 0.0, "Manual: Moving backward")
                self.print_status("Manual: Moving backward (negative linear velocity)")
            elif key == 'a':
                self.send_command_to_robot(0.0, 3.5, "Manual: Turning left")
            elif key == 'd':
                self.send_command_to_robot(0.0, -3.5, "Manual: Turning right")
            else:
                # Unknown key - just log it
                self.print_status(f"Unknown key pressed: '{key}'")
                
        except Exception as e:
            print(f"[KEYBOARD] Error handling key '{key}': {str(e)}")
            # Don't let keyboard errors crash the program
            self.print_status(f"Error handling key: {str(e)}")

    def scan_callback(self, msg):
        """Process LiDAR data from Gazebo simulation"""
        ranges = msg.ranges
        n = len(ranges)
        window = 10

        # FRONT (benar-benar depan robot, angle 0.0)
        front_angle = 0.0
        idx_front = int((front_angle - msg.angle_min) / msg.angle_increment)
        front_start = max(0, idx_front - window)
        front_end = min(n, idx_front + window)
        front_ranges = ranges[front_start:front_end]
        valid_front = [r for r in front_ranges if r > 0.01 and r < msg.range_max]
        gazebo_front_min = min(valid_front) if valid_front else float('inf')

        # LEFT (90 derajat dari depan, angle +pi/2)
        left_angle = math.pi / 2
        idx_left = int((left_angle - msg.angle_min) / msg.angle_increment)
        left_start = max(0, idx_left - window)
        left_end = min(n, idx_left + window)
        left_ranges = ranges[left_start:left_end]
        valid_left = [r for r in left_ranges if r > 0.01 and r < msg.range_max]
        gazebo_left_min = min(valid_left) if valid_left else float('inf')

        # RIGHT (90 derajat ke kanan dari depan, angle -pi/2)
        right_angle = -math.pi / 2
        idx_right = int((right_angle - msg.angle_min) / msg.angle_increment)
        right_start = max(0, idx_right - window)
        right_end = min(n, idx_right + window)
        right_ranges = ranges[right_start:right_end]
        valid_right = [r for r in right_ranges if r > 0.01 and r < msg.range_max]
        gazebo_right_min = min(valid_right) if valid_right else float('inf')

        # Store Gazebo data
        self.gazebo_front_min = gazebo_front_min
        self.gazebo_left_min = gazebo_left_min
        self.gazebo_right_min = gazebo_right_min

        # Combine with real-world LiDAR data (if available)
        self.update_combined_obstacle_data()

    def update_combined_obstacle_data(self):
        """Use OR logic: if either Gazebo OR real-world LiDAR detects obstacle, avoid it"""
        # Get Gazebo data
        gazebo_front_min = getattr(self, 'gazebo_front_min', float('inf'))
        gazebo_left_min = getattr(self, 'gazebo_left_min', float('inf'))
        gazebo_right_min = getattr(self, 'gazebo_right_min', float('inf'))
        
        # Get real-world LiDAR data
        real_world_front_min = getattr(self, 'real_world_front_min', float('inf'))
        real_world_left_min = getattr(self, 'real_world_left_min', float('inf'))
        real_world_right_min = getattr(self, 'real_world_right_min', float('inf'))
        
        # Use OR logic: if either source detects obstacle, use the closer one
        # Front: use the closer distance between Gazebo and real-world
        if gazebo_front_min != float('inf') and real_world_front_min != float('inf'):
            # Both have data, use the closer one
            if real_world_front_min < gazebo_front_min:
                front_min = real_world_front_min
                front_source = "Real-World LiDAR"
            else:
                front_min = gazebo_front_min
                front_source = "Gazebo"
        elif gazebo_front_min != float('inf'):
            # Only Gazebo has data
            front_min = gazebo_front_min
            front_source = "Gazebo"
        elif real_world_front_min != float('inf'):
            # Only real-world has data
            front_min = real_world_front_min
            front_source = "Real-World LiDAR"
        else:
            # No data from either source
            front_min = float('inf')
            front_source = "None"
        
        # Left: use the closer distance between Gazebo and real-world
        if gazebo_left_min != float('inf') and real_world_left_min != float('inf'):
            if real_world_left_min < gazebo_left_min:
                left_min = real_world_left_min
                left_source = "Real-World LiDAR"
            else:
                left_min = gazebo_left_min
                left_source = "Gazebo"
        elif gazebo_left_min != float('inf'):
            left_min = gazebo_left_min
            left_source = "Gazebo"
        elif real_world_left_min != float('inf'):
            left_min = real_world_left_min
            left_source = "Real-World LiDAR"
        else:
            left_min = float('inf')
            left_source = "None"
        
        # Right: use the closer distance between Gazebo and real-world
        if gazebo_right_min != float('inf') and real_world_right_min != float('inf'):
            if real_world_right_min < gazebo_right_min:
                right_min = real_world_right_min
                right_source = "Real-World LiDAR"
            else:
                right_min = gazebo_right_min
                right_source = "Gazebo"
        elif gazebo_right_min != float('inf'):
            right_min = gazebo_right_min
            right_source = "Gazebo"
        elif real_world_right_min != float('inf'):
            right_min = real_world_right_min
            right_source = "Real-World LiDAR"
        else:
            right_min = float('inf')
            right_source = "None"
        
        # Update combined obstacle data
        self.lidar_min_distance = front_min
        self.left_min = left_min
        self.right_min = right_min
        
        # Store sources for logging
        self.front_source = front_source
        self.left_source = left_source
        self.right_source = right_source
        
        # --- THRESHOLDS: DITURUNKAN untuk emergency stop yang lebih responsif ---
        self.obstacle_detected = self.lidar_min_distance < 1.5  # Dinaikkan dari 0.9 ke 1.5
        self.lidar_too_close = self.lidar_min_distance < 0.25  # DITURUNKAN dari 0.4 ke 0.25 untuk emergency stop yang lebih responsif
        self.left_obstacle = self.left_min < 0.6  # Dinaikkan dari 0.3 ke 0.6
        self.right_obstacle = self.right_min < 0.6  # Dinaikkan dari 0.3 ke 0.6
        self.front_obstacle_close = self.lidar_min_distance < 1.0  # Dinaikkan dari 0.5 ke 1.0
        self.front_obstacle_warning = self.lidar_min_distance < 1.3  # Dinaikkan dari 0.8 ke 1.3
        self.side_obstacle_close = min(self.left_min, self.right_min) < 0.5  # Dinaikkan dari 0.2 ke 0.5
        self.both_sides_blocked = self.left_min < 0.6 and self.right_min < 0.6  # Dinaikkan dari 0.3 ke 0.6
        # ... existing code ...
        # Batasi log dual lidar hanya setiap 2 detik
        current_time = time.time()
        if not hasattr(self, 'last_combined_log_time'):
            self.last_combined_log_time = 0
        if current_time - self.last_combined_log_time > 2.0:
            # Only log if we have valid data (not all inf)
            has_valid_data = False
            gazebo_data = []
            real_world_data = []
            
            # Check Gazebo data
            if gazebo_front_min != float('inf'):
                gazebo_data.append(f"Front={gazebo_front_min:.2f}m")
                has_valid_data = True
            if gazebo_left_min != float('inf'):
                gazebo_data.append(f"Left={gazebo_left_min:.2f}m")
                has_valid_data = True
            if gazebo_right_min != float('inf'):
                gazebo_data.append(f"Right={gazebo_right_min:.2f}m")
                has_valid_data = True
            
            # Check Real-World data
            if real_world_front_min != float('inf'):
                real_world_data.append(f"Front={real_world_front_min:.2f}m")
                has_valid_data = True
            if real_world_left_min != float('inf'):
                real_world_data.append(f"Left={real_world_left_min:.2f}m")
                has_valid_data = True
            if real_world_right_min != float('inf'):
                real_world_data.append(f"Right={real_world_right_min:.2f}m")
                has_valid_data = True
            
            # Only log if we have valid data
            if has_valid_data:
                gazebo_str = f"Gazebo: {', '.join(gazebo_data)}" if gazebo_data else "Gazebo: No data"
                real_world_str = f"Real-World: {', '.join(real_world_data)}" if real_world_data else "Real-World: No data"
                self.print_status(f"📡 {gazebo_str} | {real_world_str}")
            
            self.last_combined_log_time = current_time
        # ... existing code ...

    def log_obstacle_detection(self):
        """Log obstacle detection with source and distance information"""
        current_time = time.time()
        
        # Log every 2 seconds to avoid spam
        if not hasattr(self, 'last_obstacle_log_time'):
            self.last_obstacle_log_time = 0
        
        if current_time - self.last_obstacle_log_time > 2.0:
            # Enhanced logging with dual LiDAR information
            has_real_world_data = (hasattr(self, 'real_world_front_min') and self.real_world_front_min != float('inf')) or \
                                 (hasattr(self, 'real_world_left_min') and self.real_world_left_min != float('inf')) or \
                                 (hasattr(self, 'real_world_right_min') and self.real_world_right_min != float('inf'))
            
            has_gazebo_data = (getattr(self, 'gazebo_front_min', float('inf')) != float('inf')) or \
                             (getattr(self, 'gazebo_left_min', float('inf')) != float('inf')) or \
                             (getattr(self, 'gazebo_right_min', float('inf')) != float('inf'))
            
            # Front obstacle detection
            if self.obstacle_detected:
                if self.lidar_too_close:
                    self.print_warning(f"⚠️ FRONT OBSTACLE TOO CLOSE: {self.lidar_min_distance:.2f}m ({self.front_source})")
                elif self.front_obstacle_close:
                    self.print_warning(f"🚧 CLOSE FRONT OBSTACLE: {self.lidar_min_distance:.2f}m ({self.front_source})")
                else:
                    self.print_status(f"🚧 Front obstacle detected: {self.lidar_min_distance:.2f}m ({self.front_source})")
            
            # Side obstacles
            if self.left_obstacle:
                self.print_status(f"🚧 Left obstacle: {self.left_min:.2f}m ({self.left_source})")
            if self.right_obstacle:
                self.print_status(f"🚧 Right obstacle: {self.right_min:.2f}m ({self.right_source})")
            
            # Both sides blocked
            if self.both_sides_blocked:
                self.print_warning(f"🚧 BOTH SIDES BLOCKED: Left={self.left_min:.2f}m ({self.left_source}), Right={self.right_min:.2f}m ({self.right_source})")
            
            # No obstacles
            if not self.obstacle_detected and not self.left_obstacle and not self.right_obstacle:
                self.print_status(f"✅ Clear path - Front: {self.lidar_min_distance:.2f}m ({self.front_source})")
            
            # Dual LiDAR status summary
            if has_real_world_data and has_gazebo_data:
                self.print_status(f"📡 DUAL LIDAR ACTIVE: Gazebo + Real-World LiDAR (OR Logic: if either detects obstacle → avoid)")
            elif has_real_world_data:
                self.print_status(f"📡 REAL-WORLD LIDAR ACTIVE: Using real-world LiDAR data for obstacle detection")
            elif has_gazebo_data:
                self.print_status(f"📡 GAZEBO LIDAR ACTIVE: Using Gazebo simulation data for obstacle detection")
            else:
                self.print_status(f"📡 NO LIDAR DATA: Neither Gazebo nor real-world LiDAR providing data")
            
            # Show which data source is being used for each direction
            gazebo_front = getattr(self, 'gazebo_front_min', float('inf'))
            real_front = getattr(self, 'real_world_front_min', float('inf'))
            gazebo_left = getattr(self, 'gazebo_left_min', float('inf'))
            real_left = getattr(self, 'real_world_left_min', float('inf'))
            gazebo_right = getattr(self, 'gazebo_right_min', float('inf'))
            real_right = getattr(self, 'real_world_right_min', float('inf'))
            
            # Show which data sources are being used (closer distance wins)
            sources_used = []
            if real_front != float('inf') and (gazebo_front == float('inf') or real_front < gazebo_front):
                sources_used.append("Front: Real-World")
            elif gazebo_front != float('inf'):
                sources_used.append("Front: Gazebo")
                
            if real_left != float('inf') and (gazebo_left == float('inf') or real_left < gazebo_left):
                sources_used.append("Left: Real-World")
            elif gazebo_left != float('inf'):
                sources_used.append("Left: Gazebo")
                
            if real_right != float('inf') and (gazebo_right == float('inf') or real_right < gazebo_right):
                sources_used.append("Right: Real-World")
            elif gazebo_right != float('inf'):
                sources_used.append("Right: Gazebo")
            
            if sources_used:
                self.print_status(f"📡 Active sources: {', '.join(sources_used)}")
            
            self.last_obstacle_log_time = current_time

    def odom_callback(self, msg):
        self.last_odom_pos = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )
        self.last_odom_orientation = msg.pose.pose.orientation
        
        # Check if robot is stuck
        if self.last_position is not None:
            distance = ((self.last_position[0] - self.last_odom_pos[0])**2 + 
                       (self.last_position[1] - self.last_odom_pos[1])**2)**0.5
            if distance < self.position_threshold:
                self.stuck_time += 0.1  # Timer callback is 10Hz
                if self.stuck_time >= self.stuck_threshold and not self.recovery_active and not self.secondary_control_active:
                    self.recovery_active = True
                    self.recovery_start_time = time.time()
                    self.print_status("Robot appears stuck! Starting recovery...")
            else:
                self.stuck_time = 0
                # Reset recovery if robot is moving
                if self.recovery_active:
                    self.recovery_active = False
                    self.print_status("Robot is moving, canceling recovery")
        
        self.last_position = self.last_odom_pos

    def avoid_obstacle(self):
        """Enhanced obstacle avoidance behavior using combined Gazebo and real-world LiDAR data"""
        if not hasattr(self, 'lidar_min_distance'):
            return None
            
        twist = Twist()
        
        # Use combined obstacle data (Gazebo + Real-World LiDAR)
        front_min = self.lidar_min_distance
        left_min = self.left_min
        right_min = self.right_min
        
        # Get sources for logging
        front_source = getattr(self, 'front_source', 'Unknown')
        left_source = getattr(self, 'left_source', 'Unknown')
        right_source = getattr(self, 'right_source', 'Unknown')
        
        # If both sides are blocked, do a 360 scan
        if left_min < 0.3 and right_min < 0.3:  # Reduced from 0.5 to 0.3
            self.is_scanning = True
            self.scan_start_time = time.time()
            self.print_status(f'Both sides blocked, starting 360° scan... Left: {left_min:.2f}m ({left_source}), Right: {right_min:.2f}m ({right_source})')
            twist.linear.x = 0.0
            twist.angular.z = 2.0  # Increased turn speed
            return twist
            
        # If obstacle is on the left, turn right
        if left_min < 0.4:  # Reduced from obstacle_threshold to 0.4
            twist.linear.x = 0.0
            twist.angular.z = -2.0  # Increased turn speed
            self.print_status(f'Obstacle on left, turning right. Distance: {left_min:.2f}m ({left_source})')
        # If obstacle is on the right, turn left
        elif right_min < 0.4:  # Reduced from obstacle_threshold to 0.4
            twist.linear.x = 0.0
            twist.angular.z = 2.0  # Increased turn speed
            self.print_status(f'Obstacle on right, turning left. Distance: {right_min:.2f}m ({right_source})')
        # If front obstacle detected
        elif front_min < 1.0:
            twist.linear.x = 0.0
            twist.angular.z = 1.5  # Turn to avoid front obstacle
            self.print_status(f'Front obstacle detected, turning. Distance: {front_min:.2f}m ({front_source})')
        else:
            # No obstacles detected, move forward
            twist.linear.x = 2.0  # Increased forward speed
            twist.angular.z = 0.0
            self.print_status(f'No obstacles, moving forward. Front: {front_min:.2f}m ({front_source})')
            
        return twist

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def reset_primary_control(self):
        """Reset primary control state to initial values"""
        self.primary_active = True
        self.consecutive_turns = 0
        self.is_backing_up = False
        self.last_turn_direction = None
        self.last_turn_time = 0.0
        self.turn_start_time = None
        self.send_command_to_robot(1.6, 0.0, "Primary control reset")
        self.print_status("Primary control reset to initial state")

    def send_command_to_robot(self, linear_x, angular_z, mode_name=""):
        """Send command to both ROS and physical robot via MQTT with improved error handling"""
        try:
            # Create Twist message for ROS
            twist = Twist()
            twist.linear.x = linear_x
            twist.angular.z = angular_z
            
            # Publish to ROS
            try:
                self.cmd_vel_pub.publish(twist)
            except Exception as e:
                print(f"[ROS] Error publishing to cmd_vel: {str(e)}")
                # Don't let ROS errors crash the program
            
            # Send to physical robot via MQTT
            if self.mqtt_client is not None:
                try:
                    cmd_data = {
                        'linear': {
                            'x': linear_x,
                            'y': 0.0,
                            'z': 0.0
                        },
                        'angular': {
                            'x': 0.0,
                            'y': 0.0,
                            'z': angular_z
                        },
                        'mode': mode_name
                    }
                    self.mqtt_client.publish("robot/physical/cmd_vel", json.dumps(cmd_data))
                    if mode_name:
                        self.print_status(f"Command sent to physical robot: {mode_name}")
                except Exception as e:
                    print(f"[MQTT] Error sending command to physical robot: {str(e)}")
                    # Don't let MQTT errors crash the program
                    
        except Exception as e:
            print(f"[COMMAND] Error in send_command_to_robot: {str(e)}")
            # Don't let command errors crash the program

    def timer_callback(self):
        # EMERGENCY STOP CHECK - Highest priority for all modes
        if hasattr(self, 'lidar_min_distance') and self.lidar_min_distance < 0.25:
            # Emergency stop regardless of mode
            self.send_command_to_robot(0.0, 0.0, "EMERGENCY STOP - OBSTACLE TOO CLOSE")
            self.print_warning(f"🚨 EMERGENCY STOP: Front obstacle too close: {self.lidar_min_distance:.2f}m ({getattr(self, 'front_source', 'Unknown')})")
            return
        
        # Check if robot is paused by MQTT (but allow charging navigation to bypass this)
        if self.mqtt_paused and not self.nav_charging_mode:
            # Ensure robot stays stopped while paused
            self.send_command_to_robot(0.0, 0.0, "ROBOT IDLE")
            return

        # Navigation charging mode takes highest priority (even over MQTT pause)
        if self.nav_charging_mode:
            self.status = "BATTERY LOW! Navigating to charging station"
            self.print_status(self.status)
            
            # Get current position
            if self.last_odom_pos is not None:
                current_pos = (self.last_odom_pos[0], self.last_odom_pos[1])
                dx = self.charging_station_pos[0] - current_pos[0]
                dy = self.charging_station_pos[1] - current_pos[1]
                distance = math.sqrt(dx*dx + dy*dy)
                
                # Get robot's current orientation from odometry
                if hasattr(self, 'last_odom_orientation'):
                    # Convert quaternion to euler angles
                    qx = self.last_odom_orientation.x
                    qy = self.last_odom_orientation.y
                    qz = self.last_odom_orientation.z
                    qw = self.last_odom_orientation.w
                    
                    # Calculate yaw (rotation around z-axis)
                    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
                    
                    # Calculate target angle relative to world frame
                    target_angle = math.atan2(dy, dx)
                    
                    # Calculate angle difference relative to robot's current orientation
                    angle_diff = self.normalize_angle(target_angle - yaw)
                    
                    # Check if we've reached the charging station
                    if distance < self.charging_threshold:
                        self.nav_charging_mode = False
                        self.battery_low = False
                        self.print_status("Reached charging station! Stopping for charging")
                        # Stop the robot
                        self.send_command_to_robot(0.0, 0.0, "ROBOT STOPPED")
                        return
                    
                    # Check for obstacles during charging navigation using combined data
                    charging_obstacle_detected = self.lidar_min_distance < self.charging_obstacle_threshold
                    charging_lidar_too_close = self.lidar_min_distance < self.charging_emergency_stop_threshold
                    
                    if charging_obstacle_detected:
                        if charging_lidar_too_close:
                            # Emergency stop
                            self.send_command_to_robot(0.0, 0.0, "CHARGING NAV: EMERGENCY STOP")
                            self.print_status(f"CHARGING NAV: Emergency stop - Front obstacle too close: {self.lidar_min_distance:.2f}m ({self.front_source})")
                        else:
                            # Avoid obstacle while maintaining direction to charging station
                            if self.left_min > self.right_min:  # More space on left
                                self.send_command_to_robot(0.0, self.charging_turn_speed, "CHARGING NAV: Avoiding obstacle left")
                                self.print_status(f"CHARGING NAV: Avoiding obstacle left. Front: {self.lidar_min_distance:.2f}m ({self.front_source}), Left: {self.left_min:.2f}m ({self.left_source})")
                            else:  # More space on right
                                self.send_command_to_robot(0.0, -self.charging_turn_speed, "CHARGING NAV: Avoiding obstacle right")
                                self.print_status(f"CHARGING NAV: Avoiding obstacle right. Front: {self.lidar_min_distance:.2f}m ({self.front_source}), Right: {self.right_min:.2f}m ({self.right_source})")
                    else:
                        # No obstacles, navigate to charging station
                        if abs(angle_diff) > self.charging_angle_threshold:
                            # Need to rotate first
                            self.send_command_to_robot(0.0, self.charging_turn_speed * (1.0 if angle_diff > 0 else -1.0), "CHARGING NAV: Rotating to face charging station")
                            self.print_status(f"CHARGING NAV: Rotating to face charging station. Angle diff: {angle_diff:.2f} rad")
                        else:
                            # Move forward towards charging station
                            self.send_command_to_robot(self.charging_forward_speed, angle_diff * 0.5, "CHARGING NAV: Moving to charging station")
                            self.print_status(f"CHARGING NAV: Moving to charging station. Distance: {distance:.2f}m, Front: {self.lidar_min_distance:.2f}m ({self.front_source})")
            return

        # If in R mode, only run R-mode logic
        if self.r_mode:
            self.status = f"R-mode active - Waypoint {self.virtual_path_index}"
            self.print_status(self.status)
            
            # Enhanced obstacle avoidance for R-mode using combined data
            if self.obstacle_detected:
                if self.lidar_too_close:
                    # Emergency stop for R-mode
                    self.send_command_to_robot(0.0, 0.0, "R-MODE: EMERGENCY STOP")
                    self.print_status(f"R-MODE: Emergency stop - Front obstacle too close: {self.lidar_min_distance:.2f}m ({self.front_source})")
                    return
                elif self.front_obstacle_close:
                    # Close front obstacle - turn to avoid
                    if self.left_min > self.right_min:
                        self.send_command_to_robot(0.0, self.r_mode_turn_speed, "R-MODE: Avoiding front obstacle (left)")
                        self.print_status(f"R-MODE: Avoiding front obstacle - turning left. Front: {self.lidar_min_distance:.2f}m ({self.front_source})")
                    else:
                        self.send_command_to_robot(0.0, -self.r_mode_turn_speed, "R-MODE: Avoiding front obstacle (right)")
                        self.print_status(f"R-MODE: Avoiding front obstacle - turning right. Front: {self.lidar_min_distance:.2f}m ({self.front_source})")
                    return
                elif self.both_sides_blocked:
                    # Both sides blocked - scan 360 degrees
                    self.send_command_to_robot(0.0, self.r_mode_turn_speed, "R-MODE: Both sides blocked - scanning")
                    self.print_status(f"R-MODE: Both sides blocked - scanning. Left: {self.left_min:.2f}m ({self.left_source}), Right: {self.right_min:.2f}m ({self.right_source})")
                    return
                elif self.left_obstacle:
                    # Left side blocked - turn right
                    self.send_command_to_robot(0.0, -self.r_mode_turn_speed, "R-MODE: Left side blocked")
                    self.print_status(f"R-MODE: Left side blocked - turning right. Distance: {self.left_min:.2f}m ({self.left_source})")
                    return
                elif self.right_obstacle:
                    # Right side blocked - turn left
                    self.send_command_to_robot(0.0, self.r_mode_turn_speed, "R-MODE: Right side blocked")
                    self.print_status(f"R-MODE: Right side blocked - turning left. Distance: {self.right_min:.2f}m ({self.right_source})")
                    return
            
            # Get current waypoint
            if self.virtual_path_index < len(self.virtual_path):
                target_waypoint = self.virtual_path[self.virtual_path_index]
                
                # Calculate distance and angle to waypoint
                if self.last_odom_pos is not None:
                    current_pos = (self.last_odom_pos[0], self.last_odom_pos[1])
                    dx = target_waypoint[0] - current_pos[0]
                    dy = target_waypoint[1] - current_pos[1]
                    distance = math.sqrt(dx*dx + dy*dy)
                    
                    # Get robot's current orientation from odometry
                    if hasattr(self, 'last_odom_orientation'):
                        # Convert quaternion to euler angles
                        qx = self.last_odom_orientation.x
                        qy = self.last_odom_orientation.y
                        qz = self.last_odom_orientation.z
                        qw = self.last_odom_orientation.w
                        
                        # Calculate yaw (rotation around z-axis)
                        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
                        
                        # Calculate target angle relative to world frame
                        target_angle = math.atan2(dy, dx)
                        
                        # Calculate angle difference relative to robot's current orientation
                        angle_diff = self.normalize_angle(target_angle - yaw)
                        
                        # If we're close enough to current waypoint
                        if distance < self.r_mode_waypoint_threshold:
                            if self.specific_waypoint_navigation:
                                # For specific waypoint navigation (i, u, o), stop at the waypoint
                                waypoint_names = {
                                    0: "Start point",
                                    2: "Bottom-right point", 
                                    5: "MID LEFT"
                                }
                                waypoint_name = waypoint_names.get(self.virtual_path_index, f"Waypoint {self.virtual_path_index}")
                                self.send_command_to_robot(0.0, 0.0, f"Reached target waypoint {self.virtual_path_index}")
                                self.print_status(f"✓ Reached target waypoint {self.virtual_path_index} ({waypoint_name}), stopping and resuming primary control")
                                # Reset to primary control
                                self.r_mode = False
                                self.specific_waypoint_navigation = False
                                self.primary_active = True
                                return
                            else:
                                # For normal R-mode, continue to next waypoint
                                self.virtual_path_index = (self.virtual_path_index + 1) % len(self.virtual_path)
                                self.print_status(f"Reached waypoint {self.virtual_path_index-1}, moving to next waypoint")
                                return
                        
                        # Move towards waypoint with obstacle awareness
                        if abs(angle_diff) > self.r_mode_angle_threshold:
                            # Need to rotate first
                            self.send_command_to_robot(0.0, self.r_mode_turn_speed * (1.0 if angle_diff > 0 else -1.0), f"Rotating to face waypoint {self.virtual_path_index}")
                        else:
                            # Move forward towards waypoint with reduced speed if obstacles nearby
                            forward_speed = self.r_mode_forward_speed
                            if self.front_obstacle_warning:
                                forward_speed *= 0.5  # Reduce speed when obstacle is nearby
                                self.print_status(f"R-MODE: Reduced speed due to nearby obstacle. Front: {self.lidar_min_distance:.2f}m ({self.front_source})")
                            
                            self.send_command_to_robot(forward_speed, angle_diff * 0.5, f"Moving towards waypoint {self.virtual_path_index}")
            return

        # Manual mode takes highest priority (but not over navigation charging)
        if self.manual_mode and not self.nav_charging_mode:
            # Primary control is paused during manual mode
            self.primary_active = False
            self.secondary_control_active = False
            return

        # If we just exited R-mode or manual mode, reset primary control (but not if in navigation charging)
        if not self.primary_active and not self.secondary_control_active and not self.nav_charging_mode:
            self.reset_primary_control()
            return

        # Secondary control takes second priority
        if self.secondary_control_active:
            # Primary control is paused during secondary mode
            self.primary_active = False
            
            # Check if secondary mode should end (40 seconds timeout)
            if time.time() - self.secondary_start_time >= 40.0:
                self.secondary_control_active = False
                self.print_status("Secondary control ended, resuming primary control")
                self.reset_primary_control()
                return

            # Enhanced secondary control logic using combined obstacle data
            if self.left_obstacle and self.right_obstacle:
                self.print_status(f"Secondary: Both sides blocked! Front: {self.lidar_min_distance:.2f}m ({self.front_source})")
                self.print_status(f"Secondary: Left: {self.left_min:.2f}m ({self.left_source}), Right: {self.right_min:.2f}m ({self.right_source})")
                self.send_command_to_robot(0.0, 2.0, "Secondary: Both sides blocked")
            elif self.left_obstacle:
                self.print_status(f"Secondary: Left side blocked! Front: {self.lidar_min_distance:.2f}m ({self.front_source})")
                self.print_status(f"Secondary: Left distance: {self.left_min:.2f}m ({self.left_source})")
                self.send_command_to_robot(0.0, -2.0, "Secondary: Left side blocked")
            elif self.right_obstacle:
                self.print_status(f"Secondary: Right side blocked! Front: {self.lidar_min_distance:.2f}m ({self.front_source})")
                self.print_status(f"Secondary: Right distance: {self.right_min:.2f}m ({self.right_source})")
                self.send_command_to_robot(0.0, 2.0, "Secondary: Right side blocked")
            elif self.front_obstacle_close:
                self.print_status(f"Secondary: Front obstacle detected! Distance: {self.lidar_min_distance:.2f}m ({self.front_source})")
                if self.left_min > self.right_min:
                    self.send_command_to_robot(0.0, 1.5, "Secondary: Avoiding front obstacle (left)")
                else:
                    self.send_command_to_robot(0.0, -1.5, "Secondary: Avoiding front obstacle (right)")
            else:
                self.print_status(f"Secondary: Scanning for clear path. Front: {self.lidar_min_distance:.2f}m ({self.front_source})")
                self.send_command_to_robot(0.0, 1.0, "Secondary: Scanning for clear path")
            return

        # Primary control (only active when not in manual or secondary mode)
        if self.primary_active:
            current_time = time.time()
            
            # Enhanced primary control with combined obstacle data
            if self.obstacle_detected:
                if self.lidar_too_close:
                    # Emergency stop
                    self.send_command_to_robot(0.0, 0.0, "EMERGENCY STOP")
                    self.print_status(f"Primary: Emergency stop - Front obstacle too close: {self.lidar_min_distance:.2f}m ({self.front_source})")
                else:
                    # Check if both sides are blocked
                    if self.both_sides_blocked:
                        # If stuck in turning loop, try backing up
                        if self.consecutive_turns >= self.max_consecutive_turns:
                            if not self.is_backing_up:
                                self.is_backing_up = True
                                self.backup_start_time = time.time()
                                self.consecutive_turns = 0
                                self.print_status(f"Primary: Too many consecutive turns, backing up... Front: {self.lidar_min_distance:.2f}m ({self.front_source})")
                            
                            if self.is_backing_up:
                                if time.time() - self.backup_start_time < self.backup_duration:
                                    self.send_command_to_robot(-1.0, 0.0, "Backing up")
                                else:
                                    self.is_backing_up = False
                                    # After backing up, turn in a random direction with more space
                                    if self.left_min > self.right_min:
                                        turn_direction = 1.0  # Turn left
                                        self.print_status(f"Primary: Backing up complete - turning left. Left: {self.left_min:.2f}m ({self.left_source})")
                                    else:
                                        turn_direction = -1.0  # Turn right
                                        self.print_status(f"Primary: Backing up complete - turning right. Right: {self.right_min:.2f}m ({self.right_source})")
                                    self.send_command_to_robot(0.0, turn_direction * 2.0, "Backing up complete")
                        else:
                            # Choose direction with more space and keep turning until front is clear
                            if self.left_min > self.right_min:  # More space on left
                                if self.lidar_min_distance < 1.5:  # Front not clear yet
                                    # If we're already turning left, continue
                                    if self.last_turn_direction == 'left':
                                        self.send_command_to_robot(0.0, 1.5, "Primary: Continuing left turn")
                                        self.print_status(f"Primary: Continuing left turn. Front: {self.lidar_min_distance:.2f}m ({self.front_source})")
                                    # If we're turning right, wait for cooldown
                                    elif self.last_turn_direction == 'right' and current_time - self.last_turn_time < self.turn_cooldown:
                                        self.send_command_to_robot(0.0, 0.0, "Primary: Waiting for turn cooldown")
                                    # If no turn direction set or cooldown expired, move forward first then turn
                                    else:
                                        # Move forward first
                                        self.send_command_to_robot(1.6, 0.0, "Primary: Moving forward before turning left")
                                        # After a short delay, start turning
                                        if not hasattr(self, 'turn_start_time'):
                                            self.turn_start_time = current_time
                                        elif current_time - self.turn_start_time >= 1.0:  # Wait 1 second before turning
                                            self.send_command_to_robot(0.0, 1.5, "Primary: Starting left turn")
                                            self.print_status(f"Primary: Starting left turn. Left: {self.left_min:.2f}m ({self.left_source})")
                                else:
                                    # Front is clear, can proceed
                                    self.send_command_to_robot(1.6, 0.0, "Primary: Front clear, proceeding forward")
                                    self.print_status(f"Primary: Front clear, proceeding forward. Front: {self.lidar_min_distance:.2f}m ({self.front_source})")
                            else:  # More space on right
                                if self.lidar_min_distance < 1.5:  # Front not clear yet
                                    # If we're already turning right, continue
                                    if self.last_turn_direction == 'right':
                                        self.send_command_to_robot(0.0, -1.5, "Primary: Continuing right turn")
                                        self.print_status(f"Primary: Continuing right turn. Front: {self.lidar_min_distance:.2f}m ({self.front_source})")
                                    # If we're turning left, wait for cooldown
                                    elif self.last_turn_direction == 'left' and current_time - self.last_turn_time < self.turn_cooldown:
                                        self.send_command_to_robot(0.0, 0.0, "Primary: Waiting for turn cooldown")
                                    # If no turn direction set or cooldown expired, move forward first then turn
                                    else:
                                        # Move forward first
                                        self.send_command_to_robot(1.6, 0.0, "Primary: Moving forward before turning right")
                                        # After a short delay, start turning
                                        if not hasattr(self, 'turn_start_time'):
                                            self.turn_start_time = current_time
                                        elif current_time - self.turn_start_time >= 1.0:  # Wait 1 second before turning
                                            self.send_command_to_robot(0.0, -1.5, "Primary: Starting right turn")
                                            self.print_status(f"Primary: Starting right turn. Right: {self.right_min:.2f}m ({self.right_source})")
                                else:
                                    # Front is clear, can proceed
                                    self.send_command_to_robot(1.6, 0.0, "Primary: Front clear, proceeding forward")
                                    self.print_status(f"Primary: Front clear, proceeding forward. Front: {self.lidar_min_distance:.2f}m ({self.front_source})")
                    else:
                        # Choose direction with more space and reset consecutive turns if successful
                        if self.left_min > self.right_min:  # More space on left
                            # If we're already turning left, continue
                            if self.last_turn_direction == 'left':
                                self.send_command_to_robot(0.0, 1.5, "Primary: Continuing left turn")
                                self.print_status(f"Primary: Continuing left turn. Left: {self.left_min:.2f}m ({self.left_source})")
                            # If we're turning right, wait for cooldown
                            elif self.last_turn_direction == 'right' and current_time - self.last_turn_time < self.turn_cooldown:
                                self.send_command_to_robot(0.0, 0.0, "Primary: Waiting for turn cooldown")
                            # If no turn direction set or cooldown expired, start turning left
                            else:
                                self.send_command_to_robot(0.0, 1.5, "Primary: Starting left turn")
                                self.print_status(f"Primary: Starting left turn. Left: {self.left_min:.2f}m ({self.left_source})")
                        else:  # More space on right
                            # If we're already turning right, continue
                            if self.last_turn_direction == 'right':
                                self.send_command_to_robot(0.0, -1.5, "Primary: Continuing right turn")
                                self.print_status(f"Primary: Continuing right turn. Right: {self.right_min:.2f}m ({self.right_source})")
                            # If we're turning left, wait for cooldown
                            elif self.last_turn_direction == 'left' and current_time - self.last_turn_time < self.turn_cooldown:
                                self.send_command_to_robot(0.0, 0.0, "Primary: Waiting for turn cooldown")
                            # If no turn direction set or cooldown expired, start turning right
                            else:
                                self.send_command_to_robot(0.0, -1.5, "Primary: Starting right turn")
                                self.print_status(f"Primary: Starting right turn. Right: {self.right_min:.2f}m ({self.right_source})")
            else:
                # No obstacles detected, move forward normally
                self.send_command_to_robot(1.6, 0.0, "Primary: Moving forward")
                self.print_status(f"Primary: Moving forward. Front: {self.lidar_min_distance:.2f}m ({self.front_source})")

    def print_instructions(self):
        print('\n' + '='*50)
        print(' '*15 + 'MANUAL CONTROL MODE')
        print('='*50)
        print('W: Move Forward')
        print('S: Move Backward')
        print('A: Turn Left')
        print('D: Turn Right')
        print('T: Toggle Battery Low (Navigate to charging station)')
        print('R: Toggle R-mode (Waypoint navigation)')
        print('I: Navigate to Waypoint 0 (Start point)')
        print('Q: Navigate to Waypoint 5 (MID LEFT)')
        print('P: Navigate to Waypoint 2 (Bottom-right point)')
        print('\nMQTT Commands (via test/topic):')
        print('{"command": "waypoint_i"} - Go to Waypoint 0')
        print('{"command": "waypoint_q"} - Go to Waypoint 5')
        print('{"command": "waypoint_p"} - Go to Waypoint 2')
        print('{"command": "stop"} - Stop robot')
        print('{"command": "start"} - Start robot')
        print('\nPress any WASD key to start manual control...')
        print('Primary mode resumes after 5s of no input')  # Changed from 'Auto-roaming' to 'Primary mode'
        print('Press Ctrl+C to exit')
        print('='*50 + '\n')

    def check_for_obstacles(self):
        if not hasattr(self, 'lidar_min_distance'):
            return False
        if self.obstacle_detected:
            source = getattr(self, 'front_source', 'Unknown')
            self.get_logger().info(f'Obstacle detected at distance: {self.lidar_min_distance:.2f}m ({source})')
            return True
        return False

    def find_best_direction(self):
        """Find the best direction using combined Gazebo and real-world LiDAR data"""
        if not hasattr(self, 'lidar_min_distance'):
            return 0
            
        # Use combined obstacle data to find best direction
        front_min = self.lidar_min_distance
        left_min = self.left_min
        right_min = self.right_min
        
        # Get sources for logging
        front_source = getattr(self, 'front_source', 'Unknown')
        left_source = getattr(self, 'left_source', 'Unknown')
        right_source = getattr(self, 'right_source', 'Unknown')
        
        # Simple direction finding based on available space
        if left_min > right_min and left_min > 1.0:
            # Left has more space
            self.print_status(f'Best direction: LEFT ({left_min:.2f}m {left_source})')
            return 90  # 90 degrees left
        elif right_min > left_min and right_min > 1.0:
            # Right has more space
            self.print_status(f'Best direction: RIGHT ({right_min:.2f}m {right_source})')
            return -90  # 90 degrees right
        elif front_min > 1.5:
            # Front has space
            self.print_status(f'Best direction: FORWARD ({front_min:.2f}m {front_source})')
            return 0  # Forward
        else:
            # All directions blocked, turn around
            self.print_status(f'All directions blocked, turning around. Front: {front_min:.2f}m, Left: {left_min:.2f}m, Right: {right_min:.2f}m')
            return 180  # Turn around

    def check_input(self):
        try:
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                self.handle_key(key)
                self.last_input_time = time.time()
                self.manual_mode = True
                self.is_scanning = False
                self.turning_to_best = False
            elif time.time() - self.last_input_time > 5.0:
                self.manual_mode = False
                # No auto-roaming message, just let the current mode continue
        except Exception as e:
            self.get_logger().error(f'Error in check_input: {str(e)}')

    def cleanup(self):
        """Cleanup resources before shutdown"""
        try:
            # Stop the robot first
            self.send_command_to_robot(0.0, 0.0, "ROBOT STOPPED")
            
            # Disconnect MQTT client if it exists
            if self.mqtt_client is not None:
                try:
                    # Send disconnect message first
                    try:
                        disconnect_message = {
                            "status": "disconnected",
                            "timestamp": time.time(),
                            "message": "roam.py is shutting down",
                            "source": "roam.py"
                        }
                        self.mqtt_client.publish("robot/lidar/status", json.dumps(disconnect_message))
                    except Exception as e:
                        print(f'\nWarning: Could not send disconnect message: {str(e)}')
                    
                    # Stop the loop and disconnect
                    self.mqtt_client.loop_stop()
                    self.mqtt_client.disconnect()
                    self.mqtt_client = None
                    self.mqtt_connected = False
                    print('\nMQTT client disconnected successfully')
                except Exception as e:
                    print(f'\nWarning: Could not disconnect MQTT client: {str(e)}')
            
            # Restore terminal settings
            if hasattr(self, 'old_settings') and self.old_settings is not None:
                try:
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
                except Exception as e:
                    print(f'\nWarning: Could not restore terminal settings: {str(e)}')
            
            # Clear the screen and print final message
            os.system('clear')
            print('\nRobot stopped. Terminal restored.')
            
        except Exception as e:
            print(f'\nError during cleanup: {str(e)}')

    def get_distance_to_waypoint(self):
        if self.last_odom_pos is None or self.current_waypoint is None:
            return float('inf')
            
        target = self.waypoints[self.current_waypoint]
        current = (self.last_odom_pos[0], self.last_odom_pos[1])
        
        return ((target[0] - current[0])**2 + (target[1] - current[1])**2)**0.5
        
    def get_angle_to_waypoint(self):
        if self.last_odom_pos is None or self.current_waypoint is None:
            return 0.0
            
        target = self.waypoints[self.current_waypoint]
        current = (self.last_odom_pos[0], self.last_odom_pos[1])
        
        dx = target[0] - current[0]
        dy = target[1] - current[1]
        
        return math.atan2(dy, dx)
        
    def get_next_line_point(self):
        # Calculate next point on the circular path
        self.line_angle += 0.1  # Increment angle
        if self.line_angle >= 2 * math.pi:
            self.line_angle = 0
            
        x = self.line_center[0] + self.line_radius * math.cos(self.line_angle)
        y = self.line_center[1] + self.line_radius * math.sin(self.line_angle)
        return (x, y)
        
    def get_angle_to_line_point(self, target):
        if self.last_odom_pos is None:
            return 0.0
            
        current = (self.last_odom_pos[0], self.last_odom_pos[1])
        dx = target[0] - current[0]
        dy = target[1] - current[1]
        
        return math.atan2(dy, dx)

    def key_callback(self, key):
        if key == 'r' or key == 'R':
            self.line_following_mode = not self.line_following_mode
            self.waypoint_mode = False
            self.current_waypoint = None
            self.print_status("Line following mode: " + ("ON" if self.line_following_mode else "OFF"))
        elif key == 't' or key == 'T':
            self.waypoint_mode = False
            self.current_waypoint = None
            self.print_status("Waypoint mode disabled, continuing to roam")
        # Old waypoint handling - now handled in handle_key function
        # elif key in ['u', 'U', 'i', 'I', 'o', 'O']:
        #     self.waypoint_mode = True
        #     self.line_following_mode = False
        #     self.current_waypoint = key.upper()
        #     self.print_status(f"Moving to waypoint {self.current_waypoint}")
        # ... existing key handling code ...

    def find_closest_waypoint(self):
        """Find the closest waypoint to current position"""
        if not self.last_odom_pos:
            return 0  # Return first waypoint if no position data
            
        current = (self.last_odom_pos[0], self.last_odom_pos[1])
        min_distance = float('inf')
        closest_index = 0
        
        for i, waypoint in enumerate(self.virtual_path):
            dx = waypoint[0] - current[0]
            dy = waypoint[1] - current[1]
            distance = math.hypot(dx, dy)
            if distance < min_distance:
                min_distance = distance
                closest_index = i
                
        return closest_index

    def status_update_callback(self):
        """Callback to update status display"""
        if hasattr(self, 'last_status'):
            self.print_status(self.last_status)

    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback when MQTT client connects"""
        if rc == 0:
            self.mqtt_connected = True
            self.mqtt_connection_time = time.time()
            self.get_logger().info("Connected to MQTT broker successfully")
            self.print_status("MQTT: Connected to broker")
            
            # Subscribe to topics with QoS 1 for better reliability
            topics = [
                ("robot/physical/scan", 1),  # Untuk menerima data LiDAR dari robot fisik
                ("robot/physical/status", 1),  # Untuk menerima status dari robot fisik
                ("robot/physical/battery", 1),  # Untuk menerima data baterai dari robot fisik
                ("robot/lidar/real_world_data", 1),  # Untuk menerima data LiDAR real-world
                ("robot/lidar/status", 1),  # Untuk menerima status LiDAR real-world
                ("test/topic", 1)  # Subscribe to test topic for commands
            ]
            
            # Subscribe to all topics
            for topic, qos in topics:
                try:
                    client.subscribe(topic, qos)
                    self.get_logger().info(f"Subscribed to {topic} with QoS {qos}")
                except Exception as e:
                    self.get_logger().error(f"Failed to subscribe to {topic}: {str(e)}")
            
            self.print_status("MQTT: All topics subscribed successfully")
            
            # Reset lidar data state when reconnecting
            self.reset_lidar_data_state()
            
            # Send a ready message to indicate roam.py is ready to receive data
            try:
                ready_message = {
                    "status": "ready",
                    "timestamp": time.time(),
                    "message": "roam.py is ready to receive LiDAR data",
                    "source": "roam.py",
                    "reconnect": True  # Indicate this is a reconnection
                }
                client.publish("robot/lidar/status", json.dumps(ready_message))
                self.print_status("MQTT: Sent ready message to LiDAR systems (reconnect)")
            except Exception as e:
                self.get_logger().error(f"Failed to send ready message: {str(e)}")
                
        else:
            self.mqtt_connected = False
            self.get_logger().error(f"Failed to connect to MQTT broker with result code: {rc}")
            self.print_status("MQTT: Connection failed")

    def reset_lidar_data_state(self):
        """Reset lidar data state when reconnecting"""
        try:
            # Reset all lidar data to initial state
            self.real_world_front_min = float('inf')
            self.real_world_left_min = float('inf')
            self.real_world_right_min = float('inf')
            
            self.gazebo_front_min = float('inf')
            self.gazebo_left_min = float('inf')
            self.gazebo_right_min = float('inf')
            
            # Reset combined obstacle data
            self.lidar_min_distance = float('inf')
            self.left_min = float('inf')
            self.right_min = float('inf')
            
            # Reset obstacle flags
            self.obstacle_detected = False
            self.lidar_too_close = False
            self.left_obstacle = False
            self.right_obstacle = False
            self.front_obstacle_close = False
            self.front_obstacle_warning = False
            self.side_obstacle_close = False
            self.both_sides_blocked = False
            
            # Reset data sources
            self.front_source = "None"
            self.left_source = "None"
            self.right_source = "None"
            
            # Reset timing variables
            self.last_real_world_log_time = 0
            self.last_combined_log_time = 0
            self.last_obstacle_log_time = 0
            
            self.get_logger().info("Lidar data state reset successfully")
            self.print_status("MQTT: Lidar data state reset for reconnection")
            
        except Exception as e:
            self.get_logger().error(f"Error resetting lidar data state: {str(e)}")

    def on_mqtt_disconnect(self, client, userdata, rc):
        """Callback when MQTT client disconnects"""
        self.mqtt_connected = False
        self.get_logger().warning(f"MQTT client disconnected with code: {rc}")
        self.print_status("MQTT: Disconnected from broker")
        
        # Try to reconnect if not shutting down
        if self.running and rc != 0:
            self.get_logger().info("Attempting to reconnect to MQTT broker...")
            self.print_status("MQTT: Attempting to reconnect...")
            try:
                # Wait a bit before reconnecting to avoid rapid reconnection attempts
                time.sleep(2.0)
                client.reconnect()
            except Exception as e:
                self.get_logger().error(f"Failed to reconnect: {str(e)}")
                self.print_status("MQTT: Reconnection failed")
                # Try again after a longer delay
                try:
                    time.sleep(5.0)
                    client.reconnect()
                except Exception as e2:
                    self.get_logger().error(f"Second reconnection attempt failed: {str(e2)}")
                    self.print_status("MQTT: Second reconnection attempt failed")
                    # Schedule another reconnection attempt
                    self.schedule_reconnection()

    def schedule_reconnection(self):
        """Schedule another reconnection attempt with exponential backoff"""
        if not self.running or self.mqtt_reconnect_attempts >= self.mqtt_max_reconnect_attempts:
            self.print_status("MQTT: Max reconnection attempts reached, giving up")
            return
        
        self.mqtt_reconnect_attempts += 1
        
        # Calculate delay with exponential backoff (1s, 2s, 4s, 8s, 16s, 32s, 60s, 60s...)
        delay = min(self.mqtt_reconnect_delay * (2 ** (self.mqtt_reconnect_attempts - 1)), 60.0)
        
        self.print_status(f"MQTT: Scheduling reconnection attempt {self.mqtt_reconnect_attempts}/{self.mqtt_max_reconnect_attempts} in {delay:.1f}s")
        
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
            self.get_logger().info(f"Attempting MQTT reconnection (attempt {self.mqtt_reconnect_attempts})")
            self.print_status(f"MQTT: Attempting reconnection (attempt {self.mqtt_reconnect_attempts})")
            
            # Try to reconnect
            self.mqtt_client.reconnect()
            
            # If successful, reset reconnection attempts
            self.mqtt_reconnect_attempts = 0
            self.mqtt_reconnect_delay = 1.0
            self.print_status("MQTT: Reconnection successful")
            
        except Exception as e:
            self.get_logger().error(f"Reconnection attempt {self.mqtt_reconnect_attempts} failed: {str(e)}")
            self.print_status(f"MQTT: Reconnection attempt {self.mqtt_reconnect_attempts} failed")
            
            # Schedule next attempt
            self.schedule_reconnection()

    def on_mqtt_message(self, client, userdata, msg):
        """Callback when MQTT message is received"""
        try:
            if msg.topic == "robot/physical/scan":
                # Terima data LiDAR dari robot fisik
                scan_data = json.loads(msg.payload.decode())
                # Convert ke format LaserScan dan publish ke ROS
                self.publish_physical_scan(scan_data)
            elif msg.topic == "robot/lidar/real_world_data":
                # Terima data LiDAR real-world untuk simulasi Gazebo
                lidar_data = json.loads(msg.payload.decode())
                print(f"[MQTT] Received real-world LiDAR data: {len(lidar_data.get('ranges', []))} points")
                
                # Add detailed debug logging
                ranges = lidar_data.get('ranges', [])
                if ranges:
                    valid_ranges = [r for r in ranges if r != float('inf')]
                    if valid_ranges:
                        min_distance = min(valid_ranges)
                        print(f"[MQTT DEBUG] Real-world LiDAR min distance: {min_distance:.3f}m")
                        if min_distance < 0.5:  # Log close obstacles
                            print(f"[MQTT DEBUG] ⚠️ CLOSE OBSTACLE DETECTED: {min_distance:.3f}m")
                
                self.process_real_world_obstacles(lidar_data['ranges'])
            elif msg.topic == "robot/lidar/status":
                # Handle status dari LiDAR real-world
                status_data = json.loads(msg.payload.decode())
                self.print_status(f"LiDAR Real-World: {status_data.get('message', 'Unknown status')}")
            elif msg.topic == "robot/physical/status":
                # Ignore status messages from physical robot to prevent echo
                pass
            elif msg.topic == "robot/physical/battery":
                # Handle battery voltage data from physical robot
                data = json.loads(msg.payload.decode())
                if 'voltage' in data:
                    self.battery_voltage = data['voltage']
                    # Check if battery is low
                    if self.battery_voltage < self.battery_low_threshold and not self.nav_charging_mode:
                        self.nav_charging_mode = True
                        self.battery_low = True
                        self.print_status(f"BATTERY LOW! Voltage: {self.battery_voltage}V - Navigating to charging station (0,0)")
                    else:
                        self.battery_low = self.battery_voltage < self.battery_low_threshold
                        self.print_status(f"Battery voltage: {self.battery_voltage:.2f}V, Low: {self.battery_low}")
            elif msg.topic == "test/topic":
                # Handle command messages
                data = json.loads(msg.payload.decode())
                if 'command' in data:
                    if data['command'] == 'stop':
                        # Only pause if not in charging navigation mode
                        if not self.nav_charging_mode:
                            self.mqtt_paused = True
                            # Stop the robot
                            self.send_command_to_robot(0.0, 0.0, "ROBOT IDLE")
                            # Pause all modes
                            self.primary_active = False
                            self.secondary_control_active = False
                            self.r_mode = False
                            self.recovery_active = False
                            self.manual_mode = False  # Disable manual mode too
                            self.is_backing_up = False  # Stop any backup behavior
                            self.turning = False  # Stop any turning
                            self.is_scanning = False  # Stop any scanning
                            self.turning_to_best = False  # Stop any best direction turning
                            self.consecutive_turns = 0  # Reset turn counter
                            # Send status to physical robot
                            if self.mqtt_client is not None:
                                self.mqtt_client.publish("robot/physical/status", json.dumps({
                                    'status': 'ROBOT STOPPED',
                                    'timestamp': time.time()
                                }))
                            self.print_status("MQTT: Received STOP command - Robot IDLE")
                        else:
                            # Ignore stop command during charging navigation
                            self.print_status("MQTT: Ignoring STOP command - Charging navigation in progress")
                    elif data['command'] == 'start':
                        self.mqtt_paused = False
                        # Resume primary control
                        self.primary_active = True
                        # Send status to physical robot
                        if self.mqtt_client is not None:
                            self.mqtt_client.publish("robot/physical/status", json.dumps({
                                'status': 'ROBOT STARTED',
                                'timestamp': time.time()
                            }))
                        self.print_status("MQTT: Received START command - Resuming primary control")
                    elif data['command'].startswith('waypoint_'):
                        # Handle waypoint navigation commands
                        waypoint_key = data['command'].split('_')[1].lower()
                        if waypoint_key == 'i':
                            # Go to waypoint 0
                            self.virtual_path_index = 0
                            self.r_mode = True
                            self.specific_waypoint_navigation = True
                            self.primary_active = False
                            self.secondary_control_active = False
                            self.manual_mode = False
                            self.print_status(f"MQTT: Received waypoint command 'waypoint_i' - Navigating to waypoint 0 (Start point)")
                        elif waypoint_key == 'q':
                            # Go to waypoint 5
                            self.virtual_path_index = 5
                            self.r_mode = True
                            self.specific_waypoint_navigation = True
                            self.primary_active = False
                            self.secondary_control_active = False
                            self.manual_mode = False
                            self.print_status(f"MQTT: Received waypoint command 'waypoint_q' - Navigating to waypoint 5 (MID LEFT)")
                        elif waypoint_key == 'p':
                            # Go to waypoint 2
                            self.virtual_path_index = 2
                            self.r_mode = True
                            self.specific_waypoint_navigation = True
                            self.primary_active = False
                            self.secondary_control_active = False
                            self.manual_mode = False
                            self.print_status(f"MQTT: Received waypoint command 'waypoint_p' - Navigating to waypoint 2 (Bottom-right point)")
                        else:
                            self.print_status(f"MQTT: Unknown waypoint command '{waypoint_key}' - Valid commands: waypoint_i, waypoint_q, waypoint_p")
        except Exception as e:
            self.get_logger().error(f"Error processing MQTT message: {str(e)}")
            self.print_status(f"MQTT: Error processing message: {str(e)}")

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
            
            # DEBUGGING: Check for problematic readings (0.108m issue)
            close_readings = []
            for i, r in enumerate(ranges):
                if 0.05 < r < 0.20:  # Detect readings that might be robot body
                    angle = i  # degrees
                    # Skip rear sector
                    if not (rear_start_angle <= angle <= rear_end_angle):
                        close_readings.append((angle, r))
            
            # Log problematic readings for debugging
            if close_readings:
                current_time = time.time()
                if not hasattr(self, 'last_close_reading_log'):
                    self.last_close_reading_log = 0
                
                if current_time - self.last_close_reading_log > 10.0:  # Log every 10 seconds
                    self.print_warning(f"LiDAR: {len(close_readings)} close readings detected (might be robot body)")
                    
                    # Check if 0.108m reading is in front sector
                    front_close = [r for a, r in close_readings if -30 <= a <= 30 or 330 <= a <= 360]
                    if front_close:
                        self.print_warning(f"LiDAR: {len(front_close)} front close readings: {front_close[:3]}")
                        self.print_warning(f"LiDAR: These might be robot body detections - consider adjusting sensor position")
                    
                    self.last_close_reading_log = current_time
            
            # Additional filtering to prevent false positives
            # If reading is suspiciously consistent (like 0.108m), it might be robot body
            if hasattr(self, 'previous_real_world_front_reading'):
                # Check if reading is stuck at same value (robot body detection)
                if abs(self.real_world_front_min - self.previous_real_world_front_reading) < 0.005:
                    self.real_world_consistent_reading_count = getattr(self, 'real_world_consistent_reading_count', 0) + 1
                    if self.real_world_consistent_reading_count > 20:  # 20 consecutive similar readings
                        self.print_warning(f"LiDAR: Consistent reading {self.real_world_front_min:.3f}m detected {self.real_world_consistent_reading_count} times - filtering out")
                        self.real_world_front_min = float('inf')  # Filter out consistent readings
                else:
                    self.real_world_consistent_reading_count = 0
            
            self.previous_real_world_front_reading = self.real_world_front_min
            
            # Trigger combined obstacle data update
            self.update_combined_obstacle_data()
            
            # Batasi debug logging real-world LiDAR data setiap 3 detik
            current_time = time.time()
            if not hasattr(self, 'last_real_world_log_time'):
                self.last_real_world_log_time = 0
            if current_time - self.last_real_world_log_time > 3.0:
                self.print_status(f"📡 Real-World LiDAR (FILTERED): Front={self.real_world_front_min:.2f}m, "
                                  f"Left={self.real_world_left_min:.2f}m, Right={self.real_world_right_min:.2f}m")
                self.last_real_world_log_time = current_time
                    
        except Exception as e:
            self.get_logger().error(f"Error processing real-world obstacles: {str(e)}")
            self.print_status(f"Error processing real-world obstacles: {str(e)}")
            print(f"[ERROR] Real-world LiDAR processing error: {str(e)}")

    def publish_physical_scan(self, scan_data):
        """Convert and publish physical robot's LiDAR data to ROS"""
        try:
            scan_msg = LaserScan()
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.header.frame_id = "physical_robot_laser"
            scan_msg.angle_min = scan_data.get('angle_min', -3.14159)
            scan_msg.angle_max = scan_data.get('angle_max', 3.14159)
            scan_msg.angle_increment = scan_data.get('angle_increment', 0.0174533)
            scan_msg.time_increment = scan_data.get('time_increment', 0.0)
            scan_msg.scan_time = scan_data.get('scan_time', 0.0)
            scan_msg.range_min = scan_data.get('range_min', 0.0)
            scan_msg.range_max = scan_data.get('range_max', 30.0)
            scan_msg.ranges = scan_data.get('ranges', [])
            scan_msg.intensities = scan_data.get('intensities', [])
            
            # Publish ke topic ROS yang benar
            self.physical_scan_pub.publish(scan_msg)
            self.print_status("Published physical robot LiDAR data to ROS")
        except Exception as e:
            self.get_logger().error(f"Error publishing physical scan: {str(e)}")
            self.print_status(f"Error publishing physical scan: {str(e)}")

def main(args=None):
    """Main function to run the robot node"""
    try:
        # Initialize ROS2
        rclpy.init(args=args)
        node = RoamingRobot()
        
        # Start keyboard listener in a separate thread
        keyboard_thread = threading.Thread(target=node.keyboard_listener)
        keyboard_thread.daemon = True
        keyboard_thread.start()
        
        print("=== ROAM.PY STARTED ===")
        print("Keyboard listener started in separate thread")
        print("Main loop will handle ROS2 spinning")
        print("=" * 50)
        
        # Main loop - focus on ROS2 spinning
        while node.running:
            try:
                # Spin ROS2 node
                rclpy.spin_once(node, timeout_sec=0.1)
                
                # Small delay to prevent excessive CPU usage
                time.sleep(0.01)
                
            except KeyboardInterrupt:
                print('\nKeyboard interrupt received in main loop')
                break
            except Exception as e:
                print(f'\nError in main loop: {str(e)}')
                # Don't break immediately, try to continue
                print(f'Attempting to continue... (Error: {type(e).__name__})')
                time.sleep(1.0)  # Wait a bit before continuing
                continue  # Continue instead of break
                
    except KeyboardInterrupt:
        print('\nKeyboard interrupt received')
    except Exception as e:
        print(f'\nError in main: {str(e)}')
    finally:
        try:
            print('\n=== SHUTDOWN SEQUENCE ===')
            # Cleanup node
            if 'node' in locals():
                print("Cleaning up node...")
                node.cleanup()
                node.destroy_node()
            # Shutdown ROS2
            print("Shutting down ROS2...")
            rclpy.shutdown()
            print("✓ Cleanup completed")
        except Exception as e:
            print(f'\nError during final cleanup: {str(e)}')
        sys.exit(0)

if __name__ == '__main__':
    main() 