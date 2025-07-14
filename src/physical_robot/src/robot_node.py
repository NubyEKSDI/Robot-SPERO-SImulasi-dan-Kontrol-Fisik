#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import time

class PhysicalRobot(Node):
    def __init__(self):
        super().__init__('physical_robot')
        
        # Subscribe to cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/physical_robot/cmd_vel',
            self.cmd_vel_callback,
            10)
            
        # Publish odometry
        self.odom_pub = self.create_publisher(
            Odometry,
            '/physical_robot/odom',
            10)
            
        # Publish laser scan
        self.scan_pub = self.create_publisher(
            LaserScan,
            '/physical_robot/scan',
            10)
            
        # Initialize motor control
        self.init_motors()
        
        # Initialize sensors
        self.init_sensors()
        
        # Create timer for publishing sensor data
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('Physical robot node started')
        
    def init_motors(self):
        """Initialize motor control hardware"""
        try:
            # Import GPIO library
            import RPi.GPIO as GPIO
            
            # Set GPIO mode
            GPIO.setmode(GPIO.BCM)
            
            # Define motor pins
            self.motor_pins = {
                'front_left': {'pwm': 17, 'in1': 27, 'in2': 22},
                'front_right': {'pwm': 18, 'in1': 23, 'in2': 24},
                'rear_left': {'pwm': 19, 'in1': 25, 'in2': 8},
                'rear_right': {'pwm': 20, 'in1': 7, 'in2': 12}
            }
            
            # Setup all pins
            for motor in self.motor_pins.values():
                GPIO.setup(motor['pwm'], GPIO.OUT)
                GPIO.setup(motor['in1'], GPIO.OUT)
                GPIO.setup(motor['in2'], GPIO.OUT)
                
            # Create PWM objects
            self.pwm_objects = {}
            for motor_name, motor in self.motor_pins.items():
                self.pwm_objects[motor_name] = GPIO.PWM(motor['pwm'], 1000)
                self.pwm_objects[motor_name].start(0)
                
            self.get_logger().info('Motors initialized successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize motors: {str(e)}')
            
    def init_sensors(self):
        """Initialize sensors (LIDAR, IMU, etc)"""
        try:
            # Initialize LIDAR
            # Add your LIDAR initialization code here
            pass
            
            # Initialize IMU
            # Add your IMU initialization code here
            pass
            
            self.get_logger().info('Sensors initialized successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize sensors: {str(e)}')
            
    def cmd_vel_callback(self, msg):
        """Handle incoming velocity commands"""
        try:
            # Convert Twist message to motor commands
            linear_x = msg.linear.x
            angular_z = msg.angular.z
            
            # Calculate motor speeds
            # For differential drive with 4 wheels
            base_speed = linear_x
            turn_speed = angular_z
            
            # Calculate individual wheel speeds
            left_speed = base_speed - turn_speed
            right_speed = base_speed + turn_speed
            
            # Set motor speeds
            self.set_motor_speed('front_left', left_speed)
            self.set_motor_speed('front_right', right_speed)
            self.set_motor_speed('rear_left', left_speed)
            self.set_motor_speed('rear_right', right_speed)
            
        except Exception as e:
            self.get_logger().error(f'Error in cmd_vel_callback: {str(e)}')
            
    def set_motor_speed(self, motor_name, speed):
        """Set speed for a specific motor"""
        try:
            # Clamp speed between -1 and 1
            speed = max(min(speed, 1.0), -1.0)
            
            # Get motor pins
            motor = self.motor_pins[motor_name]
            
            # Set direction
            if speed >= 0:
                GPIO.output(motor['in1'], GPIO.HIGH)
                GPIO.output(motor['in2'], GPIO.LOW)
            else:
                GPIO.output(motor['in1'], GPIO.LOW)
                GPIO.output(motor['in2'], GPIO.HIGH)
                
            # Set PWM duty cycle
            pwm_duty = abs(speed) * 100
            self.pwm_objects[motor_name].ChangeDutyCycle(pwm_duty)
            
        except Exception as e:
            self.get_logger().error(f'Error setting motor speed: {str(e)}')
            
    def timer_callback(self):
        """Publish sensor data periodically"""
        try:
            # Publish odometry
            odom_msg = Odometry()
            # Fill odometry data
            # Add your odometry calculation code here
            self.odom_pub.publish(odom_msg)
            
            # Publish laser scan
            scan_msg = LaserScan()
            # Fill laser scan data
            # Add your LIDAR reading code here
            self.scan_pub.publish(scan_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in timer_callback: {str(e)}')
            
    def cleanup(self):
        """Cleanup resources"""
        try:
            # Stop all motors
            for motor_name in self.motor_pins.keys():
                self.set_motor_speed(motor_name, 0)
                
            # Cleanup GPIO
            import RPi.GPIO as GPIO
            GPIO.cleanup()
            
        except Exception as e:
            self.get_logger().error(f'Error during cleanup: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = PhysicalRobot()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 