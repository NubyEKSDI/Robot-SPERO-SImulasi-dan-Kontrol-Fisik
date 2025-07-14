#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

# Key mappings
KEY_MAPPING = {
    'w': (1.0, 0.0),    # Forward
    's': (-1.0, 0.0),   # Backward
    'a': (0.0, 1.0),    # Turn left
    'd': (0.0, -1.0),   # Turn right
    ' ': (0.0, 0.0),    # Stop
}

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_twist_keyboard')
        
        # Create publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/bcr_bot/cmd_vel', 10)
        
        # Save terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('Teleop node started. Use WASD to control the robot.')
        self.get_logger().info('Press Ctrl+C to exit.')
        
        # Main loop
        try:
            while True:
                # Get key press
                key = self.get_key()
                
                # Create Twist message
                twist = Twist()
                
                if key in KEY_MAPPING:
                    linear, angular = KEY_MAPPING[key]
                    twist.linear.x = linear * 0.5  # Scale down for safety
                    twist.angular.z = angular * 0.5
                    self.cmd_vel_pub.publish(twist)
                
        except KeyboardInterrupt:
            pass
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 