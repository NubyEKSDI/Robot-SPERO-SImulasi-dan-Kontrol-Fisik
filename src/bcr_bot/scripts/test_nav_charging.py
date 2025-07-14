#!/usr/bin/env python3
"""
Test script untuk memverifikasi navigation charging mode
"""

import time
import sys
import os

# Add the current directory to Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Import the roam module
from roam import RoamingRobot
import rclpy

def test_nav_charging():
    """Test navigation charging mode"""
    print("=== TEST NAVIGATION CHARGING MODE ===")
    
    try:
        # Initialize ROS2
        rclpy.init()
        
        # Create robot instance
        robot = RoamingRobot()
        
        print("Robot initialized successfully")
        print(f"Initial nav_charging_mode: {robot.nav_charging_mode}")
        print(f"Initial primary_active: {robot.primary_active}")
        print(f"Initial manual_mode: {robot.manual_mode}")
        
        # Simulate pressing 't' key
        print("\nSimulating 't' key press...")
        robot.handle_key('t')
        
        print(f"After 't' press - nav_charging_mode: {robot.nav_charging_mode}")
        print(f"After 't' press - primary_active: {robot.primary_active}")
        print(f"After 't' press - manual_mode: {robot.manual_mode}")
        print(f"After 't' press - battery_low: {robot.battery_low}")
        
        # Check if navigation charging mode is properly activated
        if robot.nav_charging_mode and not robot.primary_active and not robot.manual_mode:
            print("✓ Navigation charging mode activated correctly!")
        else:
            print("✗ Navigation charging mode not activated correctly!")
            return False
        
        # Simulate timer callback to see if navigation charging logic runs
        print("\nSimulating timer callback...")
        robot.timer_callback()
        
        # Simulate pressing 't' again to cancel
        print("\nSimulating 't' key press again to cancel...")
        robot.handle_key('t')
        
        print(f"After second 't' press - nav_charging_mode: {robot.nav_charging_mode}")
        print(f"After second 't' press - primary_active: {robot.primary_active}")
        print(f"After second 't' press - manual_mode: {robot.manual_mode}")
        print(f"After second 't' press - battery_low: {robot.battery_low}")
        
        # Check if navigation charging mode is properly deactivated
        if not robot.nav_charging_mode and robot.primary_active and not robot.battery_low:
            print("✓ Navigation charging mode deactivated correctly!")
        else:
            print("✗ Navigation charging mode not deactivated correctly!")
            return False
        
        print("\n✅ NAVIGATION CHARGING TEST PASSED!")
        return True
        
    except Exception as e:
        print(f"✗ Error during test: {str(e)}")
        return False
    finally:
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == "__main__":
    test_nav_charging() 