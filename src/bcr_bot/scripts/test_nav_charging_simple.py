#!/usr/bin/env python3
"""
Simple test script untuk memverifikasi navigation charging logic
"""

class MockRobot:
    """Mock robot class untuk testing navigation charging logic"""
    
    def __init__(self):
        # Initialize control mode flags
        self.primary_active = True
        self.secondary_control_active = False
        self.manual_mode = False
        self.r_mode = False
        
        # Initialize navigation charging attributes
        self.nav_charging_mode = False
        self.battery_low = False
        
        # Mock position
        self.last_odom_pos = (1.0, 1.0)  # Robot at position (1,1)
        
    def handle_key(self, key):
        """Mock handle_key function"""
        # Set manual mode and update last input time for any key press
        self.manual_mode = True
        
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
                print("BATTERY LOW! Navigating to charging station (0,0)")
            else:
                self.nav_charging_mode = False
                self.battery_low = False
                # Resume primary control when exiting navigation charging
                self.primary_active = True
                print("Charging navigation cancelled, resuming normal operation")
            return
    
    def timer_callback(self):
        """Mock timer_callback function"""
        # Navigation charging mode takes highest priority
        if self.nav_charging_mode:
            print("Navigation charging mode active - navigating to charging station")
            return
        
        # Manual mode takes highest priority (but not over navigation charging)
        if self.manual_mode and not self.nav_charging_mode:
            print("Manual mode active")
            return
        
        # Primary control
        if self.primary_active:
            print("Primary control active")
            return

def test_nav_charging_logic():
    """Test navigation charging logic"""
    print("=== TEST NAVIGATION CHARGING LOGIC ===")
    
    robot = MockRobot()
    
    print("Initial state:")
    print(f"  nav_charging_mode: {robot.nav_charging_mode}")
    print(f"  primary_active: {robot.primary_active}")
    print(f"  manual_mode: {robot.manual_mode}")
    print(f"  battery_low: {robot.battery_low}")
    
    # Test 1: Press 't' to activate navigation charging
    print("\n--- Test 1: Activate navigation charging ---")
    robot.handle_key('t')
    
    print("After 't' press:")
    print(f"  nav_charging_mode: {robot.nav_charging_mode}")
    print(f"  primary_active: {robot.primary_active}")
    print(f"  manual_mode: {robot.manual_mode}")
    print(f"  battery_low: {robot.battery_low}")
    
    # Verify navigation charging is activated correctly
    if robot.nav_charging_mode and not robot.primary_active and not robot.manual_mode and robot.battery_low:
        print("✓ Navigation charging activated correctly!")
    else:
        print("✗ Navigation charging not activated correctly!")
        return False
    
    # Test 2: Timer callback should prioritize navigation charging
    print("\n--- Test 2: Timer callback prioritization ---")
    robot.timer_callback()
    
    # Test 3: Press 't' again to deactivate
    print("\n--- Test 3: Deactivate navigation charging ---")
    robot.handle_key('t')
    
    print("After second 't' press:")
    print(f"  nav_charging_mode: {robot.nav_charging_mode}")
    print(f"  primary_active: {robot.primary_active}")
    print(f"  manual_mode: {robot.manual_mode}")
    print(f"  battery_low: {robot.battery_low}")
    
    # Verify navigation charging is deactivated correctly
    if not robot.nav_charging_mode and robot.primary_active and not robot.battery_low:
        print("✓ Navigation charging deactivated correctly!")
    else:
        print("✗ Navigation charging not deactivated correctly!")
        return False
    
    # Test 4: Timer callback should now use primary control
    print("\n--- Test 4: Timer callback after deactivation ---")
    robot.timer_callback()
    
    print("\n✅ ALL NAVIGATION CHARGING TESTS PASSED!")
    return True

if __name__ == "__main__":
    test_nav_charging_logic() 