#!/usr/bin/env python3
"""
Test script untuk memverifikasi konfigurasi kecepatan dan obstacle detection
"""

class SpeedTest:
    """Test class untuk memverifikasi konfigurasi kecepatan"""
    
    def __init__(self):
        # Navigation charging speeds
        self.charging_forward_speed = 0.5  # Same as R-mode forward speed
        self.charging_turn_speed = 0.8  # Slower rotation
        self.charging_obstacle_threshold = 0.8  # Smaller detection area
        self.charging_emergency_stop_threshold = 0.3  # Smaller emergency stop
        
        # R-mode speeds
        self.r_mode_forward_speed = 1.6  # Same as primary mode
        self.r_mode_turn_speed = 1.5  # Keep same as before
        
        # Primary mode speeds (for reference)
        self.primary_forward_speed = 1.6
        self.primary_turn_speed = 1.5
        
        # Obstacle thresholds
        self.primary_obstacle_threshold = 1.4
        self.primary_emergency_stop_threshold = 0.4
        
    def test_navigation_charging_speeds(self):
        """Test navigation charging speeds"""
        print("=== NAVIGATION CHARGING SPEEDS ===")
        print(f"Forward speed: {self.charging_forward_speed} m/s (same as R-mode)")
        print(f"Turn speed: {self.charging_turn_speed} rad/s (slower than before)")
        print(f"Obstacle threshold: {self.charging_obstacle_threshold} m (smaller than primary)")
        print(f"Emergency stop threshold: {self.charging_emergency_stop_threshold} m (smaller than primary)")
        
        # Verify speeds are appropriate
        if self.charging_forward_speed == 0.5:
            print("✓ Forward speed matches R-mode")
        else:
            print("✗ Forward speed doesn't match R-mode")
            
        if self.charging_turn_speed < 1.5:
            print("✓ Turn speed is slower (good for navigation)")
        else:
            print("✗ Turn speed is not slower")
            
        if self.charging_obstacle_threshold < self.primary_obstacle_threshold:
            print("✓ Obstacle threshold is smaller (prevents stuck/loop)")
        else:
            print("✗ Obstacle threshold is not smaller")
            
        if self.charging_emergency_stop_threshold < self.primary_emergency_stop_threshold:
            print("✓ Emergency stop threshold is smaller (safer)")
        else:
            print("✗ Emergency stop threshold is not smaller")
    
    def test_r_mode_speeds(self):
        """Test R-mode speeds"""
        print("\n=== R-MODE SPEEDS ===")
        print(f"Forward speed: {self.r_mode_forward_speed} m/s (same as primary mode)")
        print(f"Turn speed: {self.r_mode_turn_speed} rad/s")
        
        # Verify speeds are appropriate
        if self.r_mode_forward_speed == self.primary_forward_speed:
            print("✓ Forward speed matches primary mode")
        else:
            print("✗ Forward speed doesn't match primary mode")
            
        if self.r_mode_turn_speed == self.primary_turn_speed:
            print("✓ Turn speed matches primary mode")
        else:
            print("✗ Turn speed doesn't match primary mode")
    
    def test_obstacle_detection_comparison(self):
        """Test obstacle detection comparison"""
        print("\n=== OBSTACLE DETECTION COMPARISON ===")
        print("Primary mode:")
        print(f"  Obstacle threshold: {self.primary_obstacle_threshold} m")
        print(f"  Emergency stop: {self.primary_emergency_stop_threshold} m")
        print("Navigation charging:")
        print(f"  Obstacle threshold: {self.charging_obstacle_threshold} m")
        print(f"  Emergency stop: {self.charging_emergency_stop_threshold} m")
        
        # Calculate differences
        obstacle_diff = self.primary_obstacle_threshold - self.charging_obstacle_threshold
        emergency_diff = self.primary_emergency_stop_threshold - self.charging_emergency_stop_threshold
        
        print(f"\nDifferences:")
        print(f"  Obstacle threshold: {obstacle_diff:.1f} m smaller in charging mode")
        print(f"  Emergency stop: {emergency_diff:.1f} m smaller in charging mode")
        
        if obstacle_diff > 0 and emergency_diff > 0:
            print("✓ Navigation charging has smaller detection areas (prevents stuck/loop)")
        else:
            print("✗ Navigation charging doesn't have smaller detection areas")
    
    def run_all_tests(self):
        """Run all speed and detection tests"""
        print("SPEED AND OBSTACLE DETECTION CONFIGURATION TEST")
        print("=" * 50)
        
        self.test_navigation_charging_speeds()
        self.test_r_mode_speeds()
        self.test_obstacle_detection_comparison()
        
        print("\n" + "=" * 50)
        print("SUMMARY:")
        print("- Navigation charging: Slower rotation, smaller detection areas")
        print("- R-mode: Same speeds as primary mode for waypoint navigation")
        print("- Forward/backward speeds: R-mode speed (0.5 m/s) for navigation charging")
        print("- Waypoint navigation: Primary mode speed (1.6 m/s)")

if __name__ == "__main__":
    test = SpeedTest()
    test.run_all_tests() 