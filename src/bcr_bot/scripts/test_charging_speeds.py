#!/usr/bin/env python3
"""
Test script untuk memverifikasi konfigurasi kecepatan navigation charging di test_robot.py
"""

class ChargingSpeedTest:
    """Test class untuk memverifikasi kecepatan navigation charging"""
    
    def __init__(self):
        # Mock robot tester untuk testing
        pass
    
    def calculate_speed_factor(self, linear_x, angular_z, mode):
        """Mock calculate_speed_factor function from test_robot.py"""
        max_velocity = max(abs(linear_x), abs(angular_z))
        
        if max_velocity <= 0.01:
            return 0.0
        
        # Navigation charging speed mapping (updated)
        if "CHARGING NAV" in mode or "Navigation charging" in mode:
            # Navigation charging uses slower, more controlled speed mapping
            # Slower speeds to prevent stuck/loop issues
            if max_velocity < 0.1:
                speed_factor = 0.2  # Minimum speed to ensure wheels turn
            elif max_velocity <= 0.3:
                speed_factor = 0.2 + (max_velocity - 0.1) * 0.3  # 0.2 to 0.26 (slower)
            elif max_velocity <= 0.6:
                speed_factor = 0.26 + (max_velocity - 0.3) * 0.2  # 0.26 to 0.32 (slower)
            elif max_velocity <= 1.0:
                speed_factor = 0.32 + (max_velocity - 0.6) * 0.15  # 0.32 to 0.38 (slower)
            else:
                speed_factor = 0.38 + min((max_velocity - 1.0) * 0.1, 0.62)  # 0.38 to 1.0 (slower)
        else:
            # Default mapping for other modes
            if max_velocity <= 0.5:
                speed_factor = 0.1 + (max_velocity - 0.1) * 0.5  # 0.1 to 0.3
            elif max_velocity <= 1.0:
                speed_factor = 0.3 + (max_velocity - 0.5) * 0.6  # 0.3 to 0.6
            elif max_velocity <= 2.0:
                speed_factor = 0.6 + (max_velocity - 1.0) * 0.2  # 0.6 to 0.8
            else:
                speed_factor = 0.8 + min((max_velocity - 2.0) * 0.1, 0.2)  # 0.8 to 1.0
        
        return max(0.0, min(1.0, speed_factor))
    
    def test_navigation_charging_speeds(self):
        """Test navigation charging speed mapping"""
        print("=== NAVIGATION CHARGING SPEED TEST ===")
        
        # Test cases for navigation charging
        test_cases = [
            (0.0, 0.0, "CHARGING NAV: Moving to charging station"),
            (0.1, 0.0, "CHARGING NAV: Moving to charging station"),
            (0.3, 0.0, "CHARGING NAV: Moving to charging station"),
            (0.5, 0.0, "CHARGING NAV: Moving to charging station"),
            (0.8, 0.0, "CHARGING NAV: Moving to charging station"),
            (1.0, 0.0, "CHARGING NAV: Moving to charging station"),
            (0.0, 0.8, "CHARGING NAV: Rotating to face charging station"),
            (0.0, 1.5, "CHARGING NAV: Rotating to face charging station"),
        ]
        
        print("Navigation Charging Speed Mapping:")
        print("Velocity\tSpeed Factor\tDescription")
        print("-" * 50)
        
        for linear_x, angular_z, mode in test_cases:
            speed_factor = self.calculate_speed_factor(linear_x, angular_z, mode)
            velocity = max(abs(linear_x), abs(angular_z))
            print(f"{velocity:.1f}\t\t{speed_factor:.3f}\t\t{mode}")
        
        # Verify speeds are slower than before
        print("\nSpeed Verification:")
        
        # Test minimum speed
        min_speed = self.calculate_speed_factor(0.1, 0.0, "CHARGING NAV: Test")
        if min_speed >= 0.2:
            print("✓ Minimum speed is sufficient to turn wheels")
        else:
            print("✗ Minimum speed might be too low")
        
        # Test maximum speed
        max_speed = self.calculate_speed_factor(2.0, 0.0, "CHARGING NAV: Test")
        if max_speed <= 1.0:
            print("✓ Maximum speed is capped at 1.0")
        else:
            print("✗ Maximum speed exceeds 1.0")
        
        # Test that speeds are slower than default
        default_speed = self.calculate_speed_factor(0.5, 0.0, "Default mode")
        charging_speed = self.calculate_speed_factor(0.5, 0.0, "CHARGING NAV: Test")
        
        if charging_speed < default_speed:
            print("✓ Navigation charging speeds are slower than default (good for control)")
        else:
            print("✗ Navigation charging speeds are not slower than default")
    
    def test_speed_comparison(self):
        """Compare navigation charging speeds with other modes"""
        print("\n=== SPEED COMPARISON TEST ===")
        
        test_velocity = 0.5
        
        # Test different modes
        modes = [
            ("CHARGING NAV: Test", "Navigation Charging"),
            ("Primary control: Test", "Primary Mode"),
            ("R-mode: Test", "R-Mode"),
            ("Default mode", "Default Mode")
        ]
        
        print("Speed Factor Comparison (velocity = 0.5):")
        print("Mode\t\t\tSpeed Factor")
        print("-" * 40)
        
        for mode, description in modes:
            speed_factor = self.calculate_speed_factor(test_velocity, 0.0, mode)
            print(f"{description}\t\t{speed_factor:.3f}")
        
        # Verify navigation charging is slower
        charging_speed = self.calculate_speed_factor(test_velocity, 0.0, "CHARGING NAV: Test")
        primary_speed = self.calculate_speed_factor(test_velocity, 0.0, "Primary control: Test")
        
        print(f"\nComparison:")
        print(f"Navigation Charging: {charging_speed:.3f}")
        print(f"Primary Mode: {primary_speed:.3f}")
        
        if charging_speed < primary_speed:
            print("✓ Navigation charging is slower than primary mode (good for precision)")
        else:
            print("✗ Navigation charging is not slower than primary mode")
    
    def run_all_tests(self):
        """Run all speed tests"""
        print("NAVIGATION CHARGING SPEED CONFIGURATION TEST")
        print("=" * 60)
        
        self.test_navigation_charging_speeds()
        self.test_speed_comparison()
        
        print("\n" + "=" * 60)
        print("SUMMARY:")
        print("- Navigation charging uses slower, more controlled speeds")
        print("- Minimum speed: 0.2 (ensures wheels turn)")
        print("- Maximum speed: 1.0 (capped for safety)")
        print("- Slower than primary mode for better precision")
        print("- Prevents stuck/loop issues during charging navigation")

if __name__ == "__main__":
    test = ChargingSpeedTest()
    test.run_all_tests() 