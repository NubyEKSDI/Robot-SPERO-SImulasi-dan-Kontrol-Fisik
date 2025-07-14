#!/usr/bin/env python3
"""
Specialized test script untuk mendiagnosis masalah motor 1 yang kadang berhenti di R-mode
"""

import time
import board
import busio
import adafruit_pca9685
import RPi.GPIO as GPIO

# Constants
PCA_FREQ = 1500
fullSpeed = 0xFFFF  # 65535
halfSpeed = 0x7FFF  # 32767
noSpeed = 0x0000    # 0
speedCoef = 0.4
rmodeSpeedCoef = 0.5
MIN_MOTOR_SPEED = 0x1000  # Minimum speed to ensure motor actually moves (4096)

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

class Motor1Debugger:
    def __init__(self):
        print("[INIT] Initializing PCA9685 for Motor 1 debugging...")
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
            
        except Exception as e:
            print(f"[ERROR] Failed to initialize PCA9685: {str(e)}")
            raise

    def set_motor(self, motor_num, speed, direction):
        """Set motor speed and direction"""
        try:
            if motor_num not in pinPWM or motor_num not in pinDIR:
                print(f"[ERROR] Invalid motor number: {motor_num}")
                return
            
            # Set direction
            dir_value = arahMAJU if direction else arahMUNDUR
            self.pca.channels[pinDIR[motor_num]].duty_cycle = dir_value
            
            # Set speed
            self.pca.channels[pinPWM[motor_num]].duty_cycle = speed
            
            print(f"[MOTOR {motor_num}] Speed: {speed}, Direction: {'FORWARD' if direction else 'BACKWARD'}")
            
        except Exception as e:
            print(f"[ERROR] Failed to set motor {motor_num}: {e}")

    def check_motor_status(self, motor_num):
        """Check current motor status"""
        try:
            pwm_value = self.pca.channels[pinPWM[motor_num]].duty_cycle
            dir_value = self.pca.channels[pinDIR[motor_num]].duty_cycle
            return pwm_value, dir_value
        except Exception as e:
            print(f"[ERROR] Failed to check motor {motor_num} status: {e}")
            return 0, 0

    def test_motor_1_basic(self):
        """Basic motor 1 test"""
        print("\n" + "="*60)
        print("BASIC MOTOR 1 TEST")
        print("="*60)
        
        test_speeds = [MIN_MOTOR_SPEED, halfSpeed, fullSpeed]
        test_directions = [True, False]
        
        for speed in test_speeds:
            for direction in test_directions:
                direction_name = "FORWARD" if direction else "BACKWARD"
                speed_percent = speed / 65535 * 100
                
                print(f"\n[MOTOR 1] Testing {speed_percent:.1f}% {direction_name}")
                
                # Set motor
                self.set_motor(1, speed, direction)
                time.sleep(2)
                
                # Check status
                pwm, dir_val = self.check_motor_status(1)
                print(f"[MOTOR 1] Status - PWM: {pwm}, DIR: {dir_val}")
                
                if pwm == 0:
                    print(f"[MOTOR 1] ⚠️  MOTOR STOPPED UNEXPECTEDLY!")
                else:
                    print(f"[MOTOR 1] ✅ Running normally")
                
                # Stop
                self.set_motor(1, 0, direction)
                time.sleep(1)

    def test_motor_1_r_mode_simulation(self):
        """Simulate R-mode conditions for motor 1"""
        print("\n" + "="*60)
        print("R-MODE SIMULATION TEST")
        print("="*60)
        
        # Simulate R-mode speed calculations
        r_mode_velocities = [0.1, 0.2, 0.3, 0.5, 0.8, 1.0, 1.5]
        
        for velocity in r_mode_velocities:
            print(f"\n[MOTOR 1 R-MODE] Testing velocity: {velocity}")
            
            # Calculate speed factor using the same logic as test_robot.py
            if velocity <= 0.3:
                speed_factor = 0.5 + (velocity - 0.1) * 0.2
            elif velocity <= 0.8:
                speed_factor = 0.54 + (velocity - 0.3) * 0.2
            elif velocity <= 1.5:
                speed_factor = 0.64 + (velocity - 0.8) * 0.2
            else:
                speed_factor = 0.78 + min((velocity - 1.5) * 0.1, 0.22)
            
            # Ensure minimum speed factor
            speed_factor = max(0.5, speed_factor)
            
            # Calculate actual speed
            speed = int(fullSpeed * speed_factor * rmodeSpeedCoef)
            speed_percent = speed / 65535 * 100
            
            print(f"[MOTOR 1 R-MODE] Speed factor: {speed_factor:.3f}")
            print(f"[MOTOR 1 R-MODE] Calculated speed: {speed} ({speed_percent:.1f}%)")
            
            # Test forward
            print(f"[MOTOR 1 R-MODE] Testing FORWARD...")
            self.set_motor(1, speed, True)
            time.sleep(2)
            
            # Check if motor stopped
            pwm, dir_val = self.check_motor_status(1)
            if pwm == 0:
                print(f"[MOTOR 1 R-MODE] ⚠️  MOTOR STOPPED in forward test!")
            else:
                print(f"[MOTOR 1 R-MODE] ✅ Running in forward test")
            
            # Stop
            self.set_motor(1, 0, True)
            time.sleep(1)
            
            # Test backward
            print(f"[MOTOR 1 R-MODE] Testing BACKWARD...")
            self.set_motor(1, speed, False)
            time.sleep(2)
            
            # Check if motor stopped
            pwm, dir_val = self.check_motor_status(1)
            if pwm == 0:
                print(f"[MOTOR 1 R-MODE] ⚠️  MOTOR STOPPED in backward test!")
            else:
                print(f"[MOTOR 1 R-MODE] ✅ Running in backward test")
            
            # Stop
            self.set_motor(1, 0, False)
            time.sleep(1)

    def test_motor_1_continuous(self):
        """Test motor 1 continuously to catch intermittent issues"""
        print("\n" + "="*60)
        print("CONTINUOUS MOTOR 1 TEST")
        print("="*60)
        print("This test runs motor 1 continuously to catch intermittent stopping issues")
        print("Press Ctrl+C to stop the test")
        
        try:
            test_duration = 30  # 30 seconds
            start_time = time.time()
            check_interval = 1.0  # Check every second
            
            # Use typical R-mode speed
            speed = int(fullSpeed * 0.6 * rmodeSpeedCoef)  # 60% speed factor
            print(f"[MOTOR 1 CONTINUOUS] Starting continuous test with speed {speed}")
            
            # Start motor
            self.set_motor(1, speed, True)
            last_check = time.time()
            
            while time.time() - start_time < test_duration:
                time.sleep(0.1)
                
                # Check motor status periodically
                if time.time() - last_check >= check_interval:
                    pwm, dir_val = self.check_motor_status(1)
                    elapsed = time.time() - start_time
                    
                    if pwm == 0:
                        print(f"[MOTOR 1 CONTINUOUS] ⚠️  MOTOR STOPPED at {elapsed:.1f}s!")
                        # Try to restart
                        print(f"[MOTOR 1 CONTINUOUS] Attempting to restart...")
                        self.set_motor(1, speed, True)
                    else:
                        print(f"[MOTOR 1 CONTINUOUS] ✅ Running at {elapsed:.1f}s (PWM: {pwm})")
                    
                    last_check = time.time()
            
            # Stop motor
            self.set_motor(1, 0, True)
            print(f"[MOTOR 1 CONTINUOUS] Test completed")
            
        except KeyboardInterrupt:
            print(f"\n[MOTOR 1 CONTINUOUS] Test interrupted by user")
            self.set_motor(1, 0, True)

    def test_all_motors_comparison(self):
        """Test all motors to compare with motor 1"""
        print("\n" + "="*60)
        print("ALL MOTORS COMPARISON TEST")
        print("="*60)
        
        speed = int(fullSpeed * 0.5 * rmodeSpeedCoef)
        print(f"[COMPARISON] Testing all motors with speed {speed}")
        
        for motor in range(1, 5):
            print(f"\n[MOTOR {motor}] Testing...")
            
            # Test forward
            self.set_motor(motor, speed, True)
            time.sleep(2)
            
            pwm, dir_val = self.check_motor_status(motor)
            if pwm == 0:
                print(f"[MOTOR {motor}] ⚠️  STOPPED in forward test")
            else:
                print(f"[MOTOR {motor}] ✅ Running in forward test")
            
            # Stop
            self.set_motor(motor, 0, True)
            time.sleep(1)
            
            # Test backward
            self.set_motor(motor, speed, False)
            time.sleep(2)
            
            pwm, dir_val = self.check_motor_status(motor)
            if pwm == 0:
                print(f"[MOTOR {motor}] ⚠️  STOPPED in backward test")
            else:
                print(f"[MOTOR {motor}] ✅ Running in backward test")
            
            # Stop
            self.set_motor(motor, 0, False)
            time.sleep(1)

    def run_all_tests(self):
        """Run all motor 1 diagnostic tests"""
        print("MOTOR 1 DIAGNOSTIC TEST SUITE")
        print("="*60)
        print("This test suite will help diagnose why motor 1 stops in R-mode")
        print("="*60)
        
        try:
            # Basic test
            self.test_motor_1_basic()
            
            # R-mode simulation
            self.test_motor_1_r_mode_simulation()
            
            # All motors comparison
            self.test_all_motors_comparison()
            
            # Continuous test
            self.test_motor_1_continuous()
            
            print("\n" + "="*60)
            print("ALL TESTS COMPLETED")
            print("="*60)
            print("Check the output above for any issues with motor 1")
            print("If motor 1 stops unexpectedly, it may indicate:")
            print("1. Hardware connection issues")
            print("2. Motor driver problems")
            print("3. Power supply issues")
            print("4. Software timing problems")
            
        except KeyboardInterrupt:
            print("\n\nTest interrupted by user")
        finally:
            # Stop all motors
            for motor in range(1, 5):
                self.set_motor(motor, 0, True)
            print("All motors stopped")

if __name__ == "__main__":
    try:
        debugger = Motor1Debugger()
        debugger.run_all_tests()
    except Exception as e:
        print(f"Error: {str(e)}")
        print("Make sure the robot is connected and PCA9685 is working") 