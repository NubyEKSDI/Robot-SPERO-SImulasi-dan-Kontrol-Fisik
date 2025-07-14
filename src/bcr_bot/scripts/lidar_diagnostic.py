#!/usr/bin/env python3
"""
LiDAR Diagnostic Script
Digunakan untuk mendiagnosis masalah LiDAR A1M8 yang berhenti berputar
"""

import time
import subprocess
import os
from rplidar import RPLidar

def check_system_info():
    """Check system information"""
    print("=== SYSTEM INFORMATION ===")
    
    # Check USB devices
    print("\n1. USB Devices:")
    try:
        result = subprocess.run(['lsusb'], capture_output=True, text=True)
        print(result.stdout)
    except:
        print("Could not run lsusb")
    
    # Check tty devices
    print("\n2. TTY Devices:")
    try:
        result = subprocess.run(['ls', '-la', '/dev/ttyUSB*'], capture_output=True, text=True)
        print(result.stdout)
    except:
        print("No ttyUSB devices found")
    
    # Check USB power
    print("\n3. USB Power Information:")
    try:
        result = subprocess.run(['dmesg', '|', 'grep', '-i', 'usb'], capture_output=True, text=True)
        print(result.stdout[-500:])  # Last 500 chars
    except:
        print("Could not check USB power info")

def check_lidar_permissions():
    """Check LiDAR permissions"""
    print("\n=== LIDAR PERMISSIONS ===")
    
    lidar_port = '/dev/ttyUSB0'
    
    if os.path.exists(lidar_port):
        try:
            stat = os.stat(lidar_port)
            print(f"Device: {lidar_port}")
            print(f"Permissions: {oct(stat.st_mode)[-3:]}")
            print(f"Owner: {stat.st_uid}")
            print(f"Group: {stat.st_gid}")
            
            if oct(stat.st_mode)[-3:] != '666':
                print("⚠️  Permissions need to be fixed!")
                print("Run: sudo chmod 666 /dev/ttyUSB0")
            else:
                print("✅ Permissions OK")
                
        except Exception as e:
            print(f"Error checking permissions: {e}")
    else:
        print(f"❌ Device {lidar_port} not found")

def check_lidar_processes():
    """Check if other processes are using LiDAR"""
    print("\n=== LIDAR PROCESSES ===")
    
    lidar_port = '/dev/ttyUSB0'
    
    try:
        result = subprocess.run(['sudo', 'lsof', lidar_port], capture_output=True, text=True)
        if result.stdout:
            print("Processes using LiDAR:")
            print(result.stdout)
        else:
            print("✅ No other processes using LiDAR")
    except:
        print("Could not check processes (try running with sudo)")

def test_lidar_connection():
    """Test basic LiDAR connection"""
    print("\n=== LIDAR CONNECTION TEST ===")
    
    try:
        print("Attempting to connect to LiDAR...")
        lidar = RPLidar('/dev/ttyUSB0', baudrate=115200, timeout=2.0)
        
        print("✅ Connected to LiDAR")
        
        # Get info
        try:
            info = lidar.get_info()
            print(f"✅ LiDAR Info: {info}")
        except Exception as e:
            print(f"❌ Could not get info: {e}")
        
        # Get health
        try:
            health = lidar.get_health()
            print(f"✅ Health Status: {health}")
            
            if health[0] != 0:
                print("⚠️  HEALTH ISSUE DETECTED!")
                print("This usually indicates power supply problems")
            else:
                print("✅ Health OK")
        except Exception as e:
            print(f"❌ Could not get health: {e}")
        
        # Test scanning
        try:
            print("Testing scanning...")
            lidar.start()
            time.sleep(1.0)
            
            scan_count = 0
            for scan in lidar.iter_scans():
                scan_count += 1
                print(f"✅ Scan {scan_count}: {len(scan)} points")
                
                if scan_count >= 3:
                    break
            
            lidar.stop()
            print("✅ Scanning test successful")
            
        except Exception as e:
            print(f"❌ Scanning test failed: {e}")
        
        # Cleanup
        lidar.disconnect()
        print("✅ LiDAR disconnected")
        
    except Exception as e:
        print(f"❌ LiDAR connection failed: {e}")
        print("This could be due to:")
        print("  - Power supply issues")
        print("  - Hardware connection problems")
        print("  - Permission issues")
        print("  - Another process using the LiDAR")

def power_supply_test():
    """Test power supply stability"""
    print("\n=== POWER SUPPLY TEST ===")
    
    print("Testing LiDAR power supply stability...")
    print("This test will run for 30 seconds to check for power issues")
    
    try:
        lidar = RPLidar('/dev/ttyUSB0', baudrate=115200, timeout=2.0)
        lidar.start()
        
        start_time = time.time()
        scan_count = 0
        errors = 0
        
        while time.time() - start_time < 30:
            try:
                scan = next(lidar.iter_scans())
                scan_count += 1
                
                if scan_count % 10 == 0:
                    elapsed = time.time() - start_time
                    rate = scan_count / elapsed
                    print(f"Scan {scan_count}: {len(scan)} points, Rate: {rate:.1f} scans/sec")
                
            except Exception as e:
                errors += 1
                print(f"Error {errors}: {e}")
                
                if errors >= 5:
                    print("❌ Too many errors - power supply might be unstable")
                    break
        
        lidar.stop()
        lidar.disconnect()
        
        if errors == 0:
            print(f"✅ Power supply test passed: {scan_count} scans, 0 errors")
        else:
            print(f"⚠️  Power supply test: {scan_count} scans, {errors} errors")
            
    except Exception as e:
        print(f"❌ Power supply test failed: {e}")

def main():
    """Main diagnostic function"""
    print("🔧 LIDAR A1M8 DIAGNOSTIC TOOL")
    print("=" * 50)
    print("This tool will help diagnose why your LiDAR stops spinning")
    print()
    
    # Run all diagnostic tests
    check_system_info()
    check_lidar_permissions()
    check_lidar_processes()
    test_lidar_connection()
    
    # Ask if user wants to run power test
    print("\n" + "=" * 50)
    choice = input("Run 30-second power supply stability test? (y/n): ").strip().lower()
    
    if choice == 'y':
        power_supply_test()
    
    print("\n" + "=" * 50)
    print("🔧 TROUBLESHOOTING RECOMMENDATIONS:")
    print()
    print("If LiDAR stops spinning, try these steps in order:")
    print()
    print("1. 🔌 POWER SUPPLY (Most Common Cause):")
    print("   - Use a powered USB hub")
    print("   - Try a different USB cable")
    print("   - Check if USB cable is data+power (not power-only)")
    print("   - Ensure stable 5V/1A power supply")
    print()
    print("2. 🔧 HARDWARE:")
    print("   - Unplug LiDAR for 10 seconds")
    print("   - Try different USB port")
    print("   - Check if motor is physically stuck")
    print()
    print("3. 💻 SOFTWARE:")
    print("   - Run: sudo chmod 666 /dev/ttyUSB0")
    print("   - Kill other processes: sudo lsof /dev/ttyUSB0")
    print("   - Reboot Raspberry Pi")
    print()
    print("4. 🔄 RESET:")
    print("   - Stop all LiDAR programs")
    print("   - Unplug LiDAR for 10 seconds")
    print("   - Plug back in and wait 5 seconds")
    print("   - Run: sudo chmod 666 /dev/ttyUSB0")
    print("   - Test with: python3 test_lidar.py")
    print()
    print("=" * 50)

if __name__ == "__main__":
    main() 