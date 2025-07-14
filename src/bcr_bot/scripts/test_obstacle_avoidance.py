#!/usr/bin/env python3
"""
Test script untuk obstacle avoidance dengan LiDAR real-world
Script ini akan mengirim data LiDAR dengan obstacle dan melihat robot turning
"""

import time
import json
import paho.mqtt.client as mqtt
import math

# MQTT Configuration
MQTT_BROKER = "codex.petra.ac.id"
MQTT_PORT = 1883
TOPIC_LIDAR_REAL_WORLD_DATA = "robot/lidar/real_world_data"
TOPIC_PHYSICAL_CMD = "robot/physical/cmd_vel"

def test_obstacle_avoidance():
    """Test obstacle avoidance functionality"""
    print("=== OBSTACLE AVOIDANCE TEST ===")
    print("Testing robot turning when obstacles are detected")
    print("=" * 50)
    
    # Initialize MQTT
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
    
    try:
        print(f"Connecting to MQTT broker {MQTT_BROKER}...")
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_start()
        time.sleep(2)
        
        print("✓ MQTT connected")
        
        # Test 1: Send forward command first
        print("\n1. Sending forward command...")
        cmd_data = {
            "linear": {"x": 0.5, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
            "mode": "MANUAL CONTROL"
        }
        client.publish(TOPIC_PHYSICAL_CMD, json.dumps(cmd_data))
        time.sleep(2)
        
        # Test 2: Send LiDAR data with obstacle in front
        print("2. Sending LiDAR data with obstacle in front (should trigger turning)...")
        ranges = [float('inf')] * 360
        ranges[0] = 0.8  # Obstacle 80cm in front
        
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
        client.publish(TOPIC_LIDAR_REAL_WORLD_DATA, json.dumps(lidar_data))
        time.sleep(3)
        
        # Test 3: Send LiDAR data with obstacle on left (should turn right)
        print("3. Sending LiDAR data with obstacle on left (should turn right)...")
        ranges = [float('inf')] * 360
        ranges[0] = 0.6  # Obstacle 60cm in front
        ranges[90] = 0.5  # Obstacle 50cm on left
        ranges[270] = 2.0  # Clear on right
        
        lidar_data["ranges"] = ranges
        lidar_data["timestamp"] = time.time()
        client.publish(TOPIC_LIDAR_REAL_WORLD_DATA, json.dumps(lidar_data))
        time.sleep(3)
        
        # Test 4: Send LiDAR data with obstacle on right (should turn left)
        print("4. Sending LiDAR data with obstacle on right (should turn left)...")
        ranges = [float('inf')] * 360
        ranges[0] = 0.7  # Obstacle 70cm in front
        ranges[90] = 2.0  # Clear on left
        ranges[270] = 0.4  # Obstacle 40cm on right
        
        lidar_data["ranges"] = ranges
        lidar_data["timestamp"] = time.time()
        client.publish(TOPIC_LIDAR_REAL_WORLD_DATA, json.dumps(lidar_data))
        time.sleep(3)
        
        # Test 5: Send LiDAR data with both sides blocked (should turn to clearer side)
        print("5. Sending LiDAR data with both sides blocked...")
        ranges = [float('inf')] * 360
        ranges[0] = 0.5  # Obstacle 50cm in front
        ranges[90] = 0.6  # Obstacle 60cm on left
        ranges[270] = 0.8  # Obstacle 80cm on right (slightly clearer)
        
        lidar_data["ranges"] = ranges
        lidar_data["timestamp"] = time.time()
        client.publish(TOPIC_LIDAR_REAL_WORLD_DATA, json.dumps(lidar_data))
        time.sleep(3)
        
        # Test 6: Send LiDAR data with very close obstacle (should emergency stop)
        print("6. Sending LiDAR data with very close obstacle (should emergency stop)...")
        ranges = [float('inf')] * 360
        ranges[0] = 0.2  # Obstacle 20cm in front - very close!
        
        lidar_data["ranges"] = ranges
        lidar_data["timestamp"] = time.time()
        client.publish(TOPIC_LIDAR_REAL_WORLD_DATA, json.dumps(lidar_data))
        time.sleep(3)
        
        # Test 7: Send LiDAR data with clear path (should continue forward)
        print("7. Sending LiDAR data with clear path (should continue forward)...")
        ranges = [float('inf')] * 360
        ranges[0] = 5.0  # Clear path 5m ahead
        
        lidar_data["ranges"] = ranges
        lidar_data["timestamp"] = time.time()
        client.publish(TOPIC_LIDAR_REAL_WORLD_DATA, json.dumps(lidar_data))
        time.sleep(3)
        
        # Test 8: Stop robot
        print("8. Stopping robot...")
        cmd_data = {
            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
            "mode": "MANUAL CONTROL"
        }
        client.publish(TOPIC_PHYSICAL_CMD, json.dumps(cmd_data))
        
        print("\n✓ Obstacle avoidance test completed!")
        print("Check test_robot.py output to see if robot turned when obstacles were detected")
        
    except Exception as e:
        print(f"✗ Error: {str(e)}")
    finally:
        client.loop_stop()
        client.disconnect()

def test_continuous_obstacle_avoidance():
    """Test continuous obstacle avoidance with moving obstacles"""
    print("\n=== CONTINUOUS OBSTACLE AVOIDANCE TEST ===")
    print("Testing continuous obstacle avoidance for 30 seconds")
    print("=" * 60)
    
    # Initialize MQTT
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
    
    try:
        print(f"Connecting to MQTT broker {MQTT_BROKER}...")
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_start()
        time.sleep(2)
        
        print("✓ MQTT connected")
        
        # Start forward movement
        print("\nStarting forward movement...")
        cmd_data = {
            "linear": {"x": 0.3, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
            "mode": "MANUAL CONTROL"
        }
        client.publish(TOPIC_PHYSICAL_CMD, json.dumps(cmd_data))
        time.sleep(1)
        
        # Send continuous LiDAR data with moving obstacles
        print("Sending continuous LiDAR data with moving obstacles...")
        start_time = time.time()
        
        while time.time() - start_time < 30:
            time_offset = time.time() - start_time
            
            # Create moving obstacle pattern
            ranges = [float('inf')] * 360
            
            # Moving obstacle that changes position
            obstacle_angle = int(time_offset * 5) % 360  # Moves 5°/sec
            for i in range(obstacle_angle - 15, obstacle_angle + 16):
                angle = i % 360
                ranges[angle] = 1.0 + math.sin(time_offset * 0.2) * 0.3  # Distance varies
            
            # Add some static obstacles
            ranges[0] = 2.0 + math.sin(time_offset * 0.1) * 0.5  # Front obstacle
            ranges[90] = 1.5 + math.sin(time_offset * 0.15) * 0.3  # Left obstacle
            ranges[270] = 1.8 + math.sin(time_offset * 0.12) * 0.4  # Right obstacle
            
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
            
            client.publish(TOPIC_LIDAR_REAL_WORLD_DATA, json.dumps(lidar_data))
            
            # Print progress every 5 seconds
            elapsed = time.time() - start_time
            if int(elapsed) % 5 == 0 and elapsed > 0:
                print(f"  Progress: {elapsed:.1f}/30s")
            
            time.sleep(0.1)  # 10Hz update rate
        
        # Stop robot
        print("\nStopping robot...")
        cmd_data = {
            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
            "mode": "MANUAL CONTROL"
        }
        client.publish(TOPIC_PHYSICAL_CMD, json.dumps(cmd_data))
        
        print("✓ Continuous obstacle avoidance test completed!")
        
    except Exception as e:
        print(f"✗ Error: {str(e)}")
    finally:
        client.loop_stop()
        client.disconnect()

def main():
    """Main function"""
    print("Obstacle Avoidance Test Suite")
    print("=" * 40)
    print("This script tests robot turning when obstacles are detected")
    print("Make sure test_robot.py is running")
    print("=" * 40)
    
    try:
        # Run basic obstacle avoidance test
        test_obstacle_avoidance()
        
        # Ask if user wants to run continuous test
        print("\n" + "="*50)
        choice = input("Run continuous obstacle avoidance test? (y/n): ").strip().lower()
        
        if choice == 'y':
            test_continuous_obstacle_avoidance()
        
        print("\n" + "="*50)
        print("🎉 ALL TESTS COMPLETED!")
        print("Check test_robot.py output for results")
        print("="*50)
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\nError: {str(e)}")

if __name__ == "__main__":
    main() 