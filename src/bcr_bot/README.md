# Robot SPERO - ROS2 Control System

Robot SPERO adalah robot tour guide yang dikembangkan untuk navigasi otonom dengan kemampuan dual LiDAR integration (Gazebo + Real-World), multi-mode operation, dan MQTT communication untuk kontrol robot fisik.

## 🤖 Fitur Robot SPERO

### Multi-Mode Operation
- **Primary Control**: Navigasi otonom dengan obstacle avoidance
- **Secondary Control**: Mode cadangan untuk situasi kompleks  
- **Manual Control**: Kontrol keyboard (WASD)
- **R-Mode**: Navigasi waypoint terstruktur (6 waypoint)
- **Navigation Charging**: Auto navigasi ke charging station
- **Recovery Mode**: Sistem pemulihan saat robot stuck

### Dual LiDAR System
- **Gazebo LiDAR**: Sensor simulasi Gazebo
- **Real-World LiDAR**: Sensor fisik via MQTT (A1M8)
- **OR Logic**: Kombinasi data untuk obstacle detection optimal

## 🚀 Quick Start Guide

### 1. Build Workspace
```bash
cd ~/bcr_ws
colcon build
source install/setup.bash
```

### 2. Setup Environment Variables
```bash
export ROS_DOMAIN_ID=0
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/bcr_ws/src/bcr_bot/models
```

## 🌍 Environment & World Files

### Available Worlds

#### 1. **Gedung P (Original)**
- **File**: `worlds/gedung_p.world`
- **Deskripsi**: Model gedung P ukuran asli dengan furnitur lengkap
- **Launch**:
```bash
ros2 launch bcr_bot ign_gedung_p.launch.py
```

#### 2. **Gedung P Scaled**
- **File**: `worlds/gedung_p/gedung_p_scaled.world`
- **Deskripsi**: Model gedung P yang sudah di-scale untuk navigasi robot yang optimal
- **Launch**:
```bash
ros2 launch bcr_bot ign_gedung_p_scaled.launch.py
```

#### 3. **Empty World**
- **File**: `worlds/empty.sdf`
- **Deskripsi**: Environment kosong untuk testing dasar
- **Launch**:
```bash
ros2 launch bcr_bot ign.launch.py world:=empty.sdf
```

#### 4. **Small Warehouse**
- **File**: `worlds/small_warehouse.sdf`
- **Deskripsi**: Environment warehouse kecil untuk testing
- **Launch**:
```bash
ros2 launch bcr_bot ign.launch.py world:=small_warehouse.sdf
```

## 🗺️ Mapping System

### 1. A1M8 LiDAR Mapping (Recommended)
**Real-time mapping menggunakan A1M8 LiDAR fisik**

```bash
# Terminal 1: Start mapping system
ros2 launch bcr_bot a1m8_mapping.launch.py

# Terminal 2: Start robot control
python3 src/bcr_bot/scripts/roam.py

# Terminal 3: Send mapping command
python3 src/bcr_bot/scripts/send_a1m8_no_rear.py
```

### 2. Global Mapping (Gazebo + Real LiDAR)
**Kombinasi data Gazebo dan LiDAR fisik**

```bash
# Terminal 1: Start global mapping
ros2 launch bcr_bot global_mapping.launch.py

# Terminal 2: Start robot control  
python3 src/bcr_bot/scripts/roam.py
```

### 3. Simple Gazebo Mapping
**Mapping menggunakan hanya data Gazebo**

```bash
# Terminal 1: Launch Gazebo with world
ros2 launch bcr_bot ign_gedung_p_scaled.launch.py

# Terminal 2: Start mapping
ros2 launch bcr_bot mapping.launch.py

# Terminal 3: Start robot control
python3 src/bcr_bot/scripts/roam.py
```

### 4. Physical Robot Mapping
**Mapping menggunakan robot fisik saja**

```bash
# Start physical mapping
bash src/bcr_bot/scripts/start_physical_mapping.sh
```

## 🎮 Robot Control

### Manual Control
```bash
# Start main control script
python3 src/bcr_bot/scripts/roam.py

# Control Keys:
# W - Forward
# S - Backward  
# A - Turn Left
# D - Turn Right
# R - Toggle R-mode (waypoint navigation)
# T - Toggle charging navigation
# I - Go to Waypoint 0 (Start)
# Q - Go to Waypoint 5 (MID LEFT) 
# P - Go to Waypoint 2 (Bottom-right)
```

### MQTT Remote Control
```bash
# Send commands via MQTT
python3 test_mqtt_send.py

# Available commands:
# {"command": "start"}
# {"command": "stop"} 
# {"command": "waypoint_i"}
# {"command": "waypoint_q"}
# {"command": "waypoint_p"}
```

## 📍 Waypoint Navigation (R-Mode)

Robot SPERO memiliki 6 waypoint yang telah didefinisikan:

```python
virtual_path = [
    (0.0, 0.0),         # WP0: Start point
    (-12.06, -0.02),    # WP1: Bottom-left  
    (-12.06, -1.79),    # WP2: Bottom-right
    (15.31, -2.10),     # WP3: Top right
    (3.57, -2.10),      # WP4: Mid
    (3.57, 0.0),        # WP5: Mid left
]
```

### R-Mode Commands
```bash
# Normal R-mode (cycle through all waypoints)
Press 'R' in roam.py

# Specific waypoint navigation
Press 'I' - Go to WP0 (Start)
Press 'Q' - Go to WP5 (MID LEFT)
Press 'P' - Go to WP2 (Bottom-right)
```

## 🔧 Configuration Files

### LiDAR Configuration
- `config/a1m8_mapper_params.yaml` - A1M8 LiDAR parameters
- `config/a1m8_no_rear_config.yaml` - A1M8 tanpa rear detection
- `config/mapper_params_physical_lidar.yaml` - Physical LiDAR mapping

### Navigation Configuration  
- `config/nav2_params.yaml` - Nav2 navigation parameters
- `config/amcl_params.yaml` - AMCL localization parameters
- `config/waypoints.yaml` - Waypoint definitions

### Maps
- `config/gedung_p_map.pgm/.yaml` - Gedung P map files
- `config/bcr_map.pgm/.yaml` - BCR map files

## 🔄 Launch Files Quick Reference

### Basic Launch
```bash
# Gazebo + Robot
ros2 launch bcr_bot gazebo.launch.py

# Ignition + Robot  
ros2 launch bcr_bot ign.launch.py

# Gedung P environment
ros2 launch bcr_bot ign_gedung_p.launch.py

# Gedung P scaled environment
ros2 launch bcr_bot ign_gedung_p_scaled.launch.py
```

### Advanced Launch
```bash
# With mapping
ros2 launch bcr_bot mapping.launch.py

# With navigation
ros2 launch bcr_bot nav2.launch.py

# With RViz visualization
ros2 launch bcr_bot rviz.launch.py

# LiDAR visualization
ros2 launch bcr_bot lidar_visual.launch.py
```

## 🧪 Testing & Debugging

### LiDAR Testing
```bash
# Test A1M8 LiDAR
python3 src/bcr_bot/scripts/test_lidar_a1m8.py

# Test dual LiDAR integration
python3 src/bcr_bot/scripts/test_dual_lidar_logic.py

# Test real-world LiDAR
python3 src/bcr_bot/scripts/test_lidar_real_world.py
```

### Robot Control Testing
```bash
# Test basic robot movement
python3 src/bcr_bot/scripts/test_robot.py

# Test obstacle avoidance
python3 src/bcr_bot/scripts/test_obstacle_avoidance.py

# Test charging navigation
python3 src/bcr_bot/scripts/test_nav_charging.py

# Test MQTT communication
python3 src/bcr_bot/scripts/test_mqtt_reception.py
```

### Speed Testing
```bash
# Test different movement speeds
python3 src/bcr_bot/scripts/test_speeds.py

# Test charging speeds
python3 src/bcr_bot/scripts/test_charging_speeds.py
```

## 🛠️ Troubleshooting

### Common Issues & Solutions

#### 1. **LiDAR Connection Issues**
```bash
# Check LiDAR status
python3 src/bcr_bot/scripts/check_mqtt_status.py

# Restart LiDAR system
bash src/bcr_bot/scripts/fix_lidar_issues.sh

# Diagnose LiDAR problems
python3 src/bcr_bot/scripts/diagnose_lidar_issue.py
```

#### 2. **MQTT Connection Problems**
```bash
# Test MQTT connectivity
python3 test_mqtt_connectivity.py

# Debug MQTT communication
python3 test_mqtt_debug.py
```

#### 3. **Mapping Issues**
```bash
# Restart mapping system
bash src/bcr_bot/scripts/restart_mapping_system.sh

# Check mapping status
bash src/bcr_bot/scripts/check_mapping_status.sh

# Diagnose mapping problems
python3 src/bcr_bot/scripts/diagnose_global_mapping.py
```

#### 4. **Robot Stuck/Recovery**
```bash
# Manual recovery
python3 src/bcr_bot/scripts/test_robot_simple.py

# Quick restart roam
bash src/bcr_bot/scripts/quick_restart_roam.sh
```

## 📊 Monitoring & Visualization

### RViz Configurations
- `rviz/entire_setup.rviz` - Complete system view
- `rviz/lidar_visual.rviz` - LiDAR data visualization  
- `rviz/global_mapping.rviz` - Mapping process view
- `rviz/a1m8_mapping.rviz` - A1M8 LiDAR mapping

### Real-time Monitoring
```bash
# Monitor robot status
ros2 topic echo /bcr_bot/cmd_vel

# Monitor LiDAR data
ros2 topic echo /bcr_bot/scan

# Monitor odometry
ros2 topic echo /bcr_bot/odom

# Monitor physical LiDAR
ros2 topic echo /bcr_bot/physical_scan
```

## 🔧 Advanced Configuration

### LiDAR Thresholds
```python
obstacle_threshold = 1.5          # Obstacle detection (m)
emergency_stop_threshold = 0.25   # Emergency stop (m)  
charging_obstacle_threshold = 1.2 # Charging navigation (m)
```

### Movement Parameters
```python
forward_speed = 1.6               # Normal forward speed
turn_speed = 1.5                  # Normal turn speed
charging_forward_speed = 0.5      # Charging navigation speed
charging_turn_speed = 0.8         # Charging turn speed
```

### Battery Management
```python
battery_low_threshold = 11.0      # Low battery voltage (V)
charging_station_pos = (0.0, 0.0) # Charging station coordinates
```

## 🚁 Full System Launch (Recommended Workflow)

### For Gedung P Scaled Environment:
```bash
# Terminal 1: Launch Gazebo environment
ros2 launch bcr_bot ign_gedung_p_scaled.launch.py

# Terminal 2: Launch mapping (if needed)
ros2 launch bcr_bot a1m8_mapping.launch.py

# Terminal 3: Launch robot control
python3 src/bcr_bot/scripts/roam.py

# Terminal 4: Launch RViz for visualization
ros2 launch bcr_bot rviz.launch.py config:=entire_setup.rviz
```

### For Empty Environment Testing:
```bash
# Terminal 1: Launch empty world
ros2 launch bcr_bot ign.launch.py world:=empty.sdf

# Terminal 2: Launch robot control
python3 src/bcr_bot/scripts/roam.py

# Terminal 3: Test specific features
python3 src/bcr_bot/scripts/test_obstacle_avoidance.py
```

## 📞 Support & Documentation

### Additional Documentation
- `scripts/README_robot_control.md` - Detailed robot control guide
- `scripts/README_lidar_setup.md` - LiDAR setup instructions  
- `scripts/TROUBLESHOOTING_MAPPING.md` - Mapping troubleshooting
- `scripts/MQTT_DISCONNECT_FIX.md` - MQTT connection fixes

### Quick Help Scripts
```bash
# Complete system test
python3 src/bcr_bot/scripts/run_complete_test.py

# Quick robot verification
python3 src/bcr_bot/scripts/verify_test_robot.py

# Communication test
python3 src/bcr_bot/scripts/test_communication.py
```

---

**Developed for Robot SPERO Tour Guide Project**  
*Tugas Akhir Simulasi Robot SPERO dan Kontrol Robot SPERO* 