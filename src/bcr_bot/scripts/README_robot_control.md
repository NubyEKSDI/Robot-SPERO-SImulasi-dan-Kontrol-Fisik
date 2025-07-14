# Robot Physical Control System

## Overview
Sistem kontrol robot physical yang telah diperbaiki untuk mengikuti kecepatan dari Gazebo seperti di R-mode. Robot sekarang dapat menerima perintah dari semua mode (Primary, Secondary, R-mode, Navigation Charging, Manual) dan mengikuti kecepatan yang dikirim dari roam.py.

## Fitur Utama

### 1. Multi-Mode Support
- **Manual Control**: Kontrol manual melalui keyboard
- **Primary Control**: Mode autonomous roaming utama
- **Secondary Control**: Mode obstacle avoidance sekunder
- **R-Mode**: Mode navigasi waypoint
- **Navigation Charging**: Mode navigasi ke charging station

### 2. Velocity Mapping
Setiap mode memiliki mapping kecepatan yang berbeda:

#### R-Mode (Agresif)
- **Linear Movement**:
  - 0.1-0.3 m/s → 0.5-0.6 speed factor (increased)
  - 0.3-0.8 m/s → 0.6-0.9 speed factor (increased)
  - 0.8-1.5 m/s → 0.9-1.0 speed factor (increased)
  - 1.5+ m/s → 1.0 speed factor (capped at maximum)
- **Rotating (Special Handling)**:
  - 0.1-0.5 m/s → 0.5-0.66 speed factor (lebih cepat)
  - 0.5-1.0 m/s → 0.66-0.86 speed factor
  - 1.0-1.5 m/s → 0.86-0.96 speed factor
  - 1.5+ m/s → 0.96-1.0 speed factor

#### Primary Control (Moderat)
- 0.1-0.5 m/s → 0.1-0.26 speed factor
- 0.5-1.0 m/s → 0.26-0.51 speed factor
- 1.0-2.0 m/s → 0.51-0.71 speed factor
- 2.0+ m/s → 0.71-1.0 speed factor

#### Secondary Control (Moderat)
- 0.1-0.5 m/s → 0.15-0.27 speed factor
- 0.5-1.0 m/s → 0.27-0.47 speed factor
- 1.0-2.0 m/s → 0.47-0.72 speed factor
- 2.0+ m/s → 0.72-1.0 speed factor

#### Navigation Charging (Tinggi - agar roda berputar)
- < 0.1 m/s → 0.25 speed factor (minimum threshold)
- 0.1-0.3 m/s → 0.3-0.4 speed factor
- 0.3-0.6 m/s → 0.4-0.52 speed factor
- 0.6-1.0 m/s → 0.52-0.64 speed factor
- 1.0+ m/s → 0.64-1.0 speed factor

### 3. MQTT Topics
- `robot/physical/cmd_vel`: Manual control commands
- `robot/primary/cmd_vel`: Primary mode commands
- `robot/secondary/cmd_vel`: Secondary mode commands
- `robot/rmode/cmd_vel`: R-mode commands
- `robot/nav_charging/cmd_vel`: Navigation charging commands

### 4. Emergency Stop
Robot akan berhenti otomatis ketika menerima:
- Perintah dengan mode "EMERGENCY STOP"
- Perintah dengan mode "ROBOT IDLE"
- Perintah dengan mode "ROBOT STOPPED"
- Perintah dengan linear_x dan angular_z = 0

## Cara Penggunaan

### 1. Menjalankan Robot Controller
```bash
cd src/bcr_bot/scripts
python3 test_robot.py
```

### 2. Status Display
Robot akan menampilkan status setiap 5 detik:
```
============================================================
ROBOT STATUS - 14:30:25
============================================================
Current Mode: PRIMARY CONTROL
Motor Status: FORWARD (speed: 0.45)
Time since last command: 0.2s
MQTT Connected: YES
============================================================
```

### 3. Debug Information
Robot akan menampilkan informasi debug untuk setiap perintah:
```
[MQTT] Received command from robot/physical/cmd_vel: {'linear': {'x': 1.6, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'mode': 'Primary control: Moving forward'}
[DEBUG] Processing velocity command from PRIMARY CONTROL
[DEBUG] linear_x: 1.6, angular_z: 0.0
[DEBUG] Calculated speed_factor: 0.710 for mode: PRIMARY CONTROL
[DEBUG] Command type: LINEAR (linear_x: 1.600, angular_z: 0.000)
[DEBUG] Moving forward with linear_x=1.6, speed_factor=0.710
[STATUS] Mode: PRIMARY CONTROL, Motors: FORWARD (speed: 0.71)
```

**Contoh untuk R-mode rotating:**
```
[MQTT] Received command from robot/rmode/cmd_vel: {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 1.5}, 'mode': 'R-mode: Rotating to face waypoint'}
[DEBUG] Processing velocity command from R-MODE
[DEBUG] linear_x: 0.0, angular_z: 1.5
[DEBUG] Calculated speed_factor: 0.960 for mode: R-MODE (ROTATING)
[DEBUG] Command type: ROTATING (linear_x: 0.000, angular_z: 1.500)
[DEBUG] Using speed coefficient: 0.5 for R-MODE
[DEBUG] Turning right with angular_z=1.5, speed_factor=0.96
[STATUS] Mode: R-MODE, Motors: TURNING RIGHT (speed: 0.96)
```

**Contoh untuk R-mode linear movement:**
```
[MQTT] Received command from robot/rmode/cmd_vel: {'linear': {'x': 0.8, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'mode': 'R-mode: Moving towards waypoint'}
[DEBUG] Processing velocity command from R-MODE
[DEBUG] linear_x: 0.8, angular_z: 0.0
[DEBUG] Calculated speed_factor: 0.900 for mode: R-MODE
[DEBUG] Command type: LINEAR (linear_x: 0.800, angular_z: 0.000)
[DEBUG] Moving forward with linear_x=0.8, speed_factor=0.900
[STATUS] Mode: R-MODE, Motors: FORWARD (speed: 0.90)
```

**Contoh untuk Navigation Charging:**
```
[MQTT] Received command from robot/physical/cmd_vel: {'linear': {'x': 0.8, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'mode': 'CHARGING NAV: Moving to charging station'}
[DEBUG] Processing velocity command from NAVIGATION CHARGING
[DEBUG] linear_x: 0.8, angular_z: 0.0
[DEBUG] Calculated speed_factor: 0.640 for mode: NAVIGATION CHARGING
[DEBUG] Command type: LINEAR (linear_x: 0.800, angular_z: 0.000)
[DEBUG] Moving forward with linear_x=0.8, speed_factor=0.640
[STATUS] Mode: NAVIGATION CHARGING, Motors: FORWARD (speed: 0.64)
```

### R-mode Backward Command
```
[DEBUG] Processing velocity command from R-MODE
[DEBUG] linear_x: -0.500, angular_z: 0.000
[DEBUG] Calculated speed_factor: 1.055 for mode: R-MODE
[DEBUG] Command type: LINEAR (linear_x: -0.500, angular_z: 0.000)
[DEBUG] Using speed coefficient: 0.5 for R-MODE
[DEBUG] R-MODE BACKWARD: Using enhanced speed coefficient: 0.6
[DEBUG] R-MODE BACKWARD: Enhanced speed for faster backward movement (speed_factor: 1.055)
[DEBUG] Moving backward with linear_x=-0.5, speed_factor=1.055
```

### R-mode Forward Command
```
[DEBUG] Processing velocity command from R-MODE
[DEBUG] linear_x: 0.500, angular_z: 0.000
[DEBUG] Calculated speed_factor: 0.955 for mode: R-MODE
[DEBUG] Command type: LINEAR (linear_x: 0.500, angular_z: 0.000)
[DEBUG] Using speed coefficient: 0.5 for R-MODE
[DEBUG] R-MODE FORWARD: Increased speed for faster movement (speed_factor: 0.955)
[DEBUG] Moving forward with linear_x=0.5, speed_factor=0.955
```

## Konfigurasi Hardware

### Pin Mappings
```python
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
```

### Constants
```python
PCA_FREQ = 1500        # PCA9685 frequency
fullSpeed = 0xFFFF     # 65535 (full speed)
speedCoef = 0.3        # Speed reduction factor
```

## Troubleshooting

### 1. Motor Tidak Berputar
- Periksa koneksi PCA9685
- Periksa koneksi motor
- Periksa power supply
- Jalankan test individual motors

### 2. MQTT Tidak Terhubung
- Periksa koneksi internet
- Periksa server MQTT (codex.petra.ac.id)
- Periksa port 1883

### 3. Kecepatan Tidak Sesuai
- Periksa mapping kecepatan di `calculate_speed_factor()`
- Sesuaikan `speedCoef` jika diperlukan
- Periksa nilai `fullSpeed`

## Perubahan dari Versi Sebelumnya

1. **Velocity Mapping**: Sekarang menggunakan kecepatan aktual dari Gazebo
2. **Multi-Mode Support**: Mendukung semua mode dari roam.py
3. **Status Tracking**: Menampilkan status robot secara real-time
4. **Emergency Stop**: Penanganan emergency stop yang lebih baik
5. **Debug Information**: Informasi debug yang lebih detail

## Integrasi dengan roam.py

Robot physical sekarang akan mengikuti kecepatan dari Gazebo yang dikirim oleh roam.py:

1. **Primary Mode**: Robot akan bergerak dengan kecepatan yang sesuai dengan primary control
2. **Secondary Mode**: Robot akan menghindari obstacle dengan kecepatan yang sesuai
3. **R-Mode**: Robot akan mengikuti waypoint dengan kecepatan yang sesuai
4. **Navigation Charging**: Robot akan menuju charging station dengan kecepatan yang aman
5. **Manual Mode**: Robot akan merespons input keyboard dengan kecepatan yang sesuai 

# Robot Control System Documentation

## Overview
This system consists of two main components:
1. **roam.py** - ROS2 node running in Gazebo simulation that sends velocity commands
2. **test_robot.py** - Physical robot controller that receives commands via MQTT and controls motors

## MQTT Topics

### Command Topics (roam.py → test_robot.py)
- `robot/physical/cmd_vel` - Manual control commands
- `robot/primary/cmd_vel` - Primary control (auto-roaming) commands  
- `robot/secondary/cmd_vel` - Secondary control (obstacle avoidance) commands
- `robot/rmode/cmd_vel` - R-mode (waypoint navigation) commands
- `robot/nav_charging/cmd_vel` - Navigation charging commands

### Status Topics (test_robot.py → roam.py)
- `robot/physical/status` - Robot status updates
- `robot/physical/battery` - Battery voltage data
- `robot/physical/scan` - LiDAR scan data

### Control Commands (External → roam.py)
- `test/topic` - External control commands

## Control Methods

### 1. Keyboard Control (roam.py)
- **W/S/A/D**: Manual movement control
- **T**: Toggle battery low (navigate to charging station)
- **R**: Toggle R-mode (continuous waypoint navigation)
- **I**: Navigate to Waypoint 0 (Start point)
- **Q**: Navigate to Waypoint 5 (MID LEFT)
- **P**: Navigate to Waypoint 2 (Bottom-right point)

### 2. MQTT Commands (External Control)
Send JSON messages to topic `test/topic`:

#### Waypoint Navigation
```json
{"command": "waypoint_i"}  // Go to Waypoint 0 (Start point)
{"command": "waypoint_q"}  // Go to Waypoint 5 (MID LEFT)  
{"command": "waypoint_p"}  // Go to Waypoint 2 (Bottom-right point)
```

#### Robot Control
```json
{"command": "stop"}   // Stop robot and pause all modes
{"command": "start"}  // Resume primary control
```

## Waypoint System

### Virtual Path Coordinates
```python
virtual_path = [
    (0.0, 0.0),        # Waypoint 0 - Start point
    (-12.06, 0.03),    # Waypoint 1 - Bottom-left point
    (-12.06, -1.79),   # Waypoint 2 - Bottom-right point
    (15.31, -2.10),    # Waypoint 3 - TOP RIGHT
    (3.57, -2.10),     # Waypoint 4 - MID
    (3.57, 0.0),       # Waypoint 5 - MID LEFT
]
```

### Waypoint Navigation Modes
1. **Specific Waypoint Navigation** (i, q, p keys or MQTT commands)
   - Robot navigates to target waypoint and stops
   - Returns to primary control after reaching destination
   
2. **Continuous R-mode** (R key)
   - Robot continuously follows waypoint sequence
   - Loops through all waypoints indefinitely

## Velocity Mappings

### R-mode Speed Factors
- **Linear movement (forward)**: 0.8-1.0 speed factor (highly enhanced for very fast wheel rotation)
- **Linear movement (backward)**: 0.85-1.0 speed factor (highly enhanced for very fast backward movement)
- **Rotating**: 0.3-1.0 speed factor (further reduced for very precise turning)
- **Speed coefficient (forward)**: 0.5 (higher than other modes for faster physical robot response)
- **Speed coefficient (backward)**: 0.6 (even higher for faster backward movement)
- **Minimum threshold**: Ensures wheels turn even with low velocities

### Navigation Charging Speed Factors  
- **Linear movement**: 0.25-1.0 speed factor
- **Minimum speed**: 0.25 to ensure wheels turn
- **Enhanced mapping**: Higher speeds for better responsiveness

### Primary/Secondary Control Speed Factors
- **Primary**: 0.1-1.0 speed factor (moderate mapping)
- **Secondary**: 0.15-1.0 speed factor (moderate mapping)
- **Speed coefficient**: 0.4 (standard for normal operation)

## Debug Output Examples

### R-mode Rotating Command
```
[DEBUG] Processing velocity command from R-MODE
[DEBUG] linear_x: 0.000, angular_z: 1.500
[DEBUG] Calculated speed_factor: 0.580 for mode: R-MODE (ROTATING)
[DEBUG] Command type: ROTATING (linear_x: 0.000, angular_z: 1.500)
[DEBUG] Using speed coefficient: 0.5 for R-MODE
[DEBUG] R-MODE ROTATING: FURTHER REDUCED speed for precise turning (speed_factor: 0.580)
[DEBUG] Turning left with angular_z=1.5, speed_factor=0.58
```

### R-mode Forward Command
```
[DEBUG] Processing velocity command from R-MODE
[DEBUG] linear_x: 0.500, angular_z: 0.000
[DEBUG] Calculated speed_factor: 1.055 for mode: R-MODE
[DEBUG] Command type: LINEAR (linear_x: 0.500, angular_z: 0.000)
[DEBUG] Using speed coefficient: 0.5 for R-MODE
[DEBUG] R-MODE FORWARD: HIGHLY ENHANCED speed for fast forward movement (speed_factor: 1.055)
[DEBUG] Moving forward with linear_x=0.5, speed_factor=1.055
```

### R-mode Backward Command
```
[DEBUG] Processing velocity command from R-MODE
[DEBUG] linear_x: -0.500, angular_z: 0.000
[DEBUG] Calculated speed_factor: 1.105 for mode: R-MODE
[DEBUG] Command type: LINEAR (linear_x: -0.500, angular_z: 0.000)
[DEBUG] Using speed coefficient: 0.5 for R-MODE
[DEBUG] R-MODE BACKWARD: Using enhanced speed coefficient: 0.6
[DEBUG] R-MODE BACKWARD: HIGHLY ENHANCED speed for fast backward movement (speed_factor: 1.105)
[DEBUG] Moving backward with linear_x=-0.5, speed_factor=1.105
```

### Navigation Charging Command
```
[DEBUG] Processing velocity command from NAVIGATION CHARGING
[DEBUG] linear_x: 0.500, angular_z: 0.000
[DEBUG] Calculated speed_factor: 0.640 for mode: NAVIGATION CHARGING
[DEBUG] Command type: LINEAR (linear_x: 0.500, angular_z: 0.000)
[DEBUG] Using speed coefficient: 0.4 for NAVIGATION CHARGING
[DEBUG] Moving forward with linear_x=0.5, speed_factor=0.64
```

## Usage Instructions

### Starting the System
1. **Start roam.py** (Gazebo simulation):
   ```bash
   cd /home/nuby/bcr_ws
   source install/setup.bash
   ros2 run bcr_bot roam
   ```

2. **Start test_robot.py** (Physical robot):
   ```bash
   cd /home/nuby/bcr_ws/src/bcr_bot/scripts
   python3 test_robot.py
   ```

### Testing MQTT Commands
Use MQTT client to send commands to topic `test/topic`:

```bash
# Navigate to waypoint 0
mosquitto_pub -h codex.petra.ac.id -t "test/topic" -m '{"command": "waypoint_i"}'

# Navigate to waypoint 5  
mosquitto_pub -h codex.petra.ac.id -t "test/topic" -m '{"command": "waypoint_q"}'

# Navigate to waypoint 2
mosquitto_pub -h codex.petra.ac.id -t "test/topic" -m '{"command": "waypoint_p"}'

# Stop robot
mosquitto_pub -h codex.petra.ac.id -t "test/topic" -m '{"command": "stop"}'

# Start robot
mosquitto_pub -h codex.petra.ac.id -t "test/topic" -m '{"command": "start"}'
```

## System Features

### Multi-mode Support
- **Manual Control**: Direct keyboard input (highest priority)
- **Navigation Charging**: Battery low navigation (bypasses MQTT pause)
- **R-mode**: Waypoint navigation (keyboard or MQTT)
- **Secondary Control**: Obstacle avoidance
- **Primary Control**: Auto-roaming (lowest priority)

### Emergency Handling
- **Emergency Stop**: Immediate motor shutdown
- **Obstacle Detection**: Automatic avoidance
- **Stuck Recovery**: Backup and turn behavior
- **Battery Monitoring**: Automatic charging navigation

### MQTT Integration
- **Real-time Communication**: Between simulation and physical robot
- **Status Monitoring**: Battery, position, and system status
- **External Control**: Remote command capability
- **Fault Tolerance**: Graceful handling of connection issues 