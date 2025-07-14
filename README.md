# BCR Bot - Robot Spero ROS2 Workspace

Robot Spero merupakan robot tour guide yang dioperasikan menggunakan ROS2 dengan kemampuan navigasi otonom, penghindaran rintangan, dan kontrol manual. Sistem ini mendukung operasi hybrid antara simulasi Gazebo dan robot fisik.

## Fitur Utama

### 🤖 Multi-Mode Operation System
- **Primary Control Mode**: Navigasi otonom utama dengan algoritma obstacle avoidance
- **Secondary Control Mode**: Mode cadangan untuk situasi navigasi kompleks (timeout 40 detik)
- **Manual Control Mode**: Kontrol manual menggunakan keyboard (WASD keys)
- **R-Mode Navigation**: Navigasi waypoint terstruktur dengan 6 titik waypoint
- **Navigation Charging Mode**: Otomatis navigasi ke charging station saat baterai rendah
- **Recovery Mode**: Sistem pemulihan untuk mengatasi kondisi robot stuck

### 📡 Dual LiDAR Integration
- **Gazebo LiDAR**: Data sensor dari simulasi Gazebo
- **Real-World LiDAR**: Data sensor dari robot fisik melalui MQTT
- **OR Logic**: Menggunakan data yang mendeteksi obstacle terdekat dari kedua sumber

### 🔋 Smart Battery Management
- Auto-aktivasi charging navigation ketika voltage < 11.0V
- Manual charging navigation dengan tombol 'T'
- Monitoring status baterai real-time melalui MQTT

### 🌐 MQTT Integration
- **Topic `robot/physical/cmd_vel`**: Mengirim command velocity ke robot fisik
- **Topic `robot/physical/scan`**: Menerima data LiDAR dari robot fisik
- **Topic `robot/physical/battery`**: Menerima data voltage baterai
- **Topic `robot/lidar/real_world_data`**: Menerima data LiDAR real-world
- **Topic `test/topic`**: Menerima remote commands (start, stop, waypoint)

## Struktur Workspace

```
bcr_ws/
├── src/
│   ├── bcr_bot/           # Main ROS2 package
│   │   ├── scripts/
│   │   │   └── roam.py    # Main robot control script
│   │   ├── launch/        # Launch files
│   │   └── config/        # Configuration files
│   └── physical_robot/    # Physical robot integration
├── install/               # ROS2 install space (auto-generated)
├── build/                 # ROS2 build space (auto-generated)
├── log/                   # ROS2 log files (auto-generated)
└── README.md
```

## Instalasi dan Setup

### Prerequisites
- ROS2 Humble/Galactic
- Python 3.8+
- Gazebo (untuk simulasi)
- MQTT Broker (spin5.petra.ac.id)

### Clone Repository
```bash
git clone <repository-url>
cd bcr_ws
```

### Install Dependencies
```bash
# Install Python dependencies
pip install paho-mqtt

# Install ROS2 dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build
```

### Setup Environment
```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Source workspace
source install/setup.bash
```

## Penggunaan

### 1. Jalankan Robot Control Script
```bash
# Run main robot control
python3 src/bcr_bot/scripts/roam.py
```

### 2. Manual Control Keys
- **W**: Move Forward
- **S**: Move Backward  
- **A**: Turn Left
- **D**: Turn Right
- **T**: Toggle Battery Low (Navigate to charging station)
- **R**: Toggle R-mode (Waypoint navigation)
- **I**: Navigate to Waypoint 0 (Start point)
- **Q**: Navigate to Waypoint 5 (MID LEFT)
- **P**: Navigate to Waypoint 2 (Bottom-right point)

### 3. MQTT Commands
Send JSON commands to topic `test/topic`:
```json
{"command": "start"}         # Start robot
{"command": "stop"}          # Stop robot
{"command": "waypoint_i"}    # Go to Waypoint 0
{"command": "waypoint_q"}    # Go to Waypoint 5
{"command": "waypoint_p"}    # Go to Waypoint 2
```

### 4. Waypoint Navigation
Robot menggunakan 6 waypoint yang telah didefinisikan:
- **Waypoint 0**: (0.0, 0.0) - Start point
- **Waypoint 1**: (-12.06, -0.02) - Bottom-left point
- **Waypoint 2**: (-12.06, -1.79) - Bottom-right point
- **Waypoint 3**: (15.31, -2.10) - Top right
- **Waypoint 4**: (3.57, -2.10) - Mid
- **Waypoint 5**: (3.57, 0.0) - Mid left

## Arsitektur Sistem

### Input Systems
- **Keyboard Input**: Manual control dari operator
- **MQTT Input**: Data dari robot fisik (LiDAR, battery, commands)
- **ROS2 Input**: Data dari simulasi Gazebo

### Processing
- **Mode Selection**: Pemilihan mode operasi otomatis
- **Obstacle Detection**: Deteksi dan penghindaran rintangan
- **Path Planning**: Perencanaan jalur pergerakan

### Output Systems
- **Robot Movement**: Command velocity ke robot
- **Status Messages**: Monitoring dan debugging

## Troubleshooting

### Common Issues

1. **MQTT Connection Failed**
   - Check MQTT broker accessibility
   - Verify network connection
   - Check firewall settings

2. **Robot Stuck in Recovery Mode**
   - Check LiDAR data quality
   - Verify obstacle detection thresholds
   - Manual intervention with keyboard

3. **Charging Navigation Issues**
   - Check battery voltage readings
   - Verify charging station coordinates
   - Monitor obstacle detection during charging nav

### Debug Commands
```bash
# Check ROS2 topics
ros2 topic list

# Monitor cmd_vel
ros2 topic echo /bcr_bot/cmd_vel

# Check LiDAR data
ros2 topic echo /bcr_bot/scan
```

## Konfigurasi

### LiDAR Thresholds
```python
obstacle_threshold = 1.5          # Obstacle detection distance
emergency_stop_threshold = 0.25   # Emergency stop distance
charging_obstacle_threshold = 1.2 # Charging navigation threshold
```

### Battery Settings
```python
battery_low_threshold = 11.0      # Low battery voltage
charging_station_pos = (0.0, 0.0) # Charging station coordinates
```

## Kontribusi

1. Fork repository
2. Create feature branch
3. Commit changes
4. Push to branch
5. Create Pull Request

## Lisensi

[Sesuaikan dengan lisensi yang digunakan]

## Kontak

[Tambahkan informasi kontak atau tim pengembang] 