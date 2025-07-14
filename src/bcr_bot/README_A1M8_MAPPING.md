# A1M8 LiDAR Mapping System

## Overview
Sistem mapping real-time menggunakan RPLIDAR A1M8 untuk membangun peta ruangan dengan SLAM (Simultaneous Localization and Mapping).

## Features

### 🔴 **A1M8 LiDAR Visualization**
- Real-time laser scan dalam warna merah
- Resolusi tinggi 720 titik per putaran (0.5° increment)
- Range detection: 15cm - 12m

### 🟡 **Robot Orientation Indicators**
- **Yellow Footprint**: Menunjukkan posisi dan bentuk robot
- **Red Arrow**: Indikator arah depan robot yang jelas
- **TF Frames**: Visualisasi koordinat robot (base_link, laser, robot_front)
- **Robot Pose**: Panah merah dengan kovarians untuk lokalisasi

### ⚪ **Persistent Scan Points**
- Titik scan persisten dalam warna putih
- Fade effect selama 30 detik
- Maksimal 50,000 titik terakumulasi

### 🟢 **Robot Path Tracking**
- Jalur robot dalam warna hijau
- Tracking pergerakan real-time
- Visualisasi trajectory lengkap

### 🗺️ **SLAM Mapping**
- Real-time map building
- Loop closure detection
- Occupancy grid mapping
- Map saving capabilities

## Quick Start

### 1. Visualization Mode (Default)
```bash
./run_lidar_visual.sh
```

**Features:**
- Real-time A1M8 LiDAR (red dots)
- Persistent scan points (white dots)
- Robot orientation indicators
- Robot path tracking
- Enhanced 720-point resolution

### 2. SLAM Mapping Mode
```bash
./run_lidar_visual.sh --mapping
```

**Features:**
- All visualization features PLUS:
- Real-time SLAM mapping
- Map building and loop closure
- Enhanced robot orientation display
- Map saving capabilities

## Robot Orientation Indicators

### 🟡 **Yellow Footprint**
- Menunjukkan posisi dan ukuran robot
- Bentuk polygon sesuai dimensi robot
- Update real-time mengikuti pergerakan

### 🔴 **Red Arrow (Robot Pose)**
- Panah merah menunjukkan arah depan robot
- Kovarians menunjukkan tingkat kepercayaan lokalisasi
- Visualisasi orientasi yang jelas

### 📍 **TF Frames**
- **base_link**: Pusat robot
- **laser**: Posisi sensor A1M8
- **robot_front**: Frame khusus untuk arah depan
- **base_footprint**: Proyeksi robot ke lantai

### 🟢 **Green Path**
- Jalur pergerakan robot
- Tracking trajectory real-time
- Visualisasi rute yang telah dilalui

## Configuration Files

### `a1m8_mapper_params.yaml`
- SLAM parameters optimized for A1M8
- Laser range: 0.2m - 12m
- Enhanced queue settings
- Throttling for stability

### `a1m8_mapping.rviz`
- RViz configuration for mapping
- Robot orientation displays
- Enhanced visualization settings
- Top-down orthographic view

## Troubleshooting

### Robot Orientation Issues
If you can't see which direction is front:

1. **Check TF Frames**: Ensure robot_front frame is visible
2. **Enable Robot Pose**: Red arrow should point to front
3. **Yellow Footprint**: Should show robot shape and orientation
4. **Restart System**: If indicators not showing

### Common Issues

1. **No robot orientation visible**
   ```bash
   # Check if all transforms are being published
   ros2 run tf2_tools view_frames
   ```

2. **Robot footprint not showing**
   ```bash
   # Check if scan_to_pointcloud is running
   ros2 topic echo /robot_footprint
   ```

3. **Path not tracking**
   ```bash
   # Check robot path topic
   ros2 topic echo /robot_path
   ```

## Advanced Usage

### Saving Maps
```bash
# In mapping mode, save map when satisfied
ros2 run nav2_map_server map_saver_cli -f my_map
```

### Custom Robot Footprint
Edit `scan_to_pointcloud.py` to modify robot footprint shape:
```python
# Modify footprint vertices for your robot size
footprint_vertices = [
    [0.25, 0.25],   # Front-left
    [0.25, -0.25],  # Front-right
    [-0.25, -0.25], # Back-right
    [-0.25, 0.25],  # Back-left
    [0.35, 0.0]     # Front indicator (arrow)
]
```

## System Requirements

- ROS2 Humble
- SLAM Toolbox
- RViz2
- A1M8 LiDAR connected via Raspberry Pi
- MQTT broker running

## Performance Optimization

### Mapping Mode
- Reduced publish frequency (6.7Hz)
- Optimized queue sizes
- Throttled scan processing

### Visualization Mode
- High-frequency updates (20Hz)
- Real-time visualization
- Enhanced resolution

## Debug Commands

```bash
# Check A1M8 data
ros2 topic echo /scan

# Check robot transforms
ros2 run tf2_tools view_frames

# Check robot footprint
ros2 topic echo /robot_footprint

# Check robot path
ros2 topic echo /robot_path

# Check robot pose
ros2 topic echo /robot_pose
```

## Success Indicators

✅ **Red laser scan dots**: A1M8 data flowing  
✅ **Yellow footprint**: Robot position visible  
✅ **Red arrow**: Front direction clear  
✅ **Green path**: Robot movement tracked  
✅ **White dots**: Persistent scan points  
✅ **Map building**: SLAM working  

## Support

For issues or questions:
1. Check troubleshooting section
2. Verify all topics are publishing
3. Restart system if needed
4. Check MQTT connection to Raspberry Pi 