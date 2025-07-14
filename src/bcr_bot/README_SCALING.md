# BCR Bot Scaling System Documentation

## 🎯 Overview

BCR Bot sekarang mendukung sistem scaling yang komprehensif melalui parameter `robot_scale`. Sistem ini memungkinkan Anda untuk mengubah ukuran robot secara proporsional tanpa mengubah bentuk atau karakteristik fisik dasar robot.

## 📏 Parameter Scaling

### Robot Scale Argument
```bash
robot_scale:=<value>
```

### Scaling Rules
- **Dimensi (Length, Width, Height, Radius)**: `original_value × scale`
- **Posisi (Origin XYZ)**: `original_position × scale`
- **Mass**: `original_mass × scale³` (proporsional dengan volume)
- **Inertia**: `original_inertia × scale⁵` (proporsional dengan mass × length²)
- **Torque**: `original_torque × scale²` (proporsional dengan force × length)
- **Sensor Range**: `original_range × scale`

## 🚀 Cara Penggunaan

### 1. Launch dengan Gazebo Classic
```bash
# Robot ukuran penuh (default)
ros2 launch bcr_bot gazebo.launch.py

# Robot setengah ukuran
ros2 launch bcr_bot gazebo.launch.py robot_scale:=0.5

# Robot seperempat ukuran
ros2 launch bcr_bot gazebo.launch.py robot_scale:=0.25

# Robot dua kali ukuran
ros2 launch bcr_bot gazebo.launch.py robot_scale:=2.0
```

### 2. Launch dengan Ignition Gazebo
```bash
# Robot ukuran penuh
ros2 launch bcr_bot ign.launch.py

# Robot setengah ukuran
ros2 launch bcr_bot ign.launch.py robot_scale:=0.5

# Robot mini
ros2 launch bcr_bot ign.launch.py robot_scale:=0.3
```

### 3. Launch dengan RViz saja
```bash
# Robot ukuran penuh
ros2 launch bcr_bot rviz.launch.py

# Robot mini untuk testing
ros2 launch bcr_bot rviz.launch.py robot_scale:=0.2
```

### 4. Spawn Robot dengan Parameter Kustom
```bash
ros2 launch bcr_bot bcr_bot_ign_spawn.launch.py \
    robot_scale:=0.6 \
    position_x:=2.0 \
    position_y:=1.0 \
    camera_enabled:=true \
    two_d_lidar_enabled:=true
```

## 📊 Contoh Scaling Values

| Scale | Robot Size | Use Case |
|-------|------------|----------|
| 2.0   | 200% (Double) | Large warehouse operations |
| 1.5   | 150% (1.5x) | Heavy duty applications |
| 1.0   | 100% (Normal) | Standard operations |
| 0.75  | 75% (Default spawn) | Compact environments |
| 0.5   | 50% (Half) | Small spaces, testing |
| 0.25  | 25% (Quarter) | Miniature testing |
| 0.1   | 10% (Tiny) | Desktop simulation |

## 🔧 Komponen yang Ter-scale

### ✅ Chassis & Structure
- Main chassis (1.0m × 1.0m × 0.5m)
- Front marker
- Roof link
- Conveyor belt (jika enabled)

### ✅ Wheels
- 4 × Trolley wheels (spherical, radius 0.06m)
- 2 × Traction wheels (cylindrical, radius 0.1m)
- 4 × Dummy wheels (visual only)

### ✅ Sensors
- 2D LiDAR (radius 0.075m, height 0.06m)
- IMU (radius 0.04m, height 0.04m)
- Kinect Camera (dengan mesh scaling)
- Stereo Camera (dengan mesh scaling)

### ✅ Physics Properties
- Mass scaling (proporsional dengan volume)
- Inertia scaling (proporsional dengan mass × length²)
- Torque scaling (proporsional dengan force × length)
- Sensor range scaling

## 🎮 Testing & Validation

### 1. Visual Validation
```bash
# Launch dengan scaling berbeda untuk membandingkan
ros2 launch bcr_bot rviz.launch.py robot_scale:=0.5 &
ros2 launch bcr_bot rviz.launch.py robot_scale:=1.0 &
```

### 2. Physics Validation
```bash
# Test robot kecil di Gazebo
ros2 launch bcr_bot gazebo.launch.py robot_scale:=0.3

# Test movement
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=bcr_bot/cmd_vel
```

### 3. Sensor Validation
```bash
# Check sensor topics
ros2 topic list | grep bcr_bot

# Check LiDAR range
ros2 topic echo /bcr_bot/scan --field range_max
```

## 🚨 Pertimbangan Penting

### 1. **Minimum Scale**
- Hindari scale < 0.1 karena dapat menyebabkan instabilitas fisika
- Untuk testing detail, gunakan scale 0.2-0.3

### 2. **Maximum Scale**
- Scale > 2.0 dapat menyebabkan masalah kinerja
- Robot besar membutuhkan lebih banyak computational resources

### 3. **Performance Impact**
- Robot lebih kecil = simulasi lebih cepat
- Robot lebih besar = lebih realistis tapi lebih lambat

### 4. **Sensor Scaling**
- LiDAR range akan ter-scale proportionally
- Camera field of view tetap sama
- Mesh scaling mungkin mempengaruhi visual quality

## 🛠️ Custom Implementation

Jika Anda ingin menambahkan komponen baru yang support scaling:

```xml
<!-- Example: Adding a new scaled component -->
<link name="custom_component">
    <visual>
        <origin xyz="${x_pos * scale} ${y_pos * scale} ${z_pos * scale}" rpy="0 0 0"/>
        <geometry>
            <box size="${width * scale} ${height * scale} ${depth * scale}"/>
        </geometry>
    </visual>
    <collision>
        <origin xyz="${x_pos * scale} ${y_pos * scale} ${z_pos * scale}" rpy="0 0 0"/>
        <geometry>
            <box size="${width * scale} ${height * scale} ${depth * scale}"/>
        </geometry>
    </collision>
    <inertial>
        <mass value="${mass * scale * scale * scale}"/>
        <xacro:box_inertia m="${mass * scale * scale * scale}" 
                          x="${width * scale}" 
                          y="${height * scale}" 
                          z="${depth * scale}"/>
    </inertial>
</link>
```

## 📝 Notes

- Sistem scaling ini telah dioptimalkan untuk menghindari masalah turning stuck
- Semua physics properties di-scale secara fisik yang benar
- Mesh files (.dae) akan ter-scale secara uniform
- Parameter scale dapat dikombinasikan dengan sensor enable/disable options

## 🐛 Troubleshooting

### Problem: Robot terlalu kecil/besar
**Solution**: Adjust robot_scale parameter sesuai kebutuhan

### Problem: Robot tidak bergerak smooth
**Solution**: Pastikan scale tidak terlalu ekstrem (0.1-2.0 range)

### Problem: Sensor tidak bekerja
**Solution**: Check sensor topics dan pastikan sensor enabled

### Problem: Physics unstable
**Solution**: Gunakan scale yang lebih moderate (0.2-1.5) 