# RP-Lidar A1M8 Setup Guide

## Hardware Setup

### 1. Koneksi Fisik RP-Lidar A1M8

#### Koneksi USB:
- **Port USB**: Hubungkan RP-Lidar A1M8 ke Raspberry Pi menggunakan kabel USB
- **Port Default**: `/dev/ttyUSB0` (akan otomatis terdeteksi)
- **Alternatif Port**: Jika ada multiple USB devices, bisa jadi `/dev/ttyUSB1`, `/dev/ttyUSB2`, dll.

#### Koneksi Power:
- **Power Supply**: RP-Lidar A1M8 membutuhkan power 5V
- **Current**: ~300mA saat scanning
- **Power via USB**: Biasanya cukup power dari USB Raspberry Pi
- **External Power**: Jika diperlukan, bisa menggunakan power supply terpisah

### 2. Posisi Mounting LiDAR

#### Rekomendasi Posisi:
- **Tinggi**: 15-20cm dari permukaan tanah
- **Posisi**: Di tengah robot, menghadap ke depan
- **Orientasi**: LiDAR harus tegak lurus dengan permukaan tanah
- **Stabilitas**: Pastikan mounting tidak bergoyang saat robot bergerak

## Software Setup

### 1. Install Dependencies

```bash
# Install Python dependencies
pip3 install rplidar-roboticia
pip3 install pyserial

# Atau menggunakan pip
sudo apt-get update
sudo apt-get install python3-pip
pip3 install rplidar-roboticia pyserial
```

### 2. Verifikasi Koneksi

#### Cek Port USB:
```bash
# Cek device yang terdeteksi
ls -la /dev/ttyUSB*

# Cek permission
ls -la /dev/ttyUSB0

# Jika permission denied, tambahkan user ke group dialout
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyUSB0
```

#### Test Koneksi Manual:
```bash
# Test dengan Python
python3 -c "
from rplidar import RPLidar
lidar = RPLidar('/dev/ttyUSB0')
print('LiDAR Info:', lidar.get_info())
print('LiDAR Health:', lidar.get_health())
lidar.disconnect()
"
```

### 3. Konfigurasi test_robot.py

#### Port Configuration:
```python
# Di test_robot.py, sesuaikan port jika diperlukan
LIDAR_PORT = '/dev/ttyUSB0'  # Ganti jika port berbeda
LIDAR_BAUDRATE = 115200      # Default untuk A1M8
```

#### Troubleshooting Port:
```bash
# Jika port tidak terdeteksi
dmesg | grep ttyUSB

# Reset USB port
sudo usb-reset

# Atau restart USB subsystem
sudo modprobe -r usbserial
sudo modprobe usbserial
```

## Testing dan Verifikasi

### 1. Test Basic LiDAR

```bash
# Jalankan test_robot.py
cd src/bcr_bot/scripts
python3 test_robot.py
```

#### Output yang Diharapkan:
```
[LIDAR] Attempting to connect to LiDAR on /dev/ttyUSB0...
[LIDAR] Connected to LiDAR: {'model': 24, 'firmware': (1, 29), 'hardware': 7, 'serialnumber': '123456789'}
[LIDAR] Health status: {'status': 0, 'error_code': 0}
[LIDAR] LiDAR scanning started successfully
```

### 2. Test Data Transmission

#### Di roam.py, cek apakah data LiDAR diterima:
```
[MQTT] Received LiDAR data from physical robot
[LIDAR] Published physical robot LiDAR data to ROS
```

### 3. Visualisasi Data (Optional)

```bash
# Install visualization tools
pip3 install matplotlib numpy

# Test visualization
python3 -c "
import matplotlib.pyplot as plt
import numpy as np
from rplidar import RPLidar

lidar = RPLidar('/dev/ttyUSB0')
for scan in lidar.iter_scans():
    angles = [np.radians(meas[1]) for meas in scan]
    distances = [meas[2]/1000.0 for meas in scan]
    
    plt.clf()
    plt.polar(angles, distances)
    plt.title('LiDAR Scan')
    plt.pause(0.1)
    
    if len(scan) > 0:
        break
lidar.disconnect()
plt.show()
"
```

## Troubleshooting

### 1. Port Tidak Terdeteksi

**Gejala**: `[ERROR] Failed to initialize LiDAR: [Errno 2] No such file or directory: '/dev/ttyUSB0'`

**Solusi**:
```bash
# Cek apakah device terdeteksi
ls /dev/ttyUSB*

# Jika tidak ada, cek koneksi USB
lsusb

# Reset USB port
sudo usb-reset

# Atau restart sistem
sudo reboot
```

### 2. Permission Denied

**Gejala**: `[Errno 13] Permission denied: '/dev/ttyUSB0'`

**Solusi**:
```bash
# Tambahkan user ke group dialout
sudo usermod -a -G dialout $USER

# Set permission
sudo chmod 666 /dev/ttyUSB0

# Logout dan login kembali
# Atau restart sistem
sudo reboot
```

### 3. LiDAR Tidak Merespons

**Gejala**: Timeout atau tidak ada data

**Solusi**:
```bash
# Cek power supply
# Pastikan LED LiDAR menyala

# Reset LiDAR
# Cabut dan colok ulang USB

# Test dengan baudrate berbeda
# Beberapa LiDAR mungkin memerlukan baudrate 256000
```

### 4. Data Tidak Terkirim ke roam.py

**Gejala**: LiDAR berfungsi tapi roam.py tidak menerima data

**Solusi**:
```bash
# Cek koneksi MQTT
# Pastikan roam.py terhubung ke MQTT broker

# Cek topic subscription
# roam.py harus subscribe ke "robot/physical/scan"

# Test MQTT manual
mosquitto_pub -h codex.petra.ac.id -t "robot/physical/scan" -m '{"test": "data"}'
```

## Spesifikasi RP-Lidar A1M8

### Technical Specifications:
- **Range**: 0.15m - 12m
- **Accuracy**: ±2cm
- **Scan Frequency**: 5.5Hz
- **Angular Resolution**: 1°
- **Field of View**: 360°
- **Power**: 5V, ~300mA
- **Interface**: USB 2.0
- **Operating Temperature**: -10°C to +50°C

### Data Format:
```python
# Raw scan data format
scan = [
    (quality, angle_degrees, distance_mm),
    (quality, angle_degrees, distance_mm),
    ...
]

# Processed data format
ranges = [distance_meters] * 360  # Array 360 elements
```

## Maintenance

### 1. Pembersihan
- Bersihkan lensa LiDAR secara berkala
- Pastikan tidak ada debu atau kotoran
- Hindari goresan pada lensa

### 2. Kalibrasi
- Test akurasi dengan objek yang diketahui jaraknya
- Sesuaikan offset jika diperlukan
- Monitor drift seiring waktu

### 3. Monitoring
- Monitor health status secara berkala
- Cek error codes jika ada masalah
- Backup konfigurasi yang bekerja dengan baik 