# Navigation Charging Mode Fixes and Speed Configuration

## Overview
Perbaikan untuk masalah navigation charging mode yang tidak berjalan ketika tombol 't' ditekan, dan konfigurasi kecepatan yang dioptimalkan untuk mencegah stuck/loop.

## Masalah yang Diperbaiki

### 1. Navigation Charging Mode Tidak Berjalan
**Masalah**: Ketika tombol 't' ditekan, robot langsung kembali ke primary mode tanpa menjalankan navigation charging.

**Penyebab**: 
- Manual mode timeout mengganggu navigation charging mode
- Konflik antara manual mode dan navigation charging mode
- Logika timer_callback tidak memprioritaskan navigation charging

**Solusi**:
- Menambahkan kondisi `not self.nav_charging_mode` pada manual mode timeout
- Mematikan manual mode saat navigation charging aktif
- Memprioritaskan navigation charging mode di timer_callback

### 2. Kecepatan dan Obstacle Detection

**Masalah**: 
- Rotasi terlalu cepat untuk navigation charging
- Obstacle detection terlalu sensitif menyebabkan stuck/loop
- Kecepatan waypoint navigation tidak sesuai

**Solusi**:
- Memperlambat rotasi navigation charging
- Mengurangi threshold obstacle detection
- Menyesuaikan kecepatan waypoint navigation

## Perubahan Detail

### 1. roam.py - Navigation Charging Configuration

```python
# Kecepatan Navigation Charging (diperlambat)
self.charging_forward_speed = 0.5  # Same as R-mode forward speed
self.charging_turn_speed = 0.8     # Slower rotation (reduced from 1.5)

# Obstacle Detection (dikurangi untuk mencegah stuck/loop)
self.charging_obstacle_threshold = 0.8      # Smaller detection area
self.charging_emergency_stop_threshold = 0.3 # Smaller emergency stop

# R-mode Kecepatan (disamakan dengan primary mode)
self.r_mode_forward_speed = 1.6  # Same as primary mode (changed from 0.5)
```

### 2. roam.py - Logic Fixes

```python
# Keyboard Listener - Prevent timeout interference
if self.manual_mode and not self.nav_charging_mode and (time.time() - self.last_input_time > 5.0):
    # Only timeout if not in navigation charging mode

# Handle Key - Proper mode management
if key == 't':
    if not self.nav_charging_mode:
        self.manual_mode = False      # Prevent manual mode interference
        self.primary_active = False   # Disable primary control
        self.secondary_control_active = False
        self.r_mode = False

# Timer Callback - Navigation charging priority
if self.nav_charging_mode:
    # Use charging-specific thresholds
    charging_obstacle_detected = self.lidar_min_distance < self.charging_obstacle_threshold
    charging_lidar_too_close = self.lidar_min_distance < self.charging_emergency_stop_threshold
```

### 3. test_robot.py - Speed Configuration

```python
# Navigation Charging Speed Mapping (diperlambat)
elif "CHARGING NAV" in mode or "Navigation charging" in mode:
    # Navigation charging uses slower, more controlled speed mapping
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
```

## Konfigurasi Kecepatan Final

### Navigation Charging Mode
- **Forward Speed**: 0.5 m/s (sama dengan R-mode)
- **Turn Speed**: 0.8 rad/s (diperlambat dari 1.5)
- **Obstacle Threshold**: 0.8 m (dikurangi dari 1.4)
- **Emergency Stop**: 0.3 m (dikurangi dari 0.4)
- **Wheel Speed Factor**: 0.2-1.0 (diperlambat)

### R-Mode (Waypoint Navigation)
- **Forward Speed**: 1.6 m/s (sama dengan primary mode)
- **Turn Speed**: 1.5 rad/s (tetap sama)
- **Wheel Speed Factor**: Sesuai primary mode

### Primary Mode
- **Forward Speed**: 1.6 m/s
- **Turn Speed**: 1.5 rad/s
- **Obstacle Threshold**: 1.4 m
- **Emergency Stop**: 0.4 m

## Test Scripts

### 1. test_nav_charging_simple.py
- Test logika navigation charging tanpa ROS2
- Verifikasi mode switching yang benar

### 2. test_speeds.py
- Test konfigurasi kecepatan dan threshold
- Perbandingan antar mode

### 3. test_charging_speeds.py
- Test kecepatan roda untuk navigation charging
- Verifikasi speed factor mapping

## Cara Penggunaan

### 1. Menjalankan Navigation Charging
```bash
# Di terminal roam.py
# Tekan 't' untuk mengaktifkan navigation charging
# Robot akan menuju ke charging station (0,0)
# Tekan 't' lagi untuk membatalkan
```

### 2. Menjalankan Test Scripts
```bash
# Test logika navigation charging
python3 src/bcr_bot/scripts/test_nav_charging_simple.py

# Test konfigurasi kecepatan
python3 src/bcr_bot/scripts/test_speeds.py

# Test kecepatan roda
python3 src/bcr_bot/scripts/test_charging_speeds.py
```

## Hasil yang Diharapkan

1. **Navigation Charging Berfungsi**: Tombol 't' akan mengaktifkan navigation charging mode tanpa gangguan
2. **Tidak Ada Stuck/Loop**: Obstacle detection yang lebih kecil mencegah robot terjebak
3. **Kecepatan Optimal**: Rotasi lebih lambat untuk presisi, forward speed sesuai untuk efisiensi
4. **Mode Switching Lancar**: Transisi antar mode berjalan dengan baik

## Troubleshooting

### Jika Navigation Charging Masih Tidak Berjalan
1. Periksa apakah `nav_charging_mode` diset ke `True`
2. Periksa apakah `primary_active` diset ke `False`
3. Periksa apakah `manual_mode` diset ke `False`

### Jika Robot Masih Stuck/Loop
1. Periksa threshold obstacle detection
2. Periksa kecepatan rotasi
3. Periksa logika obstacle avoidance

### Jika Kecepatan Terlalu Lambat/Cepat
1. Sesuaikan `charging_forward_speed` di roam.py
2. Sesuaikan `charging_turn_speed` di roam.py
3. Sesuaikan speed factor mapping di test_robot.py 