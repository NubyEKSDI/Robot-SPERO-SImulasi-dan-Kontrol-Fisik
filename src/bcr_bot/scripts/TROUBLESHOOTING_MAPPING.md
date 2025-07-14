# Troubleshooting BCR Bot Mapping System

## Masalah: Lidar tidak masuk data ke roam.py setelah restart

### Gejala
- `roam.py` dimatikan, `test_robot.py` masih berjalan
- `roam.py` dinyalakan lagi, tapi data lidar tidak masuk
- Harus restart `test_robot.py` untuk bisa konek lagi

### Penyebab
Masalah ini terjadi karena:
1. Ketika `roam.py` dimatikan, `test_robot.py` masih mengirim data lidar
2. Ketika `roam.py` dinyalakan lagi, ada state yang tidak reset dengan benar
3. MQTT connection tidak dihandle dengan baik saat reconnect

### Solusi yang Sudah Diimplementasi

#### 1. Auto-Reconnect di roam.py
- `roam.py` sekarang memiliki mekanisme reconnect yang lebih robust
- Otomatis reset state lidar data saat reconnect
- Mengirim flag `reconnect: true` ke `test_robot.py`

#### 2. Auto-Restart Lidar di test_robot.py
- `test_robot.py` mendeteksi reconnect dari `roam.py`
- Otomatis restart lidar streaming saat reconnect terdeteksi
- Handle MQTT disconnect/reconnect dengan lebih baik

#### 3. Script Restart Otomatis

**Untuk restart roam.py:**
```bash
./src/bcr_bot/scripts/restart_mapping_system.sh roam
```

**Untuk restart test_robot.py di Raspberry Pi:**
```bash
./src/bcr_bot/scripts/restart_test_robot_rpi.sh
```

**Untuk restart keduanya:**
```bash
./src/bcr_bot/scripts/restart_mapping_system.sh both
```

### Cara Kerja Solusi

1. **Saat roam.py restart:**
   - MQTT client reconnect otomatis
   - Reset semua state lidar data
   - Kirim pesan "ready" dengan flag `reconnect: true`

2. **Saat test_robot.py menerima reconnect:**
   - Deteksi flag `reconnect: true`
   - Stop lidar streaming yang sedang berjalan
   - Restart lidar streaming untuk roam.py yang baru

3. **Verifikasi koneksi:**
   - roam.py: "MQTT: Connected to broker"
   - test_robot.py: "MQTT: Connected successfully"
   - test_robot.py: "Detected roam.py reconnection!"
   - test_robot.py: "Restarting lidar streaming for roam.py..."
   - roam.py: "📡 Real-World LiDAR: Front=X.XXm"

### Troubleshooting Manual

Jika auto-reconnect tidak bekerja:

1. **Restart roam.py:**
   ```bash
   pkill -f "roam.py"
   sleep 2
   cd /home/nuby/bcr_ws
   source install/setup.bash
   ros2 run bcr_bot roam
   ```

2. **Restart test_robot.py di Raspberry Pi:**
   ```bash
   pkill -f "test_robot.py"
   sleep 3
   cd /home/pi/bcr_ws
   source install/setup.bash
   python3 src/bcr_bot/scripts/test_robot.py
   ```

### Checklist Verifikasi

- [ ] MQTT broker (codex.petra.ac.id) dapat diakses
- [ ] Lidar device (/dev/ttyUSB0) terdeteksi
- [ ] roam.py menampilkan "MQTT: Connected to broker"
- [ ] test_robot.py menampilkan "MQTT: Connected successfully"
- [ ] test_robot.py menampilkan "LiDAR initialized successfully"
- [ ] roam.py menampilkan "📡 Real-World LiDAR: Front=X.XXm"
- [ ] Data lidar berubah saat robot bergerak

### Log Debug

Untuk debug lebih detail, cek log berikut:

**roam.py:**
- `MQTT: Connected to broker`
- `MQTT: Sent ready message to LiDAR systems (reconnect)`
- `📡 Real-World LiDAR: Front=X.XXm, Left=X.XXm, Right=X.XXm`

**test_robot.py:**
- `MQTT: Connected successfully`
- `Detected roam.py reconnection!`
- `Restarting lidar streaming for roam.py...`
- `LiDAR streaming started`

### Tips Tambahan

1. **Jangan matikan test_robot.py** jika hanya ingin restart roam.py
2. **Gunakan script restart** untuk memastikan clean restart
3. **Monitor log** untuk memastikan reconnect berhasil
4. **Test koneksi MQTT** jika ada masalah jaringan 