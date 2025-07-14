# Fix MQTT Disconnect Issues in roam.py

## Masalah: roam.py MQTT disconnect terus

### Gejala
- roam.py menampilkan "MQTT: Disconnected from broker"
- Koneksi MQTT tidak stabil
- Data lidar tidak masuk setelah reconnect

### Penyebab
1. **Duplikasi Publisher**: Ada dua publisher untuk topic scan yang sama
2. **MQTT Client tidak stabil**: Tidak ada auto-reconnect yang proper
3. **Threading issues**: MQTT loop tidak dihandle dengan baik dalam ROS2 node

### Solusi yang Diimplementasi

#### 1. Fixed Duplicate Publisher
```python
# SEBELUM (SALAH):
self.scan_sub = self.create_subscription(LaserScan, '/bcr_bot/scan', self.scan_callback, 10)
self.physical_scan_pub = self.create_publisher(LaserScan, '/bcr_bot/scan', 10)  # DUPLIKAT!

# SESUDAH (BENAR):
self.scan_sub = self.create_subscription(LaserScan, '/bcr_bot/scan', self.scan_callback, 10)
self.physical_scan_pub = self.create_publisher(LaserScan, '/bcr_bot/physical_scan', 10)  # TOPIC BERBEDA
```

#### 2. Improved MQTT Client Configuration
```python
# Set MQTT options for better stability
self.mqtt_client.reconnect_delay_set(min_delay=1, max_delay=120)
self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
```

#### 3. Better Reconnection Handling
```python
def on_mqtt_disconnect(self, client, userdata, rc):
    """Callback when MQTT client disconnects"""
    self.mqtt_connected = False
    self.get_logger().warning(f"MQTT client disconnected with code: {rc}")
    self.print_status("MQTT: Disconnected from broker")
    
    # Try to reconnect if not shutting down
    if self.running and rc != 0:
        self.get_logger().info("Attempting to reconnect to MQTT broker...")
        self.print_status("MQTT: Attempting to reconnect...")
        try:
            time.sleep(2.0)
            client.reconnect()
        except Exception as e:
            self.get_logger().error(f"Failed to reconnect: {str(e)}")
            self.print_status("MQTT: Reconnection failed")
```

### Scripts untuk Troubleshooting

#### 1. Quick Restart roam.py
```bash
./src/bcr_bot/scripts/quick_restart_roam.sh
```

#### 2. Check MQTT Status
```bash
python3 src/bcr_bot/scripts/check_mqtt_status.py
```

#### 3. Full System Restart
```bash
./src/bcr_bot/scripts/restart_mapping_system.sh both
```

### Langkah Troubleshooting

#### Step 1: Check MQTT Connection
```bash
python3 src/bcr_bot/scripts/check_mqtt_status.py
```

**Expected Output:**
```
✅ Connected to MQTT broker: codex.petra.ac.id
📡 Subscribed to: robot/physical/scan
📡 Subscribed to: robot/lidar/real_world_data
...
✅ robot/lidar/real_world_data: 15 messages (last: 2.1s ago)
```

#### Step 2: Quick Restart roam.py
```bash
./src/bcr_bot/scripts/quick_restart_roam.sh
```

**Expected Output:**
```
✅ roam.py started with PID: 12345
```

#### Step 3: Monitor roam.py Output
Look for these messages:
```
MQTT: Connected to broker
MQTT: All topics subscribed successfully
MQTT: Sent ready message to LiDAR systems (reconnect)
📡 Real-World LiDAR: Front=2.34m, Left=1.56m, Right=2.78m
```

#### Step 4: Check test_robot.py on Raspberry Pi
Look for these messages:
```
MQTT: Connected successfully
Detected roam.py reconnection!
Restarting lidar streaming for roam.py...
LiDAR streaming started
```

### Manual Troubleshooting

#### Jika MQTT masih disconnect:

1. **Check Network Connectivity**
```bash
ping codex.petra.ac.id
```

2. **Check MQTT Broker Status**
```bash
telnet codex.petra.ac.id 1883
```

3. **Force Kill dan Restart**
```bash
pkill -9 -f "roam.py"
sleep 5
cd /home/nuby/bcr_ws
source install/setup.bash
ros2 run bcr_bot roam
```

4. **Check ROS2 Topics**
```bash
ros2 topic list
ros2 topic echo /bcr_bot/scan
```

### Prevention Tips

1. **Jangan restart roam.py terlalu sering** - beri waktu untuk MQTT stabil
2. **Monitor log** - cek apakah ada error yang menyebabkan disconnect
3. **Gunakan script restart** - untuk memastikan clean restart
4. **Check network stability** - pastikan koneksi internet stabil

### Debug Commands

#### Check Process Status
```bash
ps aux | grep roam.py
ps aux | grep test_robot.py
```

#### Check MQTT Topics
```bash
mosquitto_sub -h codex.petra.ac.id -t "robot/lidar/status" -v
```

#### Check ROS2 Topics
```bash
ros2 topic list | grep scan
ros2 topic info /bcr_bot/scan
```

### Expected Behavior After Fix

1. **roam.py starts** → MQTT connects automatically
2. **test_robot.py detects reconnect** → Restarts lidar streaming
3. **Data flows** → Real-world LiDAR data appears in roam.py
4. **Stable connection** → No more frequent disconnects

### Jika Masalah Masih Berlanjut

1. Check firewall settings
2. Verify MQTT broker is accessible
3. Check Python dependencies (paho-mqtt)
4. Verify network configuration
5. Consider using different MQTT broker for testing 