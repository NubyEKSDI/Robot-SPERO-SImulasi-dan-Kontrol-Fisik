from rplidar import RPLidar
import socket
import time

PORT_NAME = '/dev/ttyUSB0'  # Ganti jika port berbeda
lidar = RPLidar(PORT_NAME, baudrate=115200)

HOST = '127.0.0.1'
PORT = 5005

try:
    for i, scan in enumerate(lidar.iter_scans()):
        print(f'{i}: Got {len(scan)} measurements')
        if i >= 10:
            break

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        while True:
            # Contoh data: 360 titik, 1 derajat per step
            scan = ';'.join([f'{i},{1.0 + 0.1*i}' for i in range(360)])
            s.sendall((scan + '\n').encode())
            time.sleep(0.1)
finally:
    lidar.stop()
    lidar.disconnect() 