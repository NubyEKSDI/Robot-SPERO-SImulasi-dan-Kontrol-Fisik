import numpy as np
import matplotlib.pyplot as plt
from rplidar import RPLidar

PORT_NAME = '/dev/ttyUSB0'  # Ganti jika port berbeda

lidar = RPLidar(PORT_NAME)

plt.ion()
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
scan_plot, = ax.plot([], [], 'b.', markersize=2)
ax.set_ylim(0, 12)  # RPLIDAR A1M8 max range ~12m
ax.set_title('RPLIDAR A1M8 Real-Time Visualization')

try:
    while True:
        angles = []
        distances = []
        for scan in lidar.iter_scans(max_buf_meas=360):
            angles.clear()
            distances.clear()
            for (_, angle, distance) in scan:
                if distance > 0:
                    angles.append(np.deg2rad(angle))
                    distances.append(distance / 1000.0)  # mm to m
            scan_plot.set_data(angles, distances)
            plt.pause(0.001)
            break  # Only update once per scan
except KeyboardInterrupt:
    print('Stopping visualization...')
finally:
    lidar.stop()
    lidar.disconnect()
    plt.ioff()
    plt.show() 