import math
import matplotlib.pyplot as plt
from rplidar import RPLidar

PORT = '/dev/ttyUSB1'
lidar = RPLidar(PORT)

# Init matplotlib
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, polar=True)

# Configure plot
ax.set_theta_zero_location('N')  # 0° at top
ax.set_theta_direction(-1)       # Clockwise
ax.set_rlim(0, 3000)             # Max range in mm (adjust if needed)

try:
    print("LIDAR info:", lidar.get_info())
    print("LIDAR health:", lidar.get_health())
    lidar.start_motor()

    for scan in lidar.iter_scans():
        angles = []
        distances = []

        for (_, angle, distance) in scan:
            if distance > 0:
                angles.append(math.radians(angle))
                distances.append(distance)

        ax.clear()
        ax.set_theta_zero_location('N')
        ax.set_theta_direction(-1)
        ax.set_rlim(0, 3000)
        ax.plot(angles, distances, 'go', markersize=1)
        plt.pause(0.01)

except KeyboardInterrupt:
    print("Stopping...")

finally:
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    print("LIDAR disconnected.")
