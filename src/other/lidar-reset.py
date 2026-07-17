from rplidar import RPLidar

PORT = "/dev/ttyUSB1"

lidar = RPLidar(PORT)

try:
    lidar.stop()
    lidar.stop_motor()
    lidar.clean_input()
finally:
    lidar.disconnect()

print("LiDAR reset.")
