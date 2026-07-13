from rplidar import RPLidar

PORT = "/dev/ttyUSB0"

lidar = RPLidar(PORT)

try:
    # Stop scanning if active
    lidar.stop()

    # Stop the motor
    lidar.stop_motor()

finally:
    lidar.disconnect()

print("LiDAR is now idle.")
