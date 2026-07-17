from rplidar import RPLidar

PORT = "/dev/ttyUSB1"

lidar = RPLidar(PORT)

try:
    print("Info:", lidar.get_info())
    print("Health:", lidar.get_health())

    for scan in lidar.iter_scans():
        print(scan[:5])
        break

finally:
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()