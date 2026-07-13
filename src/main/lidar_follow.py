import math
import time
from rplidar import RPLidar
import serial

# LIDAR and Serial Port
PORT = '/dev/ttyUSB1'
SERIAL_PORT = '/dev/ttyUSB0'  # UART port for Arduino
BAUD_RATE = 115200 # baud rate for the serial communication

# PID Parameters
Kp = 0.3
Ki = 0.005
Kd = 0.8

# Control variables
prev_error = 0
integral = 0
desired_distance_diff = 0  # target: equal distance to both walls (centered)
dt = 0.1  # Loop delay in seconds

# Servo range
STEERING_MIN = 45
STEERING_CENTER = 90
STEERING_MAX = 135

# Create lidar and serial objects
lidar = RPLidar(PORT)
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

def average_distance(scan, angle_range):
    readings = [
        dist for (_, angle, dist) in scan
        if angle_range[0] <= angle <= angle_range[1] and dist > 50  # ignore near-zero junk
    ]
    if len(readings) == 0:
        return None
    return sum(readings) / len(readings)

def pid_control(error, prev_error, integral, Kp, Ki, Kd, dt):
    derivative = (error - prev_error) / dt
    integral += error * dt
    output = Kp * error + Ki * integral + Kd * derivative
    return output, integral

try:
    lidar.start_motor()
    print("Starting PID wall-following...")

    for scan in lidar.iter_scans():
        # Get average distances to left and right walls
        left_dist = average_distance(scan, (265, 310))  # LEFT
        right_dist = average_distance(scan, (50, 95))  # RIGHT

        if left_dist is not None and right_dist is not None:
            # Error = how far off center the car is (+ = too close to right)
            error = left_dist - right_dist 

            control, integral = pid_control(error, prev_error, integral, Kp, Ki, Kd, dt)
            prev_error = error

            # Map PID output to steering angle
            # Assume max error around ±500mm maps to full steering range
            control = max(min(control, 500), -500)  # Clamp control effort
            steering_angle = STEERING_CENTER + (control / 500.0) * (STEERING_MAX - STEERING_CENTER)
            steering_angle = max(min(steering_angle, STEERING_MAX), STEERING_MIN)

            esc_speed = 1000

            cmd = f"{int(steering_angle)},{esc_speed}\n"
            ser.write(cmd.encode())

            # Debug
            print(f"Left: {left_dist:.1f}mm, Right: {right_dist:.1f}mm, Error: {error:.1f}, Steering: {steering_angle:.1f}")

        time.sleep(dt)

except KeyboardInterrupt:
    print("Interrupted by user. Stopping...")

finally:
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    ser.close()
    print("Shutdown complete.")
