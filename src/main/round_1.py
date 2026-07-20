import time
import serial
from rplidar import RPLidar

# ---------- Configuration ----------
LIDAR_PORT = '/dev/ttyUSB1'
ARDUINO_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

# PID Constants
Kp = 0.5
Ki = 0.005
Kd = 0.08
dt = 0.1  # seconds

# Steering limits
STEERING_MIN = 45
STEERING_CENTER = 90
STEERING_MAX = 135

# ESC values
BASE_THROTTLE = 1765
STOP_THROTTLE = 1500


# ---------- Setup ----------
lidar = None
ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
time.sleep(2)

# ---------- PID State ----------
prev_error = 0
integral = 0


# ---------- Helper Functions ----------
def send_to_arduino(throttle, steer):
    # Send comma-separated string ending with '\n' to match Arduino's parser
    cmd = f"{int(steer)},{int(throttle)}\n"
    ser.write(cmd.encode())


def average_distance(scan, angle_range):
    readings = [
        dist
        for (_, angle, dist) in scan
        if angle_range[0] <= angle <= angle_range[1] and dist > 50
    ]

    if len(readings) == 0:
        return None

    return sum(readings) / len(readings)


def pid_control(error, prev_error, integral, kp, ki, kd, dt):
    derivative = (error - prev_error) / dt
    integral += error * dt

    output = kp * error + ki * integral + kd * derivative

    return output, integral


# ---------- Main Loop ----------
try:
    lidar = RPLidar(LIDAR_PORT)
    
    try:
        lidar.stop()
    except:
        pass
    
    for scan in lidar.iter_scans():
        
        # Average distance to the left wall
        left_distance = average_distance(scan, (265, 310))

        # Ensure Left distance exists before calculating PID error
        if left_distance is None:
            # Safely skip this scan if any wall signal is missing
            continue

        # Calculate PID error
        error = left_distance - 200

        control, integral = pid_control(error, prev_error, integral, Kp, Ki, Kd, dt)

        prev_error = error

        # Limit controller output
        control = max(min(control, 500), -500)

        # Convert PID output to steering angle
        steering_angle = STEERING_CENTER + (control / 500.0) * (STEERING_MAX - STEERING_CENTER)

        steering_angle = max(min(steering_angle, STEERING_MAX), STEERING_MIN)

        send_to_arduino(BASE_THROTTLE, steering_angle)

        print(
            f"Left: {left_distance:.1f} mm | "
            f"Error: {error:.1f} | "
            f"Steering: {steering_angle:.1f}"
        )

        time.sleep(dt)

except KeyboardInterrupt:
    print("Interrupted by user.")

finally:
    send_to_arduino(STOP_THROTTLE, STEERING_CENTER)

    if lidar is not None:
        try:
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
        except Exception as e:
            print(f"Error disconnecting LiDAR: {e}")

    ser.close()

    print("Shutdown complete.")
