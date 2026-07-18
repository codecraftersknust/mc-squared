import pyrealsense2 as rs
import numpy as np
import cv2
import time
import serial
from rplidar import RPLidar
from block_detector import detect_block

# ---------- Configuration ----------
REALSENSE_RES = (640, 480)
LIDAR_PORT = '/dev/ttyUSB1'
ARDUINO_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

# HSV range for orange line detection
ORANGE_LOW = np.array([0, 55, 120])
ORANGE_HIGH = np.array([10, 255, 255])

# PID Constants
Kp = 0.5
Ki = 0.0
Kd = 0.01
dt = 0.1  # seconds

# Steering limits
STEERING_MIN = 45
STEERING_CENTER = 90
STEERING_MAX = 135
BASE_THROTTLE = 1760
STOP_THROTTLE = 1500

# ---------- Setup ----------
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, *REALSENSE_RES, rs.format.bgr8, 30)
pipeline.start(config)

lidar = RPLidar(LIDAR_PORT)
ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
time.sleep(2)

# ---------- State ----------
prev_error = 0
integral = 0
lap_count = 0
orange_count = 0
last_detect_time = 0
orange_detected = False

# ---------- Helper Functions ----------
def send_to_arduino(throttle, steer):
    cmd = f"<THROTTLE:{int(throttle)}><STEER:{int(steer)}>"
    ser.write(cmd.encode())

def check_lap(frame):
    global orange_detected, last_detect_time, orange_count, lap_count

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    roi = hsv[frame.shape[0] - 100:, :]
    mask = cv2.inRange(roi, ORANGE_LOW, ORANGE_HIGH)
    pixel_count = np.sum(mask > 0)

    if pixel_count > 5000:
        if not orange_detected and (time.time() - last_detect_time) > 0.5:
            orange_detected = True
            last_detect_time = time.time()
            orange_count += 1
            print(f"Orange detected! Count: {orange_count}")
            if orange_count == 4:
                lap_count += 1
                orange_count = 0
                print(f"Laps completed! Total laps: {lap_count}")
    else:
        orange_detected = False

def average_distance(scan, angle_range):
    readings = [
        dist for (_, angle, dist) in scan
        if angle_range[0] <= angle <= angle_range[1] and dist > 50
    ]
    return sum(readings) / len(readings) if readings else None

def pid_control(error, prev_error, integral, Kp, Ki, Kd, dt):
    derivative = (error - prev_error) / dt
    integral += error * dt
    output = Kp * error + Ki * integral + Kd * derivative
    return output, integral

# --- New ---
avoid_mode = None
avoid_start_time = 0
AVOID_DURATION = 1.5  # seconds to maintain avoidance steer

try:
    lidar.start_motor()

    for scan in lidar.iter_scans():
        # RealSense Frame
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        color_img = np.asanyarray(color_frame.get_data())

        # Lap counter (same as before)
        check_lap(color_img)
        if lap_count >= 3:
            send_to_arduino(STOP_THROTTLE, STEERING_CENTER)
            break

        # Block detection (steering override)
        block = detect_block(color_img)
        if block and avoid_mode is None:
            avoid_mode = block
            avoid_start_time = time.time()
            print(f"Detected {block} block — initiating avoidance")

        # If in avoidance mode
        if avoid_mode:
            if time.time() - avoid_start_time < AVOID_DURATION:
                steer_override = STEERING_CENTER + 30 if avoid_mode == 'red' else STEERING_CENTER - 30
                steer_override = max(min(steer_override, STEERING_MAX), STEERING_MIN)
                send_to_arduino(BASE_THROTTLE, steer_override)
                continue
            else:
                avoid_mode = None  # Clear after duration

        # Wall following as fallback
        left = average_distance(scan, (265, 310))
        if left:
            error = left - 200
            control, integral = pid_control(error, prev_error, integral, Kp, Ki, Kd, dt)
            prev_error = error
            control = max(min(control, 500), -500)
            steer = STEERING_CENTER + (control / 500.0) * (STEERING_MAX - STEERING_CENTER)
            steer = max(min(steer, STEERING_MAX), STEERING_MIN)
            send_to_arduino(BASE_THROTTLE, steer)

        time.sleep(dt)

except KeyboardInterrupt:
    send_to_arduino(STOP_THROTTLE, STEERING_CENTER)

finally:
    send_to_arduino(STOP_THROTTLE, STEERING_CENTER)
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    ser.close()
    pipeline.stop()
    cv2.destroyAllWindows()