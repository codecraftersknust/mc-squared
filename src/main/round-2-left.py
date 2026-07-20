import time
import serial
import cv2
import numpy as np
from picamera2 import Picamera2
from rplidar import RPLidar

# ---------- Configuration ----------
LIDAR_PORT = '/dev/ttyUSB1'
ARDUINO_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

# PID Constants (Your exact working parameters)
Kp = 0.5
Ki = 0.005
Kd = 0.1
dt = 0.1  # seconds

# Steering limits
STEERING_MIN = 45
STEERING_CENTER = 90
STEERING_MAX = 135

# ESC values
BASE_THROTTLE = 1765
STOP_THROTTLE = 1500

# Avoidance Hardware Profiles
AVOID_STEER_LEFT = 55     # Extreme left to clear green block
AVOID_STEER_RIGHT = 125   # Extreme right to clear red block
AVOID_DURATION = 2.0      # How long to hold the hard turn in seconds

# Vision Config (Your exact working calibration)
LOWER_RED_1 = np.array([0, 120, 70])
UPPER_RED_1 = np.array([10, 255, 255])
LOWER_RED_2 = np.array([165, 120, 70])
UPPER_RED_2 = np.array([180, 255, 255])

LOWER_GREEN = np.array([35, 80, 80])
UPPER_GREEN = np.array([85, 255, 255])

LOWER_ORANGE = np.array([5, 120, 100])
UPPER_ORANGE = np.array([22, 255, 255])

MIN_CONTOUR_AREA = 400  
BLOCK_TRIGGER_THRESHOLD = 30000 
ORANGE_PIXEL_THRESHOLD = 1500

# ---------- Hardware Setup ----------
print("Initializing Serial Connection to Arduino...")
ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
time.sleep(2)

print("Initializing Raspberry Pi Camera (Picamera2)...")
picam = Picamera2()
picam.preview_configuration.main.size = (640, 480)
picam.preview_configuration.main.format = "RGB888"
picam.preview_configuration.align()
picam.configure("preview")
picam.start()
time.sleep(1.0)

# ---------- Robot State Variables ----------
lidar = None
prev_error = 0
integral = 0
orange_line_count = 0
line_detected_flag = False

# Avoidance State Machine Variables
avoidance_mode = False
avoidance_start_time = 0
override_steering = STEERING_CENTER

# ---------- Helper Functions ----------
def send_to_arduino(throttle, steer):
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
    
    print("\n==================================================")
    print("🚀 MC-SQUARED: ROUND 2 ACTIVE (PID Follow + Avoidance)")
    print("==================================================\n")
    
    for scan in lidar.iter_scans():
        current_time = time.time()
        
        # ----------------------------------------------------------------------
        # SYSTEM LAYER A: AVOIDANCE TIMER MANAGEMENT (NON-BLOCKING)
        # ----------------------------------------------------------------------
        if avoidance_mode:
            # Check if 2 seconds have fully elapsed
            if current_time - avoidance_start_time < AVOID_DURATION:
                # Still in avoidance window: Force override steer and bypass PID/Vision checks
                send_to_arduino(BASE_THROTTLE, override_steering)
                print(f"🚨 AVOIDANCE ACTIVE: Overriding Steer to {override_steering}°", end="\r")
                time.sleep(dt)
                continue
            else:
                # 2 seconds up! Smoothly clear states and hand back control to PID
                print("\n✅ Obstacle Cleared! Resuming PID Wall Following...")
                avoidance_mode = False
                # Clear PID memory so it doesn't experience an integration spike after the manual turn
                integral = 0 
                prev_error = 0

        # ----------------------------------------------------------------------
        # SYSTEM LAYER B: PI CAMERA PROCESSING (Lap Count + Obstacle Check)
        # ----------------------------------------------------------------------
        try:
            frame_rgb = picam.capture_array()
            frame_rotated = cv2.rotate(frame_rgb, cv2.ROTATE_180)
            h, w, _ = frame_rotated.shape
            
            # --- B1. LAP COUNTING SECTION (Bottom 30%) ---
            roi_lap = frame_rotated[int(h * 0.7):h, 0:w]
            hsv_lap = cv2.cvtColor(roi_lap, cv2.COLOR_RGB2HSV)
            mask_orange = cv2.inRange(hsv_lap, LOWER_ORANGE, UPPER_ORANGE)
            orange_pixels = cv2.countNonZero(mask_orange)
            
            if orange_pixels > ORANGE_PIXEL_THRESHOLD:
                if not line_detected_flag:
                    orange_line_count += 1
                    line_detected_flag = True
                    print(f"\n🍊 LAP LINE DETECTED! Total: {orange_line_count}/12")
            else:
                line_detected_flag = False
                
            if orange_line_count >= 13:
                print("\n🏆 SUCCESS: 3 Laps completed in Round 2! Stopping vehicle.")
                break

            # --- B2. OBSTACLE SCANNING SECTION (Center ROI 40%-90%) ---
            roi_y_start = int(h * 0.40)
            roi_y_end = int(h * 0.90)
            roi_x_start = int(w * 0.15)
            roi_x_end = int(w * 0.85)
            
            roi_obstacle = frame_rotated[roi_y_start:roi_y_end, roi_x_start:roi_x_end]
            hsv_obs = cv2.cvtColor(roi_obstacle, cv2.COLOR_RGB2HSV)
            
            # Color Filters
            mask_red = cv2.bitwise_or(cv2.inRange(hsv_obs, LOWER_RED_1, UPPER_RED_1), 
                                     cv2.inRange(hsv_obs, LOWER_RED_2, UPPER_RED_2))
            mask_green = cv2.inRange(hsv_obs, LOWER_GREEN, UPPER_GREEN)
            
            # Contour Area Calculus
            contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            max_red_area = max([cv2.contourArea(c) for c in contours_red] + [0])
            
            contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            max_green_area = max([cv2.contourArea(c) for c in contours_green] + [0])
            
            # Check if an obstacle is close enough to interrupt the main loop
            if max_red_area >= BLOCK_TRIGGER_THRESHOLD and max_red_area > max_green_area:
                avoidance_mode = True
                avoidance_start_time = current_time
                override_steering = AVOID_STEER_RIGHT
                print(f"\n🛑 RED BLOCK TRIGGERED ({int(max_red_area)}px)! Executing Right Turn Avoidance...")
                send_to_arduino(BASE_THROTTLE, override_steering)
                continue
                
            elif max_green_area >= BLOCK_TRIGGER_THRESHOLD and max_green_area > max_red_area:
                avoidance_mode = True
                avoidance_start_time = current_time
                override_steering = AVOID_STEER_LEFT
                print(f"\n🛑 GREEN BLOCK TRIGGERED ({int(max_green_area)}px)! Executing Left Turn Avoidance...")
                send_to_arduino(BASE_THROTTLE, override_steering)
                continue
                
        except Exception as vision_err:
            pass

        # ----------------------------------------------------------------------
        # SYSTEM LAYER C: MAIN WALL-FOLLOWING LOGIC (Runs if no obstacle)
        # ----------------------------------------------------------------------
        left_distance = average_distance(scan, (265, 310))
        if left_distance is None:
            continue

        error = left_distance - 250
        control, integral = pid_control(error, prev_error, integral, Kp, Ki, Kd, dt)
        prev_error = error

        control = max(min(control, 500), -500)
        
        # Maintained your precise working inner left equation
        steering_angle = STEERING_CENTER + (control / 500.0) * (STEERING_MAX - STEERING_CENTER)
        steering_angle = max(min(steering_angle, STEERING_MAX), STEERING_MIN)

        send_to_arduino(BASE_THROTTLE, steering_angle)

        print(
            f"Left: {left_distance:.1f} mm | Steer: {steering_angle:.1f}° | Lines: {orange_line_count}/12", 
            end="\r"
        )
        time.sleep(dt)

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    print("\nShutting down safely...")
    send_to_arduino(STOP_THROTTLE, STEERING_CENTER)
    try:
        picam.stop()
        picam.close()
    except:
        pass
    if lidar is not None:
        try:
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
        except:
            pass
    ser.close()
    print("Shutdown complete. Robot safe.")
