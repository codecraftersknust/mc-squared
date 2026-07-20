import time
import serial
import cv2
import numpy as np
from picamera2 import Picamera2
import RPi.GPIO as GPIO

# ---------- Hardware Configuration ----------
ARDUINO_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

# Right Ultrasonic Pins
RIGHT_TRIG = 27
RIGHT_ECHO = 22

# Setpoints & Thresholds
TARGET_DIST_MM = 220        # Target distance from wall (22cm)
CORNER_SPIKE_THRESH = 1000  # Distance spike (mm) triggering corner turn
TURN_DURATION = 0.20        # Seconds to hold hard turn during corner
TURN_COOLDOWN = 0.80        # Lockout window (seconds) to prevent re-trigger loops

# Obstacle Avoidance Parameters
AVOID_DURATION = 1.80       # Seconds to hold dodge steering maneuver
AVOID_STEER_LEFT = 135      # Steer Left for Green Block (Pass on Right)
AVOID_STEER_RIGHT = 45      # Steer Right for Red Block (Pass on Left)

# Smoother PID Gains
Kp = 0.25
Ki = 0.002
Kd = 0.10
dt = 0.08

# Steering Limits
STEERING_MIN = 45           # Full Right Turn
STEERING_CENTER = 90
STEERING_MAX = 135          # Full Left Turn

# ESC Values
BASE_THROTTLE = 1665
STOP_THROTTLE = 1500

# Vision Config
# 1. Lap Counter (Orange)
LOWER_ORANGE = np.array([5, 120, 120])
UPPER_ORANGE = np.array([22, 255, 255])
ORANGE_PIXEL_THRESHOLD = 1200

# 2. Obstacles (Red & Green)
LOWER_RED_1 = np.array([0, 120, 70])
UPPER_RED_1 = np.array([10, 255, 255])
LOWER_RED_2 = np.array([165, 120, 70])
UPPER_RED_2 = np.array([180, 255, 255])

LOWER_GREEN = np.array([35, 80, 80])
UPPER_GREEN = np.array([85, 255, 255])

BLOCK_TRIGGER_THRESHOLD = 3000  # Contour pixel area required to trigger dodge

# ---------- Setup ----------
GPIO.setmode(GPIO.BCM)
GPIO.setup(RIGHT_TRIG, GPIO.OUT)
GPIO.setup(RIGHT_ECHO, GPIO.IN)
GPIO.output(RIGHT_TRIG, False)

print("Connecting to Arduino & Camera...")
ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
time.sleep(2)

picam = Picamera2()
picam.preview_configuration.main.size = (640, 480)
picam.preview_configuration.main.format = "RGB888"
picam.preview_configuration.align()
picam.configure("preview")
picam.start()
time.sleep(1.0)

# ---------- Robot State Variables ----------
prev_error = 0
integral = 0
filtered_derivative = 0

orange_line_count = 0
line_detected_flag = False

# Corner Turn State
is_turning = False
turn_start_time = 0
last_turn_end_time = 0

# Obstacle Avoidance State
avoidance_mode = False
avoidance_start_time = 0
override_steering = STEERING_CENTER
avoidance_label = ""

def send_to_arduino(throttle, steer):
    ser.write(f"{int(steer)},{int(throttle)}\n".encode())

def get_right_distance():
    """Reads distance from right sensor. Returns None on timeout/wall loss."""
    try:
        GPIO.output(RIGHT_TRIG, True)
        time.sleep(0.00001)
        GPIO.output(RIGHT_TRIG, False)
        
        pulse_start = time.time()
        timeout = time.time()
        
        while GPIO.input(RIGHT_ECHO) == 0:
            pulse_start = time.time()
            if pulse_start - timeout > 0.025:
                return None
                
        pulse_end = time.time()
        while GPIO.input(RIGHT_ECHO) == 1:
            pulse_end = time.time()
            if pulse_end - pulse_start > 0.025:
                return None
                
        dist = ((pulse_end - pulse_start) * 343000) / 2
        return dist if 20 <= dist <= 2500 else None
    except:
        return None

def smooth_pid(error, prev_error, integral, prev_derivative, kp, ki, kd, dt):
    raw_derivative = (error - prev_error) / dt
    derivative = (0.3 * raw_derivative) + (0.7 * prev_derivative)
    
    integral += error * dt
    integral = max(min(integral, 100), -100)
    
    output = (kp * error) + (ki * integral) + (kd * derivative)
    return output, integral, derivative

# ---------- Main Loop ----------
try:
    print("\n==================================================")
    print("🚀 MC-SQUARED: ROUND 2 RIGHT INNER FOLLOW + AVOIDANCE")
    print("==================================================\n")
    
    while True:
        current_time = time.time()

        # --- 1. PRIORITY 1: OBSTACLE AVOIDANCE TIMER OVERRIDE ---
        if avoidance_mode:
            if current_time - avoidance_start_time < AVOID_DURATION:
                send_to_arduino(BASE_THROTTLE, override_steering)
                print(f"🛑 DODGING {avoidance_label} ({current_time - avoidance_start_time:.2f}s)", end="\r")
                time.sleep(dt)
                continue
            else:
                print(f"\n✅ {avoidance_label} Cleared! Re-acquiring wall...")
                avoidance_mode = False
                # Hard reset PID and cooldown memory upon completing maneuver
                integral = 0
                prev_error = 0
                filtered_derivative = 0
                last_turn_end_time = current_time

        # --- 2. PRIORITY 2: CORNER TURN OVERRIDE ---
        if is_turning:
            if current_time - turn_start_time < TURN_DURATION:
                send_to_arduino(BASE_THROTTLE, STEERING_MIN)
                print(f"🌀 EXECUTING RIGHT CORNER TURN ({current_time - turn_start_time:.2f}s)", end="\r")
                time.sleep(dt)
                continue
            else:
                print("\n✅ Turn complete! Locking out re-triggers for 0.8s...")
                is_turning = False
                last_turn_end_time = current_time
                integral = 0
                prev_error = 0
                filtered_derivative = 0

        # --- 3. VISION PROCESSING (Camera Lap Counting & Block Detection) ---
        try:
            frame_rgb = picam.capture_array()
            frame_rotated = cv2.rotate(frame_rgb, cv2.ROTATE_180)
            hsv = cv2.cvtColor(frame_rotated, cv2.COLOR_RGB2HSV)
            h, w, _ = hsv.shape

            # A. Lap Line Detection (Bottom 30%)
            roi_lap = hsv[int(h * 0.70):h, 0:w]
            mask_orange = cv2.inRange(roi_lap, LOWER_ORANGE, UPPER_ORANGE)
            orange_pixels = cv2.countNonZero(mask_orange)

            if orange_pixels > ORANGE_PIXEL_THRESHOLD:
                if not line_detected_flag:
                    orange_line_count += 1
                    line_detected_flag = True
                    current_laps = orange_line_count // 4
                    print(f"\n🍊 LAP LINE DETECTED! Total: {orange_line_count}/12 | Laps: {current_laps}/3")
            else:
                line_detected_flag = False

            if orange_line_count >= 12:
                print("\n🏆 ROUND 2 COMPLETE: 3 Laps Finished Successfully!")
                break

            # B. Obstacle Detection (Center Window: Y 40-90%, X 15-85%)
            roi_obs = hsv[int(h * 0.40):int(h * 0.90), int(w * 0.15):int(w * 0.85)]
            
            mask_red = cv2.bitwise_or(
                cv2.inRange(roi_obs, LOWER_RED_1, UPPER_RED_1),
                cv2.inRange(roi_obs, LOWER_RED_2, UPPER_RED_2)
            )
            mask_green = cv2.inRange(roi_obs, LOWER_GREEN, UPPER_GREEN)

            contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            max_red_area = max([cv2.contourArea(c) for c in contours_red] + [0])
            max_green_area = max([cv2.contourArea(c) for c in contours_green] + [0])

            # Trigger Avoidance if a block crosses size threshold
            if max_red_area >= BLOCK_TRIGGER_THRESHOLD and max_red_area > max_green_area:
                avoidance_mode = True
                avoidance_start_time = current_time
                override_steering = AVOID_STEER_RIGHT
                avoidance_label = "RED BLOCK"
                print(f"\n🔴 RED BLOCK DETECTED ({int(max_red_area)}px) -> Steering RIGHT!")
                send_to_arduino(BASE_THROTTLE, override_steering)
                continue

            elif max_green_area >= BLOCK_TRIGGER_THRESHOLD and max_green_area > max_red_area:
                avoidance_mode = True
                avoidance_start_time = current_time
                override_steering = AVOID_STEER_LEFT
                avoidance_label = "GREEN BLOCK"
                print(f"\n🟢 GREEN BLOCK DETECTED ({int(max_green_area)}px) -> Steering LEFT!")
                send_to_arduino(BASE_THROTTLE, override_steering)
                continue

        except Exception as vision_err:
            pass

        # --- 4. PRIORITY 3: ULTRASONIC WALL FOLLOW & CORNER SPIKE CHECK ---
        dist = get_right_distance()
        in_cooldown = (current_time - last_turn_end_time) < TURN_COOLDOWN

        # Check for Corner Wall Drop (Only if NOT in cooldown and NO obstacle)
        if not in_cooldown and (dist is None or dist >= CORNER_SPIKE_THRESH):
            is_turning = True
            turn_start_time = current_time
            print(f"\n🚨 CORNER SPIKE ({dist}) -> Hard Right Turn Initiated!")
            send_to_arduino(BASE_THROTTLE, STEERING_MIN)
            continue

        # Holding center if ultrasonic glitch occurs on straightaway
        if dist is None:
            send_to_arduino(BASE_THROTTLE, STEERING_CENTER)
            print("⚠️ Beam Glitch -> Holding Center", end="\r")
            time.sleep(dt)
            continue

        # --- 5. SMOOTH PID WALL FOLLOW (Default) ---
        error = dist - TARGET_DIST_MM  # Negative = Too close to right wall
        control, integral, filtered_derivative = smooth_pid(
            error, prev_error, integral, filtered_derivative, Kp, Ki, Kd, dt
        )
        prev_error = error

        control = max(min(control, 400), -400)

        # Inverted Steering Equation for Right Wall:
        # Drifting close (negative error) -> Steer LEFT (< 90 deg)
        steering_angle = STEERING_CENTER - (control / 400.0) * (STEERING_MAX - STEERING_CENTER)
        steering_angle = max(min(steering_angle, STEERING_MAX), STEERING_MIN)

        send_to_arduino(BASE_THROTTLE, steering_angle)

        status_str = "⏳ COOLDOWN" if in_cooldown else "🛣️ NORMAL"
        print(
            f"Dist: {dist:4.0f}mm | Err: {error:+5.1f} | Steer: {steering_angle:5.1f}° | Mode: {status_str}", 
            end="\r"
        )
        time.sleep(dt)

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    send_to_arduino(STOP_THROTTLE, STEERING_CENTER)
    try:
        picam.stop()
        picam.close()
    except:
        pass
    GPIO.cleanup()
    ser.close()
    print("\nShutdown complete.")