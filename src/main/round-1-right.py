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
CORNER_SPIKE_THRESH = 1000  # Distance spike (mm) that triggers a corner turn
TURN_DURATION = 0.20        # Seconds to hold hard turn during corner
TURN_COOLDOWN = 0.80        # CRITICAL FIX: Seconds to IGNORE new corner spikes after turning!

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

# Fixed HSV Vision Config (Correct RGB-to-HSV Mapping)
LOWER_ORANGE = np.array([5, 120, 120])
UPPER_ORANGE = np.array([22, 255, 255])
ORANGE_PIXEL_THRESHOLD = 1200

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

is_turning = False
turn_start_time = 0
last_turn_end_time = 0  # Tracks when the previous turn completed

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
    """PID controller with low-pass derivative filter and integral clamping."""
    raw_derivative = (error - prev_error) / dt
    derivative = (0.3 * raw_derivative) + (0.7 * prev_derivative)
    
    integral += error * dt
    integral = max(min(integral, 100), -100)
    
    output = (kp * error) + (ki * integral) + (kd * derivative)
    return output, integral, derivative

# ---------- Main Loop ----------
try:
    print("\n==================================================")
    print("🚀 MC-SQUARED: RIGHT-WALL FOLLOW (COOLDOWN PROTECTED)")
    print("==================================================\n")
    
    while True:
        current_time = time.time()

        # --- 1. FIXED PI CAMERA LAP COUNTING ---
        try:
            frame_rgb = picam.capture_array()
            frame_rotated = cv2.rotate(frame_rgb, cv2.ROTATE_180)
            
            hsv = cv2.cvtColor(frame_rotated, cv2.COLOR_RGB2HSV)
            h, w, _ = hsv.shape
            
            roi = hsv[int(h * 0.70):h, 0:w]
            mask_orange = cv2.inRange(roi, LOWER_ORANGE, UPPER_ORANGE)
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
                print("\n🏆 RUN COMPLETE: 3 Laps (12 Lines) Finished Successfully!")
                break
        except Exception as e:
            pass

        # --- 2. ACTIVE CORNER TURN STATE OVERRIDE ---
        if is_turning:
            if current_time - turn_start_time < TURN_DURATION:
                # Lock steering hard right (STEERING_MIN = 45°) and bypass PID
                send_to_arduino(BASE_THROTTLE, STEERING_MIN)
                print(f"🌀 EXECUTING RIGHT TURN ({current_time - turn_start_time:.2f}s)", end="\r")
                time.sleep(dt)
                continue
            else:
                print("\n✅ Turn complete! Locking out re-triggers for 0.8s...")
                is_turning = False
                last_turn_end_time = current_time  # Mark cooldown start timestamp
                
                # Hard reset PID memory so we re-acquire smoothly
                integral = 0
                prev_error = 0
                filtered_derivative = 0

        # --- 3. WALL FOLLOW & PROTECTED CORNER SPIKE DETECTION ---
        dist = get_right_distance()

        # CHECK IF COOLDOWN IS STILL ACTIVE (Lockout re-triggers)
        in_cooldown = (current_time - last_turn_end_time) < TURN_COOLDOWN

        # CORNER SPIKE TRIGGER CONDITION (Only allowed if NOT in cooldown!)
        if not in_cooldown and (dist is None or dist >= CORNER_SPIKE_THRESH):
            is_turning = True
            turn_start_time = current_time
            print(f"\n🚨 CORNER SPIKE DETECTED ({dist}) -> Hard Right Turn Initiated!")
            send_to_arduino(BASE_THROTTLE, STEERING_MIN)
            continue

        # If sensor drops to None during normal straightaways or cooldown, hold center
        if dist is None:
            send_to_arduino(BASE_THROTTLE, STEERING_CENTER)
            print("⚠️ Temporary Beam Glitch -> Holding Center", end="\r")
            time.sleep(dt)
            continue

        # --- 4. SMOOTH PID WALL FOLLOWING ---
        error = dist - TARGET_DIST_MM  # Negative = Too close to right wall
        control, integral, filtered_derivative = smooth_pid(
            error, prev_error, integral, filtered_derivative, Kp, Ki, Kd, dt
        )
        prev_error = error

        control = max(min(control, 400), -400)

        # Inverted Steering for Right Wall:
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
