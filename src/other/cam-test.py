import time
import cv2
import numpy as np
from picamera2 import Picamera2

# ---------- Configuration (Match your main script) ----------
# Adjust these bounds if the track lines aren't lighting up white in the mask!
LOWER_ORANGE = np.array([5, 120, 100])
UPPER_ORANGE = np.array([22, 255, 255])
ORANGE_PIXEL_THRESHOLD = 1500

print("Initializing Raspberry Pi Camera (Picamera2)...")
picam = Picamera2()
picam.preview_configuration.main.size = (640, 480)
picam.preview_configuration.main.format = "RGB888"
picam.preview_configuration.align()
picam.configure("preview")
picam.start()
time.sleep(1.0)  # Let auto-exposure settle

print("\n==================================================")
print("📸 MC-SQUARED COLOR CALIBRATION TOOL STARTED")
print("Place the car on the track over the orange line.")
print("Press 'q' in either video window to quit.")
print("==================================================\n")

try:
    while True:
        # Capture the current frame
        frame_rgb = picam.capture_array()
        
        # Convert RGB to BGR for OpenCV processing
        frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, w, _ = frame.shape
        
        # Isolate the exact bottom 30% Region of Interest (ROI) used in your race loop
        roi_start_y = int(h * 0.7)
        roi = hsv[roi_start_y:h, 0:w]
        
        # Create the threshold mask (White = Orange match, Black = Ignored)
        mask = cv2.inRange(roi, LOWER_ORANGE, UPPER_ORANGE)
        orange_pixels = cv2.countNonZero(mask)
        
        # Draw a visual box on the original frame showing the exact scan area
        cv2.rectangle(frame, (0, roi_start_y), (w, h), (0, 255, 0), 2)
        
        # Print telemetry data cleanly to the console screen
        status = "🚨 LINE DETECTED" if orange_pixels > ORANGE_PIXEL_THRESHOLD else "⚪ Scanning..."
        print(f"{status} | Matched Pixels: {orange_pixels:5d} / Trigger Target: {ORANGE_PIXEL_THRESHOLD}", end="\r")
        
        # Display the live window feeds
        cv2.imshow("Original Feed (Green Box = Target Scan Zone)", frame)
        cv2.imshow("Orange Mask (Must show clear white line shape)", mask)
        
        # Break loop out instantly if user presses 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("\nExiting calibration tool...")
            break

except KeyboardInterrupt:
    print("\nCalibration tool terminated by user.")

finally:
    # Clean workspace teardown
    try:
        picam.stop()
        picam.close()
    except:
        pass
    cv2.destroyAllWindows()
    print("Camera closed cleanly.")
