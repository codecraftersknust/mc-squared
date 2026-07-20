import time
import cv2
import numpy as np
from picamera2 import Picamera2

# ---------- HSV Threshold Configuration ----------
LOWER_RED_1 = np.array([0, 120, 70])
UPPER_RED_1 = np.array([10, 255, 255])
LOWER_RED_2 = np.array([165, 120, 70])
UPPER_RED_2 = np.array([180, 255, 255])

LOWER_GREEN = np.array([35, 80, 80])
UPPER_GREEN = np.array([85, 255, 255])

# Noise filter (Ignores minor color specks in the background)
MIN_CONTOUR_AREA = 400  

# DISTANCE TRIGGER THRESHOLD:
# The action will only fire if the block's pixel area is GREATER than this value.
# Increase this value to make the car turn LATER (closer), decrease it to turn EARLIER (further away).
BLOCK_TRIGGER_THRESHOLD = 50000 

print("Initializing Raspberry Pi Camera (Picamera2)...")
picam = Picamera2()
picam.preview_configuration.main.size = (640, 480)
picam.preview_configuration.main.format = "RGB888"
picam.preview_configuration.align()
picam.configure("still")  
picam.start()
time.sleep(1.0)

print("\n==================================================")
print("🚦 MC-SQUARED: THRESHOLD OBSTACLE TESTER")
print(f"Action triggers only when area > {BLOCK_TRIGGER_THRESHOLD} px")
print("Press 'q' to exit.")
print("==================================================\n")

try:
    while True:
        frame_rgb = picam.capture_array()
        frame_rotated = cv2.rotate(frame_rgb, cv2.ROTATE_180)
        
        frame_display = cv2.cvtColor(frame_rotated, cv2.COLOR_RGB2BGR)
        h, w, _ = frame_display.shape
        
        # Define the Center ROI Coordinates (Middle-to-lower section)
        roi_y_start = int(h * 0.40)
        roi_y_end = int(h * 0.90)
        roi_x_start = int(w * 0.15)
        roi_x_end = int(w * 0.85)
        
        roi_rgb = frame_rotated[roi_y_start:roi_y_end, roi_x_start:roi_x_end]
        hsv_roi = cv2.cvtColor(roi_rgb, cv2.COLOR_RGB2HSV)
        
        # Color Masks
        mask_red1 = cv2.inRange(hsv_roi, LOWER_RED_1, UPPER_RED_1)
        mask_red2 = cv2.inRange(hsv_roi, LOWER_RED_2, UPPER_RED_2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        
        mask_green = cv2.inRange(hsv_roi, LOWER_GREEN, UPPER_GREEN)
        
        # Find Red Contours
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        closest_red = None
        max_red_area = 0
        for c in contours_red:
            area = cv2.contourArea(c)
            if area > MIN_CONTOUR_AREA and area > max_red_area:
                max_red_area = area
                closest_red = cv2.boundingRect(c)
                
        # Find Green Contours
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        closest_green = None
        max_green_area = 0
        for c in contours_green:
            area = cv2.contourArea(c)
            if area > MIN_CONTOUR_AREA and area > max_green_area:
                max_green_area = area
                closest_green = cv2.boundingRect(c)
                
        # --- DECISION LOGIC WITH AREA THRESHOLD ---
        action = "⚪ KEEP CENTER (No Block)"
        
        # Case A: Red block is the closest/largest object inside ROI
        if closest_red and (max_red_area > max_green_area):
            rx, ry, rbox_w, rbox_h = closest_red
            
            # Check if it has crossed our trigger threshold
            if max_red_area >= BLOCK_TRIGGER_THRESHOLD:
                action = "🚨 AVOID RED -> STEER RIGHT"
                box_color = (0, 0, 255)  # Solid Red Box for active threat
                text_label = f"CRITICAL RED: {int(max_red_area)}px"
            else:
                action = "⏳ Red Detected (Too far away)"
                box_color = (255, 0, 255)  # Purple Box for warning
                text_label = f"Distant Red: {int(max_red_area)}px"
                
            cv2.rectangle(frame_display, (roi_x_start + rx, roi_y_start + ry), 
                          (roi_x_start + rx + rbox_w, roi_y_start + ry + rbox_h), box_color, 3)
            cv2.putText(frame_display, text_label, (roi_x_start + rx, roi_y_start + ry - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
                        
        # Case B: Green block is the closest/largest object inside ROI
        elif closest_green and (max_green_area > max_red_area):
            gx, gy, gbox_w, gbox_h = closest_green
            
            # Check if it has crossed our trigger threshold
            if max_green_area >= BLOCK_TRIGGER_THRESHOLD:
                action = "🚨 AVOID GREEN -> STEER LEFT"
                box_color = (0, 255, 0)  # Solid Green Box for active threat
                text_label = f"CRITICAL GREEN: {int(max_green_area)}px"
            else:
                action = "⏳ Green Detected (Too far away)"
                box_color = (255, 255, 0)  # Cyan Box for warning
                text_label = f"Distant Green: {int(max_green_area)}px"
                
            cv2.rectangle(frame_display, (roi_x_start + gx, roi_y_start + gy), 
                          (roi_x_start + gx + gbox_w, roi_y_start + gy + gbox_h), box_color, 3)
            cv2.putText(frame_display, text_label, (roi_x_start + gx, roi_y_start + gy - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
        
        # Draw the Yellow Scan Box Area
        cv2.rectangle(frame_display, (roi_x_start, roi_y_start), (roi_x_end, roi_y_end), (0, 255, 255), 2)
        
        # Clean Console Printout Loop
        print(f"Status: {action:<30} | R Area: {int(max_red_area):5d} | G Area: {int(max_green_area):5d}", end="\r")
        
        cv2.imshow("Main Live Feed", frame_display)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("\nExiting calibration tool...")
            break

except KeyboardInterrupt:
    print("\nTool terminated by user.")

finally:
    try:
        picam.stop()
        picam.close()
    except:
        pass
    cv2.destroyAllWindows()
    print("Camera interface cleaned successfully.")
