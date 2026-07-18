import numpy as np
import cv2

# HSV color ranges for red and green
RED_LOWER1 = np.array([0, 100, 100])
RED_UPPER1 = np.array([10, 255, 255])
RED_LOWER2 = np.array([160, 100, 100])
RED_UPPER2 = np.array([179, 255, 255])
GREEN_LOWER = np.array([40, 70, 70])
GREEN_UPPER = np.array([80, 255, 255])

# Thresholds
MIN_BLOCK_AREA = 4000  # pixel area
BLOCK_CLOSE_HEIGHT = 60  # pixel height

def detect_block(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Red mask (2 ranges)
    red_mask1 = cv2.inRange(hsv, RED_LOWER1, RED_UPPER1)
    red_mask2 = cv2.inRange(hsv, RED_LOWER2, RED_UPPER2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)

    # Green mask
    green_mask = cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)

    # Find largest red and green contours
    contours_r, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_g, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Check red
    for cnt in contours_r:
        area = cv2.contourArea(cnt)
        x, y, w, h = cv2.boundingRect(cnt)
        if area > MIN_BLOCK_AREA and h > BLOCK_CLOSE_HEIGHT:
            return 'red'

    # Check green
    for cnt in contours_g:
        area = cv2.contourArea(cnt)
        x, y, w, h = cv2.boundingRect(cnt)
        if area > MIN_BLOCK_AREA and h > BLOCK_CLOSE_HEIGHT:
            return 'green'

    return None
