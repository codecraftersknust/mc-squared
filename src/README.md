# Control Software

This directory contains all control software developed by the team for the WRO Future Engineers competition vehicle. The software stack is split into **main** competition scripts and **other** utilities used during development and calibration.

---

## Main Scripts

### `launcher.py`
The primary entry point for the vehicle. Listens for button presses on **GPIO 16** and launches the appropriate competition script based on the number of clicks within a 2.5-second window:

| Presses | Script Launched |
|---|---|
| 1 | `round-1-left.py` — Round 1, left-wall follow |
| 2 | `round-1-right.py` — Round 1, right-wall follow |
| 3 | `round-2-left.py` — Round 2, left-wall follow + block avoidance |
| 4 | `round-2-right.py` — Round 2, right-wall follow + block avoidance |

### `round-1-left.py`
Round 1 script that follows the **left wall** using an ultrasonic sensor (GPIO 23/24). Applies a smooth PID controller (with derivative filtering and integral clamping) to hold the vehicle at 220 mm from the wall. Uses the **Pi Camera** to detect the orange starting line via HSV masking — every 4 detections counts as one lap, and the vehicle stops after completing 3 laps. Includes corner detection via distance spike (>1000 mm) with a 0.8 s cooldown to prevent re-triggering.

### `round-1-right.py`
Same structure as `round-1-left.py` but follows the **right wall** (GPIO 27/22). Used when the course direction is clockwise. Corner detection triggers a hard right turn (45°) instead of left, and the same lap counting and cooldown logic applies.

### `round-2-left.py`
Round 2 script built on `round-1-left.py` with added **colour block avoidance**. Uses the Pi Camera to detect red and green obstacles in a center ROI (Y: 40–90%, X: 15–85%). When a block's contour area exceeds the trigger threshold, the vehicle steers away for 1.8 seconds before resuming wall following:
- **Red block** → steer right (pass on left)
- **Green block** → steer left (pass on right)

Avoidance takes priority over corner turns, which take priority over normal wall following.

### `round-2-right.py`
Same as `round-2-left.py` but follows the **right wall**. Corner detection triggers a hard right turn and the avoidance steering directions remain the same.

### `motor-run/motor-run.ino`
Arduino firmware that receives `angle,speed\n` commands over UART and drives the **steering servo (D3)** and **ESC motor (D7)**. Angles are clamped to 45°–135° and ESC values to 1500–2000 µs. On startup the ESC is armed before the vehicle is ready to accept commands.

---

## Running the Main Scripts

```bash
# Start the launcher (select script via button presses)
python3 src/main/launcher.py

# Or run competition scripts directly:
python3 src/main/round-1-left.py   # Round 1 — left-wall follower
python3 src/main/round-1-right.py  # Round 1 — right-wall follower
python3 src/main/round-2-left.py   # Round 2 — left-wall + block avoidance
python3 src/main/round-2-right.py  # Round 2 — right-wall + block avoidance
```

> **Port note:** The Arduino uses `/dev/ttyUSB0` by default. Verify with `ls /dev/ttyUSB*` and update the `ARDUINO_PORT` constant at the top of each script if needed.

---

## Utility Scripts (`other/`)

| Script / Sketch | Purpose |
|---|---|
| `block_detector.py` | Live camera tool to tune and test block detection thresholds — displays bounding boxes and contour areas for red/green blocks |
| `cam-test.py` | Verifies the Pi Camera is detected and operational |
| `lidar-test.py` | Prints LiDAR device info, health status, and one sample scan |
| `lidar-reset.py` | Safely stops, idles, and disconnects the LiDAR after a crashed session |
| `plot_lidar.py` | Live polar plot of LiDAR scan data using matplotlib |
| `steering_test.py` | Interactive CLI tool to send manual servo angles to the Arduino over Serial |
| `arduino-files/angle-test/` | Arduino sketch — Serial-controlled servo angle tester |
| `arduino-files/calibrate/` | Arduino sketch — ESC calibration routine (max → min → neutral) |
| `arduino-files/esc-test/` | Arduino sketch — basic ESC run/stop cycle to verify wiring |
| `arduino-files/servo-test/` | Arduino sketch — auto-sweeps servo through 45°, 90°, 135° |

---

## Dependencies

```bash
pip install pyserial opencv-python numpy picamera2 RPi.GPIO
```

| Package | Purpose |
|---|---|
| `pyserial` | UART communication with the Arduino |
| `opencv-python` | Camera frame processing and colour detection |
| `numpy` | Array operations for image processing |
| `picamera2` | Raspberry Pi Camera interface |
| `RPi.GPIO` | Ultrasonic sensor and button GPIO control |

Arduino sketches use only the built-in `Servo.h` library — no extra installation needed.