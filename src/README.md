# Control Software

This directory contains all control software developed by the team for the WRO Future Engineers competition vehicle. The software stack is split into **main** competition scripts and **other** utilities used during development and calibration.

---

## Main Scripts

### `round_1.py`
The base Round 1 script. Uses a single RPLidar to measure distance to the **left wall** and applies a PID controller to hold the vehicle at 200 mm from it. Commands are sent to the Arduino as `steer,throttle\n` over Serial. The LiDAR is initialized safely before scanning begins.

### `round-1-left.py`
An extended Round 1 script that adds **lap counting** using the Intel RealSense camera. Detects the orange starting line in the bottom region of each frame using HSV color filtering. Every 4 orange detections counts as one lap — the vehicle automatically stops after completing 3 laps. Wall following logic is the same as `round_1.py` (left-side, 200 mm target).

### `round-2-right.py`
Same structure as `round-1-left.py` but follows the **right wall** instead (angle range 50°–95°), used when direction of travel is reversed (Clockwise). Also counts laps via the orange line and stops after 3. Intended for courses where the direction of travel keeps the right wall closer.

### `round-2.py`
The full Round 2 script. Follows the left wall by default but adds **colour block avoidance** using the `block_detector` module. When a red or green block is detected in the camera frame, the vehicle steers away from it for 1.5 seconds before resuming normal wall-following. Lap counting is the same as the other Round 2 variants.

### `lidar_follow.py`
A centered wall-following script. Reads LiDAR distances from both the **left (265°–310°)** and **right (50°–95°)** sides and steers to keep the vehicle equidistant between them. The PID error is `left_dist - right_dist`.

### `motor-run/motor-run.ino`
Arduino firmware that listens for `angle,speed\n` commands over UART and drives the **steering servo (D3)** and **ESC motor (D7)**. Angles are clamped to 45°–135° and ESC values to 1500–2000 µs. On startup, the ESC is armed briefly before the vehicle is ready to receive commands.

---

## Running the Main Scripts

```bash
# Round 1 — basic left-wall follower (LiDAR only)
python3 src/main/round_1.py

# Round 1 with lap counting (LiDAR + RealSense)
python3 src/main/round-1-left.py

# Round 2 — right-wall follower with lap counting
python3 src/main/round-2-right.py

# Round 2 — left-wall follower with block avoidance and lap counting
python3 src/main/round-2.py

# Centered wall follower (both walls)
python3 src/main/lidar_follow.py
```

> **Port note:** The LiDAR uses `/dev/ttyUSB1` and the Arduino uses `/dev/ttyUSB0` by default. Verify with `ls /dev/ttyUSB*` and update the port constants at the top of each script if they differ on your system.

---

## Utility Scripts (`other/`)

| Script / Sketch | Purpose |
|---|---|
| `cam-test.py` | Verifies the Intel RealSense camera is detected over USB |
| `lidar-test.py` | Prints LiDAR device info, health status, and one sample scan |
| `lidar-reset.py` | Safely stops, idles, and disconnects the LiDAR — useful after a crashed session |
| `plot_lidar.py` | Live polar plot of LiDAR scan data using matplotlib |
| `steering_test.py` | Interactive CLI tool to send manual servo angles to the Arduino over Serial |
| `arduino-files/angle-test/` | Arduino sketch — Serial-controlled servo angle tester |
| `arduino-files/calibrate/` | Arduino sketch — ESC calibration routine (max → min → neutral) |
| `arduino-files/esc-test/` | Arduino sketch — basic ESC run/stop cycle to verify wiring |
| `arduino-files/servo-test/` | Arduino sketch — auto-sweeps servo through 45°, 90°, 135° |

---

## Dependencies

```bash
pip install rplidar-roboticia pyserial pyrealsense2 opencv-python numpy matplotlib
```

| Package | Purpose |
|---|---|
| `rplidar-roboticia` | RPLidar A-series driver |
| `pyserial` | UART communication with the Arduino |
| `pyrealsense2` | Intel RealSense camera SDK |
| `opencv-python` | Camera frame processing and colour detection |
| `numpy` | Array operations for image and LiDAR data |
| `matplotlib` | Live LiDAR polar plot (`plot_lidar.py`) |

Arduino sketches use only the built-in `Servo.h` library — no extra installation needed.