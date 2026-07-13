# Control Software

This directory contains all control software developed by the team for the WRO Future Engineers competition vehicle. The software stack is split into **main** competition scripts and **other** utilities used during development and calibration.

---

## Structure

```
src/
├── main/
│   ├── round_1.py          # Round 1: left-wall PID follower
│   ├── lidar_follow.py     # Centered wall follower (left + right LiDAR)
│   └── motor-run/
│       └── motor-run.ino   # Arduino firmware for steering + throttle
└── other/                  # Calibration & testing utilities
    ├── cam-test.py         # RealSense camera connection test
    ├── lidar.py            # Safely idles/disconnects the LiDAR
    ├── steering_test.py    # Interactive servo angle tester
    └── arduino-files/
        ├── angle-test/     # Serial-controlled servo tester
        ├── calibrate/      # ESC calibration routine
        ├── esc-test/       # Basic ESC run/stop test
        └── servo-test/     # Automatic servo sweep test
```

---

## Main Scripts

### `round_1.py`
The primary competition script for Round 1. Uses a single RPLidar to measure distance to the **left wall** and applies a **PID controller** to keep the vehicle at a fixed offset from it. Commands are sent to the Arduino as `<THROTTLE:...><STEER:...>` over Serial.

### `lidar_follow.py`
An alternative control script that reads from **both the left and right walls** and steers to keep the vehicle centered between them. Uses the same PID approach but with `left_dist - right_dist` as the error signal.

### `motor-run/motor-run.ino`
Arduino firmware that receives `angle,speed\n` commands over UART and applies them to the **steering servo (D3)** and **ESC motor (D7)**. On startup it arms the ESC before accepting commands.

---

## Running the Main Scripts

```bash
# Round 1 — left-wall PID follower
python3 src/main/round_1.py

# Centered wall follower (left + right LiDAR)
python3 src/main/lidar_follow.py
```

> **Port note:** The LiDAR and Arduino share `/dev/ttyUSB0` and `/dev/ttyUSB1`. Verify the assignment with `ls /dev/ttyUSB*` and update the port constants at the top of each script before running.

---

## Utility Scripts (`other/`)

| Script / Sketch | Purpose |
|---|---|
| `cam-test.py` | Verifies Intel RealSense camera is detected over USB |
| `lidar.py` | Safely stops and disconnects the RPLidar between sessions |
| `steering_test.py` | Sends manual angles to the servo via Serial for physical verification |
| `arduino-files/angle-test/` | Serial-controlled sketch to test servo angles directly |
| `arduino-files/calibrate/` | ESC calibration routine — sets max, min, and neutral throttle |
| `arduino-files/esc-test/` | Runs ESC through a slow-speed/stop cycle to verify wiring |
| `arduino-files/servo-test/` | Auto-sweeps servo through 45°, 90°, 135° to verify travel limits |

---

## Dependencies

```bash
pip install rplidar-roboticia pyserial pyrealsense2
```

| Package | Purpose |
|---|---|
| `rplidar-roboticia` | RPLidar A-series driver |
| `pyserial` | UART communication with the Arduino |
| `pyrealsense2` | Intel RealSense camera SDK |

Arduino sketches use only the built-in `Servo.h` library — no extra installation needed.