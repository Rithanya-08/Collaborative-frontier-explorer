# Collaborative Frontier Explorer

> **Distributed multi-robot autonomous exploration and mapping of unknown indoor environments**
>
> 7th Semester — AI & Robotics · Amrita University

Two robots explore an unknown room simultaneously. Each robot builds its own occupancy-grid map from ultrasonic range scans and shares every scan with its teammate over MQTT, so both end up with a merged map of the whole space faster than either could alone.

---

## Robots

| | Robot A | Robot B |
|---|---|---|
| **Brain** | Raspberry Pi 4 (runs Python) | Laptop via `hardware/esp32_robot.py` |
| **Body** | Raspberry Pi 4 + GPIO | ESP32 DevKit (38-pin,PlatformIO) |
| **Camera** | **RPi Camera Module (OV5647)** — YOLOv8n inference on the Pi (`picamera2`) | **ESP32-CAM (OV2640)** — raw MJPEG stream over WiFi only |
| **Motors** | 4× TT motors,2x TB6612FNG driver | 4× TT motors, 2× TB6612FNG drivers |
| **Ranging** | 3× HC-SR04 ultrasonic (front/left/right) | 3× HC-SR04 ultrasonic (front/left/right) |
| **IMU** | MPU6050 (I2C) | MPU6050 (I2C) |
| **Firmware** | Python (`main.py`) | C++ PlatformIO (`firmware/robot_b/`) |

Robot B's **body** (motors, sensors) is controlled by the ESP32. Its **brain** (SLAM, frontier planning, MQTT logic) runs on the laptop inside `hardware/esp32_robot.py`, which sends timed move commands over MQTT and receives sensor state back at 10 Hz.

Robot A's **Pi Camera Module** runs YOLOv8n (ONNX via OpenCV DNN) directly on the Raspberry Pi at ~3 fps, estimating each object's bearing from the frame and range from the front ultrasonic to place it on the shared map. Robot B's **ESP32-CAM** is a separate board (no wires to the DevKit) that streams raw MJPEG at `http://<esp32cam-ip>:81/stream`; its stream URL is published to MQTT (`robot/B/cam`) so the laptop can pick it up without the serial monitor.

---

## How It Works

```
┌──────────────────────────────────────────────────────┐
│                   Shared WiFi network                │
│                                                      │
│   Raspberry Pi 4 (Robot A)    Laptop (Robot B brain) │
│   ┌─────────────────────┐     ┌────────────────────┐ │
│   │ main.py             │     │ esp32_robot.py     │ │
│   │  ├─ IMU / ultrasonic│     │  ├─ MQTT → ESP32   │ │
│   │  ├─ OccupancyGrid   │     │  ├─ OccupancyGrid  │ │
│   │  ├─ FrontierExplorer│◄────►  ├─ FrontierExplorer│ │
│   │  ├─ PathPlanner     │MQTT │  ├─ PathPlanner     │ │
│   │  ├─ MapMerger       │     │  └─ MapMerger       │ │
│   │  └─ ObjectDetector  │     └────────┬───────────┘ │
│   └─────────────────────┘              │ MQTT cmds   │
│                                   ┌───▼──────────┐   │
│                              Mosquitto broker     │   │
│                                   └───────────────┘   │
│                                        │             │
│                              ┌─────────▼──────────┐  │
│                              │  ESP32 DevKit       │  │
│                              │  firmware/robot_b/  │  │
│                              │  (motors, IMU, US)  │  │
│                              └─────────────────────┘  │
└──────────────────────────────────────────────────────┘
```

### Core Loop (both robots, ~50 Hz)

1. **Sense** — read IMU heading + 3 ultrasonic distances (front/left/right)
2. **Map** — Bresenham ray-cast each reading into a 500×500 probabilistic occupancy grid (5 cm/cell → 25 m × 25 m)
3. **Share** — broadcast pose + scan over MQTT; merge teammate's scans into local map
4. **Plan** — frontier-based exploration: find boundaries between free and unknown space, pick the nearest unvisited one, route there with A\*/RRT/PRM
5. **Avoid** — reactive safety layer overrides navigation when any sensor reads < 30 cm (front) or < 15 cm (side)
6. **Coordinate** — frontier allocation steers each robot away from the target its teammate is already heading to

---

## Repository Layout

```
.
├── main.py                    # Robot A entry point (run on the Raspberry Pi)
├── run_team.py                # Launch both robots from one terminal (laptop)
├── run_main_and_map.py        # Run Robot A + live map render in parallel
├── merge_maps.py              # Merge two saved map JSONs offline
├── render_latest_map.py       # Render the most recent map/*.json to PNG
├── calibrate_forward_speed.py # Measure actual distance per forward burst
├── calibrate_turn_rate.py     # Measure actual turn rate (deg/s)
├── requirements.txt
│
├── hardware/
│   ├── motor_controller.py    # L298N differential drive (Robot A, RPi GPIO)
│   ├── imu_driver.py          # MPU6050 gyro integration + heading tracking
│   ├── ultrasonic_driver.py   # HC-SR04 3-sensor array (shared TRIG)
│   └── esp32_robot.py         # Robot B body: MQTT ↔ ESP32 bridge
│
├── firmware/
│   ├── robot_b/               # ESP32 DevKit firmware (PlatformIO)
│   │   ├── src/main.cpp       # Motors, IMU, ultrasonics, MQTT command handler
│   │   ├── platformio.ini     # Board: esp32dev, libs: PubSubClient + ArduinoJson
│   │   └── include/
│   │       └── secrets.example.h  # Copy to secrets.h and fill WiFi/MQTT creds
│   ├── esp32_cam/             # ESP32-CAM MJPEG stream firmware
│   └── hw_test/               # Standalone hardware test sketch
│
├── slam/
│   └── occupancy_grid.py      # Probabilistic grid map, Bresenham ray casting, JSON save
│
├── navigation/
│   ├── frontier_explorer.py   # Flood-fill frontier detection & clustering
│   ├── path_planner.py        # A*, RRT, PRM path planning
│   └── collision_avoidance.py # Reactive obstacle avoidance primitives
│
├── communication/
│   └── robot_communicator.py  # MQTT pub/sub, MapMerger, CoordinationPlanner
│
├── vision/
│   ├── object_detector.py     # YOLOv8n (ONNX/OpenCV DNN) on RPi Camera Module — Robot A only
│   └── live_view.py           # Standalone Pi camera live view (no driving)
│
├── app/
│   ├── web_server.py          # Flask dashboard (map + robot status)
│   └── templates/dashboard.html
│
├── config/
│   └── robot_config.yaml      # GPIO pins, SLAM params, nav thresholds
│
├── scripts/                   # Standalone test & calibration utilities
│   ├── test_sensors.py
│   ├── test_ultrasonic.py
│   ├── calibrate_motors.py
│   ├── obstacle_avoidance.py
│   └── ...
│
├── maps/                      # Generated map files (gitignored, folder tracked)
└── docs/
    ├── HARDWARE_SETUP.md      # Full wiring tables & GPIO assignments (both robots)
    ├── ARCHITECTURE.md        # System design & algorithm details
    └── SETUP.md               # Software setup & Raspberry Pi configuration
```

---

## Quick Start

### Robot B (ESP32) — Flash Firmware First

```bash
# 1. Copy and fill in WiFi + MQTT credentials
cp firmware/robot_b/include/secrets.example.h firmware/robot_b/include/secrets.h
# edit secrets.h: WIFI_SSID, WIFI_PASS, MQTT_HOST, MQTT_PORT

# 2. Open firmware/robot_b/ in VS Code (PlatformIO) and click Upload
#    or from CLI:
cd firmware/robot_b
pio run --target upload
```

The ESP32 auto-scans the local /24 subnet on first boot to find the MQTT broker. It remembers the IP in NVS so subsequent boots are instant.

### Laptop — Start MQTT Broker

```bash
# Windows (install Mosquitto from mosquitto.org)
net start mosquitto

# Linux / macOS
mosquitto -v
```

### Robot A (Raspberry Pi)

```bash
# Install dependencies (once)
pip3 install -r requirements.txt

# Calibrate (first time on real hardware)
python3 calibrate_forward_speed.py
python3 calibrate_turn_rate.py

# Run
python3 main.py --robot a --broker <laptop-ip>
```

### Both Robots Together (from the laptop)

```bash
python3 run_team.py --broker <laptop-ip>
```

This launches Robot A's brain over SSH and Robot B's brain locally, keeps both map streams live, and saves a merged map on exit.

### View Live Dashboard

```bash
# On the Pi (or laptop for Robot B)
python3 app/web_server.py
# Open http://<host>:5000
```

---

## Key Parameters

All tunable constants live at the top of `main.py` and in `config/robot_config.yaml`.

| Constant | Default | Meaning |
|---|---|---|
| `OBSTACLE_THRESHOLD_M` | 0.30 m | Front stop distance |
| `SIDE_OBSTACLE_THRESHOLD_M` | 0.15 m | Side stop distance |
| `FORWARD_STEP_DISTANCE_M` | 0.051 m | Calibrated distance per forward burst (Robot A) |
| `TURN_RATE_DEG_PER_S` | 30 °/s | Calibrated turn rate (Robot A) |
| `ROBOT_B_FORWARD_STEP_DISTANCE_M` | 0.232 m | Same for Robot B |
| `ROBOT_B_TURN_RATE_DEG_PER_S` | 49.4 °/s | Same for Robot B |
| `REPLAN_INTERVAL_S` | 2.0 s | How often to re-run path planner |
| Grid size | 500×500 @ 5 cm | 25 m × 25 m map |

---

## Start Poses

Place the robots on marked floor spots before running:

| Robot | Start (x, y) | Heading |
|---|---|---|
| A | (0, 0) | +x (forward) |
| B | (0, −0.5 m) | +x (forward) |

Both robots share the same coordinate frame — mark these spots on the floor so the merged map is accurate.

---
## Demo Videos
**Single robot Exploration**
https://github.com/user-attachments/assets/51573fef-a1c3-4617-8ad4-3582383ead9d
**Multi robot Exploration**
https://github.com/user-attachments/assets/e8d8fe13-69ec-4073-a1e2-be1c797bf2f5

**Shared map**
<img width="1200" height="1200" alt="Image" src="https://github.com/user-attachments/assets/94ce3950-c4dd-4c5d-9237-3a3915faccfa" />

---
## Performance Metrics

| Metric | Measured |
|---|---|
| Control loop rate | 50 Hz |
| Map resolution | 5 cm/cell |
| Map size | 25 m × 25 m |
| Ultrasonic cycle | ~180 ms (3 sensors × 60 ms slot) |
| MQTT state publish | 10 Hz (Robot B → laptop) |
| Scan share rate | Every control loop iteration |

---

## Documentation

| File | Contents |
|---|---|
| [`docs/HARDWARE_SETUP.md`](docs/HARDWARE_SETUP.md) | Full wiring, GPIO table, power diagram, safety checklist |
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | SLAM, frontier algorithm, coordination, path planning |
| [`docs/SETUP.md`](docs/SETUP.md) | Raspberry Pi OS setup, I2C/serial config, dependency install |
| [`firmware/robot_b/src/main.cpp`](firmware/robot_b/src/main.cpp) | ESP32 firmware (documented inline) |

---

## Team

This project was developed as a case study under the following academic context:

| | |
|---|---|
| **Course** | 23CSE472 – Artificial Intelligence and Robotics |
| **Mentor** | Dr. A. Baskar |
| **Department** | Department of Computer Science and Engineering |
| **Institution** | School of Computing, Amrita Vishwa Vidyapeetham, Coimbatore, India |

### Members

| Name | Roll Number |
|---|---|
| Gaddam Jahnavi | CB.SC.U4CSE23023 |
| U Gayathri | CB.SC.U4CSE23026 |
| Rithanya K A | CB.SC.U4CSE23140 |
| Thota Neha | CB.SC.U4CSE23165 |
| Maddipatla Hasini Reddy | CB.SC.U4CSE23529 |

---

## References

- Yamauchi, B. (1997). *A frontier-based approach for autonomous exploration*
- RPLiDAR / Slamtec documentation
- Raspberry Pi GPIO reference — https://pinout.xyz
- PubSubClient (MQTT) — knolleary/PubSubClient
- ArduinoJson — bblanchon/ArduinoJson
- Ultralytics YOLOv8 — https://github.com/ultralytics/ultralytics
