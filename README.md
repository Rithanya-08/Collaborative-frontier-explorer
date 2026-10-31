# Distributed Multi-Robot Collaborative Exploration and Mapping for Unknown Indoor Environments

A robotics project demonstrating collaborative exploration, real-time mapping, and multi-robot coordination using Raspberry Pi 4 Model B.

## Project Overview

This project implements a scalable framework for multi-robot exploration and mapping with:
- **Phase 1**: Single robot autonomous navigation, obstacle detection, and SLAM
- **Phase 2**: Inter-robot communication and collaborative map merging
- **Phase 3**: Coordinated path planning and redundancy elimination

## Directory Structure

```
.
├── hardware/                    # Motor and sensor drivers
│   ├── __init__.py
│   ├── motor_controller.py     # DC motor control via L298N/TB6612FNG
│   ├── lidar_driver.py         # LIDAR sensor interface (RPLiDAR/Hokuyo)
│   ├── ultrasonic_driver.py    # HC-SR04 ultrasonic sensor
│   └── imu_driver.py           # MPU6050 IMU sensor
│
├── slam/                        # Mapping and localization
│   ├── __init__.py
│   ├── occupancy_grid.py       # Grid-based map representation
│   ├── particle_filter.py      # Particle filter localization
│   ├── scan_matcher.py         # Scan-to-scan matching
│   └── map_merger.py           # Multi-robot map merging (Phase 2)
│
├── navigation/                  # Path planning and movement
│   ├── __init__.py
│   ├── movement_controller.py  # Basic movement primitives
│   ├── frontier_explorer.py    # Frontier-based exploration
│   ├── path_planner.py         # Path planning algorithms
│   └── collision_avoidance.py  # Obstacle avoidance
│
├── communication/              # Multi-robot coordination (Phase 2)
│   ├── __init__.py
│   ├── robot_communicator.py   # Inter-robot messaging
│   ├── map_sync.py             # Map synchronization
│   └── coord_planner.py        # Collaborative path planning
│
├── app/                         # Visualization and monitoring
│   ├── __init__.py
│   ├── web_server.py           # Flask/FastAPI web server
│   ├── visualization.py        # Map visualization
│   ├── static/
│   │   ├── index.html
│   │   ├── styles.css
│   │   └── js/
│   │       └── map_viewer.js
│   └── templates/
│       └── dashboard.html
│
├── config/                      # Configuration files
│   ├── robot_config.yaml       # Robot hardware parameters
│   ├── slam_config.yaml        # SLAM parameters
│   └── nav_config.yaml         # Navigation parameters
│
├── tests/                       # Unit and integration tests
│   ├── test_motor_control.py
│   ├── test_lidar_driver.py
│   ├── test_slam.py
│   └── test_integration.py
│
├── scripts/                     # Utility scripts
│   ├── calibrate_motors.py     # Motor calibration
│   ├── test_sensors.py         # Sensor testing
│   └── setup_pi.sh             # Raspberry Pi environment setup
│
├── docs/                        # Documentation
│   ├── SETUP.md                # Hardware and software setup
│   ├── ARCHITECTURE.md         # System architecture
│   ├── API.md                  # Module API reference
│   └── DEPLOYMENT.md           # Deployment guide
│
├── main.py                      # Main robot control loop
├── requirements.txt             # Python dependencies
├── setup.py                     # Package setup
└── LICENSE
```

## Hardware Components

- **Robot Platform**: Custom-built differential drive robot
- **Main Controller**: Raspberry Pi 4 Model B
- **Sensors**:
  - LIDAR (RPLiDAR A1 / Hokuyo)
  - HC-SR04 Ultrasonic Sensor
  - MPU6050 IMU
- **Motor Control**: L298N or TB6612FNG motor driver
- **Actuators**: 4× BO motors with wheels
- **Power**: Motor battery pack (4×AA) + 5V/3A Pi power bank

## Software Stack

- **Language**: Python 3.7+
- **Key Libraries**:
  - `numpy`, `scipy` - Numerical computing
  - `opencv-python` - Image processing
  - `PyYAML` - Configuration management
  - `RPi.GPIO`, `board`, `busio` - Raspberry Pi GPIO
  - `flask` / `fastapi` - Web server for visualization
  - `matplotlib` - Map visualization
  
## Quick Start

### Phase 1: Single Robot Setup

1. **Setup Raspberry Pi**
   ```bash
   chmod +x scripts/setup_pi.sh
   ./scripts/setup_pi.sh
   ```

2. **Install Dependencies**
   ```bash
   pip install -r requirements.txt
   ```

3. **Calibrate Hardware**
   ```bash
   python scripts/calibrate_motors.py
   python scripts/test_sensors.py
   ```

4. **Run Main Robot Loop**
   ```bash
   python main.py
   ```

5. **View Map in Web Browser**
   Open http://localhost:5000 in your browser

## Phase 1 Milestones

- [ ] Motor control and wheel odometry
- [ ] LIDAR driver and scan processing
- [ ] Basic grid-based occupancy mapping
- [ ] Frontier detection and exploration
- [ ] Web-based map visualization
- [ ] Autonomous navigation in known environments
- [ ] Obstacle detection and avoidance

## Phase 2 Milestones (Multi-Robot)

- [ ] Inter-robot communication (WiFi/BLE)
- [ ] Map synchronization protocol
- [ ] Distributed map merging
- [ ] Collaborative frontier exploration
- [ ] Performance metrics comparison

## Performance Metrics

- Total exploration time
- Area coverage percentage
- Redundant path overlap
- Time to locate target object
- Communication bandwidth usage
- Localization accuracy (RMSE)

## Documentation

- See [SETUP.md](docs/SETUP.md) for detailed hardware and software setup
- See [ARCHITECTURE.md](docs/ARCHITECTURE.md) for system design details
- See [API.md](docs/API.md) for module reference

## Team

AI and Robotics - 7th Semester, Amrita University

## License

MIT License

## References

- RPLiDAR Documentation
- Raspberry Pi GPIO Documentation
- SLAM fundamentals (Particle Filter, Scan Matching)
- Frontier-based exploration (Yamauchi, 1997)
