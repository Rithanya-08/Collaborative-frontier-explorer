# System Architecture

## Overview

Multi-Robot Collaborative Exploration and Mapping System
- **Phase 1**: Single robot autonomous navigation and SLAM
- **Phase 2**: Multi-robot communication and collaboration
- **Platform**: Raspberry Pi 4 Model B with custom differential-drive robot

## Hardware Architecture

```
┌─────────────────────────────────────────────────────────┐
│                  Raspberry Pi 4 Model B                  │
├─────────────────────────────────────────────────────────┤
│                                                           │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐   │
│  │   GPIO Interface              GPIO I2C/Serial   │   │
│  │   (RPi.GPIO)  │  │ (Adafruit)   │  │  (PySerial) │   │
│  └──────────────┘  └──────────────┘  └──────────────┘   │
│         │                 │                 │             │
├─────────┼─────────────────┼─────────────────┼─────────────┤
│                           │                 │             │
│  Motor Control      IMU (I2C)        LIDAR (Serial)       │
└─────────────────────────────────────────────────────────┘
        │                   │                 │
        ▼                   ▼                 ▼
   L298N Driver       MPU6050 (6-axis)   RPLiDAR A1
   (2 Motors)      Gyro + Accelerometer
                                            │
                    ┌───────────────────────┼─────────────┐
                    ▼                       ▼             ▼
                Left Motor           Right Motor    Ultrasonic
                (PWM)                (PWM)          (×1-3, GPIO)
```

## Software Architecture

### Module Structure

```
Project/
├── hardware/                    # Low-level hardware drivers
│   ├── motor_controller.py     # L298N/TB6612FNG motor control
│   ├── lidar_driver.py         # LIDAR data acquisition
│   ├── imu_driver.py           # IMU sensor fusion
│   └── ultrasonic_driver.py    # Obstacle detection
│
├── slam/                        # Mapping and localization
│   ├── occupancy_grid.py       # Grid-based map (raycasting)
│   ├── particle_filter.py      # Monte Carlo localization (future)
│   ├── scan_matcher.py         # ICP or scan-to-scan (future)
│   └── map_merger.py           # Multi-robot map fusion (Phase 2)
│
├── navigation/                  # Path planning and control
│   ├── frontier_explorer.py    # Frontier-based exploration
│   ├── movement_controller.py  # Movement primitives
│   ├── path_planner.py         # Path planning algorithms
│   └── collision_avoidance.py  # Obstacle avoidance
│
├── communication/              # Multi-robot coordination (Phase 2)
│   ├── robot_communicator.py   # WiFi/MQTT messaging
│   ├── map_sync.py             # Map exchange protocol
│   └── coord_planner.py        # Coordination algorithms
│
├── app/                         # Visualization & monitoring
│   ├── web_server.py           # Flask visualization server
│   ├── visualization.py        # Real-time map display
│   └── static/                 # Web assets (JS, CSS, HTML)
│
├── config/                      # Configuration files
│   ├── robot_config.yaml       # Robot parameters
│   ├── slam_config.yaml        # SLAM settings
│   └── nav_config.yaml         # Navigation parameters
│
├── main.py                      # Main control loop
└── requirements.txt             # Python dependencies
```

### Data Flow Diagram

```
┌────────────────────────┐
│   Sensor Hardware      │
├─LIDAR─┬──IMU──┬────────┤
└───┬───┴───┬───┴────────┘
    │       │
    ▼       ▼
  ┌───────────────────────────────┐
  │   Hardware Drivers            │
  │  (Motor, LIDAR, IMU, Sonic)  │
  └───────────────────────────────┘
    │
    ▼
  ┌────────────────────────────────┐
  │   Sensor Fusion / Preprocessing│
  │  (Filtering, Calibration)      │
  └────────────────────────────────┘
    │
    ├─────────────┬────────────────┐
    ▼             ▼                ▼
  ┌──────────┐ ┌──────────┐  ┌──────────────┐
  │   SLAM   │ │  Locali- │  │  Odometry    │
  │ (Map)    │ │ zation   │  │ (Dead Reckng)│
  └──────────┘ └──────────┘  └──────────────┘
    │             │                │
    └─────────────┼────────────────┘
                  ▼
            ┌─────────────────┐
            │ State Estimate  │
            │ (Pos, Heading)  │
            └─────────────────┘
                  │
      ┌───────────┼───────────┐
      ▼           ▼           ▼
  ┌─────────┐ ┌─────────┐ ┌────────────┐
  │Frontier │ │Path     │ │Collision   │
  │Explorer │ │Planner  │ │Avoidance   │
  └─────────┘ └─────────┘ └────────────┘
      │           │           │
      └───────────┼───────────┘
                  ▼
            ┌───────────────┐
            │Movement Cmd   │
            │(Speed, Turn)  │
            └───────────────┘
                  │
                  ▼
            ┌───────────────┐
            │Motor Control  │
            │(PWM, GPIO)    │
            └───────────────┘
                  │
                  ▼
            ┌───────────────┐
            │  Motors →     │
            │  Movement     │
            └───────────────┘
```

### Key Algorithms

#### 1. Occupancy Grid Mapping
- **Type**: Grid-based probabilistic mapping
- **Cell Size**: 5cm × 5cm
- **Update Rule**: Bayesian inverse sensor model
- **Ray Casting**: Bresenham line algorithm
- **Resolution**: 500×500 cells covering 25m×25m area

#### 2. Frontier-Based Exploration
- **Frontier Detection**: Free cells adjacent to unknown cells
- **Clustering**: Flood-fill algorithm to group frontiers
- **Target Selection**: Closest frontier to robot
- **Progress Metric**: Coverage percentage

#### 3. Differential Drive Control
- **Model**: 2 independent wheels (left & right)
- **Kinematics**: Differential drive equations
- **Speed Control**: PWM (0-255)
- **Turning Radius**: Configurable via wheel base

#### 4. IMU Heading Integration
- **Integration Method**: Gyroscope drift compensation
- **Calibration**: Stationary offset measurement
- **Update Rate**: 50 Hz target
- **Angle Normalization**: 0-360 degrees

## Communication Architecture (Phase 2)

### Inter-Robot Communication

```
Robot 1                          Robot 2
┌──────────────┐                ┌──────────────┐
│ Local Map    │                │ Local Map    │
│ Position     │                │ Position     │
└──────────────┘                └──────────────┘
      │                              │
      │        WiFi/MQTT             │
      ├──────────────────────────────┤
      │        Map Exchange          │
      │        Frontier Sync         │
      │        Localization Hints    │
      │                              │
      ▼                              ▼
┌──────────────┐                ┌──────────────┐
│Merged Map    │◄────────────────►│ Merged Map   │
│Global Coords │                │ Global Coords│
└──────────────┘                └──────────────┘
      │                              │
      ├──────────────────────────────┤
      │    Coordinate Agreement      │
      │    Redundancy Elimination    │
      │    Collaborative Planning    │
      │                              │
      ▼                              ▼
   Explorer 1                    Explorer 2
   (Different                    (Different
    Frontiers)                    Frontiers)
```

### Map Merging Process

1. **Coordinate Transform**
   - Robot 2's map → Robot 1's coordinate frame
   - Uses relative pose between robots

2. **Grid Alignment**
   - Find corresponding cells in both maps
   - Account for rotation and translation

3. **Occupancy Fusion**
   - Weighted average of conflicting cells
   - Confidence-based resolution

4. **Frontier Update**
   - Remove explored areas
   - Add new frontiers

## Execution Flow

### Phase 1: Single Robot

```
START
  │
  ▼
Initialize Hardware
  │
  ├─► Motor Controller
  ├─► LIDAR
  ├─► IMU
  ├─► Ultrasonic
  │
  ▼
Calibrate Sensors
  │
  ▼
Main Control Loop (50 Hz)
  │
  ├─► Update Sensors (LIDAR, IMU, Ultrasonic)
  │
  ├─► Update SLAM (Occupancy Grid)
  │
  ├─► Find Frontier (Closest unexplored area)
  │
  ├─► Plan Path (Vector to frontier)
  │
  ├─► Avoid Obstacles (Collision check)
  │
  ├─► Execute Movement (Motor commands)
  │
  ├─► Monitor Progress
  │   │
  │   ├─► Coverage? > 80% → Done
  │   │
  │   └─► No Frontier? → Done
  │
  └─► Continue...
      │
      ▼
Exploration Complete
  │
  ▼
Save Map
  │
  ▼
Shutdown
  │
  ▼
END
```

## Performance Metrics

### Real-Time Constraints
- **Sensor Loop**: 50 Hz (20ms)
- **SLAM Update**: 20 Hz (50ms)
- **Navigation Update**: 10 Hz (100ms)
- **Visualization Update**: 2-5 Hz (200-500ms)

### Memory Usage
- **Grid Storage**: 500×500 grid ≈ 250KB (int8)
- **Confidence Map**: 500×500 ≈ 250KB (uint8)
- **Total SLAM Memory**: ~1 MB

### Typical Performance (Raspberry Pi 4B)
- **LIDAR Processing**: ~5-10ms per scan
- **Grid Update**: ~2-5ms
- **Frontier Detection**: ~10-20ms
- **Path Planning**: ~5-10ms
- **Motor Control**: <1ms

## Future Enhancements (Phase 2+)

1. **Advanced SLAM**
   - Particle filter localization
   - Scan matching (ICP)
   - Loop closure detection

2. **Multi-Robot Coordination**
   - Distributed frontier allocation
   - Explicit communication protocol
   - Role-based exploration

3. **Optimization**
   - GPU-accelerated processing (via OpenGL)
   - Machine learning for frontier selection
   - Multi-threading optimization

4. **Deployment**
   - Docker containerization
   - Kubernetes for multi-robot fleets
   - Cloud connectivity (AWS/Azure)

## References

- Yamauchi, B. (1997). "A frontier-based approach for autonomous exploration"
- Elfes, A. (1989). "Using occupancy grids for mobile robot perception and navigation"
- Thrun, S., Burgard, W., Fox, D. (2005). "Probabilistic Robotics"
