# Quick Start Guide - Multi-Robot Exploration

## For Development/Testing (Without Hardware)

### 1. Install Dependencies (Any Computer)

```bash
cd "Project"
pip3 install -r requirements.txt
```

### 2. Test Individual Modules

```bash
# Test occupancy grid (no hardware needed)
python3 -m slam.occupancy_grid

# Test frontier explorer
python3 -m navigation.frontier_explorer

# Test collision avoidance
python3 -m navigation.collision_avoidance
```

### 3. Run Full Robot Simulation

The main.py already includes mock sensor data. To test without hardware:

```bash
# This will run in simulation mode (no real motor/sensor control)
python3 main.py
```

Simulation features:
- LIDAR generates mock circular obstacles
- Motors accept commands (but don't move)
- Exploration algorithm runs normally
- Map saves after completion

## On Raspberry Pi Hardware

### 1. Initial Setup (First Time Only)

```bash
# SSH into Raspberry Pi
ssh pi@<pi_ip_address>

# Clone project (if using git)
git clone <your-repo-url>
cd Project

# Or extract from downloaded zip
unzip project.zip && cd Project
```

### 2. Install System Dependencies

```bash
chmod +x scripts/setup_pi.sh
./scripts/setup_pi.sh
```

This installs:
- Python3 GPIO and I2C libraries
- Required system packages
- Python pip packages

### 3. Verify Hardware

Before running exploration:

```bash
# Check I2C devices (IMU should appear at 0x68)
i2cdetect -y 1

# List serial ports (LIDAR)
ls /dev/ttyUSB*

# Test GPIO access
python3 -c "import RPi.GPIO as GPIO; GPIO.setmode(GPIO.BCM); print('GPIO OK')"
```

### 4. Configure Hardware

Edit `config/robot_config.yaml`:

```yaml
# Update GPIO pin assignments if needed
motor:
  left_forward_pin: 17      # Adjust based on your wiring
  left_backward_pin: 27
  left_pwm_pin: 22
  # ... etc

lidar:
  port: "/dev/ttyUSB0"      # Check with: ls /dev/ttyUSB*

imu:
  i2c_address: 0x68         # Check with: i2cdetect -y 1
```

### 5. Calibrate Motors

```bash
# Ensure robot can move forward in straight line
python3 scripts/calibrate_motors.py

# Follow on-screen instructions:
# 1. Robot will move forward
# 2. Observe if it drifts left or right
# 3. Adjust motor speeds in config if needed
```

### 6. Test All Sensors

```bash
python3 scripts/test_sensors.py

# Output should show:
# ✓ Motor controller initialized
# ✓ LIDAR: 360 measurements
# ✓ IMU: Acceleration, Gyro, Heading
# ✓ Ultrasonic: Front, Left, Right distances
```

### 7. Run Autonomous Exploration

```bash
# Terminal 1: Start robot exploration
python3 main.py

# You should see:
# - Sensor readings
# - Map coverage percentage increasing
# - Robot movement status
# - Map saved when done
```

### 8. (Optional) View Real-Time Map

```bash
# Terminal 2 (on Raspberry Pi): Start visualization server
python3 app/web_server.py

# On your computer:
# Open browser: http://<pi_ip>:5000
# See live map and robot position
```

## Troubleshooting Common Issues

### Motors Don't Move
```bash
# 1. Check GPIO connections
python3 -c "
from hardware.motor_controller import MotorController
m = MotorController()
m.move_forward(200, duration=2)
m.cleanup()
"

# 2. Verify power to motor driver (12V and 5V)
# 3. Check motor battery status
# 4. Adjust GPIO pin numbers in config
```

### LIDAR Not Found
```bash
# 1. Check USB connection
ls /dev/ttyUSB*

# 2. Check permissions
sudo usermod -a -G dialout pi

# 3. Test LIDAR directly
python3 -c "
from hardware.lidar_driver import LidarDriver
l = LidarDriver()
l.start_scanning()
import time; time.sleep(1)
print(l.get_latest_scan()[:5])
l.cleanup()
"
```

### IMU Not Detected
```bash
# 1. Check I2C connection
i2cdetect -y 1

# 2. Should show 68 if using default address
# 3. If not found, verify SDA/SCL wiring

# 3. Test IMU directly
python3 -c "
from hardware.imu_driver import IMUDriver
i = IMUDriver()
print(i.get_all_data())
i.cleanup()
"
```

### Robot Drifts During Movement
```bash
# Run motor calibration
python3 scripts/calibrate_motors.py

# Adjust left/right motor speeds in config until robot moves straight
```

## Performance Monitoring

### Real-Time Monitoring

```bash
# While main.py is running, you can check:
# - Map coverage: displayed every 5 seconds
# - Frontiers detected: logged in output
# - Position: shows in status line
# - Memory usage: systemctl status or top
```

### Save Maps Between Runs

Maps are automatically saved to `maps/` directory:

```bash
# View saved maps
ls -la maps/

# Maps are in JSON format (can be visualized with app/visualization.py)
```

## Next Steps After Phase 1

Once single-robot exploration is working:

1. **Optimize Performance**
   - Reduce grid resolution (if memory-limited)
   - Increase scan frequency
   - Fine-tune navigation parameters

2. **Add Second Robot**
   - Build second identical robot
   - Assign unique ID in config
   - Test both locally

3. **Network Communication**
   - Configure WiFi for both robots
   - Set up MQTT broker (or use Python sockets)
   - Implement map sharing protocol

4. **Coordinate Exploration**
   - Implement frontier allocation
   - Merge maps from both robots
   - Compare single vs. dual-robot performance

## Key Files to Edit

- `config/robot_config.yaml` - Hardware configuration
- `main.py` - Main control loop
- `navigation/frontier_explorer.py` - Exploration strategy
- `slam/occupancy_grid.py` - SLAM algorithm
- `hardware/*.py` - Sensor drivers

## Additional Resources

- Full setup instructions: [docs/SETUP.md](../docs/SETUP.md)
- System architecture: [docs/ARCHITECTURE.md](../docs/ARCHITECTURE.md)
- Hardware wiring diagram: [docs/SETUP.md#wiring-diagram](../docs/SETUP.md#wiring-diagram)
- API reference: [docs/API.md](../docs/API.md)

---

**Ready to explore!** 🤖
