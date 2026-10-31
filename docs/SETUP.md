# Hardware and Software Setup Guide

## Phase 1: Hardware Assembly

### Components Required

#### Controller & Power
- **Raspberry Pi 4 Model B** (4GB or 8GB RAM recommended)
- **Micro-USB/USB-C Power Adapter** (5V/3A minimum)
- **Power Bank** (5V/3A, ~20000mAh for extended runtime)
- **Micro-SD Card** (32GB Class 10 recommended)

#### Sensors
- **IMU**: MPU6050 (GY-521 module) - planned, not physically wired yet
  - Breakout board with I2C interface
- **Ultrasonic**: HC-SR04 × 3 (front, left, right)
  - Shared TRIG line across all 3 sensors
  - Each ECHO line needs a voltage divider (5V → ~3.3V)

#### Motor & Drive System
- **Motor Drivers**: 2× TB6612FNG modules
  - One per side (left/right); each drives a front and rear TT motor on
    its Channel A / Channel B
- **DC Motors**: 4× TT gear motors (front-left, rear-left, front-right, rear-right)
  - With wheel attachments
- **Motor Power**: 4×AA alkaline battery pack (~6V)
  - Powers VM/GND on both drivers
- **Logic Power**: Pi 3.3V → VCC on both drivers
- **Wheels & Chassis**: 4× wheels, 4-wheel drive
  - Wheel diameter: ~6.5cm (TT motor standard)

#### Miscellaneous
- **Jumper wires** (male-male, male-female)
- **Breadboard** (for prototyping connections)
- **Tape & markers** (for arena boundaries)
- **Obstacles**: Boxes, chairs, or blocks for arena

### Wiring Diagram

See "Wiring details.docx" for the authoritative connections. Summary below
(BCM numbering).

#### Motor Connections (2x TB6612FNG)
```
Raspberry Pi GPIO          Driver 1 (LEFT)         Driver 2 (RIGHT)
─────────────────          ───────────────         ────────────────
GPIO 21 ──────────────────→ STBY  ──────────────────→ STBY (shared)
GPIO 12 ──────────────────→ PWMA (front-left)
GPIO 5  ──────────────────→ AIN1
GPIO 6  ──────────────────→ AIN2
GPIO 13 ──────────────────→ PWMB (rear-left)
GPIO 16 ──────────────────→ BIN1
GPIO 20 ──────────────────→ BIN2
GPIO 18 ────────────────────────────────────────────→ PWMA (front-right)
GPIO 23 ────────────────────────────────────────────→ AIN1
GPIO 24 ────────────────────────────────────────────→ AIN2
GPIO 19 ────────────────────────────────────────────→ PWMB (rear-right)
GPIO 25 ────────────────────────────────────────────→ BIN1
GPIO 26 ────────────────────────────────────────────→ BIN2

Both drivers            Power
─────────────           ─────
VM, GND ────────→ 4xAA battery pack (~6V)
VCC ────────────→ Pi 3.3V
GND ────────────→ common ground (battery, both drivers, Pi)
```

#### IMU Connections (MPU6050 via I2C) - planned, not wired yet
```
Raspberry Pi GPIO          MPU6050 Module
─────────────────          ──────────────
GPIO 2 (SDA) ────────────→ SDA
GPIO 3 (SCL) ────────────→ SCL
3.3V ─────────────────────→ VCC
GND ──────────────────────→ GND
```

#### Ultrasonic Connections (3x HC-SR04, shared TRIG)
```
Raspberry Pi GPIO          HC-SR04 (front / left / right)
─────────────────          ───────────────────────────────
GPIO 27 (shared) ─────────→ TRIG (all 3 sensors)
GPIO 17 ──────────────────→ ECHO (front, via voltage divider)
GPIO 22 ──────────────────→ ECHO (left, via voltage divider)
GPIO 7  ──────────────────→ ECHO (right, via voltage divider)
Pi 5V ────────────────────→ VCC (all)
Common GND ────────────────→ GND (all)
```

### Assembly Steps

1. **Mount components on chassis**
   - Secure Raspberry Pi to robot body
   - Attach motors to wheels
   - Place ultrasonic sensors at strategic locations (front, left, right)

2. **Power distribution**
   - Connect motor battery (4xAA pack) to VM/GND on both TB6612FNG drivers
   - Connect Pi 3.3V to VCC on both drivers (logic power)
   - Connect Pi power bank for stable 5V/3A

3. **Sensor connections**
   - Solder or crimp all connections for reliability
   - Use color-coded wires for debugging
   - Add labels to GPIO pins

4. **Cable management**
   - Use zip ties or velcro straps
   - Keep sensor cables away from motors (interference)
   - Protect USB cables from damage

## Phase 2: Raspberry Pi OS Setup

### 1. Flash Raspberry Pi OS

```bash
# Download Raspberry Pi Imager from:
# https://www.raspberrypi.com/software/

# 1. Open Raspberry Pi Imager
# 2. Select "Raspberry Pi OS (64-bit)" - Full version recommended
# 3. Choose your SD card
# 4. Click "Write"
# 5. Wait for completion and eject
```

### 2. First Boot & Configuration

```bash
# Insert SD card, connect keyboard/mouse, plug in power

# On first boot, run:
sudo raspi-config

# In raspi-config menu:
# - Interface Options → I2C → Enable
# - Interface Options → Serial → Enable
# - Localization → Set timezone
# - Performance Options → GPU Memory → Set to 128MB (conserve for headless)
# - Finish and reboot
```

### 3. Update System

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y python3-pip python3-dev git
```

### 4. Install Required Libraries

```bash
# GPIO and Raspberry Pi specific
sudo apt install -y python3-rpi.gpio python3-board

# I2C Tools (for debugging)
sudo apt install -y i2c-tools

# System dependencies
sudo apt install -y libopenjp2-7 libtiff5 libjasper1 libharffp0d

# Install Python packages
pip3 install --upgrade pip
pip3 install RPi.GPIO numpy scipy opencv-python PyYAML
pip3 install adafruit-blinka adafruit-circuitpython-mpu6050
pip3 install flask flask-cors matplotlib
pip3 install paho-mqtt aiohttp
```

### 5. Enable SSH for Remote Access

```bash
# Enable SSH
sudo systemctl enable ssh
sudo systemctl start ssh

# Find Pi's IP address
hostname -I

# From your computer:
ssh pi@<ip_address>
# Default password: raspberry
# Change it: passwd
```

## Phase 3: Project Setup

### 1. Clone or Download Project

```bash
# If using git
git clone <your-repo-url>
cd "Project"

# Or extract downloaded zip
unzip project.zip
cd Project
```

### 2. Install Python Dependencies

```bash
pip3 install -r requirements.txt
```

### 3. Verify Hardware Connections

```bash
# Check I2C devices (IMU should appear as 0x68, once wired up)
i2cdetect -y 1

# Test GPIO access
python3 -c "import RPi.GPIO as GPIO; print('GPIO OK')"

# Verify motor wiring in isolation before anything else
python3 scripts/test_motor_wiring.py

# Verify ultrasonic wiring
python3 scripts/test_ultrasonic.py
```

## Phase 4: Calibration

### Motor Calibration

```bash
# Calibrate motor speeds to ensure straight movement
python3 scripts/calibrate_motors.py

# Adjust motor speeds in config/robot_config.yaml:
# - If robot drifts left, decrease left motor speed
# - If robot drifts right, decrease right motor speed
```

### Sensor Testing

```bash
# Test all sensors
python3 scripts/test_sensors.py

# This will:
# - Read motor controller status
# - Show IMU readings
# - Print ultrasonic distances
```

### IMU Calibration

```bash
# Calibrate IMU (keep robot stationary)
# Automatically done on robot startup, but can be manual:
python3 -c "
from hardware.imu_driver import IMUDriver
imu = IMUDriver()
imu.calibrate_gyroscope()
print('Calibration complete')
"
```

## Phase 5: Running the Robot

### Basic Test

```bash
# Single robot exploration
python3 main.py

# This will:
# 1. Initialize all components
# 2. Calibrate IMU
# 3. Begin autonomous exploration
# 4. Save map to maps/map_<timestamp>.json
```

### With Visualization Server

```bash
# In one terminal:
python3 main.py

# In another terminal:
python3 app/web_server.py

# Open browser: http://localhost:5000
# View real-time map and robot position
```

### Remote Operation (SSH)

```bash
# SSH into Raspberry Pi
ssh pi@<ip_address>

# Run robot
cd ~/Project
python3 main.py

# View map on your computer
# Open browser: http://<pi_ip>:5000
```

## Troubleshooting

### Motor Issues

**Problem**: Motors don't move
- Check motor battery connections (VM/GND on both TB6612FNG drivers)
- Confirm STBY (GPIO21) is driven HIGH - LOW holds both drivers in standby
- Verify GPIO pin assignments against "Wiring details.docx"
- Run `scripts/test_motor_wiring.py` to isolate a single motor/driver

**Problem**: Robot spins instead of moving forward
- Adjust motor speed calibration
- Check wheel attachment

**Problem**: Motors stall or the far driver browns out when all 4 start together
- This is a known power-sag issue with the shared AA battery pack /
  breadboard supply - `MotorController` and the `scripts/*.py` drive
  scripts already ramp duty cycle up gradually to spread out current draw

### IMU Issues

**Problem**: IMU not detected
```bash
i2cdetect -y 1  # Should show 68 if using default address
```

### GPIO Permission Issues

```bash
# Add user to gpio group
sudo usermod -a -G gpio pi
# Logout and login
```

### Performance Issues

If Raspberry Pi performance is inadequate:
- Decrease map resolution
- Disable web visualization
- Use 64-bit OS for better performance

## Next Steps (Phase 2)

Once Phase 1 is working:

1. **Set up second robot** with identical hardware
2. **Configure network communication**
   - WiFi direct or router-based
   - MQTT or custom protocol
3. **Implement map merging**
   - Transform robot2's map to robot1's coordinate system
   - Merge occupancy grids
4. **Collaborative exploration**
   - Share frontier information
   - Coordinate path planning

## Reference Documentation

- [Raspberry Pi GPIO Documentation](https://www.raspberrypi.com/documentation/computers/os.html)
- [MPU6050 Datasheet](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu6050/)
- [TB6612FNG Motor Driver Guide](https://www.sparkfun.com/tutorials/tb6612fng-hookup-guide)
- [HC-SR04 Ultrasonic Sensor](https://tutorials.raspberrypi.com/raspberry-pi-ultrasonic-distance-sensor/)
