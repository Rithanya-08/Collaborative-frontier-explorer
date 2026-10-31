# Hardware Setup — Multi-Robot Exploration (per robot, 2 robots identical)

## Hardware
- ESP32 DevKit (38-pin, ESP32-WROOM-32), Arduino C++ via PlatformIO in VS Code
- ESP32-CAM (AI Thinker, OV2640) — video stream over WiFi only, no wires to the DevKit
- 4x DC gear motors (TT motors), one per wheel (front-left, rear-left, front-right, rear-right)
- 2x TB6612FNG motor driver boards
  - Driver 1 controls LEFT side: front-left motor on Channel A, rear-left motor on Channel B
  - Driver 2 controls RIGHT side: front-right motor on Channel A, rear-right motor on Channel B
- MPU6050 IMU (I2C) for heading
- 3x HC-SR04 ultrasonic sensors (front, left, right), shared TRIG line
- No wheel encoders — distance is estimated from calibrated speed x time, heading from the IMU

## Power
- Motor power: 2x Li-ion 3.7V in series (7.4V nominal, 8.4V full) → switch → VM on both drivers
- Logic power: USB power bank → ESP32 DevKit (USB port)
  - ESP32 3.3V → VCC on both drivers, STBY on both drivers, MPU6050 VCC
  - ESP32 VIN/5V → HC-SR04 VCC (x3)
- ESP32-CAM: USB power bank 5V → CAM 5V pin
- Common ground: battery GND, both drivers' GND, and ESP32 GND all tied together
- Battery + goes ONLY to VM. It never touches any ESP32 pin.

## Motor wiring
```
Front-Left(+)   ──→  AO1  (Driver 1)
Front-Left(−)   ──→  AO2  (Driver 1)

Rear-Left(+)    ──→  BO1  (Driver 1)
Rear-Left(−)    ──→  BO2  (Driver 1)

Front-Right(+)  ──→  AO1  (Driver 2)
Front-Right(−)  ──→  AO2  (Driver 2)

Rear-Right(+)   ──→  BO1  (Driver 2)
Rear-Right(−)   ──→  BO2  (Driver 2)

Battery (+)     ──→  VM   (both drivers, via switch)
ESP32 3.3V      ──→  VCC  (both drivers)
ESP32 3.3V      ──→  STBY (both drivers, tied HIGH)
```

## Motor driver GPIO assignments
Direction pins are shared per side: one GPIO wire goes to BOTH AIN1 and BIN1, another to BOTH AIN2 and BIN2.
Each wheel keeps its own PWM pin.

| Signal | Driver 1 (left)  | Driver 2 (right) |
|--------|------------------|------------------|
| STBY   | 3.3V (tied HIGH) | 3.3V (tied HIGH) |
| PWMA   | GPIO25           | GPIO27           |
| AIN1   | GPIO16 (shared)  | GPIO18 (shared)  |
| AIN2   | GPIO17 (shared)  | GPIO19 (shared)  |
| PWMB   | GPIO26           | GPIO13           |
| BIN1   | GPIO16 (shared)  | GPIO18 (shared)  |
| BIN2   | GPIO17 (shared)  | GPIO19 (shared)  |

- Channel A on each driver = front motor (left or right side)
- Channel B on each driver = rear motor (left or right side)
- Front and rear motors on the same side must spin the same direction for the same IN signals.
  If one is reversed, swap that motor's two wires at the driver output.

## MPU6050 IMU
| MPU6050 | Connects to                  |
|---------|------------------------------|
| VCC     | ESP32 3.3V                   |
| GND     | Common GND                   |
| SCL     | GPIO22                       |
| SDA     | GPIO21                       |
| AD0     | GND (I2C address 0x68)       |
| INT     | GPIO33 (optional, data-ready)|

## Ultrasonic sensors (3x HC-SR04, shared TRIG line)
| Sensor | TRIG            | ECHO                         | VCC        | GND        |
|--------|-----------------|------------------------------|------------|------------|
| Front  | GPIO23 (shared) | GPIO34 (via voltage divider) | ESP32 VIN/5V | Common GND |
| Left   | GPIO23 (shared) | GPIO35 (via voltage divider) | ESP32 VIN/5V | Common GND |
| Right  | GPIO23 (shared) | GPIO32 (via voltage divider) | ESP32 VIN/5V | Common GND |

Voltage divider (one per ECHO): ECHO → 1 kΩ → ESP32 pin → 2 kΩ → GND (5V becomes about 3.33V).
Measure each divider output before connecting it: it must be 3.4V or less.

## Other pins
| Function   | GPIO              |
|------------|-------------------|
| Status LED | GPIO2 (onboard)   |
| Spare      | GPIO4, 5, 14, 36, 39 |

Do not use: GPIO0, 12, 15 (strapping), GPIO1, 3 (USB serial), GPIO6–11 (flash).

## ESP32-CAM
| ESP32-CAM | Connects to          |
|-----------|----------------------|
| 5V        | Power bank 5V        |
| GND       | Power bank GND       |
| Signals   | None — WiFi MJPEG stream to laptop only |

## Network
- 2.4 GHz WiFi, laptop and both robots on the same network
- Mosquitto MQTT broker on the laptop
- Robots publish pose, range, status; laptop publishes targets and stop commands

## Start poses
| Robot | Start (x, y) | Heading |
|-------|--------------|---------|
| A     | (0, 0) cm    | +x      |
| B     | (0, -50) cm  | +x      |

Side by side: B starts 50 cm to the right of A, both facing the same way (+x).
The shared map depends on this - mark both start spots on the floor.

## Safety checklist (can damage hardware)
- Battery + must never touch any ESP32 pin. Before first battery power-on, check with a multimeter
  (continuity mode) between battery + and ESP32 3.3V / VIN / GND: none should beep.
- Never connect STBY or VCC of the TB6612 to VM (VM and VCC are neighbouring pins on most boards).
- Check battery polarity before connecting VM — the TB6612 has no reverse-polarity protection.
- Every HC-SR04 ECHO must go through its voltage divider.
- When flashing, unplug the power bank and use only the laptop USB cable (one 5V source at a time).
- First battery power-up with the wheels off the ground.
- Firmware limits PWM duty to about 70% (TT motors are rated ~6V, battery is up to 8.4V).
