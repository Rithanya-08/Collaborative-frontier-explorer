#!/usr/bin/env python3
"""
Ultrasonic Sensor Test Script

Verifies wiring for 3x HC-SR04 sensors (front, left, right) with a
shared TRIG line before integrating with navigation/collision_avoidance.py.

Pin assignments (avoid every GPIO already used by the motor drivers -
see scripts/test_motor_wiring.py - and the I2C bus used by the IMU):
    TRIG (shared by all 3 sensors): GPIO27
    Front ECHO: GPIO17
    Left  ECHO: GPIO22
    Right ECHO: GPIO7

Remember: each ECHO line needs a voltage divider (5V -> ~3.3V) before
reaching the Pi's GPIO. TRIG does not.
"""

import time
import RPi.GPIO as GPIO

TRIG_PIN = 27
SENSORS = [
    {"name": "front", "echo": 17},
    {"name": "left",  "echo": 22},
    {"name": "right", "echo": 7},
]

MAX_DISTANCE_M = 4.0
ECHO_TIMEOUT_S = 0.05
READ_INTERVAL = 0.5


def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    GPIO.setup(TRIG_PIN, GPIO.OUT)
    GPIO.output(TRIG_PIN, GPIO.LOW)

    for sensor in SENSORS:
        GPIO.setup(sensor["echo"], GPIO.IN)

    time.sleep(0.5)  # let sensors settle


def measure(echo_pin):
    GPIO.output(TRIG_PIN, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, GPIO.LOW)

    timeout = time.time() + ECHO_TIMEOUT_S
    pulse_start = None
    while GPIO.input(echo_pin) == GPIO.LOW:
        pulse_start = time.time()
        if pulse_start > timeout:
            return None
    if pulse_start is None:
        # ECHO was already HIGH when polling started (e.g. cross-talk from
        # the shared TRIG line still pending on this sensor) - no rising
        # edge seen this cycle, so there's nothing valid to measure
        return None

    pulse_end = pulse_start
    timeout = time.time() + ECHO_TIMEOUT_S
    while GPIO.input(echo_pin) == GPIO.HIGH:
        pulse_end = time.time()
        if pulse_end > timeout:
            return None

    distance = ((pulse_end - pulse_start) * 343) / 2  # speed of sound m/s
    return min(distance, MAX_DISTANCE_M)


def main():
    print("Ultrasonic wiring test starting. Press Ctrl+C to stop.")
    setup()
    try:
        while True:
            parts = []
            for sensor in SENSORS:
                d = measure(sensor["echo"])
                parts.append(f"{sensor['name']}: {d:.2f}m" if d is not None else f"{sensor['name']}: no echo")
                time.sleep(0.05)  # small gap so sensors don't cross-talk
            print(" | ".join(parts))
            time.sleep(READ_INTERVAL)
    except KeyboardInterrupt:
        print("\nInterrupted by user. Stopping.")
    finally:
        GPIO.cleanup()
        print("GPIO cleaned up. Exiting.")


if __name__ == "__main__":
    main()
