#!/usr/bin/env python3
"""
Turn Test Script

Tests the pivot-turn logic from obstacle_avoidance.py in isolation,
without needing an actual obstacle present. Pivots left, pauses, then
pivots right.
"""

import time
import RPi.GPIO as GPIO

STBY_PIN = 21
PWM_FREQ_HZ = 1000
TURN_SPEED = 100
TURN_DURATION = 0.5
TURN_RAMP_STEP = 20
TURN_RAMP_DELAY = 0.03
PAUSE_BETWEEN = 1.0

MOTORS = [
    {"name": "Front-Left",  "pwm": 12, "in1": 5,  "in2": 6,  "invert": False},
    {"name": "Rear-Left",   "pwm": 13, "in1": 16, "in2": 20, "invert": False},
    {"name": "Front-Right", "pwm": 18, "in1": 23, "in2": 24, "invert": True},
    {"name": "Rear-Right",  "pwm": 19, "in1": 25, "in2": 26, "invert": True},
]
LEFT_MOTORS = ["Front-Left", "Rear-Left"]
RIGHT_MOTORS = ["Front-Right", "Rear-Right"]

pwm_objects = {}


def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    GPIO.setup(STBY_PIN, GPIO.OUT)
    GPIO.output(STBY_PIN, GPIO.HIGH)

    for motor in MOTORS:
        GPIO.setup(motor["pwm"], GPIO.OUT)
        GPIO.setup(motor["in1"], GPIO.OUT)
        GPIO.setup(motor["in2"], GPIO.OUT)
        pwm = GPIO.PWM(motor["pwm"], PWM_FREQ_HZ)
        pwm.start(0)
        pwm_objects[motor["name"]] = pwm


def set_motor(motor, direction, duty):
    if direction == "forward":
        high, low = (motor["in2"], motor["in1"]) if motor["invert"] else (motor["in1"], motor["in2"])
    elif direction == "backward":
        high, low = (motor["in1"], motor["in2"]) if motor["invert"] else (motor["in2"], motor["in1"])
    else:
        GPIO.output(motor["in1"], GPIO.LOW)
        GPIO.output(motor["in2"], GPIO.LOW)
        pwm_objects[motor["name"]].ChangeDutyCycle(0)
        return

    GPIO.output(high, GPIO.HIGH)
    GPIO.output(low, GPIO.LOW)
    pwm_objects[motor["name"]].ChangeDutyCycle(duty)


def stop_all():
    for motor in MOTORS:
        set_motor(motor, "stop", 0)


def pivot(direction, speed, duration):
    """direction: 'left' or 'right'."""
    left_dir = "backward" if direction == "left" else "forward"
    right_dir = "forward" if direction == "left" else "backward"

    for motor in MOTORS:
        motor_dir = left_dir if motor["name"] in LEFT_MOTORS else right_dir
        set_motor(motor, motor_dir, 0)

    for duty in range(0, int(speed) + 1, TURN_RAMP_STEP):
        for motor in MOTORS:
            motor_dir = left_dir if motor["name"] in LEFT_MOTORS else right_dir
            set_motor(motor, motor_dir, duty)
        time.sleep(TURN_RAMP_DELAY)

    time.sleep(duration)
    stop_all()


def cleanup():
    stop_all()
    GPIO.output(STBY_PIN, GPIO.LOW)
    # Pop (not iterate-then-clear) so no loop variable keeps the last PWM
    # object referenced past this loop - a lingering reference here would
    # only get garbage collected after GPIO.cleanup() runs below, and
    # PWM.__del__ would then try to stop() an already-closed GPIO handle.
    for name in list(pwm_objects.keys()):
        pwm_objects.pop(name).stop()
    GPIO.cleanup()


def main():
    print("Turn test starting. Press Ctrl+C to stop.")
    setup()
    try:
        print("Pivoting LEFT...")
        pivot("left", TURN_SPEED, TURN_DURATION)
        time.sleep(PAUSE_BETWEEN)

        print("Pivoting RIGHT...")
        pivot("right", TURN_SPEED, TURN_DURATION)

        print("Done.")
    except KeyboardInterrupt:
        print("\nInterrupted by user. Stopping.")
    finally:
        cleanup()
        print("GPIO cleaned up. Exiting.")


if __name__ == "__main__":
    main()
