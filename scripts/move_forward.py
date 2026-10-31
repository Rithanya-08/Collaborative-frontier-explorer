#!/usr/bin/env python3
"""
Move Forward Script

Drives all four wheels forward together using the confirmed wiring from
test_motor_wiring.py (2x TB6612FNG, 4 TT motors).
"""

import time
import RPi.GPIO as GPIO

STBY_PIN = 21
PWM_FREQ_HZ = 1000
SPEED = 80             # duty cycle percent (0-100)
DURATION = 5.0         # seconds to drive forward
RAMP_STEP = 5          # duty cycle increment per ramp step
RAMP_DELAY = 0.05      # seconds between ramp steps

MOTORS = [
    {"name": "Front-Left",  "pwm": 12, "in1": 5,  "in2": 6,  "invert": False},
    {"name": "Rear-Left",   "pwm": 13, "in1": 16, "in2": 20, "invert": False},
    {"name": "Front-Right", "pwm": 18, "in1": 23, "in2": 24, "invert": True},
    {"name": "Rear-Right",  "pwm": 19, "in1": 25, "in2": 26, "invert": True},
]

pwm_objects = {}


def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    GPIO.setup(STBY_PIN, GPIO.OUT)
    GPIO.output(STBY_PIN, GPIO.HIGH)  # take drivers out of standby

    for motor in MOTORS:
        GPIO.setup(motor["pwm"], GPIO.OUT)
        GPIO.setup(motor["in1"], GPIO.OUT)
        GPIO.setup(motor["in2"], GPIO.OUT)

        pwm = GPIO.PWM(motor["pwm"], PWM_FREQ_HZ)
        pwm.start(0)
        pwm_objects[motor["name"]] = pwm


def drive_forward(speed=SPEED):
    for motor in MOTORS:
        if motor["invert"]:
            GPIO.output(motor["in1"], GPIO.LOW)
            GPIO.output(motor["in2"], GPIO.HIGH)
        else:
            GPIO.output(motor["in1"], GPIO.HIGH)
            GPIO.output(motor["in2"], GPIO.LOW)

    # Ramp all motors up together instead of jumping straight to full duty
    # cycle. This spreads out the current draw so the shared battery/
    # breadboard supply doesn't sag hard enough to starve the far side
    # (right board) when all four motors demand stall current at once.
    for duty in range(0, int(speed) + 1, RAMP_STEP):
        for motor in MOTORS:
            pwm_objects[motor["name"]].ChangeDutyCycle(duty)
        time.sleep(RAMP_DELAY)

    for motor in MOTORS:
        pwm_objects[motor["name"]].ChangeDutyCycle(speed)


def stop_all():
    for motor in MOTORS:
        GPIO.output(motor["in1"], GPIO.LOW)
        GPIO.output(motor["in2"], GPIO.LOW)
        pwm_objects[motor["name"]].ChangeDutyCycle(0)


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
    print(f"Moving forward at {SPEED}% speed for {DURATION}s. Press Ctrl+C to stop early.")
    setup()
    try:
        drive_forward()
        time.sleep(DURATION)
        stop_all()
        print("Done.")
    except KeyboardInterrupt:
        print("\nInterrupted by user. Stopping.")
    finally:
        cleanup()
        print("GPIO cleaned up. Exiting.")


if __name__ == "__main__":
    main()
