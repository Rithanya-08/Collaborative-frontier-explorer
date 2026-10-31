#!/usr/bin/env python3
"""
Motor Wiring Test Script

Verifies TB6612FNG wiring for a 4WD robot (2 drivers, 4 TT motors) before
sensor integration. Spins each wheel forward then backward for ~1s and
prints progress, so miswired motors/directions are obvious.

Pinout (BCM numbering):
    Driver 1 (LEFT):  STBY=21  PWMA=12 AIN1=5  AIN2=6   (front-left, Ch A)
                                PWMB=13 BIN1=16 BIN2=20  (rear-left,  Ch B)
    Driver 2 (RIGHT): STBY=21  PWMA=18 AIN1=23 AIN2=24  (front-right, Ch A)
                                PWMB=19 BIN1=25 BIN2=26  (rear-right,  Ch B)
"""

import time
import RPi.GPIO as GPIO

STBY_PIN = 21
PWM_FREQ_HZ = 1000
TEST_SPEED = 100         # duty cycle percent (0-100)
SPIN_DURATION = 1.0      # seconds per direction
PAUSE_BETWEEN = 0.5      # seconds between steps

MOTORS = [
    {"name": "Front-Left",  "pwm": 12, "in1": 5,  "in2": 6},
    {"name": "Rear-Left",   "pwm": 13, "in1": 16, "in2": 20},
    {"name": "Front-Right", "pwm": 18, "in1": 23, "in2": 24},
    {"name": "Rear-Right",  "pwm": 19, "in1": 25, "in2": 26},
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


def drive(motor, direction, speed=TEST_SPEED):
    """direction: 'forward', 'backward', or 'stop'."""
    pwm = pwm_objects[motor["name"]]

    if direction == "forward":
        GPIO.output(motor["in1"], GPIO.HIGH)
        GPIO.output(motor["in2"], GPIO.LOW)
        pwm.ChangeDutyCycle(speed)
    elif direction == "backward":
        GPIO.output(motor["in1"], GPIO.LOW)
        GPIO.output(motor["in2"], GPIO.HIGH)
        pwm.ChangeDutyCycle(speed)
    else:  # stop
        GPIO.output(motor["in1"], GPIO.LOW)
        GPIO.output(motor["in2"], GPIO.LOW)
        pwm.ChangeDutyCycle(0)


def stop_all():
    for motor in MOTORS:
        drive(motor, "stop")


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


def test_motor(motor):
    print(f"\n=== Testing {motor['name']} (PWM={motor['pwm']}, IN1={motor['in1']}, IN2={motor['in2']}) ===")

    print(f"  {motor['name']}: FORWARD")
    drive(motor, "forward")
    time.sleep(SPIN_DURATION)
    drive(motor, "stop")
    time.sleep(PAUSE_BETWEEN)

    print(f"  {motor['name']}: BACKWARD")
    drive(motor, "backward")
    time.sleep(SPIN_DURATION)
    drive(motor, "stop")
    time.sleep(PAUSE_BETWEEN)


def main():
    print("Motor wiring test starting. Press Ctrl+C at any time to stop safely.")
    setup()
    try:
        for motor in MOTORS:
            test_motor(motor)
        print("\nAll motors tested. Stopping.")
    except KeyboardInterrupt:
        print("\nInterrupted by user. Stopping all motors.")
    finally:
        cleanup()
        print("GPIO cleaned up. Exiting.")


if __name__ == "__main__":
    main()
