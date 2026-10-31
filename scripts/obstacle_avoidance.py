#!/usr/bin/env python3
"""
Obstacle Avoidance Script

Drives forward continuously using the confirmed motor wiring from
move_forward.py. When the front ultrasonic sensor detects an obstacle
within OBSTACLE_THRESHOLD_M, it stops, backs off briefly for turn
clearance, picks the side with more open space (using the left/right
sensors - no servo sweep needed since we have 3 fixed sensors), pivots
that direction using the calibrated turn from turn_90.py, then resumes
driving forward.
"""

import time
import signal
import sys
import RPi.GPIO as GPIO

# --- Motor setup (same wiring as move_forward.py) ---
STBY_PIN = 21
PWM_FREQ_HZ = 1000
DRIVE_SPEED = 40         # forward duty cycle percent (0-100) - Li-ion
                         # delivers much more current than the old AA pack,
                         # so less duty is needed to get the same result
BACKWARD_SPEED = 60      # reverse duty cycle percent when backing off
BACKWARD_DURATION = 0.15 # seconds to back away from the obstacle before turning
TURN_SPEED = 60          # matches turn_90.py's calibrated Li-ion value
TURN_DURATION = 1.0      # scaled down from the 1.5s/~90 degree calibration
                         # in turn_90.py (same TURN_SPEED, so duration
                         # scales roughly linearly with angle - ~60 degrees)
RAMP_STEP = 5
RAMP_DELAY = 0.05
TURN_RAMP_STEP = 20     # coarser/faster ramp for turning specifically,
TURN_RAMP_DELAY = 0.03  # so less time is spent at weak, sub-turning duty

MOTORS = [
    {"name": "Front-Left",  "pwm": 12, "in1": 5,  "in2": 6,  "invert": False},
    {"name": "Rear-Left",   "pwm": 13, "in1": 16, "in2": 20, "invert": False},
    {"name": "Front-Right", "pwm": 18, "in1": 23, "in2": 24, "invert": True},
    {"name": "Rear-Right",  "pwm": 19, "in1": 25, "in2": 26, "invert": True},
]
LEFT_MOTORS = ["Front-Left", "Rear-Left"]
RIGHT_MOTORS = ["Front-Right", "Rear-Right"]

pwm_objects = {}

# --- Ultrasonic setup (same wiring as test_ultrasonic.py) ---
TRIG_PIN = 27
SENSORS = [
    {"name": "front", "echo": 17},
    {"name": "left",  "echo": 22},
    {"name": "right", "echo": 7},
]
MAX_DISTANCE_M = 4.0
ECHO_TIMEOUT_S = 0.05
OBSTACLE_THRESHOLD_M = 0.30
STOP_PAUSE = 0.3  # seconds to sit fully stopped before turning

LOOP_DELAY = 0.1


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

    GPIO.setup(TRIG_PIN, GPIO.OUT)
    GPIO.output(TRIG_PIN, GPIO.LOW)
    for sensor in SENSORS:
        GPIO.setup(sensor["echo"], GPIO.IN)

    time.sleep(0.5)  # let ultrasonic sensors settle


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

    distance = ((pulse_end - pulse_start) * 343) / 2
    return min(distance, MAX_DISTANCE_M)


def read_all():
    readings = {}
    for sensor in SENSORS:
        readings[sensor["name"]] = measure(sensor["echo"])
        time.sleep(0.05)  # avoid cross-talk between sensors
    return readings


def set_motor(motor, direction, duty):
    """direction: 'forward', 'backward', or 'stop'."""
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


def drive_forward_ramped(speed):
    for motor in MOTORS:
        set_motor(motor, "forward", 0)
    for duty in range(0, int(speed) + 1, RAMP_STEP):
        for motor in MOTORS:
            set_motor(motor, "forward", duty)
        time.sleep(RAMP_DELAY)


def drive_backward_ramped(speed, duration):
    for motor in MOTORS:
        set_motor(motor, "backward", 0)
    for duty in range(0, int(speed) + 1, RAMP_STEP):
        for motor in MOTORS:
            set_motor(motor, "backward", duty)
        time.sleep(RAMP_DELAY)
    time.sleep(duration)
    stop_all()


def pivot(direction, speed, duration):
    """direction: 'left' or 'right'.

    Only one side drives - the other stays stopped. Driving the left
    wheels alone rotates the car right; driving the right wheels alone
    rotates it left (confirmed on the actual hardware). This also only
    draws current for 2 motors instead of 4, which helps with the
    power-sag stalling seen when both sides ran in opposite directions.
    """
    active_motors = LEFT_MOTORS if direction == "right" else RIGHT_MOTORS

    for motor in MOTORS:
        if motor["name"] in active_motors:
            set_motor(motor, "forward", 0)
        else:
            set_motor(motor, "stop", 0)

    # Ramp up, same reason as drive_forward_ramped: the shared
    # battery/breadboard supply sags too hard on a hard jump to full duty.
    # Uses a coarser/faster ramp than forward driving so more of the pivot
    # happens at full torque instead of at weak intermediate duty cycles.
    for duty in range(0, int(speed) + 1, TURN_RAMP_STEP):
        for motor in MOTORS:
            if motor["name"] in active_motors:
                set_motor(motor, "forward", duty)
        time.sleep(TURN_RAMP_DELAY)

    time.sleep(duration)
    stop_all()


_cleaned_up = False


def cleanup():
    global _cleaned_up
    if _cleaned_up:
        return
    _cleaned_up = True
    stop_all()
    GPIO.output(STBY_PIN, GPIO.LOW)
    # Pop (not iterate-then-clear) so no loop variable keeps the last PWM
    # object referenced past this loop - a lingering reference here would
    # only get garbage collected after GPIO.cleanup() runs below, and
    # PWM.__del__ would then try to stop() an already-closed GPIO handle.
    for name in list(pwm_objects.keys()):
        pwm_objects.pop(name).stop()
    GPIO.cleanup()


def handle_termination(signum, frame):
    """Catch SIGTERM/SIGHUP the same way Ctrl+C/SIGINT is caught below.

    A dropped SSH connection (e.g. plink losing its link) delivers one of
    these instead of a KeyboardInterrupt, and without this handler the
    process dies immediately with no chance to run cleanup() - leaving
    the motors spinning at whatever duty cycle they were last set to.
    """
    cleanup()
    sys.exit(0)


signal.signal(signal.SIGTERM, handle_termination)
signal.signal(signal.SIGHUP, handle_termination)


def main():
    print("Obstacle avoidance starting. Press Ctrl+C to stop.")
    setup()
    moving = False
    try:
        while True:
            distances = read_all()
            front, left, right = distances["front"], distances["left"], distances["right"]
            print(f"front: {front} | left: {left} | right: {right}")

            obstacle_ahead = front is not None and front < OBSTACLE_THRESHOLD_M

            if obstacle_ahead:
                print(f"Obstacle ahead ({front:.2f}m). Stopping.")
                stop_all()
                moving = False
                time.sleep(STOP_PAUSE)

                print("Backing off for turn clearance.")
                drive_backward_ramped(BACKWARD_SPEED, BACKWARD_DURATION)
                time.sleep(STOP_PAUSE)

                # Turn toward whichever side has more open space.
                # Treat "no echo" as open space (nothing in range to reflect).
                left_space = left if left is not None else MAX_DISTANCE_M
                right_space = right if right is not None else MAX_DISTANCE_M
                direction = "left" if left_space >= right_space else "right"
                print(f"Turning {direction}.")
                pivot(direction, TURN_SPEED, TURN_DURATION)

                print("Resuming forward.")
                drive_forward_ramped(DRIVE_SPEED)
                moving = True
            else:
                if not moving:
                    drive_forward_ramped(DRIVE_SPEED)
                    moving = True

            time.sleep(LOOP_DELAY)
    except KeyboardInterrupt:
        print("\nInterrupted by user. Stopping.")
    finally:
        cleanup()
        print("GPIO cleaned up. Exiting.")


if __name__ == "__main__":
    main()
