#!/usr/bin/env python3
"""
Turn Rate Calibration

Pivots left for 10 consecutive nudges - same speed (150/255) and same
duration (HEADING_TURN_NUDGE_S) that main.py's _move_towards() actually
uses per heading correction - via the same MotorController class.

Measuring 10 nudges' worth of rotation (instead of one) makes the result
much less sensitive to measurement error than trying to read a single
~10 degree turn directly.

Usage (on the Pi, from the project root):
    1. Mark the robot's starting heading - e.g. align one edge against a
       straight line on the floor, or note the direction it's pointing
       relative to something fixed.
    2. Run: python3 calibrate_turn_rate.py
    3. Measure the TOTAL angle it rotated through (degrees), comparing
       final heading to the marked starting heading.
    4. Report that total angle back - divided by 10, it becomes the real
       per-nudge rotation, which converts to TURN_RATE_DEG_PER_S for
       main.py (replacing the estimate carried over from turn_90.py).
"""

from hardware.motor_controller import MotorController
import time

SPEED = 150            # matches main.py's _move_towards() speed
NUDGE_DURATION_S = 0.15  # matches main.py's HEADING_TURN_NUDGE_S exactly
NUM_NUDGES = 10


def main():
    print(f"Will pivot LEFT {NUM_NUDGES} times, {NUDGE_DURATION_S}s each, at speed={SPEED}.")
    print("Mark the robot's current heading now if you haven't already.")
    input("Press ENTER to start...")

    controller = MotorController()
    try:
        for i in range(1, NUM_NUDGES + 1):
            controller.turn_left(SPEED, duration=NUDGE_DURATION_S)
            print(f"Nudge {i}/{NUM_NUDGES} done.")
            time.sleep(0.3)  # small pause so nudges are visually distinct, not required for the measurement
    finally:
        controller.cleanup()

    print(f"\nDone. Measure the TOTAL rotation from the original mark (degrees) and report it back.")


if __name__ == "__main__":
    main()
