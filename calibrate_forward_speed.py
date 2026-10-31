#!/usr/bin/env python3
"""
Forward Speed Calibration

Drives forward for exactly ONE burst - same speed (150/255) and same
duration (FORWARD_STEP_S) that main.py's _move_towards() actually uses
per step, via the same MotorController class (same ramping, everything).

This deliberately does NOT drive for several seconds and extrapolate a
"meters per second" rate: the motors ramp up gradually (~0.55s to reach
full duty at this speed), so a short 0.3s burst never reaches cruising
speed at all - a rate measured from a long, mostly-at-cruising-speed
drive would overestimate what one real burst covers. Measuring the
actual burst directly avoids that assumption entirely.

Usage (on the Pi, from the project root):
    1. Place the robot on the floor with a few meters of clear space
       ahead. Mark its starting position (e.g. a piece of tape at the
       front wheels, or note the front wheel position against the floor).
    2. Run: python3 calibrate_forward_speed.py
    3. Measure the distance from the start mark to where the front
       wheels ended up, in meters (or centimeters is fine too).
    4. Report that distance back - it's used directly as
       FORWARD_STEP_DISTANCE_M in main.py, no scaling involved.
"""

from hardware.motor_controller import MotorController

SPEED = 150         # matches main.py's _move_towards()/_avoid_obstacle() speed
DURATION_S = 0.3    # matches main.py's FORWARD_STEP_S exactly
NUM_BURSTS = 5      # repeat and use the average - a single 0.3s burst is
                     # short enough that measurement error matters a lot


def main():
    print(f"Will drive forward at speed={SPEED} for {DURATION_S}s, {NUM_BURSTS} separate times")
    print("(with a pause between each so you can reset/re-mark position).")
    input("Press ENTER to start burst 1...")

    controller = MotorController()
    try:
        for i in range(1, NUM_BURSTS + 1):
            print(f"\n--- Burst {i}/{NUM_BURSTS} ---")
            controller.move_forward(SPEED, duration=DURATION_S)
            print("Measure the distance this single burst covered, then reset the robot")
            print("to the start mark before continuing.")
            if i < NUM_BURSTS:
                input("Press ENTER for the next burst...")
    finally:
        controller.cleanup()

    print("\nDone. Report all 5 measured distances (or their average) back.")


if __name__ == "__main__":
    main()
