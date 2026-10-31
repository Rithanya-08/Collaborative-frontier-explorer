#!/usr/bin/env python3
"""
Motor + power test for Robot A (Raspberry Pi 5)

Drives the robot with the same MotorController and speed (150/255) that
main.py uses, while logging the Pi's 5 V supply, and measures with the gyro
how far each pivot really turned. Shows whether the motors have enough
power to move the robot and whether driving pulls the Pi's supply down.

Moves: forward 0.5 s and back 0.5 s (skipped if the front sensor reads
under 25 cm), then pivot left 1 s and pivot right 1 s.

Usage (on the Pi, from ~/robot):
    python3 scripts/motor_power_test.py
"""

import os
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from hardware.imu_driver import IMUDriver
from hardware.motor_controller import MotorController
from hardware.ultrasonic_driver import MultiUltrasonicArray

SPEED = 150                   # same as main.py
EXPECTED_TURN_DEG = 30.0      # main.py's calibrated TURN_RATE_DEG_PER_S x 1 s
MIN_FRONT_CLEARANCE_M = 0.25


def read_5v():
    out = subprocess.run(["vcgencmd", "pmic_read_adc", "EXT5V_V"], capture_output=True, text=True).stdout
    try:
        return float(out.split("=")[-1].rstrip("V\n"))
    except ValueError:
        return None


def throttled():
    out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True, text=True).stdout
    return int(out.strip().split("=")[-1], 16)


class SupplyLogger(threading.Thread):
    """Samples the 5 V supply in the background; lowest reading per phase."""

    def __init__(self):
        super().__init__(daemon=True)
        self.phase, self.lowest, self.running = "idle", {}, True

    def run(self):
        while self.running:
            v = read_5v()
            if v is not None:
                self.lowest[self.phase] = min(v, self.lowest.get(self.phase, 99))
            time.sleep(0.05)


def heading_change(imu, before):
    time.sleep(0.4)  # let the robot settle
    imu.update_heading()
    return (imu.get_heading() - before + 180) % 360 - 180


def main():
    motors = MotorController()
    imu = IMUDriver()
    sonar = MultiUltrasonicArray()
    supply = SupplyLogger()
    results = []
    try:
        print("Calibrating gyro - keep the robot still...")
        imu.calibrate_gyroscope()
        imu.start_tracking()
        front = sonar.get_obstacle_distances().get("front")
        print(f"Front clearance: {front:.2f} m" if front is not None else "Front clearance: no echo")
        flags_before = throttled()
        supply.start()
        time.sleep(1.0)  # idle baseline

        if front is None or front >= MIN_FRONT_CLEARANCE_M:
            supply.phase = "forward"
            print("Forward 0.5 s...")
            motors.move_forward(SPEED, duration=0.5)
            time.sleep(0.5)
            after_fwd = sonar.get_obstacle_distances().get("front")
            supply.phase = "backward"
            print("Backward 0.5 s...")
            motors.move_backward(SPEED, duration=0.5)
            if front is not None and after_fwd is not None:
                results.append(f"Forward moved about {100 * (front - after_fwd):.0f} cm (front sensor "
                               f"{front:.2f} -> {after_fwd:.2f} m)")
        else:
            results.append("Forward/backward skipped - something is closer than 25 cm in front")

        for direction in ("left", "right"):
            supply.phase = "idle"
            time.sleep(0.5)
            imu.update_heading()
            before = imu.get_heading()
            supply.phase = f"pivot {direction}"
            print(f"Pivot {direction} 1 s...")
            (motors.turn_left if direction == "left" else motors.turn_right)(SPEED, duration=1.0)
            supply.phase = "idle"
            turned = heading_change(imu, before)
            results.append(f"Pivot {direction}: gyro saw {turned:+.0f}° (calibrated rate expects about "
                           f"{'+' if direction == 'left' else '-'}{EXPECTED_TURN_DEG:.0f}°)")

        time.sleep(0.5)
        supply.running = False
        new_flags = throttled() & ~flags_before
    finally:
        motors.stop()
        motors.cleanup()
        imu.cleanup()
        sonar.cleanup()

    print("\n=== Results ===")
    for line in results:
        print(line)
    print("\nLowest 5 V supply reading per phase:")
    for phase, v in supply.lowest.items():
        warn = "  <- below 4.75 V, too low" if v < 4.75 else ""
        print(f"  {phase:12s} {v:.2f} V{warn}")
    flags_now = throttled()
    print(f"\nUnder-voltage right now: {'YES' if flags_now & 0x1 else 'no'}; "
          f"new under-voltage during test: {'YES' if new_flags & 0x10000 else 'no'} "
          f"(get_throttled=0x{flags_now:x})")


if __name__ == "__main__":
    main()
