"""Hardware drivers package"""

# Robot A's drivers need RPi.GPIO and only import on the Pi; Robot B's
# (esp32_robot) run on the laptop, so a missing Pi library must not stop
# `import hardware.esp32_robot` there.
try:
    from .motor_controller import MotorController
    from .imu_driver import IMUDriver
    from .ultrasonic_driver import UltrasonicSensor, MultiUltrasonicArray
except ImportError:
    pass

__all__ = [
    'MotorController',
    'IMUDriver',
    'UltrasonicSensor',
    'MultiUltrasonicArray',
]
