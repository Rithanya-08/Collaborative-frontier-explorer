"""
Ultrasonic Sensor Driver Module

3x HC-SR04 ultrasonic distance sensors (front, left, right) sharing a
single TRIG line, for obstacle detection. See "Wiring details.docx" for
the physical connections.

Pin Configuration (BCM numbering):
    TRIG (shared by all 3 sensors): GPIO27
    Front ECHO: GPIO17
    Left  ECHO: GPIO22
    Right ECHO: GPIO7

Each ECHO line needs a voltage divider (5V -> ~3.3V) before reaching the
Pi's GPIO. TRIG does not.
"""

import RPi.GPIO as GPIO
import time
import logging
from typing import Optional, List
import threading

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

TRIG_PIN = 27
ECHO_TIMEOUT_S = 0.05

# Default sensor layout: id -> echo pin (all sensors share TRIG_PIN)
DEFAULT_SENSORS = {
    "front": 17,
    "left": 22,
    "right": 7,
}


class UltrasonicSensor:
    """
    HC-SR04 ultrasonic distance sensor driver.

    Features:
    - Distance measurement from 2cm to 400cm
    - Shares one TRIG pin across all sensors on the robot (the pin is
      configured only once, the first time it's used)
    - Moving-average filtering for noise reduction
    """

    _configured_trig_pins = set()

    def __init__(
        self,
        echo_pin: int,
        trig_pin: int = TRIG_PIN,
        max_distance: float = 4.0,  # meters
        sensor_id: str = "front"
    ):
        """
        Initialize ultrasonic sensor.

        Args:
            echo_pin: GPIO pin for echo (needs a voltage divider to 3.3V)
            trig_pin: GPIO pin for trigger, shared across all sensors
            max_distance: Maximum measurable distance in meters
            sensor_id: Identifier for this sensor (e.g., "front", "left", "right")
        """
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        self.max_distance = max_distance
        self.sensor_id = sensor_id

        self.last_distance = None
        self.distance_history = []
        self.history_size = 5

        self._setup_gpio()

    def _setup_gpio(self):
        """Setup GPIO pins."""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        if self.trig_pin not in UltrasonicSensor._configured_trig_pins:
            GPIO.setup(self.trig_pin, GPIO.OUT)
            GPIO.output(self.trig_pin, GPIO.LOW)
            UltrasonicSensor._configured_trig_pins.add(self.trig_pin)

        GPIO.setup(self.echo_pin, GPIO.IN)
        time.sleep(0.1)  # stabilize sensor

        logger.info(f"Ultrasonic sensor '{self.sensor_id}' initialized (echo=GPIO{self.echo_pin})")

    def measure_distance(self) -> Optional[float]:
        """
        Measure distance in meters.

        Returns:
            Distance in meters, or None if measurement failed
        """
        try:
            # Send trigger pulse (10 microseconds) - this fires every
            # sensor sharing this TRIG pin, so only this sensor's echo
            # line is read back
            GPIO.output(self.trig_pin, GPIO.HIGH)
            time.sleep(0.00001)
            GPIO.output(self.trig_pin, GPIO.LOW)

            timeout = time.time() + ECHO_TIMEOUT_S
            pulse_start = None
            while GPIO.input(self.echo_pin) == GPIO.LOW:
                pulse_start = time.time()
                if pulse_start > timeout:
                    return None
            if pulse_start is None:
                # ECHO was already HIGH when polling started (e.g.
                # cross-talk from the shared TRIG line still pending on
                # this sensor) - no rising edge seen this cycle
                return None

            pulse_end = pulse_start
            timeout = time.time() + ECHO_TIMEOUT_S
            while GPIO.input(self.echo_pin) == GPIO.HIGH:
                pulse_end = time.time()
                if pulse_end > timeout:
                    return None

            # Calculate distance (speed of sound ~343 m/s)
            pulse_duration = pulse_end - pulse_start
            distance = (pulse_duration * 343) / 2

            if distance > self.max_distance:
                return self.max_distance

            # Apply moving average filter
            self.distance_history.append(distance)
            if len(self.distance_history) > self.history_size:
                self.distance_history.pop(0)

            self.last_distance = sum(self.distance_history) / len(self.distance_history)
            return self.last_distance

        except Exception as e:
            logger.error(f"Error measuring distance on {self.sensor_id}: {e}")
            return None

    def get_last_distance(self) -> Optional[float]:
        """Get last measured distance without taking new measurement."""
        return self.last_distance

    def is_obstacle_detected(self, threshold: float = 0.30) -> bool:
        """
        Check if obstacle is detected within threshold.

        Args:
            threshold: Distance threshold in meters

        Returns:
            True if obstacle detected within threshold
        """
        distance = self.measure_distance()
        return distance is not None and distance < threshold

    def cleanup(self):
        """Clean up GPIO."""
        GPIO.cleanup()
        logger.info(f"Sensor '{self.sensor_id}' cleaned up")


class MultiUltrasonicArray:
    """
    Manage the robot's 3 ultrasonic sensors (front, left, right), which
    share a single TRIG line.
    """

    def __init__(self, sensor_configs: Optional[List[dict]] = None):
        """
        Initialize multiple ultrasonic sensors.

        Args:
            sensor_configs: List of dicts with keys:
                           {'id': 'front', 'echo_pin': 17, 'trig_pin': 27}
                           trig_pin defaults to the shared TRIG_PIN if omitted.
        """
        self.sensors = {}

        if sensor_configs is None:
            sensor_configs = [
                {"id": sensor_id, "echo_pin": echo_pin}
                for sensor_id, echo_pin in DEFAULT_SENSORS.items()
            ]

        for config in sensor_configs:
            self.sensors[config["id"]] = UltrasonicSensor(
                echo_pin=config["echo_pin"],
                trig_pin=config.get("trig_pin", TRIG_PIN),
                sensor_id=config["id"]
            )

        logger.info(f"Initialized {len(self.sensors)} ultrasonic sensors")

    def measure_all(self) -> dict:
        """
        Measure distance from all sensors, with a small gap between
        readings so adjacent sensors don't cross-talk on the shared TRIG.
        """
        measurements = {}
        sensor_items = list(self.sensors.items())
        for i, (sensor_id, sensor) in enumerate(sensor_items):
            measurements[sensor_id] = sensor.measure_distance()
            if i < len(sensor_items) - 1:
                time.sleep(0.05)
        return measurements

    def get_obstacle_distances(self) -> dict:
        """Get obstacle distances from all sensors."""
        return self.measure_all()

    def detect_obstacles(self, threshold: float = 0.30) -> dict:
        """
        Detect obstacles in all directions.

        Args:
            threshold: Detection distance threshold in meters

        Returns:
            Dict with sensor_id: bool pairs
        """
        distances = self.measure_all()
        return {
            sensor_id: (distance is not None and distance < threshold)
            for sensor_id, distance in distances.items()
        }

    def cleanup(self):
        """Clean up all sensors."""
        for sensor in self.sensors.values():
            sensor.cleanup()


def main():
    """Test ultrasonic sensor array."""
    array = MultiUltrasonicArray()

    try:
        print("Testing ultrasonic array...")
        for i in range(10):
            distances = array.measure_all()
            parts = [
                f"{sensor_id}: {d:.2f}m" if d is not None else f"{sensor_id}: no echo"
                for sensor_id, d in distances.items()
            ]
            print(f"Measurement {i+1}: " + " | ".join(parts))
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("Test interrupted")

    finally:
        array.cleanup()


if __name__ == "__main__":
    main()
