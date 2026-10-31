"""
Robot B (ESP32) drivers over MQTT

Robot B's ESP32 is a "body" only (firmware/robot_b): it runs timed moves and
publishes its sensors, while mapping and exploration run here on the laptop.
These classes give main.py the same interface as Robot A's MotorController,
IMUDriver and MultiUltrasonicArray, so RobotController can drive either robot.

Topics (see firmware/robot_b/src/main.cpp):
    robot/B/cmd    <- {"id":7,"cmd":"fwd|back|left|right","speed":150,"ms":300}
                      {"id":8,"cmd":"stop"}  {"id":9,"cmd":"calib"}  {"id":10,"cmd":"yaw","yaw":90}
    robot/B/done   -> {"id":7,"result":"ok|obstacle|stopped|busy|bad_cmd"}
    robot/B/state  -> {"yaw":12.3,"front":0.84,"left":-1,"right":1.2,"busy":0}  every 100 ms
    robot/B/status -> "online" / "offline" (retained, last will)

Usage:
    robot = ESP32Robot(host="localhost")
    motors, imu, ultrasonic = robot.motors, robot.imu, robot.ultrasonic
"""

import itertools
import json
import logging
import threading
import time
from typing import Optional

import paho.mqtt.client as mqtt

logger = logging.getLogger(__name__)

# Must match the firmware's ramp constants: moves ramp up first, then hold for "ms"
FWD_RAMP_PCT, FWD_RAMP_MS = 5.0, 50
TURN_RAMP_PCT, TURN_RAMP_MS = 20.0, 30
MAX_DUTY = 178          # firmware clamps speed to ~70% duty
MAX_MOVE_MS = 5000      # longest single move the firmware accepts
DONE_MARGIN_S = 2.0     # extra wait for the "done" reply beyond the move itself
STATE_STALE_S = 1.0     # sensor readings older than this are treated as missing
US_NO_ECHO = -1


class ESP32Link:
    """MQTT connection to one ESP32 robot: sends commands, caches its state."""

    def __init__(self, host: str = "localhost", port: int = 1883, robot_id: str = "B",
                 connect_timeout: float = 10.0):
        self.robot_id = robot_id
        self.topic = lambda name: f"robot/{robot_id}/{name}"
        self._ids = itertools.count(1)
        self._lock = threading.Condition()
        self._done = {}           # command id -> result
        self.state = None         # latest robot/B/state payload (dict)
        self.state_time = 0.0     # time.monotonic() when it arrived
        self.state_seq = 0        # increments per state message
        self.online = False
        self._refs = 0

        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"laptop-{robot_id}")
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.connect(host, port, keepalive=10)
        self.client.loop_start()

        with self._lock:
            if not self._lock.wait_for(lambda: self.online and self.state, timeout=connect_timeout):
                self.client.loop_stop()
                raise RuntimeError(f"Robot {robot_id} not online on {host}:{port} "
                                   f"(check its power/WiFi and that mosquitto is running)")
        logger.info(f"Connected to robot {robot_id} via MQTT {host}:{port}")

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        client.subscribe([(self.topic("state"), 0), (self.topic("done"), 1), (self.topic("status"), 1)])

    def _on_message(self, client, userdata, msg):
        name = msg.topic.rsplit("/", 1)[-1]
        payload = msg.payload.decode(errors="replace")
        with self._lock:
            if name == "status":
                self.online = payload == "online"
                if not self.online:
                    logger.error(f"Robot {self.robot_id} went offline")
            elif name == "state":
                try:
                    self.state = json.loads(payload)
                except ValueError:
                    return
                self.state_time = time.monotonic()
                self.state_seq += 1
            elif name == "done":
                try:
                    done = json.loads(payload)
                except ValueError:
                    return
                self._done[done.get("id")] = done.get("result")
            self._lock.notify_all()

    def command(self, cmd: str, wait_s: Optional[float] = 1.0, **fields) -> Optional[str]:
        """Send a command; if wait_s is set, block until its "done" reply and return the result."""
        cmd_id = next(self._ids)
        self.client.publish(self.topic("cmd"), json.dumps({"id": cmd_id, "cmd": cmd, **fields}), qos=1)
        if wait_s is None:
            return None
        with self._lock:
            if not self._lock.wait_for(lambda: cmd_id in self._done, timeout=wait_s):
                logger.warning(f"No reply to '{cmd}' (id {cmd_id}) within {wait_s:.1f}s")
                return "timeout"
            result = self._done.pop(cmd_id)
        if result != "ok":
            logger.warning(f"Robot {self.robot_id} '{cmd}' -> {result}")
        return result

    def wait_for_state(self, after_seq: int, timeout: float) -> bool:
        """Block until a state message newer than after_seq arrives."""
        with self._lock:
            return self._lock.wait_for(lambda: self.state_seq > after_seq, timeout=timeout)

    def acquire(self):
        self._refs += 1
        return self

    def release(self):
        self._refs -= 1
        if self._refs <= 0:
            self.client.loop_stop()
            self.client.disconnect()


class ESP32MotorController:
    """Same interface as hardware.motor_controller.MotorController."""

    def __init__(self, link: ESP32Link):
        self.link = link.acquire()

    def _move(self, cmd: str, speed: int, duration: Optional[float]) -> Optional[str]:
        """Send a timed move; with a duration, block until the robot reports it finished."""
        duty = min(max(int(speed), 0), 255, MAX_DUTY)
        ms = MAX_MOVE_MS if duration is None else min(int(duration * 1000), MAX_MOVE_MS)
        turn = cmd in ("left", "right")
        step_duty = (TURN_RAMP_PCT if turn else FWD_RAMP_PCT) / 100.0 * 255
        ramp_s = max(1, int(duty / step_duty)) * (TURN_RAMP_MS if turn else FWD_RAMP_MS) / 1000.0
        wait_s = None if duration is None else ramp_s + ms / 1000.0 + DONE_MARGIN_S
        return self.link.command(cmd, wait_s=wait_s, speed=duty, ms=ms)

    def move_forward(self, speed: int = 200, duration: Optional[float] = None):
        """Move forward at speed (0-255). The firmware stops by itself near an obstacle."""
        return self._move("fwd", speed, duration)

    def move_backward(self, speed: int = 200, duration: Optional[float] = None):
        return self._move("back", speed, duration)

    def turn_left(self, speed: int = 200, duration: Optional[float] = None):
        """Pivot left: right side drives, left side stopped (same as Robot A)."""
        return self._move("left", speed, duration)

    def turn_right(self, speed: int = 200, duration: Optional[float] = None):
        return self._move("right", speed, duration)

    def stop(self):
        self.link.command("stop", wait_s=1.0)

    def cleanup(self):
        logger.info("Stopping Robot B motors")
        self.stop()
        self.link.release()


class ESP32IMU:
    """Same interface as hardware.imu_driver.IMUDriver, backed by the ESP32's gyro yaw."""

    # The ESP32 integrates its gyro at 100 Hz with spike rejection, and turns
    # measured this way matched the floor-tile check (74 deg gyro vs ~75 deg
    # in photos), so main.py may trust it during moves - unlike Robot A's
    # I2C gyro, which motor noise corrupts while driving.
    reliable_during_motion = True

    def __init__(self, link: ESP32Link):
        self.link = link.acquire()
        self.heading = 0.0
        self._ignore_before_seq = 0
        self.reset_heading(0.0)  # IMUDriver starts at heading 0 too

    def calibrate_gyroscope(self, num_samples: int = 150):
        """Re-measure gyro bias on the robot (takes ~1.5 s; robot must be still)."""
        logger.info("Calibrating Robot B gyroscope (keep it stationary)...")
        self.link.command("calib", wait_s=5.0)

    def update_heading(self):
        """Pull the latest yaw from the robot's state stream."""
        link = self.link
        if link.state and link.state_seq > self._ignore_before_seq:
            self.heading = float(link.state["yaw"]) % 360

    def get_heading(self) -> float:
        return self.heading

    def reset_heading(self, angle: float = 0.0):
        """Set the robot's yaw; state messages already in flight are ignored."""
        self.heading = angle % 360
        self.link.command("yaw", wait_s=1.0, yaw=self.heading)
        self._ignore_before_seq = self.link.state_seq

    def cleanup(self):
        self.link.release()


class ESP32Ultrasonic:
    """Same interface as hardware.ultrasonic_driver.MultiUltrasonicArray."""

    SENSOR_IDS = ("front", "left", "right")

    def __init__(self, link: ESP32Link):
        self.link = link.acquire()
        self._last_seq = 0

    def measure_all(self) -> dict:
        """
        Wait for a fresh state message (10 Hz), like Robot A's blocking reads,
        so the control loop doesn't feed the same reading into SLAM repeatedly.
        Returns metres per sensor; None for no echo or stale data.
        """
        link = self.link
        link.wait_for_state(self._last_seq, timeout=0.3)
        self._last_seq = link.state_seq
        if not link.state or time.monotonic() - link.state_time > STATE_STALE_S:
            return {sensor_id: None for sensor_id in self.SENSOR_IDS}
        readings = {}
        for sensor_id in self.SENSOR_IDS:
            value = link.state.get(sensor_id, US_NO_ECHO)
            readings[sensor_id] = None if value is None or value < 0 else float(value)
        return readings

    def get_obstacle_distances(self) -> dict:
        return self.measure_all()

    def detect_obstacles(self, threshold: float = 0.30) -> dict:
        return {sensor_id: (d is not None and d < threshold) for sensor_id, d in self.measure_all().items()}

    def cleanup(self):
        self.link.release()


class ESP32Robot:
    """Connects to one ESP32 robot and builds its three drivers."""

    def __init__(self, host: str = "localhost", port: int = 1883, robot_id: str = "B"):
        self.link = ESP32Link(host, port, robot_id)
        self.motors = ESP32MotorController(self.link)
        self.imu = ESP32IMU(self.link)
        self.ultrasonic = ESP32Ultrasonic(self.link)


def main():
    """Read-only check: connect and print Robot B's sensors for 3 seconds."""
    logging.basicConfig(level=logging.INFO)
    robot = ESP32Robot()
    try:
        for _ in range(10):
            robot.imu.update_heading()
            print(f"heading={robot.imu.get_heading():6.1f}  {robot.ultrasonic.get_obstacle_distances()}")
            time.sleep(0.3)
    finally:
        robot.ultrasonic.cleanup()
        robot.imu.cleanup()
        robot.motors.link.release()  # no stop needed: nothing was moved


if __name__ == "__main__":
    main()
