"""
Multi-Robot Communication Module (Phase 2)

Robots A (Raspberry Pi) and B (ESP32, brain on the laptop) share one MQTT
broker - mosquitto on the laptop - and exchange small messages:

    team/<id>/scan    {"x","y","heading","ranges":{"front":..,"left":..,"right":..}}
                      every sensor update; also serves as the robot's live pose
    team/<id>/target  {"x","y"} or {} - the frontier this robot is heading for
    team/<id>/status  "online" / "offline" (retained, last will)
    team/<id>/object  {"label","x","y","confidence"} - a recognised object placed on the map
    team/<id>/detections  [{"label","confidence","bearing"}] - what the camera sees right now
    team/<id>/cam     "http://<ip>:<port>/stream" (retained) - live camera view

Both robots start in one shared world frame (HARDWARE_SETUP.md start poses:
A at (0, 0), B at (0, -0.5) i.e. 50 cm to A's right, both facing +x), so a teammate's scan can be
written straight into the local occupancy grid with no map alignment. Each
robot ends up with a merged map of everything either robot has seen, and
picks frontiers away from where its teammate is going.
"""

import json
import logging
import queue
import socket
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, List, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)

TEAMMATE_TIMEOUT_S = 5.0  # a teammate silent this long no longer blocks frontiers


def _port_open(host: str, port: int, timeout: float) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def find_broker(port: int = 1883) -> Optional[str]:
    """
    Locate the MQTT broker: this machine first (the laptop), else scan this
    machine's /24 for the port - the phone hotspot hands out a new subnet
    each time it is switched on, so the laptop's IP is not fixed.
    """
    if _port_open("localhost", port, 0.5):
        return "localhost"
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("10.255.255.255", 1))  # no packet is sent; just picks the LAN interface
            my_ip = s.getsockname()[0]
    except OSError:
        return None
    prefix = my_ip.rsplit(".", 1)[0]
    hosts = [f"{prefix}.{i}" for i in range(1, 255) if f"{prefix}.{i}" != my_ip]
    logger.info(f"Scanning {prefix}.0/24 for the MQTT broker...")
    with ThreadPoolExecutor(max_workers=64) as pool:
        for host, is_open in zip(hosts, pool.map(lambda h: _port_open(h, port, 0.5), hosts)):
            if is_open:
                logger.info(f"Broker found at {host}")
                return host
    return None


class RobotCommunicator:
    """MQTT messaging between robots: scans and targets out, teammates' in."""

    def __init__(self, robot_id: str, broker_address: str = "localhost", port: int = 1883):
        """
        Args:
            robot_id: This robot's id ("A" or "B")
            broker_address: Host running the MQTT broker (the laptop)
        """
        self.robot_id = robot_id
        self.broker_address = broker_address
        self.port = port
        self.is_connected = False
        self.client = None

        self._scans = queue.Queue()  # teammate scans not yet applied to the map
        self._objects = queue.Queue()  # teammate object sightings not yet merged
        self._lock = threading.Lock()
        self._team = {}              # id -> {"pose": (x, y, heading), "target": (x, y) | None, "seen": t, "online": bool}

        logger.info(f"RobotCommunicator initialized for {robot_id}")

    def connect(self, timeout: float = 5.0) -> bool:
        """
        Connect to the broker. Returns False (robot runs solo) if it can't be
        reached, rather than stopping the robot.
        """
        logger.info(f"[{self.robot_id}] Connecting to {self.broker_address}:{self.port}")
        try:
            import paho.mqtt.client as mqtt
        except ImportError:
            logger.warning("paho-mqtt not installed (pip install paho-mqtt) - running solo")
            return False

        connected = threading.Event()

        def on_connect(client, userdata, *_):  # *_ fits both paho 1.x and 2.x signatures
            client.subscribe([("team/+/scan", 0), ("team/+/target", 1), ("team/+/status", 1),
                              ("team/+/object", 1)])
            client.publish(self._topic("status"), "online", qos=1, retain=True)
            connected.set()

        client_id = f"team-{self.robot_id}"
        if hasattr(mqtt, "CallbackAPIVersion"):  # paho-mqtt 2.x
            self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
        else:                                    # paho-mqtt 1.x (e.g. older Pi installs)
            self.client = mqtt.Client(client_id=client_id)
        self.client.will_set(self._topic("status"), "offline", qos=1, retain=True)
        self.client.on_connect = on_connect
        self.client.on_message = self._on_message
        try:
            self.client.connect(self.broker_address, self.port, keepalive=10)
        except OSError as e:
            logger.warning(f"[{self.robot_id}] Broker unreachable ({e}) - running solo")
            return False
        self.client.loop_start()
        if not connected.wait(timeout):
            logger.warning(f"[{self.robot_id}] No answer from broker - running solo")
            self.client.loop_stop()
            return False
        self.is_connected = True
        logger.info(f"[{self.robot_id}] Connected to team broker")
        return True

    def _topic(self, name: str) -> str:
        return f"team/{self.robot_id}/{name}"

    def _on_message(self, client, userdata, msg):
        _, sender, kind = msg.topic.split("/", 2)
        if sender == self.robot_id:
            return
        payload = msg.payload.decode(errors="replace")
        with self._lock:
            mate = self._team.setdefault(sender, {"pose": None, "target": None, "seen": 0.0, "online": False})
            if kind == "status":
                mate["online"] = payload == "online"
                logger.info(f"Teammate {sender} is {payload}")
                return
            try:
                data = json.loads(payload)
            except ValueError:
                return
            if kind == "scan":
                mate["seen"] = time.monotonic()
                mate["online"] = True
                mate["pose"] = (data["x"], data["y"], data["heading"])
                self._scans.put(data)
            elif kind == "target":
                mate["target"] = (data["x"], data["y"]) if "x" in data else None
            elif kind == "object":
                self._objects.put(dict(data, source=sender))

    def broadcast_scan(self, x: float, y: float, heading: float, ranges: Dict[str, Optional[float]]):
        """Share one sensor update (pose + ultrasonic ranges) with the team."""
        if not self.is_connected:
            return
        msg = {"x": round(float(x), 3), "y": round(float(y), 3), "heading": round(float(heading), 1),
               "ranges": {k: (None if v is None else round(float(v), 3)) for k, v in ranges.items()}}
        self.client.publish(self._topic("scan"), json.dumps(msg), qos=0)

    def broadcast_frontier(self, target: Optional[Tuple[float, float]]):
        """Tell the team which frontier this robot is heading for (None = none)."""
        if not self.is_connected:
            return
        msg = {} if target is None else {"x": round(float(target[0]), 3), "y": round(float(target[1]), 3)}
        self.client.publish(self._topic("target"), json.dumps(msg), qos=1, retain=True)

    def broadcast_object(self, label: str, x: float, y: float, confidence: float):
        """Share a recognised object's map position with the team."""
        if not self.is_connected:
            return
        msg = {"label": label, "x": round(float(x), 3), "y": round(float(y), 3),
               "confidence": round(float(confidence), 2)}
        self.client.publish(self._topic("object"), json.dumps(msg), qos=1)

    def broadcast_detections(self, detections: List[Dict]):
        """Share what the camera currently sees (for the live dashboard)."""
        if not self.is_connected:
            return
        self.client.publish(self._topic("detections"), json.dumps(detections), qos=0)

    def announce_camera(self, url: Optional[str]):
        """Advertise this robot's live camera stream URL (None = no camera)."""
        if not self.is_connected:
            return
        self.client.publish(self._topic("cam"), url or "offline", qos=1, retain=True)

    def local_ip(self) -> Optional[str]:
        """This machine's IP address on the network the broker is reached through."""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect((self.broker_address, self.port))  # UDP: nothing is sent
                return s.getsockname()[0]
        except OSError:
            return None

    @staticmethod
    def _drain(q: queue.Queue) -> List[Dict]:
        items = []
        while True:
            try:
                items.append(q.get_nowait())
            except queue.Empty:
                return items

    def receive_scans(self) -> List[Dict]:
        """All teammate scans received since the last call."""
        return self._drain(self._scans)

    def receive_objects(self) -> List[Dict]:
        """All teammate object sightings received since the last call."""
        return self._drain(self._objects)

    def get_team_status(self) -> Dict:
        """
        Returns:
            Dict of teammate id -> {"pose": (x, y, heading) | None, "target": (x, y) | None}
            for teammates whose scans arrived within TEAMMATE_TIMEOUT_S
        """
        now = time.monotonic()
        with self._lock:
            return {
                mate_id: {"pose": m["pose"], "target": m["target"]}
                for mate_id, m in self._team.items()
                if m["online"] and now - m["seen"] < TEAMMATE_TIMEOUT_S
            }

    def disconnect(self):
        """Disconnect from the broker (teammates see this robot go offline)."""
        logger.info(f"[{self.robot_id}] Disconnecting")
        if self.is_connected:
            self.broadcast_frontier(None)
            self.announce_camera(None)
            self.client.publish(self._topic("status"), "offline", qos=1, retain=True).wait_for_publish(2)
            self.client.loop_stop()
            self.client.disconnect()
        self.is_connected = False


class MapMerger:
    """
    Merge teammates' observations into the local map.

    Since both robots share one world frame (fixed start poses), merging is
    just replaying a teammate's scan from the teammate's pose - the same
    update the teammate applied to its own map.
    """

    def __init__(self, scan_angles: Dict[str, float], max_range: float, footprint_radius: float = 0.15):
        """
        Args:
            scan_angles: sensor id -> angle relative to robot heading (degrees)
            max_range: sensor max range in metres
            footprint_radius: area around the scanning robot cleared as free
                (the robot stands there, so it can't be a wall)
        """
        self.scan_angles = scan_angles
        self.max_range = max_range
        self.footprint_radius = footprint_radius

    def apply_scan(self, occupancy_grid, scan: Dict) -> bool:
        """Write one teammate scan into occupancy_grid. Returns True if the map changed."""
        angles, distances = [], []
        for sensor_id, distance in scan.get("ranges", {}).items():
            if distance is not None and sensor_id in self.scan_angles:
                angles.append(self.scan_angles[sensor_id])
                distances.append(distance)
        if distances:
            occupancy_grid.update_from_range_scan(
                np.array(angles), np.array(distances),
                scan["x"], scan["y"], scan["heading"],
                max_range=self.max_range
            )
        occupancy_grid.mark_footprint_free(scan["x"], scan["y"], self.footprint_radius)
        return True


class CoordinationPlanner:
    """Pick frontiers so robots spread out instead of exploring the same area."""

    def __init__(self, robot_id: str, exclusion_radius_m: float = 0.75):
        """
        Args:
            exclusion_radius_m: frontiers this close to a teammate's target or
                current position are left to that teammate
        """
        self.robot_id = robot_id
        self.exclusion_radius_m = exclusion_radius_m

    def allocate_frontier(self, candidates: List[Tuple[float, float]], team_status: Dict) -> List[Tuple[float, float]]:
        """
        Filter frontier candidates against teammates' targets and positions.

        Returns:
            Candidates not claimed by a teammate, or all candidates if every
            one is claimed (better to overlap than to stop exploring early)
        """
        claimed = []
        for mate in team_status.values():
            if mate["target"] is not None:
                claimed.append(mate["target"])
            if mate["pose"] is not None:
                claimed.append(mate["pose"][:2])
        if not claimed:
            return candidates

        free = [
            f for f in candidates
            if all(np.hypot(f[0] - cx, f[1] - cy) >= self.exclusion_radius_m for cx, cy in claimed)
        ]
        return free or candidates
