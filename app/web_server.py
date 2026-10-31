"""
Live Team Dashboard (runs on the laptop)

Listens to the same MQTT messages the robots exchange and shows them live
in a browser:
- the shared occupancy map, rebuilt here from both robots' scans
- each robot's position, heading, path and current frontier target
- recognised objects placed on the map
- Robot A's camera with the AI boxes, its live detections, and Robot B's
  ESP32-CAM when it is online

Nothing on the robots depends on this - it only listens.

Usage (from the project root, while the robots run):
    python -m app.web_server
    then open http://localhost:5000 (or http://<laptop-ip>:5000 on a phone)
"""

import base64
import json
import logging
import threading
import time

import numpy as np
import paho.mqtt.client as mqtt
from flask import Flask, Response, jsonify, render_template

from communication.robot_communicator import MapMerger, find_broker
from main import ULTRASONIC_MAX_RANGE, ULTRASONIC_SCAN_ANGLES, ROBOT_B_START_POSITION, ROBOT_FOOTPRINT_RADIUS_M
from slam.occupancy_grid import OccupancyGrid
from vision.object_detector import ObjectMap

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
logger = logging.getLogger(__name__)

ROBOT_IDS = ("A", "B")
START_POSES = {"A": (0.0, 0.0), "B": ROBOT_B_START_POSITION}
ACTIVE_TIMEOUT_S = 5.0      # a robot whose scans stopped this long ago is shown as idle
PATH_STEP_M = 0.03          # record a path point after moving this far
DETECTIONS_STALE_S = 3.0
EVENT_INTERVAL_S = 0.25     # robot/state updates to the browser
MAP_INTERVAL_S = 1.0        # map image updates to the browser (when changed)


class TeamMonitor:
    """MQTT listener that keeps the team's live state and a merged map."""

    def __init__(self, broker: str, client_id: str = "dashboard"):
        self.lock = threading.Lock()
        self.broker = broker
        self.connected = False
        self.started = time.time()
        self._reset_state()

        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        self.client.connect_async(broker, 1883, keepalive=10)
        self.client.loop_start()

    def _reset_state(self):
        self.grid = OccupancyGrid(width=500, height=500, resolution=0.05, origin_x=-12.5, origin_y=-12.5)
        self.merger = MapMerger(ULTRASONIC_SCAN_ANGLES, ULTRASONIC_MAX_RANGE, ROBOT_FOOTPRINT_RADIUS_M)
        self.objects = ObjectMap()
        self.map_version = 0
        self.robots = {rid: {
            "id": rid, "online": False, "last_scan": 0.0, "pose": None, "path": [],
            "ranges": {}, "target": None, "cam": None, "body_cam": None, "detections": [], "detections_time": 0.0,
            "scans": 0, "body_online": None, "body": None,
        } for rid in ROBOT_IDS}

    def reset(self):
        """Clear the map, paths and objects (e.g. before a new run); connection info is kept."""
        with self.lock:
            keep = {rid: (r["cam"], r["body_cam"], r["online"], r["body_online"]) for rid, r in self.robots.items()}
            self._reset_state()
            self.map_version = int(time.time())  # forces browsers to take the (now empty) map
            for rid, (cam, body_cam, online, body_online) in keep.items():
                self.robots[rid].update(cam=cam, body_cam=body_cam, online=online, body_online=body_online)

    # ---------- MQTT ----------
    def _on_connect(self, client, userdata, flags, reason_code, properties):
        self.connected = True
        client.subscribe([("team/#", 0), ("robot/+/status", 1), ("robot/+/cam", 1), ("robot/+/state", 0)])
        logger.info(f"Dashboard listening on broker {self.broker}")

    def _on_disconnect(self, client, userdata, flags, reason_code, properties):
        self.connected = False

    def _on_message(self, client, userdata, msg):
        parts = msg.topic.split("/")
        if len(parts) != 3 or parts[1] not in ROBOT_IDS:
            return
        family, rid, kind = parts
        payload = msg.payload.decode(errors="replace")
        with self.lock:
            robot = self.robots[rid]
            if family == "robot":                      # Robot B's ESP32 body / camera
                if kind == "status":
                    robot["body_online"] = payload == "online"
                elif kind == "cam":
                    # Kept apart from the brain's team/<id>/cam: Robot B's brain announces
                    # "offline" when it stops, which must not hide the ESP32-CAM
                    robot["body_cam"] = None if payload == "offline" else payload
                elif kind == "state":
                    try:
                        robot["body"] = json.loads(payload)
                    except ValueError:
                        pass
                return
            if kind == "status":
                robot["online"] = payload == "online"
                return
            if kind == "cam":
                robot["cam"] = None if payload == "offline" else payload
                return
            try:
                data = json.loads(payload)
            except ValueError:
                return
            if kind == "scan":
                self._on_scan(robot, data)
            elif kind == "target":
                robot["target"] = (data["x"], data["y"]) if "x" in data else None
            elif kind == "object":
                self.objects.add(data["label"], data["x"], data["y"], data["confidence"], rid)
            elif kind == "detections":
                robot["detections"] = data
                robot["detections_time"] = time.time()

    def _on_scan(self, robot, data):
        robot["online"] = True
        robot["last_scan"] = time.time()
        robot["scans"] += 1
        robot["pose"] = (data["x"], data["y"], data["heading"])
        robot["ranges"] = data.get("ranges", {})
        path = robot["path"]
        if not path or np.hypot(path[-1][0] - data["x"], path[-1][1] - data["y"]) >= PATH_STEP_M:
            path.append((data["x"], data["y"]))
        if self.merger.apply_scan(self.grid, data):
            self.map_version += 1

    # ---------- snapshots for the browser ----------
    def snapshot(self) -> dict:
        now = time.time()
        with self.lock:
            robots = {}
            for rid, r in self.robots.items():
                active = r["online"] and now - r["last_scan"] < ACTIVE_TIMEOUT_S
                if active:
                    status = "exploring"
                elif r["online"]:
                    status = "connected"
                elif r["body_online"]:
                    status = "standby"   # ESP32 body online, brain not running
                else:
                    status = "offline"
                fresh = now - r["detections_time"] < DETECTIONS_STALE_S
                robots[rid] = {
                    "id": rid, "status": status, "pose": r["pose"], "path": r["path"][-1500:],
                    "start": START_POSES[rid], "ranges": r["ranges"], "target": r["target"],
                    "cam": r["cam"] or r["body_cam"], "detections": r["detections"] if fresh else [],
                    "scans": r["scans"],
                }
            known = int((self.grid.grid != -1).sum())
            return {
                "connected": self.connected, "broker": self.broker, "robots": robots,
                "objects": self.objects.to_list(), "known_cells": known,
                "area_m2": round(known * self.grid.resolution ** 2, 2),
                "map_version": self.map_version, "uptime": round(now - self.started),
            }

    def map_payload(self) -> dict:
        """Known part of the merged map: 0 = unknown, 1 = free, 2 = occupied, row 0 = lowest y."""
        with self.lock:
            g = self.grid.grid
            known = g != -1
            res = self.grid.resolution
            if not known.any():
                return {"empty": True, "version": self.map_version}
            ys, xs = np.where(known)
            pad = 10
            y0, y1 = max(0, ys.min() - pad), min(g.shape[0], ys.max() + pad + 1)
            x0, x1 = max(0, xs.min() - pad), min(g.shape[1], xs.max() + pad + 1)
            crop = g[y0:y1, x0:x1]
            cells = np.zeros(crop.shape, dtype=np.uint8)
            cells[(crop >= 0) & (crop <= 50)] = 1
            cells[crop > 50] = 2
            return {
                "version": self.map_version, "res": res,
                "x0": self.grid.origin_x + x0 * res, "y0": self.grid.origin_y + y0 * res,
                "w": int(cells.shape[1]), "h": int(cells.shape[0]),
                "cells": base64.b64encode(cells.tobytes()).decode(),
            }


app = Flask(__name__)
monitor = None


@app.route("/")
def index():
    return render_template("dashboard.html")


@app.route("/events")
def events():
    """Server-sent events: robot state every 0.25 s, the map at most once a second when it changed."""
    def stream():
        sent_version, sent_at = None, 0.0
        while True:
            payload = {"state": monitor.snapshot()}
            if monitor.map_version != sent_version and time.time() - sent_at >= MAP_INTERVAL_S:
                payload["map"] = monitor.map_payload()
                sent_version, sent_at = payload["map"]["version"], time.time()
            yield f"data: {json.dumps(payload)}\n\n"
            time.sleep(EVENT_INTERVAL_S)
    return Response(stream(), mimetype="text/event-stream",
                    headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})


@app.route("/api/state")
def api_state():
    return jsonify(monitor.snapshot())


@app.route("/api/map")
def api_map():
    return jsonify(monitor.map_payload())


@app.route("/api/reset", methods=["POST"])
def api_reset():
    monitor.reset()
    return jsonify({"ok": True})


def main():
    global monitor
    broker = find_broker() or "localhost"
    monitor = TeamMonitor(broker)
    logger.info("Dashboard on http://localhost:5000  (Ctrl+C to stop)")
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)


if __name__ == "__main__":
    main()
