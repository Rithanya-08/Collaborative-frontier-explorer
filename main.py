"""
Main Robot Control Loop

Orchestrates all hardware and software components:
- Sensor data collection (IMU, Ultrasonic)
- Robot movement and navigation
- SLAM and map building
- Visualization server
"""

import argparse
import time
import logging
import signal
import sys
import numpy as np
from datetime import datetime
import threading
from typing import Optional, Tuple

# Import custom modules (hardware drivers are imported per robot in
# RobotController._initialize - Robot A's need the Pi, Robot B's run here)
from slam.occupancy_grid import OccupancyGrid
from navigation.frontier_explorer import FrontierExplorer
from navigation.path_planner import PathPlanner, ALGORITHMS as PLANNERS
from communication.robot_communicator import RobotCommunicator, MapMerger, CoordinationPlanner, find_broker
from vision.object_detector import ObjectMap

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Fixed sensor angles relative to robot heading (0=front, 90=left, 270=right),
# used to feed the 3 ultrasonic readings into the occupancy grid as a scan.
ULTRASONIC_SCAN_ANGLES = {"front": 0.0, "left": 90.0, "right": 270.0}
ULTRASONIC_MAX_RANGE = 4.0  # meters, matches the HC-SR04 sensors

# Reactive obstacle avoidance (values match the tuned/tested scripts/obstacle_avoidance.py)
OBSTACLE_THRESHOLD_M = 0.30       # front - a head-on hit is the highest-risk case
SIDE_OBSTACLE_THRESHOLD_M = 0.15  # left/right - less sensitive than front, so
                                   # grazing past furniture at an angle doesn't
                                   # trigger constant avoidance stops
AVOID_SPEED = 150
AVOID_BACKWARD_DURATION_S = 0.15
AVOID_TURN_DURATION_S = 1.0

# A sensor whose reading stays put while the robot backs up and turns away
# from it again and again is not seeing a real obstacle (Robot A's left sensor
# read 0.07-0.08 m wherever the robot stood - something on the robot itself),
# and would trap the robot in an endless back-up-and-turn loop. After this
# many avoidances in a row triggered by the same unchanging reading, that
# sensor is ignored for the rest of the run.
STUCK_SENSOR_AVOIDANCES = 3
STUCK_SENSOR_TOLERANCE_M = 0.03

# Calibrated pivot-turn rate at this speed/ramp - measured via
# calibrate_turn_rate.py (10x HEADING_TURN_NUDGE_S pivots, photographed
# against a fixed reference: ~45 degrees total, ~4.5 degrees/nudge).
# Replaces the turn_90.py-derived guess of 60 deg/s, which turned out to
# be roughly double the real rate at this speed/duration.
# Used for open-loop heading estimates in _turn_nudge() instead of
# reading the gyro while motors are actively drawing current.
TURN_RATE_DEG_PER_S = 30.0

# Heading-correction turns in _move_towards() are short bounded nudges, not
# open-ended spins - each control loop iteration already takes a variable,
# non-trivial amount of time (ultrasonic reads have several built-in
# delays), so an un-timed turn_left()/turn_right() call would rotate an
# uncontrolled amount before the next heading check, overshooting the
# target repeatedly instead of converging.
HEADING_TURN_NUDGE_S = 0.15

# Forward driving is also a short bounded burst rather than a continuous
# drive - re-checking heading/obstacles/frontiers every FORWARD_STEP_S
# means smaller, more deliberate steps instead of committing to one long
# straight-line run that could overshoot a frontier or drift off course.
FORWARD_STEP_S = 0.3

# Measured directly with calibrate_forward_speed.py (see that file) at the
# exact speed/duration used below - not a meters-per-second rate scaled
# by duration, since the motors ramp up over ~0.55s and never reach
# cruising speed within one 0.3s burst, so a rate extrapolated from a
# longer, mostly-at-speed drive would overestimate a real burst's distance.
# Re-measured on the Pi 5 (2026-10-20): 5 bursts toward a wall 1.47 m away,
# read with the front ultrasonic - 6.8, 3.8, 5.5, 5.8, 3.5 cm, avg 5.1 cm
# (was 6.2 cm on the Pi 3B). The gyro saw ~0.5 deg/burst drift to the left.
FORWARD_STEP_DISTANCE_M = 0.051

# Robot B (ESP32 over MQTT, hardware/esp32_robot.py) - measured the same way
# on the real robot, same speed/durations, same ramp timing as Robot A:
# 5 forward bursts covered 116cm on the floor (veering ~1.9 deg right per
# burst), and 10 left nudges turned 74.1 deg by its gyro (~75 deg against
# the floor tiles in photos), i.e. 7.4 deg per 0.15s nudge.
ROBOT_B_FORWARD_STEP_DISTANCE_M = 0.232
ROBOT_B_TURN_RATE_DEG_PER_S = 49.4
ROBOT_B_START_POSITION = (0.0, -0.5)  # HARDWARE_SETUP.md start poses: B 50 cm to A's right

# A sensor reading that ends within this distance of the teammate's reported
# position is the teammate itself, not a wall (chassis ~15x25 cm, plus slack
# for both robots' dead-reckoning error)
TEAMMATE_RADIUS_M = 0.3
# Cells a robot stands on are cleared to free space on every scan (its own and
# the teammate's), erasing a teammate mapped as a wall before its position was
# known or when position drift made the filter above miss it
ROBOT_FOOTPRINT_RADIUS_M = 0.15

# Object recognition (Robot A's camera, vision/object_detector.py). The camera
# gives each object's bearing; its distance comes from the front ultrasonic,
# so only objects near the centre line - inside the HC-SR04's ~15 deg
# half-angle beam, with some slack - are placed on the map.
OBJECT_MAX_BEARING_DEG = 20.0
OBJECT_MIN_CONFIDENCE = 0.45  # yolov8n at 320 px scored a clearly visible chair 0.46-0.48
OBJECT_MAX_RANGE_M = 3.0  # beyond this the front reading is too unreliable to place objects

# Exploration memory tuning (see RobotController.visited_positions/current_target)
TARGET_REACHED_RADIUS_M = 0.15   # "close enough" to a target to pick a new one
VISITED_EXCLUSION_RADIUS_M = 0.4  # new frontier candidates this close to
                                   # a breadcrumb are skipped, unless every
                                   # candidate is excluded (then use them anyway)
BREADCRUMB_INTERVAL_SCANS = 15    # drop a breadcrumb roughly this often

# Route to the target (navigation/path_planner.py): re-planned this often so
# walls mapped on the way are routed around, not just bumped into
REPLAN_INTERVAL_S = 2.0


class RobotController:
    """
    Main robot controller orchestrating all components.
    
    Manages:
    - Hardware initialization and shutdown
    - Sensor data collection
    - SLAM/mapping
    - Navigation and exploration
    - Data logging
    """
    
    def __init__(self, config_file: str = "config/robot_config.yaml", robot: str = "a",
                 broker: str = "localhost", vision: bool = True, run_minutes: float = 0,
                 planner: str = "astar"):
        """
        Initialize robot controller for robot "a" (Pi, GPIO) or "b" (ESP32, MQTT).

        broker is the laptop running mosquitto: Robot B's body and the team
        messages (shared scans/targets) both go through it.
        """
        self.config_file = config_file
        self.robot = robot
        self.broker = broker
        self.vision = vision  # object recognition with Robot A's camera
        self.run_minutes = run_minutes  # stop (and save the map) after this long; 0 = until Ctrl+C
        self.planner = planner          # route algorithm: astar, rrt, prm or simple (straight line)

        # Per-robot calibration (see the constants' definitions above)
        if robot == "b":
            self.forward_step_distance_m = ROBOT_B_FORWARD_STEP_DISTANCE_M
            self.turn_rate_deg_per_s = ROBOT_B_TURN_RATE_DEG_PER_S
            start_position = ROBOT_B_START_POSITION
        else:
            self.forward_step_distance_m = FORWARD_STEP_DISTANCE_M
            self.turn_rate_deg_per_s = TURN_RATE_DEG_PER_S
            start_position = (0.0, 0.0)

        # State
        self.running = False
        self.position = np.array(start_position, dtype=float)  # x, y
        self.heading = 0.0  # degrees
        self.velocity = np.array([0.0, 0.0])
        self.last_obstacle_distances = {}
        self._stopped = False

        # Exploration memory - without this, get_next_frontier() picks the
        # closest frontier fresh every loop iteration with no history,
        # which (especially combined with position drift) causes the
        # robot to bounce back into areas it's already covered.
        self.current_target = None   # (x, y) frontier being pursued, kept
                                      # until reached instead of re-picked
                                      # every iteration
        self.visited_positions = []  # breadcrumb trail; new frontier
                                      # candidates near these are skipped
        self.direction_changes = []  # (x, y, "L"/"R") - one marker per
                                      # distinct turn, for labeling turns
                                      # along the path on the rendered map
        self._last_turn_key = None   # dedupes consecutive same-spot,
                                      # same-direction nudges into one marker
        self.ignored_sensors = set()  # sensors found stuck (see STUCK_SENSOR_AVOIDANCES)
        self._trigger_history = {}    # sensor -> its readings at consecutive avoidances it triggered
        self.waypoints = []          # planned route to current_target,
                                      # next waypoint first
        self._planned_at = 0.0       # when the route was last planned

        # Components
        self.motor_controller = None
        self.imu = None
        self.ultrasonic = None
        self.occupancy_grid = None
        self.frontier_explorer = None
        self.path_planner = None
        self.team = None                # RobotCommunicator (solo if not connected)
        self.map_merger = None
        self.coordination = None
        self._announced_target = None   # last target sent to the team
        self.detector = None            # ObjectDetector (Robot A only)
        self.object_map = ObjectMap()   # recognised objects, ours and the teammate's
        self._still_since = 0.0         # when the last move ended; earlier frames are blurred

        # Threads
        self.sensor_thread = None
        self.slam_thread = None
        
        # Stats
        self.start_time = None
        self.total_distance = 0.0
        self.scans_processed = 0
        
        self._initialize()
    
    def _initialize(self):
        """Initialize all robot components."""
        logger.info("Initializing robot components...")
        
        try:
            if self.robot == "b":
                from hardware.esp32_robot import ESP32Robot
                esp32 = ESP32Robot(host=self.broker)
                self.motor_controller = esp32.motors
                self.imu = esp32.imu
                self.ultrasonic = esp32.ultrasonic
                logger.info("✓ Robot B (ESP32) connected over MQTT")
            else:
                from hardware.motor_controller import MotorController
                from hardware.imu_driver import IMUDriver
                from hardware.ultrasonic_driver import MultiUltrasonicArray

                # Initialize motors
                self.motor_controller = MotorController()
                logger.info("✓ Motor controller initialized")

                # Initialize sensors
                self.imu = IMUDriver()
                logger.info("✓ IMU initialized")

                self.ultrasonic = MultiUltrasonicArray()
                logger.info("✓ Ultrasonic sensors initialized")
            
            # Initialize mapping
            self.occupancy_grid = OccupancyGrid(
                width=500,
                height=500,
                resolution=0.05,
                origin_x=-12.5,
                origin_y=-12.5
            )
            logger.info("✓ Occupancy grid initialized")
            
            # Initialize explorer - a small min_frontier_size so the robot
            # keeps chasing small unexplored pockets (likely with only 3
            # fixed sensor rays instead of a full sweep) rather than
            # declaring exploration complete while gaps remain
            self.frontier_explorer = FrontierExplorer(self.occupancy_grid, min_frontier_size=3)
            logger.info("✓ Frontier explorer initialized")

            self.path_planner = PathPlanner(self.occupancy_grid)
            logger.info(f"✓ Path planner initialized ({self.planner})")

            # Team link - share scans/targets with the other robot; a
            # missing broker just means exploring solo
            self.team = RobotCommunicator(self.robot.upper(), broker_address=self.broker)
            self.map_merger = MapMerger(ULTRASONIC_SCAN_ANGLES, ULTRASONIC_MAX_RANGE, ROBOT_FOOTPRINT_RADIUS_M)
            self.coordination = CoordinationPlanner(self.robot.upper())
            if self.team.connect():
                logger.info("✓ Team link connected")
            else:
                logger.info("Team link unavailable - exploring solo")

            # Object recognition - optional: exploring works without it
            if self.robot == "a" and self.vision:
                try:
                    from vision.object_detector import ObjectDetector, STREAM_PORT
                    self.detector = ObjectDetector(on_detections=self._share_detections)
                    self.detector.start()
                    ip = self.team.local_ip()
                    if ip:
                        self.team.announce_camera(f"http://{ip}:{STREAM_PORT}/stream")
                    logger.info("✓ Object recognition running (live view on port 8080)")
                except Exception as e:
                    self.detector = None
                    logger.warning(f"Object recognition unavailable ({e}) - mapping without it")

            logger.info("All components initialized successfully!")
        
        except Exception as e:
            logger.error(f"Initialization failed: {e}")
            raise
    
    def start(self):
        """Start robot operation."""
        if self.running:
            logger.warning("Robot already running")
            return
        
        logger.info("Starting robot...")
        self.running = True
        self.start_time = time.time()
        
        # Calibrate IMU
        logger.info("Calibrating IMU...")
        self.imu.calibrate_gyroscope()
        # Robot A: integrate the gyro in the background so heading stays valid
        # during moves (filtered + spike-rejected; a 1 s pivot read a smooth
        # +33.6 deg with no motor-noise jumps), so _turn_nudge() can trust it
        if hasattr(self.imu, "start_tracking"):
            self.imu.start_tracking()

        # Start main control loop
        self._control_loop()
    
    def _control_loop(self):
        """Main control loop."""
        logger.info("Entering control loop...")
        
        loop_rate = 50  # Hz
        loop_period = 1.0 / loop_rate
        
        while self.running:
            loop_start = time.time()
            if self.run_minutes and loop_start - self.start_time > self.run_minutes * 60:
                logger.info(f"Run time of {self.run_minutes} min reached")
                break

            try:
                # Update sensors
                self._update_sensors()
                
                # Update SLAM
                self._update_slam()
                
                # Recognise objects in frames taken since the last move
                self._update_vision()

                # Update navigation
                self._update_navigation()
                self._announce_target()
                self._still_since = time.monotonic()

                # Log periodic status
                if self.scans_processed % 50 == 0:
                    self._log_status()
            
            except KeyboardInterrupt:
                logger.info("Control loop interrupted")
                self.stop()
                break
            
            except Exception as e:
                logger.error(f"Error in control loop: {e}")
            
            # Maintain loop rate
            loop_time = time.time() - loop_start
            if loop_time < loop_period:
                time.sleep(loop_period - loop_time)
    
    def _update_sensors(self):
        """Update sensor readings."""
        # Get IMU data
        self.imu.update_heading()
        self.heading = self.imu.get_heading()

        # Get ultrasonic readings (front, left, right), minus any sensor found stuck
        self.last_obstacle_distances = {
            sensor_id: None if sensor_id in self.ignored_sensors else distance
            for sensor_id, distance in self.ultrasonic.get_obstacle_distances().items()
        }

    def _without_teammate(self, ranges: dict) -> dict:
        """
        Copy of the ultrasonic ranges with readings that end on the teammate
        set to None, so the other robot is never drawn into the map as a wall
        (at the start the robots stand 50 cm apart and see each other).
        Obstacle avoidance keeps using the raw readings - physically the
        teammate is still something not to drive into.
        """
        mates = [m["pose"] for m in self.team.get_team_status().values() if m["pose"]]
        if not mates:
            return ranges
        filtered = dict(ranges)
        for sensor_id, distance in ranges.items():
            if distance is None or sensor_id not in ULTRASONIC_SCAN_ANGLES:
                continue
            angle = np.radians(self.heading + ULTRASONIC_SCAN_ANGLES[sensor_id])
            hit_x = self.position[0] + distance * np.cos(angle)
            hit_y = self.position[1] + distance * np.sin(angle)
            if any(np.hypot(hit_x - mx, hit_y - my) < TEAMMATE_RADIUS_M for mx, my, _ in mates):
                filtered[sensor_id] = None
        return filtered

    def _update_slam(self):
        """Update SLAM and occupancy grid from the ultrasonic readings."""
        mapped_ranges = self._without_teammate(self.last_obstacle_distances)
        scan_angles = []
        scan_distances = []
        for sensor_id, relative_angle in ULTRASONIC_SCAN_ANGLES.items():
            distance = mapped_ranges.get(sensor_id)
            if distance is not None:
                scan_angles.append(relative_angle)
                scan_distances.append(distance)

        if scan_distances:
            # Update occupancy grid with current readings
            self.occupancy_grid.update_from_range_scan(
                np.array(scan_angles),
                np.array(scan_distances),
                self.position[0],
                self.position[1],
                self.heading,
                max_range=ULTRASONIC_MAX_RANGE
            )

            self.scans_processed += 1
        self.occupancy_grid.mark_footprint_free(self.position[0], self.position[1], ROBOT_FOOTPRINT_RADIUS_M)

        # Share this update (also our live pose) and merge the teammate's
        # scans into our map - same world frame, so no alignment needed
        self.team.broadcast_scan(self.position[0], self.position[1], self.heading, mapped_ranges)
        for scan in self.team.receive_scans():
            self.map_merger.apply_scan(self.occupancy_grid, scan)

    def _update_vision(self):
        """
        Place recognised objects on the map: bearing from the camera, range
        from the front ultrasonic, so only roughly-centred objects qualify.
        Also merges objects the teammate recognised.
        """
        for obj in self.team.receive_objects():
            self.object_map.add(obj["label"], obj["x"], obj["y"], obj["confidence"], obj["source"])

        if self.detector is None:
            return
        front = self.last_obstacle_distances.get("front")
        if front is None or front > OBJECT_MAX_RANGE_M:
            return

        for det in self.detector.get_detections(after=self._still_since):
            if det.confidence < OBJECT_MIN_CONFIDENCE or abs(det.bearing_deg) > OBJECT_MAX_BEARING_DEG:
                continue
            angle = np.radians(self.heading + det.bearing_deg)
            x = self.position[0] + front * np.cos(angle)
            y = self.position[1] + front * np.sin(angle)
            if self.object_map.add(det.label, x, y, det.confidence, self.robot.upper()):
                logger.info(f"Recognised {det.label} ({det.confidence:.2f}) at ({x:.2f}, {y:.2f})")
                self.team.broadcast_object(det.label, x, y, det.confidence)

    def _share_detections(self, detections):
        """Detector callback: live detections for the dashboard (not mapped here)."""
        self.team.broadcast_detections([
            {"label": d.label, "confidence": round(d.confidence, 2), "bearing": round(d.bearing_deg, 1)}
            for d in detections
        ])

    def _announce_target(self):
        """Tell the teammate whenever our frontier target changes."""
        if self.current_target != self._announced_target:
            self.team.broadcast_frontier(self.current_target)
            self._announced_target = self.current_target

    def _update_navigation(self):
        """Update navigation and movement."""
        front = self.last_obstacle_distances.get("front")
        left = self.last_obstacle_distances.get("left")
        right = self.last_obstacle_distances.get("right")

        # Trigger on ANY of the 3 sensors, not just front - a side sensor
        # picking up a nearby wall means continuing forward would
        # scrape/hit it even if the front itself is still clear.
        obstacle_close = (
            (front is not None and front < OBSTACLE_THRESHOLD_M)
            or (left is not None and left < SIDE_OBSTACLE_THRESHOLD_M)
            or (right is not None and right < SIDE_OBSTACLE_THRESHOLD_M)
        )
        if obstacle_close:
            self._avoid_obstacle(front, left, right)
            return

        # Drop a breadcrumb periodically so new frontier picks can avoid
        # areas already covered, independent of the occupancy grid itself
        if self.scans_processed % BREADCRUMB_INTERVAL_SCANS == 0:
            self.visited_positions.append(tuple(self.position))

        # Keep pursuing the current target instead of re-picking a
        # "closest frontier" fresh every iteration - that had no memory
        # at all, so drift or a newly-formed nearby frontier could bounce
        # the robot straight back into ground it had just covered.
        if self.current_target is not None:
            dist_to_target = np.linalg.norm(np.array(self.current_target) - self.position)
            if dist_to_target < TARGET_REACHED_RADIUS_M:
                self.current_target = None

        if self.current_target is None:
            self.current_target = self._pick_next_frontier()
            self.waypoints = []

        if self.current_target is not None:
            if not self.waypoints or time.monotonic() - self._planned_at > REPLAN_INTERVAL_S:
                self._plan_route()
            # Move on to the next waypoint once this one is reached (the
            # last one is the target itself, handled above)
            while len(self.waypoints) > 1 and np.linalg.norm(
                    np.array(self.waypoints[0]) - self.position) < TARGET_REACHED_RADIUS_M:
                self.waypoints.pop(0)
            waypoint_x, waypoint_y = self.waypoints[0]
            self._move_towards(waypoint_x, waypoint_y)
        else:
            # No frontier found - exploration complete
            logger.info("No frontier found - exploration complete!")
            self.motor_controller.stop()

    def _plan_route(self):
        """
        Plan waypoints from here to current_target around mapped walls. If the
        map shows no way through, drive straight at the target as before -
        reactive avoidance still catches whatever is really in the way.
        """
        first_plan = not self.waypoints
        path = self.path_planner.plan_path(self.position[0], self.position[1],
                                           *self.current_target, algorithm=self.planner)
        self._planned_at = time.monotonic()
        self.waypoints = path[1:] if len(path) > 1 else [tuple(self.current_target)]
        if first_plan:
            tx, ty = self.current_target
            if path:
                logger.info(f"Route to ({tx:.2f}, {ty:.2f}) via {self.planner}: "
                            f"{len(self.waypoints)} waypoint(s)")
            else:
                logger.info(f"Route to ({tx:.2f}, {ty:.2f}): no way through on the map - heading straight")

    def _pick_next_frontier(self) -> Optional[Tuple[float, float]]:
        """
        Pick the closest frontier, preferring ones away from recently
        visited ground. Falls back to the closest frontier overall if
        every candidate happens to be near a breadcrumb (better to revisit
        briefly than to report exploration complete while frontiers exist).
        """
        frontiers = self.frontier_explorer.find_frontiers()
        if not frontiers:
            return None

        def far_from_visited(f):
            fx, fy = f
            return all(
                np.hypot(fx - vx, fy - vy) >= VISITED_EXCLUSION_RADIUS_M
                for vx, vy in self.visited_positions
            )

        candidates = [f for f in frontiers if far_from_visited(f)] or frontiers

        # Leave frontiers near the teammate's target/position to the teammate
        candidates = self.coordination.allocate_frontier(candidates, self.team.get_team_status())

        return min(
            candidates,
            key=lambda f: np.hypot(f[0] - self.position[0], f[1] - self.position[1])
        )

    def _avoid_obstacle(self, front, left, right):
        """
        Reactive obstacle avoidance: stop, back off, and pivot toward
        whichever side leads back toward unexplored space (see
        _choose_avoidance_direction), before letting frontier-seeking
        navigation resume on the next loop iteration.

        This runs ahead of frontier-seeking in _update_navigation() so the
        robot never drives straight into something just because it's the
        most direct path toward the next frontier - triggered by any of
        the 3 sensors (front OR either side), not just front.
        """
        close = []
        if front is not None and front < OBSTACLE_THRESHOLD_M:
            close.append(f"front={front:.2f}m")
        if left is not None and left < SIDE_OBSTACLE_THRESHOLD_M:
            close.append(f"left={left:.2f}m")
        if right is not None and right < SIDE_OBSTACLE_THRESHOLD_M:
            close.append(f"right={right:.2f}m")
        logger.info(f"Obstacle close ({', '.join(close)}). Avoiding.")
        self._check_stuck_sensors({"front": front, "left": left, "right": right})

        self.motor_controller.stop()
        self.motor_controller.move_backward(AVOID_SPEED, duration=AVOID_BACKWARD_DURATION_S)

        turn_direction = self._choose_avoidance_direction(left, right)
        self._turn_nudge(turn_direction, AVOID_SPEED, AVOID_TURN_DURATION_S)

        # The target we were heading for is on the other side of whatever
        # we just hit - drop it so the next navigation update picks a
        # fresh frontier (in the direction we just turned toward) instead
        # of immediately steering back into the same obstacle.
        self.current_target = None
        self.waypoints = []

    def _check_stuck_sensors(self, readings: dict):
        """Ignore a sensor that keeps triggering avoidance with the same reading."""
        thresholds = {"front": OBSTACLE_THRESHOLD_M, "left": SIDE_OBSTACLE_THRESHOLD_M,
                      "right": SIDE_OBSTACLE_THRESHOLD_M}
        for sensor_id, distance in readings.items():
            if distance is None or distance >= thresholds[sensor_id]:
                self._trigger_history.pop(sensor_id, None)
                continue
            recent = self._trigger_history.setdefault(sensor_id, [])
            recent.append(distance)
            recent[:] = recent[-STUCK_SENSOR_AVOIDANCES:]
            if len(recent) == STUCK_SENSOR_AVOIDANCES and max(recent) - min(recent) < STUCK_SENSOR_TOLERANCE_M:
                self.ignored_sensors.add(sensor_id)
                del self._trigger_history[sensor_id]
                logger.warning(f"Sensor '{sensor_id}' stays at {distance:.2f} m however the robot moves - "
                               f"ignoring it for this run (check nothing on the robot blocks it)")

    def _choose_avoidance_direction(self, left, right) -> str:
        """
        Pick which way to turn when avoiding an obstacle: prefer whichever
        side (left/right of current heading) leads toward a known
        frontier - i.e. unexplored space - rather than just whichever side
        has more immediate sensor clearance. Falls back to sensor
        clearance only when frontier data doesn't favor either side (none
        found, or both sides have one).
        """
        frontiers = self.frontier_explorer.find_frontiers()
        left_has_frontier = False
        right_has_frontier = False
        for fx, fy in frontiers:
            angle_to_frontier = np.degrees(np.arctan2(fy - self.position[1], fx - self.position[0]))
            rel_angle = (angle_to_frontier - self.heading + 180) % 360 - 180  # -180..180
            if rel_angle > 0:
                left_has_frontier = True
            elif rel_angle < 0:
                right_has_frontier = True

        if left_has_frontier and not right_has_frontier:
            return "left"
        if right_has_frontier and not left_has_frontier:
            return "right"

        left_space = left if left is not None else ULTRASONIC_MAX_RANGE
        right_space = right if right is not None else ULTRASONIC_MAX_RANGE
        return "left" if left_space >= right_space else "right"

    def _turn_nudge(self, direction: str, speed: int, total_duration: float):
        """
        Turn for total_duration using a calibrated open-loop time-based
        heading estimate, without reading the IMU at all while the motors
        are actively running.

        Motor current draw introduces electrical noise that can silently
        corrupt I2C reads to the gyro (wrong values, not just outright
        read failures - confirmed by testing: sampling the gyro every
        ~20ms during a turn still produced 80-120 degree phantom jumps
        that don't match this hardware's real turn rate). So the gyro is
        only trusted while the robot is stationary; heading during the
        turn itself is estimated from TURN_RATE_DEG_PER_S, calibrated the
        same way turn_90.py/obstacle_avoidance.py time their pivots (and
        MotorController.turn_left/turn_right use the same fast turn-ramp
        those were calibrated against, so the timing carries over).
        """
        turn = self.motor_controller.turn_left if direction == "left" else self.motor_controller.turn_right
        turn(speed, duration=total_duration)  # blocks for the turn, then stops itself

        # Robot B's gyro stays accurate while its motors run (integrated on
        # the ESP32 itself), so it reports the real turn instead of an estimate
        trust_gyro = getattr(self.imu, "reliable_during_motion", False)
        if trust_gyro:
            self.imu.update_heading()
            self.heading = self.imu.get_heading()
        else:
            estimated_degrees = self.turn_rate_deg_per_s * total_duration
            sign = 1 if direction == "left" else -1
            self.heading = (self.heading + sign * estimated_degrees) % 360

        # Record this turn for the map's direction-change labels. Position
        # doesn't change during a pivot turn, so consecutive nudges in the
        # same direction at the same spot (e.g. several heading-correction
        # nudges in a row while converging on a target heading) collapse
        # into a single "L"/"R" marker instead of stacking duplicates.
        turn_key = (round(self.position[0], 2), round(self.position[1], 2), direction)
        if turn_key != self._last_turn_key:
            letter = "L" if direction == "left" else "R"
            self.direction_changes.append((self.position[0], self.position[1], letter))
            self._last_turn_key = turn_key

        # Sync the IMU driver's own internal heading/clock to this
        # estimate - _update_sensors() reads self.imu.get_heading() every
        # iteration, so without this the IMU's stale internal value
        # (never updated during the turn we just skipped reading it for)
        # would silently overwrite the estimate we just computed here.
        if not trust_gyro:
            self.imu.reset_heading(self.heading)

    def _move_towards(self, target_x: float, target_y: float, speed: int = 150):
        """Move robot towards target coordinates."""
        # Calculate direction to target
        dx = target_x - self.position[0]
        dy = target_y - self.position[1]
        
        if np.sqrt(dx**2 + dy**2) < 0.1:
            # Reached target
            self.motor_controller.stop()
            return
        
        # Calculate target heading
        target_heading = np.degrees(np.arctan2(dy, dx))
        
        # Simple heading correction
        heading_diff = (target_heading - self.heading) % 360
        if heading_diff > 180:
            heading_diff -= 360

        # Differential drive control
        if abs(heading_diff) > 5:
            # Turn towards target - a short nudge, not a continuous spin,
            # so the next loop iteration can re-check heading before
            # overshooting the target
            if heading_diff > 0:
                self._turn_nudge("left", speed, HEADING_TURN_NUDGE_S)
            else:
                self._turn_nudge("right", speed, HEADING_TURN_NUDGE_S)
        else:
            # Move forward in a short bounded burst, not continuously -
            # re-checks heading/obstacles/frontiers every FORWARD_STEP_S
            # instead of committing to a long straight-line run.
            self.motor_controller.move_forward(speed, duration=FORWARD_STEP_S)

            # Update position (simple integration - rough estimate, no
            # encoders). Uses the calibrated per-burst distance directly
            # rather than a rate x duration formula - see
            # FORWARD_STEP_DISTANCE_M's definition for why.
            distance_moved = self.forward_step_distance_m
            self.position[0] += distance_moved * np.cos(np.radians(self.heading))
            self.position[1] += distance_moved * np.sin(np.radians(self.heading))
            self.total_distance += distance_moved
    
    def _log_status(self):
        """Log robot status."""
        elapsed = time.time() - self.start_time
        coverage = self.occupancy_grid.get_coverage_percentage()
        
        logger.info(
            f"[{elapsed:.1f}s] Pos: ({self.position[0]:.2f}, {self.position[1]:.2f}), "
            f"Heading: {self.heading:.1f}°, "
            f"Scans: {self.scans_processed}, "
            f"Coverage: {coverage:.1f}%, "
            f"Distance: {self.total_distance:.2f}m"
        )
    
    def stop(self):
        """Stop robot operation."""
        if self._stopped:
            return
        self._stopped = True

        logger.info("Stopping robot...")
        self.running = False

        # Stop motor
        self.motor_controller.stop()

        # Save map, including the breadcrumb trail + final heading so the
        # rendered map can show the path/direction the robot took
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        map_file = f"maps/map_{timestamp}.json"
        trajectory = self.visited_positions + [tuple(self.position)]
        self.occupancy_grid.save_to_file(
            map_file,
            trajectory=trajectory,
            heading=self.heading,
            direction_changes=self.direction_changes,
            objects=self.object_map.to_list()
        )
        logger.info(f"Map saved to {map_file}")
        
        # Cleanup
        self._cleanup()
    
    def _cleanup(self):
        """Clean up all resources."""
        logger.info("Cleaning up resources...")
        
        if self.motor_controller:
            self.motor_controller.cleanup()
        if self.imu:
            self.imu.cleanup()
        if self.ultrasonic:
            self.ultrasonic.cleanup()
        if self.team:
            self.team.disconnect()
        if self.detector:
            self.detector.cleanup()

        logger.info("Cleanup complete")


def main():
    """Main entry point."""
    robot = None

    def handle_termination(signum, frame):
        """Catch SIGTERM/SIGHUP (e.g. a dropped SSH connection) the same
        way KeyboardInterrupt is caught below, so the robot always stops
        and saves its map instead of being left running/driving blind."""
        if robot:
            robot.stop()
        sys.exit(0)

    parser = argparse.ArgumentParser(description="Run exploration on one robot")
    parser.add_argument("--robot", choices=["a", "b"], default="a",
                        help="a = Raspberry Pi robot (run on the Pi), b = ESP32 robot over MQTT (run on the laptop)")
    parser.add_argument("--broker", default="auto",
                        help="MQTT broker host = the laptop running mosquitto; "
                             "'auto' tries this machine, then scans the local network")
    parser.add_argument("--no-vision", action="store_true",
                        help="disable camera object recognition (Robot A)")
    parser.add_argument("--minutes", type=float, default=0,
                        help="stop and save the map after this many minutes (default: run until Ctrl+C)")
    parser.add_argument("--planner", choices=PLANNERS, default="astar",
                        help="route planning to each frontier (default astar; simple = straight line)")
    args = parser.parse_args()
    if args.broker == "auto":
        args.broker = find_broker() or "localhost"  # not found: Robot A explores solo

    signal.signal(signal.SIGTERM, handle_termination)
    if hasattr(signal, "SIGHUP"):  # not available on Windows
        signal.signal(signal.SIGHUP, handle_termination)

    try:
        logger.info("Multi-Robot Exploration and Mapping - Phase 1")
        logger.info("=" * 50)

        robot = RobotController(robot=args.robot, broker=args.broker, vision=not args.no_vision,
                                run_minutes=args.minutes, planner=args.planner)
        robot.start()

    except KeyboardInterrupt:
        logger.info("\nInterrupted by user")

    except Exception as e:
        logger.error(f"Fatal error: {e}")

    finally:
        if robot:
            robot.stop()

        logger.info("Robot shutdown complete")


if __name__ == "__main__":
    main()
