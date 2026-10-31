"""
Object recognition for Robot A (Raspberry Pi 5 + OV5647 camera)

A background thread captures camera frames and runs YOLOv8n (COCO, 80
everyday object classes) through OpenCV's DNN module - no PyTorch needed
on the Pi. The model is models/yolov8n.onnx, exported at 320x320 input
(~37 ms per frame on a Pi 5 CPU).

Each detection carries a bearing (degrees, + = left of the camera's
centre line, matching the robot frame where 90 = left), which main.py
combines with the front ultrasonic range to place the object on the map.

The annotated frames are also served as a live view:
    http://<pi-ip>:8080/         page with the stream
    http://<pi-ip>:8080/stream   MJPEG stream (boxes + labels)
"""

import logging
import threading
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import List, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)

COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog",
    "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
    "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
    "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
    "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
    "teddy bear", "hair drier", "toothbrush",
]

FRAME_SIZE = (640, 480)      # camera capture size (full sensor field of view on the OV5647)
MODEL_INPUT = 320            # must match the ONNX export size
CAMERA_HFOV_DEG = 53.5       # OV5647 horizontal field of view
CONF_THRESHOLD = 0.35       # shown in the live view; main.py maps only stronger ones
NMS_THRESHOLD = 0.45
# ~3 detections/s. Only frames taken while the robot stands still are used,
# and at ~8/s the extra CPU load helped brown out the Pi 5 on a power bank
# (4.62 V, throttled=0x50005, then a reset ~90 s into a run).
DETECT_INTERVAL_S = 0.33
# Power: the Pi's 5 V input dipped to 4.70 V (under-voltage) with YOLO on all
# 4 cores at full speed. 2 threads halves the current spikes and still fits
# well inside DETECT_INTERVAL_S; the camera only needs a little more than the
# detection rate (its default ~30 fps kept the ISP busy for nothing).
DNN_THREADS = 2
CAMERA_FPS = 5

# Only object types that can plausibly be indoors; the rest of COCO
# (vehicles, large animals, ...) only ever appeared as false positives,
# e.g. a patterned sofa reported as "train".
INDOOR_CLASSES = {
    "person", "cat", "dog", "bird", "backpack", "umbrella", "handbag", "suitcase", "sports ball",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "orange",
    "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator",
    "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush", "bicycle",
}
STREAM_PORT = 8080
# Robot A's camera is mounted upside down. YOLO misses upside-down objects
# (a test frame with chairs, a bottle and a person gave no detections) and
# left/right would be swapped on the map, so flip it in the sensor (free).
CAMERA_ROTATE_180 = True


@dataclass
class Detection:
    label: str
    confidence: float
    box: Tuple[int, int, int, int]  # x, y, w, h in FRAME_SIZE pixels
    bearing_deg: float              # + = left of centre
    frame_time: float               # time.monotonic() when the frame was captured


class ObjectDetector:
    """Camera + YOLO in a background thread, plus an MJPEG live view."""

    def __init__(self, model_path: str = "models/yolov8n.onnx", stream_port: Optional[int] = STREAM_PORT,
                 on_detections=None):
        """on_detections: optional callback(list of Detection) after every processed frame."""
        import cv2
        from libcamera import Transform
        from picamera2 import Picamera2

        self._cv2 = cv2
        cv2.setNumThreads(DNN_THREADS)
        self.net = cv2.dnn.readNetFromONNX(model_path)
        self.camera = Picamera2()
        # "RGB888" in Picamera2 is BGR byte order - what OpenCV expects
        self.camera.configure(self.camera.create_preview_configuration(
            main={"size": FRAME_SIZE, "format": "RGB888"},
            transform=Transform(hflip=CAMERA_ROTATE_180, vflip=CAMERA_ROTATE_180),
            controls={"FrameDurationLimits": (1_000_000 // CAMERA_FPS,) * 2}))

        self._lock = threading.Condition()
        self._detections: List[Detection] = []
        self._frame_time = 0.0
        self._jpeg: Optional[bytes] = None
        self._viewers = 0  # open live-view streams; frames are only drawn and encoded for them
        self._running = False
        self._thread = None
        self._server = None
        self.stream_port = stream_port
        self.on_detections = on_detections

    def start(self):
        self.camera.start()
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="object-detector", daemon=True)
        self._thread.start()
        if self.stream_port:
            self._start_stream_server()
        logger.info("Object detector started")

    # ---------- detection ----------
    def _loop(self):
        cv2 = self._cv2
        while self._running:
            t0 = time.monotonic()
            frame = self.camera.capture_array()
            frame_time = time.monotonic()
            detections = self._detect(frame, frame_time)

            ok = False
            if self._viewers:
                annotated = frame.copy()
                for d in detections:
                    x, y, w, h = d.box
                    cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(annotated, f"{d.label} {d.confidence:.2f}", (x, max(12, y - 5)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
                ok, jpeg = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 70])

            with self._lock:
                self._detections = detections
                self._frame_time = frame_time
                if ok:
                    self._jpeg = jpeg.tobytes()
                self._lock.notify_all()
            if self.on_detections:
                try:
                    self.on_detections(detections)
                except Exception as e:  # a reporting hiccup must not stop detection
                    logger.debug(f"on_detections failed: {e}")

            time.sleep(max(0.0, DETECT_INTERVAL_S - (time.monotonic() - t0)))

    def _detect(self, frame: np.ndarray, frame_time: float) -> List[Detection]:
        cv2 = self._cv2
        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (MODEL_INPUT, MODEL_INPUT), swapRB=True)
        self.net.setInput(blob)
        preds = self.net.forward()[0].T  # (2100, 84): cx, cy, w, h, 80 class scores

        scores = preds[:, 4:]
        class_ids = scores.argmax(axis=1)
        confidences = scores[np.arange(len(scores)), class_ids]
        keep = confidences >= CONF_THRESHOLD
        if not keep.any():
            return []
        preds, class_ids, confidences = preds[keep], class_ids[keep], confidences[keep]

        sx, sy = w / MODEL_INPUT, h / MODEL_INPUT
        boxes = [
            [int((cx - bw / 2) * sx), int((cy - bh / 2) * sy), int(bw * sx), int(bh * sy)]
            for cx, cy, bw, bh in preds[:, :4]
        ]
        idx = cv2.dnn.NMSBoxes(boxes, confidences.tolist(), CONF_THRESHOLD, NMS_THRESHOLD)

        detections = []
        for i in np.array(idx).flatten():
            if COCO_CLASSES[class_ids[i]] not in INDOOR_CLASSES:
                continue
            x, y, bw, bh = boxes[i]
            centre_x = x + bw / 2
            bearing = (0.5 - centre_x / w) * CAMERA_HFOV_DEG
            detections.append(Detection(COCO_CLASSES[class_ids[i]], float(confidences[i]),
                                        (x, y, bw, bh), bearing, frame_time))
        return detections

    def get_detections(self, after: float, timeout: float = 0.3) -> List[Detection]:
        """
        Detections from the newest frame captured after `after` (time.monotonic()),
        waiting up to `timeout` for one. Used to skip frames taken while moving.
        """
        with self._lock:
            if not self._lock.wait_for(lambda: self._frame_time > after, timeout=timeout):
                return []
            return list(self._detections)

    # ---------- live view ----------
    def _start_stream_server(self):
        detector = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                if self.path == "/":
                    body = (b"<!doctype html><html><head><title>Robot A camera</title>"
                            b"<meta name=viewport content='width=device-width'></head>"
                            b"<body style='margin:0;background:#111'>"
                            b"<img src='/stream' style='width:100%;max-width:800px;display:block;margin:auto'>"
                            b"</body></html>")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                elif self.path == "/stream":
                    self.send_response(200)
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.end_headers()
                    last = 0.0
                    detector._viewers += 1
                    try:
                        while detector._running:
                            with detector._lock:
                                detector._lock.wait_for(lambda: detector._frame_time > last, timeout=2)
                                last, jpeg = detector._frame_time, detector._jpeg
                            if jpeg:
                                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                                 + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
                    except (BrokenPipeError, ConnectionResetError):
                        pass  # viewer closed the page
                    finally:
                        detector._viewers -= 1
                else:
                    self.send_error(404)

        self._server = ThreadingHTTPServer(("0.0.0.0", self.stream_port), Handler)
        self._server.daemon_threads = True
        threading.Thread(target=self._server.serve_forever, name="camera-stream", daemon=True).start()
        logger.info(f"Live view on http://<this-pi>:{self.stream_port}/")

    def cleanup(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        if self._server:
            self._server.shutdown()
        self.camera.stop()
        self.camera.close()


class ObjectMap:
    """Objects placed in world coordinates; repeat sightings are merged."""

    def __init__(self, merge_radius_m: float = 0.4):
        self.merge_radius_m = merge_radius_m
        self.objects = []  # dicts: label, x, y, confidence, count, source

    def add(self, label: str, x: float, y: float, confidence: float, source: str) -> Optional[dict]:
        """Add a sighting. Returns the object if it is new, None if merged into an existing one."""
        for obj in self.objects:
            if obj["label"] == label and np.hypot(obj["x"] - x, obj["y"] - y) < self.merge_radius_m:
                n = obj["count"]
                obj["x"] = (obj["x"] * n + x) / (n + 1)  # running average of sightings
                obj["y"] = (obj["y"] * n + y) / (n + 1)
                obj["count"] = n + 1
                obj["confidence"] = max(obj["confidence"], confidence)
                return None
        obj = {"label": label, "x": x, "y": y, "confidence": confidence, "count": 1, "source": source}
        self.objects.append(obj)
        return obj

    def to_list(self) -> list:
        return [dict(o, x=round(o["x"], 3), y=round(o["y"], 3), confidence=round(o["confidence"], 2))
                for o in self.objects]
