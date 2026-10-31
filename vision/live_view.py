#!/usr/bin/env python3
"""
Live view of Robot A's object recognition, without driving the robot.

Starts the camera + YOLO detector and serves the annotated stream on
http://<pi-ip>:8080/ ; detections are also printed as they change.

Usage (on the Pi, from ~/robot):
    python3 -m vision.live_view            # until Ctrl+C
    python3 -m vision.live_view --minutes 30
"""

import argparse
import logging
import time

from vision.object_detector import ObjectDetector, STREAM_PORT


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument("--minutes", type=float, default=0, help="stop after this long (0 = until Ctrl+C)")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S")

    detector = ObjectDetector()
    detector.start()
    print(f"Live view: http://<this-pi-ip>:{STREAM_PORT}/  (Ctrl+C to stop)", flush=True)
    end = time.monotonic() + args.minutes * 60 if args.minutes else None
    last_frame, last_text = 0.0, None
    try:
        while end is None or time.monotonic() < end:
            detections = detector.get_detections(after=last_frame, timeout=1.0)
            last_frame = time.monotonic()
            text = ", ".join(f"{d.label} {d.confidence:.2f}" for d in detections) or "(nothing)"
            if text != last_text:
                logging.info(text)
                last_text = text
    except KeyboardInterrupt:
        pass
    finally:
        detector.cleanup()


if __name__ == "__main__":
    main()
