#!/usr/bin/env python3
"""
Run main.py on the Pi, then automatically render the map it produces.

Runs on your Windows machine (NOT on the Pi). Launches main.py on the Pi
over SSH - you'll see its live output right here, and Ctrl+C stops it
exactly like it would in a direct SSH session. Once it exits, this
automatically fetches whichever map it just saved and renders the PNG
into the local maps/ folder - no separate render_latest_map.py step.

Usage (from a Windows terminal):
    py run_main_and_map.py

Requires pscp/plink (PuTTY) on PATH, and numpy/matplotlib installed
locally (already in requirements.txt).
"""

import json
import os
import subprocess
import sys
import time

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm

PI_HOST = "pi@10.197.110.137"
PI_PASSWORD = "1234"
PI_PROJECT_DIR = "~/Project"
PI_MAPS_DIR = f"{PI_PROJECT_DIR}/maps"

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
LOCAL_MAPS_DIR = os.path.join(PROJECT_DIR, "maps")

POLL_TIMEOUT_S = 15   # how long to wait for main.py's SIGHUP cleanup to
                       # finish writing its map file after the connection
                       # closes (e.g. from Ctrl+C)
POLL_INTERVAL_S = 1.0


def run_plink(remote_command: str) -> str:
    result = subprocess.run(
        ["plink", "-ssh", "-pw", PI_PASSWORD, PI_HOST, remote_command],
        capture_output=True, text=True, check=True,
    )
    return result.stdout.strip()


def latest_remote_map() -> str:
    """Newest map filename on the Pi, or '' if none exist yet."""
    return run_plink(f"ls -t {PI_MAPS_DIR}/*.json 2>/dev/null | head -1")


def stop_remote_main():
    """
    Explicitly SIGTERM any running main.py on the Pi.

    Relying on the SSH disconnect alone to propagate a signal to the
    remote process turned out to be unreliable in practice - Ctrl+C could
    kill this local connection while main.py kept running (and driving
    the motors) on the Pi indefinitely, orphaned. This is a guaranteed
    backstop instead: SIGTERM still lets main.py's own handler run its
    normal cleanup (stop motors, save the map) rather than a hard kill.
    """
    subprocess.run(
        ["plink", "-ssh", "-pw", PI_PASSWORD, PI_HOST,
         "pkill -TERM -f 'python3 main.py' 2>/dev/null; true"],
        capture_output=True, text=True,
    )


def run_main_interactively(before_latest: str):
    """
    Launch main.py on the Pi with this process's stdin/stdout/stderr, so
    it behaves like a direct SSH session - live output, Ctrl+C forwarded.
    """
    print(f"Launching main.py on {PI_HOST} - Ctrl+C to stop it.\n")
    try:
        subprocess.run(
            ["plink", "-ssh", "-pw", PI_PASSWORD, PI_HOST,
             f"cd {PI_PROJECT_DIR} && python3 main.py"],
        )
    except KeyboardInterrupt:
        # Windows delivers Ctrl+C to this parent process too, not just the
        # plink child - without catching it here, the interrupt would
        # crash this script before it ever gets to fetch/render the map
        # main.py just saved.
        print("\n(Ctrl+C received.)")

    # Don't assume the connection dropping already stopped main.py on the
    # Pi - confirmed unreliable (it was found still running, orphaned,
    # minutes later). Always send an explicit stop.
    print("Confirming main.py has actually stopped on the Pi...")
    stop_remote_main()

    # Give its normal cleanup (stop motors, save map) a moment to finish,
    # then poll for a genuinely new file rather than fetching immediately.
    print("\nWaiting for the Pi to finish saving the map...")
    deadline = time.time() + POLL_TIMEOUT_S
    while time.time() < deadline:
        latest = latest_remote_map()
        if latest and latest != before_latest:
            return latest
        time.sleep(POLL_INTERVAL_S)
    # Fall back to whatever's newest, even if we can't confirm it's new
    return latest_remote_map()


def fetch(remote_path: str, local_path: str):
    subprocess.run(
        ["pscp", "-pw", PI_PASSWORD, f"{PI_HOST}:{remote_path}", local_path],
        check=True,
    )


def render(json_path: str, png_path: str):
    with open(json_path) as f:
        data = json.load(f)

    grid = np.array(data["grid"], dtype=np.int16)
    resolution = data["resolution"]
    origin_x = data["origin_x"]
    origin_y = data["origin_y"]
    width = data["width"]
    height = data["height"]
    trajectory = data.get("trajectory")
    final_heading = data.get("final_heading")
    direction_changes = data.get("direction_changes")

    explored_mask = grid != -1
    explored_count = int(explored_mask.sum())
    print(f"Explored cells: {explored_count} / {grid.size} "
          f"({100 * explored_count / grid.size:.2f}%)")

    if explored_count == 0:
        print("No explored cells - nothing to render.")
        return

    ys, xs = np.where(explored_mask)
    pad = 10
    x0, x1 = max(0, xs.min() - pad), min(width, xs.max() + pad + 1)
    y0, y1 = max(0, ys.min() - pad), min(height, ys.max() + pad + 1)
    cropped = grid[y0:y1, x0:x1]

    display = np.zeros_like(cropped, dtype=np.int8)
    display[cropped == -1] = 0
    display[(cropped >= 0) & (cropped <= 50)] = 1
    display[cropped > 50] = 2

    cmap = ListedColormap(["#c8c8c8", "#ffffff", "#1a1a1a"])
    norm = BoundaryNorm([-0.5, 0.5, 1.5, 2.5], cmap.N)

    world_x0 = origin_x + x0 * resolution
    world_x1 = origin_x + x1 * resolution
    world_y0 = origin_y + y0 * resolution
    world_y1 = origin_y + y1 * resolution

    fig, ax = plt.subplots(figsize=(8, 8), dpi=150)
    ax.imshow(display, cmap=cmap, norm=norm, origin="lower",
              extent=[world_x0, world_x1, world_y0, world_y1])
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.set_title("Occupancy Grid Map (robot start at 0,0)")
    ax.plot(0, 0, marker="*", color="red", markersize=15, label="Start (0,0)", zorder=5)

    if trajectory:
        traj_x = [p[0] for p in trajectory]
        traj_y = [p[1] for p in trajectory]
        ax.plot(traj_x, traj_y, color="dodgerblue", linewidth=2,
                marker="o", markersize=3, label="Path", zorder=4)

        if final_heading is not None:
            arrow_len = max(world_x1 - world_x0, world_y1 - world_y0) * 0.06
            hx = arrow_len * np.cos(np.radians(final_heading))
            hy = arrow_len * np.sin(np.radians(final_heading))
            ax.annotate(
                "", xy=(traj_x[-1] + hx, traj_y[-1] + hy), xytext=(traj_x[-1], traj_y[-1]),
                arrowprops=dict(facecolor="orange", edgecolor="orange", width=3, headwidth=10),
                zorder=6,
            )
            ax.plot([], [], color="orange", linewidth=3, label="Final heading")

    if direction_changes:
        for tx, ty, letter in direction_changes:
            ax.annotate(
                letter, xy=(tx, ty), fontsize=8, fontweight="bold",
                color="white", ha="center", va="center",
                bbox=dict(boxstyle="circle,pad=0.25",
                          facecolor="seagreen" if letter == "L" else "darkorange",
                          edgecolor="none"),
                zorder=7,
            )
        ax.plot([], [], marker="o", markersize=8, linestyle="none",
                color="seagreen", label="L = left turn")
        ax.plot([], [], marker="o", markersize=8, linestyle="none",
                color="darkorange", label="R = right turn")

    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.2)
    fig.tight_layout()
    fig.savefig(png_path)
    print(f"Saved {png_path}")


def main():
    os.makedirs(LOCAL_MAPS_DIR, exist_ok=True)

    before_latest = latest_remote_map()
    remote_path = run_main_interactively(before_latest)

    if not remote_path:
        sys.exit("\nNo map file found on the Pi - main.py may not have saved one.")

    basename = os.path.basename(remote_path)
    local_json = os.path.join(LOCAL_MAPS_DIR, basename)

    print(f"Fetching {basename}...")
    fetch(remote_path, local_json)

    png_path = local_json[:-5] + ".png"
    render(local_json, png_path)


if __name__ == "__main__":
    main()
