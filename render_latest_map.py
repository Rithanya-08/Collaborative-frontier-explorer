#!/usr/bin/env python3
"""
Render Latest Map

Runs on your Windows machine (NOT on the Pi) - pulls the newest
maps/map_*.json saved by main.py's most recent run over SSH, then
renders it to a PNG in the local maps/ folder, matching the earlier
map_20260904_225347.png style (gray=unexplored, white=free, black=
occupied, red star=robot start).

Usage (from a Windows terminal, after running main.py on the Pi):
    py render_latest_map.py

Requires pscp/plink (PuTTY) on PATH, and numpy/matplotlib installed
locally (already in requirements.txt).
"""

import json
import os
import subprocess
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm

PI_HOST = "pi@10.197.110.137"
PI_PASSWORD = "1234"
PI_MAPS_DIR = "~/Project/maps"

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
LOCAL_MAPS_DIR = os.path.join(PROJECT_DIR, "maps")


def latest_remote_map() -> str:
    """Return the filename of the newest map JSON on the Pi."""
    result = subprocess.run(
        ["plink", "-ssh", "-pw", PI_PASSWORD, PI_HOST,
         f"ls -t {PI_MAPS_DIR}/*.json 2>/dev/null | head -1"],
        capture_output=True, text=True, check=True,
    )
    remote_path = result.stdout.strip()
    if not remote_path:
        sys.exit("No map files found on the Pi yet - run main.py first.")
    return remote_path


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
    objects = data.get("objects")

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
    display[cropped == -1] = 0             # unknown
    display[(cropped >= 0) & (cropped <= 50)] = 1   # free
    display[cropped > 50] = 2              # occupied

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

    if objects:
        # Recognised objects (camera + YOLO): purple diamond + label
        for obj in objects:
            ax.plot(obj["x"], obj["y"], marker="D", color="purple", markersize=7, zorder=8)
            ax.annotate(
                f'{obj["label"]} x{obj["count"]}', xy=(obj["x"], obj["y"]),
                xytext=(5, 5), textcoords="offset points", fontsize=8, color="purple",
                bbox=dict(boxstyle="round,pad=0.2", facecolor="white", edgecolor="purple", alpha=0.85),
                zorder=9,
            )
        ax.plot([], [], marker="D", color="purple", linestyle="none", label="Recognised object")

    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.2)
    fig.tight_layout()
    fig.savefig(png_path)
    print(f"Saved {png_path}")


def main():
    os.makedirs(LOCAL_MAPS_DIR, exist_ok=True)

    print("Checking Pi for the latest map...")
    remote_path = latest_remote_map()
    basename = os.path.basename(remote_path)
    local_json = os.path.join(LOCAL_MAPS_DIR, basename)

    print(f"Fetching {basename}...")
    fetch(remote_path, local_json)

    png_path = local_json[:-5] + ".png"  # strip ".json"
    render(local_json, png_path)


if __name__ == "__main__":
    main()
