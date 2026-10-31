#!/usr/bin/env python3
"""
Merge Robot A's and Robot B's saved maps into one shared team map.

Both robots map in the same world frame (origin = Robot A's start), so the
grids line up cell for cell. A cell is unknown only if neither robot saw it,
and occupied if either robot saw an obstacle there. Both paths and the
recognised objects (repeat sightings combined) are drawn on top.

run_team.py calls this automatically after a run. To merge the newest pair
by hand (from the project folder):
    python merge_maps.py
    python merge_maps.py maps/A_map_X.json maps/map_Y.json
"""

import json
import os
import sys
from datetime import datetime

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
MAPS_DIR = os.path.join(PROJECT_DIR, "maps")
ROBOT_COLORS = {"A": "dodgerblue", "B": "crimson"}
START_POSITIONS = {"A": (0.0, 0.0), "B": (0.0, -0.5)}  # same as main.py's ROBOT_B_START_POSITION
OBJECT_MERGE_RADIUS_M = 0.4


def merge_objects(object_lists):
    """Combine sightings of the same label within OBJECT_MERGE_RADIUS_M."""
    merged = []
    for obj in (o for objs in object_lists for o in (objs or [])):
        for m in merged:
            if m["label"] == obj["label"] and np.hypot(m["x"] - obj["x"], m["y"] - obj["y"]) < OBJECT_MERGE_RADIUS_M:
                m["count"] += obj["count"]
                m["confidence"] = max(m["confidence"], obj["confidence"])
                break
        else:
            merged.append(dict(obj))
    return merged


def merge(a_json, b_json, out_json):
    """Write the shared map JSON and return its data."""
    with open(a_json) as f:
        a = json.load(f)
    with open(b_json) as f:
        b = json.load(f)
    for key in ("width", "height", "resolution", "origin_x", "origin_y"):
        if a[key] != b[key]:
            raise ValueError(f"Maps don't line up: {key} is {a[key]} vs {b[key]}")

    # -1 = unknown, 0..100 = occupancy %: the max keeps any known value and lets obstacles win
    grid = np.maximum(np.array(a["grid"], dtype=np.int16), np.array(b["grid"], dtype=np.int16))
    shared = {k: a[k] for k in ("width", "height", "resolution", "origin_x", "origin_y")}
    shared["grid"] = grid.tolist()
    shared["robots"] = {
        name: {"source": os.path.basename(path), "trajectory": d.get("trajectory"),
               "final_heading": d.get("final_heading"), "direction_changes": d.get("direction_changes")}
        for name, path, d in (("A", a_json, a), ("B", b_json, b))
    }
    shared["objects"] = merge_objects([a.get("objects"), b.get("objects")])
    with open(out_json, "w") as f:
        json.dump(shared, f)
    return shared


def render(data, png_path):
    grid = np.array(data["grid"], dtype=np.int16)
    res, ox, oy = data["resolution"], data["origin_x"], data["origin_y"]
    known = grid != -1
    if not known.any():
        print("No explored cells - nothing to render.")
        return
    print(f"Shared map explored cells: {int(known.sum())} / {grid.size}")

    ys, xs = np.where(known)
    pad = 10
    x0, x1 = max(0, xs.min() - pad), min(data["width"], xs.max() + pad + 1)
    y0, y1 = max(0, ys.min() - pad), min(data["height"], ys.max() + pad + 1)
    cropped = grid[y0:y1, x0:x1]
    display = np.zeros_like(cropped, dtype=np.int8)
    display[(cropped >= 0) & (cropped <= 50)] = 1
    display[cropped > 50] = 2
    cmap = ListedColormap(["#c8c8c8", "#ffffff", "#1a1a1a"])
    norm = BoundaryNorm([-0.5, 0.5, 1.5, 2.5], cmap.N)
    wx0, wx1, wy0, wy1 = ox + x0 * res, ox + x1 * res, oy + y0 * res, oy + y1 * res

    fig, ax = plt.subplots(figsize=(8, 8), dpi=150)
    ax.imshow(display, cmap=cmap, norm=norm, origin="lower", extent=[wx0, wx1, wy0, wy1])
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.set_title("Shared Team Map (Robot A + Robot B)")
    arrow_len = max(wx1 - wx0, wy1 - wy0) * 0.06

    for name, robot in data["robots"].items():
        color = ROBOT_COLORS[name]
        start = START_POSITIONS[name]
        ax.plot(*start, marker="*", color=color, markeredgecolor="black",
                markersize=15, linestyle="none", label=f"Robot {name} start", zorder=5)
        # The saved trajectory can miss the first moves, so draw it from the start pose
        traj = [start] + (robot.get("trajectory") or [])
        tx, ty = [p[0] for p in traj], [p[1] for p in traj]
        ax.plot(tx, ty, color=color, linewidth=2, marker="o", markersize=3,
                label=f"Robot {name} path", zorder=4)
        if robot.get("final_heading") is not None:
            h = np.radians(robot["final_heading"])
            ax.annotate("", xy=(tx[-1] + arrow_len * np.cos(h), ty[-1] + arrow_len * np.sin(h)),
                        xytext=(tx[-1], ty[-1]),
                        arrowprops=dict(facecolor=color, edgecolor=color, width=3, headwidth=10), zorder=6)
        for cx, cy, letter in robot.get("direction_changes") or []:
            ax.annotate(letter, xy=(cx, cy), fontsize=7, fontweight="bold", color="white",
                        ha="center", va="center", zorder=7,
                        bbox=dict(boxstyle="circle,pad=0.2", facecolor=color, edgecolor="none"))

    for obj in data["objects"]:
        ax.plot(obj["x"], obj["y"], marker="D", color="purple", markersize=7, zorder=8)
        ax.annotate(f'{obj["label"]} x{obj["count"]}', xy=(obj["x"], obj["y"]),
                    xytext=(5, 5), textcoords="offset points", fontsize=8, color="purple", zorder=9,
                    bbox=dict(boxstyle="round,pad=0.2", facecolor="white", edgecolor="purple", alpha=0.85))
    if data["objects"]:
        ax.plot([], [], marker="D", color="purple", linestyle="none", label="Recognised object")

    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.2)
    fig.tight_layout()
    fig.savefig(png_path)
    plt.close(fig)


def merge_and_render(a_json, b_json):
    """Merge the two maps into maps/shared_map_<time>.json/.png; returns the PNG path."""
    out_json = os.path.join(MAPS_DIR, f"shared_map_{datetime.now():%Y%m%d_%H%M%S}.json")
    png = out_json[:-5] + ".png"
    render(merge(a_json, b_json, out_json), png)
    return png


def newest(prefix):
    files = [os.path.join(MAPS_DIR, f) for f in os.listdir(MAPS_DIR)
             if f.startswith(prefix) and f.endswith(".json")]
    if not files:
        sys.exit(f"No {prefix}*.json in maps/")
    return max(files, key=os.path.getmtime)


if __name__ == "__main__":
    if len(sys.argv) == 3:
        a_path, b_path = sys.argv[1], sys.argv[2]
    else:
        a_path, b_path = newest("A_map_"), newest("map_")
    print(f"Robot A: {os.path.basename(a_path)}\nRobot B: {os.path.basename(b_path)}")
    print(f"✓ Shared map image: {os.path.relpath(merge_and_render(a_path, b_path), PROJECT_DIR)}")
