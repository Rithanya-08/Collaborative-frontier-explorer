"""
Live Team Dashboard in the terminal

The same live picture as the browser dashboard (app/web_server.py), drawn in
the command window: each robot's status, position, heading, ultrasonic
distances (red when close enough to trigger obstacle avoidance) and target,
plus a text version of the shared map. run_team.py shows it during a run,
with the robots' log lines scrolling above it.

Standalone, while the robots run (from the project root):
    python -m app.terminal_dashboard
"""

import logging
import time

import numpy as np
from rich.console import Group
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

from app.web_server import TeamMonitor
from main import OBSTACLE_THRESHOLD_M, SIDE_OBSTACLE_THRESHOLD_M

# The monitor's own connection messages would break up the live view
logging.getLogger("app.web_server").setLevel(logging.WARNING)

ROBOT_STYLES = {"A": "bold dodger_blue1", "B": "bold red3"}
STATUS_STYLES = {"exploring": "green", "connected": "yellow", "standby": "yellow", "offline": "red"}
MAP_MAX_COLS = 72
OBSTACLE_SHARE = 0.25  # share of a map block's seen cells that must be obstacles to draw it as one


def _distance(value, threshold):
    if value is None:
        return Text("—", style="dim")
    return Text(f"{value:.2f}", style="bold red" if value < threshold else "")


def robot_table(snap: dict) -> Table:
    table = Table(expand=False, header_style="dim", pad_edge=False, box=None, padding=(0, 2))
    for column in ("Robot", "Status", "Position (m)", "Heading", "Front", "Left", "Right", "Target", "Scans"):
        table.add_column(column, justify="right" if column in ("Heading", "Front", "Left", "Right", "Scans") else "left")
    for rid, robot in snap["robots"].items():
        pose, ranges, target = robot["pose"], robot["ranges"] or {}, robot["target"]
        table.add_row(
            Text(f"Robot {rid}", style=ROBOT_STYLES[rid]),
            Text(robot["status"], style=STATUS_STYLES.get(robot["status"], "")),
            f"({pose[0]:.2f}, {pose[1]:.2f})" if pose else "—",
            f"{pose[2] % 360:.0f}°" if pose else "—",
            _distance(ranges.get("front"), OBSTACLE_THRESHOLD_M),
            _distance(ranges.get("left"), SIDE_OBSTACLE_THRESHOLD_M),
            _distance(ranges.get("right"), SIDE_OBSTACLE_THRESHOLD_M),
            f"({target[0]:.2f}, {target[1]:.2f})" if target else "—",
            str(robot["scans"]),
        )
    return table


def text_map(monitor: TeamMonitor, snap: dict, max_rows: int) -> Text:
    """
    The known part of the shared map, one character per block of cells
    (a character is about twice as tall as it is wide, so each row covers
    twice the distance of a column): █ obstacle, · free floor, blank unknown.
    """
    with monitor.lock:
        grid = monitor.grid.grid.copy()
        res, ox, oy = monitor.grid.resolution, monitor.grid.origin_x, monitor.grid.origin_y
    known = grid != -1
    if not known.any():
        return Text("No map yet - waiting for the robots' first scans", style="dim")

    ys, xs = np.where(known)
    x0, x1, y0, y1 = xs.min() - 2, xs.max() + 3, ys.min() - 2, ys.max() + 3
    cols_per_char = max(1, int(np.ceil((x1 - x0) / MAP_MAX_COLS)),
                        int(np.ceil((y1 - y0) / (2 * max_rows))))
    rows_per_char = 2 * cols_per_char
    n_cols = int(np.ceil((x1 - x0) / cols_per_char))
    n_rows = int(np.ceil((y1 - y0) / rows_per_char))

    chars = [[" "] * n_cols for _ in range(n_rows)]
    styles = [[""] * n_cols for _ in range(n_rows)]
    for r in range(n_rows):
        gy1 = y1 - r * rows_per_char          # top row of the text = highest y
        block_rows = grid[max(0, gy1 - rows_per_char):gy1]
        for c in range(n_cols):
            gx0 = x0 + c * cols_per_char
            block = block_rows[:, max(0, gx0):gx0 + cols_per_char]
            seen, occupied = int((block != -1).sum()), int((block > 50).sum())
            # Ultrasonic maps have stray obstacle dots in open floor; a block
            # only shows as a wall when a real share of what was seen is one
            if occupied and occupied >= OBSTACLE_SHARE * seen:
                chars[r][c] = "█"
            elif seen:
                chars[r][c], styles[r][c] = "·", "dim"

    def put(x, y, char, style):
        c = int(((x - ox) / res - x0) // cols_per_char)
        r = int((y1 - (y - oy) / res) // rows_per_char)
        if 0 <= r < n_rows and 0 <= c < n_cols:
            chars[r][c], styles[r][c] = char, style

    for obj in snap["objects"]:
        put(obj["x"], obj["y"], "◆", "bold magenta")
    for rid, robot in snap["robots"].items():
        if robot["target"]:
            put(*robot["target"], "+", ROBOT_STYLES[rid])
    for rid, robot in snap["robots"].items():
        if robot["pose"]:
            put(robot["pose"][0], robot["pose"][1], rid, "reverse " + ROBOT_STYLES[rid])

    text = Text()
    for r in range(n_rows):
        for char, style in zip(chars[r], styles[r]):
            text.append(char, style=style)
        if r < n_rows - 1:
            text.append("\n")
    return text


def render(monitor: TeamMonitor, run_seconds: float = 0, started: float = None, map_rows: int = 18,
           links=()):
    """links: browser dashboard addresses, shown clickable at the top."""
    snap = monitor.snapshot()
    link_line = Text()
    for i, url in enumerate(links):
        link_line.append("Browser dashboard: " if i == 0 else "   other devices: ", style="dim")
        link_line.append(url, style=f"bold cyan underline link {url}")
    header = Text()
    if started:
        elapsed = time.time() - started
        header.append(f"Run {int(elapsed // 60)}:{int(elapsed % 60):02d}", style="bold")
        if run_seconds:
            header.append(f" / {int(run_seconds // 60)}:{int(run_seconds % 60):02d}")
        header.append("   ")
    header.append(f"Mapped {snap['area_m2']:.1f} m²   ")
    if snap["objects"]:
        header.append("Objects: " + ", ".join(f"{o['label']} ×{o['count']}" for o in snap["objects"]) + "   ")
    if not snap["connected"]:
        header.append("Not connected to the MQTT broker", style="bold red")

    legend = Text.assemble(("A", "reverse " + ROBOT_STYLES["A"]), " ", ("B", "reverse " + ROBOT_STYLES["B"]),
                           " robots   + targets   ◆ objects   █ obstacle   · free   "
                           f"red distance = avoidance triggers (< {OBSTACLE_THRESHOLD_M:.2f} m front, "
                           f"< {SIDE_OBSTACLE_THRESHOLD_M:.2f} m sides)", style="dim")
    top = [link_line, header] if links else [header]
    return Panel(Group(*top, Text(), robot_table(snap), Text(), text_map(monitor, snap, map_rows), Text(), legend),
                 title="Team dashboard", title_align="left", border_style="grey50")


def main():
    monitor = TeamMonitor("localhost", client_id="terminal-dashboard")
    started = time.time()
    with Live(render(monitor, started=started), refresh_per_second=2, screen=False) as live:
        try:
            while True:
                time.sleep(0.5)
                live.update(render(monitor, started=started,
                                   map_rows=max(8, min(24, live.console.size.height - 16))))
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
