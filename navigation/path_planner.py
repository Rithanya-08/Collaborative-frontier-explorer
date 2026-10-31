"""
Path Planning Module

Plans collision-free routes through the occupancy grid from the robot to its
frontier target. main.py follows the waypoints instead of driving straight at
the target, so walls already on the map are routed around rather than found
again by the ultrasonic sensors.

Algorithms (plan_path(..., algorithm=...)):
- "astar":  A* over the grid cells, 8-connected - shortest route on the grid
- "rrt":    Rapidly-exploring Random Tree - grows a random tree from the
            start until a branch reaches the goal
- "prm":    Probabilistic Roadmap - random free points joined into a graph
            of straight, clear edges, then the shortest route over that graph
- "simple": straight line, no obstacle check (the original behaviour)

Shared by all three:
- occupied cells are grown by the robot's radius plus a margin, so a route
  never grazes a wall (the area right around the start and goal is exempt,
  otherwise a robot standing near a wall could never leave). Ultrasonic
  maps have stray obstacle dots in open floor that a full margin can seal
  off, so if no route exists the margin is halved, then dropped to the
  walls themselves - the widest clearance the map allows wins
- unknown cells are drivable - frontier targets sit at the edge of the
  unknown, and 3 ultrasonic rays leave many gaps; A* charges extra for them
- planning runs only inside the explored part of the map plus a margin,
  which keeps it fast on the Pi
- the route is shortcut to as few straight legs as the map allows
"""

import heapq
import logging
import time
from typing import List, Optional, Tuple

import numpy as np
from slam.occupancy_grid import OccupancyGrid

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

ALGORITHMS = ("astar", "rrt", "prm", "simple")

OCCUPIED_THRESHOLD = 50       # cell values above this are walls (see OccupancyGrid)
INFLATION_RADIUS_M = 0.20     # robot footprint radius 0.15 m + 5 cm margin
WINDOW_PAD_M = 1.0            # plan within the explored area plus this much
UNKNOWN_COST = 2.0            # A*: an unknown cell costs this much more than a free one

RRT_STEP_M = 0.15             # how far each new branch reaches
RRT_GOAL_BIAS = 0.1           # fraction of samples aimed straight at the goal
RRT_MAX_ITERATIONS = 3000

PRM_SAMPLES = 300             # random roadmap points
PRM_NEIGHBORS = 10            # each point tries to connect to this many nearest
PRM_CONNECT_RADIUS_M = 1.0    # ... if they are closer than this

Cell = Tuple[int, int]        # (grid_x, grid_y), window-local while planning

_NEIGHBORS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
              (1, 1, np.sqrt(2)), (1, -1, np.sqrt(2)), (-1, 1, np.sqrt(2)), (-1, -1, np.sqrt(2))]


class PathPlanner:
    """
    Plan collision-free paths in the occupancy grid.
    """

    def __init__(self, occupancy_grid: OccupancyGrid, inflation_radius_m: float = INFLATION_RADIUS_M,
                 seed: Optional[int] = None):
        """
        Initialize path planner.

        Args:
            occupancy_grid: Reference to the map
            inflation_radius_m: Clearance kept from walls
            seed: Random seed for RRT/PRM (None = different every run)
        """
        self.grid = occupancy_grid
        full = int(round(inflation_radius_m / occupancy_grid.resolution))
        self.inflation_steps = sorted({full, full // 2, 0}, reverse=True)  # cells, widest first
        self.rng = np.random.default_rng(seed)
        self.last_clearance_m = None  # wall clearance the last route kept

    def plan_path(
        self,
        start_x: float,
        start_y: float,
        goal_x: float,
        goal_y: float,
        algorithm: str = "astar"
    ) -> List[Tuple[float, float]]:
        """
        Plan a path from start to goal.

        Args:
            start_x, start_y: Start position
            goal_x, goal_y: Goal position
            algorithm: "astar", "rrt", "prm" or "simple"

        Returns:
            List of waypoints (x, y) from start to goal, both included;
            empty if the map leaves no way through
        """
        if algorithm == "simple":
            return self._simple_path(start_x, start_y, goal_x, goal_y)
        if algorithm not in ALGORITHMS:
            raise ValueError(f"Unknown algorithm {algorithm!r} - use one of {ALGORITHMS}")

        t0 = time.monotonic()
        for inflation in self.inflation_steps:
            blocked, unknown, (ox, oy), start, goal = self._prepare(start_x, start_y, goal_x, goal_y, inflation)
            if algorithm == "astar":
                cells = self._astar(blocked, unknown, start, goal)
            elif algorithm == "rrt":
                cells = self._rrt(blocked, start, goal)
            else:
                cells = self._prm(blocked, start, goal)
            if cells is not None:
                break
        else:
            logger.debug(f"{algorithm}: no path ({time.monotonic() - t0:.2f}s)")
            return []

        cells = self._shortcut(blocked, cells)
        path = [self.grid.grid_to_world(cx + ox, cy + oy) for cx, cy in cells]
        path[0], path[-1] = (start_x, start_y), (goal_x, goal_y)
        self.last_clearance_m = inflation * self.grid.resolution
        logger.debug(f"{algorithm}: {len(path)} waypoints, {self.last_clearance_m:.2f} m clearance "
                     f"({time.monotonic() - t0:.2f}s)")
        return path

    def _simple_path(
        self,
        start_x: float,
        start_y: float,
        goal_x: float,
        goal_y: float
    ) -> List[Tuple[float, float]]:
        """Simple straight-line path (no obstacle checking)."""
        return [
            (start_x, start_y),
            (goal_x, goal_y)
        ]

    def check_path_clear(
        self,
        start_x: float,
        start_y: float,
        end_x: float,
        end_y: float,
        safety_margin: float = 0.2
    ) -> bool:
        """
        Check if straight line path is clear of obstacles.

        Args:
            start_x, start_y: Start position
            end_x, end_y: End position
            safety_margin: Clearance around obstacles in meters

        Returns:
            True if path is clear
        """
        # Check cells along line, each with a square of safety_margin around it
        start_gx, start_gy = self.grid.world_to_grid(start_x, start_y)
        end_gx, end_gy = self.grid.world_to_grid(end_x, end_y)
        r = int(round(safety_margin / self.grid.resolution))

        cells = self.grid._bresenham_line(start_gx, start_gy, end_gx, end_gy)

        for gx, gy in cells:
            if self.grid.is_in_bounds(gx, gy):
                around = self.grid.grid[max(0, gy - r):gy + r + 1, max(0, gx - r):gx + r + 1]
                if (around > OCCUPIED_THRESHOLD).any():  # Occupied
                    return False

        return True

    # ---- shared helpers ----

    def _prepare(self, start_x, start_y, goal_x, goal_y, inflation: int):
        """
        Cut the planning window out of the map and build its obstacle mask,
        walls grown by inflation cells.

        Returns (blocked, unknown, (offset_x, offset_y), start, goal) with
        start/goal as window-local cells.
        """
        g = self.grid
        sx, sy = g.world_to_grid(start_x, start_y)
        gx, gy = g.world_to_grid(goal_x, goal_y)
        known_y, known_x = np.nonzero(g.grid != -1)
        xs = np.concatenate([known_x, [sx, gx]])
        ys = np.concatenate([known_y, [sy, gy]])
        pad = int(WINDOW_PAD_M / g.resolution)
        x0, x1 = max(0, xs.min() - pad), min(g.width, xs.max() + pad + 1)
        y0, y1 = max(0, ys.min() - pad), min(g.height, ys.max() + pad + 1)

        window = g.grid[y0:y1, x0:x1]
        occupied = window > OCCUPIED_THRESHOLD
        blocked = self._inflate(occupied, inflation)
        start = (int(np.clip(sx - x0, 0, x1 - x0 - 1)), int(np.clip(sy - y0, 0, y1 - y0 - 1)))
        goal = (int(np.clip(gx - x0, 0, x1 - x0 - 1)), int(np.clip(gy - y0, 0, y1 - y0 - 1)))
        for cx, cy in (start, goal):
            # Near the start/goal only the walls themselves block, not their margin
            r = inflation
            sl = (slice(max(0, cy - r), cy + r + 1), slice(max(0, cx - r), cx + r + 1))
            blocked[sl] = occupied[sl]
            blocked[cy, cx] = False
        return blocked, window == -1, (x0, y0), start, goal

    @staticmethod
    def _inflate(mask: np.ndarray, r: int) -> np.ndarray:
        """Grow True cells by a disc of radius r cells."""
        h, w = mask.shape
        out = mask.copy()
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy > r * r or (dx == 0 and dy == 0):
                    continue
                out[max(0, dy):h - max(0, -dy), max(0, dx):w - max(0, -dx)] |= \
                    mask[max(0, -dy):h - max(0, dy), max(0, -dx):w - max(0, dx)]
        return out

    def _line_free(self, blocked: np.ndarray, a: Cell, b: Cell) -> bool:
        return not any(blocked[y, x] for x, y in self.grid._bresenham_line(a[0], a[1], b[0], b[1]))

    def _shortcut(self, blocked: np.ndarray, cells: List[Cell]) -> List[Cell]:
        """Replace runs of cells with straight legs wherever the line is clear."""
        out = [cells[0]]
        i = 0
        while i < len(cells) - 1:
            j = i + 1
            while j + 1 < len(cells) and self._line_free(blocked, cells[i], cells[j + 1]):
                j += 1
            out.append(cells[j])
            i = j
        return out

    # ---- A* ----

    @staticmethod
    def _octile(a: Cell, b: Cell) -> float:
        dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
        return max(dx, dy) + (np.sqrt(2) - 1) * min(dx, dy)

    def _astar(self, blocked, unknown, start: Cell, goal: Cell) -> Optional[List[Cell]]:
        h, w = blocked.shape
        cost = np.where(unknown, UNKNOWN_COST, 1.0)
        best = {start: 0.0}
        came_from = {}
        heap = [(self._octile(start, goal), 0.0, start)]
        closed = set()
        while heap:
            _, g_cost, cur = heapq.heappop(heap)
            if cur == goal:
                path = [cur]
                while cur in came_from:
                    cur = came_from[cur]
                    path.append(cur)
                return path[::-1]
            if cur in closed:
                continue
            closed.add(cur)
            cx, cy = cur
            for dx, dy, step in _NEIGHBORS:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < w and 0 <= ny < h) or blocked[ny, nx]:
                    continue
                if dx and dy and (blocked[cy, nx] or blocked[ny, cx]):
                    continue  # no cutting diagonally past a wall corner
                new_cost = g_cost + step * cost[ny, nx]
                if new_cost < best.get((nx, ny), np.inf):
                    best[(nx, ny)] = new_cost
                    came_from[(nx, ny)] = cur
                    heapq.heappush(heap, (new_cost + self._octile((nx, ny), goal), new_cost, (nx, ny)))
        return None

    # ---- RRT ----

    def _rrt(self, blocked, start: Cell, goal: Cell) -> Optional[List[Cell]]:
        h, w = blocked.shape
        step = max(2.0, RRT_STEP_M / self.grid.resolution)
        nodes = np.zeros((RRT_MAX_ITERATIONS + 2, 2))
        nodes[0] = start
        parent = [-1]
        count = 1
        for _ in range(RRT_MAX_ITERATIONS):
            if self.rng.random() < RRT_GOAL_BIAS:
                sample = np.array(goal, dtype=float)
            else:
                sample = np.array([self.rng.integers(w), self.rng.integers(h)], dtype=float)
            nearest = int(np.argmin(((nodes[:count] - sample) ** 2).sum(axis=1)))
            near = nodes[nearest]
            dist = np.hypot(*(sample - near))
            if dist < 1:
                continue
            new = np.round(near + (sample - near) * min(1.0, step / dist)).astype(int)
            new_cell, near_cell = (int(new[0]), int(new[1])), (int(near[0]), int(near[1]))
            if blocked[new_cell[1], new_cell[0]] or not self._line_free(blocked, near_cell, new_cell):
                continue
            nodes[count] = new
            parent.append(nearest)
            count += 1
            if np.hypot(new_cell[0] - goal[0], new_cell[1] - goal[1]) <= step \
                    and self._line_free(blocked, new_cell, goal):
                nodes[count] = goal
                parent.append(count - 1)
                path, i = [], count
                while i != -1:
                    path.append((int(nodes[i][0]), int(nodes[i][1])))
                    i = parent[i]
                return path[::-1]
        return None

    # ---- PRM ----

    def _prm(self, blocked, start: Cell, goal: Cell) -> Optional[List[Cell]]:
        free = np.argwhere(~blocked)  # (y, x)
        picks = self.rng.choice(len(free), size=min(PRM_SAMPLES, len(free)), replace=False)
        points = [start, goal] + [(int(x), int(y)) for y, x in free[picks]]
        coords = np.array(points, dtype=float)
        radius = PRM_CONNECT_RADIUS_M / self.grid.resolution

        edges = [[] for _ in points]
        tried = set()
        for i in range(len(points)):
            dists = np.hypot(*(coords - coords[i]).T)
            for j in np.argsort(dists)[1:PRM_NEIGHBORS + 1]:
                j = int(j)
                pair = (min(i, j), max(i, j))
                if dists[j] > radius or pair in tried:
                    continue
                tried.add(pair)
                if self._line_free(blocked, points[i], points[j]):
                    edges[i].append((j, dists[j]))
                    edges[j].append((i, dists[j]))

        # Dijkstra from the start (node 0) to the goal (node 1)
        best = {0: 0.0}
        came_from = {}
        heap = [(0.0, 0)]
        while heap:
            d, i = heapq.heappop(heap)
            if i == 1:
                path = [1]
                while path[-1] in came_from:
                    path.append(came_from[path[-1]])
                return [points[k] for k in reversed(path)]
            if d > best[i]:
                continue
            for j, length in edges[i]:
                if d + length < best.get(j, np.inf):
                    best[j] = d + length
                    came_from[j] = i
                    heapq.heappush(heap, (d + length, j))
        return None


def main():
    """Plan around a wall with each algorithm."""
    grid = OccupancyGrid(width=100, height=100, resolution=0.05, origin_x=-2.5, origin_y=-2.5)
    grid.grid[:] = 100                   # walled-in room...
    grid.grid[20:80, 20:80] = 0          # ...3 m x 3 m of free floor
    grid.grid[20:65, 48:52] = 100        # wall up from the bottom, gap at the top
    planner = PathPlanner(grid, seed=1)
    for algorithm in ALGORITHMS:
        t0 = time.monotonic()
        path = planner.plan_path(-1.0, -1.0, 1.0, -1.0, algorithm)
        print(f"{algorithm:6s} {time.monotonic() - t0:5.2f}s  "
              + " -> ".join(f"({x:.2f},{y:.2f})" for x, y in path))


if __name__ == "__main__":
    main()
