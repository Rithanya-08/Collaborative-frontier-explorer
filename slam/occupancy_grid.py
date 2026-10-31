"""
Occupancy Grid Module

Grid-based map representation for SLAM and obstacle mapping.
Uses grid cells to represent probability of occupancy (0-100%).

Each cell represents a fixed area (e.g., 5cm x 5cm).
Cell values:
  -1 or 205: Unknown (not yet observed)
  0-50: Free space (probably empty)
  51-100: Occupied (probably contains obstacle)
"""

import numpy as np
import logging
from typing import Tuple, Optional
import json

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class OccupancyGrid:
    """
    2D occupancy grid map for robot navigation and SLAM.
    
    Features:
    - Probabilistic cell occupancy (0-100)
    - Coordinate transformation (world <-> grid)
    - Map visualization ready
    - Serialization/deserialization
    """
    
    def __init__(
        self,
        width: int = 500,           # cells in x direction
        height: int = 500,          # cells in y direction
        resolution: float = 0.05,   # meters per cell
        origin_x: float = -12.5,    # world x of grid origin (typically -width*resolution/2)
        origin_y: float = -12.5,    # world y of grid origin
    ):
        """
        Initialize occupancy grid.
        
        Args:
            width: Grid width in cells
            height: Grid height in cells
            resolution: Size of each cell in meters
            origin_x: World x coordinate of grid origin
            origin_y: World y coordinate of grid origin
        """
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        
        # Grid: -1=unknown, 0-50=free, 51-100=occupied
        self.grid = np.full((height, width), -1, dtype=np.int8)
        
        # Confidence map: how many updates each cell has received
        self.confidence = np.zeros((height, width), dtype=np.uint8)
        
        logger.info(f"Occupancy grid created: {width}x{height} cells, {resolution}m/cell")
    
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """
        Convert world coordinates to grid cell indices.
        
        Args:
            x, y: World coordinates in meters
        
        Returns:
            (grid_x, grid_y) indices
        """
        grid_x = int((x - self.origin_x) / self.resolution)
        grid_y = int((y - self.origin_y) / self.resolution)
        
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """
        Convert grid cell indices to world coordinates.
        
        Args:
            grid_x, grid_y: Grid cell indices
        
        Returns:
            (x, y) world coordinates in meters
        """
        x = self.origin_x + (grid_x + 0.5) * self.resolution
        y = self.origin_y + (grid_y + 0.5) * self.resolution
        
        return x, y
    
    def is_in_bounds(self, grid_x: int, grid_y: int) -> bool:
        """Check if grid cell is within bounds."""
        return 0 <= grid_x < self.width and 0 <= grid_y < self.height
    
    def set_occupancy(
        self,
        x: float,
        y: float,
        occupancy: int,
        confidence: int = 1
    ):
        """
        Update single cell occupancy.
        
        Args:
            x, y: World coordinates
            occupancy: Probability 0-100 (0=free, 100=occupied)
            confidence: Confidence weight for averaging (typically 1-10)
        """
        grid_x, grid_y = self.world_to_grid(x, y)
        
        if not self.is_in_bounds(grid_x, grid_y):
            return
        
        # Initialize cell if unknown
        if self.grid[grid_y, grid_x] == -1:
            self.grid[grid_y, grid_x] = occupancy
            self.confidence[grid_y, grid_x] = min(confidence, 255)
        else:
            # Cast to plain Python ints before arithmetic - self.grid and
            # self.confidence use narrow int8/uint8 dtypes to save memory,
            # and numpy scalar arithmetic on those can silently overflow
            # (wrap around) rather than widening automatically. A cell
            # revisited enough times could push confidence past 255,
            # wrap around, and eventually divide by a wrapped-to-zero
            # value - which crashed the control loop on a long run.
            old_val = int(self.grid[grid_y, grid_x])
            old_conf = int(self.confidence[grid_y, grid_x])
            new_conf = min(old_conf + confidence, 255)  # cap BEFORE dividing

            self.grid[grid_y, grid_x] = int(
                (old_val * old_conf + occupancy * confidence) / new_conf
            )
            self.confidence[grid_y, grid_x] = new_conf
    
    def update_from_range_scan(
        self,
        scan_angles: np.ndarray,
        scan_distances: np.ndarray,
        robot_x: float,
        robot_y: float,
        robot_heading: float,
        max_range: float = 4.0,
        hit_probability: int = 80,
        miss_probability: int = 20
    ):
        """
        Update occupancy grid from a set of range-sensor readings (e.g. the
        3 fixed-direction ultrasonic sensors: front/left/right).

        Args:
            scan_angles: Array of measurement angles in degrees, relative
                to the robot (0 = front, 90 = left, 270 = right)
            scan_distances: Array of distances in meters, one per angle
            robot_x, robot_y: Robot position in world coordinates
            robot_heading: Robot heading in degrees
            max_range: Max range to consider for free space
            hit_probability: Occupancy value for hit cells (1-100)
            miss_probability: Occupancy value for missed cells (0-50)
        """
        if len(scan_angles) == 0 or len(scan_distances) == 0:
            return

        # Process each range measurement
        for angle, distance in zip(scan_angles, scan_distances):
            # World angle = robot heading + sensor's relative angle
            world_angle = (robot_heading + angle) % 360
            world_angle_rad = np.radians(world_angle)
            
            # Mark cells along ray as free space (ray casting)
            if distance < max_range:
                # Ray from robot to obstacle
                end_x = robot_x + distance * np.cos(world_angle_rad)
                end_y = robot_y + distance * np.sin(world_angle_rad)
                
                # Bresenham line to mark ray as free
                self._mark_ray_free(robot_x, robot_y, end_x, end_y, miss_probability)
                
                # Mark obstacle location as occupied
                self.set_occupancy(end_x, end_y, hit_probability, confidence=2)
            else:
                # Ray extends beyond max_range, mark ray as free
                end_x = robot_x + max_range * np.cos(world_angle_rad)
                end_y = robot_y + max_range * np.sin(world_angle_rad)
                self._mark_ray_free(robot_x, robot_y, end_x, end_y, miss_probability)
    
    def _mark_ray_free(
        self,
        start_x: float,
        start_y: float,
        end_x: float,
        end_y: float,
        miss_probability: int = 20
    ):
        """Mark cells along a ray as free space."""
        # Convert to grid coordinates
        start_gx, start_gy = self.world_to_grid(start_x, start_y)
        end_gx, end_gy = self.world_to_grid(end_x, end_y)
        
        # Use Bresenham line algorithm
        cells = self._bresenham_line(start_gx, start_gy, end_gx, end_gy)
        
        for gx, gy in cells:
            if self.is_in_bounds(gx, gy):
                # Only mark if cell is unknown or less confident
                if self.grid[gy, gx] <= miss_probability:
                    self.set_occupancy(
                        *self.grid_to_world(gx, gy),
                        miss_probability,
                        confidence=1
                    )
    
    def mark_footprint_free(self, x: float, y: float, radius: float):
        """
        Mark the cells a robot is standing on as free. A robot can't be
        inside a wall, so this erases anything wrongly mapped there - e.g.
        the teammate robot seen by a sensor before its position was known.
        """
        cx, cy = self.world_to_grid(x, y)
        r = int(np.ceil(radius / self.resolution))
        y0, y1 = max(0, cy - r), min(self.height, cy + r + 1)
        x0, x1 = max(0, cx - r), min(self.width, cx + r + 1)
        if y0 >= y1 or x0 >= x1:
            return
        gy, gx = np.ogrid[y0:y1, x0:x1]
        disc = (gx - cx) ** 2 + (gy - cy) ** 2 <= r * r
        self.grid[y0:y1, x0:x1][disc] = 0
        conf = self.confidence[y0:y1, x0:x1]
        conf[disc] = np.maximum(conf[disc], 1)

    @staticmethod
    def _bresenham_line(x0: int, y0: int, x1: int, y1: int) -> list:
        """Bresenham line algorithm for ray casting."""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        
        if dx > dy:
            err = dx / 2.0
            y = y0
            for x in range(x0, x1 + sx, sx):
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
        else:
            err = dy / 2.0
            x = x0
            for y in range(y0, y1 + sy, sy):
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
        
        return points
    
    def get_cell_occupancy(self, x: float, y: float) -> int:
        """
        Get occupancy value of cell at world coordinates.
        
        Returns:
            -1 (unknown), 0-100 (occupancy probability)
        """
        grid_x, grid_y = self.world_to_grid(x, y)
        
        if not self.is_in_bounds(grid_x, grid_y):
            return -1
        
        return self.grid[grid_y, grid_x]
    
    def get_free_cells(self, threshold: int = 50) -> np.ndarray:
        """Get coordinates of free cells (occupancy < threshold)."""
        mask = (self.grid != -1) & (self.grid < threshold)
        indices = np.argwhere(mask)
        
        coords = []
        for gy, gx in indices:
            x, y = self.grid_to_world(gx, gy)
            coords.append((x, y))
        
        return np.array(coords) if coords else np.empty((0, 2))
    
    def get_occupied_cells(self, threshold: int = 50) -> np.ndarray:
        """Get coordinates of occupied cells (occupancy >= threshold)."""
        mask = self.grid >= threshold
        indices = np.argwhere(mask)
        
        coords = []
        for gy, gx in indices:
            x, y = self.grid_to_world(gx, gy)
            coords.append((x, y))
        
        return np.array(coords) if coords else np.empty((0, 2))
    
    def get_unknown_cells(self) -> np.ndarray:
        """Get coordinates of unknown cells."""
        mask = self.grid == -1
        indices = np.argwhere(mask)
        
        coords = []
        for gy, gx in indices:
            x, y = self.grid_to_world(gx, gy)
            coords.append((x, y))
        
        return np.array(coords) if coords else np.empty((0, 2))
    
    def get_coverage_percentage(self) -> float:
        """Get percentage of grid that has been explored (not unknown)."""
        explored = np.count_nonzero(self.grid != -1)
        total = self.width * self.height
        return (explored / total) * 100 if total > 0 else 0
    
    def save_to_file(
        self,
        filename: str,
        trajectory: Optional[list] = None,
        heading: Optional[float] = None,
        direction_changes: Optional[list] = None,
        objects: Optional[list] = None
    ):
        """
        Save occupancy grid to file.

        Args:
            filename: Output path
            trajectory: Optional list of (x, y) world-coordinate points the
                robot passed through, for drawing its path on the map
            heading: Optional final heading in degrees, for drawing a
                direction indicator on the map
            direction_changes: Optional list of (x, y, letter) points
                marking where the robot turned ("L"/"R"), for labeling
                turns along the path on the map
            objects: Optional list of recognised objects (dicts with label,
                x, y, confidence, count, source) to mark on the map
        """
        data = {
            'width': self.width,
            'height': self.height,
            'resolution': self.resolution,
            'origin_x': self.origin_x,
            'origin_y': self.origin_y,
            'grid': self.grid.tolist(),
            'confidence': self.confidence.tolist()
        }
        if trajectory is not None:
            data['trajectory'] = [(float(x), float(y)) for x, y in trajectory]
        if heading is not None:
            data['final_heading'] = float(heading)
        if direction_changes is not None:
            data['direction_changes'] = [(float(x), float(y), letter) for x, y, letter in direction_changes]
        if objects is not None:
            data['objects'] = objects

        with open(filename, 'w') as f:
            json.dump(data, f)

        logger.info(f"Map saved to {filename}")
    
    def load_from_file(self, filename: str):
        """Load occupancy grid from file."""
        with open(filename, 'r') as f:
            data = json.load(f)
        
        self.width = data['width']
        self.height = data['height']
        self.resolution = data['resolution']
        self.origin_x = data['origin_x']
        self.origin_y = data['origin_y']
        self.grid = np.array(data['grid'], dtype=np.int8)
        self.confidence = np.array(data['confidence'], dtype=np.uint8)
        
        logger.info(f"Map loaded from {filename}")


def main():
    """Test occupancy grid."""
    grid = OccupancyGrid(width=200, height=200, resolution=0.05)
    
    # Simulate robot position
    robot_x, robot_y = 0.0, 0.0
    
    # Simulate a 3-sensor ultrasonic scan (front, left, right)
    angles = np.array([0.0, 90.0, 270.0])
    distances = np.array([2.0, 2.0, 2.0])

    # Update grid
    grid.update_from_range_scan(angles, distances, robot_x, robot_y, 0.0)
    
    # Get statistics
    coverage = grid.get_coverage_percentage()
    free_cells = grid.get_free_cells()
    occupied_cells = grid.get_occupied_cells()
    
    print(f"Coverage: {coverage:.1f}%")
    print(f"Free cells: {len(free_cells)}")
    print(f"Occupied cells: {len(occupied_cells)}")
    print(f"Sample occupied cells: {occupied_cells[:5]}")
    
    # Save map
    grid.save_to_file("/tmp/test_map.json")


if __name__ == "__main__":
    main()
