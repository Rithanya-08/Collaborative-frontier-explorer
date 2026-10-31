"""
Frontier Explorer Module

Frontier-based exploration algorithm:
Finds edges between explored and unexplored areas
and guides robot to these frontiers for efficient exploration.
"""

import numpy as np
import logging
from typing import Optional, Tuple, List
from slam.occupancy_grid import OccupancyGrid

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class FrontierExplorer:
    """
    Frontier-based exploration.
    
    Uses frontier cells (edges between known and unknown space)
    to guide exploration efficiently.
    """
    
    def __init__(
        self,
        occupancy_grid: OccupancyGrid,
        min_frontier_size: int = 10,
        unknown_threshold: int = 50
    ):
        """
        Initialize frontier explorer.
        
        Args:
            occupancy_grid: Reference to occupancy grid
            min_frontier_size: Minimum number of cells for a frontier
            unknown_threshold: Occupancy threshold for "unknown" cells
        """
        self.grid = occupancy_grid
        self.min_frontier_size = min_frontier_size
        self.unknown_threshold = unknown_threshold
        self.frontiers = []
    
    def find_frontiers(self) -> List[Tuple[float, float]]:
        """
        Find all frontier clusters in the map.
        
        Returns:
            List of frontier centroids as (x, y) tuples
        """
        # Get the raw grid
        grid = self.grid.grid
        
        # Find frontier cells (free cells adjacent to unknown cells)
        height, width = grid.shape
        frontier_cells = np.zeros((height, width), dtype=bool)
        
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                # Check if this is a free cell
                if 0 <= grid[y, x] <= 50:
                    # Check if adjacent to unknown cell
                    neighbors = [
                        grid[y-1, x], grid[y+1, x],
                        grid[y, x-1], grid[y, x+1],
                        grid[y-1, x-1], grid[y-1, x+1],
                        grid[y+1, x-1], grid[y+1, x+1]
                    ]
                    
                    if any(n == -1 for n in neighbors):
                        frontier_cells[y, x] = True
        
        # Cluster frontier cells
        visited = np.zeros((height, width), dtype=bool)
        clusters = []
        
        for y in range(height):
            for x in range(width):
                if frontier_cells[y, x] and not visited[y, x]:
                    cluster = self._flood_fill(frontier_cells, visited, x, y)
                    if len(cluster) >= self.min_frontier_size:
                        clusters.append(cluster)
        
        # Convert clusters to centroids
        centroids = []
        for cluster in clusters:
            if len(cluster) > 0:
                cluster_array = np.array(cluster)
                center_gx = int(np.mean(cluster_array[:, 0]))
                center_gy = int(np.mean(cluster_array[:, 1]))
                
                cx, cy = self.grid.grid_to_world(center_gx, center_gy)
                centroids.append((cx, cy))
        
        self.frontiers = centroids
        return centroids
    
    @staticmethod
    def _flood_fill(
        frontier_cells: np.ndarray,
        visited: np.ndarray,
        start_x: int,
        start_y: int
    ) -> List[Tuple[int, int]]:
        """Flood fill to find connected frontier cells."""
        height, width = frontier_cells.shape
        cluster = []
        stack = [(start_x, start_y)]
        
        while stack:
            x, y = stack.pop()
            
            if not (0 <= x < width and 0 <= y < height):
                continue
            if visited[y, x] or not frontier_cells[y, x]:
                continue
            
            visited[y, x] = True
            cluster.append((x, y))
            
            # Add neighbors
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                stack.append((x + dx, y + dy))
        
        return cluster
    
    def get_next_frontier(
        self,
        robot_x: float,
        robot_y: float
    ) -> Optional[Tuple[float, float]]:
        """
        Get closest frontier to robot.
        
        Args:
            robot_x, robot_y: Current robot position
        
        Returns:
            (x, y) of closest frontier, or None if no frontier found
        """
        frontiers = self.find_frontiers()
        
        if not frontiers:
            return None
        
        # Find closest frontier
        robot_pos = np.array([robot_x, robot_y])
        min_distance = float('inf')
        closest_frontier = None
        
        for frontier in frontiers:
            distance = np.linalg.norm(np.array(frontier) - robot_pos)
            if distance < min_distance:
                min_distance = distance
                closest_frontier = frontier
        
        return closest_frontier
    
    def get_exploration_progress(self) -> float:
        """Get percentage of map explored."""
        return self.grid.get_coverage_percentage()


def main():
    """Test frontier explorer."""
    # Create a test grid
    grid = OccupancyGrid(width=100, height=100, resolution=0.1)
    
    # Simulate some 3-sensor ultrasonic scans (front, left, right)
    for i in range(10):
        angles = np.array([0.0, 90.0, 270.0])
        distances = np.array([1.0, 1.5, 1.5])

        grid.update_from_range_scan(angles, distances, i * 0.5, 0, 0)
    
    # Find frontiers
    explorer = FrontierExplorer(grid)
    frontiers = explorer.find_frontiers()
    
    print(f"Found {len(frontiers)} frontier(s)")
    for i, (fx, fy) in enumerate(frontiers[:5]):
        print(f"  Frontier {i+1}: ({fx:.2f}, {fy:.2f})")


if __name__ == "__main__":
    main()
