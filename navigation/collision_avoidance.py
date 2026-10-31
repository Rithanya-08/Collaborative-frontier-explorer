"""
Collision Avoidance Module

Prevents robot from hitting obstacles using:
- Ultrasonic sensor readings (front, left, right)
- Occupancy grid proximity
"""

import numpy as np
import logging
from typing import Tuple, Optional

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class CollisionAvoidance:
    """
    Collision avoidance using reactive control.
    """
    
    def __init__(
        self,
        safety_distance: float = 0.3,  # meters
        stop_distance: float = 0.2,
        turn_safety_angle: float = 60  # degrees
    ):
        """
        Initialize collision avoidance.
        
        Args:
            safety_distance: Distance to start steering away
            stop_distance: Distance to emergency stop
            turn_safety_angle: Angle cone considered as forward
        """
        self.safety_distance = safety_distance
        self.stop_distance = stop_distance
        self.turn_safety_angle = turn_safety_angle
    
    def check_obstacle_ahead(
        self,
        angles: np.ndarray,
        distances: np.ndarray,
        heading: float
    ) -> bool:
        """
        Check if obstacle blocks forward path.

        Args:
            angles: Sensor reading angles in degrees
            distances: Sensor reading distances in meters
            heading: Robot heading in degrees

        Returns:
            True if obstacle ahead
        """
        if len(distances) == 0:
            return False
        
        # Forward-facing angles relative to robot heading
        relative_angles = (angles - heading) % 360
        
        # Check angles from -30 to +30 degrees (forward cone)
        forward_mask = (relative_angles <= 30) | (relative_angles >= 330)
        
        if not np.any(forward_mask):
            return False
        
        min_distance = np.min(distances[forward_mask])
        return min_distance < self.stop_distance
    
    def get_safe_direction(
        self,
        angles: np.ndarray,
        distances: np.ndarray,
        heading: float
    ) -> Optional[float]:
        """
        Find safest direction to turn.

        Args:
            angles: Sensor reading angles in degrees
            distances: Sensor reading distances in meters
            heading: Robot heading in degrees

        Returns:
            Recommended turn angle in degrees, or None
        """
        if len(distances) == 0:
            return None
        
        # Calculate distance per angle
        angle_bins = {}
        for angle, distance in zip(angles, distances):
            angle_bin = int(angle / 5) * 5  # 5-degree bins
            if angle_bin not in angle_bins or distance > angle_bins[angle_bin]:
                angle_bins[angle_bin] = distance
        
        # Find direction with maximum distance
        best_angle = max(angle_bins, key=angle_bins.get)
        return float(best_angle)


def main():
    """Test collision avoidance."""
    ca = CollisionAvoidance()

    # Simulate a 3-sensor ultrasonic reading (front, left, right) with an
    # obstacle directly ahead
    angles = np.array([0.0, 90.0, 270.0])
    distances = np.array([0.15, 2.0, 2.0])

    heading = 0.0
    
    obstacle = ca.check_obstacle_ahead(angles, distances, heading)
    safe_dir = ca.get_safe_direction(angles, distances, heading)
    
    print(f"Obstacle ahead: {obstacle}")
    print(f"Safe direction: {safe_dir}")


if __name__ == "__main__":
    main()
