#!/usr/bin/env python3
"""
Sensor Testing Script

Comprehensive test of all robot sensors and actuators.
Verifies:
- Motor connectivity and functionality
- IMU readings (accelerometer, gyroscope)
- Ultrasonic distance measurements
"""

import sys
import time
import logging

logging.basicConfig(level=logging.INFO, format='%(message)s')
logger = logging.getLogger(__name__)

def test_motor_controller():
    """Test motor controller."""
    logger.info("\n" + "="*50)
    logger.info("Testing Motor Controller")
    logger.info("="*50)
    
    try:
        from hardware.motor_controller import MotorController
        motor = MotorController()
        
        # Test individual movements
        logger.info("Testing forward movement (2 seconds)...")
        motor.move_forward(150, duration=2)
        
        logger.info("Testing backward movement (2 seconds)...")
        motor.move_backward(150, duration=2)
        
        logger.info("Testing left turn (2 seconds)...")
        motor.turn_left(150, duration=2)
        
        logger.info("Testing right turn (2 seconds)...")
        motor.turn_right(150, duration=2)
        
        motor.stop()
        motor.cleanup()
        
        logger.info("✓ Motor controller: PASS")
        return True
    
    except Exception as e:
        logger.error(f"✗ Motor controller: FAIL - {e}")
        return False

def test_imu():
    """Test IMU sensor."""
    logger.info("\n" + "="*50)
    logger.info("Testing IMU Sensor")
    logger.info("="*50)
    
    try:
        from hardware.imu_driver import IMUDriver
        imu = IMUDriver()
        
        logger.info("Calibrating gyroscope...")
        imu.calibrate_gyroscope()
        
        logger.info("Reading sensor data...")
        for i in range(5):
            data = imu.get_all_data()
            imu.update_heading()
            
            logger.info(f"  [{i+1}] Accel: ({data['accel_x']:.2f}, {data['accel_y']:.2f}, {data['accel_z']:.2f}) m/s²")
            logger.info(f"      Gyro: ({data['gyro_x']:.2f}, {data['gyro_y']:.2f}, {data['gyro_z']:.2f}) °/s")
            logger.info(f"      Heading: {data['heading']:.1f}°")
            
            time.sleep(0.1)
        
        imu.cleanup()
        
        logger.info("✓ IMU: PASS")
        return True
    
    except Exception as e:
        logger.error(f"✗ IMU: FAIL - {e}")
        return False

def test_ultrasonic():
    """Test ultrasonic sensors."""
    logger.info("\n" + "="*50)
    logger.info("Testing Ultrasonic Sensors")
    logger.info("="*50)
    
    try:
        from hardware.ultrasonic_driver import MultiUltrasonicArray
        ultrasonic = MultiUltrasonicArray()
        
        for i in range(5):
            distances = ultrasonic.measure_all()
            logger.info(f"  [{i+1}] Front: {distances.get('front', 'N/A'):.2f}m, "
                       f"Left: {distances.get('left', 'N/A'):.2f}m, "
                       f"Right: {distances.get('right', 'N/A'):.2f}m")
            time.sleep(0.5)
        
        ultrasonic.cleanup()
        
        logger.info("✓ Ultrasonic: PASS")
        return True
    
    except Exception as e:
        logger.error(f"✗ Ultrasonic: FAIL - {e}")
        return False

def test_slam():
    """Test SLAM mapping."""
    logger.info("\n" + "="*50)
    logger.info("Testing SLAM (Occupancy Grid)")
    logger.info("="*50)
    
    try:
        import numpy as np
        from slam.occupancy_grid import OccupancyGrid
        
        grid = OccupancyGrid()

        # Simulate a 3-sensor ultrasonic scan (front, left, right)
        angles = np.array([0.0, 90.0, 270.0])
        distances = np.array([2.0, 2.5, 2.5])

        grid.update_from_range_scan(angles, distances, 0, 0, 0)
        
        coverage = grid.get_coverage_percentage()
        occupied = len(grid.get_occupied_cells())
        free = len(grid.get_free_cells())
        
        logger.info(f"  Coverage: {coverage:.1f}%")
        logger.info(f"  Occupied cells: {occupied}")
        logger.info(f"  Free cells: {free}")
        
        logger.info("✓ SLAM: PASS")
        return True
    
    except Exception as e:
        logger.error(f"✗ SLAM: FAIL - {e}")
        return False

def test_frontier_explorer():
    """Test frontier detection."""
    logger.info("\n" + "="*50)
    logger.info("Testing Frontier Explorer")
    logger.info("="*50)
    
    try:
        import numpy as np
        from slam.occupancy_grid import OccupancyGrid
        from navigation.frontier_explorer import FrontierExplorer
        
        grid = OccupancyGrid()
        explorer = FrontierExplorer(grid)

        # Simulate some 3-sensor ultrasonic scans (front, left, right)
        angles = np.array([0.0, 90.0, 270.0])
        for i in range(5):
            distances = np.array([2.0, 2.5, 2.5])
            grid.update_from_range_scan(angles, distances, i * 0.5, 0, 0)
        
        frontiers = explorer.find_frontiers()
        progress = explorer.get_exploration_progress()
        
        logger.info(f"  Frontiers detected: {len(frontiers)}")
        logger.info(f"  Exploration progress: {progress:.1f}%")
        
        if frontiers:
            logger.info(f"  Closest frontier: ({frontiers[0][0]:.2f}, {frontiers[0][1]:.2f})")
        
        logger.info("✓ Frontier Explorer: PASS")
        return True
    
    except Exception as e:
        logger.error(f"✗ Frontier Explorer: FAIL - {e}")
        return False

def main():
    """Run all tests."""
    logger.info("\n" + "█"*50)
    logger.info("  MULTI-ROBOT EXPLORATION - SENSOR TEST SUITE")
    logger.info("█"*50)
    
    results = {
        "Motor Controller": False,
        "IMU": False,
        "Ultrasonic": False,
        "SLAM": False,
        "Frontier Explorer": False,
    }

    # Run tests
    results["Motor Controller"] = test_motor_controller()
    results["IMU"] = test_imu()
    results["Ultrasonic"] = test_ultrasonic()
    results["SLAM"] = test_slam()
    results["Frontier Explorer"] = test_frontier_explorer()
    
    # Summary
    logger.info("\n" + "="*50)
    logger.info("TEST SUMMARY")
    logger.info("="*50)
    
    passed = sum(1 for v in results.values() if v)
    total = len(results)
    
    for test, result in results.items():
        status = "✓ PASS" if result else "✗ FAIL"
        logger.info(f"  {test:.<35} {status}")
    
    logger.info(f"\nTotal: {passed}/{total} tests passed")
    
    if passed == total:
        logger.info("\n✓ All tests passed! Robot ready for exploration.")
        return 0
    else:
        logger.warning(f"\n✗ {total - passed} test(s) failed. Check errors above.")
        return 1

if __name__ == "__main__":
    sys.exit(main())
