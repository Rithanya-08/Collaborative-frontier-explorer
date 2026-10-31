#!/usr/bin/env python3
"""
Motor Speed Calibration Script

Calibrates left and right motor speeds for straight-line movement.
Without calibration, robots typically drift left or right due to 
motor speed asymmetries.

Procedure:
1. Place robot on flat surface with clear forward path
2. Script will move robot forward with default speeds
3. Observe if robot drifts left or right
4. Script will suggest speed adjustments
5. Update config/robot_config.yaml with new values
"""

import time
import logging
from hardware.motor_controller import MotorController

logging.basicConfig(level=logging.INFO, format='%(message)s')
logger = logging.getLogger(__name__)

def calibrate_motors():
    """Interactive motor calibration."""
    logger.info("\n" + "█"*50)
    logger.info("  MOTOR SPEED CALIBRATION UTILITY")
    logger.info("█"*50)
    
    logger.info("""
Calibration Procedure:
1. Place robot on flat surface (tape path or open floor)
2. Remove obstacles from 2m forward path
3. Robot will move forward for 5 seconds
4. Observe the drift direction
5. Adjust motor speeds based on feedback

Let's begin calibration...
""")
    
    input("Press ENTER to start calibration")
    
    motor = MotorController()
    default_left = 150
    default_right = 150
    
    logger.info("\n" + "="*50)
    logger.info("Test 1: Default speeds (Left: 150, Right: 150)")
    logger.info("="*50)
    logger.info("Moving forward for 5 seconds...")
    
    motor.move_forward(150, duration=5)
    
    logger.info("\nObservation:")
    logger.info("Did the robot move straight?")
    logger.info("  1. Yes, moved straight")
    logger.info("  2. No, drifted LEFT (turn right to compensate)")
    logger.info("  3. No, drifted RIGHT (turn left to compensate)")
    
    choice = input("Enter choice (1-3): ").strip()
    
    if choice == "1":
        logger.info("\n✓ Perfect calibration achieved!")
        logger.info("No changes needed to config/robot_config.yaml")
    
    elif choice == "2":
        logger.info("\nRobot drifted LEFT. Need to DECREASE LEFT motor speed.")
        logger.info("Adjusting: Left motor from 150 to 140")
        
        motor.set_motor_speed("left", 140)
        motor.set_motor_speed("right", 150)
        
        logger.info("Testing adjusted speeds (Left: 140, Right: 150)...")
        motor.differential_drive(140, 150, duration=5)
        
        logger.info("\nDrift better?")
        if input("Y/N: ").lower() == 'y':
            logger.info("\n✓ Update config/robot_config.yaml:")
            logger.info("   motor:")
            logger.info("     default_speed_left: 140")
            logger.info("     default_speed_right: 150")
        else:
            logger.info("Try larger adjustment: reduce left speed by 5-10 more")
    
    elif choice == "3":
        logger.info("\nRobot drifted RIGHT. Need to DECREASE RIGHT motor speed.")
        logger.info("Adjusting: Right motor from 150 to 140")
        
        motor.set_motor_speed("left", 150)
        motor.set_motor_speed("right", 140)
        
        logger.info("Testing adjusted speeds (Left: 150, Right: 140)...")
        motor.differential_drive(150, 140, duration=5)
        
        logger.info("\nDrift better?")
        if input("Y/N: ").lower() == 'y':
            logger.info("\n✓ Update config/robot_config.yaml:")
            logger.info("   motor:")
            logger.info("     default_speed_left: 150")
            logger.info("     default_speed_right: 140")
        else:
            logger.info("Try larger adjustment: reduce right speed by 5-10 more")
    
    motor.stop()
    motor.cleanup()
    
    logger.info("\n" + "="*50)
    logger.info("Calibration Tips:")
    logger.info("="*50)
    logger.info("""
1. Calibration is MOTOR/BATTERY dependent
   - Different batteries affect performance
   - Fully charge before calibration

2. Fine tuning:
   - Start with 5-10 point adjustments
   - Run multiple tests
   - Aim for 2m+ straight-line movement

3. Persistence:
   - Save calibrated values in config file
   - Verify after battery changes
   - Recalibrate if motors age

4. Formula for adjustment:
   - If drifting left: increase right speed OR decrease left
   - If drifting right: increase left speed OR decrease right
   - Typical range: ±10 PWM units from 150

5. Test on different surfaces:
   - Calibrate on the actual exploration surface
   - Different floors (carpet, tile, concrete) affect traction
""")

def main():
    """Main entry point."""
    try:
        calibrate_motors()
    except KeyboardInterrupt:
        logger.info("\nCalibration cancelled")
    except Exception as e:
        logger.error(f"Error during calibration: {e}")
    
    logger.info("\nCalibration complete!")

if __name__ == "__main__":
    main()
