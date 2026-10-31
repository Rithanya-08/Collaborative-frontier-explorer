"""
Motor Controller Module

Controls four DC gear motors (front-left, rear-left, front-right, rear-right)
via two TB6612FNG motor driver boards on a Raspberry Pi. Driver 1 controls
the LEFT side (front-left on Channel A, rear-left on Channel B), Driver 2
controls the RIGHT side (front-right on Channel A, rear-right on Channel B).
See "Wiring details.docx" for the physical connections.

Pin Configuration (BCM numbering):
    STBY (shared by both drivers): GPIO21

    Driver 1 (LEFT):  PWMA=12 AIN1=5  AIN2=6   (front-left,  Channel A)
                       PWMB=13 BIN1=16 BIN2=20  (rear-left,   Channel B)
    Driver 2 (RIGHT): PWMA=18 AIN1=23 AIN2=24  (front-right, Channel A)
                       PWMB=19 BIN1=25 BIN2=26  (rear-right,  Channel B)

Driver 2 is mounted mirrored relative to Driver 1, so the same AIN/BIN
signal spins its motors the opposite physical direction - the right-side
motors are marked "invert" below to compensate (confirmed on hardware).
"""

import RPi.GPIO as GPIO
import time
from typing import Dict, Optional
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

STBY_PIN = 21
PWM_FREQUENCY_HZ = 1000

# motor name -> (pwm_pin, in1_pin, in2_pin, invert, side)
MOTOR_PINS = {
    "front_left":  (12, 5, 6, False, "left"),
    "rear_left":   (13, 16, 20, False, "left"),
    "front_right": (18, 23, 24, True, "right"),
    "rear_right":  (19, 25, 26, True, "right"),
}


class MotorController:
    """
    Controls a 4-wheel-drive robot built from two TB6612FNG motor driver
    boards (one per side, each driving a front and rear TT motor).

    Supports:
    - Per-side speed control (left/right), driving both motors on that side
    - PWM speed control (-255 to 255)
    - Forward/backward/stop and pivot turns (only one side spins - the
      current hardware has no encoders/IMU feedback for closed-loop turns)
    - Ramped speed changes when multiple motors start together, so the
      shared power supply doesn't sag under simultaneous stall current
      (confirmed necessary on hardware)
    """

    def __init__(
        self,
        stby_pin: int = STBY_PIN,
        pwm_frequency: int = PWM_FREQUENCY_HZ,
        ramp_step: float = 5.0,
        ramp_delay: float = 0.05,
        turn_ramp_step: float = 20.0,
        turn_ramp_delay: float = 0.03,
    ):
        """
        Initialize motor controller.

        Args:
            stby_pin: Shared STBY GPIO pin for both TB6612FNG drivers
            pwm_frequency: PWM frequency in Hz
            ramp_step: Duty-cycle percent increment per ramp step (forward/backward)
            ramp_delay: Seconds between ramp steps (forward/backward)
            turn_ramp_step: Duty-cycle percent increment per ramp step for
                turns - coarser/faster than the forward ramp (matches the
                pivot-turn timing calibrated in turn_90.py/obstacle_avoidance.py,
                so time-based turn-angle estimates stay valid) since less
                time should be spent at weak, sub-turning duty
            turn_ramp_delay: Seconds between ramp steps for turns
        """
        self.stby_pin = stby_pin
        self.pwm_frequency = pwm_frequency
        self.ramp_step = ramp_step
        self.ramp_delay = ramp_delay
        self.turn_ramp_step = turn_ramp_step
        self.turn_ramp_delay = turn_ramp_delay

        self.pwm_objects = {}
        self._current_duty = {name: 0.0 for name in MOTOR_PINS}

        self._setup_gpio()
        logger.info("Motor controller initialized (2x TB6612FNG, 4 motors)")

    def _setup_gpio(self):
        """Setup GPIO pins for motor control."""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(self.stby_pin, GPIO.OUT)
        GPIO.output(self.stby_pin, GPIO.HIGH)  # take drivers out of standby

        for name, (pwm_pin, in1_pin, in2_pin, _invert, _side) in MOTOR_PINS.items():
            GPIO.setup(pwm_pin, GPIO.OUT)
            GPIO.setup(in1_pin, GPIO.OUT)
            GPIO.setup(in2_pin, GPIO.OUT)

            pwm = GPIO.PWM(pwm_pin, self.pwm_frequency)
            pwm.start(0)
            self.pwm_objects[name] = pwm

        logger.info("GPIO setup complete")

    def _motor_names(self, side: str):
        """Names of the motors on a given side ('left' or 'right')."""
        return [name for name, spec in MOTOR_PINS.items() if spec[4] == side]

    def _apply_direction(self, name: str, physical_forward: bool):
        """Drive a motor's IN1/IN2 pins for the requested physical direction."""
        _pwm_pin, in1_pin, in2_pin, invert, _side = MOTOR_PINS[name]
        signal_forward = physical_forward != invert  # inverted motors flip the signal

        if signal_forward:
            GPIO.output(in1_pin, GPIO.HIGH)
            GPIO.output(in2_pin, GPIO.LOW)
        else:
            GPIO.output(in1_pin, GPIO.LOW)
            GPIO.output(in2_pin, GPIO.HIGH)

    def _drive(
        self,
        speeds: Dict[str, int],
        ramp: bool = True,
        ramp_step: Optional[float] = None,
        ramp_delay: Optional[float] = None,
    ):
        """
        Set direction and duty cycle for a set of motors.

        Args:
            speeds: mapping of motor name -> target speed (-255 to 255)
            ramp: gradually ramp duty cycle instead of jumping instantly,
                  so multiple motors starting together don't sag the
                  shared power supply
            ramp_step: override self.ramp_step for this call (e.g. turns
                use a coarser/faster ramp than forward/backward driving)
            ramp_delay: override self.ramp_delay for this call
        """
        for name, speed in speeds.items():
            _pwm_pin, in1_pin, in2_pin, _invert, _side = MOTOR_PINS[name]
            if speed == 0:
                GPIO.output(in1_pin, GPIO.LOW)
                GPIO.output(in2_pin, GPIO.LOW)
            else:
                self._apply_direction(name, physical_forward=speed > 0)

        targets = {name: min(abs(speed) / 255 * 100, 100) for name, speed in speeds.items()}

        if not ramp:
            for name, duty in targets.items():
                self.pwm_objects[name].ChangeDutyCycle(duty)
                self._current_duty[name] = duty
            return

        step_size = ramp_step if ramp_step is not None else self.ramp_step
        delay = ramp_delay if ramp_delay is not None else self.ramp_delay

        max_delta = max(
            (abs(target - self._current_duty[name]) for name, target in targets.items()),
            default=0,
        )
        num_steps = max(1, int(max_delta / step_size))

        for step in range(1, num_steps + 1):
            for name, target in targets.items():
                start = self._current_duty[name]
                self.pwm_objects[name].ChangeDutyCycle(start + (target - start) * step / num_steps)
            time.sleep(delay)

        for name, target in targets.items():
            self.pwm_objects[name].ChangeDutyCycle(target)
            self._current_duty[name] = target

    def set_motor_speed(self, motor: str, speed: int):
        """
        Set speed of one side's motor pair instantly (no ramp) - useful
        for fine calibration adjustments.

        Args:
            motor: "left" or "right"
            speed: -255 to 255
                   0 = stop
                   positive = forward
                   negative = backward
        """
        if motor not in ("left", "right"):
            logger.warning(f"Unknown motor: {motor}")
            return

        speed = max(-255, min(255, speed))
        self._drive({name: speed for name in self._motor_names(motor)}, ramp=False)

    def move_forward(self, speed: int = 200, duration: Optional[float] = None):
        """Move robot forward at given speed (0-255)."""
        self._drive({name: speed for name in MOTOR_PINS}, ramp=True)

        if duration:
            time.sleep(duration)
            self.stop()

    def move_backward(self, speed: int = 200, duration: Optional[float] = None):
        """Move robot backward at given speed (0-255)."""
        self._drive({name: -speed for name in MOTOR_PINS}, ramp=True)

        if duration:
            time.sleep(duration)
            self.stop()

    def turn_left(self, speed: int = 200, duration: Optional[float] = None):
        """
        Pivot robot left (counterclockwise).

        Left side stays stopped, right side drives forward - driving only
        one side alone is what actually pivots the robot on this hardware
        (confirmed on the actual robot; there's no differential-speed turn).
        """
        speeds = {name: 0 for name in self._motor_names("left")}
        speeds.update({name: speed for name in self._motor_names("right")})
        self._drive(speeds, ramp=True, ramp_step=self.turn_ramp_step, ramp_delay=self.turn_ramp_delay)

        if duration:
            time.sleep(duration)
            self.stop()

    def turn_right(self, speed: int = 200, duration: Optional[float] = None):
        """Pivot robot right (clockwise). Right side stopped, left side drives."""
        speeds = {name: speed for name in self._motor_names("left")}
        speeds.update({name: 0 for name in self._motor_names("right")})
        self._drive(speeds, ramp=True, ramp_step=self.turn_ramp_step, ramp_delay=self.turn_ramp_delay)

        if duration:
            time.sleep(duration)
            self.stop()

    def stop(self):
        """Stop all motors."""
        self._drive({name: 0 for name in MOTOR_PINS}, ramp=False)

    def differential_drive(
        self,
        left_speed: int,
        right_speed: int,
        duration: Optional[float] = None
    ):
        """
        Control left and right motor pairs independently.

        Args:
            left_speed: Left side speed (-255 to 255)
            right_speed: Right side speed (-255 to 255)
            duration: Duration in seconds (None = continuous)
        """
        speeds = {name: left_speed for name in self._motor_names("left")}
        speeds.update({name: right_speed for name in self._motor_names("right")})
        self._drive(speeds, ramp=True)

        if duration:
            time.sleep(duration)
            self.stop()

    def cleanup(self):
        """Clean up GPIO pins."""
        logger.info("Cleaning up GPIO")
        self.stop()
        GPIO.output(self.stby_pin, GPIO.LOW)
        # Pop (not iterate-then-clear) so no loop variable keeps the last
        # PWM object referenced past this loop - a lingering reference
        # here would only get garbage collected after GPIO.cleanup()
        # below, and PWM.__del__ would then try to stop() an
        # already-closed GPIO handle.
        for name in list(self.pwm_objects.keys()):
            self.pwm_objects.pop(name).stop()
        GPIO.cleanup()


def main():
    """Test motor controller."""
    controller = None
    try:
        controller = MotorController()

        logger.info("Testing forward movement...")
        controller.move_forward(150, duration=2)
        time.sleep(1)

        logger.info("Testing backward movement...")
        controller.move_backward(150, duration=2)
        time.sleep(1)

        logger.info("Testing left pivot...")
        controller.turn_left(150, duration=1)
        time.sleep(1)

        logger.info("Testing right pivot...")
        controller.turn_right(150, duration=1)
        time.sleep(1)

        controller.stop()
        logger.info("All tests passed!")

    except KeyboardInterrupt:
        logger.info("Test interrupted")
    finally:
        if controller:
            controller.cleanup()


if __name__ == "__main__":
    main()
