"""
IMU Driver Module

MPU6050 Inertial Measurement Unit (6-axis accelerometer + gyroscope).
Provides orientation, angular velocity, and linear acceleration data.

I2C Connection (MPU6050):
- Address: 0x68 (default) or 0x69 (if AD0 pin tied high)
- SCL: GPIO3 (I2C Clock)
- SDA: GPIO2 (I2C Data)
"""

import logging
import math
import numpy as np
from typing import Tuple, Optional
import time
import threading

# Try to import MPU6050 library
try:
    from adafruit_mpu6050 import MPU6050
    import board
    import busio
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Background heading tracking (start_tracking): same scheme as Robot B's
# ESP32 firmware - integrate at 100 Hz behind a 44 Hz low-pass filter and
# drop readings past the +-250 deg/s full scale as motor-noise glitches.
TRACK_RATE_HZ = 100
GYRO_SPIKE_DPS = 250.0


class _SmbusMPU6050:
    """
    Minimal MPU6050 reader over smbus2, with the same .gyro (rad/s),
    .acceleration (m/s^2) and .temperature interface as adafruit_mpu6050.
    Used when the Adafruit/Blinka stack isn't installed (e.g. the Pi 5).
    """

    def __init__(self, address: int, bus: int = 1):
        from smbus2 import SMBus
        self.address = address
        self.bus = SMBus(bus)
        self.bus.write_byte_data(address, 0x6B, 0x00)  # wake up
        self.bus.write_byte_data(address, 0x1A, 0x03)  # 44 Hz low-pass filter, cuts motor vibration
        self.bus.write_byte_data(address, 0x1B, 0x00)  # gyro +-250 deg/s (131 LSB per deg/s)
        self.bus.write_byte_data(address, 0x1C, 0x00)  # accel +-2 g (16384 LSB per g)
        time.sleep(0.1)

    def _read_xyz(self, register: int):
        d = self.bus.read_i2c_block_data(self.address, register, 6)
        return [int.from_bytes(bytes(d[i:i + 2]), "big", signed=True) for i in (0, 2, 4)]

    @property
    def gyro(self):
        return tuple(math.radians(v / 131.0) for v in self._read_xyz(0x43))

    @property
    def acceleration(self):
        return tuple(v / 16384.0 * 9.80665 for v in self._read_xyz(0x3B))

    @property
    def temperature(self):
        d = self.bus.read_i2c_block_data(self.address, 0x41, 2)
        return int.from_bytes(bytes(d), "big", signed=True) / 340.0 + 36.53

    def close(self):
        self.bus.close()


class IMUDriver:
    """
    MPU6050 IMU driver for orientation and motion tracking.
    
    Features:
    - Accelerometer (3-axis)
    - Gyroscope (3-axis)
    - Temperature sensor
    - Heading estimation via gyroscope integration
    - Simulation mode for testing without hardware
    """
    
    def __init__(
        self,
        i2c_address: int = 0x68,
        accel_scale: float = 9.81,  # m/s^2
        gyro_scale: float = 1.0,    # degrees/s
        gyro_z_sign: float = 1.0,
    ):
        """
        Initialize IMU driver.

        Args:
            i2c_address: I2C address of MPU6050 (0x68 or 0x69)
            accel_scale: Acceleration scale in m/s^2
            gyro_scale: Gyroscope scale in degrees/s
            gyro_z_sign: -1 if the board is mounted upside down, so that a
                left (counter-clockwise) turn still increases the heading
        """
        self.i2c_address = i2c_address
        self.accel_scale = accel_scale
        self.gyro_scale = gyro_scale
        self.gyro_z_sign = gyro_z_sign

        self._bus_lock = threading.Lock()      # I2C reads from the tracking thread and callers
        self._heading_lock = threading.Lock()
        self._tracking = False
        self._track_thread = None
        
        self.mpu = None
        self.heading = 0.0  # Current heading in degrees (0-360)
        self.last_update_time = time.time()
        
        # Offsets for calibration
        self.gyro_offset_x = 0.0
        self.gyro_offset_y = 0.0
        self.gyro_offset_z = 0.0

        # Rate-limit I2C-error logging (motor operation can cause frequent
        # transient read failures - see get_gyroscope()/get_acceleration())
        self._last_i2c_error_log = 0.0

        self._initialize()
    
    def _initialize(self):
        """Initialize I2C communication with MPU6050."""
        try:
            if HARDWARE_AVAILABLE:
                i2c = busio.I2C(board.SCL, board.SDA)
                self.mpu = MPU6050(i2c, address=self.i2c_address)
            else:
                self.mpu = _SmbusMPU6050(self.i2c_address)
            logger.info(f"MPU6050 initialized at address 0x{self.i2c_address:02x}")

        except Exception as e:
            logger.error(f"Failed to initialize MPU6050: {e}")
            logger.warning("Running in simulation mode without physical IMU")
            self.mpu = None
    
    def calibrate_gyroscope(self, num_samples: int = 100):
        """
        Calibrate gyroscope by measuring offsets at rest.
        
        Args:
            num_samples: Number of samples to average
        """
        logger.info(f"Calibrating gyroscope (keep sensor stationary)...")
        
        if self.mpu is None:
            logger.warning("No hardware IMU available for calibration")
            return
        
        gx_sum, gy_sum, gz_sum = 0, 0, 0
        
        for _ in range(num_samples):
            with self._bus_lock:
                gx, gy, gz = self.mpu.gyro
            gx_sum += gx
            gy_sum += gy
            gz_sum += gz
            time.sleep(0.01)
        
        self.gyro_offset_x = gx_sum / num_samples
        self.gyro_offset_y = gy_sum / num_samples
        self.gyro_offset_z = gz_sum / num_samples
        
        logger.info(f"Gyro offsets: ({self.gyro_offset_x:.3f}, "
                   f"{self.gyro_offset_y:.3f}, {self.gyro_offset_z:.3f})")
    
    def _log_i2c_error(self, e: Exception, context: str):
        """Rate-limited warning for I2C read failures (at most once/second) -
        these can happen frequently from motor electrical noise when polling
        the IMU during an active turn, and would otherwise spam the log."""
        now = time.time()
        if now - self._last_i2c_error_log > 1.0:
            logger.warning(f"{context} failed ({e}) - likely motor electrical noise, skipping this sample")
            self._last_i2c_error_log = now

    def get_acceleration(self) -> Tuple[float, float, float]:
        """
        Get acceleration in m/s^2.

        Returns:
            (ax, ay, az) tuple
        """
        if self.mpu is None:
            # Simulation
            return (0.0, 0.0, self.accel_scale)

        try:
            # adafruit_mpu6050's .acceleration is already in m/s^2
            with self._bus_lock:
                return self.mpu.acceleration
        except OSError as e:
            self._log_i2c_error(e, "Accelerometer read")
            return (0.0, 0.0, 0.0)

    def get_gyroscope(self) -> Tuple[float, float, float]:
        """
        Get angular velocity in degrees/s.

        Returns:
            (gx, gy, gz) tuple - rotations around x, y, z axes
        """
        if self.mpu is None:
            # Simulation
            return (0.0, 0.0, 0.0)

        reading = self._read_gyro_dps()
        return reading if reading is not None else (0.0, 0.0, 0.0)

    def _read_gyro_dps(self) -> Optional[Tuple[float, float, float]]:
        """Calibrated angular velocity in deg/s, or None if the I2C read failed."""
        try:
            with self._bus_lock:
                gx, gy, gz = self.mpu.gyro
        except OSError as e:
            self._log_i2c_error(e, "Gyroscope read")
            return None

        # .gyro is in rad/s - apply calibration offsets (also in rad/s)
        # before converting to deg/s
        gx = math.degrees(gx - self.gyro_offset_x) * self.gyro_scale
        gy = math.degrees(gy - self.gyro_offset_y) * self.gyro_scale
        gz = math.degrees(gz - self.gyro_offset_z) * self.gyro_scale * self.gyro_z_sign

        return (gx, gy, gz)
    
    def get_temperature(self) -> Optional[float]:
        """
        Get temperature in Celsius.
        
        Returns:
            Temperature in degrees Celsius
        """
        if self.mpu is None:
            return 25.0  # Simulation

        with self._bus_lock:
            return self.mpu.temperature

    @property
    def reliable_during_motion(self) -> bool:
        """True while the background thread tracks heading (valid even mid-move)."""
        return self._tracking

    def start_tracking(self):
        """
        Integrate the gyro in a background thread at TRACK_RATE_HZ, so the
        heading stays correct while the control loop is blocked in a move.
        Call after calibrate_gyroscope(), with the robot still.
        """
        if self.mpu is None or self._tracking:
            return
        self._tracking = True
        self._track_thread = threading.Thread(target=self._track_loop, name="imu-heading", daemon=True)
        self._track_thread.start()
        logger.info(f"IMU heading tracking at {TRACK_RATE_HZ} Hz")

    def _track_loop(self):
        period = 1.0 / TRACK_RATE_HZ
        last = time.monotonic()
        last_gz = 0.0
        while self._tracking:
            time.sleep(period)
            now = time.monotonic()
            dt, last = now - last, now
            reading = self._read_gyro_dps()
            if reading is not None and abs(reading[2]) <= GYRO_SPIKE_DPS:
                last_gz = reading[2]
            # a failed or glitched sample reuses the previous rate rather than
            # dropping that slice of rotation
            with self._heading_lock:
                self.heading = (self.heading + last_gz * dt) % 360

    def update_heading(self):
        """
        Update heading estimate using gyroscope integration.
        Call this periodically (e.g., 50 Hz) for accurate heading.
        No-op while start_tracking()'s thread is integrating.
        """
        if self._tracking:
            return
        current_time = time.time()
        dt = current_time - self.last_update_time
        
        if dt <= 0:
            return
        
        # Get angular velocity around Z axis (yaw)
        _, _, gz = self.get_gyroscope()
        
        # Integrate to get heading change
        self.heading += gz * dt
        
        # Normalize to 0-360
        self.heading = self.heading % 360
        
        self.last_update_time = current_time
    
    def get_heading(self) -> float:
        """
        Get estimated heading in degrees (0-360).
        
        Returns:
            Heading angle in degrees
        """
        return self.heading
    
    def reset_heading(self, angle: float = 0.0):
        """Reset heading to specific angle."""
        with self._heading_lock:
            self.heading = angle % 360
        self.last_update_time = time.time()
    
    def get_all_data(self) -> dict:
        """Get all sensor data at once."""
        ax, ay, az = self.get_acceleration()
        gx, gy, gz = self.get_gyroscope()
        
        return {
            'accel_x': ax,
            'accel_y': ay,
            'accel_z': az,
            'gyro_x': gx,
            'gyro_y': gy,
            'gyro_z': gz,
            'heading': self.heading,
            'temperature': self.get_temperature()
        }
    
    def cleanup(self):
        """Clean up IMU driver."""
        self._tracking = False
        if self._track_thread:
            self._track_thread.join(timeout=1)
        if isinstance(self.mpu, _SmbusMPU6050):
            self.mpu.close()
        logger.info("IMU driver cleanup complete")


def main():
    """Test IMU driver."""
    imu = IMUDriver()
    
    try:
        print("Calibrating gyroscope...")
        imu.calibrate_gyroscope()
        
        print("\nTesting IMU sensor data...")
        for i in range(20):
            data = imu.get_all_data()
            imu.update_heading()
            
            print(f"\n--- Reading {i+1} ---")
            print(f"Accel: ({data['accel_x']:.2f}, {data['accel_y']:.2f}, {data['accel_z']:.2f}) m/s²")
            print(f"Gyro:  ({data['gyro_x']:.2f}, {data['gyro_y']:.2f}, {data['gyro_z']:.2f}) °/s")
            print(f"Heading: {data['heading']:.1f}°")
            print(f"Temp: {data['temperature']:.1f}°C")
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("Test interrupted")
    
    finally:
        imu.cleanup()


if __name__ == "__main__":
    main()
