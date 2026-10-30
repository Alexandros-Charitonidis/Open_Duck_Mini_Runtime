"""
Driver for the DFRobot 10DOF IMU v2.0 (SEN0140): ADXL345 accelerometer +
ITG-3205 gyro. The magnetometer and barometer are not used.

Drop-in replacement for raw_imu.Imu: get_data() returns
    {"gyro": rad/s, "accelero": m/s^2 (gravity included, ~+9.81 on z upright)}
expressed in the robot frame (x forward, y left, z up).

The chips' axes are mapped to the robot frame with an axis map such as
["-y", "x", "z"], meaning robot_x = -chip_y, robot_y = chip_x, robot_z = chip_z.
Use scripts/check_imu_orientation.py to find the right maps.
"""

import time
from queue import Queue
from threading import Thread

import numpy as np
from Adafruit_PureIO.smbus import SMBus  # plain /dev/i2c-N, no blinka/gpiod needed

G = 9.80665

ADXL345_ADDR = 0x53
ADXL345_DEVID = 0x00  # expect 0xE5
ADXL345_BW_RATE = 0x2C
ADXL345_POWER_CTL = 0x2D
ADXL345_DATA_FORMAT = 0x31
ADXL345_DATAX0 = 0x32
ADXL345_SCALE = 0.0039 * G  # full resolution: 3.9 mg/LSB

ITG3205_ADDRS = [0x68, 0x69]  # depends on AD0 strap
ITG3205_WHO_AM_I = 0x00
ITG3205_SMPLRT_DIV = 0x15
ITG3205_DLPF_FS = 0x16
ITG3205_GYRO_XOUT_H = 0x1D
ITG3205_PWR_MGM = 0x3E
ITG3205_SCALE = np.deg2rad(1 / 14.375)  # LSB -> rad/s


def axis_map_matrix(axis_map):
    """["-y", "x", "z"] -> 3x3 matrix M such that robot = M @ chip."""
    M = np.zeros((3, 3))
    for row, spec in enumerate(axis_map):
        spec = spec.strip().lower()
        sign = -1.0 if spec.startswith("-") else 1.0
        M[row, "xyz".index(spec.lstrip("+-"))] = sign
    if abs(np.linalg.det(M)) != 1.0:
        raise ValueError(f"Invalid IMU axis map {axis_map}")
    if np.linalg.det(M) < 0:
        print(f"[IMU] WARNING axis map {axis_map} is left-handed (mirrored)")
    return M


class Imu:
    def __init__(
        self,
        sampling_freq,
        user_pitch_bias=0,
        calibrate=False,
        upside_down=False,
        accel_axes=("x", "y", "z"),
        gyro_axes=("x", "y", "z"),
        gyro_bias_samples=200,
        i2c_bus=1,
    ):
        # user_pitch_bias, calibrate and upside_down are accepted for
        # compatibility with raw_imu.Imu; orientation is handled by the axis maps.
        self.sampling_freq = sampling_freq
        self.accel_M = axis_map_matrix(accel_axes)
        self.gyro_M = axis_map_matrix(gyro_axes)

        self.bus = SMBus(i2c_bus)
        try:
            devid = self.bus.read_byte_data(ADXL345_ADDR, ADXL345_DEVID)
        except OSError:
            raise RuntimeError(
                f"ADXL345 not found at 0x{ADXL345_ADDR:02X} on /dev/i2c-{i2c_bus}, "
                f"check wiring with i2cdetect -y {i2c_bus}"
            )
        if devid != 0xE5:
            raise RuntimeError(f"ADXL345 DEVID is 0x{devid:02X}, expected 0xE5")
        self.gyro_addr = self._find_gyro(i2c_bus)

        # ADXL345: 100 Hz output (50 Hz bandwidth), full resolution +-16 g, measure
        self.bus.write_byte_data(ADXL345_ADDR, ADXL345_BW_RATE, 0x0A)
        self.bus.write_byte_data(ADXL345_ADDR, ADXL345_DATA_FORMAT, 0x0B)
        self.bus.write_byte_data(ADXL345_ADDR, ADXL345_POWER_CTL, 0x08)

        # ITG-3205: PLL with X gyro reference, +-2000 deg/s, 42 Hz DLPF
        # (1 kHz internal), 1 kHz / (1 + 7) = 125 Hz output
        self.bus.write_byte_data(self.gyro_addr, ITG3205_PWR_MGM, 0x01)
        self.bus.write_byte_data(self.gyro_addr, ITG3205_DLPF_FS, 0x18 | 0x03)
        self.bus.write_byte_data(self.gyro_addr, ITG3205_SMPLRT_DIV, 0x07)
        time.sleep(0.1)  # PLL settling

        self.gyro_bias = np.zeros(3)
        self.gyro_bias = self._estimate_gyro_bias(gyro_bias_samples)

        self.last_imu_data = {
            "gyro": np.zeros(3),
            "accelero": np.array([0.0, 0.0, G]),
        }
        self.imu_queue = Queue(maxsize=1)
        Thread(target=self.imu_worker, daemon=True).start()

    def _find_gyro(self, i2c_bus):
        for addr in ITG3205_ADDRS:
            try:
                who_am_i = self.bus.read_byte_data(addr, ITG3205_WHO_AM_I)
            except OSError:
                continue
            if (who_am_i & 0x7E) == 0x68:
                return addr
        raise RuntimeError(
            f"ITG-3205 gyro not found at 0x68/0x69, check wiring with i2cdetect -y {i2c_bus}"
        )

    def read_raw(self):
        """Chip-frame (accel m/s^2, gyro rad/s), gyro bias removed, no axis map."""
        raw = self.bus.read_i2c_block_data(ADXL345_ADDR, ADXL345_DATAX0, 6)
        accel = np.frombuffer(bytes(raw), dtype="<i2").astype(float) * ADXL345_SCALE

        raw = self.bus.read_i2c_block_data(self.gyro_addr, ITG3205_GYRO_XOUT_H, 6)
        gyro = np.frombuffer(bytes(raw), dtype=">i2").astype(float) * ITG3205_SCALE

        return accel, gyro - self.gyro_bias

    def _estimate_gyro_bias(self, n, max_std=0.02, attempts=5):
        """Average the gyro while the duck is still. Keep it still at startup!"""
        print("[IMU] Estimating gyro bias, keep the duck still...")
        for _ in range(attempts):
            samples = []
            for _ in range(n):
                samples.append(self.read_raw()[1])
                time.sleep(1 / 125)
            samples = np.array(samples)
            if samples.std(axis=0).max() < max_std:
                bias = samples.mean(axis=0)
                print(f"[IMU] Gyro bias (rad/s): {np.around(bias, 4)}")
                return bias
            print("[IMU] Motion detected during gyro bias estimation, retrying")
        print("[IMU] WARNING: could not get a still gyro bias, using last estimate")
        return samples.mean(axis=0)

    def imu_worker(self):
        while True:
            s = time.time()
            try:
                accel, gyro = self.read_raw()
            except OSError as e:
                print("[IMU]:", e)
                time.sleep(0.01)
                continue

            data = {
                "gyro": self.gyro_M @ gyro,
                "accelero": self.accel_M @ accel,
            }

            self.imu_queue.put(data)
            took = time.time() - s
            time.sleep(max(0, 1 / self.sampling_freq - took))

    def get_data(self):
        try:
            self.last_imu_data = self.imu_queue.get(False)  # non blocking
        except Exception:
            pass

        return self.last_imu_data


if __name__ == "__main__":
    from mini_bdx_runtime.duck_config import DuckConfig

    cfg = DuckConfig(ignore_default=True)
    imu = Imu(50, accel_axes=cfg.imu_accel_axes, gyro_axes=cfg.imu_gyro_axes)
    while True:
        data = imu.get_data()
        accel = data["accelero"]
        print("gyro", np.around(data["gyro"], 3))
        print("accelero", np.around(accel, 3), "|g|", np.around(np.linalg.norm(accel), 2))
        print("---")
        time.sleep(1 / 25)
