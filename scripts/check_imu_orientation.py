"""
Guided procedure to find the axis maps of the DFRobot 10DOF IMU, i.e. how the
chip axes relate to the robot frame (x forward, y left, z up).

Does not move any motor. Mount the IMU in the duck (or hold the board the way
it will be mounted) and follow the prompts.
"""

import json
import os
import shutil
import time

import numpy as np
from mini_bdx_runtime.dfrobot_10dof_imu import Imu

CONFIG_PATH = os.path.expanduser("~/duck_config.json")


def average_accel(imu, duration=1.5):
    samples = []
    t0 = time.time()
    while time.time() - t0 < duration:
        samples.append(imu.read_raw()[0])
        time.sleep(0.01)
    return np.mean(samples, axis=0)


def integrate_gyro(imu, duration=4.0):
    total = np.zeros(3)
    last = time.time()
    t0 = last
    while time.time() - t0 < duration:
        gyro = imu.read_raw()[1]
        now = time.time()
        total += gyro * (now - last)
        last = now
        time.sleep(0.005)
    return total


def dominant_axis(v, label):
    """Unit vector along the chip axis carrying most of v, with its sign."""
    i = int(np.argmax(np.abs(v)))
    share = abs(v[i]) / np.linalg.norm(v)
    if share < 0.85:
        print(f"  WARNING: {label} is not well aligned with one chip axis "
              f"({np.around(v, 2)}), is the board mounted at an angle?")
    u = np.zeros(3)
    u[i] = np.sign(v[i])
    return u


def to_axes(M):
    """3x3 signed permutation matrix -> ["-y", "x", "z"] style map."""
    axes = []
    for row in M:
        i = int(np.argmax(np.abs(row)))
        axes.append(("-" if row[i] < 0 else "") + "xyz"[i])
    return axes


def prompt(msg):
    input(f"\n{msg}\nPress ENTER when ready...")


imu = Imu(50)  # identity maps, read_raw() gives chip frame

print("\n=== Accelerometer ===")
prompt("1/4  Put the duck UPRIGHT and still, as if standing.")
a_up = average_accel(imu)
print(f"  chip accel: {np.around(a_up, 2)}  |g| = {np.linalg.norm(a_up):.2f} (expect ~9.81)")

prompt("2/4  Tilt the duck NOSE DOWN by 90 degrees (face pointing at the floor) and hold still.")
a_nose = average_accel(imu)
print(f"  chip accel: {np.around(a_nose, 2)}")

# Upright: accel reads +g on robot z. Nose down: robot x points down, reads -g.
acc_z = dominant_axis(a_up, "upright gravity")
acc_x = -dominant_axis(a_nose, "nose-down gravity")
acc_y = np.cross(acc_z, acc_x)
accel_M = np.array([acc_x, acc_y, acc_z])

print("\n=== Gyro ===")
prompt("3/4  Put the duck upright. After ENTER, TURN it LEFT (counter-clockwise seen "
       "from above) by about 90 degrees within 4 seconds.")
g_yaw = integrate_gyro(imu)
print(f"  integrated chip gyro (rad): {np.around(g_yaw, 2)}")

prompt("4/4  Put the duck upright. After ENTER, tilt it NOSE DOWN by about 90 degrees "
       "within 4 seconds.")
g_pitch = integrate_gyro(imu)
print(f"  integrated chip gyro (rad): {np.around(g_pitch, 2)}")

# Turning left is a positive rotation about z; nose down is a positive rotation about y (left).
gyr_z = dominant_axis(g_yaw, "yaw rotation")
gyr_y = dominant_axis(g_pitch, "pitch rotation")
gyr_x = np.cross(gyr_y, gyr_z)
gyro_M = np.array([gyr_x, gyr_y, gyr_z])

for name, M in [("accel", accel_M), ("gyro", gyro_M)]:
    if abs(np.linalg.det(M)) < 0.5:
        print(f"\nERROR: {name} axes are inconsistent (two poses hit the same chip axis). "
              "Redo the procedure more carefully.")
        exit(1)

accel_axes = to_axes(accel_M)
gyro_axes = to_axes(gyro_M)

print("\n=== Result ===")
print(f'  "imu_type": "dfrobot_10dof",')
print(f'  "imu_accel_axes": {json.dumps(accel_axes)},')
print(f'  "imu_gyro_axes": {json.dumps(gyro_axes)}')

print(f"\nSanity check, robot frame upright accel: {np.around(accel_M @ a_up, 2)} (expect ~[0, 0, +9.8])")
print(f"Sanity check, robot frame turn-left gyro: {np.around(gyro_M @ g_yaw, 2)} (expect ~[0, 0, +1.6])")

res = input(f"\nWrite these to {CONFIG_PATH}? A backup is made first. (y/N) ")
if res.lower() == "y":
    backup = CONFIG_PATH + ".bak." + time.strftime("%Y%m%d-%H%M%S")
    shutil.copy(CONFIG_PATH, backup)
    cfg = json.load(open(CONFIG_PATH))
    cfg["imu_type"] = "dfrobot_10dof"
    cfg["imu_accel_axes"] = accel_axes
    cfg["imu_gyro_axes"] = gyro_axes
    json.dump(cfg, open(CONFIG_PATH, "w"), indent=4)
    print(f"Written. Backup at {backup}")
