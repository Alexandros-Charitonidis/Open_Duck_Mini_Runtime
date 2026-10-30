import time

import numpy as np
import rustypot
from mini_bdx_runtime.duck_config import DuckConfig


class HWI:
    def __init__(self, duck_config: DuckConfig, usb_port: str = "/dev/ttyACM0"):

        self.duck_config = duck_config

        # Order matters here
        self.joints = {
            "left_hip_yaw": 20,
            "left_hip_roll": 21,
            "left_hip_pitch": 22,
            "left_knee": 23,
            "left_ankle": 24,
            "neck_pitch": 30,
            "head_pitch": 31,
            "head_yaw": 32,
            "head_roll": 33,
            # "left_antenna": None,
            # "right_antenna": None,
            "right_hip_yaw": 10,
            "right_hip_roll": 11,
            "right_hip_pitch": 12,
            "right_knee": 13,
            "right_ankle": 14,
        }

        self.zero_pos = {
            "left_hip_yaw": 0,
            "left_hip_roll": 0,
            "left_hip_pitch": 0,
            "left_knee": 0,
            "left_ankle": 0,
            "neck_pitch": 0,
            "head_pitch": 0,
            "head_yaw": 0,
            "head_roll": 0,
            # "left_antenna":0,
            # "right_antenna":0,
            "right_hip_yaw": 0,
            "right_hip_roll": 0,
            "right_hip_pitch": 0,
            "right_knee": 0,
            "right_ankle": 0,
        }

        self.init_pos = {
            "left_hip_yaw": 0.002,
            "left_hip_roll": 0.053,
            "left_hip_pitch": -0.63,
            "left_knee": 1.368,
            "left_ankle": -0.784,
            "neck_pitch": 0.0,
            "head_pitch": 0.0,
            "head_yaw": 0,
            "head_roll": 0,
            # "left_antenna": 0,
            # "right_antenna": 0,
            "right_hip_yaw": -0.003,
            "right_hip_roll": -0.065,
            "right_hip_pitch": 0.635,
            "right_knee": 1.379,
            "right_ankle": -0.796,
        }

        self.joints_offsets = self.duck_config.joints_offset

        self.kps = np.ones(len(self.joints)) * 32  # default kp
        self.kds = np.ones(len(self.joints)) * 0  # default kd
        self.low_torque_kps = np.ones(len(self.joints)) * 2

        self.usb_port = usb_port
        self.io = rustypot.feetech(usb_port, 1000000)
        self.bus_errors = 0

    def _bus_error(self, e):
        """
        A late reply (e.g. a timeout on the first read after a pause) leaves
        stale bytes in the serial input buffer, and every later rustypot
        transfer then fails ("Parsing error") or panics (PanicException, a
        BaseException). Reopening alone does not clear them: wait for late
        bytes to land, flush the input buffer, then reopen.
        """
        if isinstance(e, (KeyboardInterrupt, SystemExit)):
            raise e
        self.bus_errors += 1
        if self.bus_errors <= 5 or self.bus_errors % 50 == 0:
            print(f"[HWI] bus error #{self.bus_errors} ({type(e).__name__}: {e}), flushing port")
        self.io = None  # closes the port (rustypot opens it exclusively)
        try:
            import serial

            with serial.Serial(self.usb_port, 1000000, timeout=0) as ser:
                time.sleep(0.005)
                ser.reset_input_buffer()
                if self.bus_errors <= 3:
                    self._capture_sync_read(ser)
        except Exception as flush_error:
            print(f"[HWI] flush failed: {flush_error}")
        self.io = rustypot.feetech(self.usb_port, 1000000)

    def _capture_sync_read(self, ser):
        """
        Diagnostic: send the same SYNC_READ of present position that
        rustypot uses and print every status packet in the reply.
        """
        ids = list(self.joints.values())
        body = [0xFE, len(ids) + 4, 0x82, 56, 2, *ids]
        ser.reset_input_buffer()
        ser.write(bytes([0xFF, 0xFF, *body, ~sum(body) & 0xFF]))
        ser.flush()
        time.sleep(0.01)
        raw = ser.read(1024)
        print(f"[HWI capture] {len(raw)} bytes (expect {8 * len(ids)}): {raw.hex(' ')}")
        i = 0
        seen = []
        while i + 4 <= len(raw):
            if raw[i] != 0xFF or raw[i + 1] != 0xFF:
                print(f"[HWI capture]   junk byte at {i}: 0x{raw[i]:02X}")
                i += 1
                continue
            id, length = raw[i + 2], raw[i + 3]
            pkt = raw[i : i + 4 + length]
            if len(pkt) < 4 + length:
                print(f"[HWI capture]   truncated packet id={id}: {pkt.hex(' ')}")
                break
            chk_ok = (~sum(pkt[2:-1]) & 0xFF) == pkt[-1]
            err = pkt[4]
            if err or not chk_ok or length != 4:
                print(f"[HWI capture]   id={id} len={length} error_byte=0x{err:02X} checksum_ok={chk_ok}")
            seen.append(id)
            i += 4 + length
        missing = [x for x in ids if x not in seen]
        order_ok = seen == ids[: len(seen)]
        print(f"[HWI capture]   packets from {len(seen)} servos, missing {missing}, order_ok={order_ok}")

    def set_kps(self, kps):
        self.kps = kps
        self.io.set_kps(list(self.joints.values()), self.kps)

    def set_kds(self, kds):
        self.kds = kds
        self.io.set_kds(list(self.joints.values()), self.kds)

    def set_kp(self, id, kp):
        self.io.set_kps([id], [kp])

    def turn_on(self):
        self.io.set_kps(list(self.joints.values()), self.low_torque_kps)
        print("turn on : low KPS set")
        time.sleep(1)

        self.set_position_all(self.init_pos)
        print("turn on : init pos set")

        time.sleep(1)

        self.io.set_kps(list(self.joints.values()), self.kps)
        print("turn on : high kps")

    def turn_off(self):
        self.io.disable_torque(list(self.joints.values()))

    def set_position(self, joint_name, pos):
        """
        pos is in radians
        """
        id = self.joints[joint_name]
        pos = pos + self.joints_offsets[joint_name]
        self.io.write_goal_position([id], [pos])

    def set_position_all(self, joints_positions):
        """
        joints_positions is a dictionary with joint names as keys and joint positions as values
        Warning: expects radians
        """
        ids_positions = {
            self.joints[joint]: position + self.joints_offsets[joint]
            for joint, position in joints_positions.items()
        }

        try:
            self.io.write_goal_position(
                list(self.joints.values()), list(ids_positions.values())
            )
        except BaseException as e:
            self._bus_error(e)

    def get_present_positions(self, ignore=[]):
        """
        Returns the present positions in radians
        """

        try:
            present_positions = self.io.read_present_position(
                list(self.joints.values())
            )
        except BaseException as e:
            self._bus_error(e)
            return None

        present_positions = [
            pos - self.joints_offsets[joint]
            for joint, pos in zip(self.joints.keys(), present_positions)
            if joint not in ignore
        ]
        return np.array(np.around(present_positions, 3))

    def get_present_velocities(self, rad_s=True, ignore=[]):
        """
        Returns the present velocities in rad/s (default) or rev/min
        """
        try:
            present_velocities = self.io.read_present_velocity(
                list(self.joints.values())
            )
        except BaseException as e:
            self._bus_error(e)
            return None

        present_velocities = [
            vel
            for joint, vel in zip(self.joints.keys(), present_velocities)
            if joint not in ignore
        ]

        return np.array(np.around(present_velocities, 3))
