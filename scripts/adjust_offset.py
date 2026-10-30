"""
Re-measure the offset of a single joint, keeping the other joints'
offsets from duck_config.json so they can serve as a reference.

The robot moves to its zero pose (all joints at 0, legs straight) at low
stiffness, then the chosen joint goes limp so you can put it at its true
zero by hand. The offset is the raw servo reading at that point.

Ctrl+C at any time turns all motors off.
"""

import argparse
import json
import os
import shutil
import time

import numpy as np
from mini_bdx_runtime.duck_config import DuckConfig
from mini_bdx_runtime.rustypot_position_hwi import HWI

HOME_DIR = os.path.expanduser("~")

parser = argparse.ArgumentParser()
parser.add_argument("--joint", type=str, default="right_hip_pitch")
parser.add_argument("--duck_config_path", type=str, default=f"{HOME_DIR}/duck_config.json")
args = parser.parse_args()

cfg = DuckConfig(config_json_path=args.duck_config_path)
hwi = HWI(cfg)
joint = args.joint
if joint not in hwi.joints:
    raise SystemExit(f"Unknown joint {joint}, choose from {list(hwi.joints)}")
i = list(hwi.joints).index(joint)
joint_id = hwi.joints[joint]
old_offset = hwi.joints_offsets[joint]


def read_joint():
    for _ in range(10):
        pos = hwi.get_present_positions()
        if pos is not None:
            return pos[i]
        time.sleep(0.05)
    raise RuntimeError("Could not read servo positions")


print(f"\nAdjusting {joint} (id {joint_id}), current offset {old_offset:.3f} rad")
input(
    "\nThe duck will move to its ZERO pose (legs straight) at low stiffness.\n"
    "Hold it in the air or on a stand. Press ENTER to start, Ctrl+C to abort..."
)

new_offset = None
try:
    hwi.init_pos = hwi.zero_pos
    hwi.set_kds([0] * len(hwi.joints))
    hwi.turn_on()
    time.sleep(0.5)

    while True:
        hwi.io.disable_torque([joint_id])
        input(
            f"\n{joint} is now LIMP. Move it by hand to its true zero "
            "(for a hip pitch: thigh straight, parallel to the other leg),\n"
            "hold it there and press ENTER..."
        )
        raw = read_joint() + hwi.joints_offsets[joint]
        candidate = round(float(raw), 3)
        print(
            f"  old offset {old_offset:+.3f} rad, new offset {candidate:+.3f} rad "
            f"(change {np.degrees(candidate - old_offset):+.1f} deg)"
        )

        hwi.joints_offsets[joint] = candidate
        hwi.set_position_all(hwi.zero_pos)
        input("Keep holding it lightly. Press ENTER, the joint will hold its new zero, then let go...")
        hwi.io.enable_torque([joint_id])
        time.sleep(0.5)

        res = input("Does it match the other side now? (y = save / r = redo / q = quit without saving) ").lower()
        if res == "y":
            new_offset = candidate
            break
        if res == "q":
            break
        hwi.joints_offsets[joint] = old_offset
finally:
    hwi.turn_off()
    print("Motors turned off")

if new_offset is not None:
    backup = args.duck_config_path + ".bak." + time.strftime("%Y%m%d-%H%M%S")
    shutil.copy(args.duck_config_path, backup)
    data = json.load(open(args.duck_config_path))
    data["joints_offsets"][joint] = new_offset
    json.dump(data, open(args.duck_config_path, "w"), indent=4)
    print(f"Saved {joint} offset {new_offset} to {args.duck_config_path} (backup {backup})")
else:
    print("Nothing saved")
