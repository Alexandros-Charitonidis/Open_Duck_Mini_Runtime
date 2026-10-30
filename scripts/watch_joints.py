"""
Read-only: print live joint positions (offsets applied, radians) while you
move the joints by hand. Never enables torque or sends goal positions.
Run turn_off.py first if the motors are on.
"""

import argparse
import time

from mini_bdx_runtime.duck_config import DuckConfig
from mini_bdx_runtime.rustypot_position_hwi import HWI

parser = argparse.ArgumentParser()
parser.add_argument(
    "--joints",
    nargs="+",
    default=["left_hip_pitch", "right_hip_pitch", "left_knee", "right_knee"],
)
parser.add_argument(
    "--find",
    action="store_true",
    help="move one joint by hand at a time, prints which servo moved",
)
args = parser.parse_args()

hwi = HWI(DuckConfig())
names = list(hwi.joints)
idx = [names.index(j) for j in args.joints]


def find_mode():
    print("Move ONE joint by hand, then hold it. Ctrl+C to exit.")
    rest = None
    last_report = None
    while True:
        pos = hwi.get_present_positions()
        if pos is None:
            continue
        if rest is None:
            rest = pos.copy()
        delta = pos - rest
        i = int(abs(delta).argmax())
        if abs(delta[i]) > 0.15:  # ~9 deg
            report = (names[i], hwi.joints[names[i]], "+" if delta[i] > 0 else "-")
            if report != last_report:
                print(f"moved: {report[0]:16s} (id {report[1]:2d})  direction {report[2]}  ({delta[i]:+.2f} rad)")
                last_report = report
        elif last_report is not None and abs(delta).max() < 0.05:
            last_report = None  # back at rest, ready for the next joint
        time.sleep(0.1)


if args.find:
    try:
        find_mode()
    except KeyboardInterrupt:
        print()
    raise SystemExit

print("   ".join(f"{j:>16s}" for j in args.joints))
try:
    while True:
        pos = hwi.get_present_positions()
        if pos is not None:
            print("   ".join(f"{pos[i]:+16.3f}" for i in idx), end="\r", flush=True)
        time.sleep(0.1)
except KeyboardInterrupt:
    print()
