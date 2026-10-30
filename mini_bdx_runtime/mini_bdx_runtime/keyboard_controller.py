"""
Terminal keyboard controller, drop-in replacement for XBoxController.

Reads keys from the controlling terminal (works over SSH or on a local
console), no pygame / X server needed. Terminals do not report key releases,
so velocities are latched setpoints: each tap steps the command, SPACE zeroes
everything.

    Walk mode                       Head mode (h to toggle)
    w / s   forward / backward      w / s   head pitch
    a / d   strafe left / right     a / d   head yaw
    q / e   turn left / right       q / e   head roll
    SPACE   STOP (zero all commands)
    p / ENTER  pause / unpause   (A)
    r       sprint on/off         (LB, held)
    + / -   phase frequency offset (dpad up / down)
    b       random sound          (B)
    x       projector on/off      (X)
    ?       print this help
    Ctrl+C  quit
"""

import atexit
import os
import select
import sys
import termios
import time
import tty
from threading import Thread, Lock

import numpy as np
from mini_bdx_runtime.buttons import Buttons


X_RANGE = [-0.15, 0.15]
Y_RANGE = [-0.2, 0.2]
YAW_RANGE = [-1.0, 1.0]

# rads, kept conservative and symmetric
HEAD_PITCH_RANGE = [-0.3, 0.3]
HEAD_YAW_RANGE = [-0.5, 0.5]
HEAD_ROLL_RANGE = [-0.5, 0.5]

X_STEP = 0.05
Y_STEP = 0.05
YAW_STEP = 0.25
HEAD_STEP = 0.1


class KeyboardController:
    def __init__(self, command_freq=None, only_head_control=False):
        # command_freq kept for signature compatibility with XBoxController
        if not sys.stdin.isatty():
            raise RuntimeError("KeyboardController needs an interactive terminal")

        self.head_control_mode = only_head_control
        self.only_head_control = only_head_control

        # [lin_x, lin_y, yaw, neck_pitch, head_pitch, head_yaw, head_roll]
        self.last_commands = np.zeros(7)
        self.sprint = False
        self.lock = Lock()
        self.pending = set()  # one-shot button presses, consumed per get_last_command
        self.buttons = Buttons()

        self.fd = sys.stdin.fileno()
        self.old_term = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)  # keeps ISIG, so Ctrl+C still raises KeyboardInterrupt
        atexit.register(self.restore_terminal)

        print(__doc__)
        Thread(target=self.read_worker, daemon=True).start()

    def restore_terminal(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_term)

    def read_worker(self):
        while True:
            r, _, _ = select.select([self.fd], [], [], 0.1)
            if not r:
                continue
            data = os.read(self.fd, 32)
            if not data:  # EOF, terminal gone: stop moving
                with self.lock:
                    self.last_commands[:] = 0.0
                return
            for ch in data.decode(errors="ignore"):
                self.handle_key(ch)

    @staticmethod
    def step(value, delta, rng):
        return float(np.clip(round(value + delta, 3), rng[0], rng[1]))

    def handle_key(self, ch):
        c = self.last_commands
        with self.lock:
            if ch == " ":
                c[:] = 0.0
            elif ch in "wsadqe":
                sign = 1 if ch in "waq" else -1
                if self.head_control_mode:
                    if ch in "ws":
                        c[4] = self.step(c[4], sign * HEAD_STEP, HEAD_PITCH_RANGE)
                    elif ch in "ad":
                        c[5] = self.step(c[5], sign * HEAD_STEP, HEAD_YAW_RANGE)
                    else:
                        c[6] = self.step(c[6], sign * HEAD_STEP, HEAD_ROLL_RANGE)
                else:
                    if ch in "ws":
                        c[0] = self.step(c[0], sign * X_STEP, X_RANGE)
                    elif ch in "ad":
                        c[1] = self.step(c[1], sign * Y_STEP, Y_RANGE)
                    else:
                        c[2] = self.step(c[2], sign * YAW_STEP, YAW_RANGE)
            elif ch in "p\n\r":
                self.pending.add("A")
            elif ch == "b":
                self.pending.add("B")
            elif ch == "x":
                self.pending.add("X")
            elif ch == "h":
                if not self.only_head_control:
                    self.head_control_mode = not self.head_control_mode
                self.pending.add("Y")
            elif ch == "r":
                self.sprint = not self.sprint
            elif ch in "+=":
                self.pending.add("up")
            elif ch in "-_":
                self.pending.add("down")
            elif ch == "?":
                print(__doc__)
                return
            else:
                return
            mode = "HEAD" if self.head_control_mode else "WALK"
            sprint = " SPRINT" if self.sprint else ""
            print(
                f"[{mode}{sprint}] x={c[0]:+.2f} y={c[1]:+.2f} yaw={c[2]:+.2f} "
                f"head p={c[4]:+.2f} y={c[5]:+.2f} r={c[6]:+.2f}"
            )

    def get_last_command(self):
        with self.lock:
            pending, self.pending = self.pending, set()
            commands = np.around(self.last_commands.copy(), 3)
            sprint = self.sprint

        self.buttons.update(
            "A" in pending,
            "B" in pending,
            "X" in pending,
            "Y" in pending,
            sprint,
            False,
            "up" in pending,
            "down" in pending,
        )

        # No analog triggers on a keyboard; antennas stay at rest
        return commands, self.buttons, 0.0, 0.0


if __name__ == "__main__":
    controller = KeyboardController(20)
    try:
        while True:
            cmds, buttons, _, _ = controller.get_last_command()
            for name in ["A", "B", "X", "Y", "dpad_up", "dpad_down"]:
                if getattr(buttons, name).triggered:
                    print(f"{name} triggered")
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
