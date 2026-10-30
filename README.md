# Open Duck Mini Runtime

> **This is a fork for a modified duck** (branch `duck-cm5`). The upstream
> instructions follow below, unchanged. This section lists what differs.

## This duck: CM5 build

### Hardware differences from the reference build

| Part | This duck | Reference |
|---|---|---|
| Computer | Raspberry Pi CM5 on the official CM5 IO Board, Pi OS Trixie | Pi Zero 2W |
| Servos | Waveshare ST3215-HS (12V version, ~2x faster) | ST3215 7.4V |
| Servo power | Mean Well 12V / 10A | 2S battery |
| Serial adapter | CH343 on `/dev/ttyACM0` (no FTDI latency rule needed) | FTDI |
| IMU | DFRobot 10DOF v2.0 (ADXL345 + ITG-3205) as a stand-in | BNO055 |
| Foot switches | not fitted, contacts faked | fitted |
| Controller | terminal keyboard | Xbox controller |

Python 3.11 venv in `.venv`, managed by `uv`. Extra package: `gpiod`
(Adafruit Blinka needs it on Pi 5 / CM5).

### Files added or changed

| File | What it does |
|---|---|
| `mini_bdx_runtime/mini_bdx_runtime/keyboard_controller.py` | **New.** Terminal keyboard control, a drop-in replacement for the Xbox controller. Works over SSH. |
| `mini_bdx_runtime/mini_bdx_runtime/dfrobot_10dof_imu.py` | **New.** Driver for the DFRobot 10DOF IMU (direct I2C, no Blinka). Run it as a module to see the tilt in plain words. |
| `mini_bdx_runtime/mini_bdx_runtime/rustypot_position_hwi.py` | **Changed.** Recovers from servo bus errors (flushes and reopens the port instead of crashing) and prints the raw servo replies on the first errors. |
| `mini_bdx_runtime/mini_bdx_runtime/duck_config.py` | **Changed.** New config keys: `imu_type`, `imu_accel_axes`, `imu_gyro_axes`, `fake_feet_contacts`. |
| `mini_bdx_runtime/mini_bdx_runtime/feet_contacts.py` | **Changed.** Adds `FakeFeetContacts` (both feet always down). |
| `scripts/v2_rl_walk_mujoco.py` | **Changed.** `--keyboard` option; picks the IMU driver from the config; uses fake contacts if configured; imports the Xbox controller and antennas only when used. |
| `scripts/check_imu_orientation.py` | **New.** Guided poses to find the IMU axis maps and write them to `duck_config.json`. |
| `scripts/adjust_offset.py` | **New.** Re-measures one joint's offset, keeping the others as a reference. **Moves the servos.** |
| `scripts/watch_joints.py` | **New.** Read-only live joint angles; `--find` names the servo you move by hand. |
| `CLAUDE.md` | Notes on this duck's hardware and current status. |

### Extra keys in `~/duck_config.json`

```json
"imu_type": "dfrobot_10dof",
"imu_accel_axes": ["y", "x", "-z"],
"imu_gyro_axes": ["y", "x", "-z"],
"fake_feet_contacts": true
```

Without these keys the defaults match the reference build (BNO055, real
foot switches).

### Running

All commands from `scripts/`:

```bash
# check the IMU (read-only)
../.venv/bin/python -m mini_bdx_runtime.dfrobot_10dof_imu
# find the IMU axis maps (read-only, guided)
../.venv/bin/python check_imu_orientation.py
# check which servo is which (read-only)
../.venv/bin/python watch_joints.py --find
# re-measure one joint's offset (moves the servos)
../.venv/bin/python adjust_offset.py --joint right_hip_pitch
# walk with keyboard control (starts paused, p to unpause, SPACE to stop)
../.venv/bin/python v2_rl_walk_mujoco.py --onnx_model_path ~/policies/BEST_WALK_ONNX_2.onnx --keyboard
# relax all servos (Ctrl+C in the walk script leaves them stiff)
../.venv/bin/python turn_off.py
```

Walk script tuning: `-p` leg stiffness (default 30), `--cutoff_frequency`
smoothing filter, `--save_obs 1` records the run to `robot_saved_obs.pkl`.

Keyboard: `w`/`s` forward/back, `a`/`d` sideways, `q`/`e` turn,
`SPACE` stop, `p` pause, `r` sprint, `+`/`-` step frequency, `h` head mode,
`?` help.

### Lessons learned

- **Servo power needs ~10A.** On a 2A supply half the servos browned out
  under load, which showed up as endless `Parsing error`s.
- **Check servo IDs against joints** with `watch_joints.py --find`. Two
  swapped servos made a hip slam into the frame.
- **A firm, unresponsive crouch while held is normal** for
  BEST_WALK_ONNX_2 at zero speed. It balances by stepping, so test it
  free-standing.

---

## Simplified install via pre-built Duck image

As an alternative to setting up everything manually, the image release (current version: 0.2.3), available in the releases section of this repository, provides a ready-to-flash system image for the Open Duck Mini platform. It is tested to work on Open Duck hardware built to the standard specs.

The image includes all essential runtime dependencies, autostart scripts, and environment configurations pre-installed. It is optimized for first-boot setup, ensuring plug-and-play operation of walking, puppet, and control modules after you add your configuration data. This image is currently only tested on the Raspberry Pi Zero 2W and based on Raspberry Pi OS Lite (64-Bit). If you run into issues with the pre-built image or have any feedback, please message MaxMakesStuff on the Discord community.

Please follow the instructions in "INSTALL.md" in the docs folder closely to use this image file. If you wish to compile and publish your own release, you can learn how to do so by following "RELEASE_GUIDE.md".

## Raspberry Pi zero 2W setup

### Install Raspberry Pi OS

Download Raspberry Pi OS Lite (64-bit) from here : https://www.raspberrypi.com/software/operating-systems/

Follow the instructions here to install the OS on the SD card : https://www.raspberrypi.com/documentation/computers/getting-started.html

With the Raspberry Pi Imager, you can pre-configure session, wifi and ssh. Do it like below :

![imager_setup](https://github.com/user-attachments/assets/7a4987b2-de83-41dd-ab7f-585259685f16)

> Tip: I configure the rasp to connect to my phone's hotspot, this way I can connect to it from anywhere.

### Setup SSH (If not setup during the installation)

When first booting on the rasp, you will need to connect a screen and a keyboard. The first thing you should do is connect to a wifi network and enable SSH.

To do so, you can follow this guide : https://www.raspberrypi.com/documentation/computers/configuration.html#setting-up-wifi

Then, you can connect to your rasp using SSH without having to plug a screen and a keyboard.

### Update the system and install necessary stuff

```bash
sudo apt update
sudo apt upgrade
sudo apt install git
sudo apt install python3-pip
sudo apt install python3-virtualenvwrapper
(optional) sudo apt install python3-picamzero

```

Add this to the end of the `.bashrc`:

```bash
export WORKON_HOME=$HOME/.virtualenvs
export PROJECT_HOME=$HOME/Devel
source /usr/share/virtualenvwrapper/virtualenvwrapper.sh
```

### Enable I2C

`sudo raspi-config` -> `Interface Options` -> `I2C`

TODO set 400KHz ?

### Set the usbserial latency timer

```bash
cd  /etc/udev/rules.d/
sudo touch 99-usb-serial.rules
sudo nano 99-usb-serial.rules
# copy the following line in the file
SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
```

### Set the udev rules for the motor control board

TODO


### Setup xbox one controller over bluetooth

Turn your xbox one controller on and set it in pairing mode by long pressing the sync button on the top of the controller.

Run the following commands on the rasp :

```bash
bluetoothctl
scan on
```

Wait for the controller to appear in the list, then run :

```bash
pair <controller_mac_address>
trust <controller_mac_address>
connect <controller_mac_address>
```

The led on the controller should stop blinking and stay on.

You can test that it's working by running

```bash
python3 mini_bdx_runtime/mini_bdx_runtime/xbox_controller.py
```

## Speaker wiring and configuration
Follow this tutorial

> For now, don't activate `/dev/zero` when they ask

https://learn.adafruit.com/adafruit-max98357-i2s-class-d-mono-amp?view=all


## Install the runtime

### Make a virtual environment and activate it

```bash
mkvirtualenv -p python3 open-duck-mini-runtime
workon open-duck-mini-runtime
```

Clone this repository on your rasp, cd into the repo, then :

```bash
git clone https://github.com/apirrone/Open_Duck_Mini_Runtime
cd Open_Duck_Mini_Runtime
git checkout v2
pip install -e .
```

In Raspberry Pi 5, you need to perform the following operations

```bash
pip uninstall -y RPi.GPIO
pip install lgpio
```


## Test the IMU

```bash
python3 mini_bdx_runtime/mini_bdx_runtime/raw_imu.py
```

You can also run `python3 scripts/imu_server.py` on the robot and `python3 scripts/imu_client.py --ip <robot_ip>` on your computer to check that the frame is oriented correctly. 

> To find the ip address of the robot, run `ifconfig` on the robot

## Test motors

This will allow you to verify all your motors are connected and configured.

```bash
python3 scripts/check_motors.py
```

## Make your duck_config.json

Copy `example_config.json` in the home directory of your duck and rename it `duck_config.json`.

`cp example_config.json ~/duck_config.json`

In this file, you can configure some stuff, like registering if you installed the expression features, installed the imu upside down or and other stuff. You also write the joints offsets of your duck here

## Find the joints offsets

This script will guide you through finding the joints offsets of your robot that you can then write in your `duck_config.json`

> This procedure won't be necessary in the future as we will be flashing the offsets directly in each motor's eeprom.

```bash
cd scripts/
python find_soft_offsets.py
```

## Run the walk !

Download the [latest policy checkpoint ](https://github.com/apirrone/Open_Duck_Mini/blob/v2/BEST_WALK_ONNX_2.onnx) and copy it to your duck.

`cd scripts/`

`python v2_rl_walk_mujoco.py --onnx_model_path <path_to>/BEST_WALK_ONNX_2.onnx`



```
- The commands are : 
- A to pause/unpause
- X to turn on/off the projector
- B to play a random sound
- Y to turn on/off head control (very experimental, I don't recommend trying that, it can break your duck's head)
- left and right triggers to control the left and right antennas
- LB (new!) press and hold to increase the walking frequency, kind of a sprint mode 🙂
```