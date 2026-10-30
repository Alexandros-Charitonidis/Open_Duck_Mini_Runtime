# This duck

Hardware differs from the reference build:
- Raspberry Pi CM5 (8GB/32GB eMMC) on official CM5 IO Board, not Pi Zero 2W
- Pi OS Trixie, Python 3.11 venv managed by uv (system python is 3.13)
- Servos are Waveshare ST3215-HS (6-12.6V, 20kg.cm@12V, 0.094s/60°),
  NOT the 7.4V reference variant. Running at 12V. Roughly 2x the
  reference speed, so BEST_WALK_ONNX_2 is expected to transfer poorly.
- Servo power: Mean Well 12V/10A. A 2A bench supply browned out half
  the servos under policy load (they went silent on the bus, showing
  as rustypot "Parsing error"). Needs ~10A+ headroom.
- Serial adapter is CH343 on /dev/ttyACM0 (cdc_acm), not FTDI.
  README's ftdi_sio latency-timer rule does not apply.
- Stand-in IMU: DFRobot 10DOF v2.0 (ADXL345 0x53, ITG-3205 0x68,
  mag 0x0c, BMP280 0x77) on /dev/i2c-1, driver dfrobot_10dof_imu.py,
  selected by "imu_type" in duck_config.json. Mounted upside down,
  axis map ["y","x","-z"] (from scripts/check_imu_orientation.py),
  verified by hand. BNO085 and ICM-42688-P on order.
- gpiod is installed in the venv (blinka's `board` needs it on CM5).
- No foot switches fitted (GPIO 22/27). "fake_feet_contacts": true in
  duck_config.json reports both feet always down. Remove once fitted.
- Walk policy is at ~/policies/BEST_WALK_ONNX_2.onnx.
- 2026-10-30: servo IDs 11 and 12 were on swapped joints (right hip
  roll/pitch); re-IDed in EEPROM so they now match the standard IDs.
  Offsets re-measured with scripts/adjust_offset.py
  (right_hip_roll 0.186, right_hip_pitch 0.040).
- No gamepad: control is the terminal keyboard (--keyboard,
  keyboard_controller.py). The generic 2.4GHz pad is unused.

Offsets are in ~/duck_config.json, backed up to ~/duck_config.json.bak
(plus timestamped .bak.* copies made by the scripts).
Do not run anything that commands servo motion without asking first.

# Status (2026-10-30)

- Walk script runs end to end: keyboard, DFRobot IMU, fake feet
  contacts, bus-error recovery. No bus errors since the 12V/10A supply.
- Held/standing test: duck holds a firm crouch and barely reacts to a
  push while held. Offline check showed this is the policy's own
  standing pose (same outputs with ideal inputs); all inputs (IMU frame,
  joint order, home pose) match Open_Duck_Playground training.
  Training reference files were fetched to ~/odp_ref (not in repo).
- Next: free-standing test on the floor (hands hovering / safety line),
  then gentle walking (w). Tune stiffness with -p (30 -> 20 -> 15) and
  --cutoff_frequency, recording each run with --save_obs 1 for analysis.
- Hardware access is only on the duck (Pi). A session on another
  machine can edit code but cannot run anything against the servos/IMU.
