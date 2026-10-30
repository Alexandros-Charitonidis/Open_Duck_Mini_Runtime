# This duck

Hardware differs from the reference build:
- Raspberry Pi CM5 (8GB/32GB eMMC) on official CM5 IO Board, not Pi Zero 2W
- Pi OS Trixie, Python 3.11 venv managed by uv (system python is 3.13)
- Servos are Waveshare ST3215-HS (6-12.6V, 20kg.cm@12V, 0.094s/60°),
  NOT the 7.4V reference variant. Running at 11V. Roughly 2x the
  reference speed, so BEST_WALK_ONNX_2 is expected to transfer poorly.
- Serial adapter is CH343 on /dev/ttyACM0 (cdc_acm), not FTDI.
  README's ftdi_sio latency-timer rule does not apply.
- No IMU fitted yet. BNO085 and ICM-42688-P on order.
- Controller is a generic 2.4GHz dongle pad, not an Xbox controller.
  Button mapping may need remapping in xbox_controller.py.

Offsets are in ~/duck_config.json, backed up to ~/duck_config.json.bak.
Do not run anything that commands servo motion without asking first.