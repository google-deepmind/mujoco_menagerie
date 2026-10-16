# Changelog – Hello Robot Stretch 4 Description

All notable changes to this model will be documented in this file.

## [2026-10-16]
- Initial release, with the SG4 (standard) and PG4 (parallel jaw) grippers,
  a textured head, and LED ring geometry around the head cameras.
- Calibrated resolution and intrinsics on every camera, and a site at each line
  sensor's emitter.
- `stretch4_sensors.py`, ported from `stretch4_mujoco`, which renders the
  fisheye head cameras and ray casts the head lidars and line sensors.
- `keyboard_teleop.py`, a keyboard teleop that streams the sensors to Rerun.
