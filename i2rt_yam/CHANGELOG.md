# Changelog – YAM Description

All notable changes to this model will be documented in this file.

## [2026-10-02]

- `yam_linear_4310.xml`: refitted the finger collision primitives to the
  `tip_left.stl` / `tip_right.stl` meshes (flat grip pads on one plane, strut
  boxes following the triangular frame, no fingertip spheres), extended the
  finger joint and control ranges 5 mm past the closed stop so a closed
  command squeezes a grasped object, raised the gripper actuator gains and
  force range to match the real gripper, and removed the finger-finger contact
  exclusion so the pads can touch.

## [2026-09-30]

- Added [`yam_linear_4310.xml`](yam_linear_4310.xml) and
  [`scene_linear_4310.xml`](scene_linear_4310.xml) for the YAM / YAM Pro with
  I2RT's `linear_4310` linear gripper and wrist-mounted Intel RealSense D405
  camera.

## [2025-05-19]

- Initial release.
