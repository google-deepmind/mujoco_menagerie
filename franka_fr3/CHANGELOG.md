# Changelog – Franka Robotics FR3 Description

All notable changes to this model will be documented in this file.

## [2026-10-05]
- Update joint position limits to match the current Franka specification. The previous
  values predate Franka system image 5.9.0, which widened the limits (see libfranka 0.18.0
  and `franka_description` >= 1.1.0).

## [2025-07-02]
- Explicitly exclude contacts between the base and the first link, in `fr3.xml`, to match `fr3_mjx.xml`.

## [2025-04-25]
- Update armature, damping, friction parameters, identified in torque control.

## [2024-05-31]
- Initial release.
