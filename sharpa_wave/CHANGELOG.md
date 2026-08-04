# Changelog – Sharpa Wave Description

All notable changes to this model will be documented in this file.

## [2026-08-03]

- Set joint `armature` from the manufacturer's design parameters.
- Fixed collision geoms contributing mass. The `collision` class set no `density`,
  inflating the two bodies that carry no explicit `<inertial>` by 96 g in total.

## [2026-06-15]

- Fixed the right hand joint axes to match the real hardware. `make_right.py`
  mirrored the left hand's motion, which actuated every joint backwards; it now
  relabels the joint coordinate instead (negate the axis, keep the range).
- Named the fingertip collision geoms of both hands, e.g.
  `right_thumb_fingertip_collision`.

## [2026-05-18]

- Initial release. MJCF model of the Sharpa Wave hand (left and right) derived from
  the upstream URDF at
  [sharpa-robotics/sharpa-urdf-usd-xml@6eea427](https://github.com/sharpa-robotics/sharpa-urdf-usd-xml/commit/6eea427eb24189519f32b9f21674cd534d3f973c),
  with 22 position actuators, a VHACD palm collision decomposition (32 pieces),
  capsule fits for the finger phalanges, and the `make_right.py` mirror script.
