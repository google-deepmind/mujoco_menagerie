# Changelog

## Unreleased

### Added

- Initial MJCF model of the Sharpa Wave hand (left and right) derived from the
  upstream URDF at
  [sharpa-robotics/sharpa-urdf-usd-xml@6eea427eb24189519f32b9f21674cd534d3f973c](https://github.com/sharpa-robotics/sharpa-urdf-usd-xml/commit/6eea427eb24189519f32b9f21674cd534d3f973c).
- 22 position actuators with joint-class–specific gains.
- VHACD palm collision decomposition (32 pieces) and capsule fits for finger
  phalanges.
- `make_right.py` mirror-generation script and the `meshes_right/` mirrored
  asset directory.

### Fixed

- Right hand joint axes now match the real hardware. `make_right.py` mirrored
  the left hand's motion, which actuated every joint backwards; it now relabels
  the joint coordinate instead (negate the axis, keep the range).
