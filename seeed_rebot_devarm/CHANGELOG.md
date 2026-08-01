# Changelog – Seeed Studio reBot DevArm Description

All notable changes to this model will be documented in this file.

## [2026-07-20]
- Preserve all six URDF inertia tensor components for every body and restore the
  full-precision `base_link` center of mass.
- Add `ctrlrange` (equal to the joint limits) to all eight position actuators.
- Remove the unused `balanceinertia` compiler flag so future inertia edits that
  violate the triangle inequality fail loudly (compiled inertias unchanged).
- Move scene-level configuration out of the robot file (dead `floor` default
  class, offscreen render sizes) and drop the `statistic` override from
  `scene.xml` (#298).
- Document the disabled self-collision and the upstream licensing status.

## [2026-07-18]
- Initial release.
