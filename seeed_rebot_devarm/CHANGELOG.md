# Changelog – Seeed Studio reBot DevArm Description

All notable changes to this model will be documented in this file.

## [2026-08-09]
- Correct the stated reason for the `gripper_left`/`gripper_right` exclude. The
  0.15 mm near-contact is at the interleaved slider rail at the base of the
  fingers, not at the jaws: the two jaws cross scissor-fashion and never meet,
  so no jaw pad primitive applies here.
- Replace the single convex hull per link with a VisACD convex decomposition
  (8–12 parts per link), reducing collision volume from 3.24× to 1.89× the true
  link volume, and cap hull complexity with `<mesh maxhullvert="64"/>`.
- Enable self-collision. Eleven body pairs are excluded: nine adjacent links
  whose source meshes genuinely interpenetrate, plus `gripper_left`/
  `gripper_right` and `link2`/`link5`, which clear each other by 0.15 mm and
  0.77 mm — below what a convex decomposition can represent. Both keyframes are
  contact-free.
- Rename the top-level default class `robot` → `seeed`; move `compiler`,
  `option` and `asset` above `default` to match the repository convention.
- Re-render the hero image so the arm is no longer clipped.
- Correct the derivation notes: the earlier claim that MuJoCo's URDF importer
  mis-rotates a merged fixed-joint inertial was wrong. MuJoCo's merge matches a
  hand-computed composite-rigid-body result to 7.4e-12 kg·m², and the model's
  g(q) agrees with the URDF compiled directly by MuJoCo to 6.5e-06 N·m.
  `gripper_end` is kept as a separate body for URDF↔MJCF traceability, not to
  work around a bug.

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
