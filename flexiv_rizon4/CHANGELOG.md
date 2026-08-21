# Changelog – Flexiv Robotics Rizon4 Description

All notable changes to this model will be documented in this file.

## [2026-08-21]
- Fixed `compute_gains.py` to run with MuJoCo 3.10+ (the `mj_fullM` signature
  changed) and to index the mass-matrix diagonal by DOF address instead of
  joint id, which previously read the wrong entry whenever a free or
  multi-DOF joint precedes a joint in the model.

## [2026-02-06]
- Initial release.
