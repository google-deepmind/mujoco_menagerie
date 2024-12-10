# Unitree G1 Description (MJCF)

> [!IMPORTANT]
> Requires MuJoCo 2.3.4 or later.

## Changelog

- 12/10/2024: Use updated [g1_29dof_rev_1_0 model](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/g1_description) (sha: c20ca8f1fe5e519474c6c8d10b1ce5c719dd7a65).
- 05/20/2024: Initial release.

## Overview

This package contains a simplified robot description (MJCF) of the [G1 Humanoid
Robot](https://www.unitree.com/g1/) developed by [Unitree
Robotics](https://www.unitree.com/). It is derived from the [publicly available
MJCF
description](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/g1_description/g1_29dof_rev_1_0.xml). Specifically, this model has the fully actuated waist but not the full hands.

<p float="left">
  <img src="g1.png" width="400">
</p>

## MJCF derivation steps

1. Copied the MJCF description from [g1_description](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/g1_description/g1_29dof_rev_1_0.xml).
2. Manually edited the MJCF to extract common properties into the `<default>` section.
3. Added stand keyframe.
4. Added joint position actuators (needs tuning).

## License

This model is released under a [BSD-3-Clause License](LICENSE).
